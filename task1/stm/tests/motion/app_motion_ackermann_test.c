/* Host-buildable regression test for Ackermann-v2 rear-wheel
 * target-rate control: AppMotion_SetAckermann()/GetAckermann(), the
 * inner/outer rate mapping in AppMotion_SelectAckermannRates(), and
 * its use of the RUNTIME TURNSPEED base rate (AppMotion_GetTurnSpeed())
 * rather than any hardcoded value. Exercised directly against the
 * real Core/Src/app_motion.c (+ app_pid.c), the same way
 * app_motion_profile_test.c/app_motion_telemetry_test.c/
 * app_motion_presteer_test.c/app_motion_turnservo_test.c already do.
 *
 * PRESTEER is explicitly disabled for the rate-integration tests below
 * (AppMotion_SetPreSteer(0.0f, false)) so commands enter
 * APP_MOTION_TURN directly -- PRE_STEER/Ackermann interaction has its
 * own dedicated coverage further down.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -Itests/motion/fakes \
 *       -o tests/motion/app_motion_ackermann_test.exe \
 *       tests/motion/app_motion_ackermann_test.c Core/Src/app_motion.c \
 *       Core/Src/app_pid.c tests/motion/fakes/hw_stubs.c
 *   ./tests/motion/app_motion_ackermann_test.exe
 */

#include "app_motion.h"
#include "app_servo.h"
#include "test_fakes.h"

#include <stdio.h>

static unsigned checks;
static unsigned failures;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        failures++; \
        printf("FAIL line %d: %s\n", __LINE__, #expr); \
    } \
} while (0)

#define CHECK_LONG_EQ(actual, expected) \
    CHECK((long)(actual) == (long)(expected))

#define CHECK_NEAR(actual, expected, tolerance) \
    CHECK((((actual) > ((expected) - (tolerance)))) && ((actual) < ((expected) + (tolerance))))

static void ResetToIdle(void)
{
    MotionCommand stopCommand = { MOTION_STOP, 0 };
    AppMotion_HandleCommand(&stopCommand);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
}

static void RunAckermannDirectionCase(
    MotionType direction,
    uint32_t turnSpeed,
    int32_t expectedLeftRate,
    int32_t expectedRightRate
)
{
    AppMotionTelemetry telemetry;

    ResetToIdle();
    AppMotion_SetPreSteer(0.0f, false);
    AppMotion_SetTurnSpeed(turnSpeed);
    AppMotion_SetAckermann(0.30f, true);

    Fake_SetTickCount(1000U);
    Fake_SetEncoderLeft(500);
    Fake_SetEncoderRight(500);
    Fake_SetHeading(0.0f);

    MotionCommand command = { direction, 90 };
    AppMotion_HandleCommand(&command);
    CHECK(AppMotion_GetState() == APP_MOTION_TURN);

    Fake_SetTickCount(1010U);
    AppMotion_Update(0.01f);

    AppMotion_GetTelemetry(&telemetry);
    CHECK_LONG_EQ(telemetry.ackermannLeftRateCountsPerSec, expectedLeftRate);
    CHECK_LONG_EQ(telemetry.ackermannRightRateCountsPerSec, expectedRightRate);

    ResetToIdle();
    AppMotion_SetTurnSpeed(2000U);   /* restore default for subsequent tests */
}

int main(void)
{
    AppMotion_Init();

    /* --- 1. SET/GET round trip. -------------------------------------- */
    {
        float k;
        bool enable;

        AppMotion_SetAckermann(0.25f, true);
        AppMotion_GetAckermann(&k, &enable);
        CHECK_NEAR(k, 0.25f, 0.001f);
        CHECK(enable == true);

        AppMotion_SetAckermann(0.0f, false);
        AppMotion_GetAckermann(&k, &enable);
        CHECK_NEAR(k, 0.0f, 0.001f);
        CHECK(enable == false);
    }

    /* --- 2. Defensive motion-layer bound: AppMotion_SetAckermann()
     * itself clamps k to [0.0,0.5] -- enable is stored exactly as
     * given, unaffected by the clamp. --------------------------------- */
    {
        float k;
        bool enable;

        AppMotion_SetAckermann(-0.5f, true);
        AppMotion_GetAckermann(&k, &enable);
        CHECK_NEAR(k, 0.0f, 0.001f);
        CHECK(enable == true);

        AppMotion_SetAckermann(0.9f, false);
        AppMotion_GetAckermann(&k, &enable);
        CHECK_NEAR(k, 0.5f, 0.001f);
        CHECK(enable == false);

        AppMotion_SetAckermann(0.3f, true);
        AppMotion_GetAckermann(&k, &enable);
        CHECK_NEAR(k, 0.3f, 0.001f);
    }

    /* --- 3. Disabled: both wheels ramp at exactly the current runtime
     * TURNSPEED (1200), regardless of whatever k is stored as -- proves
     * the disabled path bypasses the (1+-k) formula entirely. --------- */
    {
        AppMotionTelemetry telemetry;

        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnSpeed(1200U);
        AppMotion_SetAckermann(0.30f, false);   /* nonzero k, but disabled */

        Fake_SetTickCount(1500U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetTickCount(1510U);
        AppMotion_Update(0.01f);

        AppMotion_GetTelemetry(&telemetry);
        CHECK_LONG_EQ(telemetry.ackermannLeftRateCountsPerSec, 1200);
        CHECK_LONG_EQ(telemetry.ackermannRightRateCountsPerSec, 1200);

        ResetToIdle();
        AppMotion_SetTurnSpeed(2000U);
    }

    /* --- 4. k=0.30, TURNSPEED=1200: all four FL/FR/BL/BR mappings,
     * matching the physically-confirmed table exactly (840/1560). ----- */
    RunAckermannDirectionCase(MOTION_FORWARD_LEFT,   1200U, 840, 1560);
    RunAckermannDirectionCase(MOTION_FORWARD_RIGHT,  1200U, 1560, 840);
    RunAckermannDirectionCase(MOTION_BACKWARD_LEFT,  1200U, 1560, 840);
    RunAckermannDirectionCase(MOTION_BACKWARD_RIGHT, 1200U, 840, 1560);

    /* --- 5. Same k=0.30 at different TURNSPEED values -- proves the
     * base genuinely comes from the runtime value, not a residual
     * hardcoded 2000. TURNSPEED=1000 -> 700/1300; TURNSPEED=1600 ->
     * 1120/2080. ------------------------------------------------------ */
    RunAckermannDirectionCase(MOTION_FORWARD_LEFT, 1000U, 700, 1300);
    RunAckermannDirectionCase(MOTION_FORWARD_LEFT, 1600U, 1120, 2080);

    /* --- 6. Average-preserving across several (TURNSPEED,k) pairs
     * (float tolerance, not exact equality). ------------------------ */
    {
        AppMotionTelemetry telemetry;
        struct { uint32_t turnSpeed; float k; } cases[] = {
            { 800U, 0.10f }, { 1200U, 0.20f }, { 1200U, 0.30f },
            { 1600U, 0.35f }, { 2000U, 0.50f }
        };
        unsigned i;

        for (i = 0U; i < (sizeof(cases) / sizeof(cases[0])); i++)
        {
            ResetToIdle();
            AppMotion_SetPreSteer(0.0f, false);
            AppMotion_SetTurnSpeed(cases[i].turnSpeed);
            AppMotion_SetAckermann(cases[i].k, true);

            Fake_SetTickCount(1700U);
            Fake_SetEncoderLeft(0);
            Fake_SetEncoderRight(0);
            Fake_SetHeading(0.0f);

            MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
            AppMotion_HandleCommand(&command);

            Fake_SetTickCount(1710U);
            AppMotion_Update(0.01f);

            AppMotion_GetTelemetry(&telemetry);
            float average =
                ((float)telemetry.ackermannLeftRateCountsPerSec +
                 (float)telemetry.ackermannRightRateCountsPerSec) * 0.5f;
            CHECK_NEAR(average, (float)cases[i].turnSpeed, 1.0f);

            ResetToIdle();
        }

        AppMotion_SetTurnSpeed(2000U);
    }

    /* --- 7. Reverse: target increments stay negative (decrease tick
     * over tick), with the correct inner/outer magnitude still applied
     * to the correct wheel -- BL: left=outer(1560), right=inner(840),
     * both decreasing, at TURNSPEED=1200,k=0.30. ------------------------ */
    {
        AppMotionTelemetry telemetry;

        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnSpeed(1200U);
        AppMotion_SetAckermann(0.30f, true);

        Fake_SetTickCount(2000U);
        Fake_SetEncoderLeft(1000);
        Fake_SetEncoderRight(1000);
        Fake_SetHeading(0.0f);

        MotionCommand command = { MOTION_BACKWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);

        Fake_SetTickCount(2010U);
        AppMotion_Update(0.01f);

        AppMotion_GetTelemetry(&telemetry);
        CHECK(telemetry.leftTargetCounts < 1000);   /* decreased (reverse) */
        CHECK(telemetry.rightTargetCounts < 1000);  /* decreased (reverse) */
        /* left=outer(1560)*0.01=15.6, right=inner(840)*0.01=8.4 */
        CHECK_NEAR((float)telemetry.leftTargetCounts, 1000.0f - 15.6f, 1.5f);
        CHECK_NEAR((float)telemetry.rightTargetCounts, 1000.0f - 8.4f, 1.5f);

        ResetToIdle();
        AppMotion_SetTurnSpeed(2000U);
    }

    /* --- 8. PRE_STEER interaction: Ackermann must not advance either
     * target during PRE (rate telemetry reads 0, targets frozen), and
     * must begin applying the correct inner/outer rates from the FRESH
     * re-anchored base at the PRE->TURN transition, with no
     * discontinuity. --------------------------------------------------- */
    {
        AppMotionTelemetry telemetry;
        unsigned tick;

        ResetToIdle();
        AppMotion_SetPreSteer(200.0f, true);
        AppMotion_SetTurnSpeed(1200U);
        AppMotion_SetAckermann(0.30f, true);

        Fake_SetTickCount(3000U);
        Fake_SetEncoderLeft(500);
        Fake_SetEncoderRight(500);
        Fake_SetHeading(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);
        CHECK_LONG_EQ(AppServo_GetUs(), 1000);   /* fixed servo, unaffected by Ackermann */

        for (tick = 0U; tick < 5U; tick++)
        {
            Fake_SetTickCount(3000U + (tick * 10U));
            AppMotion_Update(0.01f);

            AppMotion_GetTelemetry(&telemetry);
            CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);
            CHECK_LONG_EQ(telemetry.ackermannLeftRateCountsPerSec, 0);
            CHECK_LONG_EQ(telemetry.ackermannRightRateCountsPerSec, 0);
            CHECK_LONG_EQ(telemetry.leftTargetCounts, 500);   /* frozen */
            CHECK_LONG_EQ(telemetry.rightTargetCounts, 500);
            CHECK_LONG_EQ(AppServo_GetUs(), 1000);
        }

        /* Transition, simulating a tiny physical drift during the wait. */
        Fake_SetTickCount(3000U + 250U);
        Fake_SetEncoderLeft(505);
        Fake_SetEncoderRight(507);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetTickCount(3000U + 260U);
        AppMotion_Update(0.01f);   /* first real TURN tick */

        AppMotion_GetTelemetry(&telemetry);
        /* FL: left=inner(840), right=outer(1560), forward (+), from the
         * RE-ANCHORED base (505/507), not the stale PRE-entry base
         * (500/500) -- no discontinuity. */
        CHECK_LONG_EQ(telemetry.ackermannLeftRateCountsPerSec, 840);
        CHECK_LONG_EQ(telemetry.ackermannRightRateCountsPerSec, 1560);
        CHECK_NEAR((float)telemetry.leftTargetCounts, 505.0f + 8.4f, 1.5f);
        CHECK_NEAR((float)telemetry.rightTargetCounts, 507.0f + 15.6f, 1.5f);
        CHECK_LONG_EQ(AppServo_GetUs(), 1000);   /* still fixed */
        CHECK_LONG_EQ(telemetry.servoPulseUs, 1000);
        CHECK_LONG_EQ(telemetry.steeringCorrectionUs, 0);   /* steeringPID still dormant */

        ResetToIdle();
        AppMotion_SetTurnSpeed(2000U);
    }

    /* --- 9. Fixed servo remains 1000/2060 throughout an Ackermann-
     * enabled turn, and steeringPID stays dormant -- direct regression
     * check that Ackermann never touches the servo path, across
     * several ticks with heading stepped toward the target. ----------- */
    {
        AppMotionTelemetry telemetry;
        float headingSteps[] = { 0.0f, 30.0f, 60.0f, 85.0f };
        unsigned i;

        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnSpeed(1200U);
        AppMotion_SetAckermann(0.30f, true);

        Fake_SetTickCount(4000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);

        MotionCommand command = { MOTION_FORWARD_RIGHT, 90 };  /* servo endpoint 2060 */
        AppMotion_HandleCommand(&command);

        for (i = 0U; i < (sizeof(headingSteps) / sizeof(headingSteps[0])); i++)
        {
            Fake_SetTickCount(4000U + ((i + 1U) * 10U));
            Fake_SetHeading(headingSteps[i]);
            AppMotion_Update(0.01f);

            CHECK_LONG_EQ(AppServo_GetUs(), 2060);

            AppMotion_GetTelemetry(&telemetry);
            CHECK_LONG_EQ(telemetry.servoPulseUs, 2060);
            CHECK_LONG_EQ(telemetry.steeringCorrectionUs, 0);
            /* FR: left=outer(1560), right=inner(840) */
            CHECK_LONG_EQ(telemetry.ackermannLeftRateCountsPerSec, 1560);
            CHECK_LONG_EQ(telemetry.ackermannRightRateCountsPerSec, 840);
        }

        ResetToIdle();
        AppMotion_SetTurnSpeed(2000U);
    }

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
