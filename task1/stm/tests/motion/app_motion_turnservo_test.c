/* Host-buildable regression test for the fixed max-safe turn servo
 * stage: AppMotion_TurnServoEndpointUs() (FL/BR->1000us, FR/BL->2060us,
 * held for the ENTIRE PRE_STEER+TURN duration, no steeringPID-driven
 * unwind), and the runtime TURNSPEED base turn rate
 * (AppMotion_SetTurnSpeed()/GetTurnSpeed()). Exercised directly
 * against the real Core/Src/app_motion.c (+ app_pid.c), the same way
 * app_motion_profile_test.c/app_motion_telemetry_test.c/
 * app_motion_presteer_test.c already do.
 *
 * Note: Ackermann rear-wheel target-rate control does not exist on
 * this branch (confirmed by inspection at planning time) -- these
 * tests exercise TURNSPEED's direct integration into
 * case APP_MOTION_TURN:'s own ramp computation, not any Ackermann
 * rate-selection helper.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -Itests/motion/fakes \
 *       -o tests/motion/app_motion_turnservo_test.exe \
 *       tests/motion/app_motion_turnservo_test.c Core/Src/app_motion.c \
 *       Core/Src/app_pid.c tests/motion/fakes/hw_stubs.c
 *   ./tests/motion/app_motion_turnservo_test.exe
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

/* --- 1. Fixed servo mapping: correct endpoint immediately at PRE_STEER
 * entry, for every direction and EVERY requested angle (the requested
 * angle must have zero effect on the servo value). ------------------ */
static void RunFixedEndpointCase(
    MotionType direction,
    int32_t requestedAngle,
    int32_t expectedServoUs
)
{
    ResetToIdle();
    AppMotion_SetPreSteer(150.0f, true);

    Fake_SetTickCount(1000U);
    Fake_SetEncoderLeft(0);
    Fake_SetEncoderRight(0);
    Fake_SetHeading(0.0f);

    MotionCommand command = { direction, requestedAngle };
    AppMotion_HandleCommand(&command);

    CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);
    CHECK_LONG_EQ(AppServo_GetUs(), expectedServoUs);

    ResetToIdle();
}

int main(void)
{
    AppMotion_Init();

    /* AppMotion_Init()'s boot defaults now include ACKERMANN=0.270,true
     * (physically validated turn configuration) -- explicitly disabled
     * here so this file's flat-rate TURNSPEED-integration math (section
     * 6 below: both wheels ramp at exactly rate*dtSeconds) is isolated
     * from Ackermann's inner/outer redistribution, which has its own
     * dedicated coverage in app_motion_ackermann_test.c. */
    AppMotion_SetAckermann(0.0f, false);

    /* --- 1. All four directions x five requested angles -- servo value
     * is identical regardless of angle. ------------------------------ */
    {
        int32_t angles[] = { 15, 30, 45, 90, 180 };
        unsigned i;

        for (i = 0U; i < (sizeof(angles) / sizeof(angles[0])); i++)
        {
            RunFixedEndpointCase(MOTION_FORWARD_LEFT,   angles[i], 1000);
            RunFixedEndpointCase(MOTION_FORWARD_RIGHT,  angles[i], 2060);
            RunFixedEndpointCase(MOTION_BACKWARD_LEFT,  angles[i], 2060);
            RunFixedEndpointCase(MOTION_BACKWARD_RIGHT, angles[i], 1000);
        }
    }

    /* --- 2. PRE_STEER: servo already at endpoint, motors stopped (not
     * AppMotor_Brake()), targets do not advance -- across several
     * ticks. -------------------------------------------------------- */
    {
        AppMotionTelemetry telemetry;
        unsigned tick;
        uint32_t setCallCountAtEntry;

        ResetToIdle();
        AppMotion_SetPreSteer(150.0f, true);

        Fake_SetTickCount(2000U);
        Fake_SetEncoderLeft(500);
        Fake_SetEncoderRight(500);
        Fake_SetHeading(0.0f);
        Fake_ResetMotorCallCounts();

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);

        CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);
        CHECK(Fake_GetMotorStopCallCount() >= 1U);
        CHECK(Fake_GetMotorBrakeCallCount() == 0U);
        setCallCountAtEntry = Fake_GetMotorSetCallCount();

        for (tick = 0U; tick < 5U; tick++)
        {
            Fake_SetTickCount(2000U + (tick * 10U));
            AppMotion_Update(0.01f);

            CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);
            CHECK_LONG_EQ(AppServo_GetUs(), 1000);

            AppMotion_GetTelemetry(&telemetry);
            CHECK_LONG_EQ(telemetry.servoPulseUs, 1000);
            CHECK_LONG_EQ(telemetry.leftTargetCounts, 500);   /* frozen */
            CHECK_LONG_EQ(telemetry.rightTargetCounts, 500);
        }
        CHECK(Fake_GetMotorSetCallCount() == setCallCountAtEntry);  /* never commanded during PRE */

        ResetToIdle();
    }

    /* --- 3. TURN: servo remains pinned at the fixed endpoint for the
     * WHOLE turn, even as heading error shrinks toward the target --
     * the key regression test proving the steeringPID-driven "unwind"
     * is gone. steeringCorrectionUs reads 0 every tick (steeringPID is
     * not evaluated for servo generation any more). Once heading
     * settles within tolerance for enough consecutive ticks, the
     * (unmodified) gyro/completion logic still transitions to
     * APP_MOTION_BRAKING exactly as before. ---------------------------- */
    {
        AppMotionTelemetry telemetry;
        float headingSteps[] = { 0.0f, 30.0f, 60.0f, 85.0f, 89.0f };
        unsigned i;

        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, false);   /* straight into TURN, isolate this test from PRE_STEER timing */

        Fake_SetTickCount(3000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };  /* target heading = 0+90 = 90 */
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        for (i = 0U; i < (sizeof(headingSteps) / sizeof(headingSteps[0])); i++)
        {
            Fake_SetTickCount(3000U + ((i + 1U) * 10U));
            Fake_SetHeading(headingSteps[i]);
            AppMotion_Update(0.01f);

            CHECK(AppMotion_GetState() == APP_MOTION_TURN);
            CHECK_LONG_EQ(AppServo_GetUs(), 1000);   /* pinned, regardless of shrinking heading error */

            AppMotion_GetTelemetry(&telemetry);
            CHECK_LONG_EQ(telemetry.servoPulseUs, 1000);   /* MM,SP reports the real commanded value */
            CHECK_LONG_EQ(telemetry.steeringCorrectionUs, 0);  /* steeringPID dormant */
        }

        /* Now settle: heading within TURN_HEADING_TOLERANCE_DEG(2.0) of
         * the 90 deg target, for SETTLED_CYCLES_REQUIRED(5) consecutive
         * ticks -- unmodified completion logic must still fire. */
        Fake_SetHeading(90.0f);
        for (i = 0U; i < 6U; i++)
        {
            Fake_SetTickCount(3100U + (i * 10U));
            AppMotion_Update(0.01f);
        }
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

        ResetToIdle();
    }

    /* --- 4. TURNSPEED SET/GET round trip. ---------------------------- */
    {
        uint32_t rate;

        AppMotion_SetTurnSpeed(1200U);
        AppMotion_GetTurnSpeed(&rate);
        CHECK(rate == 1200U);

        AppMotion_SetTurnSpeed(2000U);
        AppMotion_GetTurnSpeed(&rate);
        CHECK(rate == 2000U);
    }

    /* --- 5. TURNSPEED defensive motion-layer clamp: SetTurnSpeed()
     * itself clamps to [500,2500], independent of the parser's own
     * range check. ------------------------------------------------------ */
    {
        uint32_t rate;

        AppMotion_SetTurnSpeed(100U);      /* below min */
        AppMotion_GetTurnSpeed(&rate);
        CHECK(rate == 500U);

        AppMotion_SetTurnSpeed(9999U);     /* above max */
        AppMotion_GetTurnSpeed(&rate);
        CHECK(rate == 2500U);

        AppMotion_SetTurnSpeed(1500U);     /* in range: unaffected */
        AppMotion_GetTurnSpeed(&rate);
        CHECK(rate == 1500U);

        AppMotion_SetTurnSpeed(2000U);     /* restore default for subsequent tests */
    }

    /* --- 6. TURNSPEED integration: case APP_MOTION_TURN:'s per-tick
     * target-count delta matches rate*dtSeconds (tolerance, not exact
     * float equality). ----------------------------------------------- */
    {
        uint32_t testRates[] = { 1000U, 1500U, 2000U };
        unsigned i;
        AppMotionTelemetry telemetry;

        for (i = 0U; i < (sizeof(testRates) / sizeof(testRates[0])); i++)
        {
            ResetToIdle();
            AppMotion_SetPreSteer(0.0f, false);
            AppMotion_SetTurnSpeed(testRates[i]);

            Fake_SetTickCount(4000U);
            Fake_SetEncoderLeft(0);
            Fake_SetEncoderRight(0);
            Fake_SetHeading(0.0f);

            MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
            AppMotion_HandleCommand(&command);
            CHECK(AppMotion_GetState() == APP_MOTION_TURN);

            Fake_SetTickCount(4010U);
            AppMotion_Update(0.01f);

            AppMotion_GetTelemetry(&telemetry);
            float expectedDelta = (float)testRates[i] * 0.01f;
            CHECK_NEAR((float)telemetry.leftTargetCounts, expectedDelta, 1.5f);
            CHECK_NEAR((float)telemetry.rightTargetCounts, expectedDelta, 1.5f);

            ResetToIdle();
        }

        AppMotion_SetTurnSpeed(2000U);   /* restore default */
    }

    /* --- 7. Zero-degree shortcut preserved: MOV,FL/FR/BL/BR,0 completes
     * immediately (APP_MOTION_EVENT_MOV_DONE), never enters
     * APP_MOTION_PRE_STEER or APP_MOTION_TURN, regardless of PRESTEER
     * being enabled. -------------------------------------------------- */
    {
        MotionType directions[] = {
            MOTION_FORWARD_LEFT, MOTION_FORWARD_RIGHT,
            MOTION_BACKWARD_LEFT, MOTION_BACKWARD_RIGHT
        };
        unsigned i;
        AppMotionEvent event;

        AppMotion_SetPreSteer(150.0f, true);

        for (i = 0U; i < (sizeof(directions) / sizeof(directions[0])); i++)
        {
            ResetToIdle();

            Fake_SetTickCount(5000U);
            Fake_SetEncoderLeft(0);
            Fake_SetEncoderRight(0);
            Fake_SetHeading(0.0f);

            MotionCommand command = { directions[i], 0 };
            AppMotion_HandleCommand(&command);

            CHECK(AppMotion_GetState() == APP_MOTION_IDLE);  /* never PRE_STEER/TURN */

            AppMotion_GetEvent(&event);
            CHECK(event.type == APP_MOTION_EVENT_MOV_DONE);
            CHECK(event.command == directions[i]);
        }

        ResetToIdle();
    }

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
