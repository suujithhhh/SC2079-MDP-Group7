/* Host-buildable regression test for PRE_STEER (Step 2 of turn tuning):
 * APP_MOTION_PRE_STEER, AppMotion_SetPreSteer()/GetPreSteer(), and the
 * four turn command-entry cases' new preSteerEnabled branch. Exercised
 * directly against the real Core/Src/app_motion.c (+ app_pid.c), the
 * same way tests/motion/app_motion_profile_test.c and
 * app_motion_telemetry_test.c already do -- see those files' header
 * comments for why AppMotion_Init() must be called first.
 *
 * Uses the fakes upgraded for this stage (tests/motion/fakes/
 * test_fakes.h): Fake_SetImuReady(), and AppServo_SetUs/GetUs() /
 * AppMotor_Set/Stop/Brake() are now real (state-tracking) fakes, not
 * inert always-0 stubs, so this test can observe exactly what the
 * real code commanded to the servo/motors during PRE_STEER.
 *
 * Expected seed pulses below (1000/2060 depending on direction) are
 * computed from the real, unmodified control law: steeringPID's
 * DEFAULT_STEERING_KP=45.0f/Ki=Kd=0.0f, output clamp +-530,
 * SERVO_CENTER_US=1530 -- e.g. FL90 from heading 0: error=+90,
 * P-term=45*90=4050 clamped to +530, isReverseTurn=false, pulse =
 * 1530-530=1000. These also match the physically observed saturation
 * values from the PRE_STEER planning stage's baseline logs.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -Itests/motion/fakes \
 *       -o tests/motion/app_motion_presteer_test.exe \
 *       tests/motion/app_motion_presteer_test.c Core/Src/app_motion.c \
 *       Core/Src/app_pid.c Core/Src/app_ir_edge.c Core/Src/app_ir_dist.c tests/motion/fakes/hw_stubs.c
 *   ./tests/motion/app_motion_presteer_test.exe
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

#define CHECK_FLOAT_EQ(actual, expected) \
    CHECK(((actual) > ((expected) - 0.001f)) && ((actual) < ((expected) + 0.001f)))

static void ResetToIdle(void)
{
    MotionCommand stopCommand = { MOTION_STOP, 0 };
    AppMotion_HandleCommand(&stopCommand);

    /* MOTION_STOP brakes (the existing 120ms active-brake window) when a
     * motor-driving state is interrupted, so IDLE is reached only after
     * the brake deadline: advance the fake clock past it and tick once. */
    if (AppMotion_GetState() == APP_MOTION_BRAKING)
    {
        Fake_SetTickCount(Fake_GetTickCount() + 200U);
        AppMotion_Update(0.01f);
    }

    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
}

static void RunPreSteerDirectionCase(
    MotionType direction,
    int32_t expectedSeedPulse
)
{
    AppMotionTelemetry telemetry;
    unsigned tick;

    ResetToIdle();
    AppMotion_SetPreSteer(200.0f, true);

    Fake_SetTickCount(1000U);
    Fake_SetEncoderLeft(500);
    Fake_SetEncoderRight(500);
    Fake_SetHeading(0.0f);
    Fake_ResetMotorCallCounts();

    MotionCommand command = { direction, 90 };
    AppMotion_HandleCommand(&command);

    /* --- Entry: PRE_STEER state, seed pulse, motors held at zero
     * output (AppMotor_Stop, NOT AppMotor_Brake -- correction #1). -- */
    CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);
    CHECK_LONG_EQ(AppServo_GetUs(), expectedSeedPulse);
    CHECK(Fake_GetMotorStopCallCount() >= 1U);
    CHECK(Fake_GetMotorBrakeCallCount() == 0U);

    uint32_t setCallCountAtEntry = Fake_GetMotorSetCallCount();
    int32_t expectedTH =
        (direction == MOTION_FORWARD_LEFT || direction == MOTION_BACKWARD_LEFT) ?
            900 : -900;

    /* --- During PRE: no rear-target ramp, no motor-drive command, no
     * left/right PID update (LO/RO truthfully 0), TH unchanged,
     * ST:PRE, LP/RP ~0 (fake encoders unmoved). ---------------------- */
    for (tick = 0U; tick < 5U; tick++)
    {
        Fake_SetTickCount(1000U + (tick * 10U));
        AppMotion_Update(0.01f);

        AppMotion_GetTelemetry(&telemetry);
        CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);
        CHECK(telemetry.movementPreSteering == 1U);
        CHECK(telemetry.movementBraking == 0U);
        CHECK_LONG_EQ(telemetry.leftOutput, 0);
        CHECK_LONG_EQ(telemetry.rightOutput, 0);
        CHECK_LONG_EQ(telemetry.leftTargetCounts, 500);   /* frozen, never ramped */
        CHECK_LONG_EQ(telemetry.rightTargetCounts, 500);
        CHECK_LONG_EQ(telemetry.leftDisplacementCounts, 0);  /* raw signed from start, encoders unmoved */
        CHECK_LONG_EQ(telemetry.rightDisplacementCounts, 0);
        CHECK_LONG_EQ(telemetry.headingTargetX10, expectedTH);  /* unchanged from entry */
        CHECK_LONG_EQ(AppServo_GetUs(), expectedSeedPulse);     /* still the frozen seed */
    }
    CHECK(Fake_GetMotorSetCallCount() == setCallCountAtEntry);  /* AppMotor_Set() never called during PRE */

    /* --- Transition: re-anchor from FRESH encoder reads (simulating a
     * tiny physical drift during the wait), not the stale command-
     * entry values -- verified by checking the FIRST real TURN tick's
     * ramped target against the fresh base, not the original 500. --- */
    bool isReverseTurn =
        (direction == MOTION_BACKWARD_LEFT) || (direction == MOTION_BACKWARD_RIGHT);

    Fake_SetTickCount(1000U + 250U);   /* past the 200ms PRESTEER duration */
    Fake_SetEncoderLeft(505);
    Fake_SetEncoderRight(507);
    AppMotion_Update(0.01f);           /* still dispatches PRE_STEER this tick; detects deadline, re-anchors, transitions */
    CHECK(AppMotion_GetState() == APP_MOTION_TURN);

    Fake_SetTickCount(1000U + 260U);
    AppMotion_Update(0.01f);           /* NOW dispatches APP_MOTION_TURN for the first time */

    AppMotion_GetTelemetry(&telemetry);
    /* TURN_TARGET_RATE_COUNTS_PER_SEC is 2000.0f (app_motion.c, unchanged
     * by this stage) -- one 0.01s tick ramps by 20 counts, signed by
     * direction, from the RE-ANCHORED base (505/507), not the stale
     * command-entry base (500/500). */
    float expectedLeftTarget = isReverseTurn ? (505.0f - 20.0f) : (505.0f + 20.0f);
    float expectedRightTarget = isReverseTurn ? (507.0f - 20.0f) : (507.0f + 20.0f);
    CHECK_LONG_EQ(telemetry.leftTargetCounts, (int32_t)expectedLeftTarget);
    CHECK_LONG_EQ(telemetry.rightTargetCounts, (int32_t)expectedRightTarget);
    CHECK(telemetry.movementPreSteering == 0U);   /* PRE flag cleared once real TURN begins */
    CHECK(telemetry.mode == APP_TUNE_MOV_TURN);   /* still armed, no re-fire of EV:START (same runSequence) */

    ResetToIdle();
}

int main(void)
{
    AppMotion_Init();

    /* AppMotion_Init()'s boot defaults now include TURNSPEED=1700 and
     * ACKERMANN=0.270,true (physically validated turn configuration) --
     * pinned back to the historic 2000/disabled baseline this file's
     * ramp-delta math (20 counts/0.01s tick, symmetric left/right) was
     * authored against, isolating PRE_STEER's own re-anchoring behavior
     * from TURNSPEED/ACKERMANN, which have their own dedicated coverage
     * elsewhere. */
    AppMotion_SetTurnSpeed(2000U);
    AppMotion_SetAckermann(0.0f, false);

    /* --- 1. SET/GET round trip. -------------------------------------- */
    {
        float durationMs;
        bool enable;

        AppMotion_SetPreSteer(275.0f, true);
        AppMotion_GetPreSteer(&durationMs, &enable);
        CHECK_FLOAT_EQ(durationMs, 275.0f);
        CHECK(enable == true);

        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_GetPreSteer(&durationMs, &enable);
        CHECK_FLOAT_EQ(durationMs, 0.0f);
        CHECK(enable == false);
    }

    /* --- 2. All four FL/FR/BL/BR enter APP_MOTION_PRE_STEER with the
     * correct existing-controller-derived seed pulse, freeze targets,
     * command zero motor output, and re-anchor correctly at the
     * PRE->TURN transition. -------------------------------------------- */
    RunPreSteerDirectionCase(MOTION_FORWARD_LEFT, 1000);
    RunPreSteerDirectionCase(MOTION_FORWARD_RIGHT, 2060);
    RunPreSteerDirectionCase(MOTION_BACKWARD_LEFT, 2060);
    RunPreSteerDirectionCase(MOTION_BACKWARD_RIGHT, 1000);

    /* --- 3. duration=0: distinct from disabled -- still enters
     * APP_MOTION_PRE_STEER (seed pulse still commanded), transitions
     * to APP_MOTION_TURN on the very next tick. ------------------------ */
    {
        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, true);

        Fake_SetTickCount(2000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);
        CHECK_LONG_EQ(AppServo_GetUs(), 1000);

        AppMotion_Update(0.01f);   /* same-iteration tick: deadline already reached (duration 0) */
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        ResetToIdle();
    }

    /* --- 4. Disabled behaviour remains baseline: motionState becomes
     * APP_MOTION_TURN immediately at command entry, never
     * APP_MOTION_PRE_STEER; deadline set at entry as today. ------------ */
    {
        ResetToIdle();
        AppMotion_SetPreSteer(200.0f, false);

        Fake_SetTickCount(3000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);
        Fake_ResetMotorCallCounts();

        MotionCommand command = { MOTION_FORWARD_RIGHT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);   /* never PRE_STEER */

        ResetToIdle();
    }

    /* --- 5. IMU-not-ready abort during PRE: motors/servo left in the
     * existing (unmodified) abort state, terminal event fires. -------- */
    {
        ResetToIdle();
        AppMotion_SetPreSteer(200.0f, true);
        Fake_SetImuReady(true);

        Fake_SetTickCount(4000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);

        MotionCommand command = { MOTION_BACKWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);

        Fake_SetImuReady(false);
        Fake_SetTickCount(4010U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

        Fake_SetImuReady(true);
        ResetToIdle();
    }

    /* --- 6. Timeout semantics: the turn-motion deadline is armed at
     * the PRE->TURN transition, not at command entry -- PRE_STEER's
     * own wait must not be double-counted against the turn budget.
     * MOV_TIMEOUT_BASE_MS=5000, MOV_TURN_TIMEOUT_MS_PER_DEGREE=100
     * (app_motion.c, unchanged) -- FL90's turn budget is
     * 5000+90*100=14000ms once TURN begins. ---------------------------- */
    {
        ResetToIdle();
        AppMotion_SetPreSteer(300.0f, true);

        Fake_SetTickCount(5000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);

        Fake_SetTickCount(5000U + 299U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);  /* still waiting */

        Fake_SetTickCount(5000U + 300U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);       /* transitioned, deadline armed HERE */

        /* 100ms before what the deadline would already have hit if it
         * had (incorrectly) been armed at command entry (5000) instead
         * of at the transition (5300) -- heading never settles
         * (headingError stays 90, outside tolerance), so only the
         * timeout path can fire. */
        Fake_SetHeading(0.0f);
        Fake_SetTickCount(5000U + 300U + 14000U - 100U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);       /* NOT yet BRAKING */

        ResetToIdle();
    }

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
