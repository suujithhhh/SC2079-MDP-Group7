/* Host-buildable regression test for the TURNBRAKE predictive-brake
 * trigger (case APP_MOTION_TURN: in Core/Src/app_motion.c), additive to
 * and independent of the existing heading-tolerance/5-cycle dwell
 * completion. Exercised directly against the real Core/Src/app_motion.c
 * (+ app_pid.c), the same way the existing tests/motion test files do.
 *
 * Every block that issues a MOV,FL/FR/BL/BR command explicitly disables
 * PRE_STEER first (AppMotion_SetPreSteer(0.0f, false)) -- PRE_STEER
 * defaults to ENABLED at boot (DEFAULT_PRESTEER_ENABLED), which would
 * otherwise route command entry through APP_MOTION_PRE_STEER for 150ms
 * before ever reaching APP_MOTION_TURN, unrelated to what this file
 * tests.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -Itests/motion/fakes \
 *       -o tests/motion/app_motion_turnbrake_test.exe \
 *       tests/motion/app_motion_turnbrake_test.c Core/Src/app_motion.c \
 *       Core/Src/app_pid.c Core/Src/app_ir_edge.c Core/Src/app_ir_dist.c tests/motion/fakes/hw_stubs.c
 *   ./tests/motion/app_motion_turnbrake_test.exe
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

int main(void)
{
    AppMotion_Init();

    /* --- 1. Enabled boot default: checked FIRST, before anything below
     * ever calls AppMotion_SetTurnBrake() -- turnBrakeEnabled/
     * turnBrakeLookaheadMs are explicitly restored by AppMotion_Init()
     * to the physically validated turn configuration (135.0ms/enabled),
     * alongside PRESTEER/TURNSPEED/ACKERMANN (see app_motion.c), so a
     * repeated Init() is deterministic -- not just a fresh process
     * start's static initializer. ---------------------------------------- */
    {
        float lookaheadMs;
        bool enable;

        AppMotion_GetTurnBrake(&lookaheadMs, &enable);
        CHECK_NEAR(lookaheadMs, 135.0f, 0.01f);
        CHECK(enable == true);
    }

    /* --- 2. SET/GET storage and clamping at the AppMotion API level
     * (not through the vision_rx.c parser -- that has its own, separately
     * testable range check). --------------------------------------------- */
    {
        float lookaheadMs;
        bool enable;

        AppMotion_SetTurnBrake(135.0f, true);
        AppMotion_GetTurnBrake(&lookaheadMs, &enable);
        CHECK_NEAR(lookaheadMs, 135.0f, 0.01f);
        CHECK(enable == true);

        AppMotion_SetTurnBrake(-50.0f, true);
        AppMotion_GetTurnBrake(&lookaheadMs, &enable);
        CHECK_NEAR(lookaheadMs, 0.0f, 0.01f);   /* clamped to MIN */

        AppMotion_SetTurnBrake(1000.0f, true);
        AppMotion_GetTurnBrake(&lookaheadMs, &enable);
        CHECK_NEAR(lookaheadMs, 500.0f, 0.01f);   /* clamped to MAX */

        AppMotion_SetTurnBrake(0.0f, false);
        AppMotion_GetTurnBrake(&lookaheadMs, &enable);
        CHECK_NEAR(lookaheadMs, 0.0f, 0.01f);   /* boundary MIN, unchanged */
        CHECK(enable == false);

        AppMotion_SetTurnBrake(500.0f, true);
        AppMotion_GetTurnBrake(&lookaheadMs, &enable);
        CHECK_NEAR(lookaheadMs, 500.0f, 0.01f);   /* boundary MAX, unchanged */
    }

    /* --- 3. No predictive trigger while disabled: a condition that WOULD
     * qualify if enabled (approaching, matching-sign yaw rate well above
     * the 5 deg/s floor, remaining angle well under the predicted
     * stopping angle) must not reach APP_MOTION_BRAKING while TURNBRAKE
     * is disabled -- proves byte-equivalent turn behaviour until
     * explicitly enabled. ------------------------------------------------- */
    {
        unsigned tick;

        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnBrake(135.0f, false);

        Fake_SetTickCount(1000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);
        Fake_SetGyroZ(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetHeading(85.0f);   /* headingError=5, remaining=5 */
        Fake_SetGyroZ(60.0f);     /* predicted = 60*0.135=8.1 >= remaining */

        for (tick = 0U; tick < 2U; tick++)
        {
            Fake_SetTickCount(1010U + (tick * 10U));
            AppMotion_Update(0.01f);
        }
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);   /* never BRAKING */

        ResetToIdle();
    }

    /* --- 4. Two consecutive qualifying samples required: the same
     * qualifying condition held for exactly one tick must not fire; a
     * second consecutive qualifying tick must. --------------------------- */
    {
        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnBrake(135.0f, true);

        Fake_SetTickCount(2000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);
        Fake_SetGyroZ(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetHeading(85.0f);
        Fake_SetGyroZ(60.0f);

        Fake_SetTickCount(2010U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);   /* 1st qualifying tick: counter=1 */

        Fake_SetTickCount(2020U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);   /* 2nd: counter=2, fires */

        ResetToIdle();
    }

    /* --- 5. Interrupted qualification resets the counter: qualify (1),
     * miss (reset to 0), qualify again (1, NOT 2 -- must not fire yet),
     * qualify a second consecutive time (2, fires). If the first
     * qualifying tick had wrongly survived the miss, the third tick here
     * would already fire. ------------------------------------------------- */
    {
        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnBrake(135.0f, true);

        Fake_SetTickCount(3000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);
        Fake_SetGyroZ(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetHeading(85.0f);

        Fake_SetGyroZ(60.0f);
        Fake_SetTickCount(3010U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);   /* counter=1 */

        Fake_SetGyroZ(0.0f);   /* miss: below the yaw-rate floor */
        Fake_SetTickCount(3020U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);   /* counter reset to 0 */

        Fake_SetGyroZ(60.0f);
        Fake_SetTickCount(3030U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);   /* counter=1 again, NOT 2 */

        Fake_SetTickCount(3040U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);   /* counter=2, fires */

        ResetToIdle();
    }

    /* --- 6. Wrong-direction yaw cannot trigger: heading still 5 degrees
     * short, but the chassis is (per the fake) rotating away from the
     * target -- headingError*gyroZ is negative -- must never fire via
     * the predictive path (nor via dwell: 5 degrees is outside the
     * +-2 degree tolerance). ---------------------------------------------- */
    {
        unsigned tick;

        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnBrake(135.0f, true);

        Fake_SetTickCount(4000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);
        Fake_SetGyroZ(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetHeading(85.0f);   /* headingError=+5 */
        Fake_SetGyroZ(-60.0f);    /* wrong sign: headingError*gyroZ < 0 */

        for (tick = 0U; tick < 2U; tick++)
        {
            Fake_SetTickCount(4010U + (tick * 10U));
            AppMotion_Update(0.01f);
        }
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        ResetToIdle();
    }

    /* --- 7. Yaw below 5 deg/s cannot trigger: remaining angle (0.3 deg)
     * is small enough that even this sub-threshold rate's OWN predicted
     * stopping angle (3.0*0.135=0.405) would otherwise cover it -- this
     * isolates the fixed noise-floor guard specifically, not just a
     * remaining-vs-predicted mismatch. Only 2 ticks are run so the
     * ordinary dwell counter (needs 5) cannot also explain a completion
     * -- headingError=0.3 is inside +-2 degrees, so dwell WOULD
     * eventually fire on its own, but not within 2 ticks. ---------------- */
    {
        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnBrake(135.0f, true);

        Fake_SetTickCount(5000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);
        Fake_SetGyroZ(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetHeading(89.7f);   /* headingError=0.3, remaining=0.3 */
        Fake_SetGyroZ(3.0f);      /* below TURN_BRAKE_MIN_YAW_RATE_DPS(5.0) */

        Fake_SetTickCount(5010U);
        AppMotion_Update(0.01f);
        Fake_SetTickCount(5020U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        ResetToIdle();
    }

    /* --- 8. Remaining angle above the predicted stopping angle cannot
     * trigger: a large remaining angle with a qualifying-direction,
     * above-floor yaw rate whose OWN predicted stopping angle still
     * falls short of it. -------------------------------------------------- */
    {
        unsigned tick;

        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnBrake(135.0f, true);

        Fake_SetTickCount(6000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);
        Fake_SetGyroZ(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetHeading(70.0f);   /* headingError=20, remaining=20 */
        Fake_SetGyroZ(60.0f);     /* predicted = 60*0.135=8.1 < remaining(20) */

        for (tick = 0U; tick < 2U; tick++)
        {
            Fake_SetTickCount(6010U + (tick * 10U));
            AppMotion_Update(0.01f);
        }
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        ResetToIdle();
    }

    /* --- 9. All four directions: FL/FR/BL/BR each reach
     * APP_MOTION_BRAKING via the predictive path with direction-
     * appropriate target/heading/yaw-rate signs, and the published
     * snapshot reports the correct direction. FL/BL target = entry
     * heading + value (approach needs heading increasing, positive
     * yaw rate); FR/BR target = entry heading - value (approach needs
     * heading decreasing, negative yaw rate) -- see app_motion.c's
     * command-entry target assignment. ------------------------------------ */
    {
        MotionType dirs[4] = {
            MOTION_FORWARD_LEFT, MOTION_FORWARD_RIGHT,
            MOTION_BACKWARD_LEFT, MOTION_BACKWARD_RIGHT
        };
        float signs[4] = { 1.0f, -1.0f, 1.0f, -1.0f };
        unsigned d;

        for (d = 0U; d < 4U; d++)
        {
            uint32_t tickBase = 7000U + (d * 100U);
            AppMotionTurnBrakeSnapshot snapshot;

            ResetToIdle();
            AppMotion_SetPreSteer(0.0f, false);
            AppMotion_SetTurnBrake(135.0f, true);

            Fake_SetTickCount(tickBase);
            Fake_SetEncoderLeft(0);
            Fake_SetEncoderRight(0);
            Fake_SetHeading(0.0f);
            Fake_SetGyroZ(0.0f);

            MotionCommand command = { dirs[d], 90 };
            AppMotion_HandleCommand(&command);
            CHECK(AppMotion_GetState() == APP_MOTION_TURN);

            Fake_SetHeading(signs[d] * 85.0f);   /* 5 degrees short, correct side */
            Fake_SetGyroZ(signs[d] * 60.0f);     /* matching-sign approach rate */

            Fake_SetTickCount(tickBase + 10U);
            AppMotion_Update(0.01f);
            Fake_SetTickCount(tickBase + 20U);
            AppMotion_Update(0.01f);
            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

            AppMotion_GetTurnBrakeSnapshot(&snapshot);
            CHECK(snapshot.direction == dirs[d]);

            ResetToIdle();
        }
    }

    /* --- 10. No qualification carryover after stop/new command: build a
     * 1-tick qualifying streak, abort, start a fresh turn, and confirm a
     * single qualifying tick on the new command does not immediately
     * fire (proves the per-command-entry and AppMotion_StopAndReset()
     * resets both work). ---------------------------------------------- */
    {
        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnBrake(135.0f, true);

        Fake_SetTickCount(8000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);
        Fake_SetGyroZ(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetHeading(85.0f);
        Fake_SetGyroZ(60.0f);
        Fake_SetTickCount(8010U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);   /* counter=1 */

        ResetToIdle();   /* abort mid-streak */
        AppMotion_SetPreSteer(0.0f, false);

        Fake_SetTickCount(9000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);
        Fake_SetGyroZ(0.0f);

        MotionCommand command2 = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command2);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetHeading(85.0f);
        Fake_SetGyroZ(60.0f);
        Fake_SetTickCount(9010U);
        AppMotion_Update(0.01f);
        /* If the counter had carried over, this single tick would
         * already be the SECOND qualifying tick and would fire. */
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        ResetToIdle();
    }

    /* --- 11. Snapshot validity, contents, and sequence increment. ------- */
    {
        AppMotionTurnBrakeSnapshot before;
        AppMotionTurnBrakeSnapshot after;
        AppMotionTurnBrakeSnapshot second;
        uint32_t sequenceAfterFirst;

        /* Fresh AppMotion_Init() re-establishes the documented
         * valid=0/sequence=0 boot contract (matches test 1's own boot-
         * state check), independent of any trigger earlier in this
         * file. */
        AppMotion_Init();
        AppMotion_GetTurnBrakeSnapshot(&before);
        CHECK_LONG_EQ(before.valid, 0);
        CHECK_LONG_EQ(before.sequence, 0);

        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnBrake(135.0f, true);

        Fake_SetTickCount(10000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);
        Fake_SetGyroZ(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetHeading(85.0f);   /* headingError=5, remaining=5 */
        Fake_SetGyroZ(60.0f);     /* predicted=8.1 */

        Fake_SetTickCount(10010U);
        AppMotion_Update(0.01f);
        Fake_SetTickCount(10020U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

        AppMotion_GetTurnBrakeSnapshot(&after);
        CHECK_LONG_EQ(after.valid, 1);
        CHECK(after.sequence != before.sequence);
        CHECK(after.direction == MOTION_FORWARD_LEFT);
        CHECK_LONG_EQ(after.targetHeadingX10, 900);        /* target=90.0 */
        CHECK_LONG_EQ(after.currentHeadingX10, 850);       /* heading=85.0 */
        CHECK_LONG_EQ(after.remainingAngleX10, 50);         /* remaining=5.0 */
        CHECK_LONG_EQ(after.gyroZDpsX10, 600);              /* gyroZ=60.0 */
        CHECK_LONG_EQ(after.predictedStoppingAngleX10, 81); /* 60*0.135=8.1 */
        CHECK_LONG_EQ(after.configuredLookaheadMsX10, 1350); /* 135.0ms */
        CHECK_LONG_EQ(after.confirmationCycles, 2);

        ResetToIdle();

        /* A second, separate trigger must bump the sequence again. */
        sequenceAfterFirst = after.sequence;

        AppMotion_SetPreSteer(0.0f, false);

        Fake_SetTickCount(11000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);
        Fake_SetGyroZ(0.0f);

        MotionCommand command2 = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command2);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetHeading(85.0f);
        Fake_SetGyroZ(60.0f);
        Fake_SetTickCount(11010U);
        AppMotion_Update(0.01f);
        Fake_SetTickCount(11020U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

        AppMotion_GetTurnBrakeSnapshot(&second);
        CHECK(second.sequence == sequenceAfterFirst + 1U);

        ResetToIdle();
    }

    /* --- 12. Existing dwell completion still works while TURNBRAKE is
     * disabled: heading held exactly at target for SETTLED_CYCLES_
     * REQUIRED(5) consecutive ticks must still reach APP_MOTION_BRAKING,
     * unmodified. ----------------------------------------------------- */
    {
        unsigned tick;

        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnBrake(135.0f, false);

        Fake_SetTickCount(12000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);
        Fake_SetGyroZ(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetHeading(90.0f);
        for (tick = 0U; tick < 6U; tick++)
        {
            Fake_SetTickCount(12010U + (tick * 10U));
            AppMotion_Update(0.01f);
        }
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

        ResetToIdle();
    }

    /* --- 13a. Timeout behaviour remains intact (TURNBRAKE enabled, but
     * never given a qualifying condition -- heading never moves, gyroZ
     * stays 0): the turn-motion deadline
     * (MOV_TIMEOUT_BASE_MS(5000)+10*MOV_TURN_TIMEOUT_MS_PER_DEGREE(100)
     * =6000ms for a 10-degree turn) still fires unmodified. -------------- */
    {
        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnBrake(135.0f, true);

        Fake_SetTickCount(13000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);
        Fake_SetGyroZ(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 10 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetTickCount(13000U + 6000U + 10U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

        ResetToIdle();
    }

    /* --- 13b. IMU-not-ready behaviour remains intact: fires immediately
     * on the very first tick, independent of TURNBRAKE. ------------------ */
    {
        ResetToIdle();
        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnBrake(135.0f, true);
        Fake_SetImuReady(true);

        Fake_SetTickCount(14000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);
        Fake_SetGyroZ(0.0f);

        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };
        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetImuReady(false);
        Fake_SetTickCount(14010U);
        AppMotion_Update(0.01f);
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

        Fake_SetImuReady(true);
        ResetToIdle();
    }

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
