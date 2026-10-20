/* Host-buildable regression test for the unified movement telemetry
 * stage (MM, line, see AppMotionTelemetry's new fields in
 * app_motion.h and the servoPulse/tuneTelemetry writes in
 * app_motion.c's APP_MOTION_DISTANCE / APP_MOTION_TURN cases and
 * AppMotion_BeginBrake()). Exercised directly against the real
 * Core/Src/app_motion.c (+ Core/Src/app_pid.c), the same way
 * tests/motion/app_motion_profile_test.c already does -- see that
 * file's own header comment for why AppMotion_Init() must be called
 * first and why tests/motion/fakes/hw_stubs.c provides real (if
 * trivial) hardware-abstraction stub bodies.
 *
 * This test additionally uses the settable fakes added alongside this
 * stage (tests/motion/fakes/test_fakes.h: Fake_SetEncoderLeft/Right,
 * Fake_SetHeading, Fake_SetGyroZ, Fake_SetTickCount) to drive
 * synthetic encoder/heading/gyro/time values through the real
 * AppMotion_HandleCommand()/AppMotion_Update(), so the new elapsedMs/
 * left-and-rightDisplacementCounts/headingYawRateX10(for turns)
 * fields can be checked against known inputs. app_comm.c's actual
 * wire-format output (MM,/EV:START/EV:END text) is NOT exercised here
 * -- app_comm.c pulls in HAL/cmsis_os/vision_rx and has never been
 * host-buildable in this repo (tests/parser/ only ever builds
 * vision_rx.c standalone); that layer is bench-verified instead (see
 * the implementation plan's physical validation section).
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -Itests/motion/fakes \
 *       -o tests/motion/app_motion_telemetry_test.exe \
 *       tests/motion/app_motion_telemetry_test.c Core/Src/app_motion.c \
 *       Core/Src/app_pid.c tests/motion/fakes/hw_stubs.c
 *   ./tests/motion/app_motion_telemetry_test.exe
 */

#include "app_motion.h"
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

/* Mirrors AppMotion_ToTenths()'s exact rounding rule (static in
 * app_motion.c, not exposed) so headingYawRateX10/etc. can be checked
 * against a known float input. */
static int32_t ExpectedTenths(float value)
{
    float scaled = value * 10.0f;

    if (scaled >= 0.0f)
    {
        scaled += 0.5f;
    }
    else
    {
        scaled -= 0.5f;
    }

    return (int32_t)scaled;
}

/* Drains the motion state machine back to APP_MOTION_IDLE between test
 * segments -- MOTION_STOP is always accepted regardless of current
 * state (see AppMotion_HandleCommand()'s own busy-gate, which only
 * rejects non-STOP commands while busy). */
static void ResetToIdle(void)
{
    MotionCommand stopCommand = { MOTION_STOP, 0 };
    AppMotion_HandleCommand(&stopCommand);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
}

/* F/B: verifies fresh-command telemetry arming (mode/active/
 * runSequence/commandValue/activeDirection/movementStartTick), the
 * existing (unmodified) MOVCAL-based target formula
 * (targetCounts = cm*75+100 with the boot-default DEFAULT_MOVCAL_*
 * constants) as a bonus cross-check that this stage didn't disturb
 * it, and -- the actual point of this stage -- elapsedMs and RAW
 * SIGNED (not direction-normalized) leftDisplacementCounts/
 * rightDisplacementCounts after a simulated tick of encoder motion. */
static void RunDistanceCase(
    MotionType direction,
    int32_t valueCm,
    int32_t startEncoder,
    int32_t encoderDeltaAfterTick
)
{
    AppMotionTelemetry telemetry;
    MotionCommand command = { direction, valueCm };
    uint32_t runSequenceBefore;

    ResetToIdle();

    Fake_SetTickCount(1000U);
    Fake_SetEncoderLeft(startEncoder);
    Fake_SetEncoderRight(startEncoder);
    Fake_SetHeading(0.0f);
    Fake_SetGyroZ(0.0f);

    AppMotion_GetTelemetry(&telemetry);
    runSequenceBefore = telemetry.runSequence;

    AppMotion_HandleCommand(&command);

    /* AppMotion_GetTelemetry() reads the published mailbox, which is
     * only refreshed by AppMotion_PublishTelemetry() -- called from
     * inside the APP_MOTION_DISTANCE tick, not at command entry. Force
     * one tick (encoders/heading unchanged from entry) so the
     * entry-time fields set directly on tuneTelemetry above become
     * observable before checking them. */
    AppMotion_Update(0.001f);

    AppMotion_GetTelemetry(&telemetry);
    CHECK(telemetry.mode == APP_TUNE_MOV_DISTANCE);
    CHECK(telemetry.active == 1U);
    CHECK(telemetry.runSequence != runSequenceBefore);
    CHECK_LONG_EQ(telemetry.commandValue, valueCm);
    CHECK(telemetry.activeDirection == direction);
    CHECK_LONG_EQ(telemetry.movementStartTick, 1000U);

    /* Bonus regression check: DEFAULT_MOVCAL_* (75 counts/cm + 100
     * offset, enabled) is unchanged by this stage -- targetCounts for
     * 30 cm is 30*75+100 = 2350, not ENCODER_COUNTS_PER_CM(72)*30. */
    if (valueCm == 30)
    {
        int32_t expectedTarget =
            (direction == MOTION_FORWARD) ?
                (startEncoder + 2350) : (startEncoder - 2350);
        CHECK_LONG_EQ(telemetry.leftTargetCounts, expectedTarget);
    }

    Fake_SetTickCount(1150U);
    Fake_SetEncoderLeft(startEncoder + encoderDeltaAfterTick);
    Fake_SetEncoderRight(startEncoder + encoderDeltaAfterTick);

    AppMotion_Update(0.15f);

    AppMotion_GetTelemetry(&telemetry);
    CHECK_LONG_EQ(telemetry.elapsedMs, 150U);
    CHECK_LONG_EQ(telemetry.leftDisplacementCounts, encoderDeltaAfterTick);
    CHECK_LONG_EQ(telemetry.rightDisplacementCounts, encoderDeltaAfterTick);
    CHECK(telemetry.movementBraking == 0U);
    CHECK(telemetry.activeDirection == direction);
}

/* FL/FR/BL/BR: verifies the same arming/elapsedMs/displacement
 * behavior as RunDistanceCase(), plus the per-direction target-heading
 * sign (TH) -- L is always +value, R is always -value, independent of
 * F/B -- and headingYawRateX10 (GZ), which turns did not populate at
 * all before this stage. */
static void RunTurnCase(
    MotionType direction,
    int32_t valueDeg,
    int32_t expectedHeadingTargetX10,
    int32_t startEncoder,
    int32_t encoderDeltaAfterTick
)
{
    AppMotionTelemetry telemetry;
    MotionCommand command = { direction, valueDeg };
    uint32_t runSequenceBefore;

    ResetToIdle();

    Fake_SetTickCount(2000U);
    Fake_SetEncoderLeft(startEncoder);
    Fake_SetEncoderRight(startEncoder);
    Fake_SetHeading(0.0f);
    Fake_SetGyroZ(0.0f);

    AppMotion_GetTelemetry(&telemetry);
    runSequenceBefore = telemetry.runSequence;

    AppMotion_HandleCommand(&command);

    /* See RunDistanceCase()'s comment -- force the first publish. */
    AppMotion_Update(0.001f);

    AppMotion_GetTelemetry(&telemetry);
    CHECK(telemetry.mode == APP_TUNE_MOV_TURN);
    CHECK(telemetry.active == 1U);
    CHECK(telemetry.runSequence != runSequenceBefore);
    CHECK_LONG_EQ(telemetry.commandValue, valueDeg);
    CHECK(telemetry.activeDirection == direction);
    CHECK_LONG_EQ(telemetry.movementStartTick, 2000U);

    Fake_SetTickCount(2200U);
    Fake_SetEncoderLeft(startEncoder + encoderDeltaAfterTick);
    Fake_SetEncoderRight(startEncoder + encoderDeltaAfterTick);
    Fake_SetGyroZ(12.5f);

    AppMotion_Update(0.2f);

    AppMotion_GetTelemetry(&telemetry);
    CHECK_LONG_EQ(telemetry.elapsedMs, 200U);
    CHECK_LONG_EQ(telemetry.leftDisplacementCounts, encoderDeltaAfterTick);
    CHECK_LONG_EQ(telemetry.rightDisplacementCounts, encoderDeltaAfterTick);
    CHECK_LONG_EQ(telemetry.headingTargetX10, expectedHeadingTargetX10);
    CHECK_LONG_EQ(telemetry.headingYawRateX10, ExpectedTenths(12.5f));
    CHECK(telemetry.movementBraking == 0U);
    CHECK(telemetry.activeDirection == direction);
}

int main(void)
{
    AppMotion_Init();

    /* --- F/B: elapsedMs, RAW SIGNED displacement (positive for F,
     * negative for B -- correction #4), MOVCAL target cross-check
     * (correction #2). ------------------------------------------- */
    RunDistanceCase(MOTION_FORWARD, 30, 500, 150);
    RunDistanceCase(MOTION_BACKWARD, 30, 500, -150);

    /* --- FL/FR/BL/BR: elapsedMs, RAW SIGNED displacement, per-
     * direction TH sign after a simulated IMU,ZERO (heading==0 at
     * start) -- FL/BL:+900, FR/BR:-900 (correction #3), GZ populated
     * for the first time. -------------------------------------------- */
    RunTurnCase(MOTION_FORWARD_LEFT,   90,  900, 500, 150);
    RunTurnCase(MOTION_FORWARD_RIGHT,  90, -900, 500, 150);
    RunTurnCase(MOTION_BACKWARD_LEFT,  90,  900, 500, -150);
    RunTurnCase(MOTION_BACKWARD_RIGHT, 90, -900, 500, -150);

    /* --- F/B and FL vs FR independence: a Forward-Left run's
     * displacement/heading must not leak into a following Forward-
     * Right run (each RunTurnCase() call above starts from a fresh
     * ResetToIdle() + fake reset, so this is implicitly covered by
     * the six calls above all passing with their own distinct
     * expected values; one more direct check here for clarity). ---- */
    {
        AppMotionTelemetry telemetry;

        ResetToIdle();
        Fake_SetTickCount(3000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);

        MotionCommand fl = { MOTION_FORWARD_LEFT, 45 };
        AppMotion_HandleCommand(&fl);
        AppMotion_Update(0.001f);
        AppMotion_GetTelemetry(&telemetry);
        CHECK_LONG_EQ(telemetry.headingTargetX10, 450);

        ResetToIdle();
        Fake_SetHeading(0.0f);

        MotionCommand fr = { MOTION_FORWARD_RIGHT, 45 };
        AppMotion_HandleCommand(&fr);
        AppMotion_Update(0.001f);
        AppMotion_GetTelemetry(&telemetry);
        CHECK_LONG_EQ(telemetry.headingTargetX10, -450);
    }

    /* --- Completion still transitions to APP_MOTION_BRAKING exactly
     * as before this stage (AppMotion_BeginBrake()'s condition was
     * extended to include APP_TUNE_MOV_TURN, but its brake-entry
     * behavior -- AppMotor_Brake()/AppServo_Center()/deadline math/
     * motionState transition -- is otherwise byte-identical). Checked
     * for both a DISTANCE and a TURN completion. PRESTEER is
     * explicitly disabled for this segment -- its default flipped to
     * enabled (150ms, physically validated) in the PRE_STEER
     * finalization stage, and this segment's turn case intentionally
     * exercises APP_MOTION_TURN's own direct completion path, not
     * PRE_STEER's -- that path (PRE_STEER -> TURN -> BRAKING) has its
     * own dedicated coverage in app_motion_presteer_test.c. --------- */
    {
        AppMotion_SetPreSteer(0.0f, false);

        ResetToIdle();
        Fake_SetTickCount(4000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);

        MotionCommand forward = { MOTION_FORWARD, 1 };
        AppMotion_HandleCommand(&forward);
        /* 1 cm -> distanceCounts = 1*75+100 = 175; settle by placing
         * both encoders exactly on target for POSITION_TOLERANCE_COUNTS
         * (15) worth of consecutive ticks. */
        Fake_SetEncoderLeft(175);
        Fake_SetEncoderRight(175);
        for (int i = 0; i < 6; i++)
        {
            AppMotion_Update(0.01f);
        }
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

        ResetToIdle();
        Fake_SetTickCount(5000U);
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        Fake_SetHeading(0.0f);

        MotionCommand turnLeft = { MOTION_FORWARD_LEFT, 5 };
        AppMotion_HandleCommand(&turnLeft);
        Fake_SetHeading(5.0f);
        for (int i = 0; i < 6; i++)
        {
            AppMotion_Update(0.01f);
        }
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
    }

    ResetToIdle();

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
