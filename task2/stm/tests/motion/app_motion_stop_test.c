/* Host-buildable regression test for the internal MOTION_STOP handling
 * (AppMotion_HandleStop() in Core/Src/app_motion.c): STOP interrupts
 * active movement through the existing, unchanged 120ms active brake,
 * supersedes any pending completion event, and ends in exactly one
 * APP_MOTION_EVENT_STOPPED. STOP is not reachable from UART in this
 * stage -- these tests call AppMotion_HandleCommand() directly.
 *
 * "Exactly one event / the old event never appears" is proven with the
 * single-slot event mailbox by checking that the event sequence number
 * advances by exactly 1 per stop operation and does not move again on
 * further Update() ticks. Time is the fake tick clock (1 tick == 1 ms in
 * tests/motion/fakes/FreeRTOS.h, MOTION_BRAKE_DURATION_MS == 120).
 *
 * Build (from RTOSV/):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -Itests/motion/fakes \
 *       -o tests/motion/app_motion_stop_test.exe \
 *       tests/motion/app_motion_stop_test.c Core/Src/app_motion.c \
 *       Core/Src/app_pid.c Core/Src/app_ir_edge.c Core/Src/app_ir_dist.c tests/motion/fakes/hw_stubs.c
 *   ./tests/motion/app_motion_stop_test.exe
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

#define BRAKE_MS 120U

/* F/B target: MOVCAL is enabled by the PRECISION boot default
 * (75 counts/cm + 100 offset), so 50 cm == 3850 counts from a zero start. */
#define MOV_50CM_TARGET_COUNTS 3850

static AppMotionEvent LastEvent(void)
{
    AppMotionEvent event;
    AppMotion_GetEvent(&event);
    return event;
}

static uint32_t EventSeq(void)
{
    return LastEvent().sequence;
}

static void SendStop(void)
{
    MotionCommand stopCommand = { MOTION_STOP, 0 };
    AppMotion_HandleCommand(&stopCommand);
}

static void Tick(uint32_t tick)
{
    Fake_SetTickCount(tick);
    AppMotion_Update(0.01f);
}

/* Moves the fake clock well past any brake deadline and ticks once. */
static void Drain(void)
{
    Tick(Fake_GetTickCount() + 200U);
}

/* Brings any state back to IDLE without caring about events. */
static void ResetToIdle(void)
{
    SendStop();
    if (AppMotion_GetState() == APP_MOTION_BRAKING)
    {
        Drain();
    }
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
}

/* Starts every block from a known state: idle, clean fakes, boot-default
 * turn config (PRESTEER enabled; individual blocks disable it as needed). */
static void Begin(uint32_t startTick)
{
    ResetToIdle();
    AppMotion_SetPreSteer(150.0f, true);
    Fake_SetImuReady(true);
    Fake_SetTickCount(startTick);
    Fake_SetEncoderLeft(0);
    Fake_SetEncoderRight(0);
    Fake_SetHeading(0.0f);
    Fake_SetGyroZ(0.0f);
    Fake_ResetMotorCallCounts();
}

/* Asserts the final published event is exactly one new STOPPED event
 * since seq0, carrying the expected command and all-zero payload. */
static void ExpectOneStopped(uint32_t seq0, MotionType command)
{
    AppMotionEvent event = LastEvent();

    CHECK_LONG_EQ(event.sequence, seq0 + 1U);
    CHECK(event.type == APP_MOTION_EVENT_STOPPED);
    CHECK(event.command == command);
    CHECK_LONG_EQ(event.value, 0);
    CHECK_LONG_EQ(event.left, 0);
    CHECK_LONG_EQ(event.right, 0);
    CHECK_LONG_EQ(event.headingX10, 0);
}

/* Further ticks after the stop operation has finished must not publish
 * anything (the superseded event never appears late). */
static void ExpectNoLateEvent(void)
{
    uint32_t seq = EventSeq();
    uint32_t i;

    for (i = 0U; i < 5U; i++)
    {
        Tick(Fake_GetTickCount() + 1000U);
    }

    CHECK_LONG_EQ(EventSeq(), seq);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
}

/* Starts MOV,<dir>,50 and returns once in APP_MOTION_DISTANCE. */
static void StartMove50(MotionType direction)
{
    MotionCommand command = { direction, 50 };
    AppMotion_HandleCommand(&command);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
}

/* Drives a started MOV,F,50 across its target so the next Update() runs
 * the normal completion path: BeginBrake(MOV_DONE). Returns the tick at
 * which braking began. */
static uint32_t ForceNormalCompletionBrake(uint32_t tick)
{
    Fake_SetEncoderLeft(MOV_50CM_TARGET_COUNTS);
    Fake_SetEncoderRight(MOV_50CM_TARGET_COUNTS);
    Tick(tick);
    return tick;
}

int main(void)
{
    AppMotion_Init();

    /* --- 1. Idle STOP: publishes STOPPED immediately, stays idle, never
     * brakes, re-asserts motor stop and servo centre. ------------------- */
    {
        uint32_t seq0;
        MotionCommand servo = { MOTION_SERVO_SET, 1800 };

        Begin(1000U);
        AppMotion_HandleCommand(&servo);
        CHECK_LONG_EQ(AppServo_GetUs(), 1800);
        Fake_ResetMotorCallCounts();

        seq0 = EventSeq();
        SendStop();

        CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
        ExpectOneStopped(seq0, MOTION_STOP);
        CHECK_LONG_EQ(Fake_GetMotorBrakeCallCount(), 0);
        CHECK(Fake_GetMotorStopCallCount() >= 1U);
        CHECK_LONG_EQ(AppServo_GetUs(), 1530);
        ExpectNoLateEvent();
    }

    /* --- 2. Two idle STOPs: each is its own stop operation and gets its
     * own STOPPED. ------------------------------------------------------ */
    {
        uint32_t seq0;

        Begin(1000U);
        seq0 = EventSeq();

        SendStop();
        ExpectOneStopped(seq0, MOTION_STOP);
        SendStop();
        CHECK_LONG_EQ(EventSeq(), seq0 + 2U);
        CHECK(LastEvent().type == APP_MOTION_EVENT_STOPPED);
        CHECK(LastEvent().command == MOTION_STOP);
        CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    }

    /* --- 3. STOP mid MOV,F/B: active brake, original completion never
     * reported, STOPPED exactly at the end of the 120ms window. --------- */
    {
        MotionType directions[] = { MOTION_FORWARD, MOTION_BACKWARD };
        unsigned i;

        for (i = 0U; i < 2U; i++)
        {
            AppMotionTelemetry telemetry;
            uint32_t seq0;
            uint32_t cancels;
            int32_t target = (directions[i] == MOTION_FORWARD) ?
                MOV_50CM_TARGET_COUNTS : -MOV_50CM_TARGET_COUNTS;

            Begin(2000U);
            StartMove50(directions[i]);
            Tick(2010U);
            CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);

            Fake_ResetMotorCallCounts();
            cancels = Fake_GetImuCancelCallCount();
            seq0 = EventSeq();

            Fake_SetTickCount(2020U);          /* STOP at T = 2020 */
            SendStop();

            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            CHECK_LONG_EQ(Fake_GetMotorBrakeCallCount(), 1);
            CHECK_LONG_EQ(AppServo_GetUs(), 1530);
            CHECK_LONG_EQ(EventSeq(), seq0);   /* nothing published yet */
            CHECK_LONG_EQ(Fake_GetImuCancelCallCount(), cancels);

            AppMotion_GetTelemetry(&telemetry);
            CHECK(telemetry.active == 0U);
            CHECK(telemetry.mode == APP_TUNE_NONE);

            /* Move the encoders past the target mid-brake and tick up to
             * the last tick before the deadline: still braking, and no
             * MOV_DONE (the distance logic is not evaluated in BRAKING). */
            Fake_SetEncoderLeft(target);
            Fake_SetEncoderRight(target);
            Tick(2020U + BRAKE_MS - 1U);
            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            CHECK_LONG_EQ(EventSeq(), seq0);

            Tick(2020U + BRAKE_MS);            /* deadline reached */
            CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
            ExpectOneStopped(seq0, directions[i]);
            ExpectNoLateEvent();
        }
    }

    /* --- 5. STOP during PRE_STEER: never starts the turn, servo
     * centres at once, one STOPPED with the turn command. -------------- */
    {
        MotionType directions[] = {
            MOTION_FORWARD_LEFT, MOTION_FORWARD_RIGHT,
            MOTION_BACKWARD_LEFT, MOTION_BACKWARD_RIGHT
        };
        uint32_t seedPulse[] = { 1000U, 2060U, 2060U, 1000U };
        unsigned i;

        for (i = 0U; i < 4U; i++)
        {
            uint32_t seq0;
            MotionCommand command = { directions[i], 90 };

            Begin(3000U);
            AppMotion_HandleCommand(&command);
            CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);
            CHECK_LONG_EQ(AppServo_GetUs(), seedPulse[i]);

            Tick(3010U);
            CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);

            Fake_ResetMotorCallCounts();
            seq0 = EventSeq();
            Fake_SetTickCount(3100U);        /* brake window: 3100..3220 */
            SendStop();

            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            CHECK_LONG_EQ(Fake_GetMotorBrakeCallCount(), 1);
            CHECK_LONG_EQ(AppServo_GetUs(), 1530);

            /* Past the PRESTEER deadline (3000+150) but inside the brake
             * window: must not transition to TURN. */
            Tick(3160U);
            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

            Drain();
            CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
            ExpectOneStopped(seq0, directions[i]);
            ExpectNoLateEvent();
        }
    }

    /* --- 6. STOP during TURN, with the predictive TURNBRAKE condition
     * armed and then the dwell condition satisfied while braking: neither
     * completion nor a TB snapshot may appear. -------------------------- */
    {
        AppMotionTurnBrakeSnapshot snapshot;
        uint32_t snapshotSeq;
        uint32_t seq0;
        unsigned tick;
        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };

        Begin(4000U);
        AppMotion_SetPreSteer(0.0f, false);
        AppMotion_SetTurnBrake(135.0f, true);

        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetHeading(85.0f);
        Fake_SetGyroZ(60.0f);
        Tick(4010U);                         /* qualifying tick 1 of 2 */
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        AppMotion_GetTurnBrakeSnapshot(&snapshot);
        snapshotSeq = snapshot.sequence;
        seq0 = EventSeq();

        Fake_SetTickCount(4020U);
        SendStop();
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
        CHECK_LONG_EQ(AppServo_GetUs(), 1530);

        /* Would have fired the 2nd qualifying tick / a dwell completion. */
        Fake_SetHeading(90.0f);
        Fake_SetGyroZ(0.0f);
        for (tick = 0U; tick < 8U; tick++)
        {
            Tick(4030U + (tick * 10U));      /* up to 4100, deadline 4140 */
        }
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
        CHECK_LONG_EQ(EventSeq(), seq0);

        Drain();
        ExpectOneStopped(seq0, MOTION_FORWARD_LEFT);
        AppMotion_GetTurnBrakeSnapshot(&snapshot);
        CHECK_LONG_EQ(snapshot.sequence, snapshotSeq);
        ExpectNoLateEvent();

        AppMotion_SetPreSteer(150.0f, true);
    }

    /* --- 7. STOP during a normal-completion brake (MOV_DONE pending):
     * the original brake deadline is neither restarted nor extended, and
     * MOV_DONE is never published. A control run first proves that, with
     * no STOP, this exact scenario does publish MOV_DONE. --------------- */
    {
        uint32_t seq0;
        uint32_t brakeStart;

        /* Control: no STOP -> MOV_DONE at the brake deadline. */
        Begin(5000U);
        StartMove50(MOTION_FORWARD);
        brakeStart = ForceNormalCompletionBrake(5010U);
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
        Tick(brakeStart + BRAKE_MS);
        CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
        CHECK(LastEvent().type == APP_MOTION_EVENT_MOV_DONE);
        CHECK(LastEvent().command == MOTION_FORWARD);

        /* With STOP at brakeStart + 60. */
        Begin(6000U);
        StartMove50(MOTION_FORWARD);
        brakeStart = ForceNormalCompletionBrake(6010U);
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

        seq0 = EventSeq();
        Fake_ResetMotorCallCounts();
        Fake_SetTickCount(brakeStart + 60U);
        SendStop();
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
        CHECK_LONG_EQ(EventSeq(), seq0);
        CHECK_LONG_EQ(Fake_GetMotorBrakeCallCount(), 0);   /* not re-braked */

        Tick(brakeStart + BRAKE_MS - 1U);    /* original deadline - 1 */
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
        CHECK_LONG_EQ(EventSeq(), seq0);

        Tick(brakeStart + BRAKE_MS);         /* original deadline: NOT +60 */
        CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
        ExpectOneStopped(seq0, MOTION_FORWARD);
        ExpectNoLateEvent();
    }

    /* --- 8. STOP while MOTION_TIMEOUT is the pending completion. ------ */
    {
        uint32_t seq0;

        /* Control: no STOP -> MOTION_TIMEOUT. */
        Begin(10000U);
        StartMove50(MOTION_FORWARD);
        Tick(10000U + 60000U);               /* far beyond the deadline */
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
        Drain();
        CHECK(LastEvent().type == APP_MOTION_EVENT_MOTION_TIMEOUT);

        Begin(10000U);
        StartMove50(MOTION_FORWARD);
        Tick(10000U + 60000U);
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

        seq0 = EventSeq();
        SendStop();
        Drain();
        CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
        ExpectOneStopped(seq0, MOTION_FORWARD);
        ExpectNoLateEvent();
    }

    /* --- 9. STOP while IMU_NOT_READY is the pending completion. ------- */
    {
        uint32_t seq0;

        /* Control: no STOP -> IMU_NOT_READY. */
        Begin(20000U);
        StartMove50(MOTION_FORWARD);
        Fake_SetImuReady(false);
        Tick(20010U);
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
        Drain();
        CHECK(LastEvent().type == APP_MOTION_EVENT_IMU_NOT_READY);

        Begin(20000U);
        StartMove50(MOTION_FORWARD);
        Fake_SetImuReady(false);
        Tick(20010U);
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

        seq0 = EventSeq();
        SendStop();
        Drain();
        CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
        ExpectOneStopped(seq0, MOTION_FORWARD);
        ExpectNoLateEvent();
        Fake_SetImuReady(true);
    }

    /* --- 10. Normal brake whose deadline has already passed when STOP
     * arrives (the real loop runs HandleCommand before Update in the same
     * iteration): STOP still wins, MOV_DONE never published. ------------- */
    {
        uint32_t seq0;
        uint32_t brakeStart;

        Begin(30000U);
        StartMove50(MOTION_FORWARD);
        brakeStart = ForceNormalCompletionBrake(30010U);
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

        seq0 = EventSeq();
        Fake_SetTickCount(brakeStart + BRAKE_MS + 50U);   /* past deadline */
        SendStop();                                        /* command first */
        CHECK_LONG_EQ(EventSeq(), seq0);
        AppMotion_Update(0.01f);                           /* then update */

        CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
        ExpectOneStopped(seq0, MOTION_FORWARD);
        ExpectNoLateEvent();
    }

    /* --- 11. Duplicate STOP during a STOP brake is coalesced: no extra
     * event, no restarted/extended deadline, no second brake. ----------- */
    {
        uint32_t seq0;

        Begin(40000U);
        StartMove50(MOTION_FORWARD);
        Tick(40010U);

        seq0 = EventSeq();
        Fake_SetTickCount(40020U);           /* first STOP at T = 40020 */
        SendStop();
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
        CHECK_LONG_EQ(Fake_GetMotorBrakeCallCount(), 1);

        Fake_SetTickCount(40080U);           /* second STOP at T + 60 */
        SendStop();
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
        CHECK_LONG_EQ(EventSeq(), seq0);
        CHECK_LONG_EQ(Fake_GetMotorBrakeCallCount(), 1);

        Tick(40020U + BRAKE_MS - 1U);        /* original deadline - 1 */
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
        CHECK_LONG_EQ(EventSeq(), seq0);

        Tick(40020U + BRAKE_MS);             /* original deadline */
        CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
        ExpectOneStopped(seq0, MOTION_FORWARD);   /* +1 in total */
        ExpectNoLateEvent();
    }

    /* --- 12/13. STOP during TUNE_*: tune telemetry cancelled with no
     * TUNE completion, one STOPPED with the tune command. --------------- */
    {
        MotionCommand tuneCommands[] = {
            { MOTION_TUNE_LEFT, 1000 },
            { MOTION_TUNE_RIGHT, 1000 },
            { MOTION_TUNE_BOTH, 1000 },
            { MOTION_TUNE_STEERING_FORWARD, 3 },
            { MOTION_TUNE_STEERING_BACKWARD, 3 }
        };
        AppMotionState tuneStates[] = {
            APP_MOTION_TUNE_LEFT, APP_MOTION_TUNE_RIGHT,
            APP_MOTION_TUNE_BOTH, APP_MOTION_TUNE_STEERING,
            APP_MOTION_TUNE_STEERING
        };
        unsigned i;

        for (i = 0U; i < 5U; i++)
        {
            AppMotionTelemetry telemetry;
            uint32_t seq0;
            uint32_t completionSeq;

            Begin(50000U);
            AppMotion_HandleCommand(&tuneCommands[i]);
            CHECK(AppMotion_GetState() == tuneStates[i]);
            Tick(50010U);

            AppMotion_GetTelemetry(&telemetry);
            CHECK(telemetry.active == 1U);
            completionSeq = telemetry.completionSequence;

            seq0 = EventSeq();
            Fake_ResetMotorCallCounts();
            Fake_SetTickCount(50020U);
            SendStop();

            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            CHECK_LONG_EQ(Fake_GetMotorBrakeCallCount(), 1);
            AppMotion_GetTelemetry(&telemetry);
            CHECK(telemetry.active == 0U);
            CHECK(telemetry.mode == APP_TUNE_NONE);
            CHECK_LONG_EQ(telemetry.completionSequence, completionSeq);

            /* The tune's own timeout/completion must not run late. */
            Tick(50020U + BRAKE_MS - 1U);
            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            Tick(50020U + BRAKE_MS);
            CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
            ExpectOneStopped(seq0, tuneCommands[i].type);

            AppMotion_GetTelemetry(&telemetry);
            CHECK_LONG_EQ(telemetry.completionSequence, completionSeq);
            ExpectNoLateEvent();
        }
    }

    /* --- 14. STOP during PWM / SPEED diagnostics: the diagnostic's own
     * completion event never appears. ----------------------------------- */
    {
        MotionCommand diagnostics[] = {
            { MOTION_PWM_TEST, 30 },
            { MOTION_SPEED_TEST, 30 }
        };
        AppMotionState diagnosticStates[] = {
            APP_MOTION_DIAGNOSTIC_PWM, APP_MOTION_DIAGNOSTIC_SPEED
        };
        unsigned i;

        for (i = 0U; i < 2U; i++)
        {
            uint32_t seq0;

            Begin(60000U);
            AppMotion_HandleCommand(&diagnostics[i]);
            CHECK(AppMotion_GetState() == diagnosticStates[i]);
            Tick(60010U);
            CHECK(AppMotion_GetState() == diagnosticStates[i]);

            seq0 = EventSeq();
            Fake_ResetMotorCallCounts();
            Fake_SetTickCount(60020U);
            SendStop();
            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            CHECK_LONG_EQ(Fake_GetMotorBrakeCallCount(), 1);

            /* Well past DIAGNOSTIC_RUN_SECONDS of simulated dt, but the
             * diagnostic code is no longer running. */
            AppMotion_Update(2.0f);
            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            CHECK_LONG_EQ(EventSeq(), seq0);

            Drain();
            CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
            ExpectOneStopped(seq0, diagnostics[i].type);
            ExpectNoLateEvent();
        }
    }

    /* --- 15. STOP during HDG,RAW / HDG,F / HDG,B diagnostics. ---------- */
    {
        MotionCommand diagnostics[] = {
            { MOTION_HEADING_RAW, 3 },
            { MOTION_HEADING_FORWARD, 3 },
            { MOTION_HEADING_BACKWARD, 3 }
        };
        AppMotionState diagnosticStates[] = {
            APP_MOTION_DIAGNOSTIC_HEADING_RAW,
            APP_MOTION_DIAGNOSTIC_HEADING,
            APP_MOTION_DIAGNOSTIC_HEADING
        };
        unsigned i;

        for (i = 0U; i < 3U; i++)
        {
            uint32_t seq0;

            Begin(70000U);
            AppMotion_HandleCommand(&diagnostics[i]);
            CHECK(AppMotion_GetState() == diagnosticStates[i]);
            Tick(70010U);
            CHECK(AppMotion_GetState() == diagnosticStates[i]);

            seq0 = EventSeq();
            Fake_ResetMotorCallCounts();
            Fake_SetTickCount(70020U);
            SendStop();
            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            CHECK_LONG_EQ(Fake_GetMotorBrakeCallCount(), 1);

            AppMotion_Update(5.0f);          /* beyond the 3 s duration */
            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            CHECK_LONG_EQ(EventSeq(), seq0);

            Drain();
            CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
            ExpectOneStopped(seq0, diagnostics[i].type);
            ExpectNoLateEvent();
        }
    }

    /* --- 16/17. STOP during IMU,ZERO / IMU,FLIP (WAIT_IMU): cancels the
     * request through the existing reset path, reports immediately, does
     * not brake (no motor is driven), and the 5 s IMU timeout error never
     * appears. ----------------------------------------------------------- */
    {
        MotionCommand requests[] = {
            { MOTION_IMU_ZERO, 0 },
            { MOTION_IMU_FLIP, 0 }
        };
        unsigned i;

        for (i = 0U; i < 2U; i++)
        {
            uint32_t seq0;
            uint32_t cancels;

            Begin(80000U);
            Fake_SetImuEvent(APP_IMU_EVENT_NONE, 0U);
            AppMotion_HandleCommand(&requests[i]);
            CHECK(AppMotion_GetState() == APP_MOTION_WAIT_IMU);
            Tick(80010U);
            CHECK(AppMotion_GetState() == APP_MOTION_WAIT_IMU);

            seq0 = EventSeq();
            cancels = Fake_GetImuCancelCallCount();
            Fake_ResetMotorCallCounts();
            SendStop();

            CHECK(AppMotion_GetState() == APP_MOTION_IDLE);   /* no brake */
            CHECK_LONG_EQ(Fake_GetImuCancelCallCount(), cancels + 1U);
            CHECK_LONG_EQ(Fake_GetMotorBrakeCallCount(), 0);
            ExpectOneStopped(seq0, requests[i].type);

            AppMotion_Update(6.0f);          /* > IMU_REQUEST_TIMEOUT_SECONDS */
            CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
            ExpectNoLateEvent();
        }
    }

    /* --- 18. A fresh movement works normally after a stop: after an
     * active-state STOP, a brake-phase STOP, and a WAIT_IMU STOP. -------- */
    {
        unsigned scenario;

        for (scenario = 0U; scenario < 3U; scenario++)
        {
            AppMotionTelemetry telemetry;
            uint32_t runSeq0;
            uint32_t seq0;
            uint32_t brakeStart;

            Begin(90000U);

            if (scenario == 0U)
            {
                StartMove50(MOTION_FORWARD);
                Tick(90010U);
                SendStop();
                Drain();
            }
            else if (scenario == 1U)
            {
                StartMove50(MOTION_FORWARD);
                (void)ForceNormalCompletionBrake(90010U);
                SendStop();
                Drain();
            }
            else
            {
                MotionCommand zero = { MOTION_IMU_ZERO, 0 };
                AppMotion_HandleCommand(&zero);
                SendStop();
            }
            CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
            CHECK(LastEvent().type == APP_MOTION_EVENT_STOPPED);

            AppMotion_GetTelemetry(&telemetry);
            runSeq0 = telemetry.runSequence;

            Fake_SetEncoderLeft(0);
            Fake_SetEncoderRight(0);
            Fake_SetTickCount(91000U);
            seq0 = EventSeq();

            StartMove50(MOTION_FORWARD);
            Tick(91005U);                    /* publishes the new run */
            AppMotion_GetTelemetry(&telemetry);
            CHECK(telemetry.runSequence != runSeq0);
            CHECK(telemetry.active == 1U);

            brakeStart = ForceNormalCompletionBrake(91010U);
            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            Tick(brakeStart + BRAKE_MS);

            CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
            CHECK_LONG_EQ(EventSeq(), seq0 + 1U);
            CHECK(LastEvent().type == APP_MOTION_EVENT_MOV_DONE);
            CHECK(LastEvent().command == MOTION_FORWARD);
            CHECK_LONG_EQ(LastEvent().value, 0);
            ExpectNoLateEvent();
        }
    }

    /* --- 19. Normal turn completion is unchanged by STOP's existence:
     * MOV,FL,90 still completes through the dwell path with MOV_DONE and
     * the usual zero payload. ------------------------------------------- */
    {
        uint32_t seq0;
        unsigned tick;
        MotionCommand command = { MOTION_FORWARD_LEFT, 90 };

        Begin(100000U);
        AppMotion_SetPreSteer(0.0f, false);
        seq0 = EventSeq();

        AppMotion_HandleCommand(&command);
        CHECK(AppMotion_GetState() == APP_MOTION_TURN);

        Fake_SetHeading(90.0f);              /* on target, gyro 0 */
        for (tick = 0U;
             (tick < 20U) && (AppMotion_GetState() == APP_MOTION_TURN);
             tick++)
        {
            Tick(100010U + (tick * 10U));
        }
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

        Drain();
        CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
        CHECK_LONG_EQ(EventSeq(), seq0 + 1U);
        CHECK(LastEvent().type == APP_MOTION_EVENT_MOV_DONE);
        CHECK(LastEvent().command == MOTION_FORWARD_LEFT);
        CHECK_LONG_EQ(LastEvent().value, 0);
        CHECK_LONG_EQ(LastEvent().left, 0);
        CHECK_LONG_EQ(LastEvent().right, 0);
        CHECK_LONG_EQ(LastEvent().headingX10, 0);

        AppMotion_SetPreSteer(150.0f, true);
    }

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
