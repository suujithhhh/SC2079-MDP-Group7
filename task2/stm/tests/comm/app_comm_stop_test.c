/* Full-stack host test for the Stage 2 wire-level priority STOP.
 *
 * Links the REAL Core/Src/app_comm.c + Core/Src/vision_rx.c +
 * Core/Src/app_motion.c + Core/Src/app_pid.c against the host fakes in
 * tests/motion/fakes (motors, servo, encoders, IMU, tick clock) and
 * tests/comm/fakes (CMSIS queues with fault injection, UART capture).
 *
 * The test plays both RTOS roles explicitly and deterministically:
 *   - "CommTask" side: bytes go through the real CommRxQueue ->
 *     AppComm_ProcessByte() -> vision_rx -> on_*() callbacks, then
 *     AppComm_ServiceTelemetry();
 *   - "MotionTask" side: the body of StartMotionTask() in main.c --
 *     AppComm_TakeMotionCommand() -> AppMotion_HandleCommand(), then
 *     AppMotion_Update().
 * Cycle() advances the fake clock 10 ms and runs MotionTask then CommTask,
 * mirroring the firmware's 10 ms loops. Races are reproduced by calling
 * the halves in the order a given scenario needs.
 *
 * Only "terminal / marker" UART lines are compared (ACK..., ERR,...,
 * MM,EV:..., TUNE,..., ...DONE results); the periodic MM,/AK,/MD,...
 * telemetry is ignored by the filter.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -Itests/comm/fakes -Itests/motion/fakes \
 *       -ICore/Inc -o tests/comm/app_comm_stop_test.exe \
 *       tests/comm/app_comm_stop_test.c Core/Src/app_comm.c \
 *       Core/Src/vision_rx.c Core/Src/app_motion.c Core/Src/app_pid.c Core/Src/app_ir_edge.c Core/Src/app_ir_dist.c \
 *       Core/Src/app_us_park.c tests/motion/fakes/hw_stubs.c \
 *       tests/comm/fakes/comm_stubs.c tests/comm/fakes/ultrasonic_stub.c \
 *       tests/comm/fakes/ir_stub.c
 *   ./tests/comm/app_comm_stop_test.exe
 */

#include "app_comm.h"
#include "app_motion.h"
#include "app_servo.h"
#include "cmsis_os.h"
#include "main.h"
#include "test_fakes.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Defined in tests/comm/fakes/comm_stubs.c (real firmware: main.c). */
extern osMessageQueueId_t MotionQueueHandle;
extern osMessageQueueId_t CommRxQueueHandle;

static unsigned checks;
static unsigned failures;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        failures++; \
        printf("FAIL line %d: %s\n", __LINE__, #expr); \
    } \
} while (0)

#define BRAKE_MS 120U
#define MOV_50CM_TARGET_COUNTS 3850

/* --- glob matching and UART capture helpers ------------------------- */

static int Glob(const char *p, const char *s)
{
    if (*p == '\0') { return *s == '\0'; }
    if (*p == '*')
    {
        for (;; s++)
        {
            if (Glob(p + 1, s)) { return 1; }
            if (*s == '\0') { return 0; }
        }
    }
    if (*p == '?') { return (*s != '\0') && Glob(p + 1, s + 1); }
    return (*p == *s) && Glob(p + 1, s + 1);
}

static const char *const terminalPrefixes[] = {
    "ACK", "ERR,", "MM,EV:", "TUNE,", "PWM,DONE", "SPD,D:", "HDG,",
    "IMU,ZERO,OK", "IMU,FLIP,OK", "ENC,RST,OK", "SERVO,SET", NULL
};

#define LINE_MAX_LEN 96U
#define KEPT_MAX 96U
static char kept[KEPT_MAX][LINE_MAX_LEN];
static unsigned keptCount;

static int IsTerminalLine(const char *line)
{
    unsigned i;

    for (i = 0U; terminalPrefixes[i] != NULL; i++)
    {
        if (strncmp(line, terminalPrefixes[i], strlen(terminalPrefixes[i])) == 0)
        {
            return 1;
        }
    }
    return 0;
}

static void Collect(void)
{
    unsigned total = Comm_TxLineCount();
    unsigned i;
    char line[LINE_MAX_LEN];

    keptCount = 0U;
    for (i = 0U; i < total; i++)
    {
        (void)Comm_TxLine(i, line, sizeof(line));
        if (IsTerminalLine(line) && (keptCount < KEPT_MAX))
        {
            strcpy(kept[keptCount++], line);
        }
    }
}

/* Number of captured lines (all lines / terminal lines only) matching a
 * glob. */
static unsigned CountRaw(const char *pattern)
{
    unsigned total = Comm_TxLineCount();
    unsigned i;
    unsigned n = 0U;
    char line[LINE_MAX_LEN];

    for (i = 0U; i < total; i++)
    {
        (void)Comm_TxLine(i, line, sizeof(line));
        if (Glob(pattern, line)) { n++; }
    }
    return n;
}

static unsigned CountTerminal(const char *pattern)
{
    unsigned i;
    unsigned n = 0U;

    Collect();
    for (i = 0U; i < keptCount; i++)
    {
        if (Glob(pattern, kept[i])) { n++; }
    }
    return n;
}

static void DumpTerminal(void)
{
    unsigned i;

    Collect();
    for (i = 0U; i < keptCount; i++) { printf("    [%u] %s\n", i, kept[i]); }
}

/* The captured terminal lines must be exactly `n` lines matching the glob
 * patterns, in order. */
static void ExpectLines(int lineNo, unsigned n, ...)
{
    va_list args;
    unsigned i;

    Collect();
    checks++;
    if (keptCount != n)
    {
        failures++;
        printf("FAIL line %d: expected %u terminal lines, got %u\n", lineNo, n, keptCount);
        DumpTerminal();
    }

    va_start(args, n);
    for (i = 0U; i < n; i++)
    {
        const char *pattern = va_arg(args, const char *);

        checks++;
        if ((i >= keptCount) || !Glob(pattern, kept[i]))
        {
            failures++;
            printf("FAIL line %d: line %u expected '%s' got '%s'\n",
                   lineNo, i, pattern, (i < keptCount) ? kept[i] : "<none>");
        }
    }
    va_end(args);
}

#define EXPECT(n, ...) ExpectLines(__LINE__, (n), __VA_ARGS__)
#define EXPECT_NONE()  ExpectLines(__LINE__, 0U)

/* --- firmware-role emulation ---------------------------------------- */

static void DrainRx(void)
{
    uint8_t byte;

    while (osMessageQueueGet(CommRxQueueHandle, &byte, NULL, 0U) == osOK)
    {
        AppComm_ProcessByte(byte);
    }
}

/* Bytes arrive from the UART ISR and CommTask parses them immediately. */
static void RxNow(const char *text)
{
    size_t i;
    size_t n = strlen(text);

    for (i = 0U; i < n; i++)
    {
        uint8_t byte = (uint8_t)text[i];
        (void)osMessageQueuePut(CommRxQueueHandle, &byte, 0U, 0U);
    }
    DrainRx();
}

/* Body of StartMotionTask()'s loop in main.c. */
static void MotionStep(void)
{
    MotionCommand command;

    if (AppComm_TakeMotionCommand(&command))
    {
        AppMotion_HandleCommand(&command);
    }

    AppMotion_Update(0.01f);
}

static void CommStep(void)
{
    DrainRx();
    AppComm_ServiceTelemetry();
}

static void Cycle(void)
{
    Fake_SetTickCount(Fake_GetTickCount() + 10U);
    MotionStep();
    CommStep();
}

static void Cycles(unsigned n)
{
    unsigned i;

    for (i = 0U; i < n; i++) { Cycle(); }
}

/* Cycles until a terminal line matching `pattern` is captured; returns the
 * ms from now until it appeared, or -1 if it never did. */
static long RunUntil(const char *pattern, unsigned maxCycles)
{
    uint32_t start = Fake_GetTickCount();
    unsigned i;

    for (i = 0U; i < maxCycles; i++)
    {
        if (CountTerminal(pattern) > 0U)
        {
            return (long)(Fake_GetTickCount() - start);
        }
        Cycle();
    }
    return (CountTerminal(pattern) > 0U) ? (long)(Fake_GetTickCount() - start) : -1L;
}

static AppMotionEvent LastEvent(void)
{
    AppMotionEvent event;
    AppMotion_GetEvent(&event);
    return event;
}

static void Fresh(void)
{
    FakeOs_ClearFaults();
    FakeOs_SetFaultQueue(MotionQueueHandle);
    (void)osMessageQueueReset(MotionQueueHandle);
    (void)osMessageQueueReset(CommRxQueueHandle);

    Fake_SetTickCount(1000U);
    Fake_SetEncoderLeft(0);
    Fake_SetEncoderRight(0);
    Fake_SetHeading(0.0f);
    Fake_SetGyroZ(0.0f);
    Fake_SetImuReady(true);
    Fake_SetImuEvent(APP_IMU_EVENT_NONE, 0U);

    AppMotion_Init();
    AppComm_Init();
    Fake_ResetMotorCallCounts();
    Comm_TxClear();
}

static void EncodersAt(int32_t counts)
{
    Fake_SetEncoderLeft(counts);
    Fake_SetEncoderRight(counts);
}

/* Starts MOV,<dir>,50 over the wire and runs one cycle so it is executing
 * and MM,EV:START has been transmitted. */
static void StartMove50(char dir)
{
    char line[16];

    snprintf(line, sizeof(line), "MOV,%c,50\n", dir);
    RxNow(line);
    Cycle();
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
}

/* Exactly one new line, ERR,BUSY. */
static void ExpectBusy(const char *command)
{
    Comm_TxClear();
    RxNow(command);
    if ((Comm_TxLineCount() != 1U) || (CountRaw("ERR,BUSY") != 1U))
    {
        printf("    not busy: %s\n", command);
    }
    CHECK(Comm_TxLineCount() == 1U);
    CHECK(CountRaw("ERR,BUSY") == 1U);
}

/* At least one reply line and no ERR,*. */
static void ExpectAnswered(const char *command)
{
    Comm_TxClear();
    RxNow(command);
    if ((Comm_TxLineCount() == 0U) || (CountRaw("ERR,*") != 0U))
    {
        printf("    not answered: %s\n", command);
    }
    CHECK(Comm_TxLineCount() >= 1U);
    CHECK(CountRaw("ERR,*") == 0U);
}

static void PutStaleMotionCommands(unsigned n)
{
    MotionCommand stale = { MOTION_FORWARD, 50 };
    unsigned i;

    for (i = 0U; i < n; i++)
    {
        CHECK(osMessageQueuePut(MotionQueueHandle, &stale, 0U, 0U) == osOK);
    }
}

int main(void)
{
    MotionQueueHandle = osMessageQueueNew(8U, sizeof(MotionCommand), NULL);
    CommRxQueueHandle = osMessageQueueNew(128U, 1U, NULL);
    CHECK(MotionQueueHandle != NULL);
    CHECK(CommRxQueueHandle != NULL);

    /* ================================================================
     * BASELINE (no STOP anywhere): the unchanged MOV path through the
     * new AppComm_TakeMotionCommand() still pairs START/END, replies ACK
     * and frees the gate; ERR,BUSY still rejects overlapping commands.
     * ================================================================ */
    Fresh();
    StartMove50('F');
    EXPECT(1, "MM,EV:START,D:F,T:0,V:50");

    Comm_TxClear();
    RxNow("MOV,F,10\n");
    EXPECT(1, "ERR,BUSY");

    Comm_TxClear();
    EncodersAt(MOV_50CM_TARGET_COUNTS);
    CHECK(RunUntil("ACK", 40U) >= 0);
    EXPECT(2, "MM,EV:END,D:F,T:*,RESULT:OK", "ACK");
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);

    Comm_TxClear();
    RxNow("MOV,B,10\n");              /* gate reopened after the event */
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 1U);
    EXPECT_NONE();
    Cycle();
    EXPECT(1, "MM,EV:START,D:B,T:0,V:10");

    /* Normal queue behaviour is untouched when no STOP is involved. */
    Fresh();
    {
        MotionCommand command;

        CHECK(!AppComm_TakeMotionCommand(&command));
        RxNow("MOV,F,5\n");
        CHECK(AppComm_TakeMotionCommand(&command));
        CHECK(command.type == MOTION_FORWARD);
        CHECK(command.value == 5);
        CHECK(!AppComm_TakeMotionCommand(&command));
    }

    /* ================================================================
     * 1. Idle STOP: accepted, ACK,STOPPED only, no brake, servo centred,
     *    nothing sent before MotionTask has actually run.
     * ================================================================ */
    Fresh();
    {
        MotionCommand servo = { MOTION_SERVO_SET, 1800 };
        uint32_t seq0;

        AppMotion_HandleCommand(&servo);
        CHECK(AppServo_GetUs() == 1800U);
        Cycle();                      /* flush the SERVO,SET reply */
        Comm_TxClear();
        Fake_ResetMotorCallCounts();
        seq0 = LastEvent().sequence;

        RxNow("STOP\n");
        EXPECT_NONE();                /* ACK,STOPPED is never sent from parsing */
        CHECK(osMessageQueueGetCount(MotionQueueHandle) == 0U);

        Cycle();
        EXPECT(1, "ACK,STOPPED");
        CHECK(LastEvent().sequence == seq0 + 1U);
        CHECK(LastEvent().type == APP_MOTION_EVENT_STOPPED);
        CHECK(LastEvent().command == MOTION_STOP);
        CHECK(Fake_GetMotorBrakeCallCount() == 0U);
        CHECK(Fake_GetMotorStopCallCount() >= 1U);
        CHECK(AppServo_GetUs() == 1530U);
        CHECK(AppMotion_GetState() == APP_MOTION_IDLE);

        Cycles(5);                    /* nothing further, ever */
        EXPECT(1, "ACK,STOPPED");

        Comm_TxClear();
        RxNow("MOV,F,5\n");           /* gate is open again */
        EXPECT_NONE();
        CHECK(osMessageQueueGetCount(MotionQueueHandle) == 1U);
    }

    /* 2. Two sequential idle STOPs are two operations, two replies. */
    Fresh();
    RxNow("STOP\n");
    Cycle();
    RxNow("STOP\n");
    Cycle();
    EXPECT(2, "ACK,STOPPED", "ACK,STOPPED");
    Cycles(3);
    EXPECT(2, "ACK,STOPPED", "ACK,STOPPED");

    /* ================================================================
     * 3. Active forward and backward STOP.
     * ================================================================ */
    {
        const char dirs[2] = { 'F', 'B' };
        unsigned d;

        for (d = 0U; d < 2U; d++)
        {
            int32_t target = (dirs[d] == 'F') ?
                MOV_50CM_TARGET_COUNTS : -MOV_50CM_TARGET_COUNTS;
            uint32_t sent;

            Fresh();
            StartMove50(dirs[d]);
            Cycle();
            EXPECT(1, "MM,EV:START,D:?,T:0,V:50");
            Fake_ResetMotorCallCounts();

            sent = Fake_GetTickCount();
            RxNow("STOP\n");          /* accepted while busy: never ERR,BUSY */
            EXPECT(1, "MM,EV:START,D:?,T:0,V:50");

            Cycle();                  /* STOP handled: brake begins */
            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            CHECK(Fake_GetMotorBrakeCallCount() == 1U);
            CHECK(AppServo_GetUs() == 1530U);

            /* Past the original target while braking: still no result. */
            EncodersAt(target);
            Cycles(10);
            CHECK(Fake_GetTickCount() == sent + 110U);
            EXPECT(1, "MM,EV:START,D:?,T:0,V:50");

            /* While the stop is outstanding new work is refused. */
            ExpectBusy("MOV,F,10\n");
            ExpectBusy("PWM,50\n");
            ExpectBusy("IMU,ZERO\n");
            CHECK(osMessageQueueGetCount(MotionQueueHandle) == 0U);

            Comm_TxClear();
            Cycle();                  /* sent + 120: one tick short */
            EXPECT_NONE();
            Cycle();                  /* sent + 130 == handled + 120 */
            EXPECT(2, "MM,EV:END,D:?,T:*,RESULT:STOPPED", "ACK,STOPPED");
            CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
            CHECK(Fake_GetMotorBrakeCallCount() == 1U);

            Cycles(20);               /* nothing late */
            EXPECT(2, "MM,EV:END,D:?,T:*,RESULT:STOPPED", "ACK,STOPPED");
        }
    }

    /* ================================================================
     * 4. Queued-but-not-started movement is cancelled; the STOP is not
     *    refused as busy; the cancelled command never runs.
     * ================================================================ */
    Fresh();
    RxNow("MOV,F,50\n");              /* reserved + queued, MotionTask idle */
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 1U);
    Fake_ResetMotorCallCounts();
    RxNow("STOP\n");
    EXPECT_NONE();                    /* no ERR,BUSY */
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 0U);

    RxNow("MOV,B,10\n");              /* nothing new may enter meanwhile */
    EXPECT(1, "ERR,BUSY");
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 0U);

    Comm_TxClear();
    Cycle();
    EXPECT(1, "ACK,STOPPED");         /* no START, no END, no ACK */
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK(Fake_GetMotorSetCallCount() == 0U);
    CHECK(LastEvent().command == MOTION_STOP);
    Cycles(5);
    EXPECT(1, "ACK,STOPPED");
    CHECK(CountRaw("MM,EV:*") == 0U);

    /* 4b. Several stale entries (direct queue puts) are all discarded. */
    Fresh();
    PutStaleMotionCommands(5U);
    RxNow("STOP\n");
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 0U);
    Cycles(4);
    EXPECT(1, "ACK,STOPPED");
    CHECK(Fake_GetMotorSetCallCount() == 0U);

    /* ================================================================
     * 5. The priority latch is consumed exactly once.
     * ================================================================ */
    Fresh();
    {
        MotionCommand command;

        RxNow("STOP\n");
        CHECK(AppComm_TakeMotionCommand(&command));
        CHECK(command.type == MOTION_STOP);
        CHECK(command.value == 0);
        CHECK(!AppComm_TakeMotionCommand(&command));
        CHECK(!AppComm_TakeMotionCommand(&command));
    }

    /* ================================================================
     * 6. Duplicate STOP coalescing; the original brake deadline holds.
     * ================================================================ */
    Fresh();
    StartMove50('F');
    Cycle();
    Fake_ResetMotorCallCounts();
    {
        uint32_t handled = Fake_GetTickCount() + 10U;

        RxNow("STOP\n");
        RxNow("STOP\n");              /* second one, before MotionTask ran */
        Cycle();
        CHECK(Fake_GetTickCount() == handled);
        CHECK(Fake_GetMotorBrakeCallCount() == 1U);

        Cycles(6);                    /* handled + 60 */
        RxNow("STOP\n");              /* third, mid-brake */
        Cycles(5);                    /* handled + 110 */
        CHECK(Fake_GetTickCount() == handled + 110U);
        CHECK(CountTerminal("ACK,STOPPED") == 0U);
        Cycle();                      /* handled + 120: original deadline */
        CHECK(Fake_GetTickCount() == handled + BRAKE_MS);
        CHECK(CountTerminal("ACK,STOPPED") == 1U);
        CHECK(Fake_GetMotorBrakeCallCount() == 1U);

        Cycles(20);
        CHECK(CountTerminal("ACK,STOPPED") == 1U);
        CHECK(CountTerminal("MM,EV:END,*") == 1U);
        EXPECT(3, "MM,EV:START,D:F,T:0,V:50",
                  "MM,EV:END,D:F,T:*,RESULT:STOPPED", "ACK,STOPPED");

        /* A STOP after the reply is a new operation (idle STOP). */
        RxNow("STOP\n");
        Cycle();
        CHECK(CountTerminal("ACK,STOPPED") == 2U);
    }

    /* 6b. A duplicate STOP that arrives after the first STOPPED event was
     *     published but before CommTask polled it must not produce a
     *     second operation / second reply. */
    Fresh();
    RxNow("STOP\n");
    Fake_SetTickCount(Fake_GetTickCount() + 10U);
    MotionStep();                         /* STOPPED published, not yet polled */
    CHECK(LastEvent().type == APP_MOTION_EVENT_STOPPED);
    RxNow("STOP\n");                      /* duplicate: coalesced */
    EXPECT_NONE();
    Cycle();
    EXPECT(1, "ACK,STOPPED");
    Cycles(5);
    EXPECT(1, "ACK,STOPPED");
    CHECK(LastEvent().sequence == 1U);    /* only one STOPPED was ever published */

    /* ================================================================
     * 7/8/9. STOP while a normal-completion / timeout / IMU-not-ready
     *        brake is pending: the pending result is replaced, the
     *        original deadline is kept.
     * ================================================================ */
    {
        unsigned kind;

        for (kind = 0U; kind < 3U; kind++)
        {
            uint32_t brakeStart;

            Fresh();
            StartMove50('F');

            if (kind == 0U)           /* MOV_DONE pending */
            {
                EncodersAt(MOV_50CM_TARGET_COUNTS);
            }
            else if (kind == 1U)      /* MOTION_TIMEOUT pending */
            {
                Fake_SetTickCount(Fake_GetTickCount() + 60000U);
            }
            else                      /* IMU_NOT_READY pending */
            {
                Fake_SetImuReady(false);
            }

            Cycle();
            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            brakeStart = Fake_GetTickCount();
            Cycles(6);                /* brakeStart + 60 */
            RxNow("STOP\n");
            Cycles(5);                /* brakeStart + 110: STOP handled at +70 */
            CHECK(CountTerminal("ACK*") == 0U);
            CHECK(CountTerminal("ERR,*") == 0U);
            Cycle();                  /* brakeStart + 120 */
            CHECK(Fake_GetTickCount() == brakeStart + BRAKE_MS);
            EXPECT(3, "MM,EV:START,D:F,T:0,V:50",
                      "MM,EV:END,D:F,T:*,RESULT:STOPPED", "ACK,STOPPED");
            Cycles(10);
            EXPECT(3, "MM,EV:START,D:F,T:0,V:50",
                      "MM,EV:END,D:F,T:*,RESULT:STOPPED", "ACK,STOPPED");
            CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
        }
    }

    /* 10. STOP wins when the brake deadline has already passed but the
     *     next Update has not run (HandleCommand precedes Update). */
    Fresh();
    StartMove50('F');
    EncodersAt(MOV_50CM_TARGET_COUNTS);
    Cycle();
    CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
    Fake_SetTickCount(Fake_GetTickCount() + BRAKE_MS + 50U);
    RxNow("STOP\n");
    Cycle();
    EXPECT(3, "MM,EV:START,D:F,T:0,V:50",
              "MM,EV:END,D:F,T:*,RESULT:STOPPED", "ACK,STOPPED");
    CHECK(CountTerminal("ACK") == 0U);

    /* 10b. STOP queued in the same loop in which the natural completion
     *      would have started: the STOP is handled first. */
    Fresh();
    StartMove50('F');
    EncodersAt(MOV_50CM_TARGET_COUNTS);
    RxNow("STOP\n");
    Cycle();
    CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
    CHECK(RunUntil("ACK,STOPPED", 30U) >= 0);
    EXPECT(3, "MM,EV:START,D:F,T:0,V:50",
              "MM,EV:END,D:F,T:*,RESULT:STOPPED", "ACK,STOPPED");
    CHECK(CountTerminal("ACK") == 0U);

    /* ================================================================
     * 11. PRE_STEER and TURN, all four directions.
     * ================================================================ */
    {
        const char *const dirs[4] = { "FL", "FR", "BL", "BR" };
        unsigned d;

        for (d = 0U; d < 4U; d++)
        {
            char command[24];
            char startPattern[48];
            int enteredTurn = 0;
            unsigned i;

            snprintf(command, sizeof(command), "MOV,%s,90\n", dirs[d]);
            snprintf(startPattern, sizeof(startPattern),
                     "MM,EV:START,D:%s,T:0,V:90", dirs[d]);

            /* --- PRE_STEER --- */
            Fresh();
            RxNow(command);
            Cycle();
            CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);
            Cycle();
            CHECK(AppMotion_GetState() == APP_MOTION_PRE_STEER);
            Fake_ResetMotorCallCounts();
            RxNow("STOP\n");
            Cycle();
            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            CHECK(AppServo_GetUs() == 1530U);
            CHECK(Fake_GetMotorBrakeCallCount() == 1U);

            for (i = 0U; i < 30U; i++)           /* past PRESTEER + brake */
            {
                Cycle();
                if (AppMotion_GetState() == APP_MOTION_TURN) { enteredTurn = 1; }
            }
            CHECK(!enteredTurn);
            EXPECT(3, startPattern,
                      "MM,EV:END,D:?*,T:*,RESULT:STOPPED", "ACK,STOPPED");
            CHECK(AppServo_GetUs() == 1530U);

            /* --- TURN (pre-steer disabled so it is turning at once) --- */
            Fresh();
            AppMotion_SetPreSteer(0.0f, false);
            RxNow(command);
            Cycle();
            CHECK(AppMotion_GetState() == APP_MOTION_TURN);
            Cycle();
            Fake_SetHeading(30.0f);
            Cycle();
            RxNow("STOP\n");
            Cycle();
            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);

            Fake_SetHeading(90.0f);               /* would satisfy completion */
            Fake_SetGyroZ(0.0f);
            for (i = 0U; i < 30U; i++) { Cycle(); }
            EXPECT(3, startPattern,
                      "MM,EV:END,D:?*,T:*,RESULT:STOPPED", "ACK,STOPPED");
            CHECK(CountRaw("TB,*") == 0U);        /* no TURNBRAKE snapshot */
            CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
        }
    }

    /* ================================================================
     * 12. IMU,ZERO / IMU,FLIP wait states.
     * ================================================================ */
    {
        const char *const requests[2] = { "IMU,ZERO\n", "IMU,FLIP\n" };
        MotionType types[2] = { MOTION_IMU_ZERO, MOTION_IMU_FLIP };
        unsigned i;

        for (i = 0U; i < 2U; i++)
        {
            uint32_t cancels;

            Fresh();
            RxNow(requests[i]);
            Cycle();
            CHECK(AppMotion_GetState() == APP_MOTION_WAIT_IMU);
            cancels = Fake_GetImuCancelCallCount();
            Fake_ResetMotorCallCounts();

            RxNow("STOP\n");
            Cycle();                              /* immediate: no brake */
            CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
            CHECK(Fake_GetImuCancelCallCount() == cancels + 1U);
            CHECK(Fake_GetMotorBrakeCallCount() == 0U);
            CHECK(LastEvent().type == APP_MOTION_EVENT_STOPPED);
            CHECK(LastEvent().command == types[i]);
            EXPECT(1, "ACK,STOPPED");

            AppMotion_Update(6.0f);               /* beyond the IMU timeout */
            Fake_SetImuEvent((i == 0U) ? APP_IMU_EVENT_ZERO_DONE : APP_IMU_EVENT_FLIP_DONE, 7U);
            Cycles(10);
            EXPECT(1, "ACK,STOPPED");             /* no timeout / OK / error */
        }
    }

    /* ================================================================
     * 13. Tuning and diagnostics: no completion line, one ACK,STOPPED.
     * ================================================================ */
    {
        const char *const commands[] = {
            "TUNE,L,1000\n", "TUNE,R,1000\n", "TUNE,B,1000\n",
            "TUNE,S,F,3\n", "TUNE,S,B,3\n",
            "PWM,50\n", "SPD,50\n", "HDG,F,3\n", "HDG,B,3\n", "HDG,RAW,3\n"
        };
        unsigned i;

        for (i = 0U; i < (sizeof(commands) / sizeof(commands[0])); i++)
        {
            Fresh();
            RxNow(commands[i]);
            Cycle();
            if (AppMotion_GetState() == APP_MOTION_IDLE)
            {
                printf("    did not start: %s", commands[i]);
            }
            CHECK(AppMotion_GetState() != APP_MOTION_IDLE);
            Cycle();
            Fake_ResetMotorCallCounts();
            RxNow("STOP\n");
            Cycle();
            CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            CHECK(Fake_GetMotorBrakeCallCount() == 1U);
            Fake_SetTickCount(Fake_GetTickCount() + 0U);
            CHECK(RunUntil("ACK,STOPPED", 30U) >= 0);

            /* Run far beyond every tune / diagnostic duration. */
            Fake_SetTickCount(Fake_GetTickCount() + 120000U);
            Cycles(5);
            if (CountTerminal("*") != 1U) { printf("    lines after: %s", commands[i]); DumpTerminal(); }
            EXPECT(1, "ACK,STOPPED");
        }
    }

    /* ================================================================
     * 14. Mailbox race and pre-STOP snapshot flush.
     * ================================================================ */

    /* (a) A normal completion published but not yet read, then STOP. */
    Fresh();
    StartMove50('F');
    EncodersAt(MOV_50CM_TARGET_COUNTS);
    Cycle();
    CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
    Fake_SetTickCount(Fake_GetTickCount() + BRAKE_MS);
    MotionStep();                         /* MOV_DONE published, CommTask hasn't polled */
    CHECK(LastEvent().type == APP_MOTION_EVENT_MOV_DONE);
    EXPECT(1, "MM,EV:START,D:F,T:0,V:50");
    RxNow("STOP\n");                      /* snapshot flush: END + ACK, in order */
    EXPECT(3, "MM,EV:START,D:F,T:0,V:50", "MM,EV:END,D:F,T:*,RESULT:OK", "ACK");
    Cycle();                              /* now the STOP itself */
    EXPECT(4, "MM,EV:START,D:F,T:0,V:50", "MM,EV:END,D:F,T:*,RESULT:OK",
              "ACK", "ACK,STOPPED");     /* no END for STOPPED: none open */
    Cycles(10);
    CHECK(CountTerminal("ACK,STOPPED") == 1U);
    CHECK(CountTerminal("ACK") == 1U);
    CHECK(CountTerminal("MM,EV:END,*") == 1U);

    /* (b) A movement that began but whose START was not yet transmitted. */
    Fresh();
    RxNow("MOV,F,50\n");
    Fake_SetTickCount(Fake_GetTickCount() + 10U);
    MotionStep();                         /* executing; no CommTask pass yet */
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    EXPECT_NONE();
    RxNow("STOP\n");                      /* flush sends the pending START */
    EXPECT(1, "MM,EV:START,D:F,T:0,V:50");
    CHECK(RunUntil("ACK,STOPPED", 30U) >= 0);
    EXPECT(3, "MM,EV:START,D:F,T:0,V:50",
              "MM,EV:END,D:F,T:*,RESULT:STOPPED", "ACK,STOPPED");

    /* (c) An unread tune completion is still delivered before the STOP
     *     reply. */
    Fresh();
    RxNow("TUNE,B,1000\n");
    Cycle();
    CHECK(AppMotion_GetState() == APP_MOTION_TUNE_BOTH);
    EncodersAt(1000);
    {
        unsigned i;

        for (i = 0U; (i < 60U) && (AppMotion_GetState() != APP_MOTION_IDLE); i++)
        {
            Fake_SetTickCount(Fake_GetTickCount() + 10U);
            MotionStep();                 /* completes without CommTask polling */
        }
    }
    if (AppMotion_GetState() == APP_MOTION_IDLE)
    {
        EXPECT_NONE();
        RxNow("STOP\n");
        EXPECT(1, "TUNE,B,*");
        Cycle();
        EXPECT(2, "TUNE,B,*", "ACK,STOPPED");
    }
    else
    {
        printf("    tune did not complete in the harness\n");
        CHECK(0);
    }

    /* (d) No END without START: a STOP that interrupts nothing. Covered
     *     above (idle STOP, queued-not-started): no MM,EV: line at all. */

    /* ================================================================
     * 15. Malformed STOP during movement: ERR,BAD_CMD, movement goes on.
     * ================================================================ */
    Fresh();
    StartMove50('F');
    Cycle();
    Comm_TxClear();
    RxNow("STOP,1\n");
    RxNow("stop\n");
    RxNow("STOP \n");
    EXPECT(3, "ERR,BAD_CMD", "ERR,BAD_CMD", "ERR,BAD_CMD");
    Cycles(3);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    CHECK(Fake_GetMotorBrakeCallCount() == 0U);
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 0U);
    EncodersAt(MOV_50CM_TARGET_COUNTS);
    CHECK(RunUntil("ACK", 40U) >= 0);
    EXPECT(5, "ERR,BAD_CMD", "ERR,BAD_CMD", "ERR,BAD_CMD",
              "MM,EV:END,D:F,T:*,RESULT:OK", "ACK");
    CHECK(CountTerminal("ACK,STOPPED") == 0U);

    /* ================================================================
     * 16. New movement after STOP (straight and turn).
     * ================================================================ */
    Fresh();
    StartMove50('F');
    Cycle();
    RxNow("STOP\n");
    CHECK(RunUntil("ACK,STOPPED", 30U) >= 0);
    {
        AppMotionTelemetry before;
        AppMotionTelemetry after;

        AppMotion_GetTelemetry(&before);
        Comm_TxClear();
        EncodersAt(0);
        RxNow("MOV,F,50\n");
        EXPECT_NONE();                    /* admitted, not BUSY */
        Cycle();
        EXPECT(1, "MM,EV:START,D:F,T:0,V:50");
        AppMotion_GetTelemetry(&after);
        CHECK(after.runSequence != before.runSequence);

        EncodersAt(MOV_50CM_TARGET_COUNTS);
        CHECK(RunUntil("ACK", 40U) >= 0);
        EXPECT(3, "MM,EV:START,D:F,T:0,V:50",
                  "MM,EV:END,D:F,T:*,RESULT:OK", "ACK");

        /* A fresh turn after a STOP completes normally too. */
        Comm_TxClear();
        EncodersAt(0);
        RxNow("MOV,FL,90\n");
        Cycle();
        EXPECT(1, "MM,EV:START,D:FL,T:0,V:90");
        Fake_SetHeading(90.0f);
        Fake_SetGyroZ(0.0f);
        CHECK(RunUntil("ACK", 80U) >= 0);
        EXPECT(3, "MM,EV:START,D:FL,T:0,V:90",
                  "MM,EV:END,D:FL,T:*,RESULT:OK", "ACK");
    }

    /* ================================================================
     * 17. Reservation table while a STOP is outstanding.
     * ================================================================ */
    Fresh();
    StartMove50('F');
    Cycle();
    RxNow("STOP\n");                      /* latch pending, MotionTask not run */
    Comm_TxClear();
    {
        unsigned pass;

        for (pass = 0U; pass < 2U; pass++)
        {
            /* pass 0: latch pending.  pass 1: STOP handled, braking. */
            ExpectBusy("MOV,F,5\n");
            ExpectBusy("MOV,FL,10\n");
            ExpectBusy("PWM,50\n");
            ExpectBusy("SPD,50\n");
            ExpectBusy("HDG,F,2\n");
            ExpectBusy("HDG,RAW,2\n");
            ExpectBusy("TUNE,L,100\n");
            ExpectBusy("TUNE,S,F,2\n");
            ExpectBusy("ENC,RST\n");
            ExpectBusy("SERVO,1500\n");
            ExpectBusy("IMU,ZERO\n");
            ExpectBusy("IMU,FLIP\n");
            ExpectBusy("WSYNC,0.65,15,1\n");
            ExpectBusy("HCTRL,3,60,1\n");
            ExpectBusy("RTGUARD,0\n");
            ExpectBusy("DPROF,1100,60,0\n");
            ExpectBusy("SPCTRL,4000,0.02,0.03,1\n");
            ExpectBusy("SPPROF,4500,1200,1\n");
            ExpectBusy("STOPCTRL,150,1700,1\n");
            ExpectBusy("MOVCAL,75,100,1\n");
            ExpectBusy("PROFILE,PRECISION\n");
            ExpectBusy("PROFILE,STANDARD\n");
            ExpectBusy("PROFILE,TASK2TEST\n");
            ExpectBusy("SFF,F,3200,4500,-12,1\n");
            ExpectBusy("PRESTEER,150,1\n");
            ExpectBusy("TURNSPEED,1700\n");
            ExpectBusy("ACKERMANN,0.27,1\n");
            ExpectBusy("TURNBRAKE,135,1\n");
            ExpectBusy("PID,L,0.1,0,0.02\n");
            ExpectBusy("PID,S,45,0,0\n");

            /* Read-only commands stay available. */
            ExpectAnswered("WSYNC,GET\n");
            ExpectAnswered("HCTRL,GET\n");
            ExpectAnswered("RTGUARD,GET\n");
            ExpectAnswered("DPROF,GET\n");
            ExpectAnswered("SPCTRL,GET\n");
            ExpectAnswered("SPPROF,GET\n");
            ExpectAnswered("STOPCTRL,GET\n");
            ExpectAnswered("MOVCAL,GET\n");
            ExpectAnswered("PROFILE,GET\n");
            ExpectAnswered("SFF,GET\n");
            ExpectAnswered("PRESTEER,GET\n");
            ExpectAnswered("TURNSPEED,GET\n");
            ExpectAnswered("ACKERMANN,GET\n");
            ExpectAnswered("TURNBRAKE,GET\n");
            ExpectAnswered("PID,GET\n");
            ExpectAnswered("ENC,GET\n");
            ExpectAnswered("SERVO,GET\n");
            ExpectAnswered("IMU,GET\n");

            /* Vision lines keep their existing (silent) behaviour. */
            Comm_TxClear();
            RxNow("TGT,1,0.95,100,200\n");
            RxNow("NUL\n");
            RxNow("RDY\n");
            CHECK(Comm_TxLineCount() == 0U);

            /* Malformed -> BAD_CMD; duplicate STOP -> coalesced, silent. */
            Comm_TxClear();
            RxNow("FOO\n");
            CHECK(CountRaw("ERR,BAD_CMD") == 1U);
            Comm_TxClear();
            RxNow("STOP\n");
            CHECK(Comm_TxLineCount() == 0U);

            if (pass == 0U)
            {
                Cycle();                  /* STOP handled: brake begins */
                CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
            }
        }
    }
    CHECK(RunUntil("ACK,STOPPED", 30U) >= 0);
    CHECK(CountTerminal("ACK,STOPPED") == 1U);   /* duplicates produced nothing */

    /* After the reply every category is admitted again. */
    ExpectAnswered("PID,GET\n");
    Comm_TxClear();
    RxNow("PID,L,0.1,0,0.02\n");
    CHECK(CountRaw("PID,L,OK") == 1U);
    CHECK(CountRaw("ERR,*") == 0U);
    Comm_TxClear();
    RxNow("WSYNC,0.65,15,1\n");
    CHECK(CountRaw("WSYNC,OK") == 1U);
    Comm_TxClear();
    RxNow("MOV,F,5\n");
    EXPECT_NONE();

    /* 17b. PID SET during ordinary movement is unchanged (accepted). */
    Fresh();
    StartMove50('F');
    Cycle();
    Comm_TxClear();
    RxNow("PID,L,0.1,0,0.02\n");
    CHECK(CountRaw("PID,L,OK") == 1U);
    CHECK(CountRaw("ERR,*") == 0U);
    ExpectAnswered("PID,GET\n");

    /* ================================================================
     * 18. Queue-drain faults.
     * ================================================================ */

    /* (a) loud reset failure once: the bounded get loop still empties the
     *     queue; STOP is dispatched first; stale commands never run. */
    Fresh();
    PutStaleMotionCommands(3U);
    FakeOs_SetResetFault(FAKEOS_RESET_ERROR, 1);
    RxNow("STOP\n");
    CHECK(FakeOs_GetResetCallCount() >= 1U);
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 0U);
    Cycle();
    EXPECT(1, "ACK,STOPPED");
    CHECK(Fake_GetMotorSetCallCount() == 0U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    Comm_TxClear();
    RxNow("MOV,F,5\n");                   /* fence already down */
    EXPECT_NONE();

    /* (b) reset reports osOK but leaves the items (false success). */
    Fresh();
    PutStaleMotionCommands(3U);
    FakeOs_SetResetFault(FAKEOS_RESET_SILENT_FAIL, 1);
    RxNow("STOP\n");
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 0U);   /* get loop drained */
    Cycle();
    EXPECT(1, "ACK,STOPPED");
    CHECK(Fake_GetMotorSetCallCount() == 0U);

    /* (c) get failures only: the reset empties the queue. */
    Fresh();
    PutStaleMotionCommands(3U);
    FakeOs_SetGetFault(-1);
    RxNow("STOP\n");
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 0U);
    Cycle();
    EXPECT(1, "ACK,STOPPED");
    FakeOs_ClearFaults();

    /* (d) first attempt fails completely, MotionTask's retry succeeds. */
    Fresh();
    PutStaleMotionCommands(3U);
    FakeOs_SetResetFault(FAKEOS_RESET_SILENT_FAIL, 1);
    FakeOs_SetGetFault(8);                /* one full get loop's worth */
    RxNow("STOP\n");
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 3U);   /* not drained */
    ExpectBusy("MOV,F,5\n");                                  /* fenced */
    Comm_TxClear();
    Cycle();                              /* retry succeeds; STOP first */
    EXPECT(1, "ACK,STOPPED");
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 0U);
    CHECK(Fake_GetMotorSetCallCount() == 0U);
    Comm_TxClear();
    RxNow("MOV,F,5\n");                   /* fence lifted */
    EXPECT_NONE();

    /* (e) persistent failure across many MotionTask loops: STOP is still
     *     dispatched first and answered truthfully; the stale commands
     *     never run; the fence holds and rejects everything; recovery
     *     once the queue works again. */
    Fresh();
    PutStaleMotionCommands(3U);
    FakeOs_SetResetFault(FAKEOS_RESET_SILENT_FAIL, -1);
    FakeOs_SetGetFault(-1);
    RxNow("STOP\n");
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 3U);
    Cycle();                              /* latch consumed: STOP handled */
    EXPECT(1, "ACK,STOPPED");
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    Cycles(10);                           /* stale commands are never read */
    EXPECT(1, "ACK,STOPPED");
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK(Fake_GetMotorSetCallCount() == 0U);
    CHECK(CountRaw("MM,EV:*") == 0U);
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 3U);

    /* stopOutstanding is clear now, only the fence remains: everything
     * that mutates state or starts motion is still refused. */
    ExpectBusy("MOV,F,5\n");
    ExpectBusy("PWM,50\n");
    ExpectBusy("WSYNC,0.65,15,1\n");
    ExpectBusy("PID,L,0.1,0,0.02\n");
    ExpectAnswered("PID,GET\n");
    ExpectAnswered("WSYNC,GET\n");

    /* STOP itself is still accepted and answered while fenced. */
    Comm_TxClear();
    RxNow("STOP\n");
    Cycle();
    EXPECT(1, "ACK,STOPPED");
    Cycles(3);
    EXPECT(1, "ACK,STOPPED");

    /* Recovery: queue works again -> stale entries discarded unrun, fence
     * drops, a fresh MOV runs normally. */
    FakeOs_ClearFaults();
    Comm_TxClear();
    Cycle();
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 0U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK(Fake_GetMotorSetCallCount() == 0U);
    EXPECT_NONE();
    EncodersAt(0);
    RxNow("MOV,F,50\n");
    EXPECT_NONE();                        /* admitted */
    Cycle();
    EXPECT(1, "MM,EV:START,D:F,T:0,V:50");
    EncodersAt(MOV_50CM_TARGET_COUNTS);
    CHECK(RunUntil("ACK", 40U) >= 0);
    EXPECT(3, "MM,EV:START,D:F,T:0,V:50",
              "MM,EV:END,D:F,T:*,RESULT:OK", "ACK");

    /* (f) The fence blocks every normal-queue read while the drain keeps
     *     failing, even if a plain get would succeed: the first 24 gets
     *     fail (on_stop's drain, the STOP loop's drain, the next loop's
     *     drain), so a stale command would be readable on the loop after
     *     that if the fence did not hold. */
    Fresh();
    PutStaleMotionCommands(3U);
    FakeOs_SetResetFault(FAKEOS_RESET_SILENT_FAIL, -1);
    FakeOs_SetGetFault(24);
    RxNow("STOP\n");
    Cycle();                              /* STOP dispatched first */
    EXPECT(1, "ACK,STOPPED");
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 3U);
    Cycle();                              /* fenced: drain fails again */
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK(Fake_GetMotorSetCallCount() == 0U);
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 3U);
    ExpectBusy("MOV,F,5\n");
    Cycle();                              /* gets work now: drained, unrun */
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 0U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK(Fake_GetMotorSetCallCount() == 0U);
    Comm_TxClear();
    RxNow("MOV,F,5\n");                   /* fence lifted */
    EXPECT_NONE();
    FakeOs_ClearFaults();

    /* Reset-error mode persistent but gets work: always drains. */
    Fresh();
    PutStaleMotionCommands(4U);
    FakeOs_SetResetFault(FAKEOS_RESET_ERROR, -1);
    RxNow("STOP\n");
    CHECK(osMessageQueueGetCount(MotionQueueHandle) == 0U);
    Cycle();
    EXPECT(1, "ACK,STOPPED");
    FakeOs_ClearFaults();

    /* ================================================================
     * PROFILE,TASK2TEST over the wire: idle apply -> PROFILE,OK, GET
     * reports TASK2TEST; rejected with ERR,BUSY (and nothing applied)
     * while a movement is running; CUSTOM is not settable.
     * ================================================================ */
    Fresh();
    AppMotion_ApplyProfile(APP_MOTION_PROFILE_PRECISION);
    CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_PRECISION);

    Comm_TxClear();
    RxNow("PROFILE,TASK2TEST\n");
    CHECK(Comm_TxLineCount() == 1U);
    CHECK(CountRaw("PROFILE,OK") == 1U);
    CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_TASK2TEST);

    Comm_TxClear();
    RxNow("PROFILE,GET\n");
    CHECK(Comm_TxLineCount() == 1U);
    CHECK(CountRaw("PROFILE,TASK2TEST") == 1U);

    Comm_TxClear();
    RxNow("PROFILE,CUSTOM\n");
    CHECK(Comm_TxLineCount() == 1U);
    CHECK(CountRaw("ERR,BAD_CMD") == 1U);
    CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_TASK2TEST);

    /* A manual profile-owned SET reports CUSTOM; reapplying restores. */
    RxNow("WSYNC,0.65,15,1\n");
    Comm_TxClear();
    RxNow("PROFILE,GET\n");
    CHECK(CountRaw("PROFILE,CUSTOM") == 1U);
    RxNow("PROFILE,TASK2TEST\n");
    Comm_TxClear();
    RxNow("PROFILE,GET\n");
    CHECK(CountRaw("PROFILE,TASK2TEST") == 1U);

    /* Busy: a running MOV,F rejects the apply; the profile is unchanged. */
    RxNow("PROFILE,PRECISION\n");
    CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_PRECISION);
    StartMove50('F');
    ExpectBusy("PROFILE,TASK2TEST\n");
    CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_PRECISION);
    Comm_TxClear();
    RxNow("PROFILE,GET\n");
    CHECK(CountRaw("PROFILE,PRECISION") == 1U);

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
