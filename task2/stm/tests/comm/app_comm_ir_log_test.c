/* Host test for the IR bench-logging path in the REAL Core/Src/app_comm.c
 * (AppComm_TrySendIrLog, reached from AppComm_ServiceTelemetry in CommTask).
 *
 * One source, four builds (every flag combination that compiles):
 *   1. -DAPP_IR_BENCH_LOG=1 (APP_IR_ENABLE defaults to 1): exact "IRRAW," format,
 *      NA rules, 5 Hz limit, STOP / terminal-reply ordering, worst-case line
 *      length, and that "IRRAW..." is not an incoming command.
 *   2. -DAPP_IR_BENCH_LOG=1 -DAPP_ULTRASONIC_BENCH_LOG=1: both bench lines,
 *      each still <= 5 Hz, never two bench lines in one service pass.
 *   3. no flags (the production default: IR on, logger off): no "IRRAW," line,
 *      snapshot never read.
 *   4. -DAPP_IR_ENABLE=0 (IR compiled out, logger off): no "IRRAW," line.
 * -DAPP_IR_BENCH_LOG=1 -DAPP_IR_ENABLE=0 must NOT compile (checked by the
 * runner, see CODEBASE_INDEX.md section 11).
 *
 * Build (from repo root), mode 1:
 *   gcc -std=c11 -Wall -Wextra -Werror -DAPP_IR_BENCH_LOG=1 \
 *       -Itests/comm/fakes -Itests/motion/fakes -ICore/Inc \
 *       -o tests/comm/app_comm_ir_log_test.exe \
 *       tests/comm/app_comm_ir_log_test.c Core/Src/app_comm.c \
 *       Core/Src/vision_rx.c Core/Src/app_motion.c Core/Src/app_pid.c Core/Src/app_ir_edge.c Core/Src/app_ir_dist.c \
 *       Core/Src/app_us_park.c tests/motion/fakes/hw_stubs.c \
 *       tests/comm/fakes/comm_stubs.c tests/comm/fakes/ultrasonic_stub.c \
 *       tests/comm/fakes/ir_stub.c
 * Modes 2-4: same command with the flags listed above instead (mode 3: drop
 * the -D).
 */

#include "app_comm.h"
#include "app_motion.h"
#include "app_ir.h"
#include "cmsis_os.h"
#include "main.h"
#include "test_fakes.h"
#include "ir_stub.h"
#include "ultrasonic_stub.h"

#include <stdio.h>
#include <string.h>

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

#define CHECK_STR(actual, expected) do { \
    checks++; \
    if (strcmp((actual), (expected)) != 0) { \
        failures++; \
        printf("FAIL line %d:\n   got      '%s'\n   expected '%s'\n", \
               __LINE__, (actual), (expected)); \
    } \
} while (0)

#define MOV_50CM_TARGET_COUNTS 3850

#if !(APP_IR_BENCH_LOG && APP_ULTRASONIC_BENCH_LOG)
static unsigned CountPrefix(const char *prefix)
{
    unsigned total = Comm_TxLineCount();
    unsigned i;
    unsigned n = 0U;
    char line[400];

    for (i = 0U; i < total; i++)
    {
        (void)Comm_TxLine(i, line, sizeof(line));
        if (strncmp(line, prefix, strlen(prefix)) == 0) { n++; }
    }
    return n;
}
#endif

static void DrainRx(void)
{
    uint8_t byte;

    while (osMessageQueueGet(CommRxQueueHandle, &byte, NULL, 0U) == osOK)
    {
        AppComm_ProcessByte(byte);
    }
}

#if APP_IR_BENCH_LOG && !APP_ULTRASONIC_BENCH_LOG
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
#endif

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

static void Fresh(void)
{
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
    Comm_TxClear();
}

#if APP_IR_ENABLE
/* A VALID publication; `keepFresh` re-stamps it every cycle (IRTask keeps
 * publishing at ~50 Hz on the robot). */
static AppIrSnapshot irSnap;
static int keepFresh;

static AppIrSnapshot GoodIr(uint32_t publishTick)
{
    AppIrSnapshot s = {0};

    s.sequence = 42U;
    s.publishTick = publishTick;
    s.lastValidTick = publishTick;
    s.leftRaw = 1820U;
    s.rightRaw = 1764U;
    s.status = (uint8_t)APP_IR_STATUS_VALID;
    s.counters.valid = 42U;
    return s;
}

static void SetIr(const AppIrSnapshot *s, int fresh)
{
    irSnap = *s;
    keepFresh = fresh;
    IrStub_Set(&irSnap);
}
#endif

static void Cycle(void)
{
    Fake_SetTickCount(Fake_GetTickCount() + 10U);
#if APP_IR_ENABLE
    if (keepFresh)
    {
        irSnap.publishTick = Fake_GetTickCount();
        irSnap.lastValidTick = irSnap.publishTick;
        IrStub_Set(&irSnap);
    }
#endif
    MotionStep();
    CommStep();
}

#if APP_IR_BENCH_LOG
#if !APP_ULTRASONIC_BENCH_LOG

static void CommOnlyCycle(void)
{
    Fake_SetTickCount(Fake_GetTickCount() + 10U);
    CommStep();
}

/* Runs `ms` of time and returns the first IRRAW line seen (if any). */
static int FirstIrLine(unsigned ms, char *out, size_t outSize)
{
    unsigned i;
    unsigned total;
    unsigned k;

    for (i = 0U; i < ms / 10U; i++)
    {
        Cycle();
        total = Comm_TxLineCount();
        for (k = 0U; k < total; k++)
        {
            (void)Comm_TxLine(k, out, outSize);
            if (strncmp(out, "IRRAW,", 6) == 0) { return 1; }
        }
    }
    return 0;
}

static void test_format_and_rate(void)
{
    AppIrSnapshot s = GoodIr(1190U);
    char line[400];
    unsigned i;
    unsigned seen = 0U;
    unsigned lines = 0U;
    uint32_t lastSeenTick = 0U;
    uint32_t firstTick = 0U;

    /* Nothing before 200 ms; exactly one line at 200 ms. */
    Fresh();
    SetIr(&s, 0);
    for (i = 0U; i < 19U; i++) { Cycle(); }
    CHECK(Fake_GetTickCount() == 1190U);
    CHECK(CountPrefix("IRRAW,") == 0U);
    Cycle();                                        /* tick 1200 */
    CHECK(CountPrefix("IRRAW,") == 1U);
    (void)Comm_TxLine(0U, line, sizeof(line));
    CHECK_STR(line, "IRRAW,S:42,L:1820,R:1764,F:00");

    /* 5 Hz limit over 2 s: >= 200 ms apart, 10 lines, nothing else sent. */
    Fresh();
    SetIr(&s, 1);
    for (i = 0U; i < 200U; i++)
    {
        Cycle();
        if (CountPrefix("IRRAW,") > seen)
        {
            seen = CountPrefix("IRRAW,");
            if (lines == 0U) { firstTick = Fake_GetTickCount(); }
            else { CHECK((Fake_GetTickCount() - lastSeenTick) >= 200U); }
            lastSeenTick = Fake_GetTickCount();
            lines++;
        }
    }
    CHECK(lines == 10U);
    CHECK(firstTick == 1200U);
    CHECK(Comm_TxLineCount() == 10U);

    /* The rate limit is wrap-safe across the 32-bit tick. */
    Fresh();
    Fake_SetTickCount(0xFFFFFF00U);
    AppComm_Init();
    Comm_TxClear();
    SetIr(&s, 1);
    seen = 0U;
    lines = 0U;
    for (i = 0U; i < 100U; i++)
    {
        Cycle();
        if (CountPrefix("IRRAW,") > seen)
        {
            seen = CountPrefix("IRRAW,");
            if (lines > 0U) { CHECK((uint32_t)(Fake_GetTickCount() - lastSeenTick) >= 200U); }
            lastSeenTick = Fake_GetTickCount();
            lines++;
        }
    }
    CHECK(lines == 5U);
}

static void test_values_and_na(void)
{
    AppIrSnapshot s;
    char line[400];

    /* 0 and 4095 are raw values, printed as-is (no plausibility rule). */
    Fresh();
    s = GoodIr(1190U);
    s.leftRaw = 0U;
    s.rightRaw = 4095U;
    SetIr(&s, 0);
    CHECK(FirstIrLine(400U, line, sizeof(line)));
    CHECK_STR(line, "IRRAW,S:42,L:0,R:4095,F:00");

    /* Never published: NA, sequence 0, STALE. */
    Fresh();
    s = (AppIrSnapshot){0};
    SetIr(&s, 0);
    CHECK(FirstIrLine(400U, line, sizeof(line)));
    CHECK_STR(line, "IRRAW,S:0,L:NA,R:NA,F:40");

    /* Freshness boundary at the 1200 ms line: age 100 printed, 101 NA. */
    Fresh();
    s = GoodIr(1100U);
    SetIr(&s, 0);
    CHECK(FirstIrLine(400U, line, sizeof(line)));
    CHECK_STR(line, "IRRAW,S:42,L:1820,R:1764,F:00");

    Fresh();
    s = GoodIr(1099U);
    SetIr(&s, 0);
    CHECK(FirstIrLine(400U, line, sizeof(line)));
    CHECK_STR(line, "IRRAW,S:42,L:NA,R:NA,F:40");

    /* Failed scan: NA, the transient bit and STALE. Values in the struct
     * are never printed for a non-VALID publication. */
    Fresh();
    s = GoodIr(1190U);
    s.status = (uint8_t)APP_IR_STATUS_ERROR;
    s.faultMask = APP_IR_FAULT_TIMEOUT;
    s.leftRaw = 999U;
    SetIr(&s, 0);
    CHECK(FirstIrLine(400U, line, sizeof(line)));
    CHECK_STR(line, "IRRAW,S:42,L:NA,R:NA,F:44");

    Fresh();
    s.faultMask = APP_IR_FAULT_DMA | APP_IR_FAULT_ADC;
    SetIr(&s, 0);
    CHECK(FirstIrLine(400U, line, sizeof(line)));
    CHECK_STR(line, "IRRAW,S:42,L:NA,R:NA,F:58");

    /* Latched faults: more than two hex digits. */
    Fresh();
    s = (AppIrSnapshot){0};
    s.sequence = 1U;
    s.status = (uint8_t)APP_IR_STATUS_FAULT;
    s.faultMask = APP_IR_FAULT_CFG_ADC;
    SetIr(&s, 0);
    CHECK(FirstIrLine(400U, line, sizeof(line)));
    CHECK_STR(line, "IRRAW,S:1,L:NA,R:NA,F:140");

    Fresh();
    s.faultMask = APP_IR_FAULT_TASK;
    SetIr(&s, 0);
    CHECK(FirstIrLine(400U, line, sizeof(line)));
    CHECK_STR(line, "IRRAW,S:1,L:NA,R:NA,F:C0");

    Fresh();
    s.faultMask = APP_IR_FAULT_TIMEOUT | APP_IR_FAULT_RECOVERY;
    SetIr(&s, 0);
    CHECK(FirstIrLine(400U, line, sizeof(line)));
    CHECK_STR(line, "IRRAW,S:1,L:NA,R:NA,F:844");
}

static void test_worst_case_length(void)
{
    AppIrSnapshot s;
    char line[400];
    size_t longest = 0U;

    Fresh();
    s = GoodIr(1190U);
    s.sequence = UINT32_MAX;
    s.leftRaw = 4095U;
    s.rightRaw = 4095U;
    SetIr(&s, 0);
    CHECK(FirstIrLine(400U, line, sizeof(line)));
    CHECK_STR(line, "IRRAW,S:4294967295,L:4095,R:4095,F:00");
    longest = strlen(line);

    Fresh();
    s.status = (uint8_t)APP_IR_STATUS_FAULT;
    s.faultMask = 0xFFBFU;                  /* every stored bit (STALE added) */
    SetIr(&s, 0);
    CHECK(FirstIrLine(400U, line, sizeof(line)));
    CHECK_STR(line, "IRRAW,S:4294967295,L:NA,R:NA,F:FFFF");
    if (strlen(line) > longest) { longest = strlen(line); }

    CHECK(longest == 37U);                  /* + '\n' = 38 B, buffer is 256 */
    printf("worst-case IRRAW line length: %u bytes (+newline)\n",
           (unsigned)longest);
}

static void test_stop_interaction(void)
{
    AppIrSnapshot s = GoodIr(1000U);
    unsigned total;
    unsigned i;
    unsigned ackIndex = 0U;
    unsigned endIndex = 0U;
    int sawAck = 0;
    int sawEnd = 0;
    char line[400];

    /* STOP outstanding (MotionTask not run yet): no IRRAW at all. */
    Fresh();
    SetIr(&s, 1);
    RxNow("STOP\n");
    Comm_TxClear();
    for (i = 0U; i < 60U; i++) { CommOnlyCycle(); }     /* 600 ms */
    CHECK(CountPrefix("IRRAW,") == 0U);
    CHECK(CountPrefix("ACK,STOPPED") == 0U);

    /* MotionTask runs: STOPPED is reported, then the log resumes. */
    Cycle();
    CHECK(CountPrefix("ACK,STOPPED") == 1U);
    for (i = 0U; i < 40U; i++) { Cycle(); }
    CHECK(CountPrefix("IRRAW,") >= 1U);

    /* STOP mid-movement with an IRRAW line due: MM,EV:END is immediately
     * followed by ACK,STOPPED. */
    Fresh();
    SetIr(&s, 1);
    RxNow("MOV,F,50\n");
    for (i = 0U; i < 17U; i++) { Cycle(); }
    RxNow("STOP\n");
    for (i = 0U; i < 60U; i++) { Cycle(); }

    total = Comm_TxLineCount();
    for (i = 0U; i < total; i++)
    {
        (void)Comm_TxLine(i, line, sizeof(line));
        if (strncmp(line, "MM,EV:END", 9) == 0) { endIndex = i; sawEnd = 1; }
        if (strcmp(line, "ACK,STOPPED") == 0) { ackIndex = i; sawAck = 1; }
    }
    CHECK(sawEnd && sawAck);
    CHECK(ackIndex == (endIndex + 1U));
    CHECK(CountPrefix("ACK,STOPPED") == 1U);
}

/* For every phase of the 200 ms log period relative to the movement, the
 * terminal reply is sent in the FIRST CommTask pass after MotionTask
 * publishes it (never deferred by an IRRAW line), MM,EV:END is immediately
 * followed by it, and IRRAW lines may appear while the movement runs. */
static void RunTerminalTrial(unsigned offset, int useStop,
                             unsigned *deferredLog, unsigned *duringMotion)
{
    AppIrSnapshot s = GoodIr(1000U);
    AppMotionEvent before;
    AppMotionEvent after;
    const char *terminal = useStop ? "ACK,STOPPED" : "ACK";
    char line[400];
    unsigned i;
    unsigned k;
    unsigned total;
    unsigned startIndex = 0U;
    unsigned endIndex = 0U;
    unsigned ackIndex = 0U;
    int sawStart = 0;
    int sawEnd = 0;
    int sawAck = 0;
    int checkNextPass = 0;

    Fresh();
    SetIr(&s, 1);
    for (i = 0U; i < offset; i++) { Cycle(); }
    RxNow("MOV,F,50\n");

    for (i = 0U; i < 400U; i++)
    {
        unsigned linesBefore;

        if (i == 30U)
        {
            if (useStop) { RxNow("STOP\n"); }
            else
            {
                Fake_SetEncoderLeft(MOV_50CM_TARGET_COUNTS);
                Fake_SetEncoderRight(MOV_50CM_TARGET_COUNTS);
            }
        }

        Fake_SetTickCount(Fake_GetTickCount() + 10U);
        irSnap.publishTick = Fake_GetTickCount();
        IrStub_Set(&irSnap);

        AppMotion_GetEvent(&before);
        MotionStep();
        AppMotion_GetEvent(&after);
        checkNextPass = (after.sequence != before.sequence);

        linesBefore = Comm_TxLineCount();
        CommStep();

        if (checkNextPass)
        {
            int found = 0;
            int irBefore = 0;

            total = Comm_TxLineCount();
            for (k = linesBefore; k < total; k++)
            {
                (void)Comm_TxLine(k, line, sizeof(line));
                if (strcmp(line, terminal) == 0) { found = 1; break; }
                if (strncmp(line, "IRRAW,", 6) == 0) { irBefore = 1; }
            }
            CHECK(found);           /* not delayed by a bench line */
            CHECK(!irBefore);

            /* Was an IRRAW line due in that pass? Then it comes next pass. */
            {
                unsigned beforeNext = Comm_TxLineCount();
                Fake_SetTickCount(Fake_GetTickCount() + 10U);
                irSnap.publishTick = Fake_GetTickCount();
                IrStub_Set(&irSnap);
                MotionStep();
                CommStep();
                total = Comm_TxLineCount();
                for (k = beforeNext; k < total; k++)
                {
                    (void)Comm_TxLine(k, line, sizeof(line));
                    if (strncmp(line, "IRRAW,", 6) == 0) { (*deferredLog)++; }
                }
            }
            break;
        }
    }

    total = Comm_TxLineCount();
    for (k = 0U; k < total; k++)
    {
        (void)Comm_TxLine(k, line, sizeof(line));
        if (strncmp(line, "MM,EV:START", 11) == 0) { startIndex = k; sawStart = 1; }
        if (strncmp(line, "MM,EV:END", 9) == 0) { endIndex = k; sawEnd = 1; }
        if (strcmp(line, terminal) == 0) { ackIndex = k; sawAck = 1; }
    }
    CHECK(sawStart && sawEnd && sawAck);
    CHECK(ackIndex == (endIndex + 1U));

    for (k = startIndex + 1U; k < endIndex; k++)
    {
        (void)Comm_TxLine(k, line, sizeof(line));
        if (strncmp(line, "IRRAW,", 6) == 0) { (*duringMotion)++; }
    }
}

static void test_terminal_reply_never_delayed(void)
{
    unsigned offset;
    unsigned deferredStop = 0U;
    unsigned deferredAck = 0U;
    unsigned duringMotion = 0U;

    for (offset = 0U; offset < 20U; offset++)
    {
        RunTerminalTrial(offset, 1, &deferredStop, &duringMotion);
        RunTerminalTrial(offset, 0, &deferredAck, &duringMotion);
    }

    /* The 20 phase offsets cover the case where IRRAW was due in the very
     * pass that carried the terminal reply. */
    CHECK(deferredStop >= 1U);
    CHECK(deferredAck >= 1U);
    CHECK(duringMotion >= 1U);              /* allowed between START and END */
    printf("terminal-reply trials: IRRAW deferred behind ACK,STOPPED %u times, "
           "behind ACK %u times; %u IRRAW lines during motion\n",
           deferredStop, deferredAck, duringMotion);
}

static void test_no_new_incoming_command(void)
{
    char line[400];

    Fresh();
    RxNow("IRRAW\n");
    CHECK(Comm_TxLineCount() == 1U);
    (void)Comm_TxLine(0U, line, sizeof(line));
    CHECK_STR(line, "ERR,BAD_CMD");

    Comm_TxClear();
    RxNow("IRRAW,S:1,L:1,R:1,F:00\n");
    CHECK(Comm_TxLineCount() == 1U);
    (void)Comm_TxLine(0U, line, sizeof(line));
    CHECK_STR(line, "ERR,BAD_CMD");
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
}

#else /* APP_ULTRASONIC_BENCH_LOG: both bench logs at once */

static void test_with_ultrasonic_log(void)
{
    AppIrSnapshot s = GoodIr(1000U);
    AppUltrasonicSnapshot us = {0};
    unsigned i;
    unsigned irLines = 0U;
    unsigned usLines = 0U;
    uint32_t lastIr = 0U;
    uint32_t lastUs = 0U;
    char line[400];

    us.sequence = 3U;
    us.result = APP_US_RESULT_NONE;

    Fresh();
    SetIr(&s, 1);
    UsStub_Set(&us);

    for (i = 0U; i < 300U; i++)                     /* 3 s */
    {
        unsigned before = Comm_TxLineCount();
        unsigned after;
        unsigned k;
        unsigned benchThisPass = 0U;

        Cycle();
        after = Comm_TxLineCount();

        for (k = before; k < after; k++)
        {
            (void)Comm_TxLine(k, line, sizeof(line));
            if (strncmp(line, "IRRAW,", 6) == 0)
            {
                if (irLines > 0U) { CHECK((Fake_GetTickCount() - lastIr) >= 200U); }
                lastIr = Fake_GetTickCount();
                irLines++;
                benchThisPass++;
            }
            else if (strncmp(line, "US,", 3) == 0)
            {
                if (usLines > 0U) { CHECK((Fake_GetTickCount() - lastUs) >= 200U); }
                lastUs = Fake_GetTickCount();
                usLines++;
                benchThisPass++;
            }
        }
        CHECK(benchThisPass <= 1U);
    }

    CHECK(irLines >= 14U && irLines <= 15U);
    CHECK(usLines >= 14U && usLines <= 15U);
    CHECK(Comm_TxLineCount() == (irLines + usLines));
}

#endif /* APP_ULTRASONIC_BENCH_LOG */

#else /* APP_IR_BENCH_LOG == 0 */

static void test_disabled_build_is_silent(void)
{
    unsigned i;

#if APP_IR_ENABLE
    AppIrSnapshot s = GoodIr(1000U);
    SetIr(&s, 1);
#endif

    Fresh();
    for (i = 0U; i < 300U; i++) { Cycle(); }        /* 3 s */

    CHECK(CountPrefix("IRRAW,") == 0U);
    CHECK(Comm_TxLineCount() == 0U);
#if APP_IR_ENABLE
    CHECK(IrStub_GetCallCount() == 0U);             /* never even read */
#endif
}

#endif /* APP_IR_BENCH_LOG */

int main(void)
{
    MotionQueueHandle = osMessageQueueNew(8U, sizeof(MotionCommand), NULL);
    CommRxQueueHandle = osMessageQueueNew(128U, 1U, NULL);
    CHECK(MotionQueueHandle != NULL);
    CHECK(CommRxQueueHandle != NULL);

    printf("mode: APP_IR_ENABLE=%d APP_IR_BENCH_LOG=%d APP_ULTRASONIC_BENCH_LOG=%d\n",
           APP_IR_ENABLE, APP_IR_BENCH_LOG, APP_ULTRASONIC_BENCH_LOG);

#if APP_IR_BENCH_LOG
#if !APP_ULTRASONIC_BENCH_LOG
    test_format_and_rate();
    test_values_and_na();
    test_worst_case_length();
    test_stop_interaction();
    test_terminal_reply_never_delayed();
    test_no_new_incoming_command();
#else
    test_with_ultrasonic_log();
#endif
#else
    test_disabled_build_is_silent();
#endif

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
