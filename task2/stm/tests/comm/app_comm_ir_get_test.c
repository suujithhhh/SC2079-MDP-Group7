/* Host test for the Phase 3 IR,GET reply in the REAL Core/Src/app_comm.c and
 * Core/Src/vision_rx.c (CommTask / MotionTask emulation, like the other
 * tests/comm suites).
 *
 * What is proved:
 *   - the exact reply line for every acquisition / range state, AGE
 *     conversion and cap, the worst-case length;
 *   - the reply is DEFERRED: a request only records that a reply is owed, and
 *     the reply never delays or overtakes
 *       * ACK,STOPPED (nothing is sent while a STOP is outstanding),
 *       * MM,EV:END and its terminal reply (sent back to back),
 *       * an ACK / ERR that is ready in the same service pass,
 *       * an ERR already sent by the parser (ERR,BUSY);
 *   - at most one reply per pass, extra requests beyond 4 owed are discarded
 *     (never answered with an ERR line), no unsolicited IR line ever;
 *   - the IR,GET reply goes out ahead of the IRRAW / US, bench lines;
 *   - with the IR feature compiled out, IR,GET is answered ERR,BAD_CMD;
 *   - IR,GET never changes motion state.
 *
 * One source, four builds (from repo root; sources as in the other comm tests):
 *   1. (no IR flags, or -DAPP_IR_ENABLE=1)    IR,GET only (the production default)
 *   2. -DAPP_IR_BENCH_LOG=1                   IR,GET + IRRAW
 *   3. -DAPP_IR_BENCH_LOG=1 -DAPP_ULTRASONIC_BENCH_LOG=1   all three lines
 *   4. -DAPP_IR_ENABLE=0                      compiled out
 *
 *   gcc -std=c11 -Wall -Wextra -Werror -DAPP_IR_ENABLE=1 \
 *       -Itests/comm/fakes -Itests/motion/fakes -ICore/Inc \
 *       -o tests/comm/app_comm_ir_get_test.exe \
 *       tests/comm/app_comm_ir_get_test.c Core/Src/app_comm.c \
 *       Core/Src/vision_rx.c Core/Src/app_motion.c Core/Src/app_pid.c Core/Src/app_ir_edge.c Core/Src/app_ir_dist.c \
 *       Core/Src/app_us_park.c tests/motion/fakes/hw_stubs.c \
 *       tests/comm/fakes/comm_stubs.c tests/comm/fakes/ultrasonic_stub.c \
 *       tests/comm/fakes/ir_stub.c
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

#if APP_IR_ENABLE
/* First line starting with `prefix`; 0 if none. */
static int FindLine(const char *prefix, char *out, size_t outSize)
{
    unsigned total = Comm_TxLineCount();
    unsigned i;

    for (i = 0U; i < total; i++)
    {
        (void)Comm_TxLine(i, out, outSize);
        if (strncmp(out, prefix, strlen(prefix)) == 0) { return 1; }
    }
    return 0;
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
#if APP_IR_ENABLE
    IrStub_ResetCounts();
#endif
}

#if APP_IR_ENABLE

static AppIrReading reading;
#if APP_IR_BENCH_LOG
static AppIrSnapshot irSnap;
static int keepFresh;
#endif

static AppIrReading GoodReading(void)
{
    AppIrReading r;

    memset(&r, 0, sizeof(r));
    r.sequence = 1532U;
    r.ageTicks = 14U;
    r.faultMask = 0U;
    r.acq = (uint8_t)APP_IR_ACQ_OK;
    r.range[0] = (uint8_t)APP_IR_RANGE_OK;
    r.range[1] = (uint8_t)APP_IR_RANGE_FAR;
    r.distanceMm[0] = 237U;
    return r;
}

static void SetReading(const AppIrReading *r)
{
    reading = *r;
    IrStub_SetReading(&reading);
}

static void Cycle(void)
{
    Fake_SetTickCount(Fake_GetTickCount() + 10U);
#if APP_IR_BENCH_LOG
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

/* Sends IR,GET, runs one pass and returns the single "IR," line (if any). */
static int AskOnce(char *out, size_t outSize)
{
    unsigned before;
    unsigned total;
    unsigned k;
    int found = 0;

    before = Comm_TxLineCount();
    RxNow("IR,GET\n");
    Cycle();
    total = Comm_TxLineCount();
    for (k = before; k < total; k++)
    {
        (void)Comm_TxLine(k, out, outSize);
        if (strncmp(out, "IR,", 3) == 0) { found++; }
    }
    return found;
}

static void ExpectReply(const AppIrReading *r, const char *expected)
{
    char line[400] = "";
    int n;

    Fresh();
    SetReading(r);
    n = AskOnce(line, sizeof(line));
    CHECK(n == 1);
    CHECK_STR(line, expected);
}

/* Every line below is a combination app_ir.c can really produce (the reading
 * states are mutually consistent with their fault mask and age):
 *   OK / WARM   fresh VALID publication  -> age <= 100 ticks, F has no 0x40
 *   ERR         fresh failed scan        -> age <= 100, transient bit + 0x40
 *   STALE       nothing published, or an old publication -> age > 100 or NA, 0x40
 *   FAULT       latched bit(s) + 0x40, whatever the age
 * The synthetic cases further down are marked as such. */
static void test_exact_lines(void)
{
    AppIrReading r = GoodReading();

    /* OK: one distance, one FAR. */
    ExpectReply(&r, "IR,S:1532,ST:OK,L:237,R:FAR,AGE:14,F:00");

    r.range[0] = (uint8_t)APP_IR_RANGE_NEAR;
    r.range[1] = (uint8_t)APP_IR_RANGE_OK;
    r.distanceMm[1] = 400U;
    ExpectReply(&r, "IR,S:1532,ST:OK,L:NEAR,R:400,AGE:14,F:00");

    r.range[0] = (uint8_t)APP_IR_RANGE_OK;
    r.distanceMm[0] = 100U;
    ExpectReply(&r, "IR,S:1532,ST:OK,L:100,R:400,AGE:14,F:00");

    /* Freshness limit: age exactly 100 ms is still OK. */
    r = GoodReading();
    r.ageTicks = 100U;
    ExpectReply(&r, "IR,S:1532,ST:OK,L:237,R:FAR,AGE:100,F:00");

    /* WARM: fresh, filter not full; distances are NA. */
    r = GoodReading();
    r.acq = (uint8_t)APP_IR_ACQ_WARMUP;
    r.ageTicks = 6U;
    r.range[0] = (uint8_t)APP_IR_RANGE_INVALID;
    r.range[1] = (uint8_t)APP_IR_RANGE_INVALID;
    r.distanceMm[0] = 0U;
    ExpectReply(&r, "IR,S:1532,ST:WARM,L:NA,R:NA,AGE:6,F:00");

    /* ERR: a FRESH failed scan (age <= 100) reports ERR... */
    r = GoodReading();
    r.acq = (uint8_t)APP_IR_ACQ_ERROR;
    r.ageTicks = 5U;
    r.faultMask = (uint16_t)(APP_IR_FAULT_TIMEOUT | APP_IR_FAULT_STALE);
    r.range[0] = (uint8_t)APP_IR_RANGE_INVALID;
    r.range[1] = (uint8_t)APP_IR_RANGE_INVALID;
    r.distanceMm[0] = 0U;
    ExpectReply(&r, "IR,S:1532,ST:ERR,L:NA,R:NA,AGE:5,F:44");
    r.ageTicks = 100U;
    ExpectReply(&r, "IR,S:1532,ST:ERR,L:NA,R:NA,AGE:100,F:44");

    /* ...and the same failed publication, once older than 100 ms, is STALE. */
    r.acq = (uint8_t)APP_IR_ACQ_STALE;
    r.ageTicks = 101U;
    ExpectReply(&r, "IR,S:1532,ST:STALE,L:NA,R:NA,AGE:101,F:44");

    /* STALE after an old good publication. */
    r.faultMask = APP_IR_FAULT_STALE;
    ExpectReply(&r, "IR,S:1532,ST:STALE,L:NA,R:NA,AGE:101,F:40");

    /* A latched fault always reads FAULT, young or old. */
    r.acq = (uint8_t)APP_IR_ACQ_FAULT;
    r.ageTicks = 14U;
    r.faultMask = (uint16_t)(APP_IR_FAULT_CFG_ADC | APP_IR_FAULT_STALE);
    ExpectReply(&r, "IR,S:1532,ST:FAULT,L:NA,R:NA,AGE:14,F:140");

    r.ageTicks = 90000U;
    ExpectReply(&r, "IR,S:1532,ST:FAULT,L:NA,R:NA,AGE:65535,F:140");

    r.ageTicks = 14U;
    r.faultMask = (uint16_t)(APP_IR_FAULT_RECOVERY | APP_IR_FAULT_TIMEOUT | APP_IR_FAULT_STALE);
    ExpectReply(&r, "IR,S:1532,ST:FAULT,L:NA,R:NA,AGE:14,F:844");

    r.faultMask = (uint16_t)(APP_IR_FAULT_TASK | APP_IR_FAULT_STALE);
    ExpectReply(&r, "IR,S:1532,ST:FAULT,L:NA,R:NA,AGE:14,F:C0");

    /* Never published: sequence 0, no age. */
    r = GoodReading();
    r.acq = (uint8_t)APP_IR_ACQ_STALE;
    r.sequence = 0U;
    r.ageTicks = UINT32_MAX;
    r.faultMask = APP_IR_FAULT_STALE;
    r.range[0] = (uint8_t)APP_IR_RANGE_INVALID;
    r.range[1] = (uint8_t)APP_IR_RANGE_INVALID;
    r.distanceMm[0] = 0U;
    ExpectReply(&r, "IR,S:0,ST:STALE,L:NA,R:NA,AGE:NA,F:40");

    /* AGE: ticks to milliseconds, capped at 65535. A very large age is a
     * legitimate STALE reading (IRTask stopped publishing a long time ago). */
    r.sequence = 1532U;
    r.ageTicks = 65535U;
    ExpectReply(&r, "IR,S:1532,ST:STALE,L:NA,R:NA,AGE:65535,F:40");
    r.ageTicks = 65536U;
    ExpectReply(&r, "IR,S:1532,ST:STALE,L:NA,R:NA,AGE:65535,F:40");
    r.ageTicks = 4000000000U;
    ExpectReply(&r, "IR,S:1532,ST:STALE,L:NA,R:NA,AGE:65535,F:40");
    r.ageTicks = 101U;
    ExpectReply(&r, "IR,S:1532,ST:STALE,L:NA,R:NA,AGE:101,F:40");
}

/* SYNTHETIC cases. These are NOT states app_ir.c can produce; they only prove
 * the formatter's robustness and that no reply overflows its buffer. They must
 * never be quoted as protocol examples. */
static void test_formatter_limits_synthetic(void)
{
    AppIrReading r = GoodReading();
    char line[400] = "";

    /* (a) an OK acquisition whose range is INVALID prints NA, never a number. */
    r.range[0] = (uint8_t)APP_IR_RANGE_INVALID;
    ExpectReply(&r, "IR,S:1532,ST:OK,L:NA,R:FAR,AGE:14,F:00");

    /* (b) the longest possible line (impossible in practice: F:FFFF would be
     * a FAULT, and an age of 65535 ms cannot be OK). */
    Fresh();
    r = GoodReading();
    r.sequence = UINT32_MAX;
    r.ageTicks = 70000U;
    r.faultMask = 0xFFFFU;
    r.range[0] = (uint8_t)APP_IR_RANGE_NEAR;
    r.range[1] = (uint8_t)APP_IR_RANGE_NEAR;
    SetReading(&r);
    CHECK(AskOnce(line, sizeof(line)) == 1);
    CHECK_STR(line, "IR,S:4294967295,ST:OK,L:NEAR,R:NEAR,AGE:65535,F:FFFF");
    CHECK(strlen(line) == 52U);             /* + newline = 53 B, buffer is 256 */
    printf("synthetic worst-case IR,GET line length: %u bytes (+newline)\n",
           (unsigned)strlen(line));

    /* (c) every acquisition state with a full mask still fits. */
    r.acq = (uint8_t)APP_IR_ACQ_STALE;
    ExpectReply(&r, "IR,S:4294967295,ST:STALE,L:NA,R:NA,AGE:65535,F:FFFF");
}

static void test_no_unsolicited_and_one_per_request(void)
{
    AppIrReading r = GoodReading();
    unsigned i;
    char line[400];

    Fresh();
    SetReading(&r);

    for (i = 0U; i < 300U; i++) { Cycle(); }        /* 3 s, no request */
    CHECK(CountPrefix("IR,") == 0U);
#if !APP_IR_BENCH_LOG
    CHECK(Comm_TxLineCount() == 0U);
#endif
    CHECK(IrStub_GetReadingCallCount() == 0U);      /* never even read */

    /* Parsing the request does not read or send anything by itself. */
    Comm_TxClear();
    RxNow("IR,GET\n");
    CHECK(Comm_TxLineCount() == 0U);
    CHECK(IrStub_GetReadingCallCount() == 0U);

    Cycle();
    CHECK(CountPrefix("IR,") == 1U);
    CHECK(IrStub_GetReadingCallCount() == 1U);
    CHECK(FindLine("IR,", line, sizeof(line)));
    CHECK_STR(line, "IR,S:1532,ST:OK,L:237,R:FAR,AGE:14,F:00");

    for (i = 0U; i < 50U; i++) { Cycle(); }
    CHECK(CountPrefix("IR,") == 1U);                /* exactly one */

    /* Two requests, two replies, one per pass. */
    Comm_TxClear();
    RxNow("IR,GET\n");
    RxNow("IR,GET\n");
    CHECK(Comm_TxLineCount() == 0U);
    Cycle();
    CHECK(CountPrefix("IR,") == 1U);
    Cycle();
    CHECK(CountPrefix("IR,") == 2U);
    for (i = 0U; i < 20U; i++) { Cycle(); }
    CHECK(CountPrefix("IR,") == 2U);
}

static void test_flood_is_bounded_and_never_an_error(void)
{
    AppIrReading r = GoodReading();
    unsigned i;
    unsigned replies = 0U;

    Fresh();
    SetReading(&r);

    for (i = 0U; i < 12U; i++) { RxNow("IR,GET\n"); }
    CHECK(Comm_TxLineCount() == 0U);                /* nothing sent from the parser */

    for (i = 0U; i < 40U; i++)
    {
        unsigned before = CountPrefix("IR,");

        Cycle();
        replies += CountPrefix("IR,") - before;
        CHECK((CountPrefix("IR,") - before) <= 1U); /* at most one per pass */
    }

    CHECK(AppComm_IrGetDroppedCount() == 8U);       /* 12 requests: 4 owed, 8 dropped */
    CHECK(AppComm_IrGetPendingHighWater() == 4U);
    CHECK(replies == 4U);                           /* 4 owed, 8 discarded */
    CHECK(CountPrefix("ERR") == 0U);                /* never an ERR line */
    CHECK(CountPrefix("IR,") == 4U);
#if !APP_IR_BENCH_LOG
    CHECK(Comm_TxLineCount() == 4U);
#endif
}

/* A client that keeps ONE IR,GET outstanding (sends the next request only
 * after the previous reply) never has more than one reply owed and never
 * drops one, in every situation: idle, during a movement, across its terminal
 * reply, across a STOP, with a rejected second MOV, and polling as fast as the
 * protocol allows (a new request in the pass right after each reply). */
static void RunPollingScenario(const char *name, unsigned passes, unsigned stopAt,
                               unsigned secondMovAt, unsigned gapPasses)
{
    AppIrReading r = GoodReading();
    unsigned requests = 0U;
    unsigned replies = 0U;
    unsigned waited = 0U;
    unsigned maxWait = 0U;
    unsigned i;
    int outstanding = 0;
    unsigned idleFor = 0U;

    Fresh();
    SetReading(&r);
    RxNow("MOV,F,50\n");

    for (i = 0U; i < passes; i++)
    {
        unsigned before = CountPrefix("IR,");

        if (i == stopAt) { RxNow("STOP\n"); }
        if (i == secondMovAt) { RxNow("MOV,F,60\n"); }       /* ERR,BUSY while moving */
        if ((i == 30U) && (stopAt == 0U))
        {
            Fake_SetEncoderLeft(MOV_50CM_TARGET_COUNTS);
            Fake_SetEncoderRight(MOV_50CM_TARGET_COUNTS);
        }

        if (!outstanding)
        {
            if (idleFor >= gapPasses)
            {
                RxNow("IR,GET\n");
                outstanding = 1;
                requests++;
                waited = 0U;
                idleFor = 0U;
            }
            else
            {
                idleFor++;
            }
        }

        Cycle();

        if (CountPrefix("IR,") > before)
        {
            replies += CountPrefix("IR,") - before;
            outstanding = 0;
            if (waited > maxWait) { maxWait = waited; }
        }
        else if (outstanding)
        {
            waited++;
        }
    }

    CHECK(requests > 10U);
    CHECK(replies == requests - (outstanding ? 1U : 0U));
    CHECK(AppComm_IrGetDroppedCount() == 0U);
    CHECK(AppComm_IrGetPendingHighWater() == 1U);
    /* A reply waits a few passes (10 ms each); behind a STOP it also waits for
     * the 120 ms brake and ACK,STOPPED, so up to about 15-20 passes. */
    CHECK(maxWait <= ((stopAt != 9999U && stopAt != 0U) ? 20U : 5U));
    printf("polling %-28s %4u requests, %4u replies, dropped %u, high-water %u, longest wait %u passes\n",
           name, requests, replies, (unsigned)AppComm_IrGetDroppedCount(),
           (unsigned)AppComm_IrGetPendingHighWater(), maxWait);
}

static void test_one_outstanding_never_drops(void)
{
    /* stopAt 0 means "no STOP, finish the movement at pass 30"; passes are
     * 10 ms each. gapPasses is the idle time between a reply and the next
     * request (0 = as fast as possible, 10 = 10 Hz). */
    RunPollingScenario("as fast as possible, idle", 400U, 9999U, 9999U, 0U);
    RunPollingScenario("10 Hz, idle", 400U, 9999U, 9999U, 10U);
    RunPollingScenario("fast, movement completes", 400U, 0U, 9999U, 0U);
    RunPollingScenario("fast, STOP mid-movement", 400U, 40U, 9999U, 0U);
    RunPollingScenario("fast, STOP at start", 400U, 1U, 9999U, 0U);
    RunPollingScenario("fast, rejected 2nd MOV", 400U, 9999U, 35U, 0U);
    RunPollingScenario("10 Hz, STOP + rejected MOV", 400U, 50U, 20U, 10U);
}

static void test_does_not_change_motion(void)
{
    AppIrReading r = GoodReading();
    unsigned i;
    unsigned sawIr = 0U;
    char line[400];
    unsigned total;
    unsigned k;

    Fresh();
    SetReading(&r);

    RxNow("MOV,F,50\n");
    for (i = 0U; i < 25U; i++) { Cycle(); }
    CHECK(AppMotion_GetState() != APP_MOTION_IDLE);

    RxNow("IR,GET\n");
    Cycle();
    CHECK(AppMotion_GetState() != APP_MOTION_IDLE);     /* still running */
    CHECK(CountPrefix("IR,") == 1U);
    CHECK(CountPrefix("ERR") == 0U);                    /* no ERR,BUSY etc. */

    /* Finish the movement: one ACK, and no second movement was queued. */
    Fake_SetEncoderLeft(MOV_50CM_TARGET_COUNTS);
    Fake_SetEncoderRight(MOV_50CM_TARGET_COUNTS);
    for (i = 0U; i < 100U; i++) { Cycle(); }
    CHECK(CountPrefix("ACK") == 1U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);

    total = Comm_TxLineCount();
    for (k = 0U; k < total; k++)
    {
        (void)Comm_TxLine(k, line, sizeof(line));
        if (strncmp(line, "IR,", 3) == 0) { sawIr++; }
    }
    CHECK(sawIr == 1U);
}

static void test_unknown_ir_lines(void)
{
    char line[400];
    static const char *const bad[] = {
        "IR\n", "IR,\n", "IR,GET,1\n", "ir,get\n", "IR,GETX\n", "IR, GET\n",
        "IRRAW\n", "IRRAW,S:1,L:1,R:1,F:00\n", "IR,SET\n"
    };
    unsigned i;

    for (i = 0U; i < sizeof(bad) / sizeof(bad[0]); i++)
    {
        AppIrReading r = GoodReading();

        Fresh();
        SetReading(&r);
        RxNow(bad[i]);
        CHECK(Comm_TxLineCount() == 1U);
        (void)Comm_TxLine(0U, line, sizeof(line));
        CHECK_STR(line, "ERR,BAD_CMD");
        Cycle();
        Cycle();
        CHECK(CountPrefix("IR,") == 0U);
        CHECK(IrStub_GetReadingCallCount() == 0U);
    }
}

/* STOP outstanding: the reply waits until after ACK,STOPPED. */
static void test_stop_outstanding(void)
{
    AppIrReading r = GoodReading();
    unsigned i;
    unsigned total;
    unsigned k;
    unsigned stoppedIndex = 0U;
    unsigned irIndex = 0U;
    int sawStopped = 0;
    int sawIr = 0;
    char line[400];

    /* STOP first, then IR,GET, MotionTask not yet run. */
    Fresh();
    SetReading(&r);
    RxNow("STOP\n");
    RxNow("IR,GET\n");
    Comm_TxClear();
    for (i = 0U; i < 30U; i++)
    {
        Fake_SetTickCount(Fake_GetTickCount() + 10U);
        CommStep();                                 /* CommTask only */
    }
    CHECK(CountPrefix("IR,") == 0U);                /* held back */
    CHECK(CountPrefix("ACK,STOPPED") == 0U);

    Cycle();                                        /* MotionTask publishes STOPPED */
    CHECK(CountPrefix("ACK,STOPPED") == 1U);
    CHECK(CountPrefix("IR,") == 0U);                /* not in the same pass */
    Cycle();
    CHECK(CountPrefix("IR,") == 1U);

    total = Comm_TxLineCount();
    for (k = 0U; k < total; k++)
    {
        (void)Comm_TxLine(k, line, sizeof(line));
        if (strcmp(line, "ACK,STOPPED") == 0) { stoppedIndex = k; sawStopped = 1; }
        if (strncmp(line, "IR,", 3) == 0) { irIndex = k; sawIr = 1; }
    }
    CHECK(sawStopped && sawIr);
    CHECK(irIndex > stoppedIndex);
    CHECK(CountPrefix("ACK,STOPPED") == 1U);

    /* IR,GET first, then STOP in the same pass. */
    Fresh();
    SetReading(&r);
    RxNow("IR,GET\n");
    RxNow("STOP\n");
    Comm_TxClear();
    Cycle();
    Cycle();
    CHECK(CountPrefix("IR,") == 0U || CountPrefix("ACK,STOPPED") == 1U);
    total = Comm_TxLineCount();
    stoppedIndex = 0U;
    irIndex = 0U;
    sawStopped = 0;
    sawIr = 0;
    for (k = 0U; k < total; k++)
    {
        (void)Comm_TxLine(k, line, sizeof(line));
        if (strcmp(line, "ACK,STOPPED") == 0) { stoppedIndex = k; sawStopped = 1; }
        if (strncmp(line, "IR,", 3) == 0) { irIndex = k; sawIr = 1; }
    }
    CHECK(sawStopped);
    for (i = 0U; i < 5U; i++) { Cycle(); }
    total = Comm_TxLineCount();
    sawIr = 0;
    for (k = 0U; k < total; k++)
    {
        (void)Comm_TxLine(k, line, sizeof(line));
        if (strcmp(line, "ACK,STOPPED") == 0) { stoppedIndex = k; }
        if (strncmp(line, "IR,", 3) == 0) { irIndex = k; sawIr = 1; }
    }
    CHECK(sawIr);
    CHECK(irIndex > stoppedIndex);                  /* reply after the STOP ack */
    CHECK(CountPrefix("ACK,STOPPED") == 1U);
}

/* An ERR sent by the parser (ERR,BUSY for a second MOV) is already on the
 * wire before the deferred IR reply. */
static void test_err_busy_first(void)
{
    AppIrReading r = GoodReading();
    char line[400];
    unsigned total;
    unsigned k;
    unsigned busyIndex = 0U;
    unsigned irIndex = 0U;

    Fresh();
    SetReading(&r);
    RxNow("MOV,F,50\n");
    RxNow("IR,GET\n");
    RxNow("MOV,F,60\n");                            /* rejected: ERR,BUSY */
    Cycle();
    Cycle();
    Cycle();

    total = Comm_TxLineCount();
    for (k = 0U; k < total; k++)
    {
        (void)Comm_TxLine(k, line, sizeof(line));
        if (strcmp(line, "ERR,BUSY") == 0) { busyIndex = k + 1U; }
        if (strncmp(line, "IR,", 3) == 0) { irIndex = k + 1U; }
    }
    CHECK(busyIndex != 0U);
    CHECK(irIndex != 0U);
    CHECK(busyIndex < irIndex);
    CHECK(CountPrefix("ERR,BUSY") == 1U);           /* the IR request did not add one */
}

/* For every phase of the service cadence relative to the movement, with an
 * IR,GET request becoming pending in the very pass in which MotionTask
 * publishes the terminal event: the terminal reply is sent in that pass,
 * MM,EV:END is immediately followed by it, the IR reply is not in that pass
 * and arrives in the next one. */
static void RunTerminalTrial(unsigned offset, int useStop, unsigned *irAfter)
{
    AppMotionEvent before;
    AppMotionEvent after;
    AppIrReading r = GoodReading();
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

    Fresh();
    SetReading(&r);
#if APP_IR_BENCH_LOG
    irSnap = (AppIrSnapshot){0};
    irSnap.sequence = 42U;
    irSnap.status = (uint8_t)APP_IR_STATUS_VALID;
    irSnap.leftRaw = 1820U;
    irSnap.rightRaw = 1764U;
    keepFresh = 1;
    irSnap.publishTick = Fake_GetTickCount();
    IrStub_Set(&irSnap);
#endif
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
#if APP_IR_BENCH_LOG
        irSnap.publishTick = Fake_GetTickCount();
        IrStub_Set(&irSnap);
#endif

        AppMotion_GetEvent(&before);
        MotionStep();
        AppMotion_GetEvent(&after);

        if (after.sequence != before.sequence)
        {
            /* The terminal event is published and an IR request arrives
             * before CommTask services it. */
            RxNow("IR,GET\n");
            linesBefore = Comm_TxLineCount();
            CommStep();

            total = Comm_TxLineCount();
            {
                int foundTerminal = 0;
                int irBeforeTerminal = 0;
                int irInPass = 0;

                for (k = linesBefore; k < total; k++)
                {
                    (void)Comm_TxLine(k, line, sizeof(line));
                    if (strcmp(line, terminal) == 0) { foundTerminal = 1; }
                    if (strncmp(line, "IR,", 3) == 0)
                    {
                        irInPass = 1;
                        if (!foundTerminal) { irBeforeTerminal = 1; }
                    }
                }
                CHECK(foundTerminal);               /* sent in the first pass */
                CHECK(!irBeforeTerminal);
                CHECK(!irInPass);                   /* IR waits one more pass */
            }

            {
                unsigned beforeNext = Comm_TxLineCount();

                Cycle();
                total = Comm_TxLineCount();
                {
                    unsigned irNow = 0U;

                    for (k = beforeNext; k < total; k++)
                    {
                        (void)Comm_TxLine(k, line, sizeof(line));
                        if (strncmp(line, "IR,", 3) == 0) { irNow++; }
                    }
                    CHECK(irNow == 1U);
                    if (irNow == 1U) { (*irAfter)++; }
                }
            }
            break;
        }

        CommStep();
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
    CHECK(startIndex < endIndex);
    CHECK(ackIndex == (endIndex + 1U));             /* END immediately before the reply */
    CHECK(CountPrefix(terminal) == 1U);
    CHECK(CountPrefix("IR,") == 1U);
}

static void test_terminal_reply_never_delayed(void)
{
    unsigned offset;
    unsigned afterStop = 0U;
    unsigned afterAck = 0U;

    for (offset = 0U; offset < 20U; offset++)
    {
        RunTerminalTrial(offset, 1, &afterStop);
        RunTerminalTrial(offset, 0, &afterAck);
    }

    CHECK(afterStop == 20U);
    CHECK(afterAck == 20U);
    printf("terminal-reply trials: IR reply came after ACK,STOPPED %u/20 and "
           "after ACK %u/20 times\n", afterStop, afterAck);
}

#if APP_IR_BENCH_LOG
/* IR,GET and the IRRAW / US, bench lines share the non-terminal slot: the
 * IR reply goes first, a bench line due in the same pass waits. */
static void test_before_bench_lines(void)
{
    AppIrReading r = GoodReading();
    unsigned irLines = 0U;
    unsigned rawLines = 0U;
    unsigned i;
    unsigned requests = 0U;
    char line[400];
#if APP_ULTRASONIC_BENCH_LOG
    AppUltrasonicSnapshot us = {0};
    unsigned usLines = 0U;

    us.sequence = 3U;
    us.result = APP_US_RESULT_NONE;
#endif

    Fresh();
    SetReading(&r);
    irSnap = (AppIrSnapshot){0};
    irSnap.sequence = 42U;
    irSnap.status = (uint8_t)APP_IR_STATUS_VALID;
    irSnap.leftRaw = 1820U;
    irSnap.rightRaw = 1764U;
    keepFresh = 1;
    irSnap.publishTick = Fake_GetTickCount();
    IrStub_Set(&irSnap);
#if APP_ULTRASONIC_BENCH_LOG
    UsStub_Set(&us);
#endif

    for (i = 0U; i < 400U; i++)                     /* 4 s */
    {
        unsigned before = Comm_TxLineCount();
        unsigned after;
        unsigned k;
        unsigned nonTerminal = 0U;
        int irThisPass = 0;
        int otherThisPass = 0;

        if ((i % 5U) == 0U)                         /* a request every 50 ms */
        {
            RxNow("IR,GET\n");
            requests++;
        }

        Cycle();
        after = Comm_TxLineCount();

        for (k = before; k < after; k++)
        {
            (void)Comm_TxLine(k, line, sizeof(line));
            if (strncmp(line, "IR,", 3) == 0) { irLines++; irThisPass = 1; nonTerminal++; }
            else if (strncmp(line, "IRRAW,", 6) == 0) { rawLines++; otherThisPass = 1; nonTerminal++; }
#if APP_ULTRASONIC_BENCH_LOG
            else if (strncmp(line, "US,", 3) == 0) { usLines++; otherThisPass = 1; nonTerminal++; }
#endif
        }
        CHECK(nonTerminal <= 1U);                   /* one non-terminal line per pass */
        (void)irThisPass;
        (void)otherThisPass;
    }

    CHECK(irLines == requests);                     /* every request answered once */
    CHECK(rawLines >= 15U);                         /* bench log still runs */
#if APP_ULTRASONIC_BENCH_LOG
    CHECK(usLines >= 15U);
#endif
    CHECK(CountPrefix("ERR") == 0U);
}
#endif /* APP_IR_BENCH_LOG */

#else /* APP_IR_ENABLE == 0: the IR feature is compiled out */

static void test_compiled_out(void)
{
    char line[400];
    unsigned i;

    Fresh();
    RxNow("IR,GET\n");
    CHECK(Comm_TxLineCount() == 1U);
    (void)Comm_TxLine(0U, line, sizeof(line));
    CHECK_STR(line, "ERR,BAD_CMD");

    for (i = 0U; i < 300U; i++)
    {
        Fake_SetTickCount(Fake_GetTickCount() + 10U);
        MotionStep();
        CommStep();
    }
    CHECK(Comm_TxLineCount() == 1U);                /* nothing else, ever */

    /* Same answer for the other IR lines; motion untouched. */
    Fresh();
    RxNow("IR,GET\n");
    RxNow("IRRAW\n");
    CHECK(Comm_TxLineCount() == 2U);
    CHECK(CountPrefix("ERR,BAD_CMD") == 2U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
}

#endif /* APP_IR_ENABLE */

int main(void)
{
    MotionQueueHandle = osMessageQueueNew(8U, sizeof(MotionCommand), NULL);
    CommRxQueueHandle = osMessageQueueNew(128U, 1U, NULL);
    CHECK(MotionQueueHandle != NULL);
    CHECK(CommRxQueueHandle != NULL);

    printf("mode: APP_IR_ENABLE=%d APP_IR_BENCH_LOG=%d APP_ULTRASONIC_BENCH_LOG=%d\n",
           APP_IR_ENABLE, APP_IR_BENCH_LOG, APP_ULTRASONIC_BENCH_LOG);

#if APP_IR_ENABLE
    test_exact_lines();
    test_formatter_limits_synthetic();
    test_no_unsolicited_and_one_per_request();
    test_flood_is_bounded_and_never_an_error();
    test_one_outstanding_never_drops();
    test_does_not_change_motion();
    test_unknown_ir_lines();
    test_stop_outstanding();
    test_err_busy_first();
    test_terminal_reply_never_delayed();
#if APP_IR_BENCH_LOG
    test_before_bench_lines();
#endif
#else
    test_compiled_out();
#endif

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
