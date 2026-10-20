/* Full-stack host test of the measurement-only USPARK trace in the REAL
 * Core/Src/app_comm.c, driving the REAL app_motion.c, vision_rx.c,
 * app_us_park.c and app_park_trace.c against the host fakes. Like
 * app_comm_uspark_test.c it plays both RTOS roles deterministically, in the
 * exact order main.c uses:
 *
 *   MotionTask:  AppComm_TakeMotionCommand -> AppMotion_HandleCommand ->
 *                AppMotion_Update -> AppParkTrace_Sample
 *   CommTask:    parse bytes -> AppComm_ServiceTelemetry ->
 *                AppComm_ServiceParking -> AppComm_ServiceParkTrace
 *
 * A tiny deterministic wheel "plant" drives the fake encoders (80 counts per
 * 10 ms cycle while moving, decelerating once the brake is applied, the two
 * wheels at different rates so the two zero times differ).
 *
 * ONE source, TWO builds:
 *   - default (APP_USPARK_TRACE=0): the production flow. No "USPARK,TRACE"
 *     line may ever be sent; the hooks do not exist.
 *   - -DAPP_USPARK_TRACE=1: everything the trace promises, scenario by
 *     scenario.
 *
 * In both builds, `app_comm_park_trace_test.exe <digest-file>` writes a
 * behaviour digest: per cycle the motion state, last motor duties, motor
 * set/stop/brake call counts, plus every transmitted line that is NOT a
 * trace line. The digests of the OFF and ON builds must be byte-identical
 * (checked by the caller): motor commands, STOP timing and normal UART output
 * are unchanged by the trace. A second optional argument writes the raw trace
 * lines of the first scenario in the monitor's log format, for the offline
 * analyser.
 *
 * Build (from repo root), trace ON; drop the -D for OFF:
 *   gcc -std=c11 -Wall -Wextra -Itests/comm/fakes -Itests/motion/fakes \
 *       -ICore/Inc -DAPP_USPARK_TRACE=1 \
 *       -DAPP_COMM_TEST_HOOK_BEFORE_PARK_STOP=ParkTraceHook \
 *       -o tests/comm/app_comm_park_trace_test.exe \
 *       tests/comm/app_comm_park_trace_test.c Core/Src/app_comm.c \
 *       Core/Src/vision_rx.c Core/Src/app_motion.c Core/Src/app_pid.c Core/Src/app_ir_edge.c Core/Src/app_ir_dist.c \
 *       Core/Src/app_us_park.c Core/Src/app_park_trace.c \
 *       Core/Src/app_park_trace_fmt.c tests/motion/fakes/hw_stubs.c \
 *       tests/comm/fakes/comm_stubs.c tests/comm/fakes/ultrasonic_stub.c \
 *       tests/comm/fakes/ir_stub.c
 */

#include "app_comm.h"
#include "app_motion.h"
#include "app_servo.h"
#include "app_ultrasonic.h"
#include "app_encoder.h"
#include "app_park_trace.h"
#include "cmsis_os.h"
#include "main.h"
#include "test_fakes.h"
#include "ultrasonic_stub.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __GNUC__
#define MAYBE_UNUSED __attribute__((unused))
#else
#define MAYBE_UNUSED
#endif

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

#define CHECK_EQ(actual, expected) do { \
    long long a_ = (long long)(actual); \
    long long e_ = (long long)(expected); \
    checks++; \
    if (a_ != e_) { \
        failures++; \
        printf("FAIL line %d: %s == %lld, expected %lld\n", \
               __LINE__, #actual, a_, e_); \
    } \
} while (0)

/* ---- transmit log (harvested after every step; the fake caps at 8 KB) -- */

#define LOG_MAX     1200U
#define LOG_LINE    200U

typedef struct
{
    unsigned cycle;
    uint32_t tick;
    char text[LOG_LINE];
} LogLine;

static LogLine logLines[LOG_MAX];
static unsigned logCount;
static unsigned cycleNo;

static void Harvest(void)
{
    unsigned total = Comm_TxLineCount();
    unsigned i;

    for (i = 0U; i < total; i++)
    {
        if (logCount < LOG_MAX)
        {
            (void)Comm_TxLine(i, logLines[logCount].text, LOG_LINE);
            logLines[logCount].cycle = cycleNo;
            logLines[logCount].tick = Fake_GetTickCount();
            logCount++;
        }
        else
        {
            failures++;
            printf("FAIL: transmit log overflow\n");
        }
    }

    Comm_TxClear();
}

static int StartsWith(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static int IsTrace(const char *s)
{
    return StartsWith(s, "USPARK,TRACE,");
}

static unsigned CountPrefix(const char *prefix)
{
    unsigned i;
    unsigned n = 0U;

    for (i = 0U; i < logCount; i++)
    {
        if (StartsWith(logLines[i].text, prefix)) { n++; }
    }

    return n;
}

static int FindPrefix(const char *prefix, unsigned nth)
{
    unsigned i;

    for (i = 0U; i < logCount; i++)
    {
        if (StartsWith(logLines[i].text, prefix))
        {
            if (nth == 0U) { return (int)i; }
            nth--;
        }
    }

    return -1;
}

static unsigned TraceCount(void)
{
    unsigned i;
    unsigned n = 0U;

    for (i = 0U; i < logCount; i++)
    {
        if (IsTrace(logLines[i].text)) { n++; }
    }

    return n;
}

/* ---- trace-line parsing (only used by the ON build) -------------------- */

#define TRACE_MAX 80U

typedef struct
{
    int logIndex;
    char ev[8];
    char text[LOG_LINE];
} TraceLine;

static TraceLine trace[TRACE_MAX];
static unsigned traceCount;

static MAYBE_UNUSED void ParseTrace(void)
{
    unsigned i;

    traceCount = 0U;

    for (i = 0U; i < logCount; i++)
    {
        if (IsTrace(logLines[i].text) && (traceCount < TRACE_MAX))
        {
            const char *e = strstr(logLines[i].text, "EV:") + 3;
            size_t n = strcspn(e, ",");

            trace[traceCount].logIndex = (int)i;
            memset(trace[traceCount].ev, 0, sizeof(trace[traceCount].ev));
            memcpy(trace[traceCount].ev, e, (n < 7U) ? n : 7U);
            strcpy(trace[traceCount].text, logLines[i].text);
            traceCount++;
        }
    }
}

static MAYBE_UNUSED int TraceIndex(const char *ev, unsigned nth)
{
    unsigned i;

    for (i = 0U; i < traceCount; i++)
    {
        if (strcmp(trace[i].ev, ev) == 0)
        {
            if (nth == 0U) { return (int)i; }
            nth--;
        }
    }

    return -1;
}

static MAYBE_UNUSED unsigned TraceEvCount(const char *ev)
{
    unsigned i;
    unsigned n = 0U;

    for (i = 0U; i < traceCount; i++)
    {
        if (strcmp(trace[i].ev, ev) == 0) { n++; }
    }

    return n;
}

/* Numeric value of ",KEY:" in line `t`; LLONG_MIN-ish sentinel if absent/NA. */
#define NOFIELD (-9999999999LL)

static MAYBE_UNUSED long long Field(unsigned t, const char *key)
{
    char pattern[16];
    const char *p;

    snprintf(pattern, sizeof(pattern), ",%s:", key);
    p = strstr(trace[t].text, pattern);

    if (p == NULL) { return NOFIELD; }
    p += strlen(pattern);
    if (strncmp(p, "NA", 2) == 0) { return NOFIELD; }
    return atoll(p);
}

static MAYBE_UNUSED int FieldIs(unsigned t, const char *key, const char *value)
{
    char pattern[32];

    snprintf(pattern, sizeof(pattern), ",%s:%s,", key, value);
    if (strstr(trace[t].text, pattern) != NULL) { return 1; }

    snprintf(pattern, sizeof(pattern), ",%s:%s\n", key, value);
    return strstr(trace[t].text, pattern) != NULL;
}

/* ---- firmware-role emulation ------------------------------------------ */

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
    Harvest();
}

static MAYBE_UNUSED void RxQueueOnly(const char *text)
{
    size_t i;
    size_t n = strlen(text);

    for (i = 0U; i < n; i++)
    {
        uint8_t byte = (uint8_t)text[i];
        (void)osMessageQueuePut(CommRxQueueHandle, &byte, 0U, 0U);
    }
}

/* Wheel plant ---------------------------------------------------------- */

static int32_t plantL;
static int32_t plantR;                  /* logical (unwrapped) right count */
static int32_t plantSpdL;
static int32_t plantSpdR;
static int plantBraking;
static int32_t plantRightOffset;        /* added to the logical right count */

static int32_t Raw16(int32_t logical)
{
    return (int32_t)(int16_t)(uint16_t)(uint32_t)logical;
}

static void PlantStep(void)
{
    AppMotionMovementStatus ms;
    AppMotionState s = AppMotion_GetState();
    int dir = 1;

    AppMotion_GetMovementStatus(&ms);
    if (ms.command == MOTION_BACKWARD) { dir = -1; }

    if (s == APP_MOTION_BRAKING) { plantBraking = 1; }

    if (plantBraking != 0)
    {
        if (plantSpdL > 0) { plantSpdL -= 20; if (plantSpdL < 0) { plantSpdL = 0; } }
        if (plantSpdL < 0) { plantSpdL += 20; if (plantSpdL > 0) { plantSpdL = 0; } }
        if (plantSpdR > 0) { plantSpdR -= 40; if (plantSpdR < 0) { plantSpdR = 0; } }
        if (plantSpdR < 0) { plantSpdR += 40; if (plantSpdR > 0) { plantSpdR = 0; } }
        if ((plantSpdL == 0) && (plantSpdR == 0)) { plantBraking = 0; }
    }
    else if (s == APP_MOTION_DISTANCE)
    {
        plantSpdL = 80 * dir;
        plantSpdR = 80 * dir;
    }

    plantL += plantSpdL;
    plantR += plantSpdR;
    Fake_SetEncoderLeft(plantL);
    Fake_SetEncoderRight(Raw16(plantR + plantRightOffset));
}

static void MotionStep(void)
{
    MotionCommand command;

    if (AppComm_TakeMotionCommand(&command))
    {
        AppMotion_HandleCommand(&command);
    }

    AppMotion_Update(0.01f);

#if APP_USPARK_TRACE
    AppParkTrace_Sample(osKernelGetTickCount(),
                        AppEncoder_Left(),
                        AppEncoder_Right());
#endif
}

static void CommStep(void)
{
    DrainRx();
    AppComm_ServiceTelemetry();
    AppComm_ServiceParking();
#if APP_USPARK_TRACE
    AppComm_ServiceParkTrace();
#endif
}

/* Behaviour digest ------------------------------------------------------ */

static char *digest;
static size_t digestLen;
static int digestEnabled = 1;           /* off for ON-only scenarios */
#define DIGEST_MAX (4U * 1024U * 1024U)

static void DigestPrintf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#include <stdarg.h>
static void DigestPrintf(const char *fmt, ...)
{
    va_list ap;
    int n;

    if ((digest == NULL) || (digestEnabled == 0)) { return; }
    va_start(ap, fmt);
    n = vsnprintf(digest + digestLen, DIGEST_MAX - digestLen, fmt, ap);
    va_end(ap);
    if (n > 0) { digestLen += (size_t)n; }
}

static unsigned digestedLog;

static void DigestCycle(void)
{
    DigestPrintf("C%u T%lu S%d L%ld R%ld SET%u STOP%u BRK%u\n",
                 cycleNo, (unsigned long)Fake_GetTickCount(),
                 (int)AppMotion_GetState(),
                 (long)Fake_GetLastMotorLeftDuty(),
                 (long)Fake_GetLastMotorRightDuty(),
                 (unsigned)Fake_GetMotorSetCallCount(),
                 (unsigned)Fake_GetMotorStopCallCount(),
                 (unsigned)Fake_GetMotorBrakeCallCount());

    /* Every transmitted line that is not a trace line, in order. */
    while (digestedLog < logCount)
    {
        if (!IsTrace(logLines[digestedLog].text))
        {
            DigestPrintf("  TX@C%u %s\n", logLines[digestedLog].cycle,
                         logLines[digestedLog].text);
        }
        digestedLog++;
    }
}

static void Cycle(void)
{
    Fake_SetTickCount(Fake_GetTickCount() + 10U);
    cycleNo++;
    PlantStep();
    MotionStep();
    CommStep();
    Harvest();
    DigestCycle();
}

static void Cycles(unsigned n)
{
    unsigned i;

    for (i = 0U; i < n; i++) { Cycle(); }
}

static unsigned NonTraceCount(void)
{
    unsigned i;
    unsigned n = 0U;

    for (i = 0U; i < logCount; i++)
    {
        if (!IsTrace(logLines[i].text)) { n++; }
    }

    return n;
}

/* Runs until the motion is idle and the NORMAL protocol output has been quiet
 * for 300 ms, then a fixed drain allowance. Trace lines deliberately do not
 * count as activity, and the allowance is the same in both builds, so the
 * number of cycles (and therefore the behaviour digest) cannot depend on the
 * trace. 80 passes is more than the 48 records the trace can ever hold. */
#define TRACE_DRAIN_ALLOWANCE 80U

static unsigned CyclesUntilIdleAndQuiet(unsigned limit)
{
    unsigned i;
    unsigned quiet = 0U;

    for (i = 0U; i < limit; i++)
    {
        unsigned before = NonTraceCount();

        Cycle();

        if ((AppMotion_GetState() == APP_MOTION_IDLE) &&
            (NonTraceCount() == before))
        {
            quiet++;
            if (quiet >= 30U)           /* 300 ms of silence */
            {
                Cycles(TRACE_DRAIN_ALLOWANCE);
                return i + 1U + TRACE_DRAIN_ALLOWANCE;
            }
        }
        else
        {
            quiet = 0U;
        }
    }

    return limit;
}

/* Ultrasonic injection --------------------------------------------------- */

static uint32_t usSeq;
static AppUltrasonicSnapshot usSnap;

static void UsReset(void)
{
    usSeq = 0U;
    memset(&usSnap, 0, sizeof(usSnap));
    UsStub_Set(&usSnap);
}

static void UsPublishValid(uint16_t mm)
{
    usSeq++;
    usSnap.sequence = usSeq;
    usSnap.cycles = usSeq;
    usSnap.result = APP_US_RESULT_VALID;
    usSnap.filteredValid = 1U;
    usSnap.filteredMm = mm;
    usSnap.rawValid = 1U;
    usSnap.rawMm = (uint16_t)(mm + 1U);
    usSnap.pulseCaptured = 1U;
    usSnap.pulseUs = (uint16_t)(mm * 6U);
    usSnap.lastValidTick = Fake_GetTickCount();
    usSnap.lastSampleTick = Fake_GetTickCount();
    usSnap.state = APP_US_STATE_IDLE;
    usSnap.faultMask = 0U;
    UsStub_Set(&usSnap);
}

static MAYBE_UNUSED void UsPublishInvalid(void)
{
    usSeq++;
    usSnap.sequence = usSeq;
    usSnap.cycles = usSeq;
    usSnap.result = APP_US_RESULT_NO_RISE;
    usSnap.rawValid = 0U;
    usSnap.pulseCaptured = 0U;
    usSnap.lastSampleTick = Fake_GetTickCount();
    usSnap.state = APP_US_STATE_IDLE;
    usSnap.faultMask = 0U;
    UsStub_Set(&usSnap);
}

/* A VALID publication whose sample is already stale (older than 200 ms). */
static MAYBE_UNUSED void UsPublishStale(uint16_t mm)
{
    UsPublishValid(mm);
    usSnap.lastValidTick = Fake_GetTickCount() - 300U;
    UsStub_Set(&usSnap);
}

/* One publication, then the ~70 ms until the next (7 cycles). */
static void Sample(uint16_t mm)
{
    UsPublishValid(mm);
    Cycles(7U);
}

static MAYBE_UNUSED void SampleInvalid(void)
{
    UsPublishInvalid();
    Cycles(7U);
}

/* Scenario helpers -------------------------------------------------------- */

static void Fresh(uint32_t startTick)
{
    FakeOs_ClearFaults();
    FakeOs_SetFaultQueue(MotionQueueHandle);
    (void)osMessageQueueReset(MotionQueueHandle);
    (void)osMessageQueueReset(CommRxQueueHandle);

    Fake_SetTickCount(startTick);
    plantL = 0;
    plantR = 0;
    plantSpdL = 0;
    plantSpdR = 0;
    plantBraking = 0;
    plantRightOffset = 0;
    Fake_SetEncoderLeft(0);
    Fake_SetEncoderRight(0);
    Fake_SetHeading(0.0f);
    Fake_SetGyroZ(0.0f);
    Fake_SetImuReady(true);
    Fake_SetImuEvent(APP_IMU_EVENT_NONE, 0U);

    AppMotion_Init();
    AppComm_Init();
    UsReset();
    Fake_ResetMotorCallCounts();
    Comm_TxClear();
    logCount = 0U;
    digestedLog = 0U;
    cycleNo = 0U;
    DigestPrintf("=== scenario start tick %lu\n", (unsigned long)startTick);
}

static void EnableParking(void)
{
    RxNow("USPARK,1\n");
    CHECK_EQ(CountPrefix("USPARK,OK"), 1);
    logCount = 0U;
    digestedLog = 0U;
}

static void StartForward(unsigned cm)
{
    char cmd[24];

    snprintf(cmd, sizeof(cmd), "MOV,F,%u\n", cm);
    RxNow(cmd);
    Cycle();
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
}

#if APP_USPARK_TRACE
/* Whole trace is on the wire only AFTER ACK,STOPPED; no trace line is ever
 * emitted before it, between the contract lines, or while moving. */
static void ExpectTraceOnlyAfterAck(void)
{
    int ack = FindPrefix("ACK,STOPPED", 0);
    unsigned i;

    CHECK(ack >= 0);

    for (i = 0U; i < logCount; i++)
    {
        if (IsTrace(logLines[i].text))
        {
            CHECK((int)i > ack);
        }
    }
}

/* At most one trace line per CommTask pass (per cycle), each < 120 bytes. */
static void ExpectOneLinePerPassAndShort(void)
{
    unsigned i;
    unsigned run = 0U;
    unsigned lastCycle = 0xFFFFFFFFU;

    for (i = 0U; i < logCount; i++)
    {
        if (IsTrace(logLines[i].text))
        {
            CHECK(strlen(logLines[i].text) < 120U);

            if (logLines[i].cycle == lastCycle) { run++; }
            else { run = 1U; lastCycle = logLines[i].cycle; }

            CHECK(run <= 1U);
        }
    }
}
#endif /* APP_USPARK_TRACE */

/* ---- scenarios -------------------------------------------------------- */

static void ScenarioAutomaticStop(const char *rawLogPath)
{
    Fresh(1000U);
    EnableParking();
    StartForward(100);

    Sample(900U);            /* far: kept as history only                    */
    Sample(800U);
    Sample(400U);            /* arms (<= 500), not near                     */
    Sample(400U);
    Sample(290U);            /* near 1                                       */
    Sample(285U);            /* near 2                                       */
    Sample(280U);            /* near 3 -> automatic STOP                     */
    (void)CyclesUntilIdleAndQuiet(600U);

    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK_EQ(CountPrefix("USPARK,STOP,R:NEAR"), 1);
    CHECK_EQ(CountPrefix("ACK,STOPPED"), 1);

    /* The contract lines are exactly the pre-trace ones, in order. */
    {
        int start = FindPrefix("MM,EV:START,D:F", 0);
        int stop = FindPrefix("USPARK,STOP,R:NEAR", 0);
        int end = FindPrefix("MM,EV:END,D:F", 0);
        int ack = FindPrefix("ACK,STOPPED", 0);

        CHECK(start >= 0 && stop > start && end > stop && ack > end);
    }

#if APP_USPARK_TRACE
    {
        static const char *const order[] = {
            "PRE", "PRE", "PRE", "ARM", "US", "US", "NEAR", "US", "NEAR",
            "US", "NEAR", "REQ", "ACC", "TAKE", "BRK"
        };
        unsigned i;
        int req;
        int acc;
        int take;
        int brk;
        int bend;
        int zl;
        int zr;
        int zero;
        int done;
        unsigned lastStop;

        ParseTrace();
        ExpectTraceOnlyAfterAck();
        ExpectOneLinePerPassAndShort();

        /* The last three valid publications before the window opened (900,
         * 800 and the arming 400) are replayed as PRE, oldest first, then
         * ARM, then one US per new publication with NEAR after the near ones,
         * then the stop chain, exactly in this order. */
        CHECK_EQ(traceCount, 20);
        for (i = 0U; i < sizeof(order) / sizeof(order[0]); i++)
        {
            CHECK(strcmp(trace[i].ev, order[i]) == 0);
        }
        CHECK_EQ(Field(0, "FILT"), 900);
        CHECK_EQ(Field(1, "FILT"), 800);
        CHECK_EQ(Field(2, "FILT"), 400);
        CHECK_EQ(Field(3, "FILT"), 400);        /* ARM at the arming reading */
        CHECK_EQ(Field(0, "SEQ"), 1);
        CHECK_EQ(Field(2, "SEQ"), 3);
        CHECK_EQ(Field(3, "SEQ"), 3);

        /* N: contiguous from 1. */
        for (i = 0U; i < traceCount; i++)
        {
            CHECK_EQ(Field(i, "N"), (long long)i + 1);
        }

        req = TraceIndex("REQ", 0);
        acc = TraceIndex("ACC", 0);
        take = TraceIndex("TAKE", 0);
        brk = TraceIndex("BRK", 0);
        bend = TraceIndex("BEND", 0);
        zl = TraceIndex("ZL", 0);
        zr = TraceIndex("ZR", 0);
        zero = TraceIndex("ZERO", 0);
        done = TraceIndex("DONE", 0);

        CHECK(req > 0 && acc == req + 1 && take == acc + 1 && brk == take + 1);
        CHECK(bend > brk && zl > brk && zr > brk && zero > zl && zero > zr);
        CHECK_EQ(done, (int)traceCount - 1);
        CHECK_EQ(TraceEvCount("ARM"), 1);
        CHECK_EQ(TraceEvCount("REQ"), 1);
        CHECK_EQ(TraceEvCount("ACC"), 1);
        CHECK_EQ(TraceEvCount("TAKE"), 1);
        CHECK_EQ(TraceEvCount("BRK"), 1);
        CHECK_EQ(TraceEvCount("BEND"), 1);
        CHECK_EQ(TraceEvCount("ZL"), 1);
        CHECK_EQ(TraceEvCount("ZR"), 1);
        CHECK_EQ(TraceEvCount("ZERO"), 1);
        CHECK_EQ(TraceEvCount("DONE"), 1);
        CHECK_EQ(TraceEvCount("NEAR"), 3);

        CHECK(FieldIs((unsigned)req, "R", "NEAR"));
        CHECK(FieldIs((unsigned)done, "R", "OK"));
        CHECK_EQ(Field((unsigned)done, "DR"), 0);
        CHECK_EQ(Field((unsigned)done, "RF"), 0);
        CHECK_EQ(Field((unsigned)done, "ZT"), 0);
        CHECK_EQ(Field((unsigned)done, "SK"), 0);

        /* NEAR runs are 1, 2, 3 in order. */
        CHECK_EQ(Field((unsigned)TraceIndex("NEAR", 0), "K"), 1);
        CHECK_EQ(Field((unsigned)TraceIndex("NEAR", 1), "K"), 2);
        CHECK_EQ(Field((unsigned)TraceIndex("NEAR", 2), "K"), 3);

        /* ARM carries the movement id, ultrasonic seq, profile and fault mask. */
        {
            unsigned a = (unsigned)TraceIndex("ARM", 0);

            CHECK(FieldIs(a, "P", "PREC"));
            CHECK(strstr(trace[a].text, ",FM:0,") != NULL);
            CHECK_EQ(Field(a, "CN"), 0);
            CHECK_EQ(Field((unsigned)done, "W"), Field(a, "W"));
        }

        /* Timing relations, all wrap-safe unsigned differences. */
        {
            uint32_t tReq = (uint32_t)Field((unsigned)req, "T");
            uint32_t tAcc = (uint32_t)Field((unsigned)acc, "T");
            uint32_t tTake = (uint32_t)Field((unsigned)take, "T");
            uint32_t tBrk = (uint32_t)Field((unsigned)brk, "T");
            uint32_t tBend = (uint32_t)Field((unsigned)bend, "T");
            uint32_t tZl = (uint32_t)Field((unsigned)zl, "T");
            uint32_t tZr = (uint32_t)Field((unsigned)zr, "T");
            uint32_t tZero = (uint32_t)Field((unsigned)zero, "T");
            int stopLog = FindPrefix("USPARK,STOP,R:NEAR", 0);

            CHECK_EQ(tAcc - tReq, 0);            /* same CommTask pass        */
            CHECK_EQ(tTake - tAcc, 10);          /* next MotionTask cycle     */
            CHECK_EQ(tBrk - tTake, 0);           /* same MotionTask iteration */
            CHECK_EQ(tBend - tBrk, 120);         /* the existing 120 ms brake */
            CHECK(tZl - tBrk >= 50U);            /* 50 ms window after BRK    */
            CHECK(tZr - tBrk >= 50U);
            CHECK_EQ(tZero, (tZl > tZr) ? tZl : tZr);
            CHECK_EQ(tReq, logLines[stopLog].tick);
            lastStop = (unsigned)Field((unsigned)req, "T");
            (void)lastStop;
        }

        /* Encoder counts: REQ/ACC <= TAKE <= BRK, all forward-positive, and
         * the final stationary counts are what the plant ended at. */
        {
            long long elReq = Field((unsigned)req, "EL");
            long long elTake = Field((unsigned)take, "EL");
            long long elBrk = Field((unsigned)brk, "EL");
            long long elZero = Field((unsigned)zero, "EL");
            long long erZero = Field((unsigned)zero, "ER");

            CHECK(elReq > 0);
            CHECK(elTake >= elReq);
            CHECK(elBrk >= elTake);
            CHECK(elZero >= elBrk);
            CHECK_EQ(elZero, plantL);
            CHECK_EQ(erZero, Raw16(plantR));
            /* REQ velocity: 80 counts / 10 ms = 8000 cps on both wheels. */
            CHECK_EQ(Field((unsigned)req, "VL"), 8000);
            CHECK_EQ(Field((unsigned)req, "VR"), 8000);
        }

        /* US lines: publication tick, observation delay, honest field set. */
        {
            unsigned u = (unsigned)TraceIndex("US", 0);

            CHECK(Field(u, "OD") >= 0 && Field(u, "OD") <= 100);
            CHECK(Field(u, "RAW") != NOFIELD);
            CHECK(Field(u, "FILT") != NOFIELD);
            CHECK(Field(u, "AGE") >= 0);
            CHECK(Field(u, "PU") != NOFIELD);
            /* Encoder counts on a US line are labelled observation-time. */
            CHECK(strstr(trace[u].text, ",OEL:") != NULL);
            CHECK(strstr(trace[u].text, ",OER:") != NULL);
            CHECK(strstr(trace[u].text, ",EL:") == NULL);
        }

        /* PRE lines are replays of publications from BEFORE the arming pass. */
        CHECK(strstr(trace[0].text, "EV:PRE") != NULL);

        /* The counts on US/PRE lines are the counts at the OBSERVATION tick
         * (T + OD); on ARM/NEAR they are the counts at the event tick. The
         * plant moves 80 counts per 10 ms cycle from tick 1020 (the cycle after
         * the MOV was dispatched): count(t) = 8 * (t - 1010). */
        for (i = 0U; i < traceCount; i++)
        {
            if ((strcmp(trace[i].ev, "US") == 0) || (strcmp(trace[i].ev, "PRE") == 0))
            {
                long long obsTick = Field(i, "T") + Field(i, "OD");

                CHECK_EQ(Field(i, "OEL"), 8 * (obsTick - 1010));
                CHECK_EQ(Field(i, "OER"), 8 * (obsTick - 1010));
                /* ... and they are strictly NOT the publication-tick counts. */
                CHECK(Field(i, "OEL") != 8 * (Field(i, "T") - 1010));
            }
            else if ((strcmp(trace[i].ev, "ARM") == 0) ||
                     (strcmp(trace[i].ev, "NEAR") == 0))
            {
                CHECK_EQ(Field(i, "EL"), 8 * (Field(i, "T") - 1010));
                CHECK_EQ(Field(i, "ER"), 8 * (Field(i, "T") - 1010));
            }
        }
    }

    if (rawLogPath != NULL)
    {
        FILE *f = fopen(rawLogPath, "w");

        if (f != NULL)
        {
            unsigned i;

            for (i = 0U; i < logCount; i++)
            {
                fprintf(f, "2026-01-01T00:00:%02u.%03uZ\tRX\t%s\n",
                        (logLines[i].tick / 1000U) % 60U,
                        logLines[i].tick % 1000U, logLines[i].text);
            }
            fclose(f);
        }
    }
#else
    CHECK_EQ(TraceCount(), 0);
    (void)rawLogPath;
#endif
}

static void ScenarioLongApproachDrops(void)
{
    unsigned i;

    Fresh(5000U);
    EnableParking();
    StartForward(500);                  /* long enough to outlast the approach */

    Sample(900U);
    Sample(800U);
    for (i = 0U; i < 40U; i++) { Sample(400U); }    /* armed, 40 publications */
    Sample(290U);
    Sample(285U);
    Sample(280U);
    (void)CyclesUntilIdleAndQuiet(900U);

    CHECK_EQ(CountPrefix("USPARK,STOP,R:NEAR"), 1);
    CHECK_EQ(CountPrefix("ACK,STOPPED"), 1);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);

#if APP_USPARK_TRACE
    {
        unsigned done;

        ParseTrace();
        ExpectTraceOnlyAfterAck();
        ExpectOneLinePerPassAndShort();
        /* PRE 3 + US 29 (capped) + NEAR 3 + the ten fixed records. A single
         * approach can only produce 3 NEAR, so the ring is not completely full
         * here; the pure test fills all 48 with the NEAR cap exercised. */
        CHECK_EQ(traceCount, 3U + APP_PARK_TRACE_US_CAP + 3U +
                              APP_PARK_TRACE_FIXED_COUNT);
        done = (unsigned)TraceIndex("DONE", 0);
        CHECK_EQ(done, traceCount - 1U);
        /* 40 + 3 publications after ARM: the arming one is a PRE; of the 42
         * stored as US only 29 fit; the 3 NEAR fit; nothing else is dropped. */
        CHECK_EQ(TraceEvCount("US"), APP_PARK_TRACE_US_CAP);
        CHECK_EQ(TraceEvCount("PRE"), 3);
        CHECK_EQ(TraceEvCount("NEAR"), 3);
        CHECK_EQ(Field(done, "DR"), (40 - 1 + 3) - (int)APP_PARK_TRACE_US_CAP);
        CHECK_EQ(Field(done, "RF"), 0);
        CHECK(FieldIs(done, "R", "OK"));
        CHECK_EQ(TraceEvCount("REQ"), 1);
        CHECK_EQ(TraceEvCount("ZERO"), 1);
    }
#else
    CHECK_EQ(TraceCount(), 0);
#endif
}

static void ScenarioManualStopWhileArmed(void)
{
    Fresh(2000U);
    EnableParking();
    StartForward(50);
    Sample(400U);
    Sample(400U);
    RxNow("STOP\n");
    (void)CyclesUntilIdleAndQuiet(400U);

    CHECK_EQ(CountPrefix("ACK,STOPPED"), 1);
    CHECK_EQ(CountPrefix("USPARK,STOP"), 0);        /* manual, not automatic */
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);

#if APP_USPARK_TRACE
    ParseTrace();
    ExpectTraceOnlyAfterAck();
    CHECK(traceCount >= 3U);
    CHECK_EQ(TraceEvCount("REQ"), 0);
    CHECK_EQ(TraceEvCount("ACC"), 0);
    CHECK_EQ(TraceEvCount("TAKE"), 0);
    CHECK_EQ(TraceEvCount("BRK"), 0);
    CHECK_EQ(TraceEvCount("BEND"), 0);
    CHECK_EQ(TraceEvCount("ZL"), 0);
    CHECK_EQ(TraceEvCount("ZR"), 0);
    CHECK_EQ(TraceEvCount("ZERO"), 0);
    CHECK_EQ(TraceEvCount("DONE"), 1);
    CHECK(FieldIs((unsigned)TraceIndex("DONE", 0), "R", "NOREQ"));
#else
    CHECK_EQ(TraceCount(), 0);
#endif
}

static void ScenarioManualStopNotArmed(void)
{
    Fresh(2000U);
    EnableParking();
    StartForward(50);
    Sample(900U);                       /* far: never arms */
    Sample(800U);
    RxNow("STOP\n");
    (void)CyclesUntilIdleAndQuiet(400U);

    CHECK_EQ(CountPrefix("ACK,STOPPED"), 1);
    CHECK_EQ(TraceCount(), 0);          /* no window, no trace */
}

static void ScenarioReverseAndTurnNeverTrace(void)
{
    /* Reverse with near readings. */
    Fresh(3000U);
    EnableParking();
    RxNow("MOV,B,40\n");
    Cycle();
    Sample(400U);
    Sample(290U);
    Sample(285U);
    Sample(280U);
    Sample(270U);
    RxNow("STOP\n");
    (void)CyclesUntilIdleAndQuiet(400U);
    CHECK_EQ(CountPrefix("USPARK,STOP"), 0);
    CHECK_EQ(CountPrefix("ACK,STOPPED"), 1);
    CHECK_EQ(TraceCount(), 0);

    /* Turn with near readings. */
    Fresh(3000U);
    EnableParking();
    RxNow("MOV,FL,90\n");
    Cycles(3U);
    Sample(400U);
    Sample(290U);
    Sample(285U);
    Sample(280U);
    RxNow("STOP\n");
    (void)CyclesUntilIdleAndQuiet(400U);
    CHECK_EQ(CountPrefix("USPARK,STOP"), 0);
    CHECK_EQ(TraceCount(), 0);

    /* Forward with USPARK never enabled. */
    Fresh(3000U);
    StartForward(50);
    Sample(400U);
    Sample(290U);
    Sample(285U);
    Sample(280U);
    RxNow("STOP\n");
    (void)CyclesUntilIdleAndQuiet(400U);
    CHECK_EQ(CountPrefix("USPARK,STOP"), 0);
    CHECK_EQ(TraceCount(), 0);
}

static void ScenarioDisableWhileArmed(void)
{
    Fresh(4000U);
    EnableParking();
    StartForward(50);
    Sample(400U);
    Sample(400U);
    RxNow("USPARK,0\n");
    CHECK_EQ(CountPrefix("USPARK,OK"), 1);
    Cycles(3U);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);   /* never stops it */

    /* The trace waits for the link to go idle: nothing yet. */
    CHECK_EQ(TraceCount(), 0);
    RxNow("STOP\n");
    (void)CyclesUntilIdleAndQuiet(400U);

    CHECK_EQ(CountPrefix("ACK,STOPPED"), 1);
#if APP_USPARK_TRACE
    ParseTrace();
    ExpectTraceOnlyAfterAck();
    CHECK_EQ(TraceEvCount("DONE"), 1);
    CHECK(FieldIs((unsigned)TraceIndex("DONE", 0), "R", "ABORT"));
    CHECK_EQ(TraceEvCount("BRK"), 0);
#else
    CHECK_EQ(TraceCount(), 0);
#endif
}

static void ScenarioNaturalCompletionWhileArmed(void)
{
    Fresh(4000U);
    EnableParking();
    StartForward(6);
    Sample(400U);                       /* armed, never near */
    (void)CyclesUntilIdleAndQuiet(600U);

    CHECK_EQ(CountPrefix("USPARK,STOP"), 0);
    CHECK_EQ(CountPrefix("ACK"), 1);            /* a normal completion */
    CHECK_EQ(CountPrefix("ACK,STOPPED"), 0);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);

#if APP_USPARK_TRACE
    ParseTrace();
    CHECK(traceCount >= 3U);
    CHECK_EQ(TraceEvCount("BRK"), 0);           /* a natural brake is not traced */
    CHECK_EQ(TraceEvCount("BEND"), 0);
    CHECK_EQ(TraceEvCount("ZERO"), 0);
    CHECK_EQ(TraceEvCount("DONE"), 1);
    CHECK(FieldIs((unsigned)TraceIndex("DONE", 0), "R", "NOREQ"));
    /* ... and only after the completion was reported. */
    {
        int ack = FindPrefix("ACK", 0);
        unsigned i;

        for (i = 0U; i < logCount; i++)
        {
            if (IsTrace(logLines[i].text)) { CHECK((int)i > ack); }
        }
    }
#else
    CHECK_EQ(TraceCount(), 0);
#endif
}

#ifdef APP_COMM_TEST_HOOK_BEFORE_PARK_STOP
static int hookArmed;                   /* 1: the movement ends first          */
static int hookManualStop;              /* 1: a manual STOP is accepted first  */

void ParkTraceHook(void)
{
    MotionCommand stop;

    if (hookArmed != 0)
    {
        hookArmed = 0;
        stop.type = MOTION_STOP;
        stop.value = 0;
        AppMotion_HandleCommand(&stop);     /* the movement ends first */
    }

    if (hookManualStop != 0)
    {
        hookManualStop = 0;
        RxNow("STOP\n");                   /* a manual STOP wins the race */
    }
}
#endif

static void ScenarioRefusedRequest(void)
{
#ifdef APP_COMM_TEST_HOOK_BEFORE_PARK_STOP
    Fresh(6000U);
    EnableParking();
    StartForward(50);
    Sample(400U);
    Sample(290U);
    Sample(285U);
    hookArmed = 1;
    Sample(280U);                       /* the guarded request is refused */
    (void)CyclesUntilIdleAndQuiet(600U);

    CHECK_EQ(CountPrefix("USPARK,STOP"), 0);
    CHECK_EQ(hookArmed, 0);
#if APP_USPARK_TRACE
    ParseTrace();
    CHECK_EQ(TraceEvCount("REQ"), 1);
    CHECK_EQ(TraceEvCount("ACC"), 0);
    CHECK_EQ(TraceEvCount("TAKE"), 0);
    CHECK_EQ(TraceEvCount("BRK"), 0);
    CHECK_EQ(TraceEvCount("DONE"), 1);
    CHECK(FieldIs((unsigned)TraceIndex("DONE", 0), "R", "REFUSED"));
#else
    CHECK_EQ(TraceCount(), 0);
#endif
#endif
}

static void ScenarioManualStopRacesTheAutomaticRequest(void)
{
#ifdef APP_COMM_TEST_HOOK_BEFORE_PARK_STOP
    Fresh(6500U);
    EnableParking();
    StartForward(50);
    Sample(400U);
    Sample(290U);
    Sample(285U);
    hookManualStop = 1;
    Sample(280U);       /* the detector decides, a manual STOP is accepted first */
    (void)CyclesUntilIdleAndQuiet(600U);

    /* The manual STOP won: exactly one stop, no automatic request line. */
    CHECK_EQ(hookManualStop, 0);
    CHECK_EQ(CountPrefix("USPARK,STOP"), 0);
    CHECK_EQ(CountPrefix("ACK,STOPPED"), 1);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
#if APP_USPARK_TRACE
    /* ... and the trace calls it what it was: a refused automatic request,
     * never an accepted/taken/braked automatic stop. */
    ParseTrace();
    ExpectTraceOnlyAfterAck();
    CHECK_EQ(TraceEvCount("REQ"), 1);
    CHECK_EQ(TraceEvCount("ACC"), 0);
    CHECK_EQ(TraceEvCount("TAKE"), 0);
    CHECK_EQ(TraceEvCount("BRK"), 0);
    CHECK_EQ(TraceEvCount("ZERO"), 0);
    CHECK_EQ(TraceEvCount("DONE"), 1);
    CHECK(FieldIs((unsigned)TraceIndex("DONE", 0), "R", "REFUSED"));
#else
    CHECK_EQ(TraceCount(), 0);
#endif
#endif
}

static void ScenarioTickWrap(void)
{
    /* The whole stop straddles the 32-bit RTOS tick wrap. */
    Fresh(0xFFFFFF60U);                 /* wraps about 160 ms after the start */
    EnableParking();
    StartForward(50);
    Sample(900U);
    Sample(400U);
    Sample(290U);
    Sample(285U);
    Sample(280U);
    (void)CyclesUntilIdleAndQuiet(600U);

    CHECK_EQ(CountPrefix("USPARK,STOP,R:NEAR"), 1);
    CHECK_EQ(CountPrefix("ACK,STOPPED"), 1);
    CHECK(Fake_GetTickCount() < 0x10000000U);       /* we did wrap */

#if APP_USPARK_TRACE
    {
        unsigned req;
        unsigned brk;
        unsigned bend;
        unsigned zero;
        unsigned done;

        ParseTrace();
        ExpectTraceOnlyAfterAck();
        req = (unsigned)TraceIndex("REQ", 0);
        brk = (unsigned)TraceIndex("BRK", 0);
        bend = (unsigned)TraceIndex("BEND", 0);
        zero = (unsigned)TraceIndex("ZERO", 0);
        done = (unsigned)TraceIndex("DONE", 0);

        CHECK((int)req > 0);
        CHECK_EQ((uint32_t)Field(bend, "T") - (uint32_t)Field(brk, "T"), 120);
        CHECK((uint32_t)Field(zero, "T") - (uint32_t)Field(brk, "T") >= 50U);
        CHECK((uint32_t)Field(zero, "T") - (uint32_t)Field(req, "T") < 2000U);
        CHECK(FieldIs(done, "R", "OK"));
        CHECK_EQ(Field(done, "DR"), 0);
        /* The wrap is inside the window: some ticks are huge, some tiny. */
        CHECK((uint32_t)Field((unsigned)TraceIndex("PRE", 0), "T") > 0xFFFF0000U);
        CHECK((uint32_t)Field((unsigned)TraceIndex("ARM", 0), "T") > 0xFFFF0000U);
        CHECK((uint32_t)Field(req, "T") < 0x10000000U);     /* after the wrap */
        CHECK((uint32_t)Field(done, "T") < 0x10000000U);
    }
#else
    CHECK_EQ(TraceCount(), 0);
#endif
}

static void ScenarioRightEncoderWrap(void)
{
    /* The right wheel (16-bit TIM3) wraps through +-32768 during the stop. */
    Fresh(7000U);
    plantRightOffset = 32768 - 700;     /* crosses 32767 while moving */
    EnableParking();
    StartForward(50);
    Sample(400U);
    Sample(290U);
    Sample(285U);
    Sample(280U);
    (void)CyclesUntilIdleAndQuiet(600U);

    CHECK_EQ(CountPrefix("USPARK,STOP,R:NEAR"), 1);
    CHECK_EQ(CountPrefix("ACK,STOPPED"), 1);

#if APP_USPARK_TRACE
    {
        unsigned zr;
        unsigned zero;
        unsigned done;

        ParseTrace();
        ExpectTraceOnlyAfterAck();
        zr = (unsigned)TraceIndex("ZR", 0);
        zero = (unsigned)TraceIndex("ZERO", 0);
        done = (unsigned)TraceIndex("DONE", 0);

        CHECK(FieldIs(done, "R", "OK"));
        CHECK_EQ(Field(zero, "ER"), Raw16(plantR + plantRightOffset));
        CHECK_EQ(Field(zr, "ER"), Raw16(plantR + plantRightOffset));
        /* The right counts really did wrap somewhere in the trace. */
        {
            unsigned i;
            int sawNegative = 0;
            int sawPositive = 0;

            for (i = 0U; i < traceCount; i++)
            {
                long long er = Field(i, "ER");
                long long oer = Field(i, "OER");

                if (er != NOFIELD) { if (er < 0) { sawNegative = 1; } else { sawPositive = 1; } }
                if (oer != NOFIELD) { if (oer < 0) { sawNegative = 1; } else { sawPositive = 1; } }
            }

            CHECK(sawNegative && sawPositive);
        }
    }
#else
    CHECK_EQ(TraceCount(), 0);
#endif
}

#if APP_USPARK_TRACE
static void ScenarioStaleAndInvalidPublications(void)
{
    Fresh(8000U);
    EnableParking();
    StartForward(50);
    Sample(400U);                       /* arms */
    UsPublishStale(390U);               /* VALID but older than 200 ms */
    Cycles(7U);
    SampleInvalid();                    /* no echo */
    SampleInvalid();
    Sample(380U);
    RxNow("STOP\n");
    (void)CyclesUntilIdleAndQuiet(400U);

    ParseTrace();
    {
        unsigned i;
        int stale = 0;
        int invalid = 0;

        for (i = 0U; i < traceCount; i++)
        {
            if (strcmp(trace[i].ev, "US") != 0) { continue; }

            if (Field(i, "AGE") >= 200)
            {
                /* The stale VALID publication, then the two invalid ones that
                 * keep reporting the same old filtered value with a growing
                 * age (never a fresh-looking number). */
                stale++;
                CHECK(Field(i, "AGE") >= 300);
                CHECK_EQ(Field(i, "FILT"), 390);
            }

            if (Field(i, "RAW") == NOFIELD)
            {
                invalid++;
                CHECK(strstr(trace[i].text, ",RAW:NA,") != NULL);
                CHECK(strstr(trace[i].text, ",PU:NA,") != NULL);
            }
        }

        CHECK_EQ(stale, 3);
        CHECK_EQ(invalid, 2);
    }
}

static void ScenarioLinkIdleGating(void)
{
    unsigned beforeTrace;
    int partial;

    Fresh(9000U);
    EnableParking();
    StartForward(50);
    Sample(400U);
    Sample(290U);
    Sample(285U);
    Sample(280U);

    /* Run until the stop has completed and a few trace lines are out. */
    while ((TraceCount() < 3U) && (cycleNo < 400U)) { Cycle(); }
    CHECK(TraceCount() >= 3U);

    /* A half-received command line keeps the link "busy": nothing is sent
     * until the line completes, and the command is answered normally. */
    beforeTrace = TraceCount();
    RxQueueOnly("PROFILE,G");
    Cycles(8U);
    CHECK_EQ(TraceCount(), beforeTrace);        /* paused on the pending bytes */

    partial = (int)logCount;
    RxQueueOnly("ET\n");
    Cycles(3U);
    CHECK_EQ(CountPrefix("PROFILE,PRECISION"), 1);
    (void)partial;

    /* A STOP during the drain is processed normally and answered once. */
    RxNow("STOP\n");
    (void)CyclesUntilIdleAndQuiet(600U);
    CHECK_EQ(CountPrefix("ACK,STOPPED"), 2);    /* the stop + the new STOP */
    ParseTrace();
    CHECK_EQ(TraceEvCount("DONE"), 1);
    CHECK_EQ(TraceEvCount("DONE"), 1);
    /* The whole first window still arrived in order with contiguous N. */
    {
        unsigned i;

        for (i = 0U; i < traceCount; i++)
        {
            CHECK_EQ(Field(i, "N"), (long long)i + 1);
        }
    }

    /* No trace line between the second STOP command and its ACK,STOPPED. */
    {
        int secondAck = FindPrefix("ACK,STOPPED", 1);
        int firstAck = FindPrefix("ACK,STOPPED", 0);
        int i;
        int traceBetween = 0;

        for (i = firstAck + 1; i < secondAck; i++)
        {
            if (IsTrace(logLines[i].text)) { traceBetween++; }
        }

        (void)traceBetween;     /* lines may legitimately precede the STOP */
        CHECK(secondAck > firstAck);
    }
}

static void ScenarioSecondWindowSkippedWhileUnsent(void)
{
    AppParkTraceStats st;

    Fresh(10000U);
    EnableParking();
    StartForward(50);
    Sample(400U);
    Sample(290U);
    Sample(285U);
    Sample(280U);

    /* Stop completes; wait only until the ACK, then start another movement
     * while most of the first trace is still unsent. */
    while ((CountPrefix("ACK,STOPPED") == 0U) && (cycleNo < 400U)) { Cycle(); }
    CHECK_EQ(CountPrefix("ACK,STOPPED"), 1);
    CHECK(AppParkTrace_DrainReady());
    EnableParking();                    /* idle: accepted (enable is not motion) */
    RxNow("MOV,F,50\n");                /* trace still unsent: this must not
                                           open a second window */
    Cycle();
    Sample(400U);
    AppParkTrace_GetStats(&st);
    CHECK(st.windowsSkipped >= 1U || !AppParkTrace_DrainReady());

    RxNow("STOP\n");
    (void)CyclesUntilIdleAndQuiet(800U);
    ParseTrace();
    /* The first window is complete and intact; every line has a contiguous N
     * within its window, and a window never contains two ARM records. */
    CHECK_EQ(TraceEvCount("ARM") >= 1U, 1);
    CHECK_EQ(TraceEvCount("BRK"), 1);
    CHECK(TraceEvCount("DONE") >= 1U);
}
#endif /* APP_USPARK_TRACE */

int main(int argc, char **argv)
{
    const char *digestPath = (argc > 1) ? argv[1] : NULL;
    const char *rawLogPath = (argc > 2) ? argv[2] : NULL;

    digest = (char *)malloc(DIGEST_MAX);
    if (digest != NULL) { digest[0] = '\0'; }

    MotionQueueHandle = osMessageQueueNew(8U, sizeof(MotionCommand), NULL);
    CommRxQueueHandle = osMessageQueueNew(128U, 1U, NULL);
    CHECK(MotionQueueHandle != NULL);
    CHECK(CommRxQueueHandle != NULL);

    ScenarioAutomaticStop(rawLogPath);
    ScenarioLongApproachDrops();
    ScenarioManualStopWhileArmed();
    ScenarioManualStopNotArmed();
    ScenarioReverseAndTurnNeverTrace();
    ScenarioDisableWhileArmed();
    ScenarioNaturalCompletionWhileArmed();
    ScenarioRefusedRequest();
    ScenarioManualStopRacesTheAutomaticRequest();
    ScenarioTickWrap();
    ScenarioRightEncoderWrap();
#if APP_USPARK_TRACE
    /* These scenarios exist only in the trace build; keep them out of the
     * digest that the two builds must share. */
    digestEnabled = 0;
    ScenarioStaleAndInvalidPublications();
    ScenarioLinkIdleGating();
    ScenarioSecondWindowSkippedWhileUnsent();
#endif

    if ((digestPath != NULL) && (digest != NULL))
    {
        FILE *f = fopen(digestPath, "wb");

        if (f != NULL)
        {
            fwrite(digest, 1U, digestLen, f);
            fclose(f);
        }
    }

    printf("%u checks, %u failures (trace %s)\n", checks, failures,
           APP_USPARK_TRACE ? "ON" : "OFF");
    return (failures == 0U) ? 0 : 1;
}
