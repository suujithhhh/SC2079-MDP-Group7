/* Host test for the USD,GET reply (front ultrasonic read, root PROTOCOL.md
 * §4.8) in the REAL Core/Src/app_comm.c and Core/Src/vision_rx.c (CommTask /
 * MotionTask emulation, like the other tests/comm suites).
 *
 * What is proved:
 *   - the exact reply line for every driver state (OK / WARM / STALE / ERR /
 *     FAULT and NEAR / FAR), the freshness limit, AGE conversion and cap, and
 *     the worst-case length;
 *   - every reply matches the frozen Pi grammar of PROTOCOL.md §4.8 (checked
 *     here by a hand-written matcher, no regex library);
 *   - the reply is DEFERRED like IR,GET: never sent from the parser, never
 *     while a STOP is outstanding, never between MM,EV:END and its terminal
 *     reply, never ahead of a terminal reply due in the same pass;
 *   - at most one reply per pass, extra requests beyond 4 owed are discarded
 *     (never answered with an ERR line), no unsolicited USD line ever;
 *   - a client keeping ONE request outstanding never drops one;
 *   - USD,GET never changes motion state;
 *   - with the bench log on, the USD reply goes out ahead of the "US," line.
 *
 * Two builds (from RTOSV/):
 *   gcc -std=c11 -Wall -Wextra -Werror \
 *       -Itests/comm/fakes -Itests/motion/fakes -ICore/Inc \
 *       -o tests/comm/app_comm_usd_get_test.exe \
 *       tests/comm/app_comm_usd_get_test.c Core/Src/app_comm.c \
 *       Core/Src/vision_rx.c Core/Src/app_motion.c Core/Src/app_pid.c Core/Src/app_ir_edge.c Core/Src/app_ir_dist.c \
 *       Core/Src/app_us_park.c tests/motion/fakes/hw_stubs.c \
 *       tests/comm/fakes/comm_stubs.c tests/comm/fakes/ultrasonic_stub.c \
 *       tests/comm/fakes/ir_stub.c
 *   (and the same with -DAPP_ULTRASONIC_BENCH_LOG=1)
 */

#include "app_comm.h"
#include "app_motion.h"
#include "app_ultrasonic.h"
#include "cmsis_os.h"
#include "main.h"
#include "test_fakes.h"
#include "ultrasonic_stub.h"

#include <ctype.h>
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

static AppUltrasonicSnapshot snap;
static int keepFresh;          /* republish the snapshot every pass (a live sensor) */

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
    keepFresh = 0;
}

/* A driver snapshot whose newest publication is VALID with a full median,
 * published `ageMs` before the NEXT service pass (Cycle() advances 10 ticks
 * first; the fake kernel runs at 1000 Hz). */
static AppUltrasonicSnapshot GoodSnapshot(uint32_t ageMs)
{
    AppUltrasonicSnapshot s;
    uint32_t next = Fake_GetTickCount() + 10U;

    memset(&s, 0, sizeof(s));
    s.sequence = 812U;
    s.cycles = 812U;
    s.lastSampleTick = next - ageMs;
    s.lastValidTick = next - ageMs;
    s.result = APP_US_RESULT_VALID;
    s.state = APP_US_STATE_WAIT_RISE;
    s.pulseUs = 2750U;
    s.rawMm = 470U;
    s.filteredMm = 468U;
    s.pulseCaptured = 1U;
    s.rawValid = 1U;
    s.filteredValid = 1U;
    s.filterSamples = APP_US_FILTER_DEPTH;
    return s;
}

static void SetSnapshot(const AppUltrasonicSnapshot *s)
{
    snap = *s;
    UsStub_Set(&snap);
}

static void Cycle(void)
{
    Fake_SetTickCount(Fake_GetTickCount() + 10U);
    if (keepFresh)
    {
        snap.sequence++;
        snap.lastSampleTick = Fake_GetTickCount();
        snap.lastValidTick = snap.lastSampleTick;
        UsStub_Set(&snap);
    }
    MotionStep();
    CommStep();
}

/* --- the frozen Pi grammar (PROTOCOL.md §4.8), hand-matched ------------- */
/* ^USD,S:(0|[1-9][0-9]{0,9}),ST:(OK|WARM|STALE|ERR|FAULT),
 *  D:(NEAR|FAR|NA|[1-9][0-9]{1,3}),AGE:(NA|0|[1-9][0-9]{0,4}),F:([0-9A-F]{2,})$ */
static int TakeLiteral(const char **p, const char *lit)
{
    size_t n = strlen(lit);

    if (strncmp(*p, lit, n) != 0) { return 0; }
    *p += n;
    return 1;
}

static int TakeNumber(const char **p, unsigned minDigits, unsigned maxDigits, int allowZero)
{
    const char *s = *p;
    unsigned n = 0U;

    if (allowZero && s[0] == '0' && !isdigit((unsigned char)s[1])) { *p = s + 1; return 1; }
    if (!isdigit((unsigned char)s[0]) || s[0] == '0') { return 0; }
    while (isdigit((unsigned char)s[n])) { n++; }
    if (n < minDigits || n > maxDigits) { return 0; }
    *p = s + n;
    return 1;
}

static int TakeOneOf(const char **p, const char *const *words, unsigned count)
{
    unsigned i;

    for (i = 0U; i < count; i++)
    {
        size_t n = strlen(words[i]);
        if (strncmp(*p, words[i], n) == 0 && ((*p)[n] == ','))
        {
            *p += n;
            return 1;
        }
    }
    return 0;
}

static int MatchesGrammar(const char *line)
{
    static const char *const states[] = { "OK", "WARM", "STALE", "ERR", "FAULT" };
    static const char *const words[] = { "NEAR", "FAR", "NA" };
    const char *p = line;
    unsigned hex = 0U;

    if (!TakeLiteral(&p, "USD,S:") || !TakeNumber(&p, 1U, 10U, 1)) { return 0; }
    if (!TakeLiteral(&p, ",ST:") || !TakeOneOf(&p, states, 5U)) { return 0; }
    if (!TakeLiteral(&p, ",D:")) { return 0; }
    if (!TakeOneOf(&p, words, 3U) && !TakeNumber(&p, 2U, 4U, 0)) { return 0; }
    if (!TakeLiteral(&p, ",AGE:")) { return 0; }
    if (!TakeLiteral(&p, "NA") && !TakeNumber(&p, 1U, 5U, 1)) { return 0; }
    if (!TakeLiteral(&p, ",F:")) { return 0; }
    while ((p[hex] >= '0' && p[hex] <= '9') || (p[hex] >= 'A' && p[hex] <= 'F')) { hex++; }
    return (hex >= 2U) && (p[hex] == '\0');
}

/* Sends USD,GET, runs one pass and returns the number of "USD," lines (the
 * last one in `out`). */
static int AskOnce(char *out, size_t outSize)
{
    unsigned before = Comm_TxLineCount();
    unsigned total;
    unsigned k;
    int found = 0;

    RxNow("USD,GET\n");
    Cycle();
    total = Comm_TxLineCount();
    for (k = before; k < total; k++)
    {
        char line[400];

        (void)Comm_TxLine(k, line, sizeof(line));
        if (strncmp(line, "USD,", 4) == 0)
        {
            found++;
            (void)snprintf(out, outSize, "%s", line);
        }
    }
    return found;
}

static void ExpectReply(const AppUltrasonicSnapshot *s, const char *expected)
{
    char line[400] = "";

    Fresh();
    SetSnapshot(s);
    CHECK(AskOnce(line, sizeof(line)) == 1);
    CHECK_STR(line, expected);
    CHECK(MatchesGrammar(line));
}

/* Every snapshot below is one app_ultrasonic.c can publish (result, validity
 * flags, filter fill and fault mask consistent); synthetic cases are marked. */
static void test_exact_lines(void)
{
    AppUltrasonicSnapshot s;

    Fresh();
    s = GoodSnapshot(14U);
    ExpectReply(&s, "USD,S:812,ST:OK,D:468,AGE:14,F:00");

    /* Freshness limit: 200 ms is still fresh, 201 ms is STALE (newest result
     * too old; a stale value is never reported as a distance). */
    Fresh();
    s = GoodSnapshot(200U);
    ExpectReply(&s, "USD,S:812,ST:OK,D:468,AGE:200,F:00");
    Fresh();
    s = GoodSnapshot(201U);
    ExpectReply(&s, "USD,S:812,ST:STALE,D:NA,AGE:201,F:40");

    /* Near the bottom of the frozen numeric range: 10 mm is a number, 9 is NEAR. */
    Fresh();
    s = GoodSnapshot(30U);
    s.filteredMm = 10U;
    ExpectReply(&s, "USD,S:812,ST:OK,D:10,AGE:30,F:00");
    s.filteredMm = 9U;
    ExpectReply(&s, "USD,S:812,ST:OK,D:NEAR,AGE:30,F:00");
    s.filteredMm = 4088U;                    /* the driver's longest valid echo */
    ExpectReply(&s, "USD,S:812,ST:OK,D:4088,AGE:30,F:00");

    /* WARM: newest result VALID but the 3-sample median is not full yet. */
    Fresh();
    s = GoodSnapshot(20U);
    s.filterSamples = 2U;
    ExpectReply(&s, "USD,S:812,ST:WARM,D:NA,AGE:20,F:00");

    /* A fresh failed cycle after good ones: the old median is NOT reported. */
    Fresh();
    s = GoodSnapshot(20U);
    s.result = APP_US_RESULT_TOO_NEAR;
    s.rawValid = 0U;
    ExpectReply(&s, "USD,S:812,ST:OK,D:NEAR,AGE:20,F:00");
    s.result = APP_US_RESULT_TOO_FAR;
    ExpectReply(&s, "USD,S:812,ST:OK,D:FAR,AGE:20,F:00");
    s.result = APP_US_RESULT_NO_FALL;        /* HC-SR04 no echo: line held high to timeout */
    ExpectReply(&s, "USD,S:812,ST:OK,D:FAR,AGE:20,F:00");
    s.result = APP_US_RESULT_NO_RISE;        /* the sensor never answered: ERR */
    ExpectReply(&s, "USD,S:812,ST:ERR,D:NA,AGE:20,F:40");
    s.result = APP_US_RESULT_EDGE_ERROR;
    ExpectReply(&s, "USD,S:812,ST:ERR,D:NA,AGE:20,F:40");
    s.result = APP_US_RESULT_LATE_RISE;
    ExpectReply(&s, "USD,S:812,ST:ERR,D:NA,AGE:20,F:40");
    s.result = APP_US_RESULT_ECHO_STUCK;
    ExpectReply(&s, "USD,S:812,ST:ERR,D:NA,AGE:20,F:40");
    s.result = APP_US_RESULT_STALL;
    ExpectReply(&s, "USD,S:812,ST:ERR,D:NA,AGE:20,F:40");

    /* A failed cycle that is itself old is STALE, not ERR. */
    Fresh();
    s = GoodSnapshot(450U);
    s.result = APP_US_RESULT_NO_FALL;
    ExpectReply(&s, "USD,S:812,ST:STALE,D:NA,AGE:450,F:40");

    /* Never published (boot): sequence 0, no age. */
    Fresh();
    memset(&s, 0, sizeof(s));
    s.state = APP_US_STATE_IDLE;
    ExpectReply(&s, "USD,S:0,ST:STALE,D:NA,AGE:NA,F:40");

    /* Driver FAULT (start-up check failed; AppUltrasonic_ReportTaskCreateFailure
     * publishes sequence 1): F carries the mask plus 40, whatever the age. */
    Fresh();
    memset(&s, 0, sizeof(s));
    s.sequence = 1U;
    s.lastSampleTick = Fake_GetTickCount() + 10U - 5U;
    s.state = APP_US_STATE_FAULT;
    s.result = APP_US_RESULT_FAULT;
    s.faultMask = APP_US_FAULT_TASK;
    ExpectReply(&s, "USD,S:1,ST:FAULT,D:NA,AGE:5,F:C0");
    s.faultMask = APP_US_FAULT_SELFTEST | APP_US_FAULT_CLOCK;
    ExpectReply(&s, "USD,S:1,ST:FAULT,D:NA,AGE:5,F:141");
    s.lastSampleTick = Fake_GetTickCount() + 10U - 90000U;
    ExpectReply(&s, "USD,S:1,ST:FAULT,D:NA,AGE:65535,F:141");

    /* AGE cap on a very old publication. */
    Fresh();
    s = GoodSnapshot(65536U);
    ExpectReply(&s, "USD,S:812,ST:STALE,D:NA,AGE:65535,F:40");
}

/* SYNTHETIC: not states the driver produces; formatter robustness only. */
static void test_formatter_limits_synthetic(void)
{
    AppUltrasonicSnapshot s;
    char line[400] = "";

    Fresh();
    s = GoodSnapshot(70000U);
    s.sequence = UINT32_MAX;
    s.faultMask = 0xFFBFU;                   /* | 0x40 = FFFF */
    SetSnapshot(&s);
    CHECK(AskOnce(line, sizeof(line)) == 1);
    CHECK_STR(line, "USD,S:4294967295,ST:FAULT,D:NA,AGE:65535,F:FFFF");
    CHECK(strlen(line) <= 48U);              /* + newline, buffer is 256 */
    CHECK(MatchesGrammar(line));
    printf("synthetic worst-case USD,GET line length: %u bytes (+newline)\n",
           (unsigned)strlen(line));

    /* OK but filteredValid clear (cannot happen with result VALID and a full
     * median): never a number. */
    Fresh();
    s = GoodSnapshot(10U);
    s.filteredValid = 0U;
    ExpectReply(&s, "USD,S:812,ST:WARM,D:NA,AGE:10,F:00");
}

static void test_grammar_matcher_rejects(void)
{
    /* The hand-written matcher itself: forms the frozen grammar forbids. */
    CHECK(!MatchesGrammar("USD,S:01,ST:OK,D:468,AGE:14,F:00"));
    CHECK(!MatchesGrammar("USD,S:1,ST:OK,D:9,AGE:14,F:00"));
    CHECK(!MatchesGrammar("USD,S:1,ST:OK,D:12345,AGE:14,F:00"));
    CHECK(!MatchesGrammar("USD,S:1,ST:BAD,D:NA,AGE:14,F:00"));
    CHECK(!MatchesGrammar("USD,S:1,ST:OK,D:468,AGE:014,F:00"));
    CHECK(!MatchesGrammar("USD,S:1,ST:OK,D:468,AGE:14,F:0"));
    CHECK(!MatchesGrammar("USD,S:1,ST:OK,D:468,AGE:14,F:0g"));
    CHECK(MatchesGrammar("USD,S:0,ST:STALE,D:NA,AGE:NA,F:40"));
    CHECK(MatchesGrammar("USD,S:1,ST:OK,D:FAR,AGE:0,F:00"));
}

static void test_no_unsolicited_and_one_per_request(void)
{
    AppUltrasonicSnapshot s;
    unsigned i;
    char line[400];

    Fresh();
    s = GoodSnapshot(14U);
    SetSnapshot(&s);
    keepFresh = 1;

    for (i = 0U; i < 300U; i++) { Cycle(); }        /* 3 s, no request */
    CHECK(CountPrefix("USD,") == 0U);

    Comm_TxClear();
    RxNow("USD,GET\n");                             /* the parser sends nothing */
    CHECK(Comm_TxLineCount() == 0U);

    Cycle();
    CHECK(CountPrefix("USD,") == 1U);
    for (i = 0U; i < 50U; i++) { Cycle(); }
    CHECK(CountPrefix("USD,") == 1U);               /* exactly one */

    /* Two requests, two replies, one per pass, increasing S. */
    Comm_TxClear();
    RxNow("USD,GET\n");
    RxNow("USD,GET\n");
    CHECK(Comm_TxLineCount() == 0U);
    Cycle();
    CHECK(CountPrefix("USD,") == 1U);
    Cycle();
    CHECK(CountPrefix("USD,") == 2U);
    for (i = 0U; i < 20U; i++) { Cycle(); }
    CHECK(CountPrefix("USD,") == 2U);
    CHECK(AskOnce(line, sizeof(line)) == 1);
    CHECK(MatchesGrammar(line));

    /* IR,GET and USD,GET owed in the same pass: one non-terminal line per pass. */
    Comm_TxClear();
    RxNow("IR,GET\n");
    RxNow("USD,GET\n");
    Cycle();
    CHECK(Comm_TxLineCount() == 1U);
    Cycle();
    CHECK(CountPrefix("USD,") == 1U);
    CHECK(CountPrefix("IR,") == 1U);
}

static void test_flood_is_bounded_and_never_an_error(void)
{
    AppUltrasonicSnapshot s;
    unsigned i;
    unsigned replies = 0U;

    Fresh();
    s = GoodSnapshot(14U);
    SetSnapshot(&s);

    for (i = 0U; i < 12U; i++) { RxNow("USD,GET\n"); }
    CHECK(Comm_TxLineCount() == 0U);

    for (i = 0U; i < 40U; i++)
    {
        unsigned before = CountPrefix("USD,");

        Cycle();
        replies += CountPrefix("USD,") - before;
        CHECK((CountPrefix("USD,") - before) <= 1U);
    }

    CHECK(AppComm_UsdGetDroppedCount() == 8U);
    CHECK(AppComm_UsdGetPendingHighWater() == 4U);
    CHECK(replies == 4U);
    CHECK(CountPrefix("ERR") == 0U);
}

/* One request outstanding at a time, as fast as the protocol allows, across a
 * movement, its terminal reply, a STOP and a rejected second MOV. */
static void RunPollingScenario(const char *name, unsigned stopAt, unsigned secondMovAt)
{
    AppUltrasonicSnapshot s;
    unsigned requests = 0U;
    unsigned replies = 0U;
    unsigned waited = 0U;
    unsigned maxWait = 0U;
    unsigned i;
    int outstanding = 0;

    Fresh();
    s = GoodSnapshot(14U);
    SetSnapshot(&s);
    keepFresh = 1;
    RxNow("MOV,F,50\n");

    for (i = 0U; i < 400U; i++)
    {
        unsigned before = CountPrefix("USD,");

        if (i == stopAt) { RxNow("STOP\n"); }
        if (i == secondMovAt) { RxNow("MOV,F,60\n"); }
        if ((i == 30U) && (stopAt == 0U))
        {
            Fake_SetEncoderLeft(MOV_50CM_TARGET_COUNTS);
            Fake_SetEncoderRight(MOV_50CM_TARGET_COUNTS);
        }
        if (!outstanding)
        {
            RxNow("USD,GET\n");
            outstanding = 1;
            requests++;
            waited = 0U;
        }

        Cycle();

        if (CountPrefix("USD,") > before)
        {
            replies += CountPrefix("USD,") - before;
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
    CHECK(AppComm_UsdGetDroppedCount() == 0U);
    CHECK(AppComm_UsdGetPendingHighWater() == 1U);
    CHECK(maxWait <= ((stopAt != 9999U && stopAt != 0U) ? 20U : 5U));
    printf("polling %-26s %4u requests, %4u replies, dropped %u, longest wait %u passes\n",
           name, requests, replies, (unsigned)AppComm_UsdGetDroppedCount(), maxWait);
}

static void test_one_outstanding_never_drops(void)
{
    RunPollingScenario("idle / movement completes", 0U, 9999U);
    RunPollingScenario("STOP mid-movement", 40U, 9999U);
    RunPollingScenario("STOP at start", 1U, 9999U);
    RunPollingScenario("rejected 2nd MOV", 9999U, 35U);
}

static void test_does_not_change_motion(void)
{
    AppUltrasonicSnapshot s;
    unsigned i;

    Fresh();
    s = GoodSnapshot(14U);
    SetSnapshot(&s);
    keepFresh = 1;

    RxNow("MOV,F,50\n");
    for (i = 0U; i < 25U; i++) { Cycle(); }
    CHECK(AppMotion_GetState() != APP_MOTION_IDLE);

    RxNow("USD,GET\n");
    Cycle();
    CHECK(AppMotion_GetState() != APP_MOTION_IDLE);
    CHECK(CountPrefix("USD,") == 1U);
    CHECK(CountPrefix("ERR") == 0U);

    Fake_SetEncoderLeft(MOV_50CM_TARGET_COUNTS);
    Fake_SetEncoderRight(MOV_50CM_TARGET_COUNTS);
    for (i = 0U; i < 100U; i++) { Cycle(); }
    CHECK(CountPrefix("ACK") == 1U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK(CountPrefix("USD,") == 1U);
}

/* STOP outstanding: the reply waits until after ACK,STOPPED. */
static void test_stop_outstanding(void)
{
    AppUltrasonicSnapshot s;
    unsigned i;
    unsigned k;
    unsigned total;
    unsigned stoppedIndex = 0U;
    unsigned usdIndex = 0U;
    char line[400];

    Fresh();
    s = GoodSnapshot(14U);
    SetSnapshot(&s);
    keepFresh = 1;
    RxNow("STOP\n");
    RxNow("USD,GET\n");
    Comm_TxClear();
    for (i = 0U; i < 30U; i++)
    {
        Fake_SetTickCount(Fake_GetTickCount() + 10U);
        CommStep();                                 /* CommTask only */
    }
    CHECK(CountPrefix("USD,") == 0U);               /* held back */

    Cycle();                                        /* MotionTask publishes STOPPED */
    CHECK(CountPrefix("ACK,STOPPED") == 1U);
    CHECK(CountPrefix("USD,") == 0U);               /* not in the same pass */
    Cycle();
    CHECK(CountPrefix("USD,") == 1U);

    total = Comm_TxLineCount();
    for (k = 0U; k < total; k++)
    {
        (void)Comm_TxLine(k, line, sizeof(line));
        if (strcmp(line, "ACK,STOPPED") == 0) { stoppedIndex = k + 1U; }
        if (strncmp(line, "USD,", 4) == 0) { usdIndex = k + 1U; }
    }
    CHECK(stoppedIndex != 0U && usdIndex > stoppedIndex);
}

/* A request becoming owed in the very pass in which MotionTask publishes the
 * terminal event: the terminal reply goes first, right after MM,EV:END; the
 * USD reply comes in the next pass. */
static void RunTerminalTrial(unsigned offset, int useStop, unsigned *usdAfter)
{
    AppMotionEvent before;
    AppMotionEvent after;
    AppUltrasonicSnapshot s;
    const char *terminal = useStop ? "ACK,STOPPED" : "ACK";
    char line[400];
    unsigned i;
    unsigned k;
    unsigned total;
    unsigned endIndex = 0U;
    unsigned ackIndex = 0U;

    Fresh();
    s = GoodSnapshot(14U);
    SetSnapshot(&s);
    keepFresh = 1;
    for (i = 0U; i < offset; i++) { Cycle(); }
    RxNow("MOV,F,50\n");

    for (i = 0U; i < 400U; i++)
    {
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
        snap.sequence++;
        snap.lastSampleTick = Fake_GetTickCount();
        snap.lastValidTick = snap.lastSampleTick;
        UsStub_Set(&snap);

        AppMotion_GetEvent(&before);
        MotionStep();
        AppMotion_GetEvent(&after);

        if (after.sequence != before.sequence)
        {
            unsigned linesBefore;
            int foundTerminal = 0;
            int usdInPass = 0;

            RxNow("USD,GET\n");
            linesBefore = Comm_TxLineCount();
            CommStep();
            total = Comm_TxLineCount();
            for (k = linesBefore; k < total; k++)
            {
                (void)Comm_TxLine(k, line, sizeof(line));
                if (strcmp(line, terminal) == 0) { foundTerminal = 1; }
                if (strncmp(line, "USD,", 4) == 0) { usdInPass = 1; }
            }
            CHECK(foundTerminal);
            CHECK(!usdInPass);

            linesBefore = Comm_TxLineCount();
            Cycle();
            total = Comm_TxLineCount();
            {
                unsigned usdNow = 0U;

                for (k = linesBefore; k < total; k++)
                {
                    (void)Comm_TxLine(k, line, sizeof(line));
                    if (strncmp(line, "USD,", 4) == 0) { usdNow++; }
                }
                CHECK(usdNow == 1U);
                if (usdNow == 1U) { (*usdAfter)++; }
            }
            break;
        }

        CommStep();
    }

    total = Comm_TxLineCount();
    for (k = 0U; k < total; k++)
    {
        (void)Comm_TxLine(k, line, sizeof(line));
        if (strncmp(line, "MM,EV:END", 9) == 0) { endIndex = k + 1U; }
        if (strcmp(line, terminal) == 0) { ackIndex = k + 1U; }
    }
    CHECK(endIndex != 0U && ackIndex == endIndex + 1U);   /* END immediately before the reply */
    CHECK(CountPrefix(terminal) == 1U);
    CHECK(CountPrefix("USD,") == 1U);
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
    printf("terminal-reply trials: USD reply after ACK,STOPPED %u/20, after ACK %u/20\n",
           afterStop, afterAck);
}

#if APP_ULTRASONIC_BENCH_LOG
/* With the bench log on, the USD reply takes the non-terminal slot first and
 * the US, line still runs; never two non-terminal lines in one pass. */
static void test_before_bench_line(void)
{
    AppUltrasonicSnapshot s;
    unsigned i;
    unsigned requests = 0U;
    unsigned usd = 0U;
    unsigned bench = 0U;
    char line[400];

    Fresh();
    s = GoodSnapshot(14U);
    SetSnapshot(&s);
    keepFresh = 1;

    for (i = 0U; i < 400U; i++)
    {
        unsigned before = Comm_TxLineCount();
        unsigned after;
        unsigned k;
        unsigned nonTerminal = 0U;

        if ((i % 5U) == 0U) { RxNow("USD,GET\n"); requests++; }
        Cycle();
        after = Comm_TxLineCount();
        for (k = before; k < after; k++)
        {
            (void)Comm_TxLine(k, line, sizeof(line));
            if (strncmp(line, "USD,", 4) == 0) { usd++; nonTerminal++; }
            else if (strncmp(line, "US,", 3) == 0) { bench++; nonTerminal++; }
        }
        CHECK(nonTerminal <= 1U);
    }
    CHECK(usd == requests);
    CHECK(bench >= 15U);
    CHECK(CountPrefix("ERR") == 0U);
}
#endif

int main(void)
{
    MotionQueueHandle = osMessageQueueNew(8U, sizeof(MotionCommand), NULL);
    CommRxQueueHandle = osMessageQueueNew(128U, 1U, NULL);
    CHECK(MotionQueueHandle != NULL);
    CHECK(CommRxQueueHandle != NULL);

    printf("mode: APP_ULTRASONIC_BENCH_LOG=%d\n", APP_ULTRASONIC_BENCH_LOG);

    test_grammar_matcher_rejects();
    test_exact_lines();
    test_formatter_limits_synthetic();
    test_no_unsolicited_and_one_per_request();
    test_flood_is_bounded_and_never_an_error();
    test_one_outstanding_never_drops();
    test_does_not_change_motion();
    test_stop_outstanding();
    test_terminal_reply_never_delayed();
#if APP_ULTRASONIC_BENCH_LOG
    test_before_bench_line();
#endif

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
