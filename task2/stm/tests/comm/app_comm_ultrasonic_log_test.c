/* Host test for the ultrasonic bench-logging path in the REAL
 * Core/Src/app_comm.c (AppComm_TrySendUltrasonicLog, reached from
 * AppComm_ServiceTelemetry in CommTask).
 *
 * One source, two builds:
 *   - APP_ULTRASONIC_BENCH_LOG=1 : format, 5 Hz limit, NA handling,
 *     STOP-outstanding suppression, ordering next to ACK,STOPPED, worst-case
 *     line length, and that "US..." is still not an incoming command.
 *   - APP_ULTRASONIC_BENCH_LOG=0 (the default) : no "US," line is ever sent
 *     and the snapshot is never even read.
 *
 * Build (from repo root), flag ON:
 *   gcc -std=c11 -Wall -Wextra -DAPP_ULTRASONIC_BENCH_LOG=1 \
 *       -Itests/comm/fakes -Itests/motion/fakes -ICore/Inc \
 *       -o tests/comm/app_comm_ultrasonic_log_test.exe \
 *       tests/comm/app_comm_ultrasonic_log_test.c Core/Src/app_comm.c \
 *       Core/Src/vision_rx.c Core/Src/app_motion.c Core/Src/app_pid.c Core/Src/app_ir_edge.c Core/Src/app_ir_dist.c \
 *       Core/Src/app_us_park.c tests/motion/fakes/hw_stubs.c \
 *       tests/comm/fakes/comm_stubs.c tests/comm/fakes/ultrasonic_stub.c \
 *       tests/comm/fakes/ir_stub.c
 * Flag OFF: same command without the -D.
 */

#include "app_comm.h"
#include "app_motion.h"
#include "app_ultrasonic.h"
#include "cmsis_os.h"
#include "main.h"
#include "test_fakes.h"
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

#if APP_ULTRASONIC_BENCH_LOG
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

#endif /* APP_ULTRASONIC_BENCH_LOG */

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

#if APP_ULTRASONIC_BENCH_LOG
static void CommOnlyCycle(void)
{
    Fake_SetTickCount(Fake_GetTickCount() + 10U);
    CommStep();
}
#endif /* APP_ULTRASONIC_BENCH_LOG */

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


static AppUltrasonicSnapshot GoodSnapshot(uint32_t lastValidTick)
{
    AppUltrasonicSnapshot s = {0};

    s.sequence = 7U;
    s.cycles = 7U;
    s.lastSampleTick = lastValidTick;
    s.lastValidTick = lastValidTick;
    s.result = APP_US_RESULT_VALID;
    s.state = APP_US_STATE_IDLE;
    s.pulseUs = 1166U;
    s.rawMm = 200U;
    s.filteredMm = 200U;
    s.pulseCaptured = 1U;
    s.rawValid = 1U;
    s.filteredValid = 1U;
    s.filterSamples = 1U;
    s.counters.valid = 6U;
    s.counters.noRise = 1U;
    s.counters.noFall = 2U;
    s.counters.tooNear = 3U;
    s.counters.tooFar = 4U;
    s.counters.edgeError = 5U;
    s.counters.lateRise = 6U;
    s.counters.echoStuck = 7U;
    s.counters.stall = 8U;
    s.counters.staleResult = 9U;
    s.counters.spurious = 10U;
    s.counters.staleCompare = 11U;
    return s;
}

#if APP_ULTRASONIC_BENCH_LOG
/* Runs `ms` of CommTask time and returns the first US line seen (if any). */
static int FirstUsLine(unsigned ms, char *out, size_t outSize)
{
    unsigned i;

    for (i = 0U; i < ms / 10U; i++)
    {
        Cycle();
        if (CountPrefix("US,") > 0U)
        {
            (void)Comm_TxLine(0U, out, outSize);
            return 1;
        }
    }
    return 0;
}

static void test_format_and_rate(void)
{
    AppUltrasonicSnapshot s;
    char line[400];
    unsigned i;
    unsigned seen;
    uint32_t lastSeenTick = 0U;
    uint32_t firstTick = 0U;
    unsigned lines = 0U;

    Fresh();
    s = GoodSnapshot(1160U);
    UsStub_Set(&s);

    /* Nothing before 200 ms; exactly one line at 200 ms. */
    for (i = 0U; i < 19U; i++) { Cycle(); }
    CHECK(Fake_GetTickCount() == 1190U);
    CHECK(CountPrefix("US,") == 0U);

    Cycle();                                        /* tick 1200 */
    CHECK(CountPrefix("US,") == 1U);
    (void)Comm_TxLine(0U, line, sizeof(line));
    CHECK_STR(line,
        "US,ST:IDLE,R:OK,PW:1166,RAW:200,FLT:200,V:1,F:1,AGE:40,"
        "SEQ:7,CYC:7,OK:6,NR:1,NF:2,NEAR:3,FAR:4,EDG:5,LATE:6,ES:7,"
        "STL:8,SRS:9,SPU:10,SCP:11,FM:00");

    /* 5 Hz limit over the next 2 s (keep the sample fresh so the content
     * is stable): lines at least 200 ms apart, 10 lines in (1000, 3000]. */
    Fresh();
    UsStub_Set(&s);
    seen = 0U;
    for (i = 0U; i < 200U; i++)
    {
        Cycle();
        if (CountPrefix("US,") > seen)
        {
            seen = CountPrefix("US,");
            if (lines == 0U) { firstTick = Fake_GetTickCount(); }
            else { CHECK((Fake_GetTickCount() - lastSeenTick) >= 200U); }
            lastSeenTick = Fake_GetTickCount();
            lines++;
        }
    }
    CHECK(lines == 10U);
    CHECK(firstTick == 1200U);
    CHECK(Comm_TxLineCount() == 10U);               /* nothing else sent */
}

static void test_not_available_and_stale_fields(void)
{
    AppUltrasonicSnapshot s = {0};
    char line[400];

    /* Never measured: every value field is NA, not 0. */
    Fresh();
    UsStub_Set(&s);
    CHECK(FirstUsLine(400U, line, sizeof(line)));
    CHECK_STR(line,
        "US,ST:IDLE,R:NONE,PW:NA,RAW:NA,FLT:NA,V:0,F:0,AGE:NA,"
        "SEQ:0,CYC:0,OK:0,NR:0,NF:0,NEAR:0,FAR:0,EDG:0,LATE:0,ES:0,"
        "STL:0,SRS:0,SPU:0,SCP:0,FM:00");

    /* Timed out, filtered value kept but 700 ms stale: still shown, but
     * F:0 and V:0 say it must not be trusted. */
    Fresh();
    s = GoodSnapshot(500U);                 /* last valid 700 ms before 1200 */
    s.result = APP_US_RESULT_NO_RISE;
    s.state = APP_US_STATE_WAIT_RISE;
    s.pulseCaptured = 0U;
    s.rawValid = 0U;
    s.pulseUs = 0U;
    s.rawMm = 0U;
    s.filteredMm = 310U;
    UsStub_Set(&s);
    CHECK(FirstUsLine(400U, line, sizeof(line)));
    CHECK_STR(line,
        "US,ST:RISE,R:NORISE,PW:NA,RAW:NA,FLT:310,V:0,F:0,AGE:700,"
        "SEQ:7,CYC:7,OK:6,NR:1,NF:2,NEAR:3,FAR:4,EDG:5,LATE:6,ES:7,"
        "STL:8,SRS:9,SPU:10,SCP:11,FM:00");

    /* A measured pulse that was rejected still reports its width. */
    Fresh();
    s = GoodSnapshot(1160U);
    s.result = APP_US_RESULT_TOO_FAR;
    s.rawValid = 0U;
    s.rawMm = 0U;
    s.pulseUs = 38000U;
    UsStub_Set(&s);
    CHECK(FirstUsLine(400U, line, sizeof(line)));
    CHECK(strstr(line, "R:FAR,PW:38000,RAW:NA,FLT:200,V:0,F:1,") != NULL);

    /* Validity is explicit: a filtered 0 with filteredValid=1 prints 0,
     * a nonzero value with filteredValid=0 prints NA. */
    Fresh();
    s = GoodSnapshot(1160U);
    s.filteredMm = 0U;
    UsStub_Set(&s);
    CHECK(FirstUsLine(400U, line, sizeof(line)));
    CHECK(strstr(line, ",FLT:0,") != NULL);

    Fresh();
    s = GoodSnapshot(1160U);
    s.filteredValid = 0U;
    s.filteredMm = 77U;
    UsStub_Set(&s);
    CHECK(FirstUsLine(400U, line, sizeof(line)));
    CHECK(strstr(line, ",FLT:NA,") != NULL);
    CHECK(strstr(line, ",AGE:NA,") != NULL);
    CHECK(strstr(line, ",F:0,") != NULL);

    /* Driver fault: state, result and the fault mask are visible. */
    Fresh();
    s = (AppUltrasonicSnapshot){0};
    s.sequence = 1U;
    s.cycles = 1U;
    s.result = APP_US_RESULT_FAULT;
    s.state = APP_US_STATE_FAULT;
    s.faultMask = APP_US_FAULT_CLOCK | APP_US_FAULT_NVIC_PRIO;
    UsStub_Set(&s);
    CHECK(FirstUsLine(400U, line, sizeof(line)));
    CHECK(strstr(line, "US,ST:FAULT,R:FAULT,") != NULL);
    CHECK(strstr(line, ",FM:05") != NULL);

    /* UltrasonicTask was never created: exactly what
     * AppUltrasonic_ReportTaskCreateFailure() publishes must be visible. */
    Fresh();
    s = (AppUltrasonicSnapshot){0};
    s.sequence = 1U;
    s.cycles = 1U;
    s.result = APP_US_RESULT_FAULT;
    s.state = APP_US_STATE_FAULT;
    s.faultMask = APP_US_FAULT_TASK;
    UsStub_Set(&s);
    CHECK(FirstUsLine(400U, line, sizeof(line)));
    CHECK_STR(line,
        "US,ST:FAULT,R:FAULT,PW:NA,RAW:NA,FLT:NA,V:0,F:0,AGE:NA,"
        "SEQ:1,CYC:1,OK:0,NR:0,NF:0,NEAR:0,FAR:0,EDG:0,LATE:0,ES:0,"
        "STL:0,SRS:0,SPU:0,SCP:0,FM:80");

    /* Self-test fault bit (above the old 8-bit range). */
    Fresh();
    s.faultMask = APP_US_FAULT_SELFTEST;
    UsStub_Set(&s);
    CHECK(FirstUsLine(400U, line, sizeof(line)));
    CHECK(strstr(line, ",FM:100") != NULL);
}

static void test_worst_case_length(void)
{
    AppUltrasonicSnapshot s = {0};
    char line[400];

    Fresh();
    s.sequence = UINT32_MAX;
    s.cycles = UINT32_MAX;
    s.lastValidTick = 0U;                   /* huge age */
    s.result = APP_US_RESULT_ECHO_STUCK;
    s.state = APP_US_STATE_TRIGGER_HIGH;
    s.pulseUs = UINT16_MAX;
    s.rawMm = UINT16_MAX;
    s.filteredMm = UINT16_MAX;
    s.pulseCaptured = 1U;
    s.rawValid = 1U;
    s.filteredValid = 1U;
    s.faultMask = 0x1FFU;
    s.counters.valid = s.counters.noRise = s.counters.noFall = UINT16_MAX;
    s.counters.tooNear = s.counters.tooFar = s.counters.edgeError = UINT16_MAX;
    s.counters.lateRise = s.counters.echoStuck = s.counters.stall = UINT16_MAX;
    s.counters.staleResult = s.counters.spurious = UINT16_MAX;
    s.counters.staleCompare = UINT16_MAX;
    UsStub_Set(&s);

    CHECK(FirstUsLine(400U, line, sizeof(line)));
    CHECK(strncmp(line, "US,", 3) == 0);
    CHECK(strlen(line) < 256U);             /* fits the shared 256 B buffer */
    CHECK(strstr(line, ",FM:1FF") != NULL);
    printf("worst-case US line length: %u bytes (+newline)\n",
           (unsigned)strlen(line));
}

static void test_stop_interaction(void)
{
    AppUltrasonicSnapshot s = GoodSnapshot(1100U);
    unsigned total;
    unsigned i;
    unsigned ackIndex = 0U;
    unsigned endIndex = 0U;
    int sawAck = 0;
    int sawEnd = 0;
    char line[400];

    /* STOP outstanding (MotionTask not run yet): no US line at all. */
    Fresh();
    UsStub_Set(&s);
    RxNow("STOP\n");
    Comm_TxClear();

    for (i = 0U; i < 60U; i++) { CommOnlyCycle(); }     /* 600 ms */
    CHECK(CountPrefix("US,") == 0U);
    CHECK(CountPrefix("ACK,STOPPED") == 0U);

    /* MotionTask runs: STOPPED is reported, then the log resumes. */
    Cycle();
    CHECK(CountPrefix("ACK,STOPPED") == 1U);
    for (i = 0U; i < 40U; i++) { Cycle(); }
    CHECK(CountPrefix("US,") >= 1U);

    /* STOP in the middle of a movement with a US line due: the MM,EV:END
     * (RESULT:STOPPED) is immediately followed by ACK,STOPPED. */
    Fresh();
    UsStub_Set(&s);
    RxNow("MOV,F,50\n");
    for (i = 0U; i < 17U; i++) { Cycle(); }             /* 170 ms, US due next */
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

static void test_no_new_incoming_command(void)
{
    char line[400];

    Fresh();
    RxNow("US\n");
    CHECK(Comm_TxLineCount() == 1U);
    (void)Comm_TxLine(0U, line, sizeof(line));
    CHECK_STR(line, "ERR,BAD_CMD");

    Comm_TxClear();
    RxNow("US,GET\n");
    (void)Comm_TxLine(0U, line, sizeof(line));
    CHECK_STR(line, "ERR,BAD_CMD");
}

#else /* APP_ULTRASONIC_BENCH_LOG == 0 */

static void test_disabled_build_is_silent(void)
{
    AppUltrasonicSnapshot s = GoodSnapshot(1160U);
    unsigned i;

    Fresh();
    UsStub_Set(&s);

    for (i = 0U; i < 300U; i++) { Cycle(); }            /* 3 s */

    CHECK(CountPrefix("US,") == 0U);
    CHECK(Comm_TxLineCount() == 0U);
    CHECK(UsStub_GetCallCount() == 0U);                 /* never even read */
}

#endif

int main(void)
{
    MotionQueueHandle = osMessageQueueNew(8U, sizeof(MotionCommand), NULL);
    CommRxQueueHandle = osMessageQueueNew(128U, 1U, NULL);
    CHECK(MotionQueueHandle != NULL);
    CHECK(CommRxQueueHandle != NULL);

#if APP_ULTRASONIC_BENCH_LOG
    printf("mode: APP_ULTRASONIC_BENCH_LOG=1\n");
    test_format_and_rate();
    test_not_available_and_stale_fields();
    test_worst_case_length();
    test_stop_interaction();
    test_no_new_incoming_command();
#else
    printf("mode: APP_ULTRASONIC_BENCH_LOG=0\n");
    test_disabled_build_is_silent();
#endif

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
