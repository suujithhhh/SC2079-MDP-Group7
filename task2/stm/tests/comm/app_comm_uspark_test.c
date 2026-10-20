/* Full-stack host test of the ultrasonic parking protection (USPARK) in the
 * REAL Core/Src/app_comm.c, driving the REAL app_motion.c, vision_rx.c and
 * app_us_park.c against the host fakes (tests/motion/fakes, tests/comm/fakes)
 * and an injectable ultrasonic snapshot (tests/comm/fakes/ultrasonic_stub.c).
 *
 * Like app_comm_stop_test.c it plays both RTOS roles deterministically:
 * Cycle() advances the fake clock 10 ms and runs MotionTask's loop body then
 * CommTask's. Ultrasonic publications are injected between cycles; a real
 * sensor publishes about every 70 ms, so each publication is followed by
 * seven cycles.
 *
 * The automatic stop must travel through the SAME priority STOP machinery as
 * the manual STOP command, so the expected wire sequence is pinned exactly:
 *   MM,EV:START ... USPARK,STOP,R:<cause>,... MM,EV:END,...,RESULT:STOPPED
 *   ACK,STOPPED
 *
 * app_comm.c is compiled with
 *   -DAPP_COMM_TEST_HOOK_BEFORE_PARK_STOP=UsParkTestHook
 * which calls a test function between the detector's decision and the STOP
 * request (never defined in firmware), to exercise the guard that refuses a
 * request for a movement that has meanwhile finished or been replaced.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -Itests/comm/fakes -Itests/motion/fakes \
 *       -ICore/Inc -DAPP_COMM_TEST_HOOK_BEFORE_PARK_STOP=UsParkTestHook \
 *       -o tests/comm/app_comm_uspark_test.exe \
 *       tests/comm/app_comm_uspark_test.c Core/Src/app_comm.c \
 *       Core/Src/vision_rx.c Core/Src/app_motion.c Core/Src/app_pid.c Core/Src/app_ir_edge.c Core/Src/app_ir_dist.c \
 *       Core/Src/app_us_park.c tests/motion/fakes/hw_stubs.c \
 *       tests/comm/fakes/comm_stubs.c tests/comm/fakes/ultrasonic_stub.c \
 *       tests/comm/fakes/ir_stub.c
 */

#include "app_comm.h"
#include "app_motion.h"
#include "app_servo.h"
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

#define MOV_50CM_TARGET_COUNTS 3850

/* --- glob + capture helpers ----------------------------------------- */

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

#define LINE_MAX_LEN 160U

static unsigned CountMatching(const char *pattern)
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

/* The lines that define the stop contract, in order. */
static const char *const interesting[] = {
    "MM,EV:", "USPARK,", "ACK", "ERR,", NULL
};

static int IsInteresting(const char *line)
{
    unsigned i;

    for (i = 0U; interesting[i] != NULL; i++)
    {
        if (strncmp(line, interesting[i], strlen(interesting[i])) == 0)
        {
            return 1;
        }
    }
    return 0;
}

#define KEPT_MAX 64U
static char kept[KEPT_MAX][LINE_MAX_LEN];
static unsigned keptCount;

static void Collect(void)
{
    unsigned total = Comm_TxLineCount();
    unsigned i;
    char line[LINE_MAX_LEN];

    keptCount = 0U;
    for (i = 0U; i < total; i++)
    {
        (void)Comm_TxLine(i, line, sizeof(line));
        if (IsInteresting(line) && (keptCount < KEPT_MAX))
        {
            strcpy(kept[keptCount++], line);
        }
    }
}

static void Dump(void)
{
    unsigned i;

    Collect();
    for (i = 0U; i < keptCount; i++) { printf("    [%u] %s\n", i, kept[i]); }
}

/* The contract lines must be exactly these patterns, in order. */
static void ExpectSequence(int lineNo, unsigned n, const char *const *patterns)
{
    unsigned i;

    Collect();
    checks++;
    if (keptCount != n)
    {
        failures++;
        printf("FAIL line %d: expected %u contract lines, got %u\n",
               lineNo, n, keptCount);
        Dump();
        return;
    }

    for (i = 0U; i < n; i++)
    {
        checks++;
        if (!Glob(patterns[i], kept[i]))
        {
            failures++;
            printf("FAIL line %d: line %u expected '%s' got '%s'\n",
                   lineNo, i, patterns[i], kept[i]);
        }
    }
}

#define EXPECT_SEQ(...) do { \
    const char *const p_[] = { __VA_ARGS__ }; \
    ExpectSequence(__LINE__, (unsigned)(sizeof(p_) / sizeof(p_[0])), p_); \
} while (0)

/* --- firmware-role emulation ----------------------------------------- */

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
    /* Mirrors StartCommTask() in main.c: parse, telemetry, then parking. */
    DrainRx();
    AppComm_ServiceTelemetry();
    AppComm_ServiceParking();
}

static void Cycle(void)
{
    Fake_SetTickCount(Fake_GetTickCount() + 10U);
    MotionStep();
    CommStep();
}

static void CommOnlyCycle(void)
{
    Fake_SetTickCount(Fake_GetTickCount() + 10U);
    CommStep();
}

static void Cycles(unsigned n)
{
    unsigned i;

    for (i = 0U; i < n; i++) { Cycle(); }
}

/* --- ultrasonic injection --------------------------------------------- */

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
    usSnap.rawMm = mm;
    usSnap.lastValidTick = Fake_GetTickCount();
    usSnap.lastSampleTick = Fake_GetTickCount();
    usSnap.state = APP_US_STATE_IDLE;
    usSnap.faultMask = 0U;
    UsStub_Set(&usSnap);
}

static void UsPublishInvalid(void)
{
    usSeq++;
    usSnap.sequence = usSeq;
    usSnap.cycles = usSeq;
    usSnap.result = APP_US_RESULT_NO_RISE;
    usSnap.rawValid = 0U;
    usSnap.lastSampleTick = Fake_GetTickCount();
    usSnap.state = APP_US_STATE_IDLE;
    usSnap.faultMask = 0U;
    UsStub_Set(&usSnap);
}

static void UsSetFault(uint16_t mask)
{
    usSnap.faultMask = mask;
    usSnap.state = APP_US_STATE_FAULT;
    UsStub_Set(&usSnap);
}

/* One publication, then the ~70 ms until the next (7 cycles). */
static void Sample(uint16_t mm)
{
    UsPublishValid(mm);
    Cycles(7U);
}

static void SampleInvalid(void)
{
    UsPublishInvalid();
    Cycles(7U);
}

/* Same, but only CommTask runs (MotionTask has not seen a STOP yet). */
static void SampleCommOnly(uint16_t mm)
{
    unsigned i;

    UsPublishValid(mm);
    for (i = 0U; i < 7U; i++) { CommOnlyCycle(); }
}

/* --- scenario helpers -------------------------------------------------- */

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
    UsReset();
    Fake_ResetMotorCallCounts();
    Comm_TxClear();
}

static void EnableParking(void)
{
    RxNow("USPARK,1\n");
    CHECK(CountMatching("USPARK,OK") == 1U);
    Comm_TxClear();
}

static void StartForward(void)
{
    RxNow("MOV,F,50\n");
    Cycle();
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    CHECK(CountMatching("MM,EV:START,D:F,*") == 1U);
}

/* Arms the final approach: a valid reading inside the 500 mm arming range
 * but above the 300 mm STOP threshold, so it arms without counting as NEAR. */
static void ArmApproach(void)
{
    Sample(400U);
}

/* Last line of a given GET response. */
static void ExpectGet(const char *pattern)
{
    Comm_TxClear();
    RxNow("USPARK,GET\n");
    CHECK(Comm_TxLineCount() == 1U);
    if (CountMatching(pattern) != 1U)
    {
        char line[LINE_MAX_LEN];

        (void)Comm_TxLine(0U, line, sizeof(line));
        printf("    GET gave '%s', expected '%s'\n", line, pattern);
    }
    CHECK(CountMatching(pattern) == 1U);
}

/* The test seam called between decision and request (see the header). */
typedef enum
{
    HOOK_NONE = 0,
    HOOK_COMPLETE_MOVEMENT,     /* movement finishes and brakes */
    HOOK_REPLACE_MOVEMENT       /* movement finishes and a NEW one starts */
} HookAction;

static HookAction hookAction;
static unsigned hookCalls;

void UsParkTestHook(void)
{
    hookCalls++;

    if (hookAction == HOOK_NONE)
    {
        return;
    }

    /* The movement reaches its target, MotionTask completes it. */
    Fake_SetEncoderLeft(MOV_50CM_TARGET_COUNTS);
    Fake_SetEncoderRight(MOV_50CM_TARGET_COUNTS);
    Fake_SetTickCount(Fake_GetTickCount() + 10U);
    MotionStep();

    if (hookAction == HOOK_REPLACE_MOVEMENT)
    {
        MotionCommand next = { MOTION_FORWARD, 50 };

        /* Run out the brake, return to idle, start a different movement. */
        Fake_SetTickCount(Fake_GetTickCount() + 300U);
        MotionStep();
        Fake_SetEncoderLeft(0);
        Fake_SetEncoderRight(0);
        AppMotion_HandleCommand(&next);
    }

    hookAction = HOOK_NONE;
}

/* ---------------------------------------------------------------------- */

#define TX_COPY_MAX 16384U

static char runText[3][TX_COPY_MAX];

/* One fixed MOV,F,50 scenario with USPARK never enabled. */
static void RunDisabledScenario(unsigned slot, int variant)
{
    Fresh();

    if (variant == 1)           /* very near, valid readings throughout */
    {
        usSnap.filteredValid = 1U;
    }

    RxNow("MOV,F,50\n");
    Cycles(5U);

    {
        unsigned i;

        for (i = 0U; i < 12U; i++)
        {
            if (variant == 1) { UsPublishValid(80U); }
            if (variant == 2) { UsSetFault(0xFFU); }
            Cycles(7U);
        }
    }

    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);

    Fake_SetEncoderLeft(MOV_50CM_TARGET_COUNTS);
    Fake_SetEncoderRight(MOV_50CM_TARGET_COUNTS);
    Cycles(30U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);

    strncpy(runText[slot], Comm_TxText(), TX_COPY_MAX - 1U);
    runText[slot][TX_COPY_MAX - 1U] = '\0';
}

static void test_disabled_is_task1_unchanged(void)
{
    RunDisabledScenario(0U, 0);         /* ultrasonic never published */
    RunDisabledScenario(1U, 1);         /* object 80 mm away, valid */
    RunDisabledScenario(2U, 2);         /* ultrasonic faulted */

    CHECK(strlen(runText[0]) > 100U);
    CHECK(strstr(runText[0], "ACK\n") != NULL);
    CHECK(strstr(runText[0], "USPARK") == NULL);
    CHECK(strstr(runText[0], "ACK,STOPPED") == NULL);

    /* Byte-for-byte identical wire output regardless of ultrasonic data. */
    CHECK(strcmp(runText[0], runText[1]) == 0);
    CHECK(strcmp(runText[0], runText[2]) == 0);
}

/* Cost and cadence: while disabled the ultrasonic snapshot is never even
 * read; while enabled the evaluation is limited to the 100 Hz service rate
 * no matter how often CommTask's loop spins (it runs once per received byte
 * as well as on its 10 ms timeout). */
static void test_service_cadence_and_disabled_cost(void)
{
    unsigned before;
    unsigned i;

    Fresh();
    before = UsStub_GetCallCount();

    for (i = 0U; i < 200U; i++)
    {
        Fake_SetTickCount(Fake_GetTickCount() + 10U);
        AppComm_ServiceParking();
    }

    CHECK(UsStub_GetCallCount() == before);         /* disabled: never read */

    EnableParking();
    before = UsStub_GetCallCount();

    Fake_SetTickCount(Fake_GetTickCount() + 10U);
    for (i = 0U; i < 50U; i++)
    {
        AppComm_ServiceParking();                   /* same instant, 50 calls */
    }

    CHECK(UsStub_GetCallCount() == before + 1U);    /* evaluated exactly once */

    Fake_SetTickCount(Fake_GetTickCount() + 9U);
    AppComm_ServiceParking();
    CHECK(UsStub_GetCallCount() == before + 1U);    /* 9 ms later: still gated */

    Fake_SetTickCount(Fake_GetTickCount() + 1U);
    AppComm_ServiceParking();
    CHECK(UsStub_GetCallCount() == before + 2U);    /* 10 ms: next evaluation */
}

static void test_command_behaviour(void)
{
    Fresh();

    ExpectGet("USPARK,E:0,ST:DISABLED,CN:0,FM:00,TH:300,RA:320,AR:500");

    /* Enable while idle. */
    Comm_TxClear();
    RxNow("USPARK,1\n");
    CHECK(CountMatching("USPARK,OK") == 1U);
    CHECK(Comm_TxLineCount() == 1U);
    ExpectGet("USPARK,E:1,ST:UNARMED,CN:0,FM:00,TH:300,RA:320,AR:500");

    /* Disable. */
    Comm_TxClear();
    RxNow("USPARK,0\n");
    CHECK(CountMatching("USPARK,OK") == 1U);
    ExpectGet("USPARK,E:0,ST:DISABLED,*");

    /* A faulted sensor rejects enable. */
    UsSetFault(0x05U);
    Comm_TxClear();
    RxNow("USPARK,1\n");
    CHECK(Comm_TxLineCount() == 1U);
    CHECK(CountMatching("ERR,US_FAULT") == 1U);
    ExpectGet("USPARK,E:0,ST:DISABLED,CN:0,FM:05,*");
    UsReset();

    /* Enable is idle-only: queued, moving, braking, STOP outstanding. */
    Fresh();
    RxNow("MOV,F,50\n");                        /* queued, not yet started */
    Comm_TxClear();
    RxNow("USPARK,1\n");
    CHECK(CountMatching("ERR,BUSY") == 1U);
    ExpectGet("USPARK,E:0,*");

    Cycles(3U);                                 /* now moving */
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    Comm_TxClear();
    RxNow("USPARK,1\n");
    CHECK(CountMatching("ERR,BUSY") == 1U);
    ExpectGet("USPARK,E:0,*");

    RxNow("STOP\n");                            /* STOP outstanding */
    Comm_TxClear();
    RxNow("USPARK,1\n");
    CHECK(CountMatching("ERR,BUSY") == 1U);
    ExpectGet("USPARK,E:0,*");                  /* GET still answers */

    Cycle();                                    /* brake begins */
    CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
    Comm_TxClear();
    RxNow("USPARK,1\n");
    CHECK(CountMatching("ERR,BUSY") == 1U);

    Cycles(30U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    Comm_TxClear();
    RxNow("USPARK,1\n");                        /* idle again: accepted */
    CHECK(CountMatching("USPARK,OK") == 1U);

    /* Malformed forms never change the setting. */
    Comm_TxClear();
    RxNow("USPARK,2\n");
    RxNow("USPARK,1,0\n");
    RxNow("uspark,0\n");
    CHECK(CountMatching("ERR,BAD_CMD") == 3U);
    ExpectGet("USPARK,E:1,*");
}

static void test_automatic_stop_wire_sequence(void)
{
    unsigned stopsBefore;

    Fresh();
    EnableParking();
    StartForward();
    ArmApproach();
    CHECK(CountMatching("USPARK,STOP*") == 0U);

    Sample(150U);
    Sample(150U);
    CHECK(CountMatching("USPARK,STOP*") == 0U);     /* two are not enough */
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);

    stopsBefore = Fake_GetMotorBrakeCallCount();
    Sample(150U);                                   /* the third */

    Cycles(30U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK(Fake_GetMotorBrakeCallCount() > stopsBefore);

    /* Exactly this, in this order. */
    EXPECT_SEQ(
        "MM,EV:START,D:F,*",
        "USPARK,STOP,R:NEAR,D:150,V:*,FM:00",
        "MM,EV:END,D:F,T:*,RESULT:STOPPED",
        "ACK,STOPPED"
    );

    /* confirmedNear persists; the movement itself is over. */
    ExpectGet("USPARK,E:1,ST:UNARMED,CN:1,*");

    /* Further near readings with the robot idle change nothing. */
    Comm_TxClear();
    Sample(100U);
    Sample(100U);
    Sample(100U);
    Sample(100U);
    CHECK(Comm_TxLineCount() == 0U ||
          CountMatching("USPARK,STOP*") == 0U);
}

static void test_threshold_boundaries_on_the_wire(void)
{
    Fresh();
    EnableParking();
    StartForward();
    Sample(400U);                                   /* armed, not near */

    Sample(301U);
    Sample(301U);
    Sample(301U);
    Sample(301U);
    CHECK(CountMatching("USPARK,STOP*") == 0U);     /* 301 never counts */
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);

    Sample(300U);
    Sample(300U);
    CHECK(CountMatching("USPARK,STOP*") == 0U);
    Sample(301U);                                   /* resets */
    Sample(300U);
    Sample(300U);
    CHECK(CountMatching("USPARK,STOP*") == 0U);
    Sample(300U);                                   /* third in a row */
    CHECK(CountMatching("USPARK,STOP,R:NEAR,D:300,*") == 1U);

    Cycles(30U);
    CHECK(CountMatching("ACK,STOPPED") == 1U);
}

/* An armed approach hovering around 400 mm for a long time keeps the robot
 * moving: no automatic STOP, no brake, GET reports ARMED. */
static void test_armed_400_does_not_stop(void)
{
    unsigned i;
    unsigned brakesBefore;

    Fresh();
    EnableParking();
    StartForward();
    brakesBefore = Fake_GetMotorBrakeCallCount();

    for (i = 0U; i < 12U; i++)
    {
        Sample((i & 1U) ? 410U : 400U);
    }

    CHECK(CountMatching("USPARK,STOP*") == 0U);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    CHECK(Fake_GetMotorBrakeCallCount() == brakesBefore);
    ExpectGet("USPARK,E:1,ST:ARMED,CN:0,FM:00,TH:300,RA:320,AR:500");
}

static void test_polling_one_publication_does_not_trigger(void)
{
    Fresh();
    EnableParking();
    StartForward();
    ArmApproach();

    UsPublishValid(150U);
    Cycles(20U);                                    /* many polls, 1 sample, < 250 ms */
    CHECK(CountMatching("USPARK,STOP*") == 0U);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
}

static void test_alternating_invalid_does_not_trigger(void)
{
    Fresh();
    EnableParking();
    StartForward();
    ArmApproach();

    Sample(190U);
    SampleInvalid();
    Sample(190U);
    SampleInvalid();
    Sample(190U);
    CHECK(CountMatching("USPARK,STOP*") == 0U);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
}

static void test_unarmed_noecho_never_stops(void)
{
    unsigned i;

    Fresh();
    EnableParking();
    StartForward();

    for (i = 0U; i < 20U; i++) { SampleInvalid(); }
    Cycles(100U);                                   /* long silence too */
    Sample(600U);
    Sample(501U);
    for (i = 0U; i < 10U; i++) { SampleInvalid(); }

    CHECK(CountMatching("USPARK,STOP*") == 0U);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
}

static void test_armed_failsafes(void)
{
    /* Three consecutive invalid publications. */
    Fresh();
    EnableParking();
    StartForward();
    ArmApproach();
    SampleInvalid();
    SampleInvalid();
    CHECK(CountMatching("USPARK,STOP*") == 0U);
    SampleInvalid();
    CHECK(CountMatching("USPARK,STOP,R:INVALID,*") == 1U);
    Cycles(30U);
    CHECK(CountMatching("ACK,STOPPED") == 1U);

    /* No new publication for more than 250 ms. */
    Fresh();
    EnableParking();
    StartForward();
    ArmApproach();
    /* ArmApproach() left 60 ms since the last publication: 19 more cycles
     * is exactly 250 ms (not enough), the next one is 260 ms (trigger). */
    Cycles(19U);
    CHECK(CountMatching("USPARK,STOP*") == 0U);
    Cycles(1U);
    CHECK(CountMatching("USPARK,STOP,R:NOUPDATE,*") == 1U);
    Cycles(30U);
    CHECK(CountMatching("ACK,STOPPED") == 1U);
    CHECK(CountMatching("USPARK,STOP*") == 1U);

    /* Hardware fault: notified once, one guarded STOP, even unarmed. */
    Fresh();
    EnableParking();
    StartForward();
    UsSetFault(APP_US_FAULT_CLOCK);
    Cycles(60U);
    CHECK(CountMatching("USPARK,FAULT,FM:01") == 1U);
    CHECK(CountMatching("USPARK,STOP,R:FAULT,*") == 1U);
    Cycles(30U);
    CHECK(CountMatching("ACK,STOPPED") == 1U);
    CHECK(CountMatching("USPARK,FAULT*") == 1U);    /* not repeated */
    EXPECT_SEQ(
        "MM,EV:START,D:F,*",
        "USPARK,FAULT,FM:01",
        "USPARK,STOP,R:FAULT,D:NA,V:*,FM:01",
        "MM,EV:END,D:F,T:*,RESULT:STOPPED",
        "ACK,STOPPED"
    );
}

static void test_forward_only(void)
{
    const char *moves[] = {
        "MOV,B,50\n", "MOV,FL,90\n", "MOV,FR,90\n", "MOV,BL,90\n", "MOV,BR,90\n"
    };
    unsigned i;

    for (i = 0U; i < 5U; i++)
    {
        unsigned k;

        Fresh();
        EnableParking();
        RxNow(moves[i]);
        Cycles(3U);
        CHECK(AppMotion_GetState() != APP_MOTION_IDLE);
        CHECK(!(AppMotion_GetState() == APP_MOTION_DISTANCE && i == 99U));

        for (k = 0U; k < 10U; k++) { Sample(100U); }
        for (k = 0U; k < 6U; k++) { SampleInvalid(); }
        Cycles(40U);

        CHECK(CountMatching("USPARK,STOP*") == 0U);
        CHECK(CountMatching("ACK,STOPPED") == 0U);
        ExpectGet("USPARK,E:1,ST:UNARMED,CN:0,*");
    }

    /* Idle with USPARK enabled: nothing at all. */
    Fresh();
    EnableParking();

    {
        unsigned k;

        for (k = 0U; k < 8U; k++) { Sample(100U); }
    }

    CHECK(Comm_TxLineCount() == 0U);
}

static void test_one_stop_per_movement(void)
{
    Fresh();
    EnableParking();
    StartForward();
    ArmApproach();
    Sample(150U);
    Sample(150U);
    Sample(150U);                                   /* request */

    /* Many more near samples while the stop is outstanding / braking. */
    {
        unsigned k;

        for (k = 0U; k < 10U; k++) { Sample(100U); }
    }

    Cycles(30U);
    CHECK(CountMatching("USPARK,STOP*") == 1U);
    CHECK(CountMatching("ACK,STOPPED") == 1U);

    /* A second forward movement is a new instance and can stop again. */
    Comm_TxClear();
    StartForward();
    ExpectGet("USPARK,E:1,ST:ARMED,CN:1,*");        /* armed from confirmedNear */
    Sample(150U);
    Sample(150U);
    Sample(150U);
    Cycles(30U);
    CHECK(CountMatching("USPARK,STOP*") == 1U);     /* this movement's one */
    CHECK(CountMatching("ACK,STOPPED") == 1U);
}

static void test_stop_in_progress_blocks_commands_like_manual(void)
{
    Fresh();
    EnableParking();
    StartForward();
    ArmApproach();
    Sample(150U);
    Sample(150U);
    UsPublishValid(150U);
    CommOnlyCycle();                                /* request raised here */
    CHECK(CountMatching("USPARK,STOP*") == 1U);

    /* Same admission rules as a manual STOP: busy until ACK,STOPPED. */
    Comm_TxClear();
    RxNow("MOV,F,10\n");
    CHECK(CountMatching("ERR,BUSY") == 1U);

    Cycles(30U);
    CHECK(CountMatching("ACK,STOPPED") == 1U);
    Comm_TxClear();
    RxNow("MOV,F,10\n");
    CHECK(CountMatching("ERR,BUSY") == 0U);         /* accepted again */
}

static void test_manual_stop_coalescing(void)
{
    /* (a) A manual STOP already outstanding wins: no automatic request. */
    Fresh();
    EnableParking();
    StartForward();
    ArmApproach();
    Comm_TxClear();
    RxNow("STOP\n");                                /* outstanding, no motion pass yet */
    SampleCommOnly(150U);
    SampleCommOnly(150U);
    SampleCommOnly(150U);
    CHECK(CountMatching("USPARK,STOP*") == 0U);
    Cycles(30U);
    CHECK(CountMatching("USPARK,STOP*") == 0U);
    CHECK(CountMatching("ACK,STOPPED") == 1U);

    /* (b) A manual STOP arriving during an automatic stop coalesces: one
     * brake, ONE ACK,STOPPED, which the manual sender also receives. */
    Fresh();
    EnableParking();
    StartForward();
    ArmApproach();
    Sample(150U);
    Sample(150U);
    UsPublishValid(150U);
    CommOnlyCycle();
    CHECK(CountMatching("USPARK,STOP*") == 1U);
    RxNow("STOP\n");                                /* during the automatic stop */
    Cycles(30U);
    CHECK(CountMatching("ACK,STOPPED") == 1U);
    CHECK(CountMatching("USPARK,STOP*") == 1U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);

    /* (c) A manual STOP after the automatic stop completed is a new
     * operation with its own reply (existing contract). */
    Comm_TxClear();
    RxNow("STOP\n");
    Cycles(5U);
    CHECK(CountMatching("ACK,STOPPED") == 1U);
}

static void test_movement_ends_before_request(void)
{
    /* The movement finishes between decision and request: the guard
     * refuses, so there is no USPARK,STOP line and NO unsolicited
     * ACK,STOPPED; the movement's own ACK is delivered. */
    Fresh();
    EnableParking();
    StartForward();
    ArmApproach();
    Sample(150U);
    Sample(150U);

    hookAction = HOOK_COMPLETE_MOVEMENT;
    hookCalls = 0U;
    UsPublishValid(150U);
    CommOnlyCycle();                                /* decision -> hook -> request */
    CHECK(hookCalls == 1U);
    Cycles(30U);

    CHECK(CountMatching("USPARK,STOP*") == 0U);
    CHECK(CountMatching("ACK,STOPPED") == 0U);
    CHECK(CountMatching("ACK") == 1U);              /* its own completion */
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);

    /* The movement was replaced by a different forward movement: still
     * refused (different instance), and the new movement keeps running. */
    Fresh();
    EnableParking();
    StartForward();
    ArmApproach();
    Sample(150U);
    Sample(150U);

    hookAction = HOOK_REPLACE_MOVEMENT;
    hookCalls = 0U;
    UsPublishValid(150U);
    CommOnlyCycle();
    CHECK(hookCalls == 1U);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    Cycles(3U);

    CHECK(CountMatching("USPARK,STOP*") == 0U);
    CHECK(CountMatching("ACK,STOPPED") == 0U);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
}

static void test_completion_and_disable(void)
{
    /* Normal completion resets the detector. */
    Fresh();
    EnableParking();
    StartForward();
    ArmApproach();
    Sample(150U);
    Sample(150U);
    Fake_SetEncoderLeft(MOV_50CM_TARGET_COUNTS);
    Fake_SetEncoderRight(MOV_50CM_TARGET_COUNTS);
    Cycles(30U);
    CHECK(CountMatching("ACK") >= 1U);
    CHECK(CountMatching("USPARK,STOP*") == 0U);
    ExpectGet("USPARK,E:1,ST:UNARMED,CN:0,*");

    /* Disabling mid-approach clears the detector; later near readings do
     * nothing and the movement is not touched. */
    Fresh();
    EnableParking();
    StartForward();
    ArmApproach();
    Sample(150U);
    Sample(150U);
    Comm_TxClear();
    RxNow("USPARK,0\n");
    CHECK(CountMatching("USPARK,OK") == 1U);
    ExpectGet("USPARK,E:0,ST:DISABLED,CN:0,*");
    Sample(150U);
    Sample(150U);
    Sample(150U);
    CHECK(CountMatching("USPARK,STOP*") == 0U);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);

    /* Disabling cannot cancel an already accepted automatic STOP. */
    Fresh();
    EnableParking();
    StartForward();
    ArmApproach();
    Sample(150U);
    Sample(150U);
    UsPublishValid(150U);
    CommOnlyCycle();                                /* automatic STOP accepted */
    CHECK(CountMatching("USPARK,STOP*") == 1U);
    Comm_TxClear();
    RxNow("USPARK,0\n");
    CHECK(CountMatching("USPARK,OK") == 1U);
    Cycles(30U);
    CHECK(CountMatching("ACK,STOPPED") == 1U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
}

int main(void)
{
    MotionQueueHandle = osMessageQueueNew(8U, sizeof(MotionCommand), NULL);
    CommRxQueueHandle = osMessageQueueNew(128U, 1U, NULL);
    CHECK(MotionQueueHandle != NULL);
    CHECK(CommRxQueueHandle != NULL);

    test_disabled_is_task1_unchanged();
    test_service_cadence_and_disabled_cost();
    test_command_behaviour();
    test_automatic_stop_wire_sequence();
    test_threshold_boundaries_on_the_wire();
    test_armed_400_does_not_stop();
    test_polling_one_publication_does_not_trigger();
    test_alternating_invalid_does_not_trigger();
    test_unarmed_noecho_never_stops();
    test_armed_failsafes();
    test_forward_only();
    test_one_stop_per_movement();
    test_stop_in_progress_blocks_commands_like_manual();
    test_manual_stop_coalescing();
    test_movement_ends_before_request();
    test_completion_and_disable();

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
