/* Host test for MOV,FIRL / MOV,FIRR on the wire (Phase 5): the REAL
 * Core/Src/app_comm.c, vision_rx.c, app_motion.c and app_ir_edge.c, with the
 * CommTask / MotionTask emulation of the other tests/comm suites (MotionTask
 * calls the IR service between AppMotion_HandleCommand() and AppMotion_Update(),
 * exactly as main.c does) and raw IR publications injected every 20 ms.
 *
 *   gcc -std=c11 -Wall -Wextra -Werror -Itests/comm/fakes -Itests/motion/fakes \
 *       -ICore/Inc -o tests/comm/app_comm_fir_test.exe \
 *       tests/comm/app_comm_fir_test.c Core/Src/app_comm.c \
 *       Core/Src/vision_rx.c Core/Src/app_motion.c Core/Src/app_pid.c \
 *       Core/Src/app_ir_edge.c Core/Src/app_ir_dist.c Core/Src/app_us_park.c \
 *       tests/motion/fakes/hw_stubs.c tests/comm/fakes/comm_stubs.c \
 *       tests/comm/fakes/ultrasonic_stub.c tests/comm/fakes/ir_stub.c
 *   ./tests/comm/app_comm_fir_test.exe
 * and with -DAPP_IR_ENABLE=0 (the IR feature compiled out): MOV,FIRL/FIRR are
 * answered ERR,BAD_CMD and nothing moves.
 *
 * What is proved: the exact wire sequence of an edge completion (START, the
 * one IREDGE decision line, END, then exactly one ACK, END immediately followed
 * by its reply, for 20 phase offsets); IR_NO_OBSTACLE / IR_NO_EDGE / IR_SENSOR
 * each end with one END and one ERR; the IR_WAIT failures have no START or END;
 * STOP at every stage gives exactly one ACK,STOPPED, no IREDGE after it (also
 * when the decision was published but not yet sent); parser / BUSY rejections;
 * IR,GET keeps working during the move; USPARK is not applied to FIR moves;
 * and ordinary MOV,F is unchanged.
 */

#include "app_comm.h"
#include "app_motion.h"
#include "app_ir.h"
#include "app_ir_edge.h"
#include "cmsis_os.h"
#include "main.h"
#include "test_fakes.h"
#include "ir_stub.h"
#include "ultrasonic_stub.h"

#include <stdio.h>
#include <stdlib.h>
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

/* ---------------------------------------------------------------------- */
/* Wire helpers                                                           */
/* ---------------------------------------------------------------------- */

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
/* Index of the first line starting with prefix at or after `from`, or -1. */
static int IndexOf(const char *prefix, unsigned from)
{
    unsigned total = Comm_TxLineCount();
    unsigned i;
    char line[400];

    for (i = from; i < total; i++)
    {
        (void)Comm_TxLine(i, line, sizeof(line));
        if (strncmp(line, prefix, strlen(prefix)) == 0) { return (int)i; }
    }
    return -1;
}

static void LineAt(int index, char *out, size_t size)
{
    out[0] = '\0';
    if (index >= 0)
    {
        (void)Comm_TxLine((unsigned)index, out, size);
    }
}

#endif /* APP_IR_ENABLE */

/* Terminal replies: ACK, ACK,STOPPED and every ERR. */
static unsigned TerminalCount(void)
{
    unsigned total = Comm_TxLineCount();
    unsigned i;
    unsigned n = 0U;
    char line[400];

    for (i = 0U; i < total; i++)
    {
        (void)Comm_TxLine(i, line, sizeof(line));
        if ((strcmp(line, "ACK") == 0) || (strncmp(line, "ACK,", 4) == 0) ||
            (strncmp(line, "ERR,", 4) == 0))
        {
            n++;
        }
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

/* ---------------------------------------------------------------------- */
/* World                                                                  */
/* ---------------------------------------------------------------------- */

static int32_t encLeft, encRight;
static uint32_t tick;

#if APP_IR_ENABLE
static unsigned cycleNo;
static int publishing;
static int nearUs;                         /* publish near ultrasonic readings */
static AppIrSnapshot snap;
static uint32_t pubSeq;
static uint16_t visRaw[2], clrRaw[2];
static AppIrReading reading;
static uint32_t usSeq;
static AppUltrasonicSnapshot usSnap;

static uint32_t NextSeq(uint32_t s)
{
    s++;
    return (s == 0U) ? 1U : s;
}

static void ComputeSamples(void)
{
    int side;

    for (side = 0; side < 2; side++)
    {
        AppIrEdgeDetector d;

        (void)AppIrEdge_Start(&d, (AppIrEdgeSide)side, 0U);
        visRaw[side] = (uint16_t)(d.visibleMinAdc + 50U);
        clrRaw[side] = (uint16_t)(d.clearMaxAdc - 50U);
    }
}

/* cls: 'V' visible, 'C' clear, 'N' NEAR, 'E' ERROR publication, 'F' FAULT. */
static void PublishIr(char cls, int side)
{
    int other = 1 - side;
    uint16_t sel;
    uint16_t opp;

    pubSeq = NextSeq(pubSeq);
    snap.sequence = pubSeq;
    snap.publishTick = tick;
    snap.lastValidTick = tick;
    snap.status = (uint8_t)APP_IR_STATUS_VALID;

    switch (cls)
    {
        case 'N': sel = 4095U; opp = clrRaw[other]; break;
        case 'E': sel = 0U; opp = 0U; snap.status = (uint8_t)APP_IR_STATUS_ERROR; break;
        case 'F': sel = 0U; opp = 0U; snap.status = (uint8_t)APP_IR_STATUS_FAULT; break;
        case 'V': sel = visRaw[side]; opp = clrRaw[other]; break;
        default: sel = clrRaw[side]; opp = visRaw[other]; break;
    }

    snap.leftRaw = (side == 0) ? sel : opp;
    snap.rightRaw = (side == 0) ? opp : sel;
}

static void UsPublish(uint16_t mm)
{
    usSeq++;
    usSnap.sequence = usSeq;
    usSnap.cycles = usSeq;
    usSnap.result = APP_US_RESULT_VALID;
    usSnap.filteredValid = 1U;
    usSnap.filteredMm = mm;
    usSnap.rawValid = 1U;
    usSnap.rawMm = mm;
    usSnap.lastValidTick = tick;
    usSnap.lastSampleTick = tick;
    usSnap.state = APP_US_STATE_IDLE;
    usSnap.faultMask = 0U;
    UsStub_Set(&usSnap);
}
#endif /* APP_IR_ENABLE */

static void Plant(void)
{
    encLeft += Fake_GetLastMotorLeftDuty() * 3 / 4;
    encRight += Fake_GetLastMotorRightDuty() * 3 / 4;
    Fake_SetEncoderLeft(encLeft);
    Fake_SetEncoderRight(encRight);
}

static void MotionStep(void)
{
    MotionCommand command;

    if (AppComm_TakeMotionCommand(&command))
    {
        AppMotion_HandleCommand(&command);
    }

#if APP_IR_ENABLE
    if (AppMotion_IrEdgeArmed())
    {
        AppMotion_ServiceIrEdge(&snap, tick);
    }
#endif

    AppMotion_Update(0.01f);
}

static void CommStep(void)
{
    DrainRx();
    AppComm_ServiceTelemetry();
}

#if APP_IR_ENABLE
/* One 10 ms iteration of both tasks; the IR task publishes every 20 ms. */
static void Cycle(char cls, int side)
{
    tick += 10U;
    Fake_SetTickCount(tick);
    Plant();

    if (publishing && ((cycleNo++ & 1U) == 0U))
    {
        PublishIr(cls, side);
    }

    if (nearUs && ((cycleNo & 7U) == 0U))
    {
        UsPublish(150U);
    }

    MotionStep();
    CommStep();
}

static void Cycles(unsigned n, char cls, int side)
{
    while (n-- != 0U)
    {
        Cycle(cls, side);
    }
}

static void Fresh(void)
{
    (void)osMessageQueueReset(MotionQueueHandle);
    (void)osMessageQueueReset(CommRxQueueHandle);

    tick = 1000U;
    Fake_SetTickCount(tick);
    encLeft = 0;
    encRight = 0;
    Fake_SetEncoderLeft(0);
    Fake_SetEncoderRight(0);
    Fake_SetHeading(0.0f);
    Fake_SetGyroZ(0.0f);
    Fake_SetImuReady(true);
    Fake_SetImuEvent(APP_IMU_EVENT_NONE, 0U);
    Fake_ResetMotorCallCounts();

    AppMotion_Init();
    AppMotion_ApplyProfile(APP_MOTION_PROFILE_TASK2TEST);
    AppComm_Init();
    Comm_TxClear();
    IrStub_ResetCounts();

    memset(&snap, 0, sizeof(snap));
    publishing = 0;
    nearUs = 0;
    cycleNo = 0U;
}

static unsigned RunToIdle(char cls, int side, unsigned limit)
{
    unsigned i;

    for (i = 0U; (i < limit) && (AppMotion_GetState() != APP_MOTION_IDLE); i++)
    {
        Cycle(cls, side);
    }
    return i;
}
#endif /* APP_IR_ENABLE */

#if APP_IR_ENABLE

/* A whole pass: start before the obstacle, FAR -> visible -> clear. */
static void DoPass(const char *command, int side, unsigned offsetCycles)
{
    Fresh();
    publishing = 1;
    Cycles(offsetCycles, 'C', side);              /* shifts the phase of the 20 ms / 10 ms grids */
    RxNow(command);
    Cycles(10U, 'C', side);                       /* ready, then open space */
    Cycles(16U, 'V', side);                       /* the obstacle goes by */
    (void)RunToIdle('C', side, 200U);
    Cycles(40U, 'C', side);                       /* let the telemetry drain */
}

typedef struct
{
    char dir[8];
    char res[16];
    unsigned sa;
    long sc, sr, cc, cr;
    unsigned un, amb, miss;
    unsigned long rdy;
    char w[8];
} EdgeLine;

static int ParseEdge(const char *line, EdgeLine *e)
{
    return sscanf(line,
                  "IREDGE,D:%7[A-Z],R:%15[A-Z_],SA:%u,SC:%ld,SR:%ld,CC:%ld,CR:%ld,"
                  "UN:%u,AMB:%u,MISS:%u,RDY:%lu,W:%7s",
                  e->dir, e->res, &e->sa, &e->sc, &e->sr, &e->cc, &e->cr,
                  &e->un, &e->amb, &e->miss, &e->rdy, e->w) == 12;
}

/* The full set of wire invariants of a movement that ended in `result`
 * (the END RESULT:) with terminal reply `reply`. */
static void ExpectDecisionSequence(const char *dir, const char *edgeResult,
                                   const char *endResult, const char *reply)
{
    char line[400];
    char want[80];
    int iStart, iEdge, iEnd;

    (void)snprintf(want, sizeof(want), "MM,EV:START,D:%s,T:0,V:", dir);
    iStart = IndexOf(want, 0U);
    CHECK(iStart >= 0);
    CHECK(CountPrefix("MM,EV:START") == 1U);

    (void)snprintf(want, sizeof(want), "IREDGE,D:%s,R:%s,", dir, edgeResult);
    iEdge = IndexOf(want, 0U);
    CHECK(iEdge >= 0);
    CHECK(CountPrefix("IREDGE") == 1U);               /* the decision is sent exactly once */

    (void)snprintf(want, sizeof(want), "MM,EV:END,D:%s,T:", dir);
    iEnd = IndexOf(want, 0U);
    CHECK(iEnd >= 0);
    CHECK(CountPrefix("MM,EV:END") == 1U);
    LineAt(iEnd, line, sizeof(line));
    (void)snprintf(want, sizeof(want), ",RESULT:%s", endResult);
    CHECK(strstr(line, want) != NULL);

    CHECK(iStart < iEdge);
    CHECK(iEdge < iEnd);                              /* the decision precedes the brake's end */

    /* END immediately followed by its one terminal reply */
    LineAt(iEnd + 1, line, sizeof(line));
    CHECK_STR(line, reply);
    CHECK(TerminalCount() == 1U);
    CHECK(CountPrefix("ACK,STOPPED") == 0U);
}

static void TestEdgeCompletion(void)
{
    int side;

    for (side = 0; side < 2; side++)
    {
        const char *dir = side ? "FIRR" : "FIRL";
        const char *cmd = side ? "MOV,FIRR,200\n" : "MOV,FIRL,200\n";
        char line[400];
        EdgeLine e;
        int iEdge;

        DoPass(cmd, side, 0U);
        ExpectDecisionSequence(dir, "EDGE", "OK", "ACK");

        iEdge = IndexOf("IREDGE", 0U);
        LineAt(iEdge, line, sizeof(line));
        CHECK(ParseEdge(line, &e));
        CHECK_STR(e.dir, dir);
        CHECK_STR(e.res, "EDGE");
        CHECK(e.sa == 0U);
        CHECK(e.sc > 0L);                             /* the obstacle was confirmed while driving */
        CHECK(e.cc > e.sc);                           /* then the edge, further on */
        CHECK(e.sr > 0L);
        CHECK(e.cr >= 0L);
        CHECK(e.miss == 0U);
        CHECK(e.rdy >= 10UL);                         /* the readiness wait is reported */
        CHECK_STR(e.w, "NA");
    }
}

/* END is immediately followed by its reply for every phase of the cycle grids. */
static void TestPhaseOffsets(void)
{
    unsigned off;

    for (off = 0U; off < 20U; off++)
    {
        DoPass("MOV,FIRL,200\n", 0, off);
        ExpectDecisionSequence("FIRL", "EDGE", "OK", "ACK");
    }
}

static void TestMaximumTravelErrors(void)
{
    int mode;

    /* 0: open space, 1: obstacle that never ends, 2: NEAR forever */
    for (mode = 0; mode < 3; mode++)
    {
        char cls = (mode == 0) ? 'C' : (mode == 1) ? 'V' : 'N';
        const char *edgeRes = (mode == 0) ? "NO_OBSTACLE" : "NO_EDGE";
        char reply[40];
        char endRes[24];
        char line[400];
        EdgeLine e;

        Fresh();
        publishing = 1;
        RxNow("MOV,FIRR,12\n");
        Cycles(4U, cls, 1);                          /* MotionTask takes the command */
        (void)RunToIdle(cls, 1, 1500U);
        Cycles(40U, cls, 1);

        (void)snprintf(reply, sizeof(reply), "ERR,IR_%s", edgeRes);
        (void)snprintf(endRes, sizeof(endRes), "IR_%s", edgeRes);
        ExpectDecisionSequence("FIRR", edgeRes, endRes, reply);

        LineAt(IndexOf("IREDGE", 0U), line, sizeof(line));
        CHECK(ParseEdge(line, &e));
        CHECK(e.cc == -1L);
        CHECK(e.cr == -1L);
        CHECK((mode == 0) == (e.sc == -1L));
    }
}

static void TestSensorFailures(void)
{
    int mode;
    char line[400];
    EdgeLine e;

    /* driving: the IR task goes silent / FAULT / ERROR publications */
    for (mode = 0; mode < 3; mode++)
    {
        Fresh();
        publishing = 1;
        RxNow("MOV,FIRL,300\n");
        Cycles(10U, 'C', 0);
        Cycles(20U, 'V', 0);
        if (mode == 0)
        {
            publishing = 0;
        }
        (void)RunToIdle((mode == 1) ? 'F' : 'E', 0, 200U);
        Cycles(40U, 'E', 0);

        ExpectDecisionSequence("FIRL", "SENSOR", "IR_SENSOR", "ERR,IR_SENSOR");
        LineAt(IndexOf("IREDGE", 0U), line, sizeof(line));
        CHECK(ParseEdge(line, &e));
        CHECK_STR(e.w, (mode == 1) ? "FAULT" : "DATA");
    }

    /* never ready: no START and no END at all */
    Fresh();
    publishing = 0;
    RxNow("MOV,FIRR,100\n");
    Cycles(60U, 'C', 1);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK(CountPrefix("MM,EV:START") == 0U);
    CHECK(CountPrefix("MM,EV:END") == 0U);
    CHECK(CountPrefix("MM,") == 0U);
    CHECK(CountPrefix("ERR,IR_SENSOR") == 1U);
    CHECK(TerminalCount() == 1U);
    CHECK(CountPrefix("IREDGE") == 1U);
    LineAt(IndexOf("IREDGE", 0U), line, sizeof(line));
    CHECK(ParseEdge(line, &e));
    CHECK_STR(e.res, "SENSOR");
    CHECK_STR(e.w, "READY");
    CHECK(Fake_GetMotorSetCallCount() == 0U);

    /* FAULT while waiting */
    Fresh();
    publishing = 1;
    RxNow("MOV,FIRL,100\n");
    Cycles(10U, 'F', 0);
    CHECK(CountPrefix("ERR,IR_SENSOR") == 1U);
    CHECK(CountPrefix("MM,EV:") == 0U);
    CHECK(TerminalCount() == 1U);
}

/* STOP at every stage: one ACK,STOPPED and no IREDGE after it. */
static void StopCase(int kind, unsigned k)
{
    char line[400];
    int iAck;

    Fresh();
    publishing = 1;
    RxNow("MOV,FIRL,300\n");

    if (kind == 0)
    {
        /* IR_WAIT: stationary, no START yet */
        Cycles(1U, 'C', 0);
        CHECK(AppMotion_GetState() == APP_MOTION_IR_WAIT);
    }
    else
    {
        Cycles(10U, 'C', 0);
        CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    }
    if (kind >= 2)
    {
        Cycles(16U, 'V', 0);
    }
    if (kind == 3)
    {
        Cycles(k, 'C', 0);
    }
    if (kind == 4)
    {
        /* the decision is published by MotionTask but CommTask has not sent it:
         * run MotionTask alone until it has, then STOP */
        unsigned guard;

        for (guard = 0U; (guard < 80U) && (AppMotion_GetState() == APP_MOTION_DISTANCE); guard++)
        {
            tick += 10U;
            Fake_SetTickCount(tick);
            Plant();
            if ((cycleNo++ & 1U) == 0U)
            {
                PublishIr('C', 0);
            }
            MotionStep();
        }
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
        CHECK(CountPrefix("IREDGE") == 0U);          /* decided, not yet transmitted */
    }

    RxNow("STOP\n");
    Cycles(40U, 'C', 0);

    iAck = IndexOf("ACK,STOPPED", 0U);
    CHECK(iAck >= 0);
    CHECK(CountPrefix("ACK,STOPPED") == 1U);
    CHECK(TerminalCount() == 1U);
    CHECK(CountPrefix("ACK\n") == 0U);
    CHECK(CountPrefix("ERR,") == 0U);

    if (kind == 0)
    {
        CHECK(CountPrefix("MM,EV:") == 0U);            /* never started: no START, no END */
    }
    else
    {
        int iEnd = IndexOf("MM,EV:END,D:FIRL,", 0U);

        CHECK(iEnd >= 0);
        LineAt(iEnd, line, sizeof(line));
        CHECK(strstr(line, "RESULT:STOPPED") != NULL);
        CHECK(iEnd + 1 == iAck);                       /* END immediately before its reply */
    }

    /* never an IREDGE after (or because of) a STOP */
    {
        int iEdge = IndexOf("IREDGE", 0U);

        CHECK((iEdge < 0) || (iEdge < iAck));
        if (kind <= 2)
        {
            CHECK(iEdge < 0);
        }
        if (kind == 4)
        {
            CHECK(iEdge < 0);                           /* the unsent decision was dropped */
        }
    }

    /* and nothing late */
    {
        unsigned before = Comm_TxLineCount();

        Cycles(100U, 'V', 0);
        Cycles(100U, 'C', 0);
        CHECK(CountPrefix("ACK,STOPPED") == 1U);
        CHECK(TerminalCount() == 1U);
        (void)before;
    }
}

static void TestStop(void)
{
    unsigned k;

    StopCase(0, 0U);
    StopCase(1, 0U);
    StopCase(2, 0U);
    StopCase(4, 0U);

    for (k = 0U; k < 14U; k++)
    {
        StopCase(3, k);
    }
}

static void TestParserAndBusy(void)
{
    char line[400];

    Fresh();
    RxNow("MOV,FIRL,0\n");
    CHECK(CountPrefix("ERR,BAD_CMD") == 1U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);

    Comm_TxClear();
    RxNow("MOV,FIR,L,5\n");
    CHECK(CountPrefix("ERR,BAD_CMD") == 1U);

    Comm_TxClear();
    RxNow("MOV,FIRL,1000\n");
    CHECK(CountPrefix("ERR,BAD_CMD") == 1U);

    /* a second command while the first is waiting / driving is BUSY */
    Fresh();
    publishing = 1;
    RxNow("MOV,FIRL,100\n");
    Cycles(1U, 'C', 0);
    CHECK(AppMotion_GetState() == APP_MOTION_IR_WAIT);
    Comm_TxClear();
    RxNow("MOV,F,10\n");
    CHECK(CountPrefix("ERR,BUSY") == 1U);
    Comm_TxClear();
    RxNow("MOV,FIRR,10\n");
    CHECK(CountPrefix("ERR,BUSY") == 1U);
    RxNow("STOP\n");
    Cycles(10U, 'C', 0);
    CHECK(CountPrefix("ACK,STOPPED") == 1U);

    /* the IMU must be ready, like for MOV,F */
    Fresh();
    Fake_SetImuReady(false);
    RxNow("MOV,FIRL,100\n");
    Cycles(5U, 'C', 0);
    CHECK(CountPrefix("ERR,IMU_NOT_READY") == 1U);
    CHECK(TerminalCount() == 1U);
    (void)line;
}

/* IR,GET keeps working during the move; the move still ends with one ACK. */
static void TestIrGetDuringMove(void)
{
    AppIrReading r;
    unsigned i;

    memset(&r, 0, sizeof(r));
    r.sequence = 77U;
    r.ageTicks = 5U;
    r.acq = (uint8_t)APP_IR_ACQ_OK;
    r.range[0] = (uint8_t)APP_IR_RANGE_OK;
    r.range[1] = (uint8_t)APP_IR_RANGE_FAR;
    r.distanceMm[0] = 150U;
    reading = r;
    IrStub_SetReading(&reading);

    Fresh();
    IrStub_SetReading(&reading);
    publishing = 1;
    RxNow("MOV,FIRL,200\n");
    Cycles(10U, 'C', 0);
    for (i = 0U; i < 8U; i++)
    {
        RxNow("IR,GET\n");
        Cycles(4U, 'V', 0);
    }
    (void)RunToIdle('C', 0, 200U);
    Cycles(40U, 'C', 0);

    CHECK(CountPrefix("IR,S:77,ST:OK,L:150,R:FAR,AGE:5,F:00") == 8U);
    ExpectDecisionSequence("FIRL", "EDGE", "OK", "ACK");
}

/* USPARK is not applied to FIR moves (decision D3). */
static void TestUsparkNotApplied(void)
{
    Fresh();
    UsPublish(600U);
    RxNow("USPARK,1\n");
    CHECK(CountPrefix("USPARK,OK") == 1U);
    nearUs = 1;                                        /* near ultrasonic readings all along */
    publishing = 1;
    RxNow("MOV,FIRR,300\n");
    Cycles(10U, 'C', 1);
    Cycles(60U, 'C', 1);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    CHECK(CountPrefix("USPARK,STOP") == 0U);
    CHECK(CountPrefix("ACK,STOPPED") == 0U);
    RxNow("STOP\n");
    Cycles(40U, 'C', 1);
    CHECK(CountPrefix("ACK,STOPPED") == 1U);
    CHECK(CountPrefix("USPARK,STOP") == 0U);
    nearUs = 0;
    UsPublish(600U);
    RxNow("USPARK,0\n");
}

/* Ordinary MOV,F is unaffected, before and after FIR moves. */
static void TestPlainMove(void)
{
    char line[400];

    Fresh();
    publishing = 1;
    RxNow("MOV,F,10\n");
    Cycles(4U, 'C', 0);                            /* MotionTask takes the command */
    (void)RunToIdle('C', 0, 800U);
    Cycles(40U, 'C', 0);
    CHECK(CountPrefix("MM,EV:START,D:F,T:0,V:10") == 1U);
    CHECK(CountPrefix("MM,EV:END,D:F,") == 1U);
    CHECK(CountPrefix("IREDGE") == 0U);
    CHECK(TerminalCount() == 1U);
    LineAt(IndexOf("MM,EV:END,D:F,", 0U) + 1, line, sizeof(line));
    CHECK_STR(line, "ACK");
}

#endif /* APP_IR_ENABLE */

#if !APP_IR_ENABLE
/* The IR feature compiled out: FIR commands are rejected and nothing moves. */
static void TestCompiledOut(void)
{
    unsigned i;

    (void)osMessageQueueReset(MotionQueueHandle);
    (void)osMessageQueueReset(CommRxQueueHandle);
    tick = 1000U;
    Fake_SetTickCount(tick);
    Fake_SetImuReady(true);
    AppMotion_Init();
    AppComm_Init();
    Comm_TxClear();

    RxNow("MOV,FIRL,50\n");
    RxNow("MOV,FIRR,50\n");
    for (i = 0U; i < 20U; i++)
    {
        tick += 10U;
        Fake_SetTickCount(tick);
        MotionStep();
        CommStep();
    }
    CHECK(CountPrefix("ERR,BAD_CMD") == 2U);
    CHECK(TerminalCount() == 2U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK(Fake_GetMotorSetCallCount() == 0U);
    CHECK(CountPrefix("MM,") == 0U);
    CHECK(CountPrefix("IREDGE") == 0U);

    /* ordinary movement still works */
    Comm_TxClear();
    RxNow("MOV,F,5\n");
    for (i = 0U; (i < 800U); i++)
    {
        tick += 10U;
        Fake_SetTickCount(tick);
        Plant();
        MotionStep();
        CommStep();
    }
    CHECK(CountPrefix("MM,EV:START,D:F,T:0,V:5") == 1U);
    CHECK(CountPrefix("ACK") == 1U);
}
#endif

int main(void)
{
    MotionQueueHandle = osMessageQueueNew(8U, sizeof(MotionCommand), NULL);
    CommRxQueueHandle = osMessageQueueNew(128U, 1U, NULL);
    CHECK(MotionQueueHandle != NULL);
    CHECK(CommRxQueueHandle != NULL);

#if APP_IR_ENABLE
    ComputeSamples();
    pubSeq = 5000U;
    memset(&usSnap, 0, sizeof(usSnap));
    UsStub_Set(&usSnap);

    TestEdgeCompletion();
    TestPhaseOffsets();
    TestMaximumTravelErrors();
    TestSensorFailures();
    TestStop();
    TestParserAndBusy();
    TestIrGetDuringMove();
    TestUsparkNotApplied();
    TestPlainMove();
#else
    TestCompiledOut();
#endif

    printf("mode: APP_IR_ENABLE=%d\n", APP_IR_ENABLE);
    printf("%u checks, %u failures\n", checks, failures);
    return failures == 0U ? 0 : 1;
}
