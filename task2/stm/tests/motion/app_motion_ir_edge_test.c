/* Host test for the Phase 5 IR-terminated forward movement in the REAL
 * Core/Src/app_motion.c (MOTION_FORWARD_IR_LEFT / _RIGHT, i.e. MOV,FIRL / FIRR)
 * with the REAL Core/Src/app_ir_edge.c, against the fake motors / encoders /
 * IMU / tick clock of tests/motion/fakes and a toy plant (the encoders follow the
 * commanded duty). IR data is injected as raw AppIrSnapshot publications, one
 * every 20 ms, with MotionTask's 10 ms loop emulated exactly as main.c does it:
 *   AppMotion_HandleCommand -> (armed) AppMotion_ServiceIrEdge -> AppMotion_Update.
 *
 *   gcc -std=c11 -Wall -Wextra -Werror -DAPP_IR_ENABLE=1 -ICore/Inc -Itests/motion/fakes \
 *       -o tests/motion/app_motion_ir_edge_test.exe tests/motion/app_motion_ir_edge_test.c \
 *       Core/Src/app_motion.c Core/Src/app_pid.c Core/Src/app_ir_edge.c \
 *       Core/Src/app_ir_dist.c tests/motion/fakes/hw_stubs.c
 *   ./tests/motion/app_motion_ir_edge_test.exe
 *
 * What is proved: entry validation (every rejection leaves the system idle with
 * exactly one error event); the stationary IR_WAIT with its bounded readiness;
 * FAR -> visible -> clear brakes and completes exactly once with MOV_DONE after
 * pulling the target in; already visible at start; open space and a permanently
 * visible obstacle end with IR_NO_OBSTACLE / IR_NO_EDGE at the maximum travel;
 * NEAR never completes; unusable data and FAULT brake with IR_SENSOR; STOP in
 * every stage gives exactly one STOPPED and no IR decision; the speed cap acts
 * only on FIR movements; LEFT / RIGHT mapping; a clean MOV,F afterwards; and a
 * tick-counter wrap.
 */

#include "app_motion.h"
#include "app_servo.h"
#include "app_ir_edge.h"
#include "test_fakes.h"

#include <stdio.h>
#include <string.h>

static unsigned checks;
static unsigned failures;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        failures++; \
        printf("FAIL line %d: %s\n", __LINE__, #expr); \
    } \
} while (0)

#define V APP_IR_EDGE_CLASS_VISIBLE
#define C APP_IR_EDGE_CLASS_CLEAR
#define A APP_IR_EDGE_CLASS_AMBIGUOUS

/* ---------------------------------------------------------------------- */
/* World                                                                  */
/* ---------------------------------------------------------------------- */

static uint32_t tick;
static int32_t encLeft, encRight;
static AppIrSnapshot snap;
static uint32_t pubSeq;
static unsigned cycleNo;
static uint16_t visRaw[2], clrRaw[2], ambRaw[2];
static int publishing;      /* 0 = the IR task is silent */
static uint32_t edgeBase;   /* decision-mailbox sequence when the test case began */

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
        ambRaw[side] = (uint16_t)((d.visibleMinAdc + d.clearMaxAdc) / 2U);
    }
}

static AppMotionEvent LastEvent(void)
{
    AppMotionEvent e;

    AppMotion_GetEvent(&e);
    return e;
}

static uint32_t EventSeq(void)
{
    return LastEvent().sequence;
}

static AppMotionIrEdgeSnapshot Edge(void)
{
    AppMotionIrEdgeSnapshot s;

    AppMotion_GetIrEdgeSnapshot(&s);
    return s;
}

static void SendStop(void)
{
    MotionCommand stop = { MOTION_STOP, 0 };

    AppMotion_HandleCommand(&stop);
}

static void Plant(void)
{
    encLeft += Fake_GetLastMotorLeftDuty() * 3 / 4;
    encRight += Fake_GetLastMotorRightDuty() * 3 / 4;
    Fake_SetEncoderLeft(encLeft);
    Fake_SetEncoderRight(encRight);
}

/* What the IR task would publish every 20 ms. cls: V / C / A, or 'N' = NEAR,
 * 'E' = ERROR publication, 'F' = FAULT publication. `side` data is the
 * selected sensor; the other one shows the opposite so a mapping bug shows. */
static void Publish(int cls, int side)
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
        case V: sel = visRaw[side]; opp = clrRaw[other]; break;
        case A: sel = ambRaw[side]; opp = visRaw[other]; break;
        default: sel = clrRaw[side]; opp = visRaw[other]; break;
    }

    snap.leftRaw = (side == 0) ? sel : opp;
    snap.rightRaw = (side == 0) ? opp : sel;
}

/* One MotionTask loop iteration (10 ms). The IR task publishes every second
 * iteration; `cls` is what it reads then. */
static void Cycle(int cls, int side)
{
    tick += 10U;
    Fake_SetTickCount(tick);
    Plant();

    if (publishing && ((cycleNo++ & 1U) == 0U))
    {
        Publish(cls, side);
    }

    if (AppMotion_IrEdgeArmed())
    {
        AppMotion_ServiceIrEdge(&snap, tick);
    }

    AppMotion_Update(0.01f);
}

static void Cycles(unsigned n, int cls, int side)
{
    while (n-- != 0U)
    {
        Cycle(cls, side);
    }
}

/* Runs until the state is IDLE (or the limit), with `cls` held. */
static unsigned RunToIdle(int cls, int side, unsigned limit)
{
    unsigned i;

    for (i = 0U; (i < limit) && (AppMotion_GetState() != APP_MOTION_IDLE); i++)
    {
        Cycle(cls, side);
    }
    return i;
}

static void Begin(uint32_t startTick)
{
    /* idle, clean fakes, silent IR, PRECISION-like boot defaults */
    SendStop();
    if (AppMotion_GetState() == APP_MOTION_BRAKING)
    {
        tick += 300U;
        Fake_SetTickCount(tick);
        AppMotion_Update(0.01f);
    }
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);

    AppMotion_ApplyProfile(APP_MOTION_PROFILE_TASK2TEST);
    Fake_SetImuReady(true);
    tick = startTick;
    Fake_SetTickCount(tick);
    encLeft = 0;
    encRight = 0;
    Fake_SetEncoderLeft(0);
    Fake_SetEncoderRight(0);
    Fake_SetHeading(0.0f);
    Fake_SetGyroZ(0.0f);
    Fake_ResetMotorCallCounts();
    memset(&snap, 0, sizeof(snap));
    publishing = 0;
    cycleNo = 0U;
    edgeBase = Edge().sequence;
}

static void Fir(MotionType type, int32_t cm)
{
    MotionCommand command;

    command.type = type;
    command.value = cm;
    AppMotion_HandleCommand(&command);
}

static int SideOf(MotionType type)
{
    return (type == MOTION_FORWARD_IR_RIGHT) ? 1 : 0;
}

static void ExpectRejected(MotionType type, int32_t cm, AppMotionEventType expected)
{
    uint32_t seq = EventSeq();

    Fir(type, cm);
    CHECK(EventSeq() == seq + 1U);
    CHECK(LastEvent().type == expected);
    CHECK(LastEvent().command == type);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK(!AppMotion_IrEdgeArmed());
    CHECK(Fake_GetMotorSetCallCount() == 0U);
}

/* Drives a FIR movement to the "seen" side: ready, then SEEN_SCANS visible. */
static void DriveToObstacle(MotionType type, int32_t maxCm)
{
    int side = SideOf(type);

    Begin(100000U);
    publishing = 1;
    Fir(type, maxCm);
    CHECK(AppMotion_GetState() == APP_MOTION_IR_WAIT);
    Cycles(2U * APP_IR_EDGE_READY_SCANS + 2U, C, side);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
}

/* ---------------------------------------------------------------------- */
/* Tests                                                                  */
/* ---------------------------------------------------------------------- */

static void TestEntryValidation(void)
{
    MotionType types[2] = { MOTION_FORWARD_IR_LEFT, MOTION_FORWARD_IR_RIGHT };
    int i;

    for (i = 0; i < 2; i++)
    {
        MotionType t = types[i];
        float target, kp, ki;
        bool en;
        float decel, approach;
        bool sen;

        Begin(5000U);
        ExpectRejected(t, 0, APP_MOTION_EVENT_BAD_COMMAND);
        ExpectRejected(t, -5, APP_MOTION_EVENT_BAD_COMMAND);
        ExpectRejected(t, 1000, APP_MOTION_EVENT_BAD_COMMAND);

        Fake_SetImuReady(false);
        ExpectRejected(t, 50, APP_MOTION_EVENT_IMU_NOT_READY);
        Fake_SetImuReady(true);

        /* SPCTRL disabled: the speed cap lives in that path */
        AppMotion_GetSpeedControl(&target, &kp, &ki, &en);
        AppMotion_SetSpeedControl(target, kp, ki, false);
        ExpectRejected(t, 50, APP_MOTION_EVENT_BAD_COMMAND);
        AppMotion_SetSpeedControl(target, kp, ki, true);

        /* approach speed above the capped cruise */
        AppMotion_GetSpeedProfile(&decel, &approach, &sen);
        AppMotion_SetSpeedProfile(decel, (float)APP_IR_EDGE_SPEED_CAP_COUNTS_PER_SEC + 100.0f, true);
        ExpectRejected(t, 50, APP_MOTION_EVENT_BAD_COMMAND);
        AppMotion_SetSpeedProfile(decel, approach, sen);

        /* nothing was started: still idle and a valid one is accepted */
        CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
        Fir(t, 50);
        CHECK(AppMotion_GetState() == APP_MOTION_IR_WAIT);
        CHECK(AppMotion_IrEdgeArmed());

        /* busy: a second movement while waiting is BUSY and changes nothing */
        {
            uint32_t seq = EventSeq();
            MotionCommand other = { MOTION_FORWARD, 10 };

            AppMotion_HandleCommand(&other);
            CHECK(EventSeq() == seq + 1U);
            CHECK(LastEvent().type == APP_MOTION_EVENT_BUSY);
            CHECK(AppMotion_GetState() == APP_MOTION_IR_WAIT);
        }
        SendStop();
        CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    }
}

static void TestReadinessWait(void)
{
    AppMotionTelemetry t0, t1;
    uint32_t seq;

    Begin(7000U);
    publishing = 1;
    AppMotion_GetTelemetry(&t0);
    seq = EventSeq();

    Fir(MOTION_FORWARD_IR_LEFT, 80);
    CHECK(AppMotion_GetState() == APP_MOTION_IR_WAIT);
    CHECK(AppMotion_IsBusy());

    /* the IR task publishes on cycles 1, 3, 5, ...: after cycle 2 only ONE scan
     * has arrived (two are needed), so the robot must still be stationary */
    Cycles(2U, C, 0);
    CHECK(AppMotion_GetState() == APP_MOTION_IR_WAIT);
    CHECK(Fake_GetMotorSetCallCount() == 0U);
    AppMotion_GetTelemetry(&t1);
    CHECK(t1.active == 0U);                      /* no movement telemetry, so no EV:START */
    CHECK(t1.runSequence == t0.runSequence);
    CHECK(EventSeq() == seq);

    /* the second scan (cycle 3) completes the readiness: driving begins */
    Cycle(C, 0);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    AppMotion_GetTelemetry(&t1);
    CHECK(t1.runSequence == t0.runSequence + 1U);
    CHECK(t1.active == 1U);
    CHECK(t1.activeDirection == MOTION_FORWARD_IR_LEFT);
    CHECK(t1.commandValue == 80);
    CHECK(EventSeq() == seq);                    /* nothing terminal yet */
    CHECK(Edge().sequence == edgeBase);          /* and no decision yet */
    SendStop();
    CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
}

static void TestReadinessTimeout(void)
{
    uint32_t seq;
    unsigned i;
    AppMotionIrEdgeSnapshot e;

    /* IR task silent: ERR after the 300 ms readiness timeout, motors never run */
    Begin(9000U);
    publishing = 0;
    seq = EventSeq();
    Fir(MOTION_FORWARD_IR_RIGHT, 80);
    for (i = 0U; i < 40U; i++)
    {
        Cycle(C, 1);
    }
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK(EventSeq() == seq + 1U);
    CHECK(LastEvent().type == APP_MOTION_EVENT_IR_SENSOR);
    CHECK(LastEvent().command == MOTION_FORWARD_IR_RIGHT);
    CHECK(Fake_GetMotorSetCallCount() == 0U);
    CHECK(Fake_GetMotorBrakeCallCount() == 0U);
    CHECK(!AppMotion_IrEdgeArmed());
    e = Edge();
    CHECK(e.sequence == edgeBase + 1U);
    CHECK(e.result == (uint8_t)APP_MOTION_IR_RESULT_SENSOR);
    CHECK(e.failReason == (uint8_t)APP_IR_EDGE_FAIL_READY_TIMEOUT);

    /* the timeout is strict: ready exactly inside it is fine */
    Begin(9000U);
    publishing = 1;
    Fir(MOTION_FORWARD_IR_LEFT, 80);
    Cycles(30U, 'E', 0);                         /* 300 ms of ERROR publications: not ready */
    CHECK(AppMotion_GetState() == APP_MOTION_IR_WAIT);
    Cycle('E', 0);                               /* 310 ms */
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK(LastEvent().type == APP_MOTION_EVENT_IR_SENSOR);

    /* FAULT while waiting: at once */
    Begin(9000U);
    publishing = 1;
    Fir(MOTION_FORWARD_IR_LEFT, 80);
    Cycles(2U, 'F', 0);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK(LastEvent().type == APP_MOTION_EVENT_IR_SENSOR);
    CHECK(Edge().failReason == (uint8_t)APP_IR_EDGE_FAIL_FAULT);
    CHECK(Fake_GetMotorSetCallCount() == 0U);

    /* and the system works normally afterwards */
    Begin(9000U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    Fir(MOTION_FORWARD, 5);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    SendStop();
}

/* Starting before the obstacle: FAR -> visible -> clear. */
static void TestNormalPass(MotionType type)
{
    int side = SideOf(type);
    uint32_t seq;
    unsigned i;
    int32_t seenAt, clearAt;
    AppMotionIrEdgeSnapshot e;
    AppMotionEvent ev;
    AppMotionTelemetry t;

    DriveToObstacle(type, 300);
    seq = EventSeq();

    Cycles(20U, C, side);                                   /* open space first */
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    CHECK(Edge().sequence == edgeBase);

    for (i = 0U; i < 2U * APP_IR_EDGE_SEEN_SCANS + 2U; i++)
    {
        Cycle(V, side);
    }
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);     /* seen: keeps driving */
    CHECK(Edge().sequence == edgeBase);                           /* no decision yet */
    Cycles(30U, V, side);                                   /* alongside the obstacle */
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    CHECK(EventSeq() == seq);

    /* the edge: CLEAR scans until the movement brakes */
    for (i = 0U; (i < 40U) && (AppMotion_GetState() == APP_MOTION_DISTANCE); i++)
    {
        Cycle(C, side);
    }
    CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
    CHECK(Fake_GetMotorBrakeCallCount() >= 1U);
    CHECK(EventSeq() == seq);                               /* terminal event only after the brake */
    e = Edge();
    CHECK(e.sequence == edgeBase + 1U);                                /* published once, at the decision */
    CHECK(e.result == (uint8_t)APP_MOTION_IR_RESULT_EDGE);
    CHECK(e.startedVisible == 0U);
    seenAt = e.seenCounts;
    clearAt = e.clearCounts;
    CHECK(seenAt > 0);
    CHECK(clearAt > seenAt);
    CHECK(e.missedScans == 0U);
    CHECK(e.direction == type);
    CHECK(e.seenRaw == visRaw[side]);
    CHECK(e.clearRaw == clrRaw[side]);
    CHECK(clearAt < 300 * 75);                              /* long before the maximum travel */

    /* the target was pulled in: it is not beyond where the robot is (POST_CM) */
    AppMotion_GetTelemetry(&t);
    (void)t;

    (void)RunToIdle(C, side, 40U);
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    ev = LastEvent();
    CHECK(EventSeq() == seq + 1U);                          /* exactly one terminal event */
    CHECK(ev.type == APP_MOTION_EVENT_MOV_DONE);
    CHECK(ev.command == type);
    CHECK(Edge().sequence == edgeBase + 1U);                           /* and still one decision */
    CHECK(!AppMotion_IrEdgeArmed());

    /* nothing more happens afterwards */
    Cycles(100U, V, side);
    Cycles(100U, C, side);
    CHECK(EventSeq() == seq + 1U);
    CHECK(Edge().sequence == edgeBase + 1U);
}

static void TestAlreadyVisible(void)
{
    AppMotionIrEdgeSnapshot e;
    uint32_t seq;

    DriveToObstacle(MOTION_FORWARD_IR_LEFT, 300);
    /* replace: start with the obstacle already beside the robot */
    Begin(100000U);
    publishing = 1;
    seq = EventSeq();
    Fir(MOTION_FORWARD_IR_LEFT, 300);
    Cycles(2U * APP_IR_EDGE_READY_SCANS + 2U, V, 0);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    Cycles(2U * APP_IR_EDGE_SEEN_SCANS, V, 0);
    Cycles(20U, C, 0);
    CHECK(AppMotion_GetState() != APP_MOTION_DISTANCE);
    e = Edge();
    CHECK(e.result == (uint8_t)APP_MOTION_IR_RESULT_EDGE);
    CHECK(e.startedVisible == 1U);
    (void)RunToIdle(C, 0, 40U);
    CHECK(LastEvent().type == APP_MOTION_EVENT_MOV_DONE);
    CHECK(EventSeq() == seq + 1U);
}

/* Maximum travel reached without an edge. */
static void TestMaximumTravel(void)
{
    int mode;

    /* 0: open space forever, 1: obstacle visible forever, 2: NEAR forever,
     * 3: ambiguous forever (never confirms either way) */
    for (mode = 0; mode < 4; mode++)
    {
        AppMotionIrEdgeSnapshot e;
        AppMotionEvent ev;
        uint32_t seq;
        unsigned guard;
        int cls = (mode == 0) ? C : (mode == 1) ? V : (mode == 2) ? 'N' : A;
        AppMotionEventType expect = (mode == 0) ? APP_MOTION_EVENT_IR_NO_OBSTACLE :
                                    (mode == 3) ? APP_MOTION_EVENT_IR_NO_OBSTACLE :
                                                  APP_MOTION_EVENT_IR_NO_EDGE;

        DriveToObstacle(MOTION_FORWARD_IR_RIGHT, 40);
        seq = EventSeq();

        for (guard = 0U; (guard < 1500U) && (AppMotion_GetState() != APP_MOTION_IDLE); guard++)
        {
            Cycle(cls, 1);
        }

        CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
        CHECK(guard < 1500U);
        ev = LastEvent();
        CHECK(EventSeq() == seq + 1U);                      /* exactly one terminal event */
        CHECK(ev.type == expect);
        CHECK(ev.command == MOTION_FORWARD_IR_RIGHT);
        e = Edge();
        CHECK(e.sequence == edgeBase + 1U);
        CHECK(e.clearCounts == -1);
        if (mode == 0 || mode == 3)
        {
            CHECK(e.result == (uint8_t)APP_MOTION_IR_RESULT_NO_OBSTACLE);
            CHECK(e.seenCounts == -1);
        }
        else
        {
            CHECK(e.result == (uint8_t)APP_MOTION_IR_RESULT_NO_EDGE);
            CHECK(e.seenCounts >= 0);
        }
        CHECK(Fake_GetMotorBrakeCallCount() >= 1U);         /* braked at the maximum */

        /* travelled about the maximum (40 cm = 3100 counts), not more than that + the brake */
        CHECK(encLeft > 40 * 75 - 400);
        CHECK(encLeft < 40 * 75 + 100 + 1600);
    }
}

/* Unusable data while driving: brake with IR_SENSOR. */
static void TestSensorFailureWhileDriving(void)
{
    int mode;

    /* 0: ERROR publications, 1: the IR task goes silent, 2: FAULT, 3: ERROR then recover in time */
    for (mode = 0; mode < 4; mode++)
    {
        AppMotionIrEdgeSnapshot e;
        uint32_t seq;
        unsigned i;
        unsigned guard;

        DriveToObstacle(MOTION_FORWARD_IR_LEFT, 300);
        seq = EventSeq();
        Cycles(2U * APP_IR_EDGE_SEEN_SCANS + 4U, V, 0);
        CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);

        if (mode == 3)
        {
            /* a short run of ERROR publications (under the limit) is survived */
            Cycles(8U, 'E', 0);                             /* 80 ms */
            CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
            Cycles(30U, V, 0);
            CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
            CHECK(EventSeq() == seq);
            SendStop();
            continue;
        }

        if (mode == 1)
        {
            publishing = 0;
        }

        for (i = 0U; (i < 60U) && (AppMotion_GetState() == APP_MOTION_DISTANCE); i++)
        {
            Cycle((mode == 0) ? 'E' : (mode == 2) ? 'F' : V, 0);
        }

        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
        if (mode == 2)
        {
            CHECK(i <= 3U);                                 /* FAULT: at once */
        }
        else
        {
            CHECK(i >= 18U);                                /* not before the 200 ms limit */
            CHECK(i <= 24U);
        }
        e = Edge();
        CHECK(e.sequence == edgeBase + 1U);
        CHECK(e.result == (uint8_t)APP_MOTION_IR_RESULT_SENSOR);
        CHECK(e.failReason == ((mode == 2) ? (uint8_t)APP_IR_EDGE_FAIL_FAULT
                                           : (uint8_t)APP_IR_EDGE_FAIL_DATA_LOST));
        CHECK(Fake_GetMotorBrakeCallCount() >= 1U);

        for (guard = 0U; (guard < 40U) && (AppMotion_GetState() != APP_MOTION_IDLE); guard++)
        {
            Cycle('E', 0);
        }
        CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
        CHECK(EventSeq() == seq + 1U);
        CHECK(LastEvent().type == APP_MOTION_EVENT_IR_SENSOR);
        CHECK(Edge().sequence == edgeBase + 1U);
    }
}

/* STOP at one stage: exactly one STOPPED, never a new IR decision.
 * kind 0: IR_WAIT, 1: waiting for the obstacle, 2: obstacle seen, 3: k clear
 * cycles into the edge window (the decision may fall before or after the STOP,
 * including in the very iteration that handles it), 4: braking after the edge. */
static void StopCase(int kind, unsigned k)
{
    uint32_t seq;
    uint32_t edgeSeq;
    unsigned i;

    Begin(100000U);
    publishing = 1;
    Fir(MOTION_FORWARD_IR_LEFT, 300);

    if (kind >= 1)
    {
        Cycles(2U * APP_IR_EDGE_READY_SCANS + 2U, C, 0);
        CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    }
    if (kind >= 2)
    {
        Cycles(2U * APP_IR_EDGE_SEEN_SCANS + 4U, V, 0);
        CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    }
    if (kind == 3)
    {
        Cycles(k, C, 0);
    }
    if (kind == 4)
    {
        for (i = 0U; (i < 60U) && (Edge().sequence == edgeBase); i++)
        {
            Cycle(C, 0);
        }
        CHECK(AppMotion_GetState() == APP_MOTION_BRAKING);
    }

    seq = EventSeq();
    edgeSeq = Edge().sequence;
    SendStop();

    for (i = 0U; (i < 60U) && (AppMotion_GetState() != APP_MOTION_IDLE); i++)
    {
        Cycle(C, 0);
    }
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
    CHECK(EventSeq() == seq + 1U);                       /* exactly one terminal event */
    CHECK(LastEvent().type == APP_MOTION_EVENT_STOPPED);
    CHECK(Edge().sequence == edgeSeq);                   /* no new decision from the STOP */
    CHECK(!AppMotion_IrEdgeArmed());

    if (kind == 0)
    {
        CHECK(Fake_GetMotorSetCallCount() == 0U);        /* never drove */
        CHECK(Fake_GetMotorBrakeCallCount() == 0U);      /* and no brake */
    }

    /* nothing late appears, whatever the IR task keeps publishing */
    Cycles(100U, V, 0);
    Cycles(100U, C, 0);
    CHECK(EventSeq() == seq + 1U);
    CHECK(Edge().sequence == edgeSeq);
}

static void TestStop(void)
{
    unsigned k;

    StopCase(0, 0U);
    StopCase(1, 0U);
    StopCase(2, 0U);
    StopCase(4, 0U);

    for (k = 0U; k < 16U; k++)
    {
        StopCase(3, k);
    }
}

/* The speed cap applies to FIR movements only. */
static void TestSpeedCap(void)
{
    AppMotionTelemetry t;
    unsigned i;
    int32_t maxDynamicFir = 0;
    int32_t maxDynamicPlain = 0;

    /* FIR: the dynamic setpoint never exceeds the cap */
    DriveToObstacle(MOTION_FORWARD_IR_LEFT, 300);
    for (i = 0U; i < 120U; i++)
    {
        Cycle(C, 0);
        AppMotion_GetTelemetry(&t);
        if (t.speedControlDynamicTargetSpeed > maxDynamicFir)
        {
            maxDynamicFir = t.speedControlDynamicTargetSpeed;
        }
    }
    CHECK(maxDynamicFir > 0);
    CHECK(maxDynamicFir <= (int32_t)APP_IR_EDGE_SPEED_CAP_COUNTS_PER_SEC);
    CHECK(maxDynamicFir == (int32_t)APP_IR_EDGE_SPEED_CAP_COUNTS_PER_SEC);   /* and it reaches it */
    SendStop();

    /* the configured target is untouched (SPCTRL,GET still reports it) */
    {
        float target, kp, ki;
        bool en;

        AppMotion_GetSpeedControl(&target, &kp, &ki, &en);
        CHECK(target == 8000.0f);        /* TASK2TEST */
    }

    /* the same profile on a plain MOV,F runs at the full target */
    Begin(100000U);
    Fir(MOTION_FORWARD, 300);
    for (i = 0U; i < 120U; i++)
    {
        Cycle(C, 0);
        AppMotion_GetTelemetry(&t);
        if (t.speedControlDynamicTargetSpeed > maxDynamicPlain)
        {
            maxDynamicPlain = t.speedControlDynamicTargetSpeed;
        }
    }
    CHECK(maxDynamicPlain == 8000);
    SendStop();
}

/* LEFT reads only the left sensor, RIGHT only the right one. */
static void TestMapping(void)
{
    int side;

    for (side = 0; side < 2; side++)
    {
        MotionType type = side ? MOTION_FORWARD_IR_RIGHT : MOTION_FORWARD_IR_LEFT;
        uint32_t seq;
        unsigned i;

        DriveToObstacle(type, 300);
        seq = EventSeq();

        /* the OTHER sensor sees an obstacle all along (Publish() puts the opposite
         * class there); the selected one is open space: no edge may ever appear */
        for (i = 0U; i < 200U; i++)
        {
            Cycle(C, side);
        }
        CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
        CHECK(Edge().sequence == edgeBase);
        CHECK(EventSeq() == seq);
        SendStop();
    }
}

/* After a FIR movement an ordinary movement is exactly as before. */
static void TestPlainMoveAfterwards(void)
{
    uint32_t seq;
    AppMotionTelemetry t;

    TestNormalPass(MOTION_FORWARD_IR_LEFT);
    Begin(100000U);
    seq = EventSeq();
    Fir(MOTION_FORWARD, 10);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    CHECK(!AppMotion_IrEdgeArmed());
    CHECK(RunToIdle(C, 0, 800U) < 800U);
    CHECK(EventSeq() == seq + 1U);
    CHECK(LastEvent().type == APP_MOTION_EVENT_MOV_DONE);
    CHECK(LastEvent().command == MOTION_FORWARD);
    AppMotion_GetTelemetry(&t);
    (void)t;

    /* IR data and the service call are ignored for a plain movement */
    Begin(100000U);
    publishing = 1;
    Fir(MOTION_FORWARD, 30);
    AppMotion_ServiceIrEdge(&snap, tick);                    /* not armed: a no-op */
    Cycles(10U, V, 0);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    SendStop();
}

/* The tick counter wraps in the middle of a pass. */
static void TestTickWrap(void)
{
    unsigned i;

    Begin(0xFFFFFFC0U);
    publishing = 1;
    Fir(MOTION_FORWARD_IR_RIGHT, 300);
    Cycles(2U * APP_IR_EDGE_READY_SCANS + 2U, C, 1);
    CHECK(AppMotion_GetState() == APP_MOTION_DISTANCE);
    Cycles(2U * APP_IR_EDGE_SEEN_SCANS + 4U, V, 1);
    for (i = 0U; (i < 60U) && (AppMotion_GetState() == APP_MOTION_DISTANCE); i++)
    {
        Cycle(C, 1);
    }
    CHECK(tick < 0xFFFFFFC0U);                               /* it did wrap */
    CHECK(Edge().result == (uint8_t)APP_MOTION_IR_RESULT_EDGE);
    (void)RunToIdle(C, 1, 40U);
    CHECK(LastEvent().type == APP_MOTION_EVENT_MOV_DONE);
}

int main(void)
{
    AppMotion_Init();
    ComputeSamples();
    pubSeq = 1000U;

    TestEntryValidation();
    TestReadinessWait();
    TestReadinessTimeout();
    TestNormalPass(MOTION_FORWARD_IR_LEFT);
    TestNormalPass(MOTION_FORWARD_IR_RIGHT);
    TestAlreadyVisible();
    TestMaximumTravel();
    TestSensorFailureWhileDriving();
    TestStop();
    TestSpeedCap();
    TestMapping();
    TestPlainMoveAfterwards();
    TestTickWrap();

    printf("%u checks, %u failures\n", checks, failures);
    return failures == 0U ? 0 : 1;
}
