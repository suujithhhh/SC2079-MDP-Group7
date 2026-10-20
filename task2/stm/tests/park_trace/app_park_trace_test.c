/* Host test of the measurement-only USPARK trace: ring, window state machine,
 * capacity proof, velocity estimate, zero-speed detector, timeouts, tick and
 * encoder wrap-around, and the compact text formatter. Pure: no HAL, no
 * RTOS; critical sections are counted by tests/park_trace/fakes/FreeRTOS.h.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -DAPP_USPARK_TRACE=1 -ICore/Inc \
 *       -Itests/park_trace/fakes -o tests/park_trace/app_park_trace_test.exe \
 *       tests/park_trace/app_park_trace_test.c Core/Src/app_park_trace.c \
 *       Core/Src/app_park_trace_fmt.c
 *   ./tests/park_trace/app_park_trace_test.exe
 *
 * A second build deliberately breaks the capacity proof to exercise the
 * ring-full / overwrite-protection / recovery paths (the caps are overridden
 * only by a host test, never in firmware):
 *   add  -DAPP_PARK_TRACE_ALLOW_OVERCOMMIT -DAPP_PARK_TRACE_US_CAP=70U \
 *        -DOVERCOMMIT_TEST
 */

#include "app_park_trace.h"

#include <stdio.h>
#include <string.h>

int FakeCrit_Depth;
int FakeCrit_MaxDepth;
unsigned FakeCrit_Enters;
unsigned FakeCrit_Underflows;

void FakeCrit_Enter(void)
{
    FakeCrit_Depth++;
    FakeCrit_Enters++;
    if (FakeCrit_Depth > FakeCrit_MaxDepth) { FakeCrit_MaxDepth = FakeCrit_Depth; }
}

void FakeCrit_Exit(void)
{
    if (FakeCrit_Depth == 0) { FakeCrit_Underflows++; }
    else { FakeCrit_Depth--; }
}

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

#define CHECK_STR(actual, expected) do { \
    checks++; \
    if (strcmp((actual), (expected)) != 0) { \
        failures++; \
        printf("FAIL line %d:\n   got      '%s'\n   expected '%s'\n", \
               __LINE__, (actual), (expected)); \
    } \
} while (0)

/* ---- helpers ----------------------------------------------------------- */

#define POPPED_MAX 80U
static AppParkTraceRecord popped[POPPED_MAX];
static unsigned poppedCount;

static void Reset(void)
{
    AppParkTrace_Init();
    FakeCrit_Depth = 0;
    FakeCrit_MaxDepth = 0;
    FakeCrit_Enters = 0U;
    FakeCrit_Underflows = 0U;
}

static unsigned PopAll(void)
{
    poppedCount = 0U;

    while ((poppedCount < POPPED_MAX) &&
           AppParkTrace_Pop(&popped[poppedCount]))
    {
        poppedCount++;
    }

    return poppedCount;
}

static unsigned CountEv(uint8_t ev)
{
    unsigned i;
    unsigned n = 0U;

    for (i = 0U; i < poppedCount; i++)
    {
        if (popped[i].ev == ev) { n++; }
    }

    return n;
}

static int IndexOf(uint8_t ev, unsigned nth)
{
    unsigned i;

    for (i = 0U; i < poppedCount; i++)
    {
        if (popped[i].ev == ev)
        {
            if (nth == 0U) { return (int)i; }
            nth--;
        }
    }

    return -1;
}

static const AppParkTraceRecord *First(uint8_t ev)
{
    int i = IndexOf(ev, 0U);

    return (i < 0) ? NULL : &popped[i];
}

static void Pub(uint32_t tick, uint16_t seq, uint16_t mm)
{
    AppParkTraceRecord r;

    AppParkTrace_InitRecord(&r, APP_PARK_EV_US, tick, 100, 200);
    r.seq = seq;
    r.raw = mm;
    r.filt = mm;
    r.age = 0U;
    r.pulse = (uint16_t)(mm * 5U);
    r.obs = 3U;
    (void)AppParkTrace_Submit(&r);
}

static void PubInvalid(uint32_t tick, uint16_t seq)
{
    AppParkTraceRecord r;

    AppParkTrace_InitRecord(&r, APP_PARK_EV_US, tick, 100, 200);
    r.seq = seq;
    r.obs = 2U;
    (void)AppParkTrace_Submit(&r);     /* raw/filt/age/pulse stay NA */
}

static void Ev(uint8_t ev, uint32_t tick, int32_t el, int32_t er)
{
    AppParkTraceRecord r;

    AppParkTrace_InitRecord(&r, ev, tick, el, er);
    (void)AppParkTrace_Submit(&r);
}

static void Arm(uint32_t tick, uint16_t w)
{
    AppParkTraceRecord r;

    AppParkTrace_InitRecord(&r, APP_PARK_EV_ARM, tick, 111, 222);
    r.w = w;
    r.seq = 2U;
    r.filt = 480U;
    r.flags = 2U;
    (void)AppParkTrace_Submit(&r);
}

static void Near(uint32_t tick, uint8_t k)
{
    AppParkTraceRecord r;

    AppParkTrace_InitRecord(&r, APP_PARK_EV_NEAR, tick, 1, 2);
    r.arg = k;
    r.seq = (uint16_t)(2U + k);
    r.filt = 290U;
    (void)AppParkTrace_Submit(&r);
}

static void Req(uint32_t tick, int32_t el, int32_t er)
{
    AppParkTraceRecord r;

    AppParkTrace_InitRecord(&r, APP_PARK_EV_REQ, tick, el, er);
    r.arg = APP_PARK_CAUSE_NEAR;
    r.seq = 5U;
    r.obs = 11U;
    (void)AppParkTrace_Submit(&r);
}

/* Wheel model: moves `perStep` counts every 10 ms until `stopAfterMs`, then
 * holds (optionally toggling by one count every 10 ms: encoder dither). */
typedef struct
{
    int32_t start;
    int32_t perStep;
    uint32_t stopAfterMs;
    int32_t dither;
} Wheel;

static int32_t WPos(const Wheel *w, uint32_t dtMs)
{
    uint32_t m = (dtMs < w->stopAfterMs) ? dtMs : w->stopAfterMs;
    uint32_t p = (uint32_t)w->start + (uint32_t)(w->perStep * (int32_t)(m / 10U));

    if ((dtMs >= w->stopAfterMs) && (w->dither != 0))
    {
        p += (dtMs / 10U) & 1U;
    }

    return (int32_t)p;
}

/* What the 16-bit TIM3 path hands to the trace: wrapped to int16. */
static int32_t Raw16(int32_t logical)
{
    return (int32_t)(int16_t)(uint16_t)(uint32_t)logical;
}

static void RunSamples(uint32_t brk, const Wheel *l, const Wheel *r,
                       uint32_t fromMs, uint32_t toMs, uint32_t stepMs)
{
    uint32_t t;

    for (t = fromMs; t <= toMs; t += stepMs)
    {
        AppParkTrace_Sample(brk + t, WPos(l, t), Raw16(WPos(r, t)));
    }
}

/* Drives a complete automatic stop up to and including BRK. Returns the BRK
 * tick. Approach speed 8 counts/ms on both wheels from `base`. */
static uint32_t ApproachToBrake(uint32_t base, uint16_t w, uint32_t *brkOut)
{
    uint32_t t;
    uint32_t brk = base + 290U;

    Pub(base + 0U, 1U, 900U);
    Pub(base + 70U, 2U, 480U);              /* arms (<= 500) */
    Arm(base + 71U, w);

    /* MotionTask samples every 10 ms from the moment the window is open. */
    for (t = 80U; t < 280U; t += 10U)
    {
        AppParkTrace_Sample(base + t, (int32_t)(8U * t), Raw16((int32_t)(8U * t)));

        if (t == 140U) { Pub(base + 140U, 3U, 290U); Near(base + 141U, 1U); }
        if (t == 210U) { Pub(base + 210U, 4U, 280U); Near(base + 211U, 2U); }
    }

    AppParkTrace_Sample(base + 280U, 8 * 280, Raw16(8 * 280));
    Pub(base + 280U, 5U, 270U);
    Near(base + 281U, 3U);
    Req(base + 281U, 8 * 281, Raw16(8 * 281));
    Ev(APP_PARK_EV_ACC, base + 282U, 8 * 282, Raw16(8 * 282));
    Ev(APP_PARK_EV_TAKE, base + 290U, 8 * 290, Raw16(8 * 290));
    Ev(APP_PARK_EV_BRK, brk, 8 * 290, Raw16(8 * 290));

    if (brkOut != NULL) { *brkOut = brk; }
    return brk;
}

/* ---- tests ------------------------------------------------------------- */

static void test_layout_and_helpers(void)
{
    AppParkTraceRecord r;

    CHECK_EQ(sizeof(AppParkTraceRecord), 36);

    AppParkTrace_InitRecord(&r, APP_PARK_EV_US, 5U, -3, 9);
    CHECK_EQ(r.tick, 5);
    CHECK_EQ(r.el, -3);
    CHECK_EQ(r.er, 9);
    CHECK_EQ(r.ev, APP_PARK_EV_US);
    CHECK_EQ(r.raw, APP_PARK_TRACE_NA_U16);
    CHECK_EQ(r.filt, APP_PARK_TRACE_NA_U16);
    CHECK_EQ(r.age, APP_PARK_TRACE_NA_U16);
    CHECK_EQ(r.pulse, APP_PARK_TRACE_NA_U16);
    CHECK_EQ(r.obs, APP_PARK_TRACE_NA_U16);
    CHECK_EQ(r.vl, APP_PARK_TRACE_NA_V);
    CHECK_EQ(r.vr, APP_PARK_TRACE_NA_V);
    CHECK_EQ(r.n, 0);

    /* tick difference is wrap-safe */
    CHECK_EQ(AppParkTrace_TickDelta(10U, 4294967290U), 16);
    CHECK_EQ(AppParkTrace_TickDelta(0U, 0xFFFFFFFFU), 1);
    CHECK_EQ(AppParkTrace_TickDelta(5U, 5U), 0);

    /* left (32-bit) difference crosses INT32_MAX / INT32_MIN */
    CHECK_EQ(AppParkTrace_Delta32(INT32_MAX - 2, INT32_MIN + 3), 6);
    CHECK_EQ(AppParkTrace_Delta32(INT32_MIN + 3, INT32_MAX - 2), -6);
    CHECK_EQ(AppParkTrace_Delta32(-1, 1), 2);
    CHECK_EQ(AppParkTrace_Delta32(100, 100), 0);

    /* right (16-bit) difference crosses 32767 / -32768 in both directions */
    CHECK_EQ(AppParkTrace_Delta16(32766, -32767), 3);
    CHECK_EQ(AppParkTrace_Delta16(-32767, 32766), -3);
    CHECK_EQ(AppParkTrace_Delta16(-5, 5), 10);
    /* AppEncoder_Right() may report +32768 for raw -32768: same modulo 2^16 */
    CHECK_EQ(AppParkTrace_Delta16(32767, 32768), 1);
    CHECK_EQ(AppParkTrace_Delta16(32768, -32768), 0);
    CHECK_EQ(AppParkTrace_Delta16(32768, -32767), 1);
}

static void test_event_order_and_pre_replay(void)
{
    uint32_t brk;
    const AppParkTraceRecord *r;
    unsigned i;

    Reset();
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_IDLE);

    /* Five valid publications before ARM; two invalid ones in between. */
    Pub(1000U, 1U, 900U);
    PubInvalid(1070U, 2U);
    Pub(1140U, 3U, 800U);
    Pub(1210U, 4U, 700U);
    PubInvalid(1280U, 5U);
    Pub(1350U, 6U, 480U);                       /* the arming publication */

    /* Nothing is recorded before ARM. */
    CHECK_EQ(PopAll(), 0);
    CHECK(!AppParkTrace_DrainReady());

    Arm(1351U, 42U);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_ARMED);

    Pub(1420U, 7U, 450U);
    Near(1421U, 1U);
    Req(1500U, 5000, 5001);
    Ev(APP_PARK_EV_ACC, 1501U, 5002, 5003);
    Ev(APP_PARK_EV_TAKE, 1510U, 5004, 5005);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_TAKE);
    Ev(APP_PARK_EV_BRK, 1510U, 5006, 5007);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_BRAKE);
    brk = 1510U;

    /* Nothing is visible before DONE. */
    CHECK(!AppParkTrace_DrainReady());
    CHECK_EQ(PopAll(), 0);

    {
        Wheel l = { 5006, 5, 20U, 0 };
        Wheel rr = { 5007, 5, 20U, 0 };

        RunSamples(brk, &l, &rr, 10U, 100U, 10U);
        Ev(APP_PARK_EV_BEND, brk + 121U, 5200, 5200);
        RunSamples(brk, &l, &rr, 110U, 130U, 10U);
    }

    CHECK(AppParkTrace_DrainReady());
    CHECK_EQ(PopAll(), 15);

    /* The last three VALID publications before ARM, oldest first, as PRE. */
    CHECK_EQ(popped[0].ev, APP_PARK_EV_PRE);
    CHECK_EQ(popped[0].seq, 3);
    CHECK_EQ(popped[1].ev, APP_PARK_EV_PRE);
    CHECK_EQ(popped[1].seq, 4);
    CHECK_EQ(popped[2].ev, APP_PARK_EV_PRE);
    CHECK_EQ(popped[2].seq, 6);
    CHECK_EQ(popped[3].ev, APP_PARK_EV_ARM);
    CHECK_EQ(popped[3].w, 42);
    CHECK_EQ(CountEv(APP_PARK_EV_PRE), 3);

    /* Exact event order and contiguous indices from 1. */
    {
        static const uint8_t expected[] = {
            APP_PARK_EV_PRE, APP_PARK_EV_PRE, APP_PARK_EV_PRE,
            APP_PARK_EV_ARM, APP_PARK_EV_US, APP_PARK_EV_NEAR,
            APP_PARK_EV_REQ, APP_PARK_EV_ACC, APP_PARK_EV_TAKE,
            APP_PARK_EV_BRK, APP_PARK_EV_ZL, APP_PARK_EV_ZR,
            APP_PARK_EV_ZERO, APP_PARK_EV_BEND, APP_PARK_EV_DONE
        };

        for (i = 0U; (i < poppedCount) && (i < sizeof(expected)); i++)
        {
            CHECK_EQ(popped[i].ev, expected[i]);
        }
    }

    for (i = 0U; i < poppedCount; i++)
    {
        CHECK_EQ(popped[i].n, i + 1U);
    }

    /* Event ticks: ZL/ZR/ZERO at BRK + 70 (stationary from +20, window +50). */
    CHECK_EQ(First(APP_PARK_EV_ZL)->tick, brk + 70U);
    CHECK_EQ(First(APP_PARK_EV_ZERO)->tick, brk + 70U);
    CHECK_EQ(First(APP_PARK_EV_BEND)->tick, brk + 121U);
    CHECK_EQ(First(APP_PARK_EV_DONE)->tick, brk + 121U);
    /* Publication-time ticks of the replayed history are kept untouched. */
    CHECK_EQ(popped[0].tick, 1140);
    CHECK_EQ(popped[1].tick, 1210);
    CHECK_EQ(popped[2].tick, 1350);

    CHECK_EQ(popped[poppedCount - 1U].ev, APP_PARK_EV_DONE);
    r = First(APP_PARK_EV_DONE);
    CHECK(r != NULL);
    CHECK_EQ(r->arg, APP_PARK_RES_OK);
    CHECK_EQ(r->w, 42);
    CHECK_EQ(r->raw, 0);                         /* dropped */
    CHECK_EQ(r->filt, 0);                        /* ring-full drops */
    CHECK_EQ(r->fm, 0);                          /* skipped windows */
    CHECK_EQ(r->flags, 0);

    /* The window is over once everything has been drained. */
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_IDLE);
    CHECK(!AppParkTrace_DrainReady());
    CHECK_EQ(FakeCrit_Depth, 0);
    CHECK_EQ(FakeCrit_Underflows, 0);
    CHECK(FakeCrit_MaxDepth <= 1);               /* the module never nests */
}

static void test_pre_replay_fewer_than_three(void)
{
    Reset();
    Pub(100U, 1U, 480U);                        /* only one valid so far */
    Arm(101U, 3U);
    Ev(APP_PARK_EV_BRK, 150U, 0, 0);            /* ignored: not in TAKE */
    AppParkTrace_MovementEnded(200U);
    CHECK_EQ(PopAll(), 3);
    CHECK_EQ(popped[0].ev, APP_PARK_EV_PRE);
    CHECK_EQ(popped[1].ev, APP_PARK_EV_ARM);
    CHECK_EQ(popped[2].ev, APP_PARK_EV_DONE);

    /* No valid publication at all: ARM alone (arming from confirmedNear). */
    Reset();
    PubInvalid(100U, 1U);
    Arm(101U, 3U);
    AppParkTrace_MovementEnded(200U);
    CHECK_EQ(PopAll(), 2);
    CHECK_EQ(popped[0].ev, APP_PARK_EV_ARM);
    CHECK_EQ(popped[1].ev, APP_PARK_EV_DONE);
}

static void test_out_of_order_and_duplicates_ignored(void)
{
    Reset();

    /* Nothing is accepted without an open window. */
    Ev(APP_PARK_EV_NEAR, 10U, 0, 0);
    Ev(APP_PARK_EV_REQ, 11U, 0, 0);
    Ev(APP_PARK_EV_ACC, 12U, 0, 0);
    Ev(APP_PARK_EV_TAKE, 13U, 0, 0);
    Ev(APP_PARK_EV_BRK, 14U, 0, 0);
    Ev(APP_PARK_EV_BEND, 15U, 0, 0);
    Ev(APP_PARK_EV_DONE, 16U, 0, 0);
    Ev(APP_PARK_EV_ZERO, 17U, 0, 0);
    Ev(APP_PARK_EV_NONE, 18U, 0, 0);
    Ev(200, 19U, 0, 0);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_IDLE);
    CHECK_EQ(PopAll(), 0);

    /* Each step needs its own predecessor. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);
    Ev(APP_PARK_EV_ACC, 110U, 0, 0);            /* no REQ yet */
    Ev(APP_PARK_EV_TAKE, 111U, 0, 0);
    Ev(APP_PARK_EV_BRK, 112U, 0, 0);
    Ev(APP_PARK_EV_BEND, 113U, 0, 0);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_ARMED);
    Req(120U, 0, 0);
    Req(121U, 0, 0);                            /* duplicate REQ */
    Ev(APP_PARK_EV_TAKE, 122U, 0, 0);           /* no ACC yet */
    Ev(APP_PARK_EV_ACC, 123U, 0, 0);
    Ev(APP_PARK_EV_ACC, 124U, 0, 0);            /* duplicate ACC */
    Ev(APP_PARK_EV_BRK, 125U, 0, 0);            /* no TAKE yet */
    Ev(APP_PARK_EV_TAKE, 126U, 0, 0);
    Ev(APP_PARK_EV_TAKE, 127U, 0, 0);           /* duplicate TAKE */
    Ev(APP_PARK_EV_BRK, 128U, 0, 0);
    Ev(APP_PARK_EV_BRK, 129U, 0, 0);            /* duplicate BRK */
    Ev(APP_PARK_EV_BEND, 130U, 0, 0);
    Ev(APP_PARK_EV_BEND, 131U, 0, 0);           /* duplicate BEND */
    Ev(APP_PARK_EV_ARM, 132U, 0, 0);            /* re-arm inside the window */
    AppParkTrace_Poll(128U + 2500U);           /* BRAKE bounded wait */
    CHECK_EQ(PopAll(), 8);       /* PRE ARM REQ ACC TAKE BRK BEND DONE */
    CHECK_EQ(CountEv(APP_PARK_EV_REQ), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_ACC), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_TAKE), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_BRK), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_BEND), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_ARM), 1);
    CHECK_EQ(First(APP_PARK_EV_DONE)->raw, 0);   /* nothing counted as dropped */
}

static void test_manual_stop_and_non_automatic_paths_record_nothing(void)
{
    /* A manual STOP produces no REQ/ACC: MotionTask's TAKE/BRK/BEND hooks fire
     * while the window is merely ARMED and must record nothing. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 5U);
    Ev(APP_PARK_EV_TAKE, 200U, 1, 1);
    Ev(APP_PARK_EV_BRK, 200U, 1, 1);
    AppParkTrace_Sample(210U, 2, 2);
    AppParkTrace_Sample(260U, 2, 2);
    Ev(APP_PARK_EV_BEND, 330U, 3, 3);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_ARMED);
    AppParkTrace_MovementEnded(340U);
    CHECK_EQ(PopAll(), 3);                       /* PRE ARM DONE */
    CHECK_EQ(First(APP_PARK_EV_DONE)->arg, APP_PARK_RES_NOREQ);
    CHECK_EQ(CountEv(APP_PARK_EV_BRK), 0);
    CHECK_EQ(CountEv(APP_PARK_EV_ZL), 0);
    CHECK_EQ(CountEv(APP_PARK_EV_ZERO), 0);

    /* USPARK disabled while merely armed: ABORT; while a stop is in progress:
     * the stop is never interrupted. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 5U);
    AppParkTrace_Disabled(150U);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_DRAIN);
    CHECK_EQ(PopAll(), 3);
    CHECK_EQ(First(APP_PARK_EV_DONE)->arg, APP_PARK_RES_ABORT);

    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 5U);
    Req(120U, 0, 0);
    AppParkTrace_Disabled(150U);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_REQ);
    Reset();
    AppParkTrace_Disabled(10U);                  /* idle: nothing */
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_IDLE);

    /* Refused guarded request. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 5U);
    Req(120U, 0, 0);
    AppParkTrace_Refused(121U);
    CHECK_EQ(PopAll(), 4);
    CHECK_EQ(First(APP_PARK_EV_DONE)->arg, APP_PARK_RES_REFUSED);

    /* Movement ended after REQ but before ACC. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 5U);
    Req(120U, 0, 0);
    AppParkTrace_MovementEnded(130U);
    CHECK_EQ(PopAll(), 4);
    CHECK_EQ(First(APP_PARK_EV_DONE)->arg, APP_PARK_RES_NOACC);

    /* Finish outside a window is ignored. */
    Reset();
    AppParkTrace_MovementEnded(5U);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_IDLE);
    CHECK_EQ(PopAll(), 0);

    /* The race CommTask can lose: it judged the movement "over" from a status
     * it read earlier, but MotionTask has meanwhile taken the stop. A stop
     * that is TAKEn or BRAKing must never be cut short by that observation,
     * and a refusal or a disable must not either. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);
    Req(120U, 0, 0);
    Ev(APP_PARK_EV_ACC, 121U, 0, 0);
    Ev(APP_PARK_EV_TAKE, 130U, 0, 0);
    AppParkTrace_MovementEnded(131U);
    AppParkTrace_Refused(131U);
    AppParkTrace_Disabled(131U);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_TAKE);
    Ev(APP_PARK_EV_BRK, 132U, 0, 0);
    AppParkTrace_MovementEnded(133U);
    AppParkTrace_Refused(133U);
    AppParkTrace_Disabled(133U);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_BRAKE);

    /* ... while a window that has only a REQ or only an ACC does close. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);
    Req(120U, 0, 0);
    Ev(APP_PARK_EV_ACC, 121U, 0, 0);
    AppParkTrace_Refused(122U);                     /* ACC state: not refusable */
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_ACC);
    AppParkTrace_MovementEnded(123U);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_DRAIN);
    PopAll();
    CHECK_EQ(First(APP_PARK_EV_DONE)->arg, APP_PARK_RES_NOACC);
}

static void test_velocity_estimate(void)
{
    uint32_t brk;
    const AppParkTraceRecord *r;

    Reset();
    ApproachToBrake(10000U, 9U, &brk);

    {
        Wheel l = { 8 * 290, 0, 0U, 0 };
        Wheel rr = { 8 * 290, 0, 0U, 0 };

        RunSamples(brk, &l, &rr, 10U, 70U, 10U);
        Ev(APP_PARK_EV_BEND, brk + 121U, 2320, 2320);
        RunSamples(brk, &l, &rr, 80U, 130U, 10U);
    }

    CHECK_EQ(PopAll() > 0U, 1);
    r = First(APP_PARK_EV_REQ);
    CHECK(r != NULL);
    CHECK_EQ(r->vl, 8000);                      /* 8 counts/ms */
    CHECK_EQ(r->vr, 8000);
    r = First(APP_PARK_EV_BRK);
    CHECK(r != NULL);
    CHECK_EQ(r->vl, 8000);
    CHECK_EQ(r->vr, 8000);

    /* Other events never carry a velocity. */
    r = First(APP_PARK_EV_ACC);
    CHECK_EQ(r->vl, APP_PARK_TRACE_NA_V);

    /* Fewer than two samples: not available, never zero. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);
    Req(110U, 0, 0);
    PopAll();
    AppParkTrace_MovementEnded(120U);
    PopAll();
    CHECK_EQ(First(APP_PARK_EV_REQ)->vl, APP_PARK_TRACE_NA_V);

    /* Forward through BOTH the tick wrap and the encoder wraps: four samples
     * 16 ms apart, +80 left and +60 right counts per step. Logical left
     * INT32_MAX-100 .. +240 crosses INT32_MAX; logical right 32700 .. +180
     * crosses 32767 and is seen by the trace as a wrapped 16-bit value. */
    Reset();
    Pub(0xFFFFFF00U, 1U, 480U);
    Arm(0xFFFFFF01U, 1U);
    AppParkTrace_Sample(0xFFFFFFE0U, INT32_MAX - 100,
                        Raw16(32700));
    AppParkTrace_Sample(0xFFFFFFF0U, (int32_t)((uint32_t)INT32_MAX - 20U),
                        Raw16(32760));
    AppParkTrace_Sample(0x00000000U, (int32_t)((uint32_t)INT32_MAX + 60U),
                        Raw16(32820));
    AppParkTrace_Sample(0x00000010U, (int32_t)((uint32_t)INT32_MAX + 140U),
                        Raw16(32880));
    Req(0x00000011U, 0, 0);
    AppParkTrace_MovementEnded(0x00000012U);
    PopAll();
    r = First(APP_PARK_EV_REQ);
    CHECK(r != NULL);
    CHECK_EQ(r->vl, (240 * 1000) / 48);          /* +240 counts in 48 ms */
    CHECK_EQ(r->vr, (180 * 1000) / 48);          /* +180 counts in 48 ms */

    /* Backwards through the same wraps: negative velocity. */
    Reset();
    Pub(0xFFFFFF00U, 1U, 480U);
    Arm(0xFFFFFF01U, 1U);
    AppParkTrace_Sample(0xFFFFFFE0U, (int32_t)((uint32_t)INT32_MIN + 100U),
                        Raw16(-32700));
    AppParkTrace_Sample(0xFFFFFFF0U, (int32_t)((uint32_t)INT32_MIN + 20U),
                        Raw16(-32760));
    AppParkTrace_Sample(0x00000000U, (int32_t)((uint32_t)INT32_MIN - 60U),
                        Raw16(-32820));
    AppParkTrace_Sample(0x00000010U, (int32_t)((uint32_t)INT32_MIN - 140U),
                        Raw16(-32880));
    Req(0x00000011U, 0, 0);
    AppParkTrace_MovementEnded(0x00000012U);
    PopAll();
    r = First(APP_PARK_EV_REQ);
    CHECK(r != NULL);
    CHECK_EQ(r->vl, (-240 * 1000) / 48);
    CHECK_EQ(r->vr, (-180 * 1000) / 48);

    /* Clamped to +-9999 counts/s. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);
    AppParkTrace_Sample(110U, 1000, 1000);
    AppParkTrace_Sample(120U, 900, 1100);
    AppParkTrace_Sample(130U, 800, 1200);
    Req(131U, 0, 0);
    AppParkTrace_MovementEnded(132U);
    PopAll();
    CHECK_EQ(First(APP_PARK_EV_REQ)->vl, -9999);
    CHECK_EQ(First(APP_PARK_EV_REQ)->vr, 9999);

    /* A stale estimate (samples > 1 s apart) is not reported. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);
    AppParkTrace_Sample(110U, 0, 0);
    AppParkTrace_Sample(2000U, 100, 100);
    Req(2001U, 0, 0);
    AppParkTrace_MovementEnded(2002U);
    PopAll();
    CHECK_EQ(First(APP_PARK_EV_REQ)->vl, APP_PARK_TRACE_NA_V);
}

/* ---- BRK / BEND latched from deep inside AppMotion_Update() ----------- */

static void test_latch_path(void)
{
    AppParkTraceStats st;
    const AppParkTraceRecord *r;

    /* BRK is only latched: nothing happens until the next Sample(), and the
     * record carries the LATCHED tick and counts, not the sample's. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);
    Req(120U, 0, 0);
    Ev(APP_PARK_EV_ACC, 121U, 0, 0);
    Ev(APP_PARK_EV_TAKE, 130U, 0, 0);
    AppParkTrace_Latch(APP_PARK_EV_BRK, 1000U, 111, 222);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_TAKE);
    CHECK_EQ(FakeCrit_Depth, 0);
    AppParkTrace_Sample(1003U, 120, 230);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_BRAKE);

    /* BEND latched, then flushed by the next sample; zero confirmed first. */
    {
        Wheel l = { 120, 0, 0U, 0 };
        Wheel rr = { 230, 0, 0U, 0 };
        uint32_t t;

        for (t = 1013U; t <= 1103U; t += 10U)
        {
            AppParkTrace_Sample(t, WPos(&l, 0), Raw16(WPos(&rr, 0)));
        }
        CHECK(!AppParkTrace_DrainReady());          /* zero seen, BEND not yet */
        AppParkTrace_Latch(APP_PARK_EV_BEND, 1121U, 120, 230);
        CHECK(!AppParkTrace_DrainReady());          /* still only latched */
        AppParkTrace_Sample(1123U, 120, 230);
        CHECK(AppParkTrace_DrainReady());
    }

    PopAll();
    r = First(APP_PARK_EV_BRK);
    CHECK(r != NULL);
    CHECK_EQ(r->tick, 1000);
    CHECK_EQ(r->el, 111);
    CHECK_EQ(r->er, 222);
    r = First(APP_PARK_EV_BEND);
    CHECK(r != NULL);
    CHECK_EQ(r->tick, 1121);
    CHECK(IndexOf(APP_PARK_EV_BRK, 0) < IndexOf(APP_PARK_EV_ZERO, 0));
    CHECK(IndexOf(APP_PARK_EV_ZERO, 0) < IndexOf(APP_PARK_EV_BEND, 0));
    CHECK_EQ(First(APP_PARK_EV_DONE)->arg, APP_PARK_RES_OK);
    CHECK_EQ(FakeCrit_Depth, 0);

    /* Both latched in one cycle (BEND first): BRK is still submitted first. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);
    Req(120U, 0, 0);
    Ev(APP_PARK_EV_ACC, 121U, 0, 0);
    Ev(APP_PARK_EV_TAKE, 130U, 0, 0);
    AppParkTrace_Latch(APP_PARK_EV_BEND, 1121U, 5, 5);
    AppParkTrace_Latch(APP_PARK_EV_BRK, 1000U, 5, 5);
    AppParkTrace_Sample(1122U, 500, 500);          /* still rolling */
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_BRAKE);
    AppParkTrace_Poll(1000U + 2500U);
    PopAll();
    CHECK(IndexOf(APP_PARK_EV_BRK, 0) >= 0);
    CHECK(IndexOf(APP_PARK_EV_BRK, 0) < IndexOf(APP_PARK_EV_BEND, 0));

    /* Nothing traced (manual STOP / natural brake / no window): the latch is
     * consumed and ignored, and the module stays idle. */
    Reset();
    AppParkTrace_Latch(APP_PARK_EV_BRK, 10U, 1, 1);
    AppParkTrace_Latch(APP_PARK_EV_BEND, 20U, 1, 1);
    AppParkTrace_Sample(30U, 1, 1);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_IDLE);
    CHECK_EQ(PopAll(), 0);
    AppParkTrace_Latch(APP_PARK_EV_BRK, 40U, 1, 1);    /* slot is free again */
    AppParkTrace_Sample(50U, 1, 1);
    AppParkTrace_GetStats(&st);
    CHECK_EQ(st.lifetimeDropped, 0);

    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);                                      /* ARMED, no REQ */
    AppParkTrace_Latch(APP_PARK_EV_BRK, 200U, 1, 1);    /* manual STOP brake */
    AppParkTrace_Sample(210U, 1, 1);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_ARMED);
    AppParkTrace_Latch(APP_PARK_EV_BEND, 330U, 1, 1);
    AppParkTrace_Sample(340U, 1, 1);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_ARMED);
    AppParkTrace_MovementEnded(350U);
    CHECK_EQ(PopAll(), 3);                              /* PRE ARM DONE */

    /* An unflushed slot written again is counted as a drop, not overwritten. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);
    Req(120U, 0, 0);
    Ev(APP_PARK_EV_ACC, 121U, 0, 0);
    Ev(APP_PARK_EV_TAKE, 130U, 0, 0);
    AppParkTrace_Latch(APP_PARK_EV_BRK, 1000U, 7, 8);
    AppParkTrace_Latch(APP_PARK_EV_BRK, 1001U, 9, 9);  /* overrun */
    AppParkTrace_Latch(200U, 1002U, 9, 9);             /* unknown event */
    AppParkTrace_Sample(1010U, 7, 8);
    AppParkTrace_GetStats(&st);
    CHECK_EQ(st.lifetimeDropped, 1);
    CHECK_EQ(st.windowDropped, 1);
    AppParkTrace_Poll(1000U + 2500U);
    PopAll();
    CHECK_EQ(First(APP_PARK_EV_BRK)->tick, 1000);       /* the first one kept */
    CHECK_EQ(First(APP_PARK_EV_DONE)->raw, 1);
    CHECK_EQ(FakeCrit_Depth, 0);
    CHECK_EQ(FakeCrit_Underflows, 0);
}

/* ---- zero-speed detector ---------------------------------------------- */

static unsigned ZeroRun(uint32_t brk, const Wheel *l, const Wheel *rr,
                        uint32_t untilMs, uint32_t stepMs,
                        uint32_t bendAtMs)
{
    uint32_t t;
    bool bendDone = false;

    for (t = stepMs; t <= untilMs; t += stepMs)
    {
        if (!bendDone && (t >= bendAtMs))
        {
            Ev(APP_PARK_EV_BEND, brk + bendAtMs, 0, 0);
            bendDone = true;
        }
        AppParkTrace_Sample(brk + t, WPos(l, t), Raw16(WPos(rr, t)));
    }

    if (!bendDone) { Ev(APP_PARK_EV_BEND, brk + bendAtMs, 0, 0); }
    return PopAll();
}

static void OpenStop(uint32_t brk, int32_t el, int32_t er)
{
    Pub(brk - 300U, 1U, 480U);
    Arm(brk - 290U, 1U);
    Req(brk - 10U, el, er);
    Ev(APP_PARK_EV_ACC, brk - 9U, el, er);
    Ev(APP_PARK_EV_TAKE, brk - 1U, el, er);
    Ev(APP_PARK_EV_BRK, brk, el, er);
}

static void test_zero_confirmation_per_wheel(void)
{
    const AppParkTraceRecord *zl;
    const AppParkTraceRecord *zr;
    const AppParkTraceRecord *zero;

    /* Left holds from 30 ms, right from 60 ms; 10 ms cadence. */
    Reset();
    OpenStop(1000U, 10000, 5000);
    {
        Wheel l = { 10000, 50, 30U, 0 };
        Wheel rr = { 5000, 50, 60U, 0 };

        (void)ZeroRun(1000U, &l, &rr, 300U, 10U, 121U);
    }
    zl = First(APP_PARK_EV_ZL);
    zr = First(APP_PARK_EV_ZR);
    zero = First(APP_PARK_EV_ZERO);
    CHECK(zl && zr && zero);
    CHECK_EQ(zl->tick, 1080);            /* still since 1030, window full at +50 */
    CHECK_EQ(zr->tick, 1110);
    CHECK_EQ(zero->tick, 1110);
    CHECK_EQ(zero->flags, 0);
    CHECK_EQ(zl->el, 10150);             /* left held at 10000 + 3 * 50 */
    CHECK_EQ(zr->er, 5300);              /* right held at 5000 + 6 * 50 */
    /* distinct, left strictly before right, ZERO after both */
    CHECK(IndexOf(APP_PARK_EV_ZL, 0) < IndexOf(APP_PARK_EV_ZR, 0));
    CHECK(IndexOf(APP_PARK_EV_ZR, 0) < IndexOf(APP_PARK_EV_ZERO, 0));
    CHECK_EQ(CountEv(APP_PARK_EV_ZL), 1);        /* first time only */
    CHECK_EQ(CountEv(APP_PARK_EV_ZR), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_ZERO), 1);
    CHECK_EQ(First(APP_PARK_EV_DONE)->arg, APP_PARK_RES_OK);

    /* Right stops first: ZR is recorded before ZL, ZERO at the later one. */
    Reset();
    OpenStop(1000U, 10000, 5000);
    {
        Wheel l = { 10000, 50, 60U, 0 };
        Wheel rr = { 5000, 50, 30U, 0 };

        (void)ZeroRun(1000U, &l, &rr, 300U, 10U, 121U);
    }
    CHECK(IndexOf(APP_PARK_EV_ZR, 0) < IndexOf(APP_PARK_EV_ZL, 0));
    CHECK_EQ(First(APP_PARK_EV_ZR)->tick, 1080);
    CHECK_EQ(First(APP_PARK_EV_ZL)->tick, 1110);
    CHECK_EQ(First(APP_PARK_EV_ZERO)->tick, 1110);

    /* Both already stationary at BRK: earliest confirmation is BRK + 50. */
    Reset();
    OpenStop(1000U, 10000, 5000);
    {
        Wheel l = { 10000, 0, 0U, 0 };
        Wheel rr = { 5000, 0, 0U, 0 };

        (void)ZeroRun(1000U, &l, &rr, 200U, 10U, 121U);
    }
    CHECK_EQ(First(APP_PARK_EV_ZL)->tick, 1050);
    CHECK_EQ(First(APP_PARK_EV_ZR)->tick, 1050);
    CHECK_EQ(First(APP_PARK_EV_ZERO)->tick, 1050);
}

static void test_zero_definition_boundaries(void)
{
    /* One count of dither is still zero: the first confirmation is exact. */
    Reset();
    OpenStop(1000U, 100, 100);
    {
        Wheel l = { 100, 0, 0U, 1 };
        Wheel rr = { 100, 0, 0U, 1 };

        (void)ZeroRun(1000U, &l, &rr, 200U, 10U, 121U);
    }
    CHECK_EQ(First(APP_PARK_EV_ZERO)->tick, 1050);

    /* A creep of one count every 10 ms (5 counts over 50 ms) is not zero
     * while it lasts. It stops at +100 (value 10); the first window whose
     * counts differ by at most one is [90..140] = {9, 10, 10, ...}. */
    Reset();
    OpenStop(1000U, 0, 0);
    {
        Wheel l = { 0, 1, 100U, 0 };       /* +1 per 10 ms until 100 ms */
        Wheel rr = { 0, 1, 100U, 0 };

        (void)ZeroRun(1000U, &l, &rr, 400U, 10U, 121U);
    }
    CHECK_EQ(First(APP_PARK_EV_ZERO)->tick, 1140);

    /* A single 2-count step 40 ms ago blocks confirmation until it is 50 ms
     * old: moves 2 counts at t=+20, so first stillness at t=+70 + ... */
    Reset();
    OpenStop(1000U, 0, 0);
    {
        Wheel l = { 0, 2, 10U, 0 };         /* 2 counts at +10 then holds */
        Wheel rr = { 0, 0, 0U, 0 };

        (void)ZeroRun(1000U, &l, &rr, 300U, 10U, 121U);
    }
    CHECK_EQ(First(APP_PARK_EV_ZL)->tick, 1060);     /* window [10..60] flat */
    CHECK_EQ(First(APP_PARK_EV_ZR)->tick, 1050);
    CHECK_EQ(First(APP_PARK_EV_ZERO)->tick, 1060);

    /* A sampling gap longer than the window must not hide movement: 100
     * counts between a 60 ms-old sample and the newest is NOT stationary,
     * and a window the history does not reach back to is never claimed. */
    {
        AppParkTraceStats st;

        Reset();
        OpenStop(1000U, 0, 0);                      /* PRE ARM REQ ACC TAKE BRK */
        AppParkTrace_Sample(1030U, 0, 0);           /* history covers 30 ms only */
        AppParkTrace_GetStats(&st);
        CHECK_EQ(st.stored, 6);
        AppParkTrace_Sample(1060U, 100, 100);       /* +100 over a 60 ms gap */
        AppParkTrace_GetStats(&st);
        CHECK_EQ(st.stored, 6);                     /* NOT reported stationary */
        AppParkTrace_Sample(1110U, 100, 100);       /* flat since 1060 (50 ms) */
        AppParkTrace_GetStats(&st);
        CHECK_EQ(st.stored, 9);                     /* ZL ZR ZERO */
        Ev(APP_PARK_EV_BEND, 1121U, 0, 0);
        PopAll();
        CHECK_EQ(First(APP_PARK_EV_ZL)->tick, 1110);
        CHECK_EQ(First(APP_PARK_EV_ZR)->tick, 1110);
        CHECK_EQ(First(APP_PARK_EV_ZERO)->tick, 1110);
    }

    /* Irregular cadence (49 / 51 ms gaps) still needs a covered window. */
    Reset();
    OpenStop(1000U, 7, 7);
    {
        Wheel l = { 7, 0, 0U, 0 };
        Wheel rr = { 7, 0, 0U, 0 };
        static const uint32_t at[] = { 20U, 40U, 60U, 89U, 140U };
        unsigned i;

        for (i = 0U; i < sizeof(at) / sizeof(at[0]); i++)
        {
            AppParkTrace_Sample(1000U + at[i], WPos(&l, at[i]), Raw16(WPos(&rr, at[i])));
        }
        Ev(APP_PARK_EV_BEND, 1121U, 0, 0);
    }
    PopAll();
    /* samples at 0,20,40,60,89,140: at 60 the oldest (0) is 60 > 50 old:
     * window = {20,40,60} + coverage from the older sample -> confirmed. */
    CHECK_EQ(First(APP_PARK_EV_ZERO)->tick, 1060);
}

static void test_zero_timeout_and_completion_order(void)
{
    const AppParkTraceRecord *z;

    /* Wheels never settle: ZERO with the timeout flag at exactly BRK+2000. */
    Reset();
    OpenStop(5000U, 0, 0);
    {
        Wheel l = { 0, 10, 100000U, 0 };
        Wheel rr = { 0, 10, 100000U, 0 };
        uint32_t t;

        for (t = 10U; t <= 1990U; t += 10U)
        {
            AppParkTrace_Sample(5000U + t, WPos(&l, t), Raw16(WPos(&rr, t)));
        }

        Ev(APP_PARK_EV_BEND, 5121U, 0, 0);
        CHECK(AppParkTrace_GetState() == APP_PARK_STATE_BRAKE);   /* 1990: no */
        AppParkTrace_Sample(5000U + 2000U, WPos(&l, 2000U), Raw16(WPos(&rr, 2000U)));
    }
    CHECK(AppParkTrace_DrainReady());
    PopAll();
    z = First(APP_PARK_EV_ZERO);
    CHECK(z != NULL);
    CHECK_EQ(z->tick, 7000);
    CHECK_EQ(z->flags & APP_PARK_FLAG_ZERO_TIMEOUT, APP_PARK_FLAG_ZERO_TIMEOUT);
    CHECK_EQ(CountEv(APP_PARK_EV_ZL), 0);
    CHECK_EQ(CountEv(APP_PARK_EV_ZR), 0);
    CHECK_EQ(First(APP_PARK_EV_DONE)->arg, APP_PARK_RES_TIMEOUT);
    CHECK_EQ(First(APP_PARK_EV_DONE)->flags & APP_PARK_FLAG_ZERO_TIMEOUT,
             APP_PARK_FLAG_ZERO_TIMEOUT);

    /* Only the left wheel settles: ZL, then ZERO with the timeout flag. */
    Reset();
    OpenStop(5000U, 0, 0);
    {
        Wheel l = { 0, 10, 30U, 0 };
        Wheel rr = { 0, 10, 100000U, 0 };
        uint32_t t;

        Ev(APP_PARK_EV_BEND, 5121U, 0, 0);
        for (t = 10U; t <= 2000U; t += 10U)
        {
            AppParkTrace_Sample(5000U + t, WPos(&l, t), Raw16(WPos(&rr, t)));
        }
    }
    PopAll();
    CHECK_EQ(CountEv(APP_PARK_EV_ZL), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_ZR), 0);
    CHECK_EQ(First(APP_PARK_EV_ZERO)->flags & APP_PARK_FLAG_ZERO_TIMEOUT, 1);
    CHECK(IndexOf(APP_PARK_EV_ZL, 0) < IndexOf(APP_PARK_EV_ZERO, 0));

    /* Zero confirmed BEFORE the brake ends: DONE waits for BEND, and ZERO is
     * recorded before BEND. */
    Reset();
    OpenStop(1000U, 0, 0);
    {
        Wheel l = { 0, 0, 0U, 0 };
        Wheel rr = { 0, 0, 0U, 0 };
        uint32_t t;

        for (t = 10U; t <= 100U; t += 10U)
        {
            AppParkTrace_Sample(1000U + t, WPos(&l, t), Raw16(WPos(&rr, t)));
        }

        CHECK(AppParkTrace_GetState() == APP_PARK_STATE_BRAKE);   /* no DONE yet */
        CHECK(!AppParkTrace_DrainReady());
        Ev(APP_PARK_EV_BEND, 1121U, 0, 0);
        CHECK(AppParkTrace_DrainReady());
    }
    PopAll();
    CHECK(IndexOf(APP_PARK_EV_ZERO, 0) < IndexOf(APP_PARK_EV_BEND, 0));
    CHECK(IndexOf(APP_PARK_EV_BEND, 0) < IndexOf(APP_PARK_EV_DONE, 0));
    CHECK_EQ(First(APP_PARK_EV_ZERO)->tick, 1050);
    CHECK_EQ(First(APP_PARK_EV_BEND)->tick, 1121);
    CHECK_EQ(First(APP_PARK_EV_DONE)->tick, 1121);

    /* Zero confirmed AFTER the brake ends: BEND first, DONE with ZERO. */
    Reset();
    OpenStop(1000U, 0, 0);
    {
        Wheel l = { 0, 30, 120U, 0 };
        Wheel rr = { 0, 30, 120U, 0 };
        uint32_t t;

        for (t = 10U; t <= 200U; t += 10U)
        {
            if (t == 120U) { Ev(APP_PARK_EV_BEND, 1121U, 0, 0); }
            AppParkTrace_Sample(1000U + t, WPos(&l, t), Raw16(WPos(&rr, t)));
        }
    }
    PopAll();
    CHECK(IndexOf(APP_PARK_EV_BEND, 0) < IndexOf(APP_PARK_EV_ZERO, 0));
    CHECK(IndexOf(APP_PARK_EV_ZERO, 0) < IndexOf(APP_PARK_EV_DONE, 0));
    CHECK_EQ(First(APP_PARK_EV_ZERO)->tick, 1170);
    CHECK_EQ(First(APP_PARK_EV_DONE)->tick, 1170);
}

static void test_poll_timeouts(void)
{
    /* REQ with no ACC. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);
    Req(1000U, 0, 0);
    AppParkTrace_Poll(2999U);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_REQ);
    AppParkTrace_Poll(3000U);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_DRAIN);
    PopAll();
    CHECK_EQ(First(APP_PARK_EV_DONE)->arg, APP_PARK_RES_TIMEOUT);

    /* ACC and TAKE but the brake never starts (STOP during a natural brake). */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);
    Req(1000U, 0, 0);
    Ev(APP_PARK_EV_ACC, 1001U, 0, 0);
    Ev(APP_PARK_EV_TAKE, 1010U, 0, 0);
    AppParkTrace_Poll(2999U);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_TAKE);
    AppParkTrace_Poll(3000U);
    CHECK_EQ(PopAll(), 6);                       /* PRE ARM REQ ACC TAKE DONE */
    CHECK_EQ(First(APP_PARK_EV_DONE)->arg, APP_PARK_RES_TIMEOUT);

    /* Timeout across the 32-bit tick wrap. */
    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);
    Req(0xFFFFFC18U, 0, 0);                      /* 1000 ticks before wrap */
    AppParkTrace_Poll(0x000003E7U);              /* +1999 */
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_REQ);
    AppParkTrace_Poll(0x000003E8U);              /* +2000 */
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_DRAIN);

    /* BEND never arrives: closed by Poll after the grace period. */
    Reset();
    OpenStop(1000U, 0, 0);
    {
        Wheel l = { 0, 0, 0U, 0 };
        Wheel rr = { 0, 0, 0U, 0 };
        uint32_t t;

        for (t = 10U; t <= 200U; t += 10U)
        {
            AppParkTrace_Sample(1000U + t, WPos(&l, t), Raw16(WPos(&rr, t)));
        }
    }
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_BRAKE);
    AppParkTrace_Poll(1000U + 2499U);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_BRAKE);
    AppParkTrace_Poll(1000U + 2500U);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_DRAIN);
    PopAll();
    CHECK_EQ(First(APP_PARK_EV_DONE)->arg, APP_PARK_RES_TIMEOUT);

    /* Poll does nothing while merely ARMED or idle. */
    Reset();
    AppParkTrace_Poll(123456U);
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);
    AppParkTrace_Poll(999999U);
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_ARMED);
}

static void test_tick_and_encoder_wraparound_full_window(void)
{
    uint32_t brk = 0xFFFFFFB0U;                  /* 80 ticks before the wrap */
    const AppParkTraceRecord *zero;

    Reset();
    OpenStop(brk, INT32_MAX - 30, 32700);
    {
        Wheel l = { INT32_MAX - 30, 10, 30U, 1 };     /* crosses INT32_MAX */
        Wheel rr = { 32700, 10, 40U, 1 };             /* crosses 32767 */

        (void)ZeroRun(brk, &l, &rr, 400U, 10U, 121U);
    }
    zero = First(APP_PARK_EV_ZERO);
    CHECK(zero != NULL);
    CHECK_EQ(zero->flags, 0);
    /* right stops 40 ms after BRK; window full 50 ms later = BRK + 90 */
    CHECK_EQ(zero->tick, (uint32_t)(brk + 90U));
    CHECK_EQ(First(APP_PARK_EV_ZL)->tick, (uint32_t)(brk + 80U));
    CHECK_EQ(First(APP_PARK_EV_DONE)->arg, APP_PARK_RES_OK);

    /* Negative direction through the right-hand wrap. */
    Reset();
    OpenStop(1000U, INT32_MIN + 15, -32760);
    {
        Wheel l = { INT32_MIN + 15, -10, 20U, 0 };    /* crosses INT32_MIN */
        Wheel rr = { -32760, -10, 20U, 0 };           /* crosses -32768 */

        (void)ZeroRun(1000U, &l, &rr, 300U, 10U, 121U);
    }
    CHECK_EQ(First(APP_PARK_EV_ZERO)->tick, 1070);
    CHECK_EQ(First(APP_PARK_EV_DONE)->arg, APP_PARK_RES_OK);

    /* A right wheel sitting exactly on +32768 / -32768 is one position. */
    Reset();
    OpenStop(1000U, 5, 32768);
    {
        uint32_t t;

        for (t = 10U; t <= 120U; t += 10U)
        {
            AppParkTrace_Sample(1000U + t, 5, (t & 10U) ? 32768 : -32768);
        }
        Ev(APP_PARK_EV_BEND, 1121U, 0, 0);
    }
    PopAll();
    CHECK_EQ(First(APP_PARK_EV_ZERO)->flags, 0);
    CHECK_EQ(First(APP_PARK_EV_ZERO)->tick, 1050);
}

/* ---- capacity, caps, overflow ------------------------------------------ */

#ifndef OVERCOMMIT_TEST

static void test_capacity_proof_worst_valid_window(void)
{
    unsigned i;
    unsigned us = 0U;
    unsigned near = 0U;
    AppParkTraceStats st;
    const AppParkTraceRecord *done;

    Reset();

    /* Three valid publications of history. */
    Pub(100U, 1U, 900U);
    Pub(170U, 2U, 800U);
    Pub(240U, 3U, 480U);
    Arm(241U, 77U);

    /* Far more US and NEAR candidates than the caps allow. */
    for (i = 0U; i < 100U; i++)
    {
        Pub(300U + 70U * i, (uint16_t)(10U + i), 450U);
        if (i < 20U) { Near(301U + 70U * i, (uint8_t)(1U + (i % 3U))); }
    }

    Req(9000U, 1, 2);
    Ev(APP_PARK_EV_ACC, 9001U, 1, 2);
    Ev(APP_PARK_EV_TAKE, 9010U, 1, 2);
    Ev(APP_PARK_EV_BRK, 9010U, 1, 2);
    {
        Wheel l = { 1, 0, 0U, 0 };
        Wheel rr = { 2, 0, 0U, 0 };

        RunSamples(9010U, &l, &rr, 10U, 70U, 10U);
        Ev(APP_PARK_EV_BEND, 9131U, 3, 4);
        RunSamples(9010U, &l, &rr, 80U, 200U, 10U);
    }

    AppParkTrace_GetStats(&st);
    CHECK(st.state == APP_PARK_STATE_DRAIN);
    CHECK_EQ(st.stored, APP_PARK_TRACE_RING_SIZE);        /* exactly full */
    CHECK_EQ(st.windowRingFull, 0);                        /* ... but never over */

    PopAll();
    CHECK_EQ(poppedCount, 48);
    for (i = 0U; i < poppedCount; i++)
    {
        if (popped[i].ev == APP_PARK_EV_US) { us++; }
        if (popped[i].ev == APP_PARK_EV_NEAR) { near++; }
        CHECK_EQ(popped[i].n, i + 1U);
    }

    CHECK_EQ(us, APP_PARK_TRACE_US_CAP);
    CHECK_EQ(near, APP_PARK_TRACE_NEAR_CAP);
    CHECK_EQ(CountEv(APP_PARK_EV_PRE), 3);
    CHECK_EQ(CountEv(APP_PARK_EV_DONE), 1);
    CHECK_EQ(popped[47].ev, APP_PARK_EV_DONE);

    /* Every terminal record is present despite the pressure. */
    CHECK_EQ(CountEv(APP_PARK_EV_ARM), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_REQ), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_ACC), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_TAKE), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_BRK), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_BEND), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_ZL), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_ZR), 1);
    CHECK_EQ(CountEv(APP_PARK_EV_ZERO), 1);

    /* The dropped count is exact: (100 - 29) US + (20 - 6) NEAR. */
    done = First(APP_PARK_EV_DONE);
    CHECK_EQ(done->raw, (100U - APP_PARK_TRACE_US_CAP) + (20U - APP_PARK_TRACE_NEAR_CAP));
    CHECK_EQ(done->filt, 0);
    CHECK_EQ(done->arg, APP_PARK_RES_OK);
    AppParkTrace_GetStats(&st);
    CHECK_EQ(st.lifetimeDropped, done->raw);
}

static void test_ring_wraps_across_windows(void)
{
    unsigned round;
    unsigned i;

    Reset();

    /* Five windows of 20+ records walk head and tail around the 48-slot ring
     * several times; every window must come out complete and ordered. */
    for (round = 0U; round < 5U; round++)
    {
        uint32_t base = 100000U * (round + 1U);
        uint32_t brk;
        Wheel l = { 0, 0, 0U, 0 };
        Wheel rr = { 0, 0, 0U, 0 };

        l.start = (int32_t)(8U * 290U);
        rr.start = (int32_t)(8U * 290U);
        ApproachToBrake(base, (uint16_t)(round + 1U), &brk);
        RunSamples(brk, &l, &rr, 10U, 70U, 10U);
        Ev(APP_PARK_EV_BEND, brk + 121U, 0, 0);
        RunSamples(brk, &l, &rr, 80U, 130U, 10U);

        CHECK(AppParkTrace_DrainReady());
        PopAll();
        CHECK(poppedCount >= 17U);
        CHECK_EQ(popped[poppedCount - 1U].ev, APP_PARK_EV_DONE);
        CHECK_EQ(popped[poppedCount - 1U].w, round + 1U);
        CHECK_EQ(popped[poppedCount - 1U].raw, 0);

        for (i = 0U; i < poppedCount; i++)
        {
            CHECK_EQ(popped[i].n, i + 1U);
        }

        CHECK(AppParkTrace_GetState() == APP_PARK_STATE_IDLE);
    }
}

static void test_arm_while_previous_trace_unsent_is_skipped(void)
{
    AppParkTraceStats st;
    const AppParkTraceRecord *done;

    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 1U);
    AppParkTrace_MovementEnded(150U);          /* DRAIN, 3 records */
    CHECK(AppParkTrace_DrainReady());

    /* A new window tries to open before the first was sent. */
    Pub(200U, 2U, 480U);
    Arm(201U, 2U);
    Pub(210U, 3U, 400U);
    Near(211U, 1U);
    AppParkTrace_GetStats(&st);
    CHECK_EQ(st.windowsSkipped, 1);
    CHECK(st.state == APP_PARK_STATE_DRAIN);
    CHECK_EQ(st.stored, 3);                                 /* untouched */

    CHECK_EQ(PopAll(), 3);
    CHECK_EQ(popped[0].ev, APP_PARK_EV_PRE);
    CHECK_EQ(popped[2].ev, APP_PARK_EV_DONE);
    CHECK_EQ(popped[2].fm, 0);                              /* skip came later */

    /* The next window that does open reports the earlier skip in DONE. */
    Pub(300U, 4U, 480U);
    Arm(301U, 3U);
    AppParkTrace_MovementEnded(350U);
    PopAll();
    done = First(APP_PARK_EV_DONE);
    CHECK(done != NULL);
    CHECK_EQ(done->fm, 1);
    CHECK_EQ(done->w, 3);
}

#else   /* OVERCOMMIT_TEST: caps deliberately larger than the ring */

static void test_ring_overflow_counts_and_protects(void)
{
    unsigned i;
    AppParkTraceStats st;
    const AppParkTraceRecord *done;
    uint16_t firstSeq;

    Reset();
    Pub(100U, 1U, 480U);
    Arm(101U, 9U);

    /* 80 US publications against a 48-slot ring and a cap of 70. */
    for (i = 0U; i < 80U; i++)
    {
        Pub(200U + i, (uint16_t)(100U + i), 450U);
    }

    AppParkTrace_GetStats(&st);
    CHECK_EQ(st.stored, 48);
    CHECK(st.windowRingFull > 0U);

    Req(1000U, 0, 0);                     /* ring full: counted, state advances */
    CHECK(AppParkTrace_GetState() == APP_PARK_STATE_REQ);
    AppParkTrace_Refused(1001U);   /* DONE is lost too */
    AppParkTrace_GetStats(&st);
    CHECK_EQ(st.stored, 48);
    CHECK(st.state == APP_PARK_STATE_DRAIN);

    /* Unread records were never overwritten: the first one is still PRE seq 1. */
    CHECK_EQ(PopAll(), 48);
    CHECK_EQ(popped[0].ev, APP_PARK_EV_PRE);
    CHECK_EQ(popped[0].seq, 1);
    CHECK_EQ(popped[1].ev, APP_PARK_EV_ARM);
    firstSeq = popped[2].seq;
    CHECK_EQ(firstSeq, 100);
    for (i = 0U; i < poppedCount; i++)
    {
        CHECK_EQ(popped[i].n, i + 1U);
    }
    CHECK_EQ(CountEv(APP_PARK_EV_DONE), 0);              /* it was dropped */

    /* Exact drop accounting: 48 stored of (PRE + ARM + 80 US + REQ + DONE). */
    AppParkTrace_GetStats(&st);
    CHECK_EQ(st.lifetimeDropped, (1U + 1U + 80U + 1U + 1U) - 48U);
    CHECK_EQ(st.windowRingFull, st.windowDropped);
    CHECK(st.state == APP_PARK_STATE_IDLE);               /* drained -> idle */

    /* Recovery: a fresh, complete window works afterwards. */
    Pub(5000U, 500U, 480U);
    Arm(5001U, 10U);
    AppParkTrace_MovementEnded(5010U);
    CHECK_EQ(PopAll(), 5);                       /* 3 PRE (history) ARM DONE */
    CHECK_EQ(CountEv(APP_PARK_EV_PRE), 3);
    CHECK_EQ(popped[0].seq, 178);                /* oldest of the last three */
    done = First(APP_PARK_EV_DONE);
    CHECK(done != NULL);
    CHECK_EQ(done->raw, 0);                              /* window counters reset */
    CHECK_EQ(done->w, 10);
    CHECK_EQ(popped[0].n, 1);
}

#endif /* OVERCOMMIT_TEST */

/* ---- formatter --------------------------------------------------------- */

static void Fmt(const AppParkTraceRecord *r, char *buf, size_t size)
{
    int n = AppParkTrace_Format(r, buf, size);

    if (n < 0) { strcpy(buf, "<REFUSED>"); }
}

static void test_formatter_exact_lines(void)
{
    AppParkTraceRecord r;
    char buf[160];

    AppParkTrace_InitRecord(&r, APP_PARK_EV_US, 123456U, -10, 25);
    r.n = 3U; r.seq = 77U; r.obs = 4U; r.raw = 1234U; r.filt = 1230U;
    r.age = 12U; r.pulse = 7000U;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:US,N:3,T:123456,OD:4,SEQ:77,RAW:1234,"
                   "FILT:1230,AGE:12,PU:7000,OEL:-10,OER:25\n");

    r.ev = APP_PARK_EV_PRE;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:PRE,N:3,T:123456,OD:4,SEQ:77,RAW:1234,"
                   "FILT:1230,AGE:12,PU:7000,OEL:-10,OER:25\n");

    /* Unavailable fields print NA, never 0 or a sentinel number. */
    AppParkTrace_InitRecord(&r, APP_PARK_EV_US, 5U, 0, 0);
    r.n = 1U; r.seq = 1U; r.obs = 2U;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:US,N:1,T:5,OD:2,SEQ:1,RAW:NA,FILT:NA,"
                   "AGE:NA,PU:NA,OEL:0,OER:0\n");

    /* Oversized values are clamped, not truncated mid-number. */
    AppParkTrace_InitRecord(&r, APP_PARK_EV_US, 5U, 0, 0);
    r.n = 1U; r.seq = 1U; r.obs = 5000U; r.raw = 20000U; r.filt = 20000U;
    r.age = 5000U; r.pulse = 65534U;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:US,N:1,T:5,OD:999,SEQ:1,RAW:9999,"
                   "FILT:9999,AGE:999,PU:65534,OEL:0,OER:0\n");

    AppParkTrace_InitRecord(&r, APP_PARK_EV_ARM, 1000U, 5, 6);
    r.n = 4U; r.w = 7U; r.seq = 9U; r.filt = 480U; r.arg = 1U; r.flags = 2U;
    r.fm = 0x1A0U;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:ARM,N:4,T:1000,W:7,SEQ:9,FILT:480,CN:1,"
                   "P:T2T,FM:1A0,EL:5,ER:6\n");
    r.flags = 0U; r.arg = 0U;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:ARM,N:4,T:1000,W:7,SEQ:9,FILT:480,CN:0,"
                   "P:PREC,FM:1A0,EL:5,ER:6\n");

    AppParkTrace_InitRecord(&r, APP_PARK_EV_NEAR, 2000U, 1, 2);
    r.n = 6U; r.arg = 2U; r.seq = 10U; r.filt = 290U;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:NEAR,N:6,T:2000,K:2,SEQ:10,FILT:290,"
                   "EL:1,ER:2\n");

    AppParkTrace_InitRecord(&r, APP_PARK_EV_REQ, 3000U, 100, -100);
    r.n = 9U; r.arg = APP_PARK_CAUSE_NEAR; r.seq = 11U; r.obs = 14U;
    r.vl = 8000; r.vr = -7990;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:REQ,N:9,T:3000,R:NEAR,SEQ:11,PD:14,"
                   "EL:100,ER:-100,VL:8000,VR:-7990\n");
    r.arg = APP_PARK_CAUSE_NOUPDATE; r.obs = APP_PARK_TRACE_NA_U16;
    r.vl = APP_PARK_TRACE_NA_V; r.vr = APP_PARK_TRACE_NA_V;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:REQ,N:9,T:3000,R:NOUPDATE,SEQ:11,PD:NA,"
                   "EL:100,ER:-100,VL:NA,VR:NA\n");
    r.arg = APP_PARK_CAUSE_INVALID;
    Fmt(&r, buf, sizeof(buf));
    CHECK(strstr(buf, "R:INVALID,") != NULL);
    r.arg = APP_PARK_CAUSE_FAULT;
    Fmt(&r, buf, sizeof(buf));
    CHECK(strstr(buf, "R:FAULT,") != NULL);

    AppParkTrace_InitRecord(&r, APP_PARK_EV_ACC, 3001U, 1, 2);
    r.n = 10U;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:ACC,N:10,T:3001,EL:1,ER:2\n");
    r.ev = APP_PARK_EV_TAKE; r.n = 11U;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:TAKE,N:11,T:3001,EL:1,ER:2\n");
    r.ev = APP_PARK_EV_BEND; r.n = 13U;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:BEND,N:13,T:3001,EL:1,ER:2\n");
    r.ev = APP_PARK_EV_ZL; r.n = 14U;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:ZL,N:14,T:3001,EL:1,ER:2\n");
    r.ev = APP_PARK_EV_ZR; r.n = 15U;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:ZR,N:15,T:3001,EL:1,ER:2\n");

    AppParkTrace_InitRecord(&r, APP_PARK_EV_BRK, 3010U, 5, 6);
    r.n = 12U; r.vl = -250; r.vr = 12;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:BRK,N:12,T:3010,EL:5,ER:6,VL:-250,VR:12\n");

    AppParkTrace_InitRecord(&r, APP_PARK_EV_ZERO, 4000U, 5, 6);
    r.n = 16U;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:ZERO,N:16,T:4000,EL:5,ER:6,TO:0\n");
    r.flags = APP_PARK_FLAG_ZERO_TIMEOUT;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:ZERO,N:16,T:4000,EL:5,ER:6,TO:1\n");

    AppParkTrace_InitRecord(&r, APP_PARK_EV_DONE, 4100U, 0, 0);
    r.n = 17U; r.w = 7U; r.arg = APP_PARK_RES_OK; r.raw = 0U; r.filt = 0U;
    r.fm = 0U;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:DONE,N:17,T:4100,W:7,R:OK,DR:0,RF:0,"
                   "ZT:0,SK:0\n");
    r.arg = APP_PARK_RES_TIMEOUT; r.flags = APP_PARK_FLAG_ZERO_TIMEOUT;
    r.raw = 3U; r.filt = 1U; r.fm = 2U;
    Fmt(&r, buf, sizeof(buf));
    CHECK_STR(buf, "USPARK,TRACE,EV:DONE,N:17,T:4100,W:7,R:TO,DR:3,RF:1,"
                   "ZT:1,SK:2\n");
    {
        static const struct { uint8_t res; const char *name; } names[] = {
            { APP_PARK_RES_NOREQ, "NOREQ" }, { APP_PARK_RES_NOACC, "NOACC" },
            { APP_PARK_RES_REFUSED, "REFUSED" }, { APP_PARK_RES_ABORT, "ABORT" }
        };
        unsigned i;
        char want[24];

        for (i = 0U; i < 4U; i++)
        {
            r.arg = names[i].res;
            Fmt(&r, buf, sizeof(buf));
            snprintf(want, sizeof(want), ",R:%s,", names[i].name);
            CHECK(strstr(buf, want) != NULL);
        }
    }

    /* Unknown events are refused. */
    AppParkTrace_InitRecord(&r, APP_PARK_EV_NONE, 1U, 0, 0);
    CHECK_EQ(AppParkTrace_Format(&r, buf, sizeof(buf)), -1);
    r.ev = 200U;
    CHECK_EQ(AppParkTrace_Format(&r, buf, sizeof(buf)), -1);
    CHECK_EQ(AppParkTrace_Format(NULL, buf, sizeof(buf)), -1);
    CHECK_EQ(AppParkTrace_Format(&r, NULL, sizeof(buf)), -1);
    CHECK_EQ(AppParkTrace_Format(&r, buf, 0U), -1);
}

static void test_formatter_length_bounds(void)
{
    unsigned ev;
    unsigned maxLen = 0U;
    char buf[200];

    /* Every event at the widest value of every field of its real domain. */
    for (ev = APP_PARK_EV_US; ev <= APP_PARK_EV_DONE; ev++)
    {
        AppParkTraceRecord r;
        int n;

        AppParkTrace_InitRecord(&r, (uint8_t)ev, 0xFFFFFFFFU,
                                INT32_MIN, -32768);
        r.n = 48U;
        r.seq = 65535U;
        r.w = 65535U;
        r.raw = 65534U;
        r.filt = 65534U;
        r.age = 65534U;
        r.pulse = 65534U;
        r.obs = 65534U;
        r.fm = 65535U;
        r.vl = -32767;
        r.vr = -32767;
        r.arg = (ev == APP_PARK_EV_REQ) ? APP_PARK_CAUSE_NOUPDATE :
                (ev == APP_PARK_EV_DONE) ? APP_PARK_RES_REFUSED : 1U;
        r.flags = 3U;

        n = AppParkTrace_Format(&r, buf, sizeof(buf));
        CHECK(n > 0);
        CHECK(n < (int)APP_PARK_TRACE_LINE_MAX);
        CHECK_EQ(buf[n - 1], '\n');
        CHECK_EQ(buf[n], '\0');
        CHECK_EQ(strlen(buf), (size_t)n);
        if ((unsigned)n > maxLen) { maxLen = (unsigned)n; }
    }

    printf("    longest trace line: %u bytes (limit < %u)\n",
           maxLen, APP_PARK_TRACE_LINE_MAX);
    CHECK(maxLen < APP_PARK_TRACE_LINE_MAX);

    /* A US line whose left count is far outside the real domain would be too
     * long: it must be REFUSED, never truncated or sent. (Encoder counts and
     * every other field are bounded by the producers' types.) */
    {
        AppParkTraceRecord r;

        AppParkTrace_InitRecord(&r, APP_PARK_EV_US, 0xFFFFFFFFU,
                                INT32_MIN, INT32_MIN);
        r.n = 48U; r.seq = 65535U; r.raw = 9999U; r.filt = 9999U;
        r.age = 999U; r.pulse = 65534U; r.obs = 999U;
        CHECK_EQ(AppParkTrace_Format(&r, buf, sizeof(buf)), -1);
    }

    /* Buffer-size boundary: exactly len+1 fits, len does not. */
    {
        AppParkTraceRecord r;
        int n;
        char tight[160];

        AppParkTrace_InitRecord(&r, APP_PARK_EV_ACC, 3001U, 1, 2);
        r.n = 10U;
        n = AppParkTrace_Format(&r, tight, sizeof(tight));
        CHECK(n > 0);
        CHECK_EQ(AppParkTrace_Format(&r, tight, (size_t)n + 1U), n);
        CHECK_EQ(AppParkTrace_Format(&r, tight, (size_t)n), -1);
    }
}

static void test_critical_sections_balanced(void)
{
    /* Re-run a full cycle and check the books. */
    uint32_t brk;
    Wheel l = { 8 * 290, 0, 0U, 0 };
    Wheel rr = { 8 * 290, 0, 0U, 0 };

    Reset();
    ApproachToBrake(1000U, 3U, &brk);
    RunSamples(brk, &l, &rr, 10U, 70U, 10U);
    Ev(APP_PARK_EV_BEND, brk + 121U, 0, 0);
    RunSamples(brk, &l, &rr, 80U, 130U, 10U);
    PopAll();
    AppParkTrace_Poll(999999U);
    (void)AppParkTrace_DrainReady();
    {
        AppParkTraceStats st;
        AppParkTrace_GetStats(&st);
    }
    CHECK(FakeCrit_Enters > 0U);
    CHECK_EQ(FakeCrit_Depth, 0);
    CHECK_EQ(FakeCrit_Underflows, 0);
    CHECK(FakeCrit_MaxDepth <= 1);
}

int main(void)
{
    test_layout_and_helpers();
    test_event_order_and_pre_replay();
    test_pre_replay_fewer_than_three();
    test_out_of_order_and_duplicates_ignored();
    test_manual_stop_and_non_automatic_paths_record_nothing();
    test_velocity_estimate();
    test_latch_path();
    test_zero_confirmation_per_wheel();
    test_zero_definition_boundaries();
    test_zero_timeout_and_completion_order();
    test_poll_timeouts();
    test_tick_and_encoder_wraparound_full_window();
#ifndef OVERCOMMIT_TEST
    test_capacity_proof_worst_valid_window();
    test_ring_wraps_across_windows();
    test_arm_while_previous_trace_unsent_is_skipped();
#else
    test_ring_overflow_counts_and_protects();
#endif
    test_formatter_exact_lines();
    test_formatter_length_bounds();
    test_critical_sections_balanced();

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
