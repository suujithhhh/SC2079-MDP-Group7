/*
 * app_park_trace.c
 *
 * Fixed 48-entry trace ring, window state machine, encoder velocity estimate
 * and the zero-speed detector for the measurement-only USPARK trace. See
 * app_park_trace.h for the design, the capacity proof and the definitions.
 *
 * Rules this file keeps:
 *   - no printing, no formatting, no allocation, no waiting: the only
 *     external calls are taskENTER_CRITICAL()/taskEXIT_CRITICAL() and memset;
 *   - every public function is bounded (the longest loop is the 16-entry
 *     zero-speed history, run only between BRK and ZERO);
 *   - a dropped record only increments counters. Nothing here returns a value
 *     that any control path reads;
 *   - no 36-byte record lives on a stack: the chains that reach this module
 *     run on MotionTask's 1 KB stack, whose static worst case without the
 *     trace is already 728 B. Hooks that run inside AppMotion_Update() (BRK,
 *     BEND) only LATCH tick and counts into a few static words, and the next
 *     AppParkTrace_Sample() call, made by MotionTask at the top of its loop,
 *     submits them at a shallow depth.
 *
 * Concurrency: CommTask and MotionTask both call in. All shared state is
 * touched only inside one critical section per call (the same primitive the
 * rest of the firmware uses for its published snapshots). The unlocked reads
 * are the fast "nothing to do" test in AppParkTrace_Sample() and the
 * MotionTask-private latch words.
 */

#include "app_park_trace.h"

#if APP_USPARK_TRACE

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#define VEL_HIST   5U       /* about 40 ms of MotionTask cycles              */
#define ZERO_HIST  16U      /* >> the 50 ms window at a 10 ms cadence        */
#define VEL_MAX_DT_MS     1000U
#define VEL_CLAMP         9999
#define COUNT_CLAMP       2000000   /* keeps delta*1000 inside int32          */

typedef struct
{
    uint32_t tick;
    int32_t  el;
    int32_t  er;
} EncSample;

/* Events raised deep inside AppMotion_Update() (BRK, BEND). MotionTask is both
 * the sole writer and the sole reader, so no lock is needed. */
typedef struct
{
    volatile uint8_t pending;
    uint32_t tick;
    int32_t  el;
    int32_t  er;
} LatchedEvent;

static AppParkTraceRecord ring[APP_PARK_TRACE_RING_SIZE];
static uint8_t ringHead;
static uint8_t ringTail;
static uint8_t ringUsed;

static volatile AppParkTraceState state;
static uint16_t windowId;
static uint8_t nextIndex;
static uint8_t usUsed;
static uint8_t nearUsed;
static uint16_t droppedWindow;
static uint16_t droppedRingFull;
static uint32_t droppedLifetime;
static uint16_t windowsSkipped;

static uint32_t reqTick;
static uint32_t brkTick;
static bool bendSeen;
static bool zeroDone;
static bool zeroTimeout;
static bool leftStill;
static bool rightStill;

/* Last valid publications seen by CommTask (copied into the ring at ARM). */
static AppParkTraceRecord history[APP_PARK_TRACE_PRE_COUNT];
static uint8_t historyCount;
static uint8_t historyNext;

static EncSample vel[VEL_HIST];
static uint8_t velCount;
static uint8_t velNext;

static EncSample zh[ZERO_HIST];
static uint8_t zhCount;
static uint8_t zhNext;

/* Scratch records. They are used ONLY while the critical section is held (so
 * they are never shared) and live in .bss rather than on a stack. */
static AppParkTraceRecord scratchCopy;      /* SubmitLocked(): REQ/BRK/PRE   */
static AppParkTraceRecord scratchDone;      /* CloseDone()                   */
static AppParkTraceRecord scratchZero;      /* StoreZeroEvent()              */
static AppParkTraceRecord scratchLatch;     /* FlushLatch()                  */

static LatchedEvent latchBrk;
static LatchedEvent latchBend;
static uint16_t latchOverruns;              /* MotionTask-only until flushed */

/* ---------------------------------------------------------------------- */

static void ResetWindow(void)
{
    nextIndex = 1U;
    usUsed = 0U;
    nearUsed = 0U;
    droppedWindow = 0U;
    droppedRingFull = 0U;
    reqTick = 0U;
    brkTick = 0U;
    bendSeen = false;
    zeroDone = false;
    zeroTimeout = false;
    leftStill = false;
    rightStill = false;
    velCount = 0U;
    velNext = 0U;
    zhCount = 0U;
    zhNext = 0U;
}

void AppParkTrace_Init(void)
{
    taskENTER_CRITICAL();

    memset(ring, 0, sizeof(ring));
    memset(history, 0, sizeof(history));
    memset(vel, 0, sizeof(vel));
    memset(zh, 0, sizeof(zh));
    ringHead = 0U;
    ringTail = 0U;
    ringUsed = 0U;
    historyCount = 0U;
    historyNext = 0U;
    windowId = 0U;
    droppedLifetime = 0U;
    windowsSkipped = 0U;
    latchBrk.pending = 0U;
    latchBend.pending = 0U;
    latchOverruns = 0U;
    ResetWindow();
    state = APP_PARK_STATE_IDLE;

    taskEXIT_CRITICAL();
}

void AppParkTrace_InitRecord(AppParkTraceRecord *r, uint8_t ev, uint32_t tick,
                             int32_t el, int32_t er)
{
    memset(r, 0, sizeof(*r));
    r->tick = tick;
    r->el = el;
    r->er = er;
    r->ev = ev;
    r->raw = APP_PARK_TRACE_NA_U16;
    r->filt = APP_PARK_TRACE_NA_U16;
    r->age = APP_PARK_TRACE_NA_U16;
    r->pulse = APP_PARK_TRACE_NA_U16;
    r->obs = APP_PARK_TRACE_NA_U16;
    r->vl = APP_PARK_TRACE_NA_V;
    r->vr = APP_PARK_TRACE_NA_V;
}

/* ---- locked helpers (the critical section is already held) ------------ */

static void CountDrop(bool ringFull)
{
    if (droppedWindow < UINT16_MAX) { droppedWindow++; }
    if (ringFull && (droppedRingFull < UINT16_MAX)) { droppedRingFull++; }
    if (droppedLifetime < UINT32_MAX) { droppedLifetime++; }
}

static bool Store(const AppParkTraceRecord *r)
{
    if (ringUsed >= APP_PARK_TRACE_RING_SIZE)
    {
        CountDrop(true);
        return false;
    }

    ring[ringHead] = *r;
    ring[ringHead].n = nextIndex;
    nextIndex++;
    ringHead = (uint8_t)((ringHead + 1U) % APP_PARK_TRACE_RING_SIZE);
    ringUsed++;
    return true;
}

static void CloseDone(uint32_t tick, uint8_t result)
{
    AppParkTrace_InitRecord(&scratchDone, APP_PARK_EV_DONE, tick, 0, 0);
    scratchDone.arg = result;
    scratchDone.w = windowId;
    scratchDone.raw = droppedWindow;
    scratchDone.filt = droppedRingFull;
    scratchDone.fm = windowsSkipped;
    scratchDone.flags = zeroTimeout ? APP_PARK_FLAG_ZERO_TIMEOUT : 0U;
    (void)Store(&scratchDone);
    state = APP_PARK_STATE_DRAIN;
}

static void AddVelSample(uint32_t tick, int32_t el, int32_t er)
{
    if (velCount > 0U)
    {
        const EncSample *last =
            &vel[(velNext + VEL_HIST - 1U) % VEL_HIST];

        if ((int32_t)(tick - last->tick) <= 0)
        {
            return;     /* not newer: keep the cadence clean */
        }
    }

    vel[velNext].tick = tick;
    vel[velNext].el = el;
    vel[velNext].er = er;
    velNext = (uint8_t)((velNext + 1U) % VEL_HIST);
    if (velCount < VEL_HIST) { velCount++; }
}

/* Signed counts/s over the retained samples, or NA. Integer only. */
static int16_t Velocity(bool right)
{
    const EncSample *newest;
    const EncSample *oldest;
    uint32_t dt;
    int32_t d;
    int32_t v;

    if (velCount < 2U)
    {
        return APP_PARK_TRACE_NA_V;
    }

    newest = &vel[(velNext + VEL_HIST - 1U) % VEL_HIST];
    oldest = &vel[(velNext + VEL_HIST - velCount) % VEL_HIST];
    dt = AppParkTrace_TickDelta(newest->tick, oldest->tick);

    if ((dt == 0U) || (dt > VEL_MAX_DT_MS))
    {
        return APP_PARK_TRACE_NA_V;
    }

    d = right ? (int32_t)AppParkTrace_Delta16(oldest->er, newest->er)
              : AppParkTrace_Delta32(oldest->el, newest->el);

    if (d > COUNT_CLAMP) { d = COUNT_CLAMP; }
    if (d < -COUNT_CLAMP) { d = -COUNT_CLAMP; }

    v = (d * 1000) / (int32_t)dt;

    if (v > VEL_CLAMP) { v = VEL_CLAMP; }
    if (v < -VEL_CLAMP) { v = -VEL_CLAMP; }

    return (int16_t)v;
}

/* True once the wheel moved by at most APP_PARK_TRACE_ZERO_MAX_COUNTS over the
 * rolling window ending at the newest sample.
 *
 * The range is taken over every sample inside the window PLUS the latest
 * sample at or before the window start (its position is the wheel's position
 * at the start of the window, sample-and-hold). That last sample is also what
 * proves the window is fully covered by samples taken at or after BRK: if the
 * history does not reach back 50 ms, nothing is claimed. With a 10 ms cadence
 * the boundary sample is exactly 50 ms old; with a coarser or jittery cadence
 * the older sample's position still counts, so a wheel that moved across a
 * sampling gap is never reported stationary. */
static bool WheelStill(bool right)
{
    const EncSample *newest;
    int32_t lo = 0;
    int32_t hi = 0;
    bool covered = false;
    uint8_t k;

    if (zhCount == 0U)
    {
        return false;
    }

    newest = &zh[(zhNext + ZERO_HIST - 1U) % ZERO_HIST];

    for (k = 0U; k < zhCount; k++)
    {
        const EncSample *e = &zh[(zhNext + ZERO_HIST - 1U - k) % ZERO_HIST];
        uint32_t age = AppParkTrace_TickDelta(newest->tick, e->tick);
        int32_t d = right ? (int32_t)AppParkTrace_Delta16(e->er, newest->er)
                          : AppParkTrace_Delta32(e->el, newest->el);

        if (d < lo) { lo = d; }
        if (d > hi) { hi = d; }

        if (age >= APP_PARK_TRACE_ZERO_WINDOW_MS)
        {
            covered = true;     /* the window start is represented */
            break;
        }
    }

    return covered &&
           (((uint32_t)hi - (uint32_t)lo) <=
            (uint32_t)APP_PARK_TRACE_ZERO_MAX_COUNTS);
}

static void StoreZeroEvent(uint8_t ev, uint32_t tick, int32_t el, int32_t er,
                           uint8_t flags)
{
    AppParkTrace_InitRecord(&scratchZero, ev, tick, el, er);
    scratchZero.flags = flags;
    (void)Store(&scratchZero);
}

static void CompleteIfReady(uint32_t tick)
{
    if (zeroDone && bendSeen && (state == APP_PARK_STATE_BRAKE))
    {
        CloseDone(tick, zeroTimeout ? APP_PARK_RES_TIMEOUT : APP_PARK_RES_OK);
    }
}

static void ZeroStep(uint32_t now, int32_t el, int32_t er)
{
    bool newer = true;

    if (zhCount > 0U)
    {
        const EncSample *last = &zh[(zhNext + ZERO_HIST - 1U) % ZERO_HIST];

        newer = ((int32_t)(now - last->tick) > 0);
    }

    if (newer)
    {
        zh[zhNext].tick = now;
        zh[zhNext].el = el;
        zh[zhNext].er = er;
        zhNext = (uint8_t)((zhNext + 1U) % ZERO_HIST);
        if (zhCount < ZERO_HIST) { zhCount++; }

        if (!leftStill && WheelStill(false))
        {
            leftStill = true;
            StoreZeroEvent(APP_PARK_EV_ZL, now, el, er, 0U);
        }

        if (!rightStill && WheelStill(true))
        {
            rightStill = true;
            StoreZeroEvent(APP_PARK_EV_ZR, now, el, er, 0U);
        }
    }

    if (leftStill && rightStill)
    {
        zeroDone = true;
        StoreZeroEvent(APP_PARK_EV_ZERO, now, el, er, 0U);
    }
    else if (AppParkTrace_TickDelta(now, brkTick) >= APP_PARK_TRACE_TIMEOUT_MS)
    {
        zeroDone = true;
        zeroTimeout = true;
        StoreZeroEvent(APP_PARK_EV_ZERO, now, el, er,
                       APP_PARK_FLAG_ZERO_TIMEOUT);
    }

    if (zeroDone)
    {
        CompleteIfReady(now);
    }
}

static void HistoryPush(const AppParkTraceRecord *r)
{
    history[historyNext] = *r;
    history[historyNext].ev = APP_PARK_EV_US;
    historyNext = (uint8_t)((historyNext + 1U) % APP_PARK_TRACE_PRE_COUNT);
    if (historyCount < APP_PARK_TRACE_PRE_COUNT) { historyCount++; }
}

static bool InWindow(void)
{
    return (state >= APP_PARK_STATE_ARMED) && (state <= APP_PARK_STATE_BRAKE);
}

/* The state machine. The critical section is held by the caller. */
static bool SubmitLocked(const AppParkTraceRecord *r)
{
    AppParkTraceRecord *copy = &scratchCopy;
    bool stored = false;

    switch (r->ev)
    {
        case APP_PARK_EV_US:
            if (r->raw != APP_PARK_TRACE_NA_U16)
            {
                HistoryPush(r);
            }

            if (InWindow())
            {
                if (usUsed < APP_PARK_TRACE_US_CAP)
                {
                    stored = Store(r);
                    if (stored) { usUsed++; }
                }
                else
                {
                    CountDrop(false);
                }
            }
            break;

        case APP_PARK_EV_ARM:
            if (state == APP_PARK_STATE_IDLE)
            {
                uint8_t i;
                uint8_t first;

                ResetWindow();
                windowId = r->w;

                first = (uint8_t)((historyNext + APP_PARK_TRACE_PRE_COUNT -
                                   historyCount) % APP_PARK_TRACE_PRE_COUNT);

                for (i = 0U; i < historyCount; i++)
                {
                    *copy = history[(first + i) % APP_PARK_TRACE_PRE_COUNT];
                    copy->ev = APP_PARK_EV_PRE;
                    (void)Store(copy);
                }

                stored = Store(r);
                state = APP_PARK_STATE_ARMED;
            }
            else if (state == APP_PARK_STATE_DRAIN)
            {
                /* The previous trace has not been sent yet. */
                if (windowsSkipped < UINT16_MAX) { windowsSkipped++; }
            }
            /* ARMED..BRAKE: re-arming inside an open window is not a new one. */
            break;

        case APP_PARK_EV_NEAR:
            if (state == APP_PARK_STATE_ARMED)
            {
                if (nearUsed < APP_PARK_TRACE_NEAR_CAP)
                {
                    stored = Store(r);
                    if (stored) { nearUsed++; }
                }
                else
                {
                    CountDrop(false);
                }
            }
            break;

        case APP_PARK_EV_REQ:
            if (state == APP_PARK_STATE_ARMED)
            {
                *copy = *r;
                copy->vl = Velocity(false);
                copy->vr = Velocity(true);
                reqTick = r->tick;
                stored = Store(copy);
                state = APP_PARK_STATE_REQ;
            }
            break;

        case APP_PARK_EV_ACC:
            if (state == APP_PARK_STATE_REQ)
            {
                stored = Store(r);
                state = APP_PARK_STATE_ACC;
            }
            break;

        case APP_PARK_EV_TAKE:
            if (state == APP_PARK_STATE_ACC)
            {
                stored = Store(r);
                state = APP_PARK_STATE_TAKE;
            }
            break;

        case APP_PARK_EV_BRK:
            if (state == APP_PARK_STATE_TAKE)
            {
                AddVelSample(r->tick, r->el, r->er);
                *copy = *r;
                copy->vl = Velocity(false);
                copy->vr = Velocity(true);
                stored = Store(copy);

                brkTick = r->tick;
                bendSeen = false;
                zeroDone = false;
                zeroTimeout = false;
                leftStill = false;
                rightStill = false;

                /* The rolling window uses samples at or after BRK only. */
                zh[0].tick = r->tick;
                zh[0].el = r->el;
                zh[0].er = r->er;
                zhNext = 1U;
                zhCount = 1U;

                state = APP_PARK_STATE_BRAKE;
            }
            break;

        case APP_PARK_EV_BEND:
            if ((state == APP_PARK_STATE_BRAKE) && !bendSeen)
            {
                stored = Store(r);
                bendSeen = true;
                CompleteIfReady(r->tick);
            }
            break;

        default:
            break;
    }

    return stored;
}

bool AppParkTrace_Submit(const AppParkTraceRecord *r)
{
    bool stored;

    if (r == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    stored = SubmitLocked(r);
    taskEXIT_CRITICAL();

    return stored;
}

void AppParkTrace_Latch(uint8_t ev, uint32_t tick, int32_t el, int32_t er)
{
    LatchedEvent *slot;

    if (ev == APP_PARK_EV_BRK)
    {
        slot = &latchBrk;
    }
    else if (ev == APP_PARK_EV_BEND)
    {
        slot = &latchBend;
    }
    else
    {
        return;
    }

    if (slot->pending != 0U)
    {
        /* Cannot happen (flushed every MotionTask cycle); counted if it does. */
        if (latchOverruns < UINT16_MAX) { latchOverruns++; }
        return;
    }

    slot->tick = tick;
    slot->el = el;
    slot->er = er;
    slot->pending = 1U;
}

/* The critical section is held by the caller. */
static void FlushLatch(LatchedEvent *slot, uint8_t ev)
{
    if (slot->pending != 0U)
    {
        AppParkTrace_InitRecord(&scratchLatch, ev, slot->tick, slot->el,
                                slot->er);
        slot->pending = 0U;
        (void)SubmitLocked(&scratchLatch);
    }
}

void AppParkTrace_Sample(uint32_t nowTick, int32_t el, int32_t er)
{
    AppParkTraceState s = state;

    if ((latchBrk.pending == 0U) && (latchBend.pending == 0U) &&
        (latchOverruns == 0U) &&
        ((s == APP_PARK_STATE_IDLE) || (s == APP_PARK_STATE_DRAIN)))
    {
        return;
    }

    taskENTER_CRITICAL();

    if (latchOverruns != 0U)
    {
        uint16_t n = latchOverruns;

        latchOverruns = 0U;
        while (n > 0U) { CountDrop(false); n--; }
    }

    /* BRK before BEND; both are ignored unless they belong to the traced stop. */
    FlushLatch(&latchBrk, APP_PARK_EV_BRK);
    FlushLatch(&latchBend, APP_PARK_EV_BEND);

    if (InWindow())
    {
        AddVelSample(nowTick, el, er);

        if ((state == APP_PARK_STATE_BRAKE) && !zeroDone)
        {
            ZeroStep(nowTick, el, er);
        }
    }

    taskEXIT_CRITICAL();
}

void AppParkTrace_Poll(uint32_t nowTick)
{
    taskENTER_CRITICAL();

    if ((state == APP_PARK_STATE_REQ) ||
        (state == APP_PARK_STATE_ACC) ||
        (state == APP_PARK_STATE_TAKE))
    {
        if (AppParkTrace_TickDelta(nowTick, reqTick) >=
            APP_PARK_TRACE_TIMEOUT_MS)
        {
            CloseDone(nowTick, APP_PARK_RES_TIMEOUT);
        }
    }
    else if (state == APP_PARK_STATE_BRAKE)
    {
        if (AppParkTrace_TickDelta(nowTick, brkTick) >=
            (APP_PARK_TRACE_TIMEOUT_MS + APP_PARK_TRACE_BEND_GRACE_MS))
        {
            zeroTimeout = zeroTimeout || !zeroDone;
            CloseDone(nowTick, APP_PARK_RES_TIMEOUT);
        }
    }

    taskEXIT_CRITICAL();
}

void AppParkTrace_MovementEnded(uint32_t nowTick)
{
    taskENTER_CRITICAL();

    if (state == APP_PARK_STATE_ARMED)
    {
        CloseDone(nowTick, APP_PARK_RES_NOREQ);
    }
    else if ((state == APP_PARK_STATE_REQ) || (state == APP_PARK_STATE_ACC))
    {
        CloseDone(nowTick, APP_PARK_RES_NOACC);
    }
    /* TAKE, BRAKE: MotionTask owns the rest of the window. */

    taskEXIT_CRITICAL();
}

void AppParkTrace_Refused(uint32_t nowTick)
{
    taskENTER_CRITICAL();

    if (state == APP_PARK_STATE_REQ)
    {
        CloseDone(nowTick, APP_PARK_RES_REFUSED);
    }

    taskEXIT_CRITICAL();
}

void AppParkTrace_Disabled(uint32_t nowTick)
{
    taskENTER_CRITICAL();

    if (state == APP_PARK_STATE_ARMED)
    {
        CloseDone(nowTick, APP_PARK_RES_ABORT);
    }

    taskEXIT_CRITICAL();
}

bool AppParkTrace_DrainReady(void)
{
    return state == APP_PARK_STATE_DRAIN;
}

bool AppParkTrace_Pop(AppParkTraceRecord *out)
{
    bool popped = false;

    if (out == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();

    if (state == APP_PARK_STATE_DRAIN)
    {
        if (ringUsed > 0U)
        {
            *out = ring[ringTail];
            ringTail = (uint8_t)((ringTail + 1U) % APP_PARK_TRACE_RING_SIZE);
            ringUsed--;
            popped = true;
        }

        if (ringUsed == 0U)
        {
            state = APP_PARK_STATE_IDLE;
        }
    }

    taskEXIT_CRITICAL();

    return popped;
}

AppParkTraceState AppParkTrace_GetState(void)
{
    return state;
}

uint16_t AppParkTrace_GetWindow(void)
{
    uint16_t w;

    taskENTER_CRITICAL();
    w = windowId;
    taskEXIT_CRITICAL();

    return w;
}

void AppParkTrace_GetStats(AppParkTraceStats *stats)
{
    if (stats == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    stats->state = state;
    stats->window = windowId;
    stats->stored = ringUsed;
    stats->windowDropped = droppedWindow;
    stats->windowRingFull = droppedRingFull;
    stats->lifetimeDropped = droppedLifetime;
    stats->windowsSkipped = windowsSkipped;
    taskEXIT_CRITICAL();
}

#endif /* APP_USPARK_TRACE */
