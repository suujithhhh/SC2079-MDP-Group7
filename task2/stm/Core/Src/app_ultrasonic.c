/*
 * app_ultrasonic.c
 *
 * Non-blocking ultrasonic ranging driver core. See app_ultrasonic.h for the
 * hardware description and the division of labour. This file has no HAL
 * dependency: all timer/GPIO access goes through app_ultrasonic_hw.h so it
 * can be host-tested against a simulated TIM8.
 *
 * Concurrency model
 *   - `rec` (the ISR record) is written by the two interrupt entry points.
 *     The task only touches it inside taskENTER_CRITICAL(), which masks the
 *     TIM8_CC interrupt (priority 5, see app_ultrasonic_hw.h), so every
 *     multi-field read and every DIER/CCER/CCR3 read-modify-write is atomic
 *     with respect to the interrupt.
 *   - `published` is written only by UltrasonicTask (inside a critical
 *     section) and copied by AppUltrasonic_GetSnapshot() inside one too.
 *     The interrupts never touch it.
 *   - Interrupt work is bounded: no loops, no RTOS calls, no floating point.
 */

#include "app_ultrasonic.h"
#include "app_ultrasonic_hw.h"
#include "FreeRTOS.h"
#include "task.h"

/* What the interrupts concluded about a cycle. Classification of a captured
 * pulse width (near/far/valid) is deliberately left to the task. */
typedef enum
{
    ISR_OUTCOME_NONE = 0,
    ISR_OUTCOME_CAPTURED,
    ISR_OUTCOME_NO_RISE,
    ISR_OUTCOME_NO_FALL,
    ISR_OUTCOME_EDGE_ERROR,
    ISR_OUTCOME_LATE_RISE
} IsrOutcome;

typedef struct
{
    volatile AppUltrasonicState state;
    volatile IsrOutcome outcome;
    volatile uint32_t cycleId;      /* id of the cycle this record belongs to */
    volatile uint16_t trigStart;    /* TIM8 count when PC6 went high          */
    volatile uint16_t rise;         /* TIM8 count of the rising Echo edge     */
    volatile uint16_t pulse;        /* fall - rise, valid when CAPTURED       */
    volatile uint16_t spurious;     /* saturating, ISR-owned                  */
    volatile uint16_t staleCompare; /* saturating, ISR-owned                  */
} IsrRecord;

static IsrRecord rec;

/* Task-owned state (UltrasonicTask only). */
static AppUltrasonicSnapshot published;
static uint32_t lastTriggerTick;
static uint32_t armedTick;
static uint32_t armedCycleId;
static bool hasTriggered;
static bool cycleInFlight;
static uint16_t faultMask;
static bool faultPublished;
static uint16_t history[APP_US_FILTER_DEPTH];
static uint8_t historyCount;
static uint8_t historyNext;

static void IncSat16(uint16_t *counter)
{
    if (*counter < UINT16_MAX)
    {
        (*counter)++;
    }
}

static void IncSatIsr16(volatile uint16_t *counter)
{
    if (*counter < UINT16_MAX)
    {
        (*counter)++;
    }
}

/* ---------------------------------------------------------------------- *
 * Interrupt side
 * ---------------------------------------------------------------------- */

/* Latch the cycle outcome and restore the capture hardware to its idle
 * configuration (both interrupts off, rising-edge polarity, flags cleared,
 * PC6 low). The state is written LAST so a task that sees DONE sees a
 * complete record. */
static void Finish(IsrOutcome outcome, uint16_t pulse)
{
    AppUltrasonicHw_EnableCompareIrq(false);
    AppUltrasonicHw_EnableCaptureIrq(false);
    AppUltrasonicHw_TriggerLow();
    AppUltrasonicHw_SetCapturePolarity(false);
    AppUltrasonicHw_ClearCaptureFlags();

    rec.outcome = outcome;
    rec.pulse = pulse;
    rec.state = APP_US_STATE_DONE;
}

void AppUltrasonic_OnCapture(void)
{
    /* Reading CCR2 first keeps the capture value; the overrun flag is then
     * read and cleared (the HAL clears CC2IF itself before this callback
     * but never touches CC2OF). */
    uint16_t capture = AppUltrasonicHw_ReadCapture();
    bool overrun = AppUltrasonicHw_TakeCaptureOverrun();
    uint16_t sinceTrigger = (uint16_t)(capture - rec.trigStart);

    switch (rec.state)
    {
        case APP_US_STATE_WAIT_RISE:
            if (overrun || (sinceTrigger < APP_US_TRIG_PULSE_US))
            {
                /* Two edges collided, or an edge predates the end of the
                 * Trigger pulse: out of order. */
                Finish(ISR_OUTCOME_EDGE_ERROR, 0U);
            }
            else if (sinceTrigger >
                     (uint16_t)(APP_US_TRIG_PULSE_US +
                                APP_US_RISE_MAX_DELAY_US))
            {
                Finish(ISR_OUTCOME_LATE_RISE, 0U);
            }
            else
            {
                rec.rise = capture;
                AppUltrasonicHw_SetCapturePolarity(true);
                AppUltrasonicHw_ClearCaptureFlags();
                rec.state = APP_US_STATE_WAIT_FALL;
            }
            break;

        case APP_US_STATE_WAIT_FALL:
            if (overrun)
            {
                Finish(ISR_OUTCOME_EDGE_ERROR, 0U);
            }
            else if (sinceTrigger > APP_US_ECHO_TIMEOUT_US)
            {
                /* Fell after the timeout deadline (the compare event was
                 * pending in the same interrupt entry): too late. */
                Finish(ISR_OUTCOME_NO_FALL, 0U);
            }
            else
            {
                /* 16-bit wrap-safe: unsigned subtraction modulo 65536. */
                Finish(ISR_OUTCOME_CAPTURED, (uint16_t)(capture - rec.rise));
            }
            break;

        default:
            /* Duplicate / late edge in a state that does not expect one. */
            IncSatIsr16(&rec.spurious);
            AppUltrasonicHw_EnableCaptureIrq(false);
            AppUltrasonicHw_ClearCaptureFlags();
            break;
    }
}

void AppUltrasonic_OnCompare(void)
{
    switch (rec.state)
    {
        case APP_US_STATE_TRIGGER_HIGH:
            /* End of the 10 us Trigger pulse; the same channel now becomes
             * the hardware no-Echo timeout. */
            AppUltrasonicHw_TriggerLow();
            AppUltrasonicHw_SetCapturePolarity(false);
            AppUltrasonicHw_ClearCaptureFlags();
            AppUltrasonicHw_EnableCaptureIrq(true);
            AppUltrasonicHw_SetCompare(
                (uint16_t)(rec.trigStart + APP_US_ECHO_TIMEOUT_US));
            rec.state = APP_US_STATE_WAIT_RISE;
            break;

        case APP_US_STATE_WAIT_RISE:
            Finish(ISR_OUTCOME_NO_RISE, 0U);
            break;

        case APP_US_STATE_WAIT_FALL:
            Finish(ISR_OUTCOME_NO_FALL, 0U);
            break;

        default:
            IncSatIsr16(&rec.staleCompare);
            AppUltrasonicHw_EnableCompareIrq(false);
            break;
    }
}

/* ---------------------------------------------------------------------- *
 * Task side
 * ---------------------------------------------------------------------- */

static void ClearFilter(void)
{
    historyCount = 0U;
    historyNext = 0U;
}

static void PushFilter(uint16_t mm)
{
    history[historyNext] = mm;
    historyNext = (uint8_t)((historyNext + 1U) % APP_US_FILTER_DEPTH);

    if (historyCount < APP_US_FILTER_DEPTH)
    {
        historyCount++;
    }
}

/* Deterministic, integer-only: 1 sample -> itself, 2 -> floor of the mean,
 * 3 -> median (rejects any single outlier). Requires historyCount >= 1. */
static uint16_t FilterValue(void)
{
    uint16_t a;
    uint16_t b;
    uint16_t c;
    uint16_t t;

    if (historyCount == 1U)
    {
        return history[0];
    }

    if (historyCount == 2U)
    {
        return (uint16_t)(((uint32_t)history[0] + (uint32_t)history[1]) / 2U);
    }

    a = history[0];
    b = history[1];
    c = history[2];

    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { t = a; a = b; b = t; }

    return b;
}

/* Compile-time guarantees, always checked against the PRODUCTION limits
 * (a host-test build may override only APP_US_MIN_VALID_PULSE_US):
 *  - the production minimum pulse converts to a strictly positive distance,
 *    so a valid sample can never be 0 mm or negative;
 *  - the largest possible 16-bit pulse cannot overflow the signed 32-bit
 *    intermediate product (65535 * 1744 + 5000 = 114 298 040). */
_Static_assert(
    ((((int32_t)APP_US_MIN_VALID_PULSE_US_PRODUCTION *
       APP_US_MM_PER_US_X10000) + 5000) / 10000 + APP_US_OFFSET_MM) > 0,
    "production minimum pulse must convert to a positive distance");
_Static_assert(
    ((int64_t)UINT16_MAX * APP_US_MM_PER_US_X10000 + 5000) <= INT32_MAX,
    "pulse-to-distance product must fit int32_t");

/* mm = ((pulse * slope + 5000) / 10000) + offset, in signed 32-bit
 * arithmetic. The numerator is non-negative, so the division floors; only
 * the offset can make the result negative. */
int32_t AppUltrasonic_PulseToMm(uint16_t pulseUs)
{
    return ((((int32_t)pulseUs * APP_US_MM_PER_US_X10000) + 5000) / 10000)
         + APP_US_OFFSET_MM;
}

static AppUltrasonicResult ClassifyPulse(uint16_t pulseUs, uint16_t *mm)
{
    int32_t value;

    if (pulseUs < APP_US_MIN_VALID_PULSE_US)
    {
        return APP_US_RESULT_TOO_NEAR;
    }

    if (pulseUs > APP_US_MAX_VALID_PULSE_US)
    {
        return APP_US_RESULT_TOO_FAR;
    }

    value = AppUltrasonic_PulseToMm(pulseUs);

    if (value <= 0)
    {
        /* A zero or negative distance is never reported as VALID. */
        return APP_US_RESULT_TOO_NEAR;
    }

    if (value > (int32_t)UINT16_MAX)
    {
        return APP_US_RESULT_TOO_FAR;
    }

    *mm = (uint16_t)value;
    return APP_US_RESULT_VALID;
}

static void CountResult(AppUltrasonicCounters *counters,
                        AppUltrasonicResult result)
{
    switch (result)
    {
        case APP_US_RESULT_VALID:      IncSat16(&counters->valid);     break;
        case APP_US_RESULT_NO_RISE:    IncSat16(&counters->noRise);    break;
        case APP_US_RESULT_NO_FALL:    IncSat16(&counters->noFall);    break;
        case APP_US_RESULT_TOO_NEAR:   IncSat16(&counters->tooNear);   break;
        case APP_US_RESULT_TOO_FAR:    IncSat16(&counters->tooFar);    break;
        case APP_US_RESULT_EDGE_ERROR: IncSat16(&counters->edgeError); break;
        case APP_US_RESULT_LATE_RISE:  IncSat16(&counters->lateRise);  break;
        case APP_US_RESULT_ECHO_STUCK: IncSat16(&counters->echoStuck); break;
        case APP_US_RESULT_STALL:      IncSat16(&counters->stall);     break;
        default:                       break;
    }
}

/* Build and publish a new snapshot. Validity is carried by `result` /
 * rawValid; rawMm is only meaningful when rawValid is set. */
static void Publish(
    uint32_t now,
    AppUltrasonicResult result,
    bool pulseCaptured,
    uint16_t pulseUs,
    bool rawValid,
    uint16_t rawMm)
{
    AppUltrasonicSnapshot next = published;   /* sole writer: safe to read */

    next.sequence++;
    if (next.sequence == 0U)
    {
        next.sequence++;
    }

    next.cycles++;
    CountResult(&next.counters, result);
    next.lastSampleTick = now;
    next.result = result;
    next.pulseCaptured = pulseCaptured ? 1U : 0U;
    next.pulseUs = pulseCaptured ? pulseUs : 0U;
    next.rawValid = rawValid ? 1U : 0U;
    next.rawMm = rawValid ? rawMm : 0U;
    next.faultMask = faultMask;

    if (rawValid)
    {
        next.lastValidTick = now;
    }

    next.filterSamples = historyCount;
    next.filteredValid = (historyCount > 0U) ? 1U : 0U;
    next.filteredMm = (historyCount > 0U) ? FilterValue() : 0U;

    taskENTER_CRITICAL();
    published = next;
    taskEXIT_CRITICAL();
}

/* Turn what the interrupts latched into a published result. Counters are
 * bumped on the local copy inside Publish(), so a reader never sees a
 * counter change without the matching sequence number. */
static void ProcessOutcome(uint32_t now, IsrOutcome outcome, uint16_t pulseUs)
{
    AppUltrasonicResult result;
    bool captured = false;
    bool valid = false;
    uint16_t mm = 0U;

    switch (outcome)
    {
        case ISR_OUTCOME_CAPTURED:
            captured = true;
            result = ClassifyPulse(pulseUs, &mm);
            valid = (result == APP_US_RESULT_VALID);
            break;
        case ISR_OUTCOME_NO_RISE:
            result = APP_US_RESULT_NO_RISE;
            break;
        case ISR_OUTCOME_NO_FALL:
            result = APP_US_RESULT_NO_FALL;
            break;
        case ISR_OUTCOME_LATE_RISE:
            result = APP_US_RESULT_LATE_RISE;
            break;
        case ISR_OUTCOME_EDGE_ERROR:
        default:
            result = APP_US_RESULT_EDGE_ERROR;
            break;
    }

    if (valid)
    {
        /* Never mix an old scene with a new one: drop history that has
         * been idle longer than the staleness limit. */
        if ((historyCount > 0U) &&
            ((uint32_t)(now - published.lastValidTick) >
             APP_US_STALE_TICKS_RTOS))
        {
            ClearFilter();
        }

        PushFilter(mm);
    }

    Publish(now, result, captured, pulseUs, valid, mm);
}

/* Abort a cycle that is still in flight (watchdog). Returns false if the
 * interrupt finished it first, in which case the DONE record is left for
 * the next Service() pass. */
static bool AbortCycle(void)
{
    bool aborted = false;

    taskENTER_CRITICAL();

    if ((rec.state == APP_US_STATE_TRIGGER_HIGH) ||
        (rec.state == APP_US_STATE_WAIT_RISE) ||
        (rec.state == APP_US_STATE_WAIT_FALL))
    {
        AppUltrasonicHw_EnableCompareIrq(false);
        AppUltrasonicHw_EnableCaptureIrq(false);
        AppUltrasonicHw_TriggerLow();
        AppUltrasonicHw_SetCapturePolarity(false);
        AppUltrasonicHw_ClearCaptureFlags();
        rec.state = APP_US_STATE_IDLE;
        aborted = true;
    }

    taskEXIT_CRITICAL();

    return aborted;
}

/* Start a measurement. PC6 is raised FIRST, then the counter is read and
 * the CH3 compare is set 11 ticks ahead, so the pulse is at least 10 us.
 * Everything happens inside one critical section, which also masks the
 * TIM8_CC interrupt while CCR3 / DIER are being written. */
static void ArmTrigger(uint32_t now)
{
    uint16_t start;

    armedCycleId++;

    taskENTER_CRITICAL();

    rec.cycleId = armedCycleId;
    rec.outcome = ISR_OUTCOME_NONE;
    rec.state = APP_US_STATE_TRIGGER_HIGH;

    AppUltrasonicHw_ClearCompareFlag();
    AppUltrasonicHw_TriggerHigh();
    start = AppUltrasonicHw_Counter();
    rec.trigStart = start;
    AppUltrasonicHw_SetCompare((uint16_t)(start + APP_US_TRIG_PULSE_US));
    AppUltrasonicHw_EnableCompareIrq(true);

    taskEXIT_CRITICAL();

    lastTriggerTick = now;
    armedTick = now;
    hasTriggered = true;
    cycleInFlight = true;
}

void AppUltrasonic_Init(void)
{
    taskENTER_CRITICAL();

    rec.state = APP_US_STATE_IDLE;
    rec.outcome = ISR_OUTCOME_NONE;
    rec.cycleId = 0U;
    rec.trigStart = 0U;
    rec.rise = 0U;
    rec.pulse = 0U;
    rec.spurious = 0U;
    rec.staleCompare = 0U;

    taskEXIT_CRITICAL();

    lastTriggerTick = 0U;
    armedTick = 0U;
    armedCycleId = 0U;
    hasTriggered = false;
    cycleInFlight = false;
    faultPublished = false;
    ClearFilter();

    taskENTER_CRITICAL();
    published = (AppUltrasonicSnapshot){0};
    published.result = APP_US_RESULT_NONE;
    taskEXIT_CRITICAL();

    faultMask = AppUltrasonicHw_Setup();

    if (faultMask != 0U)
    {
        taskENTER_CRITICAL();
        rec.state = APP_US_STATE_FAULT;
        taskEXIT_CRITICAL();
    }
}

void AppUltrasonic_ReportTaskCreateFailure(void)
{
    /* Pre-scheduler, single context: plain stores only (see the header). */
    AppUltrasonicHw_TriggerLow();

    rec.state = APP_US_STATE_FAULT;
    faultMask = APP_US_FAULT_TASK;
    faultPublished = true;

    published = (AppUltrasonicSnapshot){0};
    published.sequence = 1U;
    published.cycles = 1U;
    published.result = APP_US_RESULT_FAULT;
    published.faultMask = faultMask;
}

void AppUltrasonic_Service(uint32_t nowTick)
{
    AppUltrasonicState state;
    IsrOutcome outcome = ISR_OUTCOME_NONE;
    uint16_t pulse = 0U;
    uint32_t id = 0U;
    bool haveResult = false;

    if (faultMask != 0U)
    {
        /* Refused to run: publish the fault once, never touch PC6. */
        if (!faultPublished)
        {
            faultPublished = true;
            Publish(nowTick, APP_US_RESULT_FAULT, false, 0U, false, 0U);
        }
        return;
    }

    /* 1. Collect a finished cycle (one critical section, one handshake). */
    taskENTER_CRITICAL();
    state = rec.state;

    if (state == APP_US_STATE_DONE)
    {
        outcome = rec.outcome;
        pulse = rec.pulse;
        id = rec.cycleId;
        rec.state = APP_US_STATE_IDLE;
        haveResult = true;
    }

    taskEXIT_CRITICAL();

    if (haveResult)
    {
        state = APP_US_STATE_IDLE;

        if (cycleInFlight && (id == armedCycleId))
        {
            cycleInFlight = false;
            ProcessOutcome(nowTick, outcome, pulse);
        }
        else
        {
            /* A result for a cycle nobody is waiting for. */
            taskENTER_CRITICAL();
            IncSat16(&published.counters.staleResult);
            taskEXIT_CRITICAL();
        }
    }

    /* 2. Watchdogs: a lost compare interrupt must never leave PC6 high or
     *    the driver wedged. */
    if (((state == APP_US_STATE_TRIGGER_HIGH) &&
         ((uint32_t)(nowTick - armedTick) >=
          APP_US_TRIGGER_STALL_TICKS_RTOS)) ||
        (((state == APP_US_STATE_WAIT_RISE) ||
          (state == APP_US_STATE_WAIT_FALL)) &&
         ((uint32_t)(nowTick - armedTick) >=
          APP_US_CAPTURE_STALL_TICKS_RTOS)))
    {
        if (AbortCycle())
        {
            cycleInFlight = false;
            state = APP_US_STATE_IDLE;
            Publish(nowTick, APP_US_RESULT_STALL, false, 0U, false, 0U);
        }
    }

    /* 3. Trigger at most once per APP_US_MIN_TRIGGER_TICKS_RTOS. */
    if ((state == APP_US_STATE_IDLE) &&
        (!hasTriggered ||
         ((uint32_t)(nowTick - lastTriggerTick) >=
          APP_US_MIN_TRIGGER_TICKS_RTOS)))
    {
        if (AppUltrasonicHw_EchoHigh())
        {
            /* Echo is already high: a trigger now would be meaningless.
             * Skip it, drop the filter history, keep the spacing. */
            hasTriggered = true;
            lastTriggerTick = nowTick;
            ClearFilter();
            Publish(nowTick, APP_US_RESULT_ECHO_STUCK, false, 0U, false, 0U);
        }
        else
        {
            ArmTrigger(nowTick);
        }
    }
}

void AppUltrasonic_GetSnapshot(AppUltrasonicSnapshot *snapshot)
{
    if (snapshot == 0)
    {
        return;
    }

    taskENTER_CRITICAL();

    *snapshot = published;
    snapshot->state = rec.state;
    snapshot->counters.spurious = rec.spurious;
    snapshot->counters.staleCompare = rec.staleCompare;

    taskEXIT_CRITICAL();
}
