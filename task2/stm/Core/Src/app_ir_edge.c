/*
 * app_ir_edge.c
 *
 * Phase 5 obstacle-edge detector. HAL-free and RTOS-free. See app_ir_edge.h
 * for the rules and the reasoning. Compiled only when APP_IR_ENABLE is 1.
 */

#include "app_ir_edge.h"

#if APP_IR_ENABLE

#include "app_ir_dist.h"

static AppIrSensor SensorOf(uint8_t side)
{
    return (side == (uint8_t)APP_IR_EDGE_SIDE_RIGHT) ? APP_IR_SENSOR_RIGHT
                                                     : APP_IR_SENSOR_LEFT;
}

/* "At least as close as the visible threshold": NEAR, or converted and at
 * most VISIBLE_MAX. Monotone in the count (the conversion falls as the count
 * rises), false at 0 and true at 4095. */
static bool IsVisibleCount(AppIrSensor sensor, uint16_t count)
{
    uint16_t mm;
    AppIrRange range = AppIrDist_Convert(sensor, count, &mm);

    return (range == APP_IR_RANGE_NEAR) ||
           ((range == APP_IR_RANGE_OK) && (mm <= APP_IR_EDGE_VISIBLE_MAX_MM));
}

/* "At least as far as the clear threshold": FAR, or converted and at least
 * CLEAR_MIN. Monotone the other way: true at 0, false at 4095. */
static bool IsClearCount(AppIrSensor sensor, uint16_t count)
{
    uint16_t mm;
    AppIrRange range = AppIrDist_Convert(sensor, count, &mm);

    return (range == APP_IR_RANGE_FAR) ||
           ((range == APP_IR_RANGE_OK) && (mm >= APP_IR_EDGE_CLEAR_MIN_MM));
}

static uint16_t IncSat16(uint16_t value)
{
    return (value == UINT16_MAX) ? value : (uint16_t)(value + 1U);
}

static AppIrEdgeOutcome Fail(AppIrEdgeDetector *d, AppIrEdgeFailReason reason)
{
    d->phase = (uint8_t)APP_IR_EDGE_PHASE_FAILED;
    d->failReason = (uint8_t)reason;
    return APP_IR_EDGE_OUT_FAILED;
}

bool AppIrEdge_Start(AppIrEdgeDetector *detector, AppIrEdgeSide side, uint32_t nowTick)
{
    AppIrSensor sensor;
    uint16_t lo;
    uint16_t hi;

    if (detector == 0)
    {
        return false;
    }

    *detector = (AppIrEdgeDetector){0};
    detector->startTick = nowTick;
    detector->lastUsableTick = nowTick;
    detector->side = (uint8_t)side;

    if ((side != APP_IR_EDGE_SIDE_LEFT) && (side != APP_IR_EDGE_SIDE_RIGHT))
    {
        detector->phase = (uint8_t)APP_IR_EDGE_PHASE_FAILED;
        detector->failReason = (uint8_t)APP_IR_EDGE_FAIL_CONFIG;
        return false;
    }

    sensor = SensorOf(detector->side);

    /* Lowest VISIBLE count: smallest c in [0, 4095] with IsVisibleCount(c). */
    if (!IsVisibleCount(sensor, APP_IR_ADC_MAX) || IsVisibleCount(sensor, 0U))
    {
        detector->phase = (uint8_t)APP_IR_EDGE_PHASE_FAILED;
        detector->failReason = (uint8_t)APP_IR_EDGE_FAIL_CONFIG;
        return false;
    }

    lo = 0U;                    /* not visible */
    hi = APP_IR_ADC_MAX;        /* visible     */

    while ((uint16_t)(hi - lo) > 1U)
    {
        uint16_t mid = (uint16_t)(lo + ((hi - lo) / 2U));

        if (IsVisibleCount(sensor, mid))
        {
            hi = mid;
        }
        else
        {
            lo = mid;
        }
    }

    detector->visibleMinAdc = hi;

    /* Highest CLEAR count: largest c with IsClearCount(c). */
    if (!IsClearCount(sensor, 0U) || IsClearCount(sensor, APP_IR_ADC_MAX))
    {
        detector->phase = (uint8_t)APP_IR_EDGE_PHASE_FAILED;
        detector->failReason = (uint8_t)APP_IR_EDGE_FAIL_CONFIG;
        return false;
    }

    lo = 0U;                    /* clear     */
    hi = APP_IR_ADC_MAX;        /* not clear */

    while ((uint16_t)(hi - lo) > 1U)
    {
        uint16_t mid = (uint16_t)(lo + ((hi - lo) / 2U));

        if (IsClearCount(sensor, mid))
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }

    detector->clearMaxAdc = lo;

    if (detector->clearMaxAdc >= detector->visibleMinAdc)
    {
        detector->phase = (uint8_t)APP_IR_EDGE_PHASE_FAILED;
        detector->failReason = (uint8_t)APP_IR_EDGE_FAIL_CONFIG;
        return false;
    }

    detector->phase = (uint8_t)APP_IR_EDGE_PHASE_READY;
    return true;
}

AppIrEdgeClass AppIrEdge_ClassifyRaw(const AppIrEdgeDetector *detector, uint16_t raw)
{
    if (raw >= detector->visibleMinAdc)
    {
        return APP_IR_EDGE_CLASS_VISIBLE;
    }

    if (raw <= detector->clearMaxAdc)
    {
        return APP_IR_EDGE_CLASS_CLEAR;
    }

    return APP_IR_EDGE_CLASS_AMBIGUOUS;
}

/* The class the current phase is counting. */
static AppIrEdgeClass WantedClass(const AppIrEdgeDetector *d)
{
    return (d->phase == (uint8_t)APP_IR_EDGE_PHASE_WAIT_CLEAR)
               ? APP_IR_EDGE_CLASS_CLEAR
               : APP_IR_EDGE_CLASS_VISIBLE;
}

/* Handles the one new publication. Returns the outcome it caused. */
static AppIrEdgeOutcome ConsumePublication(AppIrEdgeDetector *d,
                                           const AppIrSnapshot *s,
                                           uint32_t nowTick)
{
    uint32_t expected;
    uint16_t raw;
    AppIrEdgeClass cls;

    /* Made before the command started: not a measurement of this pass. Only
     * remembered, so the next publication is a normal successor. */
    if ((int32_t)(s->publishTick - d->startTick) < 0)
    {
        d->lastSequence = s->sequence;
        d->haveSequence = 1U;
        return APP_IR_EDGE_OUT_NONE;
    }

    if (d->haveSequence != 0U)
    {
        expected = d->lastSequence + 1U;

        if (expected == 0U)
        {
            expected = 1U;      /* the sequence skips 0 on wrap */
        }

        if (s->sequence != expected)
        {
            d->missedScans = IncSat16(d->missedScans);
            d->streak = 0U;
            d->readyCount = 0U;
        }
    }

    d->lastSequence = s->sequence;
    d->haveSequence = 1U;

    if ((s->status != (uint8_t)APP_IR_STATUS_VALID) ||
        ((uint32_t)(nowTick - s->publishTick) > APP_IR_STALE_TICKS))
    {
        d->unusableScans = IncSat16(d->unusableScans);
        d->streak = 0U;
        d->readyCount = 0U;
        return APP_IR_EDGE_OUT_NONE;
    }

    raw = (d->side == (uint8_t)APP_IR_EDGE_SIDE_RIGHT) ? s->rightRaw : s->leftRaw;
    cls = AppIrEdge_ClassifyRaw(d, raw);
    d->lastUsableTick = nowTick;

    if (cls == APP_IR_EDGE_CLASS_AMBIGUOUS)
    {
        d->ambiguousScans = IncSat16(d->ambiguousScans);
    }

    if (d->phase == (uint8_t)APP_IR_EDGE_PHASE_READY)
    {
        if (d->firstScanDone == 0U)
        {
            /* What the robot starts next to (the first usable scan). */
            d->startedVisible = (cls == APP_IR_EDGE_CLASS_VISIBLE) ? 1U : 0U;
            d->firstScanDone = 1U;
        }

        if (d->readyCount < UINT8_MAX)
        {
            d->readyCount++;
        }
    }

    if (cls == WantedClass(d))
    {
        if (d->streak < UINT8_MAX)
        {
            d->streak++;
        }
    }
    else
    {
        d->streak = 0U;
    }

    if (d->phase == (uint8_t)APP_IR_EDGE_PHASE_READY)
    {
        if (d->readyCount >= APP_IR_EDGE_READY_SCANS)
        {
            d->phase = (uint8_t)APP_IR_EDGE_PHASE_WAIT_OBSTACLE;
            return APP_IR_EDGE_OUT_READY;
        }

        return APP_IR_EDGE_OUT_NONE;
    }

    if ((d->phase == (uint8_t)APP_IR_EDGE_PHASE_WAIT_OBSTACLE) &&
        (d->streak >= APP_IR_EDGE_SEEN_SCANS))
    {
        d->phase = (uint8_t)APP_IR_EDGE_PHASE_WAIT_CLEAR;
        d->streak = 0U;
        d->seenRaw = raw;
        return APP_IR_EDGE_OUT_SEEN;
    }

    if ((d->phase == (uint8_t)APP_IR_EDGE_PHASE_WAIT_CLEAR) &&
        (d->streak >= APP_IR_EDGE_CLEAR_SCANS))
    {
        d->phase = (uint8_t)APP_IR_EDGE_PHASE_CLEARED;
        d->clearRaw = raw;
        return APP_IR_EDGE_OUT_CLEARED;
    }

    return APP_IR_EDGE_OUT_NONE;
}

AppIrEdgeOutcome AppIrEdge_Step(AppIrEdgeDetector *detector,
                                const AppIrSnapshot *snapshot,
                                uint32_t nowTick)
{
    AppIrEdgeOutcome outcome = APP_IR_EDGE_OUT_NONE;

    if ((detector == 0) || (snapshot == 0) ||
        (detector->phase == (uint8_t)APP_IR_EDGE_PHASE_CLEARED) ||
        (detector->phase == (uint8_t)APP_IR_EDGE_PHASE_FAILED))
    {
        return APP_IR_EDGE_OUT_NONE;
    }

    /* A latched fault never recovers: fail at once, however old. */
    if (snapshot->status == (uint8_t)APP_IR_STATUS_FAULT)
    {
        return Fail(detector, APP_IR_EDGE_FAIL_FAULT);
    }

    /* A new publication: not sequence 0 (never published), and not the one
     * already consumed. */
    if ((snapshot->sequence != 0U) &&
        ((detector->haveSequence == 0U) ||
         (snapshot->sequence != detector->lastSequence)))
    {
        outcome = ConsumePublication(detector, snapshot, nowTick);

        if (outcome != APP_IR_EDGE_OUT_NONE)
        {
            return outcome;
        }
    }

    if (detector->phase == (uint8_t)APP_IR_EDGE_PHASE_READY)
    {
        if ((uint32_t)(nowTick - detector->startTick) > APP_IR_EDGE_READY_TIMEOUT_TICKS)
        {
            return Fail(detector, APP_IR_EDGE_FAIL_READY_TIMEOUT);
        }
    }
    else if ((uint32_t)(nowTick - detector->lastUsableTick) >
             APP_IR_EDGE_UNAVAILABLE_LIMIT_TICKS)
    {
        return Fail(detector, APP_IR_EDGE_FAIL_DATA_LOST);
    }

    return APP_IR_EDGE_OUT_NONE;
}

#endif /* APP_IR_ENABLE */
