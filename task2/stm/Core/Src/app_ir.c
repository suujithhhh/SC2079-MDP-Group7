/*
 * app_ir.c
 *
 * IR acquisition core: raw two-channel ADC scans, one at a time, published as
 * an atomic snapshot (Phase 1, unchanged), plus the Phase 3 per-sensor filter
 * whose output travels in the same snapshot. HAL-free (all hardware access
 * goes through app_ir_hw.h) so the host tests can link it against a fake
 * port. See app_ir.h for the hardware mapping and the public contract and
 * app_ir_dist.h for the filter and the millimetre conversion.
 *
 * Scan life cycle (IRTask, AppIr_Service, every APP_IR_SERVICE_PERIOD_TICKS):
 *   1. If a scan is in flight, collect its outcome (DONE / ERROR from the
 *      ISR, or a timeout), validate, recover after any error, publish.
 *   2. If APP_IR_SAMPLE_PERIOD_TICKS have passed since the last start, start
 *      the next scan: check the handles are ready, write the sentinel into
 *      both DMA words, ARM the ISR record, then call the HAL start.
 * Arming BEFORE the start matters: the scan takes ~9 us and can complete
 * before HAL_ADC_Start_DMA() returns if IRTask is pre-empted in between.
 *
 * Concurrency:
 *   - The ISR record (isrArmed/isrOutcome/isrErrorCode/isrSpurious) is shared
 *     with DMA2_Stream0_IRQHandler (priority 5). IRTask only touches it inside
 *     taskENTER_CRITICAL(), which masks that interrupt. The ISR entry points
 *     disarm on the first outcome, so one scan yields at most one outcome and
 *     any later callback is only counted as spurious.
 *   - When a timeout is declared the record is disarmed inside the same
 *     critical section that observed "no outcome", so a completion racing
 *     the timeout is either collected or counted as spurious, never both.
 *   - The snapshot has a single writer (IRTask) and is copied in and out
 *     under one critical section. The ISR never touches it.
 *   - Everything else here is IRTask-private, including the Phase 3 filters.
 *     The filtered counts are published only through the snapshot, so a
 *     consumer sees raw and filtered values of one and the same scan.
 *
 * Fault policy: configuration, task-creation and recovery failures LATCH
 * (status FAULT, the ADC is never touched again). Start, busy, timeout, DMA,
 * ADC and data errors are transient: published as ERROR, followed by a
 * recovery of the ADC/DMA handles, and the next scan starts on schedule.
 * There is deliberately no "N consecutive failures" latch.
 */

#include "app_ir.h"

#if APP_IR_ENABLE

#include "app_ir_hw.h"
#include "FreeRTOS.h"
#include "task.h"

/* The DMA element order, the snapshot index and AppIrSensor must agree. */
_Static_assert(
    ((unsigned)APP_IR_DMA_INDEX_LEFT == (unsigned)APP_IR_SENSOR_LEFT) &&
    ((unsigned)APP_IR_DMA_INDEX_RIGHT == (unsigned)APP_IR_SENSOR_RIGHT),
    "IR sensor indices must match the DMA element order");

typedef enum
{
    IR_OUTCOME_NONE = 0,
    IR_OUTCOME_DONE,
    IR_OUTCOME_ERROR
} IrOutcome;

/* DMA destination (halfword elements, word aligned). Plain .bss, which the
 * linker places in SRAM1 (0x2000_0000): DMA2 cannot reach CCMRAM. */
static volatile uint16_t irDmaBuffer[APP_IR_DMA_COUNT] __attribute__((aligned(4)));

/* Shared with the ISR (see the header comment). */
static volatile uint8_t isrArmed;
static volatile uint8_t isrOutcome;
static volatile uint32_t isrErrorCode;
static volatile uint16_t isrSpurious;

/* IRTask-private state. */
static AppIrSnapshot published;
static AppIrCounters counters;
static bool scanInFlight;
static uint32_t scanStartTick;
static bool hasStarted;
static uint32_t lastStartTick;
static bool faulted;
static bool faultPublished;
static uint16_t latchedMask;

/* Phase 3 (IRTask-private). Index 0 = LEFT, 1 = RIGHT. filteredNow* is what
 * the next VALID publication carries; it is refreshed by UpdateFilters()
 * before Publish() so the two never nest on the stack. */
static AppIrFilter filters[2];
static uint32_t filterLastTick;
static bool filterHasTick;
static uint32_t firstStartTick;
static uint16_t filteredNow[2];
static uint8_t filteredNowSamples;

static void IncSat16(uint16_t *counter)
{
    if (*counter < UINT16_MAX)
    {
        (*counter)++;
    }
}

/* ---------------------------------------------------------------------- *
 * Interrupt entry points
 * ---------------------------------------------------------------------- */

static void IncSatIsr16(volatile uint16_t *counter)
{
    uint16_t value = *counter;

    if (value < UINT16_MAX)
    {
        *counter = (uint16_t)(value + 1U);
    }
}

void AppIr_OnConvComplete(void)
{
    if (isrArmed != 0U)
    {
        isrArmed = 0U;
        isrOutcome = (uint8_t)IR_OUTCOME_DONE;
    }
    else
    {
        IncSatIsr16(&isrSpurious);
    }
}

void AppIr_OnError(uint32_t halAdcErrorCode)
{
    if (isrArmed != 0U)
    {
        isrArmed = 0U;
        isrErrorCode = halAdcErrorCode;
        isrOutcome = (uint8_t)IR_OUTCOME_ERROR;
    }
    else
    {
        IncSatIsr16(&isrSpurious);
    }
}

/* ---------------------------------------------------------------------- *
 * Publication
 * ---------------------------------------------------------------------- */

static void Publish(
    uint32_t now,
    AppIrStatus status,
    uint16_t transientMask,
    uint16_t left,
    uint16_t right)
{
    AppIrSnapshot next = published;     /* sole writer: safe to read */

    next.sequence++;
    if (next.sequence == 0U)
    {
        next.sequence++;
    }

    next.publishTick = now;
    next.status = (uint8_t)status;
    next.faultMask = (uint16_t)(transientMask | latchedMask);

    if (status == APP_IR_STATUS_VALID)
    {
        next.leftRaw = left;
        next.rightRaw = right;
        next.lastValidTick = now;
        next.consecutiveFailures = 0U;
        next.filteredRaw[APP_IR_DMA_INDEX_LEFT] = filteredNow[APP_IR_DMA_INDEX_LEFT];
        next.filteredRaw[APP_IR_DMA_INDEX_RIGHT] = filteredNow[APP_IR_DMA_INDEX_RIGHT];
        next.filterSamples = filteredNowSamples;
    }
    else
    {
        next.leftRaw = 0U;
        next.rightRaw = 0U;
        next.filteredRaw[APP_IR_DMA_INDEX_LEFT] = 0U;
        next.filteredRaw[APP_IR_DMA_INDEX_RIGHT] = 0U;
        next.filterSamples = 0U;

        if (next.consecutiveFailures < UINT8_MAX)
        {
            next.consecutiveFailures++;
        }
    }

    next.counters = counters;

    taskENTER_CRITICAL();
    published = next;
    taskEXIT_CRITICAL();
}

/* Phase 3: feeds one VALID scan to the two filters and prepares the filtered
 * counts the publication will carry.
 *   - A gap of more than APP_IR_STALE_TICKS since the previous VALID scan
 *     (a run of failed scans) empties the filters: the old window no longer
 *     describes the present.
 *   - Scans during the first APP_IR_DIST_STARTUP_TICKS after the first start
 *     are published (raw) but not filtered.
 *   - Transient ERROR publications leave the filters untouched.
 * filteredNowSamples reaching APP_IR_FILTER_WINDOW means the filtered counts
 * are usable; otherwise they are 0. */
static void UpdateFilters(uint32_t now, uint16_t left, uint16_t right)
{
    if (filterHasTick &&
        ((uint32_t)(now - filterLastTick) > APP_IR_STALE_TICKS))
    {
        AppIrFilter_Reset(&filters[APP_IR_DMA_INDEX_LEFT]);
        AppIrFilter_Reset(&filters[APP_IR_DMA_INDEX_RIGHT]);
    }

    filterLastTick = now;
    filterHasTick = true;

    if ((uint32_t)(now - firstStartTick) >= APP_IR_DIST_STARTUP_TICKS)
    {
        AppIrFilter_Push(&filters[APP_IR_DMA_INDEX_LEFT], left);
        AppIrFilter_Push(&filters[APP_IR_DMA_INDEX_RIGHT], right);
    }

    filteredNow[APP_IR_DMA_INDEX_LEFT] = 0U;
    filteredNow[APP_IR_DMA_INDEX_RIGHT] = 0U;
    filteredNowSamples = filters[APP_IR_DMA_INDEX_LEFT].count;

    if (AppIrFilter_IsFull(&filters[APP_IR_DMA_INDEX_LEFT]))
    {
        (void)AppIrFilter_Output(
            &filters[APP_IR_DMA_INDEX_LEFT],
            &filteredNow[APP_IR_DMA_INDEX_LEFT]);
        (void)AppIrFilter_Output(
            &filters[APP_IR_DMA_INDEX_RIGHT],
            &filteredNow[APP_IR_DMA_INDEX_RIGHT]);
    }
}

/* A scan failed (mask != 0): return the hardware to READY, then publish. A
 * failed recovery latches FAULT and the ADC is never started again. */
static void FailScan(uint32_t now, uint16_t mask)
{
    if (AppIrHw_Recover())
    {
        IncSat16(&counters.recoveries);
        Publish(now, APP_IR_STATUS_ERROR, mask, 0U, 0U);
    }
    else
    {
        latchedMask = (uint16_t)(latchedMask | APP_IR_FAULT_RECOVERY);
        faulted = true;
        faultPublished = true;
        Publish(now, APP_IR_STATUS_FAULT, mask, 0U, 0U);
    }
}

/* ---------------------------------------------------------------------- *
 * Scan collection and start
 * ---------------------------------------------------------------------- */

/* Returns false while the in-flight scan is neither finished nor timed out. */
static bool CollectScan(uint32_t now)
{
    bool timedOut = (uint32_t)(now - scanStartTick) >= APP_IR_SCAN_TIMEOUT_TICKS;
    uint8_t outcome;
    uint32_t errorCode;
    uint16_t mask = 0U;
    uint16_t left = 0U;
    uint16_t right = 0U;

    taskENTER_CRITICAL();
    outcome = isrOutcome;
    errorCode = isrErrorCode;

    if ((outcome == (uint8_t)IR_OUTCOME_NONE) && timedOut)
    {
        isrArmed = 0U;      /* a later completion is now only "spurious" */
    }
    taskEXIT_CRITICAL();

    if ((outcome == (uint8_t)IR_OUTCOME_NONE) && !timedOut)
    {
        return false;
    }

    scanInFlight = false;

    if (outcome == (uint8_t)IR_OUTCOME_DONE)
    {
        left = irDmaBuffer[APP_IR_DMA_INDEX_LEFT];
        right = irDmaBuffer[APP_IR_DMA_INDEX_RIGHT];

        /* Integrity only (the sentinel is > APP_IR_ADC_MAX): 0 and 4095 are
         * valid raw conversions. */
        if ((AppIrHw_Remaining() != 0U) ||
            (left > APP_IR_ADC_MAX) ||
            (right > APP_IR_ADC_MAX))
        {
            mask = APP_IR_FAULT_DATA;
            IncSat16(&counters.dataError);
        }
    }
    else if (outcome == (uint8_t)IR_OUTCOME_ERROR)
    {
        if (AppIrHw_ErrorIsDma(errorCode))
        {
            mask = APP_IR_FAULT_DMA;
            IncSat16(&counters.dmaError);
        }
        else
        {
            mask = APP_IR_FAULT_ADC;
            IncSat16(&counters.adcError);
        }

        if (AppIrHw_TakeOverrun() && (mask != APP_IR_FAULT_ADC))
        {
            mask = (uint16_t)(mask | APP_IR_FAULT_ADC);
            IncSat16(&counters.adcError);
        }
    }
    else
    {
        mask = APP_IR_FAULT_TIMEOUT;
        IncSat16(&counters.timeout);

        if (AppIrHw_TakeOverrun())
        {
            mask = (uint16_t)(mask | APP_IR_FAULT_ADC);
            IncSat16(&counters.adcError);
        }
    }

    if (mask == 0U)
    {
        IncSat16(&counters.valid);
        UpdateFilters(now, left, right);
        Publish(now, APP_IR_STATUS_VALID, 0U, left, right);
    }
    else
    {
        FailScan(now, mask);
    }

    return true;
}

static void StartScan(uint32_t now)
{
    if (!hasStarted)
    {
        firstStartTick = now;
    }

    hasStarted = true;
    lastStartTick = now;

    if (!AppIrHw_Ready())
    {
        IncSat16(&counters.busy);
        FailScan(now, APP_IR_FAULT_BUSY);
        return;
    }

    irDmaBuffer[APP_IR_DMA_INDEX_LEFT] = APP_IR_DMA_SENTINEL;
    irDmaBuffer[APP_IR_DMA_INDEX_RIGHT] = APP_IR_DMA_SENTINEL;

    taskENTER_CRITICAL();
    isrOutcome = (uint8_t)IR_OUTCOME_NONE;
    isrErrorCode = 0U;
    isrArmed = 1U;
    taskEXIT_CRITICAL();

    scanStartTick = now;
    scanInFlight = true;

    if (!AppIrHw_Start(irDmaBuffer))
    {
        taskENTER_CRITICAL();
        isrArmed = 0U;
        taskEXIT_CRITICAL();

        scanInFlight = false;
        IncSat16(&counters.startFail);
        FailScan(now, APP_IR_FAULT_START);
    }
}

/* ---------------------------------------------------------------------- *
 * Task API
 * ---------------------------------------------------------------------- */

void AppIr_Init(void)
{
    static const AppIrSnapshot empty = {0};
    static const AppIrCounters zero = {0};
    uint16_t cfg;

    taskENTER_CRITICAL();
    isrArmed = 0U;
    isrOutcome = (uint8_t)IR_OUTCOME_NONE;
    isrErrorCode = 0U;
    isrSpurious = 0U;
    published = empty;
    taskEXIT_CRITICAL();

    counters = zero;
    scanInFlight = false;
    scanStartTick = 0U;
    hasStarted = false;
    lastStartTick = 0U;
    faulted = false;
    faultPublished = false;
    latchedMask = 0U;

    AppIrFilter_Reset(&filters[APP_IR_DMA_INDEX_LEFT]);
    AppIrFilter_Reset(&filters[APP_IR_DMA_INDEX_RIGHT]);
    filterLastTick = 0U;
    filterHasTick = false;
    firstStartTick = 0U;
    filteredNow[APP_IR_DMA_INDEX_LEFT] = 0U;
    filteredNow[APP_IR_DMA_INDEX_RIGHT] = 0U;
    filteredNowSamples = 0U;

    cfg = AppIrHw_CheckConfig();

    if (cfg != 0U)
    {
        /* Fail closed: never start the ADC. Published by the first Service()
         * so it carries a real tick. */
        latchedMask = cfg;
        faulted = true;
    }
}

void AppIr_ReportTaskCreateFailure(void)
{
    static const AppIrSnapshot empty = {0};

    /* Pre-scheduler (no critical section available yet, nothing else runs).
     * No hardware is touched: the ADC is simply never started. */
    published = empty;
    published.sequence = 1U;
    published.publishTick = 0U;
    published.status = (uint8_t)APP_IR_STATUS_FAULT;
    published.faultMask = APP_IR_FAULT_TASK;
    published.consecutiveFailures = 1U;
    latchedMask = APP_IR_FAULT_TASK;
    faulted = true;
    faultPublished = true;
}

void AppIr_Service(uint32_t nowTick)
{
    if (faulted)
    {
        if (!faultPublished)
        {
            faultPublished = true;
            Publish(nowTick, APP_IR_STATUS_FAULT, 0U, 0U, 0U);
        }
        return;
    }

    if (scanInFlight)
    {
        if (!CollectScan(nowTick))
        {
            return;
        }

        if (faulted)
        {
            return;
        }
    }

    if (!hasStarted ||
        ((uint32_t)(nowTick - lastStartTick) >= APP_IR_SAMPLE_PERIOD_TICKS))
    {
        StartScan(nowTick);
    }
}

void AppIr_GetSnapshot(AppIrSnapshot *snapshot)
{
    if (snapshot == 0)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = published;
    snapshot->counters.spurious = isrSpurious;
    taskEXIT_CRITICAL();
}

/* Phase 3: derive the calibrated reading from ONE snapshot. */
void AppIr_GetReading(AppIrReading *reading, uint32_t nowTick)
{
    AppIrSnapshot snapshot;
    uint8_t sensor;

    if (reading == 0)
    {
        return;
    }

    AppIr_GetSnapshot(&snapshot);

    reading->sequence = snapshot.sequence;
    reading->faultMask = AppIr_ReportMask(&snapshot, nowTick);
    reading->ageTicks = (snapshot.sequence == 0U)
        ? UINT32_MAX
        : (uint32_t)(nowTick - snapshot.publishTick);

    for (sensor = 0U; sensor < 2U; sensor++)
    {
        reading->range[sensor] = (uint8_t)APP_IR_RANGE_INVALID;
        reading->distanceMm[sensor] = 0U;
        reading->filteredRaw[sensor] = 0U;
    }

    /* First match wins: FAULT, STALE, ERROR, WARMUP, OK. A latched FAULT
     * stays a FAULT however old it is; a failed scan that is itself old is
     * STALE (IRTask stopped publishing), not ERROR. */
    if (snapshot.status == (uint8_t)APP_IR_STATUS_FAULT)
    {
        reading->acq = (uint8_t)APP_IR_ACQ_FAULT;
    }
    else if ((snapshot.sequence == 0U) ||
             (snapshot.status == (uint8_t)APP_IR_STATUS_NONE) ||
             (reading->ageTicks > APP_IR_STALE_TICKS))
    {
        reading->acq = (uint8_t)APP_IR_ACQ_STALE;
    }
    else if (snapshot.status != (uint8_t)APP_IR_STATUS_VALID)
    {
        reading->acq = (uint8_t)APP_IR_ACQ_ERROR;
    }
    else if (snapshot.filterSamples < APP_IR_FILTER_WINDOW)
    {
        reading->acq = (uint8_t)APP_IR_ACQ_WARMUP;
    }
    else
    {
        reading->acq = (uint8_t)APP_IR_ACQ_OK;

        for (sensor = 0U; sensor < 2U; sensor++)
        {
            reading->filteredRaw[sensor] = snapshot.filteredRaw[sensor];
            reading->range[sensor] = (uint8_t)AppIrDist_Convert(
                (AppIrSensor)sensor,
                snapshot.filteredRaw[sensor],
                &reading->distanceMm[sensor]);
        }
    }
}

#ifdef APP_IR_TEST_HOOKS
void AppIr_TestSetSequence(uint32_t sequence)
{
    published.sequence = sequence;
}
#endif

#endif /* APP_IR_ENABLE */
