/*
 * app_ir.h
 *
 * IR sensors. Phase 1: non-blocking RAW acquisition of the two analogue IR
 * sensors (raw 12-bit ADC counts in AppIrSnapshot). Phase 3: on top of that
 * unchanged acquisition, a per-sensor sample filter and a calibrated
 * conversion to millimetres FROM THE SENSOR FACE (app_ir_dist.h), exposed
 * through AppIr_GetReading(). Nothing here detects obstacles or is connected
 * to motion / STOP: the reading is telemetry for the RPi / Algo side only, and
 * no motion code may depend on it.
 *
 * Hardware (pre-existing CubeMX configuration, none of it changed here):
 *   PC0 = ADC1_IN10, regular rank 1 -> DMA element 0 -> LEFT  sensor
 *   PC1 = ADC1_IN11, regular rank 2 -> DMA element 1 -> RIGHT sensor
 *   ADC1: 12-bit, right aligned, scan of 2, single (not continuous),
 *         software start, DMA requests NOT continuous (DDS = 0).
 *   DMA2 Stream0 channel 0: peripheral-to-memory, halfword/halfword,
 *         memory increment, NORMAL (not circular) mode, priority 5 IRQ.
 *   ADC_IRQn is NOT enabled (no ADC1 interrupt handler exists).
 * Because the DMA runs in normal mode with DDS = 0, every two-channel scan
 * is explicitly started by IRTask; at most one scan is ever in flight.
 *
 * Division of labour (same split as the ultrasonic driver):
 *   - AppIr_OnConvComplete() / AppIr_OnError(): interrupt entry points,
 *     called only from the HAL ADC callbacks in main.c. A few stores each.
 *   - AppIr_Service(): IRTask. Starts scans, collects or times them out,
 *     validates, publishes and recovers.
 *   - AppIr_GetSnapshot(): any task, atomic copy of the latest publication.
 *   - app_ir_hw.c: the only code that touches hadc1 / hdma_adc1.
 *
 * Compile-time switches:
 *   APP_IR_ENABLE     default 1 (production). 1 = build the acquisition module,
 *                     create IRTask and answer IR,GET. 0 = nothing of this
 *                     module is compiled; no task, no heap, no ADC activity
 *                     (IR,GET is then answered ERR,BAD_CMD).
 *   APP_IR_BENCH_LOG  default 0. 1 = CommTask sends the bench-only "IRRAW,"
 *                     line (see PROTOCOL.md). It needs the acquisition, so
 *                     APP_IR_ENABLE=0 together with APP_IR_BENCH_LOG=1 is a
 *                     compile error.
 * Never commit APP_IR_BENCH_LOG as 1 (production emits no unsolicited IRRAW).
 */

#ifndef INC_APP_IR_H_
#define INC_APP_IR_H_

#include <stdint.h>
#include <stdbool.h>

#ifndef APP_IR_BENCH_LOG
#define APP_IR_BENCH_LOG 0
#endif

#ifndef APP_IR_ENABLE
#define APP_IR_ENABLE 1
#endif

#if (APP_IR_ENABLE != 0) && (APP_IR_ENABLE != 1)
#error "APP_IR_ENABLE must be defined as 0 or 1"
#endif

#if (APP_IR_BENCH_LOG != 0) && (APP_IR_BENCH_LOG != 1)
#error "APP_IR_BENCH_LOG must be defined as 0 or 1"
#endif

#if APP_IR_BENCH_LOG && !APP_IR_ENABLE
#error "APP_IR_BENCH_LOG=1 requires APP_IR_ENABLE=1"
#endif

#if APP_IR_ENABLE

#include "app_ir_dist.h"

/* ---------------------------------------------------------------------- *
 * Timing (RTOS ticks; configTICK_RATE_HZ = 1000, so 1 tick = 1 ms).
 * PROVISIONAL apart from the ADC facts; confirm on the bench.
 * ---------------------------------------------------------------------- */

/* IRTask wake period. A scan started on one wake is collected on the next,
 * so a sample is published about 10 ms after it was taken. */
#define APP_IR_SERVICE_PERIOD_TICKS  10U

/* Spacing between scan starts (about 50 scans per second). */
#define APP_IR_SAMPLE_PERIOD_TICKS   20U

/* A scan takes about 9.1 us (2 x (84 + 12) ADC cycles at 21 MHz), plus
 * 3 us of ADC stabilisation on the first start after a recovery. A scan
 * still incomplete this many ticks after it was started is lost. */
#define APP_IR_SCAN_TIMEOUT_TICKS    3U

/* A reading is stale when the latest publication is not VALID or is older
 * than this (5 sample periods). */
#define APP_IR_STALE_TICKS           100U

/* Phase 3: raw samples published as VALID during this many ticks after the
 * first scan start are not fed to the filter (the sensor's first
 * measurement is not guaranteed settled right after power-up). The filtered
 * output is available once the window is full, i.e. after
 * APP_IR_FILTER_WINDOW further samples (about 280 ms after the first start
 * for a window of 9). */
#define APP_IR_DIST_STARTUP_TICKS    100U

/* Largest 12-bit right-aligned conversion. 0 and 4095 are VALID raw values
 * (the acquisition applies no plausibility rule); the Phase 3 conversion
 * classifies a filtered 0 as FAR and 4095 as NEAR, never as a fault. */
#define APP_IR_ADC_MAX               4095U

/* Written to both DMA words before every scan. It cannot be a 12-bit
 * result, so a word the DMA did not write is detected. */
#define APP_IR_DMA_SENTINEL          0xFFFFU

/* Number of DMA elements per scan: [0] = LEFT (PC0, IN10, rank 1),
 * [1] = RIGHT (PC1, IN11, rank 2). */
#define APP_IR_DMA_COUNT             2U
#define APP_IR_DMA_INDEX_LEFT        0U
#define APP_IR_DMA_INDEX_RIGHT       1U

/* ---------------------------------------------------------------------- *
 * Types
 * ---------------------------------------------------------------------- */

typedef enum
{
    APP_IR_STATUS_NONE = 0,   /* nothing published yet                      */
    APP_IR_STATUS_VALID,      /* leftRaw/rightRaw are this scan's results   */
    APP_IR_STATUS_ERROR,      /* this scan failed; the driver retries       */
    APP_IR_STATUS_FAULT       /* latched: the ADC is never touched again    */
} AppIrStatus;

/* faultMask bits. TRANSIENT bits describe the publication they appear in and
 * the driver retries on the next sample period. LATCHED bits stay set in
 * every later publication and stop all ADC activity. */
#define APP_IR_FAULT_START      0x0001U /* transient: HAL start refused, or ADC not enabled */
#define APP_IR_FAULT_BUSY       0x0002U /* transient: ADC/DMA handle not ready before start */
#define APP_IR_FAULT_TIMEOUT    0x0004U /* transient: no completion within the timeout      */
#define APP_IR_FAULT_DMA        0x0008U /* transient: DMA error callback                    */
#define APP_IR_FAULT_ADC        0x0010U /* transient: ADC error callback / overrun flag     */
#define APP_IR_FAULT_DATA       0x0020U /* transient: DMA count not 0, or word > 4095       */
#define APP_IR_FAULT_STALE      0x0040U /* read time only (AppIr_ReportMask), never stored  */
#define APP_IR_FAULT_TASK       0x0080U /* latched: IRTask could not be created             */
#define APP_IR_FAULT_CFG_ADC    0x0100U /* latched: ADC1 register setup not as expected     */
#define APP_IR_FAULT_CFG_DMA    0x0200U /* latched: DMA2 Stream0 setup / linkage unexpected */
#define APP_IR_FAULT_CFG_NVIC   0x0400U /* latched: DMA2_Stream0 IRQ off or not maskable    */
#define APP_IR_FAULT_RECOVERY   0x0800U /* latched: ADC/DMA could not be returned to READY  */

#define APP_IR_FAULT_LATCHED_MASK \
    (APP_IR_FAULT_TASK | APP_IR_FAULT_CFG_ADC | APP_IR_FAULT_CFG_DMA | \
     APP_IR_FAULT_CFG_NVIC | APP_IR_FAULT_RECOVERY)

/* Counters saturate at UINT16_MAX. */
typedef struct
{
    uint16_t valid;
    uint16_t timeout;
    uint16_t startFail;
    uint16_t busy;
    uint16_t dmaError;
    uint16_t adcError;
    uint16_t dataError;
    uint16_t recoveries;    /* successful AppIrHw_Recover() calls             */
    uint16_t spurious;      /* ISR: callback while no scan was awaited        */
} AppIrCounters;

typedef struct
{
    uint32_t sequence;              /* 0 = never published; 0 skipped on wrap  */
    uint32_t publishTick;           /* RTOS tick of this publication           */
    uint32_t lastValidTick;         /* RTOS tick of the latest VALID one       */
    uint16_t leftRaw;               /* meaningful ONLY when status == VALID    */
    uint16_t rightRaw;              /* meaningful ONLY when status == VALID    */
    uint16_t faultMask;             /* APP_IR_FAULT_* (never STALE)            */
    uint8_t status;                 /* AppIrStatus                             */
    uint8_t consecutiveFailures;    /* ERROR publications in a row, sat. 255   */
    AppIrCounters counters;
    /* Phase 3 (filtered raw counts of THIS publication; zero unless VALID).
     * filteredRaw[] is meaningful only when filterSamples reaches
     * APP_IR_FILTER_WINDOW. Index 0 = LEFT, 1 = RIGHT. */
    uint16_t filteredRaw[2];
    uint8_t filterSamples;
} AppIrSnapshot;

/* Why a reading is or is not a usable measurement. At most one applies; the
 * first match in this order wins: FAULT, STALE, ERROR, WARMUP, OK. */
typedef enum
{
    APP_IR_ACQ_OK = 0,      /* fresh, filter full: per-sensor ranges apply     */
    APP_IR_ACQ_WARMUP,      /* fresh, but the filter has not filled yet        */
    APP_IR_ACQ_STALE,       /* nothing published, or newest publication > 100 ms old */
    APP_IR_ACQ_ERROR,       /* newest publication is a failed scan (transient) */
    APP_IR_ACQ_FAULT        /* latched acquisition fault: no measurement ever  */
} AppIrAcqState;

/* One consistent, calibrated reading (index 0 = LEFT, 1 = RIGHT). Distances
 * are millimetres FROM THE SENSOR FACE; the 45 mm face-to-tyre offset is NOT
 * applied. range[] and distanceMm[] are meaningful only when acq ==
 * APP_IR_ACQ_OK; otherwise range[] is APP_IR_RANGE_INVALID and the numbers
 * are 0. distanceMm[i] is meaningful only when range[i] == APP_IR_RANGE_OK. */
typedef struct
{
    uint32_t sequence;          /* publication it was derived from (0 = none)  */
    uint32_t ageTicks;          /* nowTick - publishTick; UINT32_MAX if none   */
    uint16_t faultMask;         /* AppIr_ReportMask(): FAULT bits + STALE 0x40 */
    uint8_t acq;                /* AppIrAcqState                               */
    uint8_t range[2];           /* AppIrRange                                  */
    uint16_t distanceMm[2];
    uint16_t filteredRaw[2];    /* diagnostic: filtered count, 0 unless OK     */
} AppIrReading;

/* ---------------------------------------------------------------------- *
 * Task API
 * ---------------------------------------------------------------------- */

/* IRTask, once, before the first Service(). Verifies (never changes) the
 * ADC/DMA/NVIC configuration; on a mismatch publishes a latched FAULT and
 * the ADC is never started. */
void AppIr_Init(void);

/* main(), when osThreadNew(StartIRTask) returned NULL. Pre-scheduler: no
 * critical section, plain stores. Publishes FAULT with APP_IR_FAULT_TASK;
 * touches no hardware. */
void AppIr_ReportTaskCreateFailure(void);

/* IRTask, every APP_IR_SERVICE_PERIOD_TICKS, with osKernelGetTickCount().
 * Never waits for the hardware. (Only the error-recovery path can spend up
 * to HAL's bounded 5 ms DMA-abort wait, in IRTask.) */
void AppIr_Service(uint32_t nowTick);

/* Atomic copy of the latest publication plus the ISR spurious counter. Any
 * task. */
void AppIr_GetSnapshot(AppIrSnapshot *snapshot);

/* Phase 3. Any task. Takes ONE atomic snapshot and derives the calibrated
 * reading from it at `nowTick` (osKernelGetTickCount()). Never blocks, never
 * touches hardware; the filtering itself runs in IRTask. */
void AppIr_GetReading(AppIrReading *reading, uint32_t nowTick);

/* True only for a VALID publication not older than APP_IR_STALE_TICKS.
 * Wrap-safe. The raw values are never consulted for this decision. */
static inline bool AppIr_IsFresh(const AppIrSnapshot *snapshot, uint32_t nowTick)
{
    if ((snapshot == 0) || (snapshot->status != (uint8_t)APP_IR_STATUS_VALID))
    {
        return false;
    }

    return (uint32_t)(nowTick - snapshot->publishTick) <= APP_IR_STALE_TICKS;
}

/* faultMask plus APP_IR_FAULT_STALE when the reading is not fresh. */
static inline uint16_t AppIr_ReportMask(const AppIrSnapshot *snapshot, uint32_t nowTick)
{
    uint16_t mask = (snapshot != 0) ? snapshot->faultMask : 0U;

    if (!AppIr_IsFresh(snapshot, nowTick))
    {
        mask = (uint16_t)(mask | APP_IR_FAULT_STALE);
    }

    return mask;
}

/* ---------------------------------------------------------------------- *
 * Interrupt entry points. Called ONLY from HAL_ADC_ConvCpltCallback /
 * HAL_ADC_ErrorCallback in main.c (inside DMA2_Stream0_IRQHandler, priority
 * 5, masked by taskENTER_CRITICAL()). Bounded, no RTOS or HAL calls.
 * ---------------------------------------------------------------------- */
void AppIr_OnConvComplete(void);
void AppIr_OnError(uint32_t halAdcErrorCode);

#ifdef APP_IR_TEST_HOOKS
/* Host tests only: preload the publication sequence (wrap test). */
void AppIr_TestSetSequence(uint32_t sequence);
#endif

#endif /* APP_IR_ENABLE */

#endif /* INC_APP_IR_H_ */
