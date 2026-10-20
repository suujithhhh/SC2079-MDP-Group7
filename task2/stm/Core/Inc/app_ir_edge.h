/*
 * app_ir_edge.h
 *
 * Phase 5: low-latency obstacle-edge detector for ONE side IR sensor. Pure
 * policy: no ADC, no HAL, no RTOS, no conversion of its own. It consumes the
 * RAW per-scan publications of app_ir (AppIrSnapshot) and is independent of
 * the Phase 3 nine-sample median and of IR,GET, which are unchanged. Compiled
 * only when APP_IR_ENABLE is 1.
 *
 * Why raw scans and not the Phase 3 reading: the median of 9 needs about
 * 100 ms to move a step to its output, and the edge pass must also confirm in
 * well under that. The detector classifies every newly published raw scan on
 * its own and needs APP_IR_EDGE_SEEN_SCANS / APP_IR_EDGE_CLEAR_SCANS
 * consecutive scans of one class before it decides.
 *
 * Why four consecutive scans (the default) and not a 3-median or a 2-of-3
 * vote: the Sharp GP2Y0A21YK measures only once per 38.3 +-9.6 ms
 * (datasheet, user-verified; up to 5 ms output delay), so the ADC (one scan
 * per 20 ms) sees the same sensor measurement in up to 3 consecutive scans.
 * Fewer than (N - 1) * 20 ms > 47.9 ms of agreeing scans can be satisfied by
 * ONE held measurement. Four scans span 60 ms and therefore always contain
 * two independent measurements. APP_IR_EDGE_*_SCANS below 4 do not compile.
 *
 * Raw thresholds are not typed in: AppIrEdge_Start() derives, per sensor,
 * the lowest count that AppIrDist_Convert() reports as NEAR or as at most
 * APP_IR_EDGE_VISIBLE_MAX_MM, and the highest count it reports as FAR or as
 * at least APP_IR_EDGE_CLEAR_MIN_MM, by bisection over the real conversion
 * (monotone in the count; the host test checks all 4096 counts of both
 * sensors). So the thresholds can never disagree with IR,GET.
 *
 * Scan classes (one per publication):
 *   VISIBLE    count at or above the visible threshold (NEAR included: an
 *              object is there, its distance is simply not trusted)
 *   CLEAR      count at or below the clear threshold (FAR included)
 *   AMBIGUOUS  in between (the hysteresis band)
 *   UNUSABLE   publication is not VALID (ERROR) or is older than
 *              APP_IR_STALE_TICKS; NEVER counts as clear
 * A FAULT publication fails the detector at once.
 *
 * Each publication is consumed once, by its sequence number (never 0). A
 * sequence that is not the successor of the previous one (a missed scan)
 * resets both streaks. Publications made before AppIrEdge_Start() are
 * ignored.
 */

#ifndef INC_APP_IR_EDGE_H_
#define INC_APP_IR_EDGE_H_

#include "app_ir.h"

#if APP_IR_ENABLE

#include <stdint.h>
#include <stdbool.h>

/* mm from the sensor face (same reference as IR,GET). User-accepted trusted
 * range is 100-200 mm; beyond 200 mm the reading is only "far enough". */
#ifndef APP_IR_EDGE_VISIBLE_MAX_MM
#define APP_IR_EDGE_VISIBLE_MAX_MM    200U
#endif

#ifndef APP_IR_EDGE_CLEAR_MIN_MM
#define APP_IR_EDGE_CLEAR_MIN_MM      250U
#endif

#if (APP_IR_EDGE_VISIBLE_MAX_MM < APP_IR_DIST_MIN_MM) || \
    (APP_IR_EDGE_CLEAR_MIN_MM > APP_IR_DIST_MAX_MM) || \
    (APP_IR_EDGE_VISIBLE_MAX_MM >= APP_IR_EDGE_CLEAR_MIN_MM)
#error "APP_IR_EDGE thresholds need 100 <= VISIBLE_MAX < CLEAR_MIN <= 400 mm"
#endif

/* Consecutive newly published scans of one class needed to decide. */
#ifndef APP_IR_EDGE_SEEN_SCANS
#define APP_IR_EDGE_SEEN_SCANS        4U
#endif

#ifndef APP_IR_EDGE_CLEAR_SCANS
#define APP_IR_EDGE_CLEAR_SCANS       4U
#endif

/* Longest GP2Y0A21YK measurement cycle: 38.3 + 9.6 = 47.9 ms (datasheet,
 * user-verified), rounded up. */
#define APP_IR_EDGE_SENSOR_CYCLE_MAX_MS  48U

/* The agreeing scans must span more than one sensor cycle, so that two
 * independent sensor measurements are always involved. */
#if ((APP_IR_EDGE_SEEN_SCANS - 1U) * APP_IR_SAMPLE_PERIOD_TICKS) <= \
    APP_IR_EDGE_SENSOR_CYCLE_MAX_MS
#error "APP_IR_EDGE_SEEN_SCANS too small: one held sensor measurement could confirm"
#endif

#if ((APP_IR_EDGE_CLEAR_SCANS - 1U) * APP_IR_SAMPLE_PERIOD_TICKS) <= \
    APP_IR_EDGE_SENSOR_CYCLE_MAX_MS
#error "APP_IR_EDGE_CLEAR_SCANS too small: one held sensor measurement could confirm"
#endif

/* Readiness before the robot may drive: this many consecutive newly
 * published VALID scans, within READY_TIMEOUT_TICKS of the start. */
#ifndef APP_IR_EDGE_READY_SCANS
#define APP_IR_EDGE_READY_SCANS       2U
#endif

#ifndef APP_IR_EDGE_READY_TIMEOUT_TICKS
#define APP_IR_EDGE_READY_TIMEOUT_TICKS      300U
#endif

/* While searching: no usable (VALID, fresh) publication for longer than this
 * is a sensor failure. */
#ifndef APP_IR_EDGE_UNAVAILABLE_LIMIT_TICKS
#define APP_IR_EDGE_UNAVAILABLE_LIMIT_TICKS  200U
#endif

#if (APP_IR_EDGE_READY_SCANS < 1U) || (APP_IR_EDGE_READY_SCANS > 15U)
#error "APP_IR_EDGE_READY_SCANS must be 1..15"
#endif

#if (APP_IR_EDGE_SEEN_SCANS > 15U) || (APP_IR_EDGE_CLEAR_SCANS > 15U)
#error "APP_IR_EDGE_*_SCANS must be at most 15"
#endif

/* MOV,FIRL / MOV,FIRR (app_motion.c) only. Speed cap: the SPCTRL cruise target
 * is limited to this while a FIR move runs (approach and return keep the
 * profile's own speed). 3000 counts/s = 40 cm/s at 75 counts/cm is the speed at
 * which a 10 cm obstacle face (the smallest in Task 2) is always confirmed by
 * four scans (see tests/ir/app_ir_edge_sweep_test.c). Do not raise it before
 * the bench validation gates in PROTOCOL.md pass: 4500 at most for 10 cm
 * faces; 6000 / 8000 only if FIR is used exclusively along faces of at least
 * 30 cm. */
#ifndef APP_IR_EDGE_SPEED_CAP_COUNTS_PER_SEC
#define APP_IR_EDGE_SPEED_CAP_COUNTS_PER_SEC  3000U
#endif

/* Travel after the confirmed edge: the target is pulled in to this many cm
 * ahead of the position at confirmation (0 = stop at once through the
 * existing crossing / brake path). Tune on the bench. */
#ifndef APP_IR_EDGE_POST_CM
#define APP_IR_EDGE_POST_CM                   0U
#endif

typedef enum
{
    APP_IR_EDGE_SIDE_LEFT = 0,      /* raw index 0, PC0 */
    APP_IR_EDGE_SIDE_RIGHT = 1      /* raw index 1, PC1 */
} AppIrEdgeSide;

typedef enum
{
    APP_IR_EDGE_PHASE_READY = 0,    /* waiting for READY_SCANS valid scans   */
    APP_IR_EDGE_PHASE_WAIT_OBSTACLE,/* ready; waiting to confirm "seen"      */
    APP_IR_EDGE_PHASE_WAIT_CLEAR,   /* seen; waiting to confirm "clear"      */
    APP_IR_EDGE_PHASE_CLEARED,      /* terminal: edge confirmed              */
    APP_IR_EDGE_PHASE_FAILED        /* terminal: see failReason              */
} AppIrEdgePhase;

typedef enum
{
    APP_IR_EDGE_OUT_NONE = 0,
    APP_IR_EDGE_OUT_READY,          /* once: the sensor is ready, may drive  */
    APP_IR_EDGE_OUT_SEEN,           /* once: obstacle confirmed visible      */
    APP_IR_EDGE_OUT_CLEARED,        /* once: obstacle confirmed gone         */
    APP_IR_EDGE_OUT_FAILED          /* once: see failReason                  */
} AppIrEdgeOutcome;

typedef enum
{
    APP_IR_EDGE_FAIL_NONE = 0,
    APP_IR_EDGE_FAIL_FAULT,         /* latched acquisition fault             */
    APP_IR_EDGE_FAIL_READY_TIMEOUT, /* never ready within the timeout        */
    APP_IR_EDGE_FAIL_DATA_LOST,     /* no usable scan for UNAVAILABLE_LIMIT  */
    APP_IR_EDGE_FAIL_CONFIG         /* internal: thresholds could not be derived */
} AppIrEdgeFailReason;

typedef enum
{
    APP_IR_EDGE_CLASS_VISIBLE = 0,
    APP_IR_EDGE_CLASS_CLEAR,
    APP_IR_EDGE_CLASS_AMBIGUOUS
} AppIrEdgeClass;

typedef struct
{
    uint16_t visibleMinAdc;     /* count >= this: VISIBLE (derived at Start) */
    uint16_t clearMaxAdc;       /* count <= this: CLEAR   (derived at Start) */
    uint32_t startTick;
    uint32_t lastUsableTick;    /* tick a usable scan was last consumed      */
    uint32_t lastSequence;
    uint16_t seenRaw;           /* raw count of the scan that confirmed SEEN */
    uint16_t clearRaw;          /* ... and CLEARED                           */
    uint16_t missedScans;       /* sequence gaps (saturating)                */
    uint16_t unusableScans;     /* ERROR / stale publications (saturating)   */
    uint16_t ambiguousScans;    /* (saturating)                              */
    uint8_t side;               /* AppIrEdgeSide                             */
    uint8_t phase;              /* AppIrEdgePhase                            */
    uint8_t failReason;         /* AppIrEdgeFailReason                       */
    uint8_t haveSequence;
    uint8_t firstScanDone;
    uint8_t readyCount;
    uint8_t streak;             /* consecutive scans of the wanted class     */
    uint8_t startedVisible;     /* first usable scan was VISIBLE             */
} AppIrEdgeDetector;

/* Prepares a detector for `side` at `nowTick` and derives its thresholds.
 * Returns false (detector left FAILED/CONFIG) for an unknown side or if the
 * thresholds cannot be derived. */
bool AppIrEdge_Start(AppIrEdgeDetector *detector, AppIrEdgeSide side, uint32_t nowTick);

/* Consumes the snapshot's publication if it is new (see above) and applies
 * the timeouts. Call as often as convenient; a publication is processed
 * once. Each outcome other than NONE is returned exactly once; after
 * CLEARED or FAILED every call returns NONE. */
AppIrEdgeOutcome AppIrEdge_Step(AppIrEdgeDetector *detector,
                                const AppIrSnapshot *snapshot,
                                uint32_t nowTick);

/* Classification of one raw count with the detector's derived thresholds. */
AppIrEdgeClass AppIrEdge_ClassifyRaw(const AppIrEdgeDetector *detector, uint16_t raw);

#endif /* APP_IR_ENABLE */

#endif /* INC_APP_IR_EDGE_H_ */
