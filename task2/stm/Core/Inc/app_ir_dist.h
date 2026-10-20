/*
 * app_ir_dist.h
 *
 * Phase 3 IR sensors: pure (no HAL, no RTOS) raw-ADC -> millimetre conversion
 * and the sample filter. Nothing here touches hardware or motion. The
 * acquisition lives in app_ir.c; this module only turns counts into numbers.
 *
 * Sensors: two Sharp GP2Y0A21YK analogue sensors, LEFT = PC0 / ADC1 channel
 * 10, RIGHT = PC1 / ADC1 channel 11. Both face SIDEWAYS (LEFT to the robot's
 * left, RIGHT to its right).
 *
 * REFERENCE (user-confirmed 2026-10-12): every calibration distance was
 * measured from the SENSOR FACE, along the beam. The conversion therefore
 * returns the distance from the sensor face. The face is recessed 45 mm
 * inboard of the corresponding tyre's outer lateral reference, along the beam;
 * that robot-geometry offset is NOT applied here. The consumer (Algo) computes
 *     clearance_mm = sensor_distance_mm - 45
 * (PROTOCOL.md section 2).
 *
 * CALIBRATION (provisional, in-sample, single run per distance; temperature,
 * supply voltage, target material and lighting were not recorded). The
 * curves are copied verbatim from the calibration reports:
 *   LEFT : d_cm = 31785.22973379 / (ADC + 26.48617844) - 0.29510522
 *   RIGHT: d_cm = 32831.45       / (ADC - 94.6824)     - 1.02464
 * They are NEVER interchangeable. Here they are scaled by 10 (millimetres).
 *
 * CALIBRATED DOMAIN: 10-40 cm (100-400 mm) from the sensor face, plus a
 * GUARD BAND of APP_IR_DIST_GUARD_MM (5 mm, the placement uncertainty of the
 * calibration) at each end. A target placed at exactly 10 or 40 cm produces
 * counts that scatter across the curve's own 100 / 400 mm points (the RIGHT 40 cm
 * calibration median, 893, lies at 401.0 mm on its fitted curve), so a strict
 * cutoff would call such a target NEAR/FAR half the time.
 *   - a filtered count whose curve value is within [95, 405] mm is converted
 *     and the result is CLAMPED to 100..400 mm;
 *     the fitted curves are evaluated only within this 95-405 mm band, and the
 *     clamped 95-100 / 400-405 mm results are not claimed as independently
 *     calibrated measurements outside 100-400 mm; nothing beyond the band is
 *     ever converted;
 *   - a filtered count ABOVE the guard band's upper ADC bound is NEAR (closer
 *     than about 95 mm, or saturated);
 *   - a filtered count BELOW its lower bound is FAR (farther than about 405 mm,
 *     or no target in range).
 * The cutoffs are per sensor because the two fitted curves differ. They are
 * integer comparisons, so 0 and 4095 are NEAR/FAR (never a fault and never a
 * distance). Below 10 cm the sensor output is not a
 * one-to-one function of distance: it peaks near 5-6 cm and then falls, so a
 * very close object can fold back INTO the calibrated ADC domain and be
 * reported as a valid distance. That region is a documented blind, untrusted
 * region; this module cannot detect it. It also cannot detect an unplugged
 * input, which may float to any in-domain value.
 */

#ifndef INC_APP_IR_DIST_H_
#define INC_APP_IR_DIST_H_

#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------- *
 * Filter (replaceable: only these two constants and AppIrFilter_Output())
 *
 * The filter is an order statistic over the last APP_IR_FILTER_WINDOW raw
 * counts of ONE sensor: the output is element APP_IR_FILTER_RANK (0-based,
 * ascending) of the sorted window. WINDOW 9 / RANK 4 is the MEDIAN of 9: at
 * the 50 Hz scan rate it spans 180 ms and moves a step change to the output
 * after 5 samples (100 ms) in both directions. RANK below the middle biases
 * toward smaller counts (farther objects): that direction is not
 * conservative for a clearance use, so it is not the default.
 * ---------------------------------------------------------------------- */
#ifndef APP_IR_FILTER_WINDOW
#define APP_IR_FILTER_WINDOW 9U
#endif

#ifndef APP_IR_FILTER_RANK
#define APP_IR_FILTER_RANK   4U
#endif

#if (APP_IR_FILTER_WINDOW < 1U) || (APP_IR_FILTER_WINDOW > 15U)
#error "APP_IR_FILTER_WINDOW must be 1..15"
#endif

#if APP_IR_FILTER_RANK >= APP_IR_FILTER_WINDOW
#error "APP_IR_FILTER_RANK must be below APP_IR_FILTER_WINDOW"
#endif

/* ---------------------------------------------------------------------- *
 * Calibrated domain
 * ---------------------------------------------------------------------- */

/* Reported distance limits (mm from the sensor face). */
#define APP_IR_DIST_MIN_MM      100U
#define APP_IR_DIST_MAX_MM      400U

/* Half-width of the guard band around 100..400 mm: the +-5 mm ruler/target
 * placement uncertainty of the calibration. Results inside the band are
 * clamped to 100 / 400 mm; outside it the sensor is NEAR / FAR. */
#define APP_IR_DIST_GUARD_MM    5U

/* Integer ADC cutoffs of the guard band [95, 405] mm, derived from each curve
 * (lowest / highest count whose curve value lies in the band; verified
 * exhaustively against a double-precision evaluation by the host test). A
 * count below FAR_LIMIT is FAR, above NEAR_LIMIT is NEAR, otherwise it is
 * converted and clamped to 100..400 mm.
 *               strict 100..400 mm        guard band 95..405 mm (used)
 * LEFT : counts 763 .. 3060            753 (404.82 mm) .. 3218 (95.02 mm)
 * RIGHT: counts 895 .. 3072            886 (404.65 mm) .. 3214 (95.01 mm)
 * (for example the RIGHT 40 cm median 893 = 401.0 mm is converted and
 * reported as 400.) */
#define APP_IR_LEFT_ADC_FAR_LIMIT    753U    /* below: FAR   */
#define APP_IR_LEFT_ADC_NEAR_LIMIT   3218U   /* above: NEAR  */
#define APP_IR_RIGHT_ADC_FAR_LIMIT   886U
#define APP_IR_RIGHT_ADC_NEAR_LIMIT  3214U

typedef enum
{
    APP_IR_SENSOR_LEFT = 0,
    APP_IR_SENSOR_RIGHT = 1
} AppIrSensor;

typedef enum
{
    APP_IR_RANGE_OK = 0,    /* *mm is a distance in 100..400 (clamped guard band) */
    APP_IR_RANGE_NEAR,      /* above the upper ADC cutoff: < ~95 mm or saturated  */
    APP_IR_RANGE_FAR,       /* below the lower ADC cutoff: > ~405 mm or no target */
    APP_IR_RANGE_INVALID    /* no meaningful value (bad input, or not available) */
} AppIrRange;

/* Converts ONE filtered ADC count of `sensor` to millimetres from the sensor
 * face. *mm is written only for APP_IR_RANGE_OK and set to 0 otherwise. An
 * unknown sensor or a count above 4095 gives APP_IR_RANGE_INVALID. Single
 * precision float (the F407 has a hardware FPU); the range decision is made
 * on the integer count before any arithmetic. */
AppIrRange AppIrDist_Convert(AppIrSensor sensor, uint16_t filteredAdc, uint16_t *mm);

/* ---------------------------------------------------------------------- *
 * Sample filter (one instance per sensor; owned by a single task)
 * ---------------------------------------------------------------------- */
typedef struct
{
    uint16_t samples[APP_IR_FILTER_WINDOW];
    uint8_t next;       /* ring write index                     */
    uint8_t count;      /* valid samples, saturates at WINDOW   */
} AppIrFilter;

void AppIrFilter_Reset(AppIrFilter *filter);
void AppIrFilter_Push(AppIrFilter *filter, uint16_t rawAdc);

/* True once WINDOW samples have been pushed since the last reset. */
bool AppIrFilter_IsFull(const AppIrFilter *filter);

/* Writes the order statistic of the window into *out and returns true, or
 * returns false (and leaves *out alone) while the window is not full. */
bool AppIrFilter_Output(const AppIrFilter *filter, uint16_t *out);

#endif /* INC_APP_IR_DIST_H_ */
