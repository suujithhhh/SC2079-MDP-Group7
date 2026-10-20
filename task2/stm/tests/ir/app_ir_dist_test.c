/* Host test of the Phase 3 IR conversion and filter (Core/Src/app_ir_dist.c):
 *   - every ADC count 0..4095, for both sensors, against an independent
 *     double-precision evaluation of the two calibration-report curves
 *     (coefficients are typed in again below, so a copy/paste or LEFT/RIGHT
 *     mix-up in the firmware cannot also be in the reference);
 *   - the integer guard-band cutoffs ([95, 405] mm, clamped to 100..400 mm),
 *     derived from those curves, and the counts immediately inside, on and
 *     outside them for each sensor;
 *   - the in-sample calibration medians from the two reports;
 *   - 0 / 4095 / out-of-range / invalid inputs;
 *   - the order-statistic filter: warm-up, ring overwrite, outlier
 *     rejection, reset, and the configured window / rank.
 *
 * Build (from repo root), default window 9 / rank 4 (median of 9):
 *   gcc -std=c11 -Wall -Wextra -Werror -DAPP_IR_ENABLE=1 -ICore/Inc \
 *       -o tests/ir/app_ir_dist_test.exe tests/ir/app_ir_dist_test.c \
 *       Core/Src/app_ir_dist.c
 *   ./tests/ir/app_ir_dist_test.exe
 * The filter is replaceable by constants; the test follows them, so these
 * must pass too:
 *   ... -DAPP_IR_FILTER_WINDOW=5U -DAPP_IR_FILTER_RANK=2U
 *   ... -DAPP_IR_FILTER_WINDOW=9U -DAPP_IR_FILTER_RANK=2U   (3rd-lowest of 9)
 *   ... -DAPP_IR_FILTER_WINDOW=1U -DAPP_IR_FILTER_RANK=0U   (no filtering)
 */

#include "app_ir.h"
#include "app_ir_dist.h"

#include <stdio.h>
#include <stdlib.h>
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

/* Independent reference (centimetres), straight from the calibration reports:
 *   LEFT : distance_cm = 31785.22973379 / (ADC_L + 26.48617844) - 0.29510522
 *   RIGHT: distance_cm = 32831.45       / (ADC_R - 94.6824)     - 1.02464   */
static double RefCm(int sensor, int adc)
{
    if (sensor == 0)
    {
        return 31785.22973379 / ((double)adc + 26.48617844) - 0.29510522;
    }

    return 32831.45 / ((double)adc - 94.6824) - 1.02464;
}

/* Counts whose curve value lies in [lo, hi] millimetres, on the decreasing
 * branch of the curve (denominator > 0). */
static int RefInside(int sensor, int adc, double loMm, double hiMm)
{
    double mm;

    if (sensor == 1 && (double)adc <= 94.6824)
    {
        return 0;
    }

    mm = RefCm(sensor, adc) * 10.0;
    return (mm >= loMm) && (mm <= hiMm);
}

static void RefBounds(int sensor, double loMm, double hiMm, int *lo, int *hi, int *count)
{
    int adc;

    *lo = 99999;
    *hi = -1;
    *count = 0;
    for (adc = 0; adc <= 4095; adc++)
    {
        if (RefInside(sensor, adc, loMm, hiMm))
        {
            if (adc < *lo) { *lo = adc; }
            if (adc > *hi) { *hi = adc; }
            (*count)++;
        }
    }
}

static int Cmp16(const void *a, const void *b)
{
    unsigned x = *(const uint16_t *)a;
    unsigned y = *(const uint16_t *)b;

    return (x > y) - (x < y);
}

static void test_exhaustive_conversion(void)
{
    int sensor;
    int adc;
    unsigned ties = 0U;
    unsigned clampedLow = 0U;
    unsigned clampedHigh = 0U;
    const int farLimit[2] = { APP_IR_LEFT_ADC_FAR_LIMIT, APP_IR_RIGHT_ADC_FAR_LIMIT };
    const int nearLimit[2] = { APP_IR_LEFT_ADC_NEAR_LIMIT, APP_IR_RIGHT_ADC_NEAR_LIMIT };
    const double g = (double)APP_IR_DIST_GUARD_MM;

    CHECK(APP_IR_DIST_GUARD_MM == 5U);

    for (sensor = 0; sensor < 2; sensor++)
    {
        int gLo;
        int gHi;
        int gCount;
        int sLo;
        int sHi;
        int sCount;
        unsigned previous = 400U;
        int havePrevious = 0;

        /* Guard band [95, 405] mm: contiguous, and exactly the firmware's
         * cutoffs. The strict [100, 400] mm span lies inside it. */
        RefBounds(sensor, (double)APP_IR_DIST_MIN_MM - g, (double)APP_IR_DIST_MAX_MM + g, &gLo, &gHi, &gCount);
        RefBounds(sensor, (double)APP_IR_DIST_MIN_MM, (double)APP_IR_DIST_MAX_MM, &sLo, &sHi, &sCount);
        CHECK(gCount == (gHi - gLo + 1));
        CHECK(sCount == (sHi - sLo + 1));
        CHECK(gLo == farLimit[sensor]);
        CHECK(gHi == nearLimit[sensor]);
        CHECK(gLo < sLo);
        CHECK(gHi > sHi);

        for (adc = 0; adc <= 4095; adc++)
        {
            uint16_t mm = 0xBEEFU;
            AppIrRange range = AppIrDist_Convert((AppIrSensor)sensor, (uint16_t)adc, &mm);

            if (adc < gLo)
            {
                CHECK(range == APP_IR_RANGE_FAR);
                CHECK(mm == 0U);
            }
            else if (adc > gHi)
            {
                CHECK(range == APP_IR_RANGE_NEAR);
                CHECK(mm == 0U);
            }
            else
            {
                double refMm = RefCm(sensor, adc) * 10.0;
                double expect = (double)(unsigned)(refMm + 0.5);
                double frac = refMm - (double)(unsigned)refMm;
                int tie = (frac > 0.498) && (frac < 0.502);

                /* The clamp: guard-band counts outside the strict span give
                 * exactly 100 / 400, never a value beyond them. */
                if (expect < (double)APP_IR_DIST_MIN_MM) { expect = (double)APP_IR_DIST_MIN_MM; }
                if (expect > (double)APP_IR_DIST_MAX_MM) { expect = (double)APP_IR_DIST_MAX_MM; }

                CHECK(range == APP_IR_RANGE_OK);
                CHECK(mm >= APP_IR_DIST_MIN_MM);
                CHECK(mm <= APP_IR_DIST_MAX_MM);

                if (adc < sLo) { CHECK(mm == APP_IR_DIST_MAX_MM); clampedHigh++; }
                if (adc > sHi) { CHECK(mm == APP_IR_DIST_MIN_MM); clampedLow++; }

                if (tie && (adc >= sLo) && (adc <= sHi))
                {
                    ties++;
                    CHECK(((double)mm == expect) || ((double)mm == expect - 1.0));
                }
                else
                {
                    CHECK((double)mm == expect);
                }

                /* Farther away means a smaller count means a larger distance
                 * (non-increasing in the count). */
                if (havePrevious)
                {
                    CHECK(mm <= previous);
                }
                previous = mm;
                havePrevious = 1;
            }
        }
    }

    printf("exhaustive conversion: guard-band cutoffs LEFT %d..%d, RIGHT %d..%d; "
           "%u counts clamped to 400 mm, %u to 100 mm; %u within 0.002 mm of a rounding tie\n",
           farLimit[0], nearLimit[0], farLimit[1], nearLimit[1], clampedHigh, clampedLow, ties);
}

/* Immediately outside, on and inside the guard band, and on the strict
 * 100 / 400 mm points, for each sensor on its own cutoffs. */
static void test_guard_band_boundaries(void)
{
    static const struct
    {
        AppIrSensor sensor;
        uint16_t strictFar;     /* lowest count with curve >= ... 400 mm (strict) */
        uint16_t strictNear;    /* highest count with curve >= 100 mm (strict)    */
        uint16_t guardFar;
        uint16_t guardNear;
    } t[2] = {
        { APP_IR_SENSOR_LEFT, 763U, 3060U, APP_IR_LEFT_ADC_FAR_LIMIT, APP_IR_LEFT_ADC_NEAR_LIMIT },
        { APP_IR_SENSOR_RIGHT, 895U, 3072U, APP_IR_RIGHT_ADC_FAR_LIMIT, APP_IR_RIGHT_ADC_NEAR_LIMIT }
    };
    unsigned i;
    uint16_t mm;

    for (i = 0U; i < 2U; i++)
    {
        AppIrSensor s = t[i].sensor;

        /* far end: outside / on / inside the guard band, then the strict point */
        mm = 0xBEEFU;
        CHECK(AppIrDist_Convert(s, (uint16_t)(t[i].guardFar - 1U), &mm) == APP_IR_RANGE_FAR);
        CHECK(mm == 0U);
        CHECK(AppIrDist_Convert(s, t[i].guardFar, &mm) == APP_IR_RANGE_OK);
        CHECK(mm == APP_IR_DIST_MAX_MM);                /* on the cutoff: clamped */
        CHECK(AppIrDist_Convert(s, (uint16_t)(t[i].guardFar + 1U), &mm) == APP_IR_RANGE_OK);
        CHECK(mm == APP_IR_DIST_MAX_MM);                /* just inside: clamped */
        CHECK(AppIrDist_Convert(s, (uint16_t)(t[i].strictFar - 1U), &mm) == APP_IR_RANGE_OK);
        CHECK(mm == APP_IR_DIST_MAX_MM);                /* just outside the strict span */
        CHECK(AppIrDist_Convert(s, t[i].strictFar, &mm) == APP_IR_RANGE_OK);
        CHECK(mm >= 399U && mm <= APP_IR_DIST_MAX_MM);  /* on the strict point: unclamped */

        /* near end */
        CHECK(AppIrDist_Convert(s, t[i].strictNear, &mm) == APP_IR_RANGE_OK);
        CHECK(mm >= APP_IR_DIST_MIN_MM && mm <= 101U);
        CHECK(AppIrDist_Convert(s, (uint16_t)(t[i].strictNear + 1U), &mm) == APP_IR_RANGE_OK);
        CHECK(mm == APP_IR_DIST_MIN_MM);
        CHECK(AppIrDist_Convert(s, (uint16_t)(t[i].guardNear - 1U), &mm) == APP_IR_RANGE_OK);
        CHECK(mm == APP_IR_DIST_MIN_MM);
        CHECK(AppIrDist_Convert(s, t[i].guardNear, &mm) == APP_IR_RANGE_OK);
        CHECK(mm == APP_IR_DIST_MIN_MM);                /* on the cutoff: clamped */
        mm = 0xBEEFU;
        CHECK(AppIrDist_Convert(s, (uint16_t)(t[i].guardNear + 1U), &mm) == APP_IR_RANGE_NEAR);
        CHECK(mm == 0U);                                /* outside: no number */
    }

    /* The motivating case: the RIGHT 40 cm calibration median (893) is
     * 401.0 mm on its curve; it is reported as 400, not FAR. */
    CHECK(AppIrDist_Convert(APP_IR_SENSOR_RIGHT, 893U, &mm) == APP_IR_RANGE_OK);
    CHECK(mm == 400U);
    /* ...and the LEFT/RIGHT cutoffs differ, so the policy is per sensor. */
    CHECK(APP_IR_LEFT_ADC_FAR_LIMIT != APP_IR_RIGHT_ADC_FAR_LIMIT);
    CHECK(APP_IR_LEFT_ADC_NEAR_LIMIT != APP_IR_RIGHT_ADC_NEAR_LIMIT);
    CHECK(AppIrDist_Convert(APP_IR_SENSOR_LEFT, 889U, &mm) == APP_IR_RANGE_OK);
    CHECK(AppIrDist_Convert(APP_IR_SENSOR_RIGHT, 885U, &mm) == APP_IR_RANGE_FAR);
    CHECK(AppIrDist_Convert(APP_IR_SENSOR_LEFT, 3216U, &mm) == APP_IR_RANGE_OK);
    CHECK(AppIrDist_Convert(APP_IR_SENSOR_RIGHT, 3215U, &mm) == APP_IR_RANGE_NEAR);
    CHECK(AppIrDist_Convert(APP_IR_SENSOR_LEFT, 3215U, &mm) == APP_IR_RANGE_OK);
}

static void test_saturation_and_invalid_input(void)
{
    uint16_t mm;
    int sensor;

    for (sensor = 0; sensor < 2; sensor++)
    {
        /* 0 and 4095 are classified, never a fault and never a distance. */
        mm = 0xBEEFU;
        CHECK(AppIrDist_Convert((AppIrSensor)sensor, 0U, &mm) == APP_IR_RANGE_FAR);
        CHECK(mm == 0U);
        mm = 0xBEEFU;
        CHECK(AppIrDist_Convert((AppIrSensor)sensor, 4095U, &mm) == APP_IR_RANGE_NEAR);
        CHECK(mm == 0U);

        /* Not a 12-bit value: invalid, never converted. */
        mm = 0xBEEFU;
        CHECK(AppIrDist_Convert((AppIrSensor)sensor, 4096U, &mm) == APP_IR_RANGE_INVALID);
        CHECK(mm == 0U);
        mm = 0xBEEFU;
        CHECK(AppIrDist_Convert((AppIrSensor)sensor, 0xFFFFU, &mm) == APP_IR_RANGE_INVALID);
        CHECK(mm == 0U);
        mm = 0xBEEFU;
        CHECK(AppIrDist_Convert((AppIrSensor)sensor, 0xFFFFU, &mm) != APP_IR_RANGE_OK);
    }

    mm = 0xBEEFU;
    CHECK(AppIrDist_Convert((AppIrSensor)2, 2000U, &mm) == APP_IR_RANGE_INVALID);
    CHECK(mm == 0U);
    mm = 0xBEEFU;
    CHECK(AppIrDist_Convert((AppIrSensor)255, 2000U, &mm) == APP_IR_RANGE_INVALID);
    CHECK(mm == 0U);
    CHECK(AppIrDist_Convert(APP_IR_SENSOR_LEFT, 2000U, 0) == APP_IR_RANGE_INVALID);
}

/* The two sensors have different curves: one count gives different
 * distances, and the firmware must pick each sensor's own. */
static void test_left_right_coefficients_not_interchanged(void)
{
    uint16_t l = 0U;
    uint16_t r = 0U;

    CHECK(AppIrDist_Convert(APP_IR_SENSOR_LEFT, 1637U, &l) == APP_IR_RANGE_OK);
    CHECK(AppIrDist_Convert(APP_IR_SENSOR_RIGHT, 1637U, &r) == APP_IR_RANGE_OK);
    CHECK(l == 188U);       /* 31785.22973379 / 1663.48617844 - 0.29510522 = 18.81 cm */
    CHECK(r == 203U);       /* 32831.45 / 1542.3176 - 1.02464              = 20.26 cm */
    CHECK(l != r);
}

/* In-sample calibration medians from the reports (integer-valued medians
 * only). expected = what the report's curve gives; the firmware must agree to
 * within the 0.5 mm rounding. These tie the firmware to the report numbers,
 * not to this file's own formula. */
static void test_report_medians(void)
{
    /* LEFT: ir_left_calibration_analysis.csv, median_L_adc and
     * model_median_error_mm  (model mm = distance_cm * 10 + error). */
    static const struct { uint16_t adc; double distanceMm; double errorMm; } left[] = {
        { 3026U, 100.0,  1.1779360578651321 },
        { 2365U, 130.0, -0.04110679573855691 },
        { 2211U, 140.0, -0.8932819802455327 },
        { 2049U, 150.0,  0.19489533917333546 },
        { 1720U, 180.0, -0.9557399466399374 },
        { 1556U, 200.0, -2.0947656069893483 },
        { 1226U, 250.0,  0.8260375972838574 },
        { 1010U, 300.0,  3.7122723870700014 },
        {  867U, 350.0,  2.7928921767606596 },
        {  776U, 400.0, -6.866601276120292 }
    };
    /* RIGHT: ir_right_calibration_summary.csv, median and predicted_cm. */
    static const struct { uint16_t adc; double predictedCm; } right[] = {
        { 3063U, 10.035989426961505 },
        { 2847U, 10.904019817259002 },
        { 2284U, 13.971566451056262 },
        { 2146U, 14.980418562494808 },
        { 2010U, 16.116881556663692 },
        { 1911U, 17.05119522077899 },
        { 1823U, 17.971554563453182 },
        { 1728U, 19.07644574718662 },
        { 1370U, 24.71910896829466 },
        { 1152U, 30.02700955588032 }
    };
    unsigned i;
    uint16_t mm;

    for (i = 0U; i < sizeof(left) / sizeof(left[0]); i++)
    {
        double expected = left[i].distanceMm + left[i].errorMm;
        double diff;

        CHECK(AppIrDist_Convert(APP_IR_SENSOR_LEFT, left[i].adc, &mm) == APP_IR_RANGE_OK);
        diff = (double)mm - expected;
        CHECK(diff <= 0.5001 && diff >= -0.5001);
    }

    for (i = 0U; i < sizeof(right) / sizeof(right[0]); i++)
    {
        double expected = right[i].predictedCm * 10.0;
        double diff;

        CHECK(AppIrDist_Convert(APP_IR_SENSOR_RIGHT, right[i].adc, &mm) == APP_IR_RANGE_OK);
        diff = (double)mm - expected;
        CHECK(diff <= 0.5001 && diff >= -0.5001);
    }

    /* The RIGHT 40 cm median (893) is 401.0 mm on its curve: inside the guard
     * band, reported as 400. */
    CHECK(AppIrDist_Convert(APP_IR_SENSOR_RIGHT, 893U, &mm) == APP_IR_RANGE_OK);
    CHECK(mm == 400U);
}

static void test_filter(void)
{
    AppIrFilter f;
    uint16_t out = 0xBEEFU;
    uint16_t sorted[APP_IR_FILTER_WINDOW];
    unsigned i;
    unsigned seed = 12345U;

    /* Window must be full before there is an output. */
    AppIrFilter_Reset(&f);
    CHECK(!AppIrFilter_IsFull(&f));
    CHECK(!AppIrFilter_Output(&f, &out));
    CHECK(out == 0xBEEFU);

    for (i = 0U; i + 1U < APP_IR_FILTER_WINDOW; i++)
    {
        AppIrFilter_Push(&f, (uint16_t)(1000U + i));
        CHECK(!AppIrFilter_IsFull(&f));
        CHECK(!AppIrFilter_Output(&f, &out));
    }

    AppIrFilter_Push(&f, 1000U + APP_IR_FILTER_WINDOW);
    CHECK(AppIrFilter_IsFull(&f));
    CHECK(AppIrFilter_Output(&f, &out));

    /* Reset empties it. */
    AppIrFilter_Reset(&f);
    CHECK(!AppIrFilter_IsFull(&f));
    CHECK(f.count == 0U);
    CHECK(!AppIrFilter_Output(&f, &out));

    /* Randomised: the output is the configured order statistic of the LAST
     * `window` samples (ring overwrite), including 0 and 4095, and duplicates. */
    for (i = 0U; i < 400U; i++)
    {
        uint16_t history[APP_IR_FILTER_WINDOW];
        unsigned k;

        AppIrFilter_Reset(&f);
        for (k = 0U; k < APP_IR_FILTER_WINDOW + (i % 7U) * 3U + (i % 3U); k++)
        {
            uint16_t value;

            seed = seed * 1103515245U + 12345U;
            value = (uint16_t)(((seed >> 8) % 4U == 0U)
                                   ? (((seed >> 12) & 1U) ? 4095U : 0U)
                                   : ((seed >> 10) % 4096U));
            if (((seed >> 20) & 3U) == 0U) { value = (uint16_t)(seed >> 22) & 0x3U; } /* duplicates */

            AppIrFilter_Push(&f, value);
            memmove(&history[0], &history[1], sizeof(history) - sizeof(history[0]));
            history[APP_IR_FILTER_WINDOW - 1U] = value;
        }

        memcpy(sorted, history, sizeof(sorted));
        qsort(sorted, APP_IR_FILTER_WINDOW, sizeof(sorted[0]), Cmp16);
        CHECK(AppIrFilter_Output(&f, &out));
        CHECK(out == sorted[APP_IR_FILTER_RANK]);
    }

    /* Null safety. */
    AppIrFilter_Reset(0);
    AppIrFilter_Push(0, 5U);
    CHECK(!AppIrFilter_IsFull(0));
    CHECK(!AppIrFilter_Output(0, &out));
    CHECK(!AppIrFilter_Output(&f, 0));
}

/* Behaviour that depends on the median-of-9 default; skipped for the
 * alternative window / rank builds above. */
#if (APP_IR_FILTER_WINDOW == 9U) && (APP_IR_FILTER_RANK == 4U)
static void test_median_of_9_rejects_spikes(void)
{
    AppIrFilter f;
    uint16_t out = 0U;
    unsigned i;

    /* 4 upward spikes in 9 samples: the median is still a quiet sample. */
    AppIrFilter_Reset(&f);
    for (i = 0U; i < 9U; i++)
    {
        AppIrFilter_Push(&f, (i % 2U == 0U) ? 1500U : (uint16_t)(1900U + i));
    }
    CHECK(AppIrFilter_Output(&f, &out));
    CHECK(out == 1500U);

    /* 5 of 9 high: the median follows them (a median cannot do better). */
    AppIrFilter_Reset(&f);
    for (i = 0U; i < 9U; i++)
    {
        AppIrFilter_Push(&f, (i < 5U) ? 2000U : 1500U);
    }
    CHECK(AppIrFilter_Output(&f, &out));
    CHECK(out == 2000U);

    /* Step response: after 5 new samples the output has moved, after 4 not. */
    AppIrFilter_Reset(&f);
    for (i = 0U; i < 9U; i++) { AppIrFilter_Push(&f, 1000U); }
    for (i = 0U; i < 4U; i++) { AppIrFilter_Push(&f, 3000U); }
    CHECK(AppIrFilter_Output(&f, &out) && out == 1000U);
    AppIrFilter_Push(&f, 3000U);
    CHECK(AppIrFilter_Output(&f, &out) && out == 3000U);
}
#endif

int main(void)
{
    printf("filter: window %u, rank %u (0-based, ascending)\n",
           (unsigned)APP_IR_FILTER_WINDOW, (unsigned)APP_IR_FILTER_RANK);

    test_exhaustive_conversion();
    test_guard_band_boundaries();
    test_saturation_and_invalid_input();
    test_left_right_coefficients_not_interchanged();
    test_report_medians();
    test_filter();
#if (APP_IR_FILTER_WINDOW == 9U) && (APP_IR_FILTER_RANK == 4U)
    test_median_of_9_rejects_spikes();
#endif

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
