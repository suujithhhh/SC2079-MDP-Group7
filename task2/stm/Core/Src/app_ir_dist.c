/*
 * app_ir_dist.c
 *
 * Phase 3 IR conversion and filter. HAL-free and RTOS-free. See
 * app_ir_dist.h for the calibration, the reference (sensor face) and the
 * calibrated domain.
 *
 * Compiled only when the IR feature is on (APP_IR_ENABLE), so a default build
 * contains none of it.
 */

#include "app_ir.h"

#if APP_IR_ENABLE

#include "app_ir_dist.h"

/* mm = A / (adc + B) - C. Calibration-report values in cm, times 10.
 * LEFT : 31785.22973379 / (ADC + 26.48617844) - 0.29510522
 * RIGHT: 32831.45       / (ADC - 94.6824)     - 1.02464 */
#define IR_LEFT_A_MM    317852.2973379f
#define IR_LEFT_B       26.48617844f
#define IR_LEFT_C_MM    2.9510522f

#define IR_RIGHT_A_MM   328314.5f
#define IR_RIGHT_B      (-94.6824f)
#define IR_RIGHT_C_MM   10.2464f

AppIrRange AppIrDist_Convert(AppIrSensor sensor, uint16_t filteredAdc, uint16_t *mm)
{
    float a;
    float b;
    float c;
    float value;
    uint16_t result;
    uint16_t farLimit;
    uint16_t nearLimit;

    if (mm != 0)
    {
        *mm = 0U;
    }

    if ((mm == 0) || (filteredAdc > APP_IR_ADC_MAX))
    {
        return APP_IR_RANGE_INVALID;
    }

    if (sensor == APP_IR_SENSOR_LEFT)
    {
        a = IR_LEFT_A_MM;
        b = IR_LEFT_B;
        c = IR_LEFT_C_MM;
        farLimit = APP_IR_LEFT_ADC_FAR_LIMIT;
        nearLimit = APP_IR_LEFT_ADC_NEAR_LIMIT;
    }
    else if (sensor == APP_IR_SENSOR_RIGHT)
    {
        a = IR_RIGHT_A_MM;
        b = IR_RIGHT_B;
        c = IR_RIGHT_C_MM;
        farLimit = APP_IR_RIGHT_ADC_FAR_LIMIT;
        nearLimit = APP_IR_RIGHT_ADC_NEAR_LIMIT;
    }
    else
    {
        return APP_IR_RANGE_INVALID;
    }

    /* Integer decision first: nothing outside the guard band is ever converted
     * (the curves are evaluated only inside the 95-405 mm band), and the divisor below can never approach the curve's
     * pole. */
    if (filteredAdc < farLimit)
    {
        return APP_IR_RANGE_FAR;
    }

    if (filteredAdc > nearLimit)
    {
        return APP_IR_RANGE_NEAR;
    }

    value = (a / ((float)filteredAdc + b)) - c;

    /* Round half up, then clamp the guard band (curve value 95..405 mm) onto
     * 100..400 mm. Checked for every count by the host test. */
    result = (uint16_t)(value + 0.5f);

    if (result < APP_IR_DIST_MIN_MM)
    {
        result = APP_IR_DIST_MIN_MM;
    }
    else if (result > APP_IR_DIST_MAX_MM)
    {
        result = APP_IR_DIST_MAX_MM;
    }

    *mm = result;
    return APP_IR_RANGE_OK;
}

void AppIrFilter_Reset(AppIrFilter *filter)
{
    uint8_t i;

    if (filter == 0)
    {
        return;
    }

    for (i = 0U; i < APP_IR_FILTER_WINDOW; i++)
    {
        filter->samples[i] = 0U;
    }

    filter->next = 0U;
    filter->count = 0U;
}

void AppIrFilter_Push(AppIrFilter *filter, uint16_t rawAdc)
{
    if (filter == 0)
    {
        return;
    }

    filter->samples[filter->next] = rawAdc;
    filter->next++;

    if (filter->next >= APP_IR_FILTER_WINDOW)
    {
        filter->next = 0U;
    }

    if (filter->count < APP_IR_FILTER_WINDOW)
    {
        filter->count++;
    }
}

bool AppIrFilter_IsFull(const AppIrFilter *filter)
{
    return (filter != 0) && (filter->count >= APP_IR_FILTER_WINDOW);
}

bool AppIrFilter_Output(const AppIrFilter *filter, uint16_t *out)
{
    uint16_t sorted[APP_IR_FILTER_WINDOW];
    uint8_t i;
    uint8_t j;

    if ((filter == 0) || (out == 0) || !AppIrFilter_IsFull(filter))
    {
        return false;
    }

    for (i = 0U; i < APP_IR_FILTER_WINDOW; i++)
    {
        uint16_t value = filter->samples[i];

        /* Insertion sort, ascending. */
        for (j = i; (j > 0U) && (sorted[j - 1U] > value); j--)
        {
            sorted[j] = sorted[j - 1U];
        }

        sorted[j] = value;
    }

    *out = sorted[APP_IR_FILTER_RANK];
    return true;
}

#endif /* APP_IR_ENABLE */
