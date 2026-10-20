/*
 * app_pid.c
 *
 *  Created on: 2 Sept 2026
 *      Author: ksuji
 */


#include "app_pid.h"

#include <stddef.h>


static float Clamp(
    float value,
    float minimum,
    float maximum
)
{
    if (value < minimum)
        return minimum;

    if (value > maximum)
        return maximum;

    return value;
}


void PID_Init(
    PIDController *pid,
    float kp,
    float ki,
    float kd,
    float outputMin,
    float outputMax,
    float integralMin,
    float integralMax
)
{
    if (pid == NULL)
        return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->outputMin = outputMin;
    pid->outputMax = outputMax;

    pid->integralMin = integralMin;
    pid->integralMax = integralMax;

    PID_Reset(pid);
}


void PID_SetGains(
    PIDController *pid,
    float kp,
    float ki,
    float kd
)
{
    if (pid == NULL)
        return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}


void PID_Reset(PIDController *pid)
{
    if (pid == NULL)
        return;

    pid->integral = 0.0f;
    pid->previousError = 0.0f;
    pid->initialized = 0U;
}


float PID_Update(
    PIDController *pid,
    float setpoint,
    float measurement,
    float dtSeconds
)
{
    if ((pid == NULL) || (dtSeconds <= 0.0f))
        return 0.0f;

    float error = setpoint - measurement;

    /* ---------- P ---------- */

    float proportional =
        pid->kp * error;


    /* ---------- I ---------- */

    float candidateIntegral =
        pid->integral + error * dtSeconds;

    candidateIntegral = Clamp(
        candidateIntegral,
        pid->integralMin,
        pid->integralMax
    );


    /* ---------- D ---------- */

    float derivative = 0.0f;

    if (pid->initialized)
    {
        derivative =
            (error - pid->previousError)
            / dtSeconds;
    }


    /* First calculate using the candidate integral. */

    float output =
        proportional
        + pid->ki * candidateIntegral
        + pid->kd * derivative;


    /*
     * Simple anti-windup:
     *
     * If output is saturated AND the current error
     * would push it even further into saturation,
     * don't accumulate more integral.
     */
    uint8_t allowIntegral = 1U;

    if ((output > pid->outputMax) &&
        (error > 0.0f))
    {
        allowIntegral = 0U;
    }

    if ((output < pid->outputMin) &&
        (error < 0.0f))
    {
        allowIntegral = 0U;
    }

    if (allowIntegral)
    {
        pid->integral = candidateIntegral;
    }


    /*
     * Recalculate using the actual accepted
     * integral value.
     */

    output =
        proportional
        + pid->ki * pid->integral
        + pid->kd * derivative;


    output = Clamp(
        output,
        pid->outputMin,
        pid->outputMax
    );


    pid->previousError = error;
    pid->initialized = 1U;

    return output;
}
