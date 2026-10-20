/*
 * app_pid.h
 *
 *  Created on: 2 Sept 2026
 *      Author: ksuji
 */

#ifndef INC_APP_PID_H_
#define INC_APP_PID_H_

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float previousError;

    float outputMin;
    float outputMax;

    float integralMin;
    float integralMax;

    uint8_t initialized;

} PIDController;


/* Configure a PID controller. */
void PID_Init(
    PIDController *pid,
    float kp,
    float ki,
    float kd,
    float outputMin,
    float outputMax,
    float integralMin,
    float integralMax
);


/* Change gains without recreating the controller. */
void PID_SetGains(
    PIDController *pid,
    float kp,
    float ki,
    float kd
);


/* Clear integral + derivative history. */
void PID_Reset(PIDController *pid);


/* Run one PID calculation. */
float PID_Update(
    PIDController *pid,
    float setpoint,
    float measurement,
    float dtSeconds
);

#endif /* INC_APP_PID_H_ */
