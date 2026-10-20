/*
 * app_servo.h
 *
 *  Created on: 1 Sept 2026
 *      Author: ksuji
 */

#ifndef INC_APP_SERVO_H_
#define INC_APP_SERVO_H_

#include <stdint.h>

/* Existing steering PID range: calibrated centre +/- 300 us. */
#define SERVO_MIN_US    1000U
#define SERVO_CENTER_US 1530U
#define SERVO_MAX_US    2060U

void AppServo_Init(void);

void AppServo_SetUs(uint32_t pulseUs);
uint32_t AppServo_GetUs(void);
void AppServo_Center(void);

#endif /* INC_APP_SERVO_H_ */
