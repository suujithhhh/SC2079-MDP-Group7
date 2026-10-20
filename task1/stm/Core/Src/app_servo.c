/*
 * app_servo.c
 *
 *  Created on: 1 Sept 2026
 *      Author: ksuji
 */


#include "app_servo.h"
#include "main.h"

extern TIM_HandleTypeDef htim12;

static volatile uint32_t commandedPulseUs = SERVO_CENTER_US;

void AppServo_Init(void)
{
    /* Put steering at the known physical centre BEFORE PWM starts. */
    AppServo_SetUs(SERVO_CENTER_US);

    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

void AppServo_SetUs(uint32_t pulseUs)
{
    if (pulseUs < SERVO_MIN_US)
    {
        pulseUs = SERVO_MIN_US;
    }
    else if (pulseUs > SERVO_MAX_US)
    {
        pulseUs = SERVO_MAX_US;
    }

    commandedPulseUs = pulseUs;
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pulseUs);
}

uint32_t AppServo_GetUs(void)
{
    return commandedPulseUs;
}

void AppServo_Center(void)
{
    AppServo_SetUs(SERVO_CENTER_US);
}
