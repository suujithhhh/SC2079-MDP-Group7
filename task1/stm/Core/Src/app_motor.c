/*
 * app_motor.c
 *
 *  Created on: 1 Sept 2026
 *      Author: ksuji
 */


#include "app_motor.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;

static uint32_t ClampDuty(uint32_t duty)
{
    return duty > MOTOR_MAX_DUTY ? MOTOR_MAX_DUTY : duty;
}

void AppMotor_Init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    AppMotor_Stop();
}

/* LEFT MOTOR — Motor A
 * Forward  = TIM4 CH4 / PB9
 * Backward = TIM4 CH3 / PB8
 */

void AppMotor_LeftForward(uint32_t duty)
{
    duty = ClampDuty(duty);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, duty);
}

void AppMotor_LeftBackward(uint32_t duty)
{
    duty = ClampDuty(duty);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty);
}

/* RIGHT MOTOR — Motor B
 * Forward  = TIM9 CH2 / PE6
 * Backward = TIM9 CH1 / PE5
 */

void AppMotor_RightForward(uint32_t duty)
{
    duty = ClampDuty(duty);

    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, duty);
}

void AppMotor_RightBackward(uint32_t duty)
{
    duty = ClampDuty(duty);

    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, duty);
}

void AppMotor_Forward(uint32_t leftDuty, uint32_t rightDuty)
{
    AppMotor_LeftForward(leftDuty);
    AppMotor_RightForward(rightDuty);
}

void AppMotor_Backward(uint32_t leftDuty, uint32_t rightDuty)
{
    AppMotor_LeftBackward(leftDuty);
    AppMotor_RightBackward(rightDuty);
}

void AppMotor_SetLeft(int32_t duty)
{
    if (duty > (int32_t)MOTOR_MAX_DUTY)
        duty = MOTOR_MAX_DUTY;

    if (duty < -(int32_t)MOTOR_MAX_DUTY)
        duty = -(int32_t)MOTOR_MAX_DUTY;

    if (duty > 0)
    {
        AppMotor_LeftForward((uint32_t)duty);
    }
    else if (duty < 0)
    {
        AppMotor_LeftBackward((uint32_t)(-duty));
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0U);
    }
}


void AppMotor_SetRight(int32_t duty)
{
    if (duty > (int32_t)MOTOR_MAX_DUTY)
        duty = MOTOR_MAX_DUTY;

    if (duty < -(int32_t)MOTOR_MAX_DUTY)
        duty = -(int32_t)MOTOR_MAX_DUTY;

    if (duty > 0)
    {
        AppMotor_RightForward((uint32_t)duty);
    }
    else if (duty < 0)
    {
        AppMotor_RightBackward((uint32_t)(-duty));
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0U);
    }
}


void AppMotor_Set(int32_t leftDuty, int32_t rightDuty)
{
    AppMotor_SetLeft(leftDuty);
    AppMotor_SetRight(rightDuty);
}

void AppMotor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0U);

    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0U);
}

void AppMotor_Brake(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_MAX_DUTY);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_MAX_DUTY);

    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, MOTOR_MAX_DUTY);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, MOTOR_MAX_DUTY);
}
