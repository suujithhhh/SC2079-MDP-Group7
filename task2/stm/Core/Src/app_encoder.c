/*
 * app_encoder.c
 *
 *  Created on: 1 Sept 2026
 *      Author: ksuji
 */


#include "app_encoder.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
static AppEncoderSnapshot encoderSnapshot = {0};

void AppEncoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    AppEncoder_Reset();
}

void AppEncoder_Reset(void)
{
    taskENTER_CRITICAL();

    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);

    encoderSnapshot = (AppEncoderSnapshot){0};

    taskEXIT_CRITICAL();
}

int32_t AppEncoder_Left(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

int32_t AppEncoder_Right(void)
{
    int32_t raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    return -raw;
}

int32_t AppEncoder_Average(void)
{
    return (AppEncoder_Left() + AppEncoder_Right()) / 2;
}

void AppEncoder_Update(void)
{
    AppEncoderSnapshot snapshot;

    taskENTER_CRITICAL();

    snapshot.left = AppEncoder_Left();
    snapshot.right = AppEncoder_Right();

    snapshot.average =
        (snapshot.left + snapshot.right) / 2;

    encoderSnapshot = snapshot;

    taskEXIT_CRITICAL();
}

AppEncoderSnapshot AppEncoder_GetSnapshot(void)
{
    AppEncoderSnapshot snapshot;

    taskENTER_CRITICAL();
    snapshot = encoderSnapshot;
    taskEXIT_CRITICAL();

    return snapshot;
}
