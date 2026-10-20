/*
 * app_imu.h
 *
 *  Created on: 1 Sept 2026
 *      Author: ksuji
 */

#ifndef INC_APP_IMU_H_
#define INC_APP_IMU_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float headingDeg;
    float gyroZDps;
    int8_t headingSign;
    uint8_t ready;
} AppIMUSnapshot;

typedef enum
{
    APP_IMU_EVENT_NONE = 0,
    APP_IMU_EVENT_ZERO_DONE,
    APP_IMU_EVENT_ZERO_FAILED,
    APP_IMU_EVENT_FLIP_DONE,
    APP_IMU_EVENT_FLIP_FAILED
} AppIMUEventType;

typedef struct
{
    AppIMUEventType type;
    uint32_t sequence;
} AppIMUEvent;

bool AppIMU_Init(void);
bool AppIMU_Calibrate(void);
bool AppIMU_Update(float dtSeconds);

void AppIMU_ZeroHeading(void);

bool AppIMU_RequestZero(uint32_t *eventSequence);
bool AppIMU_RequestFlip(uint32_t *eventSequence);
bool AppIMU_CancelPendingRequest(void);
bool AppIMU_ServiceRequest(void);

void AppIMU_GetSnapshot(AppIMUSnapshot *snapshot);
void AppIMU_GetEvent(AppIMUEvent *event);

float AppIMU_Heading(void);
float AppIMU_GyroZ(void);

bool AppIMU_IsReady(void);



#endif /* INC_APP_IMU_H_ */
