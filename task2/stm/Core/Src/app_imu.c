/*
 * app_imu.c
 *
 *  Created on: 1 Sept 2026
 *      Author: ksuji
 */


#include "app_imu.h"
#include "icm20948.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>

#define APP_IMU_READ_FAILURE_THRESHOLD 3U

extern I2C_HandleTypeDef hi2c2;

static float headingDeg = 0.0f;
static float gyroZDps   = 0.0f;
static bool imuInitialized = false;
static bool imuCalibrated = false;
static bool imuReady    = false;
static int8_t headingSign = 1;
static uint8_t consecutiveReadFailures = 0U;

typedef enum
{
    APP_IMU_REQUEST_NONE = 0,
    APP_IMU_REQUEST_ZERO,
    APP_IMU_REQUEST_FLIP
} AppIMURequestType;

static AppIMURequestType pendingRequest =
    APP_IMU_REQUEST_NONE;
static bool requestActive = false;
static AppIMUEvent latestEvent = {
    APP_IMU_EVENT_NONE,
    0U
};

static bool AppIMU_Request(
    AppIMURequestType request,
    uint32_t *eventSequence
)
{
    bool accepted = false;

    if (eventSequence == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();

    if (imuReady && !requestActive)
    {
        *eventSequence = latestEvent.sequence;
        pendingRequest = request;
        requestActive = true;
        accepted = true;
    }

    taskEXIT_CRITICAL();

    return accepted;
}

static void AppIMU_PublishEvent(
    AppIMUEventType type
)
{
    taskENTER_CRITICAL();

    latestEvent.type = type;
    latestEvent.sequence++;

    if (latestEvent.sequence == 0U)
    {
        latestEvent.sequence++;
    }

    requestActive = false;

    taskEXIT_CRITICAL();
}

bool AppIMU_Init(void)
{
    taskENTER_CRITICAL();

    headingDeg = 0.0f;
    gyroZDps = 0.0f;
    imuInitialized = false;
    imuCalibrated = false;
    imuReady = false;
    headingSign = 1;
    consecutiveReadFailures = 0U;
    pendingRequest = APP_IMU_REQUEST_NONE;
    requestActive = false;
    latestEvent.type = APP_IMU_EVENT_NONE;
    latestEvent.sequence = 0U;

    taskEXIT_CRITICAL();

    if (ICM20948_Init(&hi2c2) != HAL_OK)
    {
        return false;
    }

    taskENTER_CRITICAL();

    headingDeg = 0.0f;
    gyroZDps   = 0.0f;
    imuInitialized = true;

    taskEXIT_CRITICAL();

    return true;
}

bool AppIMU_Calibrate(void)
{
    bool initialized;

    taskENTER_CRITICAL();
    initialized = imuInitialized;

    if (initialized)
    {
        /* Public readiness starts only after calibration has completed. */
        imuCalibrated = false;
        imuReady = false;
        gyroZDps = 0.0f;
        consecutiveReadFailures = 0U;
    }

    taskEXIT_CRITICAL();

    if (!initialized)
    {
        return false;
    }

    if (ICM20948_CalibrateGyroZ(500U, 2U) != HAL_OK)
    {
        taskENTER_CRITICAL();
        imuCalibrated = false;
        imuReady = false;
        gyroZDps = 0.0f;
        consecutiveReadFailures = 0U;
        taskEXIT_CRITICAL();
        return false;
    }

    taskENTER_CRITICAL();
    headingDeg = 0.0f;
    gyroZDps = 0.0f;
    imuCalibrated = true;
    imuReady = true;
    consecutiveReadFailures = 0U;
    taskEXIT_CRITICAL();

    return true;
}

bool AppIMU_Update(float dtSeconds)
{
    bool canRead;
    float gyroZ;

    taskENTER_CRITICAL();
    canRead = imuInitialized && imuCalibrated;
    taskEXIT_CRITICAL();

    if (!canRead)
    {
        return false;
    }

    if (ICM20948_ReadGyroZ(&gyroZ) != HAL_OK)
    {
        taskENTER_CRITICAL();

        if (imuInitialized && imuCalibrated)
        {
            if (consecutiveReadFailures <
                APP_IMU_READ_FAILURE_THRESHOLD)
            {
                consecutiveReadFailures++;
            }

            if (consecutiveReadFailures >=
                APP_IMU_READ_FAILURE_THRESHOLD)
            {
                imuReady = false;
                gyroZDps = 0.0f;
            }
        }

        taskEXIT_CRITICAL();
        return false;
    }

    taskENTER_CRITICAL();

    if (!imuInitialized || !imuCalibrated)
    {
        taskEXIT_CRITICAL();
        return false;
    }

    consecutiveReadFailures = 0U;
    gyroZDps = gyroZ * (float)headingSign;

    /* Gyroscope gives degrees/second.
       heading change = angular velocity × time */
    headingDeg += gyroZDps * dtSeconds;
    imuReady = true;

    taskEXIT_CRITICAL();

    return true;
}

void AppIMU_ZeroHeading(void)
{
    taskENTER_CRITICAL();
    headingDeg = 0.0f;
    taskEXIT_CRITICAL();
}

bool AppIMU_RequestZero(uint32_t *eventSequence)
{
    return AppIMU_Request(
        APP_IMU_REQUEST_ZERO,
        eventSequence
    );
}

bool AppIMU_RequestFlip(uint32_t *eventSequence)
{
    return AppIMU_Request(
        APP_IMU_REQUEST_FLIP,
        eventSequence
    );
}

bool AppIMU_CancelPendingRequest(void)
{
    bool cancelled = false;

    taskENTER_CRITICAL();

    /* Once IMUTask has claimed the request, it remains the sole I2C owner. */
    if (requestActive &&
        (pendingRequest != APP_IMU_REQUEST_NONE))
    {
        pendingRequest = APP_IMU_REQUEST_NONE;
        requestActive = false;
        cancelled = true;
    }

    taskEXIT_CRITICAL();

    return cancelled;
}

bool AppIMU_ServiceRequest(void)
{
    AppIMURequestType request;
    AppIMUEventType eventType;
    bool success;

    taskENTER_CRITICAL();
    request = pendingRequest;

    if (request != APP_IMU_REQUEST_NONE)
    {
        pendingRequest = APP_IMU_REQUEST_NONE;
    }

    taskEXIT_CRITICAL();

    if (request == APP_IMU_REQUEST_NONE)
    {
        return false;
    }

    if (request == APP_IMU_REQUEST_ZERO)
    {
        /* Calibration performs I2C and must remain outside the critical section. */
        success = AppIMU_Calibrate();
        eventType = success
            ? APP_IMU_EVENT_ZERO_DONE
            : APP_IMU_EVENT_ZERO_FAILED;
    }
    else
    {
        taskENTER_CRITICAL();

        success = imuReady;

        if (success)
        {
            headingSign = (int8_t)-headingSign;
            headingDeg = -headingDeg;
            gyroZDps = -gyroZDps;
        }

        taskEXIT_CRITICAL();

        eventType = success
            ? APP_IMU_EVENT_FLIP_DONE
            : APP_IMU_EVENT_FLIP_FAILED;
    }

    AppIMU_PublishEvent(eventType);

    return true;
}

void AppIMU_GetSnapshot(AppIMUSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();

    snapshot->headingDeg = headingDeg;
    snapshot->gyroZDps = gyroZDps;
    snapshot->headingSign = headingSign;
    snapshot->ready = imuReady ? 1U : 0U;

    taskEXIT_CRITICAL();
}

void AppIMU_GetEvent(AppIMUEvent *event)
{
    if (event == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *event = latestEvent;
    taskEXIT_CRITICAL();
}

float AppIMU_Heading(void)
{
    AppIMUSnapshot snapshot;

    AppIMU_GetSnapshot(&snapshot);

    return snapshot.headingDeg;
}

float AppIMU_GyroZ(void)
{
    AppIMUSnapshot snapshot;

    AppIMU_GetSnapshot(&snapshot);

    return snapshot.gyroZDps;
}

bool AppIMU_IsReady(void)
{
    AppIMUSnapshot snapshot;

    AppIMU_GetSnapshot(&snapshot);

    return snapshot.ready != 0U;
}
