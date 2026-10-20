/* Host-test stubs for the hardware-abstraction functions app_motion.c's
 * code still references (AppEncoder_*, AppIMU_*, AppMotor_*, AppServo_*,
 * xTaskGetTickCount()). These exist purely to satisfy the linker --
 * MinGW's ld does not defer undefined-symbol resolution for sections
 * that --gc-sections later discards (unlike some ELF linkers), so real
 * (if trivial) definitions are required.
 *
 * AppEncoder_Left/Right, AppIMU_Heading/GyroZ/IsReady,
 * AppServo_SetUs/GetUs, AppMotor_Set/Stop/Brake, and
 * xTaskGetTickCount() are backed by settable/observable fake state
 * (see test_fakes.h) so app_motion_telemetry_test.c and
 * app_motion_presteer_test.c can drive synthetic encoder/heading/
 * gyro/time/IMU-ready values through the real
 * AppMotion_HandleCommand()/AppMotion_Update(), and observe what the
 * real code actually commanded to the servo/motors. All default to
 * the same inert 0/0.0f/true values the original always-constant
 * stubs returned, so the existing app_motion_profile_test.c (which
 * never calls any Fake_Set*()) is unaffected. */

#include "app_encoder.h"
#include "app_imu.h"
#include "app_motor.h"
#include "app_servo.h"
#include "task.h"
#include "test_fakes.h"

static int32_t fakeEncoderLeft = 0;
static int32_t fakeEncoderRight = 0;
static float fakeHeading = 0.0f;
static float fakeGyroZ = 0.0f;
static uint32_t fakeTickCount = 0U;
static bool fakeImuReady = true;
static AppIMUEvent fakeImuEvent = { APP_IMU_EVENT_NONE, 0U };
static uint32_t imuCancelCallCount = 0U;

static uint32_t fakeServoPulseUs = 0U;
static int32_t lastMotorLeftDuty = 0;
static int32_t lastMotorRightDuty = 0;
static uint32_t motorSetCallCount = 0U;
static uint32_t motorStopCallCount = 0U;
static uint32_t motorBrakeCallCount = 0U;

void Fake_SetEncoderLeft(int32_t value)  { fakeEncoderLeft = value; }
void Fake_SetEncoderRight(int32_t value) { fakeEncoderRight = value; }
void Fake_SetHeading(float value)        { fakeHeading = value; }
void Fake_SetGyroZ(float value)          { fakeGyroZ = value; }
void Fake_SetTickCount(uint32_t value)   { fakeTickCount = value; }
void Fake_SetImuReady(bool ready)        { fakeImuReady = ready; }
void Fake_SetImuEvent(AppIMUEventType type, uint32_t sequence)
{
    fakeImuEvent.type = type;
    fakeImuEvent.sequence = sequence;
}

uint32_t Fake_GetTickCount(void)           { return fakeTickCount; }
uint32_t Fake_GetImuCancelCallCount(void)  { return imuCancelCallCount; }

int32_t Fake_GetLastMotorLeftDuty(void)  { return lastMotorLeftDuty; }
int32_t Fake_GetLastMotorRightDuty(void) { return lastMotorRightDuty; }
uint32_t Fake_GetMotorSetCallCount(void)   { return motorSetCallCount; }
uint32_t Fake_GetMotorStopCallCount(void)  { return motorStopCallCount; }
uint32_t Fake_GetMotorBrakeCallCount(void) { return motorBrakeCallCount; }

void Fake_ResetMotorCallCounts(void)
{
    motorSetCallCount = 0U;
    motorStopCallCount = 0U;
    motorBrakeCallCount = 0U;
}

TickType_t xTaskGetTickCount(void) { return fakeTickCount; }

void AppEncoder_Reset(void) { fakeEncoderLeft = 0; fakeEncoderRight = 0; }
int32_t AppEncoder_Left(void) { return fakeEncoderLeft; }
int32_t AppEncoder_Right(void) { return fakeEncoderRight; }

bool AppIMU_CancelPendingRequest(void) { imuCancelCallCount++; return true; }
bool AppIMU_RequestZero(uint32_t *eventSequence)
{
    if (eventSequence != NULL) { *eventSequence = fakeImuEvent.sequence; }
    return true;
}
bool AppIMU_RequestFlip(uint32_t *eventSequence)
{
    if (eventSequence != NULL) { *eventSequence = fakeImuEvent.sequence; }
    return true;
}
void AppIMU_GetEvent(AppIMUEvent *event)
{
    if (event != NULL) { *event = fakeImuEvent; }
}
float AppIMU_Heading(void) { return fakeHeading; }
float AppIMU_GyroZ(void) { return fakeGyroZ; }
bool AppIMU_IsReady(void) { return fakeImuReady; }

void AppMotor_Set(int32_t leftDuty, int32_t rightDuty)
{
    lastMotorLeftDuty = leftDuty;
    lastMotorRightDuty = rightDuty;
    motorSetCallCount++;
}
void AppMotor_SetLeft(int32_t duty) { (void)duty; }
void AppMotor_SetRight(int32_t duty) { (void)duty; }
void AppMotor_Stop(void) { motorStopCallCount++; }
void AppMotor_Brake(void) { motorBrakeCallCount++; }

void AppServo_SetUs(uint32_t pulseUs) { fakeServoPulseUs = pulseUs; }
uint32_t AppServo_GetUs(void) { return fakeServoPulseUs; }
void AppServo_Center(void) { fakeServoPulseUs = 1530U; }
