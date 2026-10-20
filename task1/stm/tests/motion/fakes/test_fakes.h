/* Host-test control knobs for tests/motion/fakes/hw_stubs.c + task.h --
 * lets a test drive synthetic encoder/heading/gyro/tick/IMU-ready
 * values through the real AppMotion_HandleCommand()/AppMotion_Update(),
 * and observe what the real code actually commanded to the servo and
 * motors. Test-only, not part of the firmware. */
#ifndef TEST_FAKES_H
#define TEST_FAKES_H

#include <stdint.h>
#include <stdbool.h>

void Fake_SetEncoderLeft(int32_t value);
void Fake_SetEncoderRight(int32_t value);
void Fake_SetHeading(float value);
void Fake_SetGyroZ(float value);
void Fake_SetTickCount(uint32_t value);
void Fake_SetImuReady(bool ready);

int32_t Fake_GetLastMotorLeftDuty(void);
int32_t Fake_GetLastMotorRightDuty(void);
uint32_t Fake_GetMotorSetCallCount(void);
uint32_t Fake_GetMotorStopCallCount(void);
uint32_t Fake_GetMotorBrakeCallCount(void);
void Fake_ResetMotorCallCounts(void);

#endif /* TEST_FAKES_H */
