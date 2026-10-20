/*
 * app_motor.h
 *
 *  Created on: 1 Sept 2026
 *      Author: ksuji
 */

#ifndef INC_APP_MOTOR_H_
#define INC_APP_MOTOR_H_

#include <stdint.h>

#define MOTOR_LEFT_BASE_DUTY   96U
#define MOTOR_RIGHT_BASE_DUTY  99U
#define MOTOR_MAX_DUTY         99U

void AppMotor_Init(void);


void AppMotor_LeftForward(uint32_t duty);
void AppMotor_LeftBackward(uint32_t duty);

void AppMotor_RightForward(uint32_t duty);
void AppMotor_RightBackward(uint32_t duty);

void AppMotor_Forward(uint32_t leftDuty, uint32_t rightDuty);
void AppMotor_Backward(uint32_t leftDuty, uint32_t rightDuty);

void AppMotor_SetLeft(int32_t duty);
void AppMotor_SetRight(int32_t duty);

void AppMotor_Set(int32_t leftDuty, int32_t rightDuty);

void AppMotor_Stop(void);
void AppMotor_Brake(void);


#endif /* INC_APP_MOTOR_H_ */
