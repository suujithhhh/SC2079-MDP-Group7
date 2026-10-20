/*
 * app_encoder.h
 *
 *  Created on: 1 Sept 2026
 *      Author: ksuji
 */

#ifndef INC_APP_ENCODER_H_
#define INC_APP_ENCODER_H_

#include <stdint.h>

#define ENCODER_COUNTS_PER_CM 72U

typedef struct
{
    int32_t left;
    int32_t right;
    int32_t average;
} AppEncoderSnapshot;

void AppEncoder_Update(void);
AppEncoderSnapshot AppEncoder_GetSnapshot(void);

void AppEncoder_Init(void);
void AppEncoder_Reset(void);

int32_t AppEncoder_Left(void);
int32_t AppEncoder_Right(void);

int32_t AppEncoder_Average(void);

#endif /* INC_APP_ENCODER_H_ */
