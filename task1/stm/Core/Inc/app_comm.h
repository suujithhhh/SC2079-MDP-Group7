/*
 * app_comm.h
 *
 *  Created on: 1 Sept 2026
 *      Author: ksuji
 */

#ifndef INC_APP_COMM_H_
#define INC_APP_COMM_H_

#include <stdint.h>

typedef struct
{
    uint32_t errorCount;
    uint32_t droppedCount;
    uint8_t recovering;
} AppCommRxStatus;

void AppComm_Init(void);
void AppComm_ProcessByte(uint8_t byte);
void AppComm_ServiceTelemetry(void);
void AppComm_GetRxStatus(AppCommRxStatus *status);

#endif /* INC_APP_COMM_H_ */
