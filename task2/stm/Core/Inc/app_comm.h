/*
 * app_comm.h
 *
 *  Created on: 1 Sept 2026
 *      Author: ksuji
 */

#ifndef INC_APP_COMM_H_
#define INC_APP_COMM_H_

#include <stdint.h>
#include <stdbool.h>

#include "app_motion.h"
#include "app_park_trace.h"
#include "app_ir.h"

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

/* Ultrasonic parking protection (USPARK). CommTask only: StartCommTask()
 * calls it right after AppComm_ServiceTelemetry(). Non-blocking; returns at
 * once while USPARK is disabled (the boot default). */
void AppComm_ServiceParking(void);

#if APP_USPARK_TRACE
/* Measurement-only USPARK trace (bench builds, -DAPP_USPARK_TRACE=1). CommTask
 * only: StartCommTask() calls it right after AppComm_ServiceParking(). It
 * closes a finished window, expires bounded waits and, only when the link is
 * idle and the window's DONE record exists, sends at most ONE trace line per
 * call. Never blocks beyond that one line's normal UART transmit. */
void AppComm_ServiceParkTrace(void);
#endif

#if APP_IR_ENABLE
/* IR,GET diagnostics (host tests, debugger; not on the wire). CommTask owns the
 * counters; read them from CommTask or from a test that drives it.
 *   dropped:   IR,GET requests discarded because 4 replies were already owed.
 *              A client that keeps ONE request outstanding never raises it.
 *   highWater: the most replies ever owed at once since AppComm_Init() (1 for
 *              a one-outstanding client). */
uint16_t AppComm_IrGetDroppedCount(void);
uint8_t AppComm_IrGetPendingHighWater(void);
#endif

/* USD,GET diagnostics, same meaning as the IR,GET pair above (always built). */
uint16_t AppComm_UsdGetDroppedCount(void);
uint8_t AppComm_UsdGetPendingHighWater(void);

/* MotionTask's command source. Returns true and fills *command with the
 * next command to dispatch: a pending priority STOP ({MOTION_STOP, 0})
 * first, otherwise the next queued command. While a STOP's queue fence is
 * raised it returns false without reading the normal queue. Non-blocking;
 * call only from MotionTask. */
bool AppComm_TakeMotionCommand(MotionCommand *command);

#endif /* INC_APP_COMM_H_ */
