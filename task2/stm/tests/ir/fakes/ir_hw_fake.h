/* Host-test fake of Core/Inc/app_ir_hw.h (the ADC1 / DMA2 Stream0 port).
 *
 * Models just enough of one ADC scan for the REAL Core/Src/app_ir.c:
 *   - AppIrHw_Start() records the buffer and its contents at start time,
 *     marks a scan "in flight" and counts an OVERLAP if a scan was already in
 *     flight (a start before completion / error / recovery);
 *   - the test then decides the outcome with IrHwFake_Complete() /
 *     IrHwFake_Error() (which call the real ISR entry points, as the DMA
 *     interrupt would), or lets the scan time out;
 *   - an outcome can also be fired INSIDE AppIrHw_Start(), modelling the scan
 *     finishing before HAL_ADC_Start_DMA() returns (IRTask pre-empted);
 *   - the critical-section depth seen by Start/Recover is recorded, so the
 *     tests can prove the HAL is never called inside a critical section.
 */
#ifndef IR_HW_FAKE_H
#define IR_HW_FAKE_H

#include "app_ir_hw.h"

typedef enum
{
    IRFAKE_INSIDE_NONE = 0,
    IRFAKE_INSIDE_COMPLETE,     /* write values, NDTR 0, OnConvComplete() */
    IRFAKE_INSIDE_ERROR         /* OnError(insideErrorCode)               */
} IrFakeInside;

typedef struct
{
    /* behaviour */
    uint16_t cfgFault;          /* AppIrHw_CheckConfig() result            */
    bool ready;                 /* AppIrHw_Ready() result                  */
    bool startResult;           /* AppIrHw_Start() result                  */
    bool recoverResult;         /* AppIrHw_Recover() result                */
    bool overrun;               /* sticky until AppIrHw_TakeOverrun()      */
    uint16_t remaining;         /* AppIrHw_Remaining() result              */
    IrFakeInside inside;        /* outcome fired inside AppIrHw_Start()    */
    uint16_t insideLeft;
    uint16_t insideRight;
    uint32_t insideErrorCode;

    /* observation */
    unsigned checkCalls;
    unsigned readyCalls;
    unsigned startCalls;
    unsigned recoverCalls;
    unsigned overrunCalls;
    unsigned overlaps;          /* Start while a scan was in flight        */
    unsigned halInCritical;     /* Start/Recover called inside a crit. sec */
    bool inFlight;
    volatile uint16_t *buffer;  /* buffer passed to the latest Start       */
    uint16_t bufferAtStart[2];  /* its contents when Start was called      */
} IrHwFake;

extern IrHwFake irHw;

/* Healthy defaults; clears every counter. */
void IrHwFake_Reset(void);

/* DMA wrote both words, NDTR = 0, transfer-complete callback. */
void IrHwFake_Complete(uint16_t left, uint16_t right);
/* Transfer-complete callback without the DMA writing anything. */
void IrHwFake_CompleteNoWrite(void);
/* HAL_ADC_ErrorCallback with this hadc->ErrorCode. */
void IrHwFake_Error(uint32_t halAdcErrorCode);

#endif /* IR_HW_FAKE_H */
