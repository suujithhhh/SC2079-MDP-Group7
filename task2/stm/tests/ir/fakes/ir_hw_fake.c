/* See ir_hw_fake.h. */
#include "ir_hw_fake.h"
#include "FreeRTOS.h"

#include <string.h>

#define FAKE_HAL_ADC_ERROR_DMA 0x04U    /* value of HAL_ADC_ERROR_DMA */

IrHwFake irHw;

void IrHwFake_Reset(void)
{
    memset(&irHw, 0, sizeof(irHw));
    irHw.ready = true;
    irHw.startResult = true;
    irHw.recoverResult = true;
}

static void WriteBuffer(uint16_t left, uint16_t right)
{
    if (irHw.buffer != NULL)
    {
        irHw.buffer[0] = left;
        irHw.buffer[1] = right;
    }
}

void IrHwFake_Complete(uint16_t left, uint16_t right)
{
    WriteBuffer(left, right);
    irHw.remaining = 0U;
    irHw.inFlight = false;
    AppIr_OnConvComplete();
}

void IrHwFake_CompleteNoWrite(void)
{
    irHw.remaining = 0U;
    irHw.inFlight = false;
    AppIr_OnConvComplete();
}

void IrHwFake_Error(uint32_t halAdcErrorCode)
{
    irHw.inFlight = false;
    AppIr_OnError(halAdcErrorCode);
}

uint16_t AppIrHw_CheckConfig(void)
{
    irHw.checkCalls++;
    return irHw.cfgFault;
}

bool AppIrHw_Ready(void)
{
    irHw.readyCalls++;
    return irHw.ready;
}

bool AppIrHw_Start(volatile uint16_t *buffer)
{
    irHw.startCalls++;

    if (FakeCrit_Depth != 0)
    {
        irHw.halInCritical++;
    }

    if (irHw.inFlight)
    {
        irHw.overlaps++;
    }

    irHw.buffer = buffer;
    irHw.bufferAtStart[0] = buffer[0];
    irHw.bufferAtStart[1] = buffer[1];

    if (!irHw.startResult)
    {
        return false;
    }

    irHw.inFlight = true;
    irHw.remaining = 2U;

    if (irHw.inside == IRFAKE_INSIDE_COMPLETE)
    {
        IrHwFake_Complete(irHw.insideLeft, irHw.insideRight);
    }
    else if (irHw.inside == IRFAKE_INSIDE_ERROR)
    {
        IrHwFake_Error(irHw.insideErrorCode);
    }

    return true;
}

uint16_t AppIrHw_Remaining(void)
{
    return irHw.remaining;
}

bool AppIrHw_TakeOverrun(void)
{
    bool was = irHw.overrun;

    irHw.overrunCalls++;
    irHw.overrun = false;
    return was;
}

bool AppIrHw_ErrorIsDma(uint32_t halAdcErrorCode)
{
    return (halAdcErrorCode & FAKE_HAL_ADC_ERROR_DMA) != 0U;
}

bool AppIrHw_Recover(void)
{
    irHw.recoverCalls++;

    if (FakeCrit_Depth != 0)
    {
        irHw.halInCritical++;
    }

    if (irHw.recoverResult)
    {
        irHw.inFlight = false;     /* stream aborted, ADC off */
    }

    return irHw.recoverResult;
}
