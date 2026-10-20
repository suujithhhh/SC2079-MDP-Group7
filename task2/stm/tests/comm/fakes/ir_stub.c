/* See ir_stub.h. */
#include "ir_stub.h"

#if APP_IR_ENABLE

static AppIrSnapshot current;
static AppIrReading currentReading;
static unsigned getCalls;
static unsigned readingCalls;

void IrStub_Set(const AppIrSnapshot *snapshot) { current = *snapshot; }
unsigned IrStub_GetCallCount(void) { return getCalls; }

void IrStub_SetReading(const AppIrReading *reading) { currentReading = *reading; }
unsigned IrStub_GetReadingCallCount(void) { return readingCalls; }
void IrStub_ResetCounts(void) { getCalls = 0U; readingCalls = 0U; }

void AppIr_GetSnapshot(AppIrSnapshot *snapshot)
{
    getCalls++;
    *snapshot = current;
}

void AppIr_GetReading(AppIrReading *reading, uint32_t nowTick)
{
    (void)nowTick;
    readingCalls++;
    *reading = currentReading;
}

#endif /* APP_IR_ENABLE */
