/* Test-only snapshot / reading provider for the IR telemetry in app_comm.c
 * (the IRRAW bench log and the IR,GET reply).
 * Stands in for Core/Src/app_ir.c so the REAL app_comm.c formatters can be
 * driven with arbitrary snapshots and readings. Compiles to nothing unless
 * APP_IR_ENABLE is 1 (the header then declares no IR types). */
#ifndef IR_STUB_H
#define IR_STUB_H

#include "app_ir.h"

#if APP_IR_ENABLE
void IrStub_Set(const AppIrSnapshot *snapshot);
unsigned IrStub_GetCallCount(void);

/* AppIr_GetReading() returns this reading exactly as set (the derivation
 * from a snapshot lives in app_ir.c and is tested by tests/ir/). */
void IrStub_SetReading(const AppIrReading *reading);
unsigned IrStub_GetReadingCallCount(void);
/* Zeroes both call counters (the data is kept). */
void IrStub_ResetCounts(void);
#endif

#endif /* IR_STUB_H */
