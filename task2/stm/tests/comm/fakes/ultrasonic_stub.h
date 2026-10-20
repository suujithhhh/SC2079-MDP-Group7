/* Test-only snapshot provider for the ultrasonic bench log
 * (APP_ULTRASONIC_BENCH_LOG). Stands in for Core/Src/app_ultrasonic.c so the
 * REAL app_comm.c "US," formatter can be driven with arbitrary snapshots. */
#ifndef ULTRASONIC_STUB_H
#define ULTRASONIC_STUB_H

#include "app_ultrasonic.h"

void UsStub_Set(const AppUltrasonicSnapshot *snapshot);
unsigned UsStub_GetCallCount(void);

#endif /* ULTRASONIC_STUB_H */
