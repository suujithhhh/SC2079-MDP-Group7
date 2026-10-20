/* See ultrasonic_stub.h. */
#include "ultrasonic_stub.h"

static AppUltrasonicSnapshot current;
static unsigned getCalls;

void UsStub_Set(const AppUltrasonicSnapshot *snapshot) { current = *snapshot; }
unsigned UsStub_GetCallCount(void) { return getCalls; }

void AppUltrasonic_GetSnapshot(AppUltrasonicSnapshot *snapshot)
{
    getCalls++;
    *snapshot = current;
}
