/* Host-test fake for task.h -- see FreeRTOS.h in this same directory for
 * why this exists and what it deliberately does not implement. */
#ifndef FAKE_TASK_H
#define FAKE_TASK_H

#include <stdint.h>

typedef uint32_t TickType_t;

/* Settable via Fake_SetTickCount() (tests/motion/fakes/test_fakes.h),
 * defined in hw_stubs.c -- defaults to 0, same as the always-0 inline
 * this replaces, so the existing app_motion_profile_test.c (which never
 * calls Fake_SetTickCount()) is unaffected. Used by the new
 * app_motion_telemetry_test.c to simulate elapsed time for the
 * unified movement telemetry's T:/elapsedMs fields. */
TickType_t xTaskGetTickCount(void);

#endif /* FAKE_TASK_H */
