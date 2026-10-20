/* Host-test fake for FreeRTOS.h -- NOT a real FreeRTOS port.
 *
 * app_motion.c is not otherwise HAL/RTOS-coupled (see app_motor.h,
 * app_encoder.h, app_servo.h, app_imu.h, app_pid.h, all plain stdint and
 * stdbool), but it does include FreeRTOS.h and task.h for
 * taskENTER_CRITICAL()/taskEXIT_CRITICAL() (used by every AppMotion Set
 * and Get function to guard its statics) and TickType_t,
 * xTaskGetTickCount() and pdMS_TO_TICKS() (used only by
 * AppMotion_SetMovementDeadline() and AppMotion_DeadlineReached(),
 * neither reachable from the profile/SFF functions this test suite
 * exercises). This header supplies just enough of that surface to
 * compile and link app_motion.c on the host as a single-threaded test
 * binary -- critical sections are no-ops (no real concurrency here),
 * matching this project's established host-test pattern (tests/parser/
 * already builds vision_rx.c standalone the same way). Placed ahead of
 * Core/Inc on the include path (see the build command in
 * app_motion_profile_test.c's own header comment) so this fake is
 * picked up instead of the real Middlewares FreeRTOS headers. */
#ifndef FAKE_FREERTOS_H
#define FAKE_FREERTOS_H

#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL()  ((void)0)

#define pdMS_TO_TICKS(ms) (ms)

#endif /* FAKE_FREERTOS_H */
