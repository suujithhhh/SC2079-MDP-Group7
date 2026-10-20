/* Host-buildable regression test for the MOV,<dir>,<amt> parsing and
 * typed dispatch added for the F/B/FL/FR/BL/BR protocol.
 *
 * Builds directly against the live Core/Src/vision_rx.c (no HAL deps),
 * stubbing every on_*() callback vision_rx.c declares as extern.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -o tests/parser/vision_rx_move_test.exe \
 *       tests/parser/vision_rx_move_test.c Core/Src/vision_rx.c
 *   ./tests/parser/vision_rx_move_test.exe
 */

#include "vision_rx.h"

#include <stdio.h>
#include <string.h>

typedef enum {
    EV_NONE = 0,
    EV_MOVE,
    EV_BAD,
    EV_OTHER
} event_t;

static vision_rx_t rx;
static event_t event_seen;
static unsigned callback_count;
static unsigned checks;
static unsigned failures;

static VisionMoveType move_type;
static uint16_t move_amount;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        failures++; \
        printf("FAIL line %d: %s\n", __LINE__, #expr); \
    } \
} while (0)

static void reset_event(void)
{
    event_seen = EV_NONE;
    callback_count = 0U;
    move_amount = 0U;
}

static void record(event_t event)
{
    event_seen = event;
    callback_count++;
}

static void feed_bytes(const char *text)
{
    size_t index;
    size_t length = strlen(text);

    reset_event();
    for (index = 0U; index < length; index++) {
        vision_rx_feed(&rx, (uint8_t)text[index]);
    }
}

/* --- vision_rx.h callback stubs ----------------------------------- */

void on_target(uint8_t id, float conf, uint16_t cx, uint16_t cy)
{
    (void)id; (void)conf; (void)cx; (void)cy;
    record(EV_OTHER);
}

void on_move(VisionMoveType type, uint16_t amount)
{
    record(EV_MOVE);
    move_type = type;
    move_amount = amount;
}

void on_encoder_reset(void) { record(EV_OTHER); }
void on_encoder_get(void) { record(EV_OTHER); }
void on_pwm_test(uint8_t duty) { (void)duty; record(EV_OTHER); }
void on_speed_test(uint8_t duty) { (void)duty; record(EV_OTHER); }
void on_imu_get(void) { record(EV_OTHER); }
void on_imu_zero(void) { record(EV_OTHER); }
void on_imu_flip(void) { record(EV_OTHER); }
void on_heading_test(char dir, uint16_t seconds) { (void)dir; (void)seconds; record(EV_OTHER); }
void on_heading_raw_test(uint16_t seconds) { (void)seconds; record(EV_OTHER); }
void on_servo_set(uint16_t pulse_us) { (void)pulse_us; record(EV_OTHER); }
void on_servo_get(void) { record(EV_OTHER); }
void on_no_target(void) { record(EV_OTHER); }
void on_vision_ready(void) { record(EV_OTHER); }
void on_bad_frame(const char *line) { (void)line; record(EV_BAD); }
void on_pid_set(char target, float kp, float ki, float kd)
{
    (void)target; (void)kp; (void)ki; (void)kd;
    record(EV_OTHER);
}
void on_pid_get(void) { record(EV_OTHER); }
void on_tune_motor(char target, int32_t counts) { (void)target; (void)counts; record(EV_OTHER); }
void on_tune_steering(char dir, uint16_t seconds) { (void)dir; (void)seconds; record(EV_OTHER); }

/* Stage A1: mandatory stubs -- vision_rx.c's object code references these
 * regardless of whether any MOV-only case exercises them. Real WSYNC
 * coverage lives in vision_rx_wsync_test.c. */
void on_wheel_sync_set(float gain, float maxDuty, int enable) { (void)gain; (void)maxDuty; (void)enable; record(EV_OTHER); }
void on_wheel_sync_get(void) { record(EV_OTHER); }

/* Stage A2: mandatory stubs -- vision_rx.c's object code references these
 * regardless of whether any MOV-only case exercises them. Real HCTRL
 * coverage lives in vision_rx_hctrl_test.c. */
void on_heading_rate_set(float krate, float maxCorrection, int enable) { (void)krate; (void)maxCorrection; (void)enable; record(EV_OTHER); }
void on_heading_rate_get(void) { record(EV_OTHER); }

/* Stage A3.1: mandatory stubs -- vision_rx.c's object code references
 * these regardless of whether any MOV-only case exercises them. Real
 * RTGUARD coverage lives in vision_rx_rtguard_test.c. */
void on_reverse_torque_guard_set(int enable) { (void)enable; record(EV_OTHER); }
void on_reverse_torque_guard_get(void) { record(EV_OTHER); }

/* Stage A3.2: mandatory stubs -- vision_rx.c's object code references
 * these regardless of whether any MOV-only case exercises them. Real
 * DPROF coverage lives in vision_rx_dprof_test.c. */
void on_decel_profile_set(float decelCounts, float approachDuty, int enable) { (void)decelCounts; (void)approachDuty; (void)enable; record(EV_OTHER); }
void on_decel_profile_get(void) { record(EV_OTHER); }

void on_speed_control_set(float targetSpeed, float kp, float ki, int enable) { (void)targetSpeed; (void)kp; (void)ki; (void)enable; record(EV_OTHER); }
void on_speed_control_get(void) { record(EV_OTHER); }

void on_speed_profile_set(float decelStartCounts, float approachSpeed, int enable) { (void)decelStartCounts; (void)approachSpeed; (void)enable; record(EV_OTHER); }
void on_speed_profile_get(void) { record(EV_OTHER); }

void on_stop_control_set(float brakeLeadCounts, float brakeEntrySpeed, int enable) { (void)brakeLeadCounts; (void)brakeEntrySpeed; (void)enable; record(EV_OTHER); }
void on_stop_control_get(void) { record(EV_OTHER); }

void on_distance_cal_set(float slopeCountsPerCm, float offsetCounts, int enable) { (void)slopeCountsPerCm; (void)offsetCounts; (void)enable; record(EV_OTHER); }
void on_distance_cal_get(void) { record(EV_OTHER); }

void on_motion_profile_set(VisionProfileType profile) { (void)profile; record(EV_OTHER); }
void on_motion_profile_get(void) { record(EV_OTHER); }

void on_servo_feed_forward_set(char direction, float startSpeed, float fullSpeed, float maxUs, int enable) { (void)direction; (void)startSpeed; (void)fullSpeed; (void)maxUs; (void)enable; record(EV_OTHER); }
void on_servo_feed_forward_get(void) { record(EV_OTHER); }
void on_pre_steer_set(float durationMs, int enable) { (void)durationMs; (void)enable; record(EV_OTHER); }
void on_pre_steer_get(void) { record(EV_OTHER); }
void on_turn_brake_set(float lookaheadMs, int enable) { (void)lookaheadMs; (void)enable; record(EV_OTHER); }
void on_turn_brake_get(void) { record(EV_OTHER); }

void on_turn_speed_set(uint32_t countsPerSec) { (void)countsPerSec; record(EV_OTHER); }
void on_turn_speed_get(void) { record(EV_OTHER); }

void on_ackermann_set(float k, int enable) { (void)k; (void)enable; record(EV_OTHER); }
void on_ackermann_get(void) { record(EV_OTHER); }/* --- assertions ---------------------------------------------------- */

static void expect_move(const char *line, VisionMoveType type, uint16_t amount)
{
    feed_bytes(line);
    CHECK(event_seen == EV_MOVE);
    CHECK(callback_count == 1U);
    CHECK(move_type == type);
    CHECK(move_amount == amount);
}

static void expect_bad(const char *line)
{
    feed_bytes(line);
    CHECK(event_seen == EV_BAD);
    CHECK(callback_count == 1U);
}

int main(void)
{
    vision_rx_init(&rx);

    /* Straight motion: unchanged F/B behaviour. */
    expect_move("MOV,F,150\n", VISION_MOVE_FORWARD, 150U);
    expect_move("MOV,B,7\n", VISION_MOVE_BACKWARD, 7U);
    expect_move("MOV,F,0\n", VISION_MOVE_FORWARD, 0U);
    expect_move("MOV,F,999\n", VISION_MOVE_FORWARD, 999U);

    /* New turn tokens: FL/FR/BL/BR. */
    expect_move("MOV,FL,45\n", VISION_MOVE_FORWARD_LEFT, 45U);
    expect_move("MOV,FR,90\n", VISION_MOVE_FORWARD_RIGHT, 90U);
    expect_move("MOV,BL,30\n", VISION_MOVE_BACKWARD_LEFT, 30U);
    expect_move("MOV,BR,180\n", VISION_MOVE_BACKWARD_RIGHT, 180U);
    expect_move("MOV,FL,0\n", VISION_MOVE_FORWARD_LEFT, 0U);
    expect_move("MOV,BR,360\n", VISION_MOVE_BACKWARD_RIGHT, 360U);

    /* Bare L/R must now be rejected -- no longer part of the protocol. */
    expect_bad("MOV,L,45\n");
    expect_bad("MOV,R,90\n");

    /* Bounds: turn degrees > 360 rejected, distance > 999 rejected. */
    expect_bad("MOV,FL,361\n");
    expect_bad("MOV,BR,361\n");
    expect_bad("MOV,F,1000\n");
    expect_bad("MOV,B,1000\n");

    /* Malformed direction tokens. */
    expect_bad("MOV,FX,10\n");
    expect_bad("MOV,X,10\n");
    expect_bad("MOV,FLR,10\n");
    expect_bad("MOV,,10\n");
    expect_bad("MOV,F,\n");
    expect_bad("MOV,FL,\n");
    expect_bad("MOV,F\n");
    expect_bad("MOV,FL\n");

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
