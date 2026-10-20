/* Host-buildable regression test for TURNSPEED,<countsPerSec> /
 * TURNSPEED,GET parsing and strict rejection.
 *
 * Builds directly against the live Core/Src/vision_rx.c (no HAL deps),
 * stubbing every on_*() callback vision_rx.c declares as extern.
 * MOV/RTGUARD/PRESTEER/etc. coverage lives separately.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -o tests/parser/vision_rx_turnspeed_test.exe \
 *       tests/parser/vision_rx_turnspeed_test.c Core/Src/vision_rx.c
 *   ./tests/parser/vision_rx_turnspeed_test.exe
 */

#include "vision_rx.h"

#include <stdio.h>
#include <string.h>

typedef enum {
    EV_NONE = 0,
    EV_TURNSPEED_SET,
    EV_TURNSPEED_GET,
    EV_MOVE,
    EV_PID_SET,
    EV_BAD,
    EV_OTHER
} event_t;

static vision_rx_t rx;
static event_t event_seen;
static unsigned callback_count;
static unsigned checks;
static unsigned failures;

static uint32_t ts_rate;

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
    ts_rate = 0xFFFFFFFFU;
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
    (void)type; (void)amount;
    record(EV_MOVE);
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
    record(EV_PID_SET);
}

void on_pid_get(void) { record(EV_OTHER); }
void on_tune_motor(char target, int32_t counts) { (void)target; (void)counts; record(EV_OTHER); }
void on_tune_steering(char dir, uint16_t seconds) { (void)dir; (void)seconds; record(EV_OTHER); }

void on_wheel_sync_set(float gain, float maxDuty, int enable) { (void)gain; (void)maxDuty; (void)enable; record(EV_OTHER); }
void on_wheel_sync_get(void) { record(EV_OTHER); }

void on_heading_rate_set(float krate, float maxCorrection, int enable) { (void)krate; (void)maxCorrection; (void)enable; record(EV_OTHER); }
void on_heading_rate_get(void) { record(EV_OTHER); }

void on_reverse_torque_guard_set(int enable) { (void)enable; record(EV_OTHER); }
void on_reverse_torque_guard_get(void) { record(EV_OTHER); }

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

void on_turn_speed_set(uint32_t countsPerSec)
{
    ts_rate = countsPerSec;
    record(EV_TURNSPEED_SET);
}

void on_turn_speed_get(void) { record(EV_TURNSPEED_GET); }

void on_ackermann_set(float k, int enable) { (void)k; (void)enable; record(EV_OTHER); }
void on_ackermann_get(void) { record(EV_OTHER); }

/* --- assertions -------------------------------------------------- */

static void expect_turnspeed_set(const char *line, uint32_t countsPerSec)
{
    feed_bytes(line);
    CHECK(event_seen == EV_TURNSPEED_SET);
    CHECK(callback_count == 1U);
    CHECK(ts_rate == countsPerSec);
}

static void expect_turnspeed_get(const char *line)
{
    feed_bytes(line);
    CHECK(event_seen == EV_TURNSPEED_GET);
    CHECK(callback_count == 1U);
}

static void expect_bad(const char *line)
{
    feed_bytes(line);
    CHECK(event_seen == EV_BAD);
    CHECK(callback_count == 1U);
}

static void expect_move(const char *line)
{
    feed_bytes(line);
    CHECK(event_seen == EV_MOVE);
    CHECK(callback_count == 1U);
}

static void expect_pid_set(const char *line)
{
    feed_bytes(line);
    CHECK(event_seen == EV_PID_SET);
    CHECK(callback_count == 1U);
}

int main(void)
{
    vision_rx_init(&rx);

    /* --- Valid TURNSPEED,SET ---------------------------------------- */
    expect_turnspeed_set("TURNSPEED,1200\n", 1200U);
    expect_turnspeed_set("TURNSPEED,500\n", 500U);    /* lower bound */
    expect_turnspeed_set("TURNSPEED,2500\n", 2500U);  /* upper bound */
    expect_turnspeed_set("TURNSPEED,2000\n", 2000U);  /* default value, still a valid explicit SET */

    /* --- Valid TURNSPEED,GET ------------------------------------------- */
    expect_turnspeed_get("TURNSPEED,GET\n");

    /* --- Existing commands still dispatch (regression: TURNSPEED did
     * not disturb unrelated parsing) --------------------------------- */
    expect_move("MOV,F,80\n");
    expect_move("MOV,FL,90\n");
    expect_pid_set("PID,S,45,0,0\n");

    /* --- Invalid: missing / empty field ------------------------------ */
    expect_bad("TURNSPEED,\n");
    expect_bad("TURNSPEED\n");

    /* --- Invalid: trailing garbage ------------------------------------ */
    expect_bad("TURNSPEED,1200,\n");
    expect_bad("TURNSPEED,1200,extra\n");
    expect_bad("TURNSPEED,1200x\n");

    /* --- Invalid: non-numeric / malformed ------------------------------ */
    expect_bad("TURNSPEED,abc\n");
    expect_bad("TURNSPEED,-500\n");   /* no leading '-' accepted by the bounded-u32 parser */
    expect_bad("TURNSPEED,1200.5\n");
    expect_bad("TURNSPEED,+1200\n");

    /* --- Invalid: out of range ----------------------------------------- */
    expect_bad("TURNSPEED,499\n");    /* below 500 */
    expect_bad("TURNSPEED,2501\n");   /* above 2500 */
    expect_bad("TURNSPEED,0\n");
    expect_bad("TURNSPEED,99999\n");

    /* --- TURNSPEED,GET must be exact ------------------------------------ */
    expect_bad("TURNSPEED,GET,\n");
    expect_bad("TURNSPEED,GETX\n");

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
