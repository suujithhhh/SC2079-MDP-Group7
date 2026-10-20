/* Host-buildable regression test for PRE_STEER's
 * PRESTEER,<durationMs>,<enable> / PRESTEER,GET parsing and strict
 * rejection.
 *
 * Builds directly against the live Core/Src/vision_rx.c (no HAL deps),
 * stubbing every on_*() callback vision_rx.c declares as extern.
 * MOV/STOPCTRL/etc. coverage lives separately.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -o tests/parser/vision_rx_presteer_test.exe \
 *       tests/parser/vision_rx_presteer_test.c Core/Src/vision_rx.c
 *   ./tests/parser/vision_rx_presteer_test.exe
 */

#include "vision_rx.h"

#include <stdio.h>
#include <string.h>

typedef enum {
    EV_NONE = 0,
    EV_PRESTEER_SET,
    EV_PRESTEER_GET,
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

static float ps_duration_ms;
static int   ps_enable;

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
    ps_duration_ms = -1.0f;
    ps_enable = -1;
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

void on_stop(void) { record(EV_OTHER); }
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
void on_us_park_set(int enable) { (void)enable; record(EV_OTHER); }
void on_us_park_get(void) { record(EV_OTHER); }
void on_ir_get(void) { record(EV_OTHER); }
void on_usd_get(void) { record(EV_OTHER); }

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

void on_pre_steer_set(float durationMs, int enable)
{
    ps_duration_ms = durationMs;
    ps_enable = enable;
    record(EV_PRESTEER_SET);
}

void on_pre_steer_get(void) { record(EV_PRESTEER_GET); }

void on_turn_speed_set(uint32_t countsPerSec) { (void)countsPerSec; record(EV_OTHER); }
void on_turn_speed_get(void) { record(EV_OTHER); }

void on_ackermann_set(float k, int enable) { (void)k; (void)enable; record(EV_OTHER); }
void on_ackermann_get(void) { record(EV_OTHER); }

void on_turn_brake_set(float lookaheadMs, int enable) { (void)lookaheadMs; (void)enable; record(EV_OTHER); }
void on_turn_brake_get(void) { record(EV_OTHER); }
/* --- assertions -------------------------------------------------- */

static void expect_presteer_set(const char *line, float durationMs, int enable)
{
    feed_bytes(line);
    CHECK(event_seen == EV_PRESTEER_SET);
    CHECK(callback_count == 1U);
    CHECK(ps_duration_ms == durationMs);
    CHECK(ps_enable == enable);
}

static void expect_presteer_get(const char *line)
{
    feed_bytes(line);
    CHECK(event_seen == EV_PRESTEER_GET);
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

    /* --- Valid PRESTEER,SET -------------------------------------- */
    expect_presteer_set("PRESTEER,150,1\n", 150.0f, 1);
    expect_presteer_set("PRESTEER,150,0\n", 150.0f, 0);
    expect_presteer_set("PRESTEER,0,1\n", 0.0f, 1);        /* lower bound, and 0 is a VALID value, distinct from disabled */
    expect_presteer_set("PRESTEER,1000,1\n", 1000.0f, 1);  /* upper bound */
    expect_presteer_set("PRESTEER,200.5,1\n", 200.5f, 1);

    /* --- Valid PRESTEER,GET ---------------------------------------- */
    expect_presteer_get("PRESTEER,GET\n");

    /* --- Existing commands still dispatch (regression: PRESTEER did
     * not disturb unrelated parsing) --------------------------------- */
    expect_move("MOV,F,80\n");
    expect_move("MOV,FL,90\n");
    expect_pid_set("PID,S,45,0,0\n");

    /* --- Invalid: missing / empty fields --------------------------- */
    expect_bad("PRESTEER,150\n");     /* missing enable */
    expect_bad("PRESTEER,\n");        /* nothing */
    expect_bad("PRESTEER\n");         /* no comma at all */
    expect_bad("PRESTEER,,1\n");      /* empty durationMs */
    expect_bad("PRESTEER,150,\n");    /* empty enable */

    /* --- Invalid: trailing garbage --------------------------------- */
    expect_bad("PRESTEER,150,1,\n");       /* trailing comma */
    expect_bad("PRESTEER,150,1,extra\n");  /* extra field */
    expect_bad("PRESTEER,150,1x\n");       /* junk after enable */
    expect_bad("PRESTEER,150x,1\n");       /* junk after durationMs */

    /* --- Invalid: non-numeric --------------------------------------- */
    expect_bad("PRESTEER,abc,1\n");
    expect_bad("PRESTEER,150,def\n");

    /* --- Invalid: out of range --------------------------------------- */
    expect_bad("PRESTEER,-0.1,1\n");     /* durationMs below 0 */
    expect_bad("PRESTEER,1000.1,1\n");   /* durationMs above 1000 */
    expect_bad("PRESTEER,150,2\n");      /* enable not 0/1 */
    expect_bad("PRESTEER,150,-1\n");     /* enable not 0/1 */

    /* --- PRESTEER,GET must be exact ----------------------------------- */
    expect_bad("PRESTEER,GET,\n");
    expect_bad("PRESTEER,GETX\n");

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
