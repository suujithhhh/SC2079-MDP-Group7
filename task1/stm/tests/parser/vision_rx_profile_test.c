/* Host-buildable regression test for the motion-profiles branch's
 * PROFILE,PRECISION / PROFILE,STANDARD / PROFILE,GET parsing and strict
 * rejection.
 *
 * Builds directly against the live Core/Src/vision_rx.c (no HAL deps),
 * stubbing every on_*() callback vision_rx.c declares as extern.
 * MOV/WSYNC/HCTRL/RTGUARD/DPROF/SPCTRL/SPPROF/STOPCTRL/MOVCAL coverage
 * lives separately. Unlike every other SET command's own test file, this
 * suite has no numeric fields to bounds-test -- PROFILE,PRECISION and
 * PROFILE,STANDARD are exact literal matches, so "bad argument count"
 * here means trailing/extra tokens after the profile name, not an
 * out-of-range number.
 *
 * The idle-only (ERR,BUSY while moving) gate lives in app_comm.c's
 * on_motion_profile_set(), one layer above this parser -- exactly like
 * every other command's own busy-gating, it is not exercised by this
 * suite (vision_rx.c has no concept of motion state); see
 * app_comm.c's on_motion_profile_set() directly for that behavior.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -o tests/parser/vision_rx_profile_test.exe \
 *       tests/parser/vision_rx_profile_test.c Core/Src/vision_rx.c
 *   ./tests/parser/vision_rx_profile_test.exe
 */

#include "vision_rx.h"

#include <stdio.h>
#include <string.h>

typedef enum {
    EV_NONE = 0,
    EV_PROFILE_SET,
    EV_PROFILE_GET,
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

static VisionProfileType pf_profile;
static int                pf_profile_valid;

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
    pf_profile_valid = 0;
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

void on_motion_profile_set(VisionProfileType profile)
{
    pf_profile = profile;
    pf_profile_valid = 1;
    record(EV_PROFILE_SET);
}

void on_motion_profile_get(void) { record(EV_PROFILE_GET); }

void on_servo_feed_forward_set(char direction, float startSpeed, float fullSpeed, float maxUs, int enable) { (void)direction; (void)startSpeed; (void)fullSpeed; (void)maxUs; (void)enable; record(EV_OTHER); }
void on_servo_feed_forward_get(void) { record(EV_OTHER); }
void on_pre_steer_set(float durationMs, int enable) { (void)durationMs; (void)enable; record(EV_OTHER); }
void on_pre_steer_get(void) { record(EV_OTHER); }
void on_turn_brake_set(float lookaheadMs, int enable) { (void)lookaheadMs; (void)enable; record(EV_OTHER); }
void on_turn_brake_get(void) { record(EV_OTHER); }

void on_turn_speed_set(uint32_t countsPerSec) { (void)countsPerSec; record(EV_OTHER); }
void on_turn_speed_get(void) { record(EV_OTHER); }

void on_ackermann_set(float k, int enable) { (void)k; (void)enable; record(EV_OTHER); }
void on_ackermann_get(void) { record(EV_OTHER); }/* --- assertions -------------------------------------------------- */

static void expect_profile_set(const char *line, VisionProfileType profile)
{
    feed_bytes(line);
    CHECK(event_seen == EV_PROFILE_SET);
    CHECK(callback_count == 1U);
    CHECK(pf_profile_valid == 1);
    CHECK(pf_profile == profile);
}

static void expect_profile_get(const char *line)
{
    feed_bytes(line);
    CHECK(event_seen == EV_PROFILE_GET);
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

    /* --- Valid PROFILE,PRECISION / PROFILE,STANDARD ----------------- */
    expect_profile_set("PROFILE,PRECISION\n", VISION_PROFILE_PRECISION);
    expect_profile_set("PROFILE,STANDARD\n", VISION_PROFILE_STANDARD);

    /* --- Valid PROFILE,GET ------------------------------------------- */
    expect_profile_get("PROFILE,GET\n");

    /* --- Existing commands still dispatch ---------------------------- */
    expect_move("MOV,F,80\n");
    expect_pid_set("PID,S,45,0,0\n");

    /* --- Invalid: bad profile name ------------------------------------ */
    expect_bad("PROFILE,CUSTOM\n");      /* CUSTOM is report-only, never settable */
    expect_bad("PROFILE,BOGUS\n");
    expect_bad("PROFILE,precision\n");   /* case-sensitive */
    expect_bad("PROFILE,Standard\n");    /* case-sensitive */
    expect_bad("PROFILE,PRECISIONX\n");
    expect_bad("PROFILE,STANDARDX\n");
    expect_bad("PROFILE,\n");            /* empty name */
    expect_bad("PROFILE\n");             /* no comma at all */

    /* --- Invalid: bad argument count / trailing garbage --------------- */
    expect_bad("PROFILE,PRECISION,\n");        /* trailing comma */
    expect_bad("PROFILE,PRECISION,1\n");       /* extra field */
    expect_bad("PROFILE,STANDARD,extra\n");    /* extra field */
    expect_bad("PROFILE,PRECISIONSTANDARD\n"); /* concatenated, not two tokens */

    /* --- PROFILE,GET must be exact ------------------------------------ */
    expect_bad("PROFILE,GET,\n");
    expect_bad("PROFILE,GETX\n");
    expect_bad("PROFILE,get\n");         /* case-sensitive */

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
