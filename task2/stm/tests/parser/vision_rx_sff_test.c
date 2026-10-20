/* Host-buildable regression test for Stage SFF1's
 * SFF,<F|B>,<startSpeed>,<fullSpeed>,<maxUs>,<enable> / SFF,GET parsing
 * and strict rejection.
 *
 * Builds directly against the live Core/Src/vision_rx.c (no HAL deps),
 * stubbing every on_*() callback vision_rx.c declares as extern.
 * MOV/WSYNC/HCTRL/RTGUARD/DPROF/SPCTRL/SPPROF/STOPCTRL/MOVCAL/PROFILE
 * coverage lives separately. The fullSpeed>startSpeed cross-invariant
 * lives in app_comm.c's on_servo_feed_forward_set(), not in
 * vision_rx.c -- this suite covers only the independent per-field range
 * bounds vision_rx.c itself enforces.
 *
 * The idle-only (ERR,BUSY while moving) gate also lives in app_comm.c's
 * on_servo_feed_forward_set(), one layer above this parser -- exactly
 * like every other command's own busy-gating, it is not exercised by
 * this suite (vision_rx.c has no concept of motion state).
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -o tests/parser/vision_rx_sff_test.exe \
 *       tests/parser/vision_rx_sff_test.c Core/Src/vision_rx.c
 *   ./tests/parser/vision_rx_sff_test.exe
 */

#include "vision_rx.h"

#include <stdio.h>
#include <string.h>

typedef enum {
    EV_NONE = 0,
    EV_SFF_SET,
    EV_SFF_GET,
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

static char  sff_direction;
static float sff_start_speed;
static float sff_full_speed;
static float sff_max_us;
static int   sff_enable;

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
    sff_direction = '\0';
    sff_start_speed = -1.0f;
    sff_full_speed = -1.0f;
    sff_max_us = -9999.0f;
    sff_enable = -1;
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
void on_pre_steer_set(float durationMs, int enable) { (void)durationMs; (void)enable; record(EV_OTHER); }
void on_pre_steer_get(void) { record(EV_OTHER); }
void on_turn_brake_set(float lookaheadMs, int enable) { (void)lookaheadMs; (void)enable; record(EV_OTHER); }
void on_turn_brake_get(void) { record(EV_OTHER); }

void on_turn_speed_set(uint32_t countsPerSec) { (void)countsPerSec; record(EV_OTHER); }
void on_turn_speed_get(void) { record(EV_OTHER); }

void on_ackermann_set(float k, int enable) { (void)k; (void)enable; record(EV_OTHER); }
void on_ackermann_get(void) { record(EV_OTHER); }void on_servo_feed_forward_set(char direction, float startSpeed, float fullSpeed, float maxUs, int enable)
{
    sff_direction = direction;
    sff_start_speed = startSpeed;
    sff_full_speed = fullSpeed;
    sff_max_us = maxUs;
    sff_enable = enable;
    record(EV_SFF_SET);
}

void on_servo_feed_forward_get(void) { record(EV_SFF_GET); }

/* --- assertions -------------------------------------------------- */

static void expect_sff_set(const char *line, char direction, float startSpeed, float fullSpeed, float maxUs, int enable)
{
    feed_bytes(line);
    CHECK(event_seen == EV_SFF_SET);
    CHECK(callback_count == 1U);
    CHECK(sff_direction == direction);
    CHECK(sff_start_speed == startSpeed);
    CHECK(sff_full_speed == fullSpeed);
    CHECK(sff_max_us == maxUs);
    CHECK(sff_enable == enable);
}

static void expect_sff_get(const char *line)
{
    feed_bytes(line);
    CHECK(event_seen == EV_SFF_GET);
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

    /* --- Valid SFF,F,SET / SFF,B,SET --------------------------------- */
    expect_sff_set("SFF,F,1000,4000,-15,0\n", 'F', 1000.0f, 4000.0f, -15.0f, 0);
    expect_sff_set("SFF,F,1000,4000,-15,1\n", 'F', 1000.0f, 4000.0f, -15.0f, 1);
    expect_sff_set("SFF,B,2000,5000,20,1\n", 'B', 2000.0f, 5000.0f, 20.0f, 1);
    expect_sff_set("SFF,F,0,8000,-100,1\n", 'F', 0.0f, 8000.0f, -100.0f, 1);   /* startSpeed/maxUs lower bounds, fullSpeed upper */
    expect_sff_set("SFF,B,8000,8000,100,1\n", 'B', 8000.0f, 8000.0f, 100.0f, 1); /* startSpeed/fullSpeed upper, maxUs upper -- fullSpeed==startSpeed IS parser-valid (cross-check lives in app_comm.c) */
    expect_sff_set("SFF,F,1000.0,4000.0,-15.0,1\n", 'F', 1000.0f, 4000.0f, -15.0f, 1);

    /* --- Valid SFF,GET ----------------------------------------------- */
    expect_sff_get("SFF,GET\n");

    /* --- Existing commands still dispatch ---------------------------- */
    expect_move("MOV,F,80\n");
    expect_pid_set("PID,S,45,0,0\n");

    /* --- Invalid: bad direction letter -------------------------------- */
    expect_bad("SFF,X,1000,4000,-15,1\n");
    expect_bad("SFF,f,1000,4000,-15,1\n");   /* case-sensitive */
    expect_bad("SFF,,1000,4000,-15,1\n");
    expect_bad("SFF,FF,1000,4000,-15,1\n");  /* direction token must be exactly one char */

    /* --- Invalid: missing / empty fields ------------------------------ */
    expect_bad("SFF,F,1000,4000,-15\n");     /* missing enable */
    expect_bad("SFF,F,1000,4000\n");         /* missing maxUs + enable */
    expect_bad("SFF,F,1000\n");              /* missing fullSpeed + maxUs + enable */
    expect_bad("SFF,F,\n");
    expect_bad("SFF,F\n");
    expect_bad("SFF,\n");
    expect_bad("SFF\n");
    expect_bad("SFF,F,,4000,-15,1\n");       /* empty startSpeed */
    expect_bad("SFF,F,1000,,-15,1\n");       /* empty fullSpeed */
    expect_bad("SFF,F,1000,4000,,1\n");      /* empty maxUs */
    expect_bad("SFF,F,1000,4000,-15,\n");    /* empty enable */

    /* --- Invalid: trailing garbage -------------------------------- */
    expect_bad("SFF,F,1000,4000,-15,1,\n");       /* trailing comma */
    expect_bad("SFF,F,1000,4000,-15,1,extra\n");  /* extra field */
    expect_bad("SFF,F,1000,4000,-15,1x\n");       /* junk after enable */
    expect_bad("SFF,F,1000,4000,-15x,1\n");       /* junk after maxUs */

    /* --- Invalid: non-numeric ----------------------------------------- */
    expect_bad("SFF,F,abc,4000,-15,1\n");
    expect_bad("SFF,F,1000,def,-15,1\n");
    expect_bad("SFF,F,1000,4000,ghi,1\n");
    expect_bad("SFF,F,1000,4000,-15,jkl\n");

    /* --- Invalid: out of range (both directions of all three bounds) - */
    expect_bad("SFF,F,-0.1,4000,-15,1\n");    /* startSpeed below 0 */
    expect_bad("SFF,F,8000.1,4000,-15,1\n");  /* startSpeed above 8000 */
    expect_bad("SFF,F,1000,-0.1,-15,1\n");    /* fullSpeed below 0 */
    expect_bad("SFF,F,1000,8000.1,-15,1\n");  /* fullSpeed above 8000 */
    expect_bad("SFF,F,1000,4000,-100.1,1\n"); /* maxUs below -100 */
    expect_bad("SFF,F,1000,4000,100.1,1\n");  /* maxUs above 100 */
    expect_bad("SFF,F,1000,4000,-15,2\n");    /* enable not 0/1 */
    expect_bad("SFF,F,1000,4000,-15,-1\n");   /* enable not 0/1 */

    /* --- SFF,GET must be exact ---------------------------------------- */
    expect_bad("SFF,GET,\n");
    expect_bad("SFF,GETX\n");
    expect_bad("SFF,get\n");   /* case-sensitive */

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
