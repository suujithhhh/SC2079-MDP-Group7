/* Host-buildable regression test for the strict parsing of USPARK,<e> /
 * USPARK,GET (ultrasonic parking protection control).
 *
 * Builds directly against the live Core/Src/vision_rx.c (no HAL deps),
 * stubbing every on_*() callback vision_rx.c declares as extern. Legacy
 * commands (RTGUARD, STOP, MOV, PID, ...) must dispatch exactly as before.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -o tests/parser/vision_rx_uspark_test.exe \
 *       tests/parser/vision_rx_uspark_test.c Core/Src/vision_rx.c
 *   ./tests/parser/vision_rx_uspark_test.exe
 */

#include "vision_rx.h"

#include <stdio.h>
#include <string.h>

typedef enum {
    EV_NONE = 0,
    EV_RTGUARD_SET,
    EV_RTGUARD_GET,
    EV_USPARK_SET,
    EV_USPARK_GET,
    EV_STOP,
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

static int rt_enable;
static int us_enable;

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
    rt_enable = -1;
    us_enable = -1;
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

void on_stop(void) { record(EV_STOP); }
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

void on_reverse_torque_guard_set(int enable)
{
    rt_enable = enable;
    record(EV_RTGUARD_SET);
}

void on_reverse_torque_guard_get(void) { record(EV_RTGUARD_GET); }
void on_us_park_set(int enable)
{
    us_enable = enable;
    record(EV_USPARK_SET);
}

void on_us_park_get(void) { record(EV_USPARK_GET); }
void on_ir_get(void) { record(EV_OTHER); }
void on_usd_get(void) { record(EV_OTHER); }

/* Stage A3.2: mandatory stubs -- vision_rx.c's object code references
 * these regardless of whether any RTGUARD-only case exercises them. */
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
void on_ackermann_get(void) { record(EV_OTHER); }/* --- assertions -------------------------------------------------- */

static void expect_rtguard_set(const char *line, int enable)
{
    feed_bytes(line);
    CHECK(event_seen == EV_RTGUARD_SET);
    CHECK(callback_count == 1U);
    CHECK(rt_enable == enable);
}

static void expect_rtguard_get(const char *line)
{
    feed_bytes(line);
    CHECK(event_seen == EV_RTGUARD_GET);
    CHECK(callback_count == 1U);
}

static void expect_uspark_set(const char *line, int enable)
{
    feed_bytes(line);
    CHECK(callback_count == 1U);
    CHECK(event_seen == EV_USPARK_SET);
    CHECK(us_enable == enable);
}

static void expect_uspark_get(const char *line)
{
    feed_bytes(line);
    CHECK(callback_count == 1U);
    CHECK(event_seen == EV_USPARK_GET);
}

static void expect_stop(const char *line)
{
    feed_bytes(line);
    CHECK(callback_count == 1U);
    CHECK(event_seen == EV_STOP);
}

/* Fed one byte at a time with the callback count checked after EVERY byte:
 * nothing may fire before the terminator. */
static void expect_fragmented_uspark_set(const char *line, int enable)
{
    size_t index;
    size_t length = strlen(line);

    reset_event();
    for (index = 0U; index < length; index++) {
        vision_rx_feed(&rx, (uint8_t)line[index]);
        if (index + 1U < length) {
            CHECK(callback_count == 0U);
        }
    }
    CHECK(callback_count == 1U);
    CHECK(event_seen == EV_USPARK_SET);
    CHECK(us_enable == enable);
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
    char overlong[64];
    size_t index;

    vision_rx_init(&rx);

    /* --- exact accepted forms ---------------------------------------- */
    expect_uspark_set("USPARK,0\n", 0);
    expect_uspark_set("USPARK,1\n", 1);
    expect_uspark_get("USPARK,GET\n");

    /* CRLF is accepted like every other command. */
    expect_uspark_set("USPARK,1\r\n", 1);
    expect_uspark_set("USPARK,0\r\n", 0);
    expect_uspark_get("USPARK,GET\r\n");

    /* Same leading-zero behaviour as RTGUARD (shared bounded parser). */
    expect_uspark_set("USPARK,01\n", 1);

    /* --- fragmented UART input: nothing fires early ------------------- */
    expect_fragmented_uspark_set("USPARK,1\n", 1);
    expect_fragmented_uspark_set("USPARK,0\n", 0);

    /* --- missing / empty field ---------------------------------------- */
    expect_bad("USPARK,\n");
    expect_bad("USPARK\n");

    /* --- extra fields / trailing garbage ------------------------------ */
    expect_bad("USPARK,1,\n");
    expect_bad("USPARK,1,0\n");
    expect_bad("USPARK,1,extra\n");
    expect_bad("USPARK,1x\n");
    expect_bad("USPARK,0x\n");
    expect_bad("USPARK,GET,\n");
    expect_bad("USPARK,GET,1\n");
    expect_bad("USPARK,GETX\n");
    expect_bad("USPARK,GET \n");

    /* --- malformed integers / out of range ---------------------------- */
    expect_bad("USPARK,abc\n");
    expect_bad("USPARK,-1\n");
    expect_bad("USPARK,+1\n");
    expect_bad("USPARK,1.0\n");
    expect_bad("USPARK,2\n");
    expect_bad("USPARK,10\n");
    expect_bad("USPARK,99\n");
    expect_bad("USPARK,0x1\n");
    expect_bad("USPARK,1\t\n");

    /* --- case and whitespace: strict, like every other command -------- */
    expect_bad("uspark,1\n");
    expect_bad("Uspark,1\n");
    expect_bad("USpark,GET\n");
    expect_bad("uspark,get\n");
    expect_bad("USPARK,get\n");
    expect_bad(" USPARK,1\n");
    expect_bad("USPARK ,1\n");
    expect_bad("USPARK, 1\n");
    expect_bad("USPARK,1 \n");
    expect_bad("USPARK;1\n");
    expect_bad("USPARK:1\n");
    expect_bad("USPARKX,1\n");
    expect_bad("XUSPARK,1\n");
    expect_bad("US,1\n");

    /* --- overlong frame: discarded to the newline, then recovers ------ */
    for (index = 0U; index < sizeof(overlong) - 1U; index++) {
        overlong[index] = 'A';
    }
    overlong[sizeof(overlong) - 1U] = '\0';
    {
        char frame[96];

        (void)snprintf(frame, sizeof(frame), "USPARK,1%s\n", overlong);
        expect_bad(frame);
    }
    expect_uspark_set("USPARK,1\n", 1);             /* recovered */

    /* --- legacy commands unchanged ------------------------------------ */
    expect_stop("STOP\n");
    expect_move("MOV,F,80\n");
    expect_pid_set("PID,S,45,0,0\n");
    expect_rtguard_set("RTGUARD,1\n", 1);
    expect_rtguard_get("RTGUARD,GET\n");
    expect_bad("RTGUARD,2\n");
    expect_bad("stop\n");

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
