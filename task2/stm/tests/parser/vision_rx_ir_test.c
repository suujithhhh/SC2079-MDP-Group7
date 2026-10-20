/* Host-buildable regression test for the IR,GET parsing (Phase 3 IR
 * telemetry request) and its strict rejection rules.
 *
 * Builds directly against the live Core/Src/vision_rx.c (no HAL deps),
 * stubbing every on_*() callback vision_rx.c declares as extern. Only the
 * exact line "IR,GET" reaches on_ir_get(); every other "IR..." line (and the
 * bench-only IRRAW line) is an unknown command (on_bad_frame). The same for
 * USD,GET (front ultrasonic): only that exact line reaches on_usd_get(). Existing
 * commands must dispatch exactly as before.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -o tests/parser/vision_rx_ir_test.exe \
 *       tests/parser/vision_rx_ir_test.c Core/Src/vision_rx.c
 *   ./tests/parser/vision_rx_ir_test.exe
 */

#include "vision_rx.h"

#include <stdio.h>
#include <string.h>

typedef enum {
    EV_NONE = 0,
    EV_RTGUARD_SET,
    EV_RTGUARD_GET,
    EV_IR_GET,
    EV_USD_GET,
    EV_STOP,
    EV_USPARK_GET,
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
void on_us_park_set(int enable) { (void)enable; record(EV_OTHER); }
void on_us_park_get(void) { record(EV_USPARK_GET); }
void on_ir_get(void) { record(EV_IR_GET); }
void on_usd_get(void) { record(EV_USD_GET); }

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
void on_ackermann_get(void) { record(EV_OTHER); }

/* --- assertions -------------------------------------------------- */

static void expect_event(const char *line, event_t event)
{
    feed_bytes(line);
    CHECK(event_seen == event);
    CHECK(callback_count == 1U);
}

static void expect_ir_get(const char *line) { expect_event(line, EV_IR_GET); }
static void expect_usd_get(const char *line) { expect_event(line, EV_USD_GET); }
static void expect_bad(const char *line) { expect_event(line, EV_BAD); }

int main(void)
{
    vision_rx_init(&rx);

    /* --- The one valid form ------------------------------------------ */
    expect_ir_get("IR,GET\n");
    expect_ir_get("IR,GET\r\n");             /* CRLF accepted like every command */

    /* A second request on the same parser instance still works. */
    expect_ir_get("IR,GET\n");

    /* --- Exact match only: every variant is an unknown command -------- */
    expect_bad("ir,get\n");
    expect_bad("Ir,Get\n");
    expect_bad("IR,get\n");
    expect_bad("IR,GET,\n");
    expect_bad("IR,GET,1\n");
    expect_bad("IR,GETX\n");
    expect_bad("IR,GE\n");
    expect_bad("IR,GET \n");                 /* trailing space */
    expect_bad(" IR,GET\n");                 /* leading space */
    expect_bad("IR,\n");
    expect_bad("IR\n");
    expect_bad("IRGET\n");
    expect_bad("XIR,GET\n");
    expect_bad("IR,SET\n");
    expect_bad("IR,1\n");
    expect_bad("IR,L\n");

    /* --- The bench-only IRRAW line is never a command ----------------- */
    expect_bad("IRRAW\n");
    expect_bad("IRRAW,S:1,L:1,R:1,F:00\n");
    expect_bad("IRRAW,GET\n");

    /* --- Malformed framing ------------------------------------------- */
    feed_bytes("IR,GET\r");                   /* lone CR, no LF yet */
    CHECK(callback_count == 0U);
    feed_bytes("IR,GET\n");                   /* CR then text is rejected first... */
    /* ...the earlier lone CR followed by 'I' marks the line overflowed. */
    CHECK(event_seen == EV_BAD);
    CHECK(callback_count == 1U);
    expect_ir_get("IR,GET\n");                /* parser recovered */

    feed_bytes("IR,\x01GET\n");              /* control character */
    CHECK(event_seen == EV_BAD);
    CHECK(callback_count == 1U);
    expect_ir_get("IR,GET\n");

    /* --- USD,GET (front ultrasonic, PROTOCOL.md 4.8): exact match only - */
    expect_usd_get("USD,GET\n");
    expect_usd_get("USD,GET\r\n");
    expect_ir_get("IR,GET\n");               /* the two never mix */
    expect_usd_get("USD,GET\n");
    expect_bad("US,GET\n");                  /* rejected by the frozen spelling */
    expect_bad("usd,get\n");
    expect_bad("USD,get\n");
    expect_bad("USD,GET,\n");
    expect_bad("USD,GET,1\n");
    expect_bad("USD,GETX\n");
    expect_bad("USD,GET \n");
    expect_bad(" USD,GET\n");
    expect_bad("USD,\n");
    expect_bad("USD\n");
    expect_bad("USDGET\n");
    expect_bad("USD,SET\n");
    expect_bad("US,ST:IDLE\n");              /* a bench line echoed back is not a command */

    /* --- Neighbouring commands are untouched ------------------------- */
    expect_event("STOP\n", EV_STOP);
    expect_event("USPARK,GET\n", EV_USPARK_GET);
    expect_event("RTGUARD,GET\n", EV_RTGUARD_GET);
    expect_event("MOV,F,80\n", EV_MOVE);
    expect_event("PID,S,45,0,0\n", EV_PID_SET);

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
