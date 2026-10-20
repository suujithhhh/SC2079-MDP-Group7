/* Host-buildable regression test for the MOV,FIRL / MOV,FIRR parsing (Phase 5,
 * forward until the far edge of an obstacle seen by the left / right IR
 * sensor) and its strict rejection rules.
 *
 * Builds directly against the live Core/Src/vision_rx.c (no HAL deps),
 * stubbing every on_*() callback vision_rx.c declares as extern. The exact
 * lines MOV,FIRL,<1-999> and MOV,FIRR,<1-999> reach on_move() with the typed
 * VISION_MOVE_FORWARD_IR_LEFT / _RIGHT; every other MOV,FI... form, a missing,
 * zero, signed, fractional or out-of-range amount, any case variant and any
 * extra field is on_bad_frame(). The ordinary MOV forms must dispatch exactly
 * as before.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -o tests/parser/vision_rx_fir_test.exe \
 *       tests/parser/vision_rx_fir_test.c Core/Src/vision_rx.c
 *   ./tests/parser/vision_rx_fir_test.exe
 */

#include "vision_rx.h"

#include <stdio.h>
#include <string.h>

typedef enum {
    EV_NONE = 0,
    EV_RTGUARD_SET,
    EV_RTGUARD_GET,
    EV_IR_GET,
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
static VisionMoveType move_type;
static unsigned move_amount;

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
    move_type = (VisionMoveType)-1;
    move_amount = 0xFFFFFFFFU;
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
    move_type = type;
    move_amount = amount;
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
void on_ackermann_get(void) { record(EV_OTHER); }

/* --- assertions -------------------------------------------------- */

static void expect_event(const char *line, event_t event)
{
    feed_bytes(line);
    CHECK(event_seen == event);
    CHECK(callback_count == 1U);
}

static void expect_fir(const char *line, VisionMoveType type, unsigned amount)
{
    expect_event(line, EV_MOVE);
    CHECK(move_type == type);
    CHECK(move_amount == amount);
}

static void expect_bad(const char *line) { expect_event(line, EV_BAD); }

static void expect_move(const char *line, VisionMoveType type, unsigned amount)
{
    expect_event(line, EV_MOVE);
    CHECK(move_type == type);
    CHECK(move_amount == amount);
}

int main(void)
{
    char line[48];
    unsigned amount;

    vision_rx_init(&rx);

    /* --- The valid forms ---------------------------------------------- */
    expect_fir("MOV,FIRL,1\n", VISION_MOVE_FORWARD_IR_LEFT, 1U);
    expect_fir("MOV,FIRR,1\n", VISION_MOVE_FORWARD_IR_RIGHT, 1U);
    expect_fir("MOV,FIRL,80\n", VISION_MOVE_FORWARD_IR_LEFT, 80U);
    expect_fir("MOV,FIRR,150\n", VISION_MOVE_FORWARD_IR_RIGHT, 150U);
    expect_fir("MOV,FIRL,999\n", VISION_MOVE_FORWARD_IR_LEFT, 999U);
    expect_fir("MOV,FIRR,999\r\n", VISION_MOVE_FORWARD_IR_RIGHT, 999U);   /* CRLF accepted */
    expect_fir("MOV,FIRL,007\n", VISION_MOVE_FORWARD_IR_LEFT, 7U);         /* like MOV,F */

    /* every amount in 1..999, both sides */
    for (amount = 1U; amount <= 999U; amount++) {
        (void)snprintf(line, sizeof(line), "MOV,FIRL,%u\n", amount);
        feed_bytes(line);
        CHECK(event_seen == EV_MOVE);
        CHECK(callback_count == 1U);
        CHECK(move_type == VISION_MOVE_FORWARD_IR_LEFT);
        CHECK(move_amount == amount);
        (void)snprintf(line, sizeof(line), "MOV,FIRR,%u\n", amount);
        feed_bytes(line);
        CHECK(event_seen == EV_MOVE);
        CHECK(callback_count == 1U);
        CHECK(move_type == VISION_MOVE_FORWARD_IR_RIGHT);
        CHECK(move_amount == amount);
    }

    /* --- The amount: 1..999 only -------------------------------------- */
    expect_bad("MOV,FIRL,0\n");             /* a zero-length search has no meaning */
    expect_bad("MOV,FIRR,0\n");
    expect_bad("MOV,FIRL,00\n");
    expect_bad("MOV,FIRL,1000\n");
    expect_bad("MOV,FIRR,1000\n");
    expect_bad("MOV,FIRL,99999999999\n");
    expect_bad("MOV,FIRL,-1\n");
    expect_bad("MOV,FIRL,+5\n");
    expect_bad("MOV,FIRL,5.0\n");
    expect_bad("MOV,FIRL,5 \n");            /* trailing space */
    expect_bad("MOV,FIRL, 5\n");            /* leading space */
    expect_bad("MOV,FIRL,0x10\n");
    expect_bad("MOV,FIRL,5,\n");
    expect_bad("MOV,FIRL,5,5\n");           /* an extra field */
    expect_bad("MOV,FIRL,\n");              /* missing amount */
    expect_bad("MOV,FIRR,\n");
    expect_bad("MOV,FIRL\n");
    expect_bad("MOV,FIRR\n");

    /* --- The direction token is exact -------------------------------- */
    expect_bad("MOV,FIR,80\n");
    expect_bad("MOV,FIR,L,80\n");           /* the rejected four-field shape */
    expect_bad("MOV,FIR,R,80\n");
    expect_bad("MOV,FIRX,80\n");
    expect_bad("MOV,FIRLL,80\n");
    expect_bad("MOV,FIRLR,80\n");
    expect_bad("MOV,FIRB,80\n");
    expect_bad("MOV,FI,80\n");
    expect_bad("MOV,FIL,80\n");
    expect_bad("MOV,BIRL,80\n");
    expect_bad("MOV,IRL,80\n");
    expect_bad("MOV,firl,80\n");
    expect_bad("MOV,Firl,80\n");
    expect_bad("MOV,FIRl,80\n");
    expect_bad("mov,FIRL,80\n");
    expect_bad("Mov,FIRL,80\n");
    expect_bad(" MOV,FIRL,80\n");
    expect_bad("XMOV,FIRL,80\n");
    expect_bad("MOV;FIRL;80\n");

    /* --- Framing ----------------------------------------------------- */
    feed_bytes("MOV,FIRL,5\r");               /* lone CR, no LF yet */
    CHECK(callback_count == 0U);
    feed_bytes("MOV,FIRL,5\n");               /* CR then text marks the line bad */
    CHECK(event_seen == EV_BAD);
    CHECK(callback_count == 1U);
    expect_fir("MOV,FIRL,5\n", VISION_MOVE_FORWARD_IR_LEFT, 5U);   /* recovered */

    feed_bytes("MOV,FIR\x01L,5\n");           /* control character */
    CHECK(event_seen == EV_BAD);
    CHECK(callback_count == 1U);
    expect_fir("MOV,FIRR,5\n", VISION_MOVE_FORWARD_IR_RIGHT, 5U);

    /* byte at a time, split at every position of a valid line */
    {
        const char *valid = "MOV,FIRR,123\n";
        size_t split;

        for (split = 1U; split < strlen(valid); split++) {
            char first[24];
            char second[24];

            memcpy(first, valid, split);
            first[split] = '\0';
            (void)strcpy(second, valid + split);

            reset_event();
            {
                size_t k;

                for (k = 0U; k < strlen(first); k++) {
                    vision_rx_feed(&rx, (uint8_t)first[k]);
                }
                CHECK(callback_count == 0U);
                for (k = 0U; k < strlen(second); k++) {
                    vision_rx_feed(&rx, (uint8_t)second[k]);
                }
            }
            CHECK(callback_count == 1U);
            CHECK(event_seen == EV_MOVE);
            CHECK(move_type == VISION_MOVE_FORWARD_IR_RIGHT);
            CHECK(move_amount == 123U);
        }
    }

    /* a line longer than the 32-byte buffer is rejected, then the parser recovers */
    expect_bad("MOV,FIRL,00000000000000000000000000000005\n");
    expect_fir("MOV,FIRL,9\n", VISION_MOVE_FORWARD_IR_LEFT, 9U);

    /* two commands back to back */
    feed_bytes("MOV,FIRL,3\nMOV,FIRR,4\n");
    CHECK(callback_count == 2U);
    CHECK(move_type == VISION_MOVE_FORWARD_IR_RIGHT);
    CHECK(move_amount == 4U);

    /* --- The ordinary MOV forms are untouched ------------------------- */
    expect_move("MOV,F,80\n", VISION_MOVE_FORWARD, 80U);
    expect_move("MOV,F,0\n", VISION_MOVE_FORWARD, 0U);             /* MOV,F,0 is still valid */
    expect_move("MOV,B,12\n", VISION_MOVE_BACKWARD, 12U);
    expect_move("MOV,FL,90\n", VISION_MOVE_FORWARD_LEFT, 90U);
    expect_move("MOV,FR,45\n", VISION_MOVE_FORWARD_RIGHT, 45U);
    expect_move("MOV,BL,30\n", VISION_MOVE_BACKWARD_LEFT, 30U);
    expect_move("MOV,BR,360\n", VISION_MOVE_BACKWARD_RIGHT, 360U);
    expect_bad("MOV,L,90\n");
    expect_bad("MOV,R,90\n");
    expect_bad("MOV,FL,361\n");
    expect_bad("MOV,F,1000\n");
    expect_bad("MOV,F\n");
    expect_bad("MOV,FI,5\n");

    /* --- Neighbouring commands are untouched ------------------------- */
    expect_event("STOP\n", EV_STOP);
    expect_event("IR,GET\n", EV_IR_GET);
    expect_event("USPARK,GET\n", EV_USPARK_GET);
    expect_event("PID,S,45,0,0\n", EV_PID_SET);
    expect_bad("IRRAW,S:1\n");

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
