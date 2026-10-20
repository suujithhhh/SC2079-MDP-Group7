/* vision_rx_stop_test.c -- host regression for the exact uppercase
 * "STOP" priority-stop command (Stage 2).
 *
 * Builds directly against the live Core/Src/vision_rx.c (no HAL deps),
 * stubbing every on_*() callback vision_rx.c declares as extern. The
 * parser only recognises the syntax; priority dispatch, duplicate
 * coalescing and the reply are app_comm.c's job (tests/comm/).
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -o tests/parser/vision_rx_stop_test.exe \
 *       tests/parser/vision_rx_stop_test.c Core/Src/vision_rx.c
 *   ./tests/parser/vision_rx_stop_test.exe
 */

#include "vision_rx.h"

#include <stdio.h>
#include <string.h>

typedef enum {
    EV_NONE = 0,
    EV_STOP,
    EV_STOPCTRL_SET,
    EV_STOPCTRL_GET,
    EV_MOVE,
    EV_BAD,
    EV_OTHER
} event_t;

#define LOG_MAX 32U

static vision_rx_t rx;
static event_t log_events[LOG_MAX];
static unsigned log_count;
static unsigned checks;
static unsigned failures;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        failures++; \
        printf("FAIL line %d: %s\n", __LINE__, #expr); \
    } \
} while (0)

static void reset_log(void)
{
    log_count = 0U;
}

static void record(event_t event)
{
    if (log_count < LOG_MAX) {
        log_events[log_count] = event;
    }
    log_count++;
}

static unsigned count_of(event_t event)
{
    unsigned i;
    unsigned n = 0U;

    for (i = 0U; (i < log_count) && (i < LOG_MAX); i++) {
        if (log_events[i] == event) {
            n++;
        }
    }
    return n;
}

static void feed_n(const char *data, size_t length)
{
    size_t index;

    for (index = 0U; index < length; index++) {
        vision_rx_feed(&rx, (uint8_t)data[index]);
    }
}

static void feed_bytes(const char *text)
{
    reset_log();
    feed_n(text, strlen(text));
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
    record(EV_OTHER);
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

void on_stop_control_set(float brakeLeadCounts, float brakeEntrySpeed, int enable)
{
    (void)brakeLeadCounts; (void)brakeEntrySpeed; (void)enable;
    record(EV_STOPCTRL_SET);
}

void on_stop_control_get(void) { record(EV_STOPCTRL_GET); }

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

static void expect_stop(const char *line)
{
    feed_bytes(line);
    CHECK(log_count == 1U);
    CHECK(count_of(EV_STOP) == 1U);
    CHECK(count_of(EV_BAD) == 0U);
}

/* Exactly one on_bad_frame and nothing else (in particular no on_stop). */
static void expect_bad(const char *line)
{
    feed_bytes(line);
    CHECK(log_count == 1U);
    CHECK(count_of(EV_BAD) == 1U);
    CHECK(count_of(EV_STOP) == 0U);
}

static void expect_bad_n(const char *data, size_t length)
{
    reset_log();
    feed_n(data, length);
    CHECK(log_count == 1U);
    CHECK(count_of(EV_BAD) == 1U);
    CHECK(count_of(EV_STOP) == 0U);
}

int main(void)
{
    vision_rx_init(&rx);

    /* --- Accepted: exact LF and CRLF forms ---------------------------- */
    expect_stop("STOP\n");
    expect_stop("STOP\r\n");

    /* --- Malformed variants ------------------------------------------ */
    expect_bad("STOP,\n");
    expect_bad("STOP,1\n");
    expect_bad("STOP,STOP\n");
    expect_bad("STOP,0\n");
    expect_bad("STOP \n");        /* trailing space */
    expect_bad(" STOP\n");        /* leading space */
    expect_bad("XSTOP\n");        /* prefix */
    expect_bad("STOPX\n");        /* suffix */
    expect_bad("STOPS\n");
    expect_bad("STOP!\n");
    expect_bad("STO\n");          /* truncated */
    expect_bad("ST\n");
    expect_bad("S\n");
    expect_bad("STOPSTOP\n");
    expect_bad("STOP\tX\n");      /* control char */
    expect_bad("STOP\t\n");
    expect_bad("ST\x01OP\n");     /* control char inside */
    expect_bad("STOP\x7f\n");     /* DEL */
    expect_bad("STOP\x80\n");     /* high byte */
    expect_bad("STOP\rX\n");      /* CR not followed by LF */
    expect_bad("STOP\r\r\n");     /* double CR */
    expect_bad_n("ST\0OP\n", 6U); /* embedded NUL */

    /* --- Case sensitivity ----------------------------------------------- */
    expect_bad("stop\n");
    expect_bad("Stop\n");
    expect_bad("sTOP\n");
    expect_bad("STOp\n");
    expect_bad("STOP,stop\n");

    /* --- STOPCTRL is a different command and is unaffected ----------- */
    feed_bytes("STOPCTRL,GET\n");
    CHECK(log_count == 1U);
    CHECK(count_of(EV_STOPCTRL_GET) == 1U);
    CHECK(count_of(EV_STOP) == 0U);

    feed_bytes("STOPCTRL,150,1700,1\n");
    CHECK(log_count == 1U);
    CHECK(count_of(EV_STOPCTRL_SET) == 1U);
    CHECK(count_of(EV_STOP) == 0U);

    expect_bad("STOPCTRL\n");
    expect_bad("STOPCTRL,\n");
    expect_bad("STOPCTRL,GET,\n");

    /* --- Adjacent commands ------------------------------------------- */
    feed_bytes("MOV,F,10\nSTOP\nMOV,B,5\n");
    CHECK(log_count == 3U);
    CHECK(log_events[0] == EV_MOVE);
    CHECK(log_events[1] == EV_STOP);
    CHECK(log_events[2] == EV_MOVE);

    feed_bytes("STOP\nMOV,F,10\n");
    CHECK(log_count == 2U);
    CHECK(log_events[0] == EV_STOP);
    CHECK(log_events[1] == EV_MOVE);

    feed_bytes("MOV,F,10\nSTOP\n");
    CHECK(log_count == 2U);
    CHECK(log_events[0] == EV_MOVE);
    CHECK(log_events[1] == EV_STOP);

    feed_bytes("XSTOP\nSTOP\n");
    CHECK(log_count == 2U);
    CHECK(log_events[0] == EV_BAD);
    CHECK(log_events[1] == EV_STOP);

    feed_bytes("STOP\nXSTOP\n");
    CHECK(log_count == 2U);
    CHECK(log_events[0] == EV_STOP);
    CHECK(log_events[1] == EV_BAD);

    feed_bytes("STOPCTRL,GET\nSTOP\n");
    CHECK(log_count == 2U);
    CHECK(log_events[0] == EV_STOPCTRL_GET);
    CHECK(log_events[1] == EV_STOP);

    /* --- Multiple STOP lines (parser reports each; coalescing is
     *     app_comm.c's job) ---------------------------------------------- */
    feed_bytes("STOP\nSTOP\n");
    CHECK(log_count == 2U);
    CHECK(count_of(EV_STOP) == 2U);

    feed_bytes("STOP\r\nSTOP\r\nSTOP\n");
    CHECK(log_count == 3U);
    CHECK(count_of(EV_STOP) == 3U);

    /* --- Blank-line behaviour ------------------------------------------ */
    feed_bytes("\n");
    CHECK(log_count == 0U);
    feed_bytes("\nSTOP\n");
    CHECK(log_count == 1U);
    CHECK(count_of(EV_STOP) == 1U);
    feed_bytes("STOP\n\nSTOP\n");
    CHECK(log_count == 2U);
    CHECK(count_of(EV_STOP) == 2U);

    /* --- Byte-split input ---------------------------------------------- */
    reset_log();
    feed_n("S", 1U);
    feed_n("T", 1U);
    CHECK(log_count == 0U);
    feed_n("O", 1U);
    feed_n("P", 1U);
    CHECK(log_count == 0U);
    feed_n("\n", 1U);
    CHECK(log_count == 1U);
    CHECK(count_of(EV_STOP) == 1U);

    reset_log();
    feed_n("STOP", 4U);
    CHECK(log_count == 0U);
    feed_n("\r", 1U);
    CHECK(log_count == 0U);
    feed_n("\n", 1U);
    CHECK(log_count == 1U);
    CHECK(count_of(EV_STOP) == 1U);

    reset_log();
    feed_n("STO", 3U);
    feed_n("P,", 2U);
    feed_n("1\n", 2U);
    CHECK(log_count == 1U);
    CHECK(count_of(EV_BAD) == 1U);
    CHECK(count_of(EV_STOP) == 0U);

    /* A partial line followed by a complete STOP line is one bad line
     * (the prefix belongs to the same line), never a STOP. */
    reset_log();
    feed_n("ST", 2U);
    feed_n("STOP\n", 5U);
    CHECK(log_count == 1U);
    CHECK(count_of(EV_BAD) == 1U);
    CHECK(count_of(EV_STOP) == 0U);

    /* --- Line-buffer boundaries and overflow recovery ----------------- */
    /* 31 characters (the maximum) starting with STOP: dispatched, bad. */
    expect_bad("STOPXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
    /* 32 characters: over the limit, discarded as overflow, one bad. */
    expect_bad("STOPXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
    /* A long junk line ending in STOP is a single bad line... */
    expect_bad("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAASTOP\n");
    /* ...and a clean STOP right after is accepted again. */
    expect_stop("STOP\n");
    /* Ten STOPs without a newline overflow; the newline resynchronises. */
    expect_bad("STOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOP\n");
    expect_stop("STOP\r\n");
    /* Overflow then the next line is not swallowed. */
    feed_bytes("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\nSTOP\n");
    CHECK(log_count == 2U);
    CHECK(log_events[0] == EV_BAD);
    CHECK(log_events[1] == EV_STOP);

    /* --- Existing commands still dispatch ---------------------------- */
    feed_bytes("MOV,F,80\n");
    CHECK(log_count == 1U);
    CHECK(log_events[0] == EV_MOVE);
    feed_bytes("MOV,STOP\n");
    CHECK(log_count == 1U);
    CHECK(log_events[0] == EV_BAD);

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
