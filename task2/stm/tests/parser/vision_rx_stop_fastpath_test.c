/* Host-buildable regression test for the exact-STOP FAST PATH in
 * vision_rx_feed() (Core/Src/vision_rx.c).
 *
 * The exact frame "STOP" is recognised in vision_rx_feed(), as soon as a
 * complete, LF-terminated, overflow-free line exists, instead of inside the
 * large vision_rx_dispatch() frame (about 360 bytes at -O0 on CommTask's 1 KB
 * stack). This test proves the fast path changed NOTHING observable:
 *   - exact STOP / STOP\n / STOP\r\n are accepted, the callback fires exactly
 *     once and only when the terminating LF arrives;
 *   - one byte at a time and every possible fragmentation point;
 *   - repeated STOPs; STOP followed/preceded by other commands in one stream;
 *   - every near-miss (case, spaces, extra fields, prefixes/suffixes, control
 *     characters, bare/doubled CR) is rejected exactly as before, through the
 *     existing on_bad_frame() behaviour, and never calls on_stop();
 *   - an over-long frame, including one that STARTS with STOP, recovers and a
 *     following valid STOP is accepted;
 *   - STOPCTRL,... (a different command sharing the prefix) is untouched;
 *   - no duplicate parsing: a STOP line produces exactly one callback of any
 *     kind, and the parser state is reset for the next line.
 * A runtime test cannot see stack-frame size; that is checked separately by
 * tests/stack/check_commtask_stack.sh against the linked ELF.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -o tests/parser/vision_rx_stop_fastpath_test.exe \
 *       tests/parser/vision_rx_stop_fastpath_test.c Core/Src/vision_rx.c
 *   ./tests/parser/vision_rx_stop_fastpath_test.exe
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
    EV_OTHER,
    EV_STOPCTRL_SET
} event_t;

#define LOG_MAX 64U

static vision_rx_t rx;
static event_t event_seen;
static event_t log_events[LOG_MAX];
static unsigned log_len;
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
    log_len = 0U;
    rt_enable = -1;
    us_enable = -1;
}

static void record(event_t event)
{
    event_seen = event;
    callback_count++;

    if (log_len < LOG_MAX) {
        log_events[log_len++] = event;
    }
}

static void feed_bytes(const char *text)
{
    size_t index;
    size_t length = strlen(text);

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

void on_stop_control_set(float brakeLeadCounts, float brakeEntrySpeed, int enable)
{
    (void)brakeLeadCounts; (void)brakeEntrySpeed; (void)enable;
    record(EV_STOPCTRL_SET);
}
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

/* --- helpers ----------------------------------------------------------- */

/* A line that must be accepted as STOP: exactly one callback, on_stop. */
static void expect_stop_line(const char *line)
{
    reset_event();
    feed_bytes(line);
    CHECK(callback_count == 1U);
    CHECK(event_seen == EV_STOP);
}

/* A near-miss: rejected through on_bad_frame() exactly once, never on_stop. */
static void expect_rejected(const char *line)
{
    reset_event();
    feed_bytes(line);
    CHECK(callback_count == 1U);
    CHECK(event_seen == EV_BAD);
}

/* The callback must fire on the terminating LF and not one byte earlier. */
static void expect_fires_on_lf_only(const char *line)
{
    size_t index;
    size_t length = strlen(line);

    reset_event();
    for (index = 0U; index < length; index++) {
        vision_rx_feed(&rx, (uint8_t)line[index]);
        if ((index + 1U) < length) {
            CHECK(callback_count == 0U);
        }
    }
    CHECK(callback_count == 1U);
    CHECK(event_seen == EV_STOP);
}

/* Split a frame at every position: first part, then the rest. */
static void expect_every_split(const char *line)
{
    size_t length = strlen(line);
    size_t split;
    char first[48];

    for (split = 1U; split < length; split++) {
        memcpy(first, line, split);
        first[split] = '\0';

        reset_event();
        feed_bytes(first);
        CHECK(callback_count == 0U);
        feed_bytes(line + split);
        CHECK(callback_count == 1U);
        CHECK(event_seen == EV_STOP);
    }
}

int main(void)
{
    char overlong[96];
    size_t index;

    vision_rx_init(&rx);

    /* --- accepted forms --------------------------------------------- */
    expect_stop_line("STOP\n");
    expect_stop_line("STOP\r\n");

    /* Fires only on the LF, byte at a time and for every split point. */
    expect_fires_on_lf_only("STOP\n");
    expect_fires_on_lf_only("STOP\r\n");
    expect_every_split("STOP\n");
    expect_every_split("STOP\r\n");

    /* No LF, no callback: the line is not complete. */
    reset_event();
    feed_bytes("STOP");
    CHECK(callback_count == 0U);
    feed_bytes("\r");
    CHECK(callback_count == 0U);
    feed_bytes("\n");
    CHECK(callback_count == 1U);
    CHECK(event_seen == EV_STOP);

    /* --- repeated STOPs, one callback each ---------------------------- */
    reset_event();
    feed_bytes("STOP\nSTOP\nSTOP\n");
    CHECK(callback_count == 3U);
    CHECK((log_events[0] == EV_STOP) && (log_events[1] == EV_STOP) &&
          (log_events[2] == EV_STOP));

    reset_event();
    feed_bytes("STOP\r\nSTOP\nSTOP\r\n");
    CHECK(callback_count == 3U);

    /* --- ordering with other commands in one stream -------------------- */
    reset_event();
    feed_bytes("STOP\nMOV,F,10\n");
    CHECK(callback_count == 2U);
    CHECK((log_events[0] == EV_STOP) && (log_events[1] == EV_MOVE));

    reset_event();
    feed_bytes("MOV,F,10\nSTOP\n");
    CHECK(callback_count == 2U);
    CHECK((log_events[0] == EV_MOVE) && (log_events[1] == EV_STOP));

    reset_event();
    feed_bytes("STOP\r\nMOV,FL,90\r\nSTOP\r\n");
    CHECK(callback_count == 3U);
    CHECK((log_events[0] == EV_STOP) && (log_events[1] == EV_MOVE) &&
          (log_events[2] == EV_STOP));

    reset_event();
    feed_bytes("PID,S,45,0,0\nSTOP\nRTGUARD,1\n");
    CHECK(callback_count == 3U);
    CHECK((log_events[0] == EV_PID_SET) && (log_events[1] == EV_STOP) &&
          (log_events[2] == EV_RTGUARD_SET));
    CHECK(rt_enable == 1);

    /* A valid STOP right after a blank line / bare newline. */
    reset_event();
    feed_bytes("\n\nSTOP\n");
    CHECK(callback_count == 1U);
    CHECK(event_seen == EV_STOP);

    /* --- near misses: rejected exactly as before ------------------------ */
    expect_rejected("stop\n");
    expect_rejected("Stop\n");
    expect_rejected("sTOP\n");
    expect_rejected("STOp\n");
    expect_rejected(" STOP\n");
    expect_rejected("STOP \n");
    expect_rejected("  STOP  \n");
    expect_rejected("\tSTOP\n");
    expect_rejected("STOP,\n");
    expect_rejected("STOP,1\n");
    expect_rejected("STOP,STOP\n");
    expect_rejected("STOP1\n");
    expect_rejected("STOPS\n");
    expect_rejected("STOPX\n");
    expect_rejected("XSTOP\n");
    expect_rejected("STO\n");
    expect_rejected("ST\n");
    expect_rejected("S\n");
    expect_rejected("STOPSTOP\n");
    expect_rejected("STOP STOP\n");
    expect_rejected("STOP;\n");
    expect_rejected("STOP:\n");

    /* Control characters and doubled/bare CR: the existing overflow
     * recovery rejects the whole line once. */
    expect_rejected("STOP\x01\n");
    expect_rejected("ST\tOP\n");
    expect_rejected("STOP\r\r\n");
    expect_rejected("ST\rOP\n");
    expect_rejected("\x7f" "STOP\n");

    /* A near miss never calls on_stop, and the next exact STOP works. */
    reset_event();
    feed_bytes("STOP,1\n");
    CHECK(event_seen == EV_BAD);
    expect_stop_line("STOP\n");
    expect_rejected("stop\n");
    expect_stop_line("STOP\r\n");

    /* --- STOPCTRL shares the prefix but is a different command ---------- */
    reset_event();
    feed_bytes("STOPCTRL,150,1700,1\n");
    CHECK(callback_count == 1U);
    CHECK(event_seen == EV_STOPCTRL_SET);

    reset_event();
    feed_bytes("STOPCTRL,GET\n");
    CHECK(callback_count == 1U);
    CHECK(event_seen != EV_STOP);

    /* --- over-long frames recover -------------------------------------- */
    for (index = 0U; index < sizeof(overlong) - 1U; index++) {
        overlong[index] = 'A';
    }
    overlong[sizeof(overlong) - 1U] = '\0';

    reset_event();
    feed_bytes(overlong);
    CHECK(callback_count == 0U);                    /* nothing until the LF */
    feed_bytes("\n");
    CHECK(callback_count == 1U);
    CHECK(event_seen == EV_BAD);
    expect_stop_line("STOP\n");                     /* recovered */

    /* An over-long line that STARTS with STOP is not a STOP. */
    memcpy(overlong, "STOP", 4);
    reset_event();
    feed_bytes(overlong);
    feed_bytes("\n");
    CHECK(callback_count == 1U);
    CHECK(event_seen == EV_BAD);
    expect_stop_line("STOP\n");

    /* Exactly at the buffer limit (31 chars) is accepted for other
     * commands but "STOP" padded to that length is still not STOP. */
    reset_event();
    feed_bytes("STOP                           \n");
    CHECK(event_seen == EV_BAD);
    expect_stop_line("STOP\r\n");

    /* --- no duplicate parsing, parser state reset ------------------------ */
    reset_event();
    feed_bytes("STOP\n");
    CHECK(callback_count == 1U);                    /* one callback, any kind */
    CHECK(log_len == 1U);
    feed_bytes("MOV,F,5\n");                        /* not glued onto STOP */
    CHECK(callback_count == 2U);
    CHECK(event_seen == EV_MOVE);

    /* Mixed with USPARK (the other early-path command). */
    reset_event();
    feed_bytes("STOP\nUSPARK,1\nSTOP\nUSPARK,GET\n");
    CHECK(callback_count == 4U);
    CHECK((log_events[0] == EV_STOP) && (log_events[1] == EV_USPARK_SET) &&
          (log_events[2] == EV_STOP) && (log_events[3] == EV_USPARK_GET));

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
