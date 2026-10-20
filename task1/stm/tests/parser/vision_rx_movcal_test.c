/* Host-buildable regression test for Stage A4's
 * MOVCAL,<slopeCountsPerCm>,<offsetCounts>,<enable> / MOVCAL,GET parsing
 * and strict rejection.
 *
 * Builds directly against the live Core/Src/vision_rx.c (no HAL deps),
 * stubbing every on_*() callback vision_rx.c declares as extern.
 * MOV/WSYNC/HCTRL/RTGUARD/DPROF/SPCTRL/SPPROF/STOPCTRL coverage lives
 * separately. Note: MOVCAL's own configuration invariant
 * (slopeCountsPerCm+offsetCounts>0) lives in app_comm.c's
 * on_distance_cal_set(), not in vision_rx.c -- this suite covers only the
 * independent per-field range bounds vision_rx.c itself enforces.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -o tests/parser/vision_rx_movcal_test.exe \
 *       tests/parser/vision_rx_movcal_test.c Core/Src/vision_rx.c
 *   ./tests/parser/vision_rx_movcal_test.exe
 */

#include "vision_rx.h"

#include <stdio.h>
#include <string.h>

typedef enum {
    EV_NONE = 0,
    EV_MOVCAL_SET,
    EV_MOVCAL_GET,
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

static float mc_slope_counts_per_cm;
static float mc_offset_counts;
static int   mc_enable;

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
    mc_slope_counts_per_cm = -1.0f;
    mc_offset_counts = -9999.0f;
    mc_enable = -1;
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

void on_distance_cal_set(float slopeCountsPerCm, float offsetCounts, int enable)
{
    mc_slope_counts_per_cm = slopeCountsPerCm;
    mc_offset_counts = offsetCounts;
    mc_enable = enable;
    record(EV_MOVCAL_SET);
}

void on_distance_cal_get(void) { record(EV_MOVCAL_GET); }

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

static void expect_movcal_set(const char *line, float slopeCountsPerCm, float offsetCounts, int enable)
{
    feed_bytes(line);
    CHECK(event_seen == EV_MOVCAL_SET);
    CHECK(callback_count == 1U);
    CHECK(mc_slope_counts_per_cm == slopeCountsPerCm);
    CHECK(mc_offset_counts == offsetCounts);
    CHECK(mc_enable == enable);
}

static void expect_movcal_get(const char *line)
{
    feed_bytes(line);
    CHECK(event_seen == EV_MOVCAL_GET);
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

    /* --- Valid MOVCAL,SET ------------------------------------------- */
    expect_movcal_set("MOVCAL,75,100,0\n", 75.0f, 100.0f, 0);
    expect_movcal_set("MOVCAL,75,100,1\n", 75.0f, 100.0f, 1);
    expect_movcal_set("MOVCAL,50,-500,1\n", 50.0f, -500.0f, 1);  /* both lower bounds -- parser only, see below */
    expect_movcal_set("MOVCAL,100,500,1\n", 100.0f, 500.0f, 1);  /* both upper bounds */
    expect_movcal_set("MOVCAL,75.0,100.0,1\n", 75.0f, 100.0f, 1);
    expect_movcal_set("MOVCAL,75,-100,1\n", 75.0f, -100.0f, 1);  /* negative offset, still in range */

    /* --- Valid MOVCAL,GET ----------------------------------------------- */
    expect_movcal_get("MOVCAL,GET\n");

    /* --- Existing commands still dispatch ---------------------------- */
    expect_move("MOV,F,80\n");
    expect_pid_set("PID,S,45,0,0\n");

    /* --- Invalid: missing / empty fields ----------------------------- */
    expect_bad("MOVCAL,75,100\n");    /* missing enable */
    expect_bad("MOVCAL,75\n");        /* missing offsetCounts + enable */
    expect_bad("MOVCAL,\n");          /* nothing */
    expect_bad("MOVCAL\n");           /* no comma at all */
    expect_bad("MOVCAL,75,,1\n");     /* empty offsetCounts */
    expect_bad("MOVCAL,,100,1\n");    /* empty slopeCountsPerCm */
    expect_bad("MOVCAL,75,100,\n");   /* empty enable */

    /* --- Invalid: trailing garbage -------------------------------- */
    expect_bad("MOVCAL,75,100,1,\n");       /* trailing comma */
    expect_bad("MOVCAL,75,100,1,extra\n");  /* extra field */
    expect_bad("MOVCAL,75,100,1x\n");       /* junk after enable */
    expect_bad("MOVCAL,75,100x,1\n");       /* junk after offsetCounts */

    /* --- Invalid: non-numeric ----------------------------------------- */
    expect_bad("MOVCAL,abc,100,1\n");
    expect_bad("MOVCAL,75,def,1\n");
    expect_bad("MOVCAL,75,100,ghi\n");

    /* --- Invalid: out of range (both directions of both bounds) ------ */
    expect_bad("MOVCAL,49,100,1\n");     /* slopeCountsPerCm below 50 */
    expect_bad("MOVCAL,100.1,100,1\n");  /* slopeCountsPerCm above 100 */
    expect_bad("MOVCAL,75,-500.1,1\n");  /* offsetCounts below -500 */
    expect_bad("MOVCAL,75,500.1,1\n");   /* offsetCounts above 500 */
    expect_bad("MOVCAL,75,100,2\n");     /* enable not 0/1 */
    expect_bad("MOVCAL,75,100,-1\n");    /* enable not 0/1 */

    /* --- Cross-field invariant (slope+offset>0) is NOT enforced here ---
     * "MOVCAL,50,-500,1" above is accepted by the parser: both fields are
     * independently in range (this is exactly the combination that
     * app_comm.c's on_distance_cal_set() must reject as slope+offset<=0 --
     * that check lives one layer up, in the callback this test stubs out,
     * not in vision_rx.c). This assertion documents that boundary rather
     * than silently omitting it. */

    /* --- MOVCAL,GET must be exact ------------------------------------ */
    expect_bad("MOVCAL,GET,\n");
    expect_bad("MOVCAL,GETX\n");

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
