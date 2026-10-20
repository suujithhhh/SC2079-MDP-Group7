/* vision_rx.c -- STM32 side of the Pi vision link.
 *
 * Reference parser for whoever owns the STM32 firmware. Drop these two
 * files in, feed every received byte to vision_rx_feed(), and implement
 * on_target() / on_no_target().
 *
 * Protocol (ASCII, newline terminated, 115200 8N1):
 *
 *     RDY\n                        Pi vision came up -- reset your state
 *     TGT,<id>,<conf>,<cx>,<cy>\n  one detection
 *     NUL\n                        inference ran, found nothing
 *     STOP\n                       priority stop (exact uppercase, no fields)
 *     MOV,<dir>,<amt>\n            movement command, Pi is master
 *     MOV,FIRL,<cm>\n / MOV,FIRR,<cm>\n   forward to the far edge of an obstacle seen by the
 *                               left / right IR sensor, at most <cm> (1-999)
 *                                  dir: F B FL FR BL BR (bare L / R rejected)
 *     ENC,RST\n                    reset both encoder counters (bench diagnostic)
 *     ENC,GET\n                    report both encoder counters (bench diagnostic)
 *     PWM,<0-99>\n                 forward speed sweep (bench diagnostic)
 *     SPD,<0-99>\n                 wheel-speed sample test (bench diagnostic)
 *     IMU,GET\n                    report yaw / gyro-Z (bench diagnostic)
 *     IMU,ZERO\n                   zero relative yaw (bench diagnostic)
 *     IMU,FLIP\n                   flip yaw sign convention (bench diagnostic)
 *     HDG,<F|B>,<1-5>\n           heading-hold drive test in seconds
 *     HDG,RAW,<1-5>\n           fixed-centre forward IMU sign test
 *     SERVO,<1230-1830>\n        set steering pulse within the safe range
 *     SERVO,GET\n                report current steering-centre candidate
 *     TUNE,L|R|B,<counts>\n     tune one or both rear position PIDs
 *     TUNE,S,F|B,<seconds>\n    tune steering heading hold while moving
 *     WSYNC,<g>,<m>,<e>\n       rear-wheel sync trim: gain 0-1, maxDuty 0-39, enable 0/1
 *     WSYNC,GET\n               report current rear-wheel sync params
 *     HCTRL,<k>,<m>,<e>\n       heading rate damping: krate 0-30, maxCorrection 0-300, enable 0/1
 *     HCTRL,GET\n               report current heading rate damping params
 *     RTGUARD,<e>\n             reverse-torque guard (approach zone only): enable 0/1
 *     RTGUARD,GET\n             report current reverse-torque guard state
 *     USPARK,<e>\n              ultrasonic parking protection: 0 disable / 1 enable
 *     USPARK,GET\n              report ultrasonic parking protection state
 *     IR,GET\n                  report the calibrated IR distances (telemetry; deferred reply)
 *     USD,GET\n                 report the front ultrasonic distance (telemetry; deferred reply)
 *     DPROF,<d>,<a>,<e>\n       decel profile: decelCounts 300-3000, approachDuty 40-90, enable 0/1
 *     DPROF,GET\n               report current decel profile params
 *     SPCTRL,<t>,<p>,<i>,<e>\n  chassis speed control: target 0-8000 counts/s, Kp 0-1.0, Ki 0-1.0, enable 0/1
 *     SPCTRL,GET\n              report current chassis speed control params
 *     SPPROF,<d>,<a>,<e>\n      speed profile: decelStartCounts 300-6000, approachSpeed 1-8000, enable 0/1
 *     SPPROF,GET\n              report current speed profile params
 *     STOPCTRL,<l>,<s>,<e>\n    final stop gate: brakeLeadCounts 20-500, brakeEntrySpeed 500-2500, enable 0/1
 *     STOPCTRL,GET\n            report current final stop gate params
 *     MOVCAL,<m>,<b>,<e>\n      distance calibration: slopeCountsPerCm 50-100, offsetCounts -500-500, enable 0/1
 *     MOVCAL,GET\n              report current distance calibration params
 *     PROFILE,PRECISION\n       atomically load the PRECISION tuning profile
 *     PROFILE,STANDARD\n        atomically load the STANDARD tuning profile
 *     PROFILE,TASK2TEST\n       atomically load the TASK2TEST tuning profile (physically tuned Task 2 test profile)
 *     PROFILE,GET\n             report PRECISION, STANDARD, TASK2TEST, or CUSTOM (report-only)
 *     SFF,<F|B>,<s>,<f>,<m>,<e>\n  speed-aware servo feed-forward: startSpeed 0-8000, fullSpeed 0-8000, maxUs -100-100, enable 0/1
 *     SFF,GET\n                 report current speed-aware servo feed-forward params (both directions)
 *     PRESTEER,<d>,<e>\n        pre-turn servo/wheel sequencing: durationMs 0-1000, enable 0/1
 *     PRESTEER,GET\n            report current pre-steer params
 *     TURNSPEED,<r>\n           base turn rate: countsPerSec 500-2500
 *     TURNSPEED,GET\n           report current base turn rate
 *     ACKERMANN,<k>,<e>\n       rear-wheel target-rate differential: k 0.0-0.5, enable 0/1
 *     ACKERMANN,GET\n           report current Ackermann params
 *
 *     id    0-255      class id
 *     conf  0.00-1.00  confidence, always two decimals
 *     cx cy 0-9999     detection centre, pixels
 *     dir   F B FL FR BL BR   forward, backward, forward-left, forward-right,
 *                             backward-left, backward-right
 *     amt   0-999      centimetres for F/B, degrees for FL/FR/BL/BR (0-360)
 *
 * Lines longer than the fixed receive buffer are discarded up to the
 * next newline rather than overflowing.
 *
 * Call vision_rx_feed() straight from the UART RX interrupt if your
 * callbacks are short, or from the main loop against a DMA ring buffer
 * if they are not. The parser keeps no dynamic state beyond the struct.
 */

#include "vision_rx.h"
#include "app_servo.h"

#include <string.h>
#include <stdlib.h>

#define TUNE_MOTOR_MAX_ABS_COUNTS 7500L
#define TUNE_STEERING_MAX_SECONDS 5L
#define DIAGNOSTIC_DUTY_MAX        99U
#define HEADING_TEST_MIN_SECONDS   1U
#define HEADING_TEST_MAX_SECONDS   5U
#define MOVE_DISTANCE_MAX_CM       999U
#define MOVE_TURN_MAX_DEGREES      360U

/* Stage A1 rear-wheel sync (WSYNC). maxDuty is hard-limited to 39 so the
 * sync trim can never request the leading wheel below the firmware's
 * reliable 60-duty cruise floor (motor max 99 - 39 = 60). */
#define WSYNC_GAIN_MAX             1.0f
#define WSYNC_MAX_DUTY_LIMIT      39.0f

/* Stage A2 heading rate damping (HCTRL). Hard ceilings are safe upper
 * bounds for later experiments; the physically-informed search bracket
 * is documented in app_motion.c's compiled defaults. */
#define HCTRL_KRATE_MAX             30.0f
#define HCTRL_MAX_CORRECTION_LIMIT 300.0f

/* Stage A3.2 deceleration profile (DPROF). decelCounts default 1100 and
 * approachDuty default 60 are INITIAL physical-test values (see
 * app_motion.c); these are just the parser's safe outer bounds. */
#define DPROF_DECEL_COUNTS_MIN     300.0f
#define DPROF_DECEL_COUNTS_MAX    3000.0f
#define DPROF_APPROACH_DUTY_MIN    40.0f
#define DPROF_APPROACH_DUTY_MAX    90.0f

/* Stage A3.3a closed-loop chassis speed control (SPCTRL). Target is a
 * generous outer bound on plausible chassis counts/s; Kp/Ki ceilings are
 * safe upper bounds for bench tuning, not physically-informed defaults
 * (see app_motion.c's compiled speedPID defaults). */
#define SPCTRL_TARGET_SPEED_MIN      0.0f
#define SPCTRL_TARGET_SPEED_MAX   8000.0f
#define SPCTRL_KP_MAX                1.0f
#define SPCTRL_KI_MAX                1.0f

/* Stage A3.3b distance-based linear speed profile (SPPROF), layered on
 * SPCTRL. decelStartCounts floor matches DPROF's own MIN (=APPROACH_ZONE_
 * COUNTS=300 in app_motion.c), keeping the profile's ratio denominator
 * well clear of zero; ceiling is wider than DPROF's (6000 vs 3000) since
 * this drives a smoother PI-mediated speed transition rather than a raw
 * duty step. approachSpeed's floor is 1.0, not 0.0 -- a strict protocol
 * validity bound (a zero-duration/zero-speed floor is a degenerate case
 * this stage does not define), NOT a claim that 1 counts/s is a usable
 * physical target; its ceiling matches SPCTRL's own target ceiling, since
 * approachSpeed is itself a target-speed quantity. The tighter,
 * configuration-dependent bound (approachSpeed <= the currently-configured
 * SPCTRL cruise target) cannot be checked here -- see app_comm.c. */
#define SPPROF_DECEL_COUNTS_MIN     300.0f
#define SPPROF_DECEL_COUNTS_MAX    6000.0f
#define SPPROF_APPROACH_SPEED_MIN     1.0f
#define SPPROF_APPROACH_SPEED_MAX  8000.0f

/* Stage A3.3c final controlled stop gate (STOPCTRL), layered on SPCTRL.
 * brakeLeadCounts floor of 20 sits just above POSITION_TOLERANCE_COUNTS
 * (=15 in app_motion.c) so the gate is meaningfully distinct from plain
 * tolerance; ceiling of 500 stays an order of magnitude below SPPROF's
 * own decelStartCounts range and well under the shortest validated move
 * (F30/B30), so a short move cannot brake immediately after startup just
 * because it begins inside a large window. brakeEntrySpeed ceiling of
 * 2500 stays safely below the validated cruise target (4000) so the gate
 * can never be configured to fire during full cruise; floor of 500 stays
 * below the physically observed final-approach speed range. No
 * relationship to SPPROF's approachSpeed or SPCTRL's cruise target is
 * enforced -- unlike SPPROF's own approachSpeed<=cruise invariant (which
 * was structurally required), brakeEntrySpeed above or below approachSpeed
 * is a meaningful, valid tuning choice, not a configuration error. */
#define STOPCTRL_BRAKE_LEAD_COUNTS_MIN     20.0f
#define STOPCTRL_BRAKE_LEAD_COUNTS_MAX    500.0f
#define STOPCTRL_BRAKE_ENTRY_SPEED_MIN    500.0f
#define STOPCTRL_BRAKE_ENTRY_SPEED_MAX   2500.0f

/* Stage A4 distance (counts/cm) calibration (MOVCAL), for MOV,F/B target
 * generation only. slopeCountsPerCm brackets both the true hardware scale
 * (ENCODER_COUNTS_PER_CM=72 in app_encoder.h) and the initial calibrated
 * candidate (75) with margin for bench iteration. offsetCounts is
 * symmetric, not just positive: the candidate is +100, but a future
 * recalibration could plausibly need a negative correction instead --
 * strtof already parses a leading '-' natively, no parser change needed
 * for that. The tighter, configuration-dependent invariant
 * (slopeCountsPerCm+offsetCounts>0, so a 1 cm command can never resolve to
 * zero or negative counts) cannot be checked here from two independently-
 * ranged fields in a generic way -- see app_comm.c. */
#define MOVCAL_SLOPE_COUNTS_PER_CM_MIN    50.0f
#define MOVCAL_SLOPE_COUNTS_PER_CM_MAX   100.0f
#define MOVCAL_OFFSET_COUNTS_MIN        -500.0f
#define MOVCAL_OFFSET_COUNTS_MAX         500.0f

/* Stage SFF1 speed-aware servo feed-forward (SFF), layered on the
 * existing heading PID + HCTRL output during straight MOV,F/MOV,B only.
 * startSpeed/fullSpeed share SPCTRL's own target-speed ceiling (both
 * profiles' cruise targets, 4000/6000, fit comfortably inside it).
 * maxUs is deliberately tight (+-100, well inside steeringPID's own
 * +-530 output clamp) -- this is meant to stay a SMALL feed-forward
 * term, not a second steering controller; its sign is not assumed here
 * or anywhere else -- physical correction direction is entirely
 * determined by which sign is tuned in for each direction. The tighter,
 * configuration-dependent invariant (fullSpeed>startSpeed) cannot be
 * checked here from two independently-ranged fields -- see app_comm.c. */
#define SFF_START_SPEED_MIN     0.0f
#define SFF_START_SPEED_MAX  8000.0f
#define SFF_FULL_SPEED_MIN      0.0f
#define SFF_FULL_SPEED_MAX   8000.0f
#define SFF_MAX_US_MIN        -100.0f
#define SFF_MAX_US_MAX          100.0f

/* PRE_STEER (PRESTEER): non-blocking wait ahead of MOV,FL/FR/BL/BR's
 * existing turn logic. durationMs ceiling of 1000 keeps any worst-case
 * misconfiguration small relative to a turn's own timeout budget; 0 is
 * a valid, in-range value (distinct from disabled -- see
 * app_motion.c). */
#define PRESTEER_DURATION_MS_MIN     0.0f
#define PRESTEER_DURATION_MS_MAX  1000.0f

/* TURNSPEED: replaces the previously-hardcoded 2000 counts/sec base
 * turn rate. 2500 stays a hardware-safety-neutral ceiling --
 * leftPID/rightPID's own output clamp bounds the motor command
 * independent of the target-rate magnitude (see app_motion.c) -- it
 * is a tuning-quality bound only, set a little above the operator's
 * own planned physical sweep (which does not exceed 2000). 500 avoids
 * a target-position setpoint moving so slowly it lingers inside
 * POSITION_TOLERANCE_COUNTS and chatters against the motor min-duty
 * floor. */
#define TURNSPEED_COUNTS_PER_SEC_MIN   500U
#define TURNSPEED_COUNTS_PER_SEC_MAX  2500U

/* Ackermann-v2 (ACKERMANN): redistributes the runtime TURNSPEED base
 * rate between inner/outer rear wheel targets. k=0.5 keeps
 * innerRate>=50% and outerRate<=150% of whatever TURNSPEED currently
 * is -- confirmed safe in app_motion.c (leftPID/rightPID's own output
 * clamp bounds the motor command independent of the target-rate
 * magnitude). */
#define ACKERMANN_K_MIN   0.0f
#define ACKERMANN_K_MAX   0.5f

/* TURNBRAKE: turn-only predictive brake trigger. 500ms ceiling is a
 * deliberately conservative sanity bound -- at the observed ~52-56
 * deg/s near-target yaw rate on the bench, 500ms would already predict
 * a ~27-28 degree stopping angle, implausibly large for this
 * application; the bound exists to catch operator error, not because
 * larger values are expected to be useful (see app_motion.c). */
#define TURNBRAKE_LOOKAHEAD_MS_MIN   0.0f
#define TURNBRAKE_LOOKAHEAD_MS_MAX 500.0f

static int vision_rx_parse_bounded_u32(
    const char *text,
    uint32_t minimum,
    uint32_t maximum,
    uint32_t *value
)
{
    uint32_t parsed = 0U;

    if ((text == NULL) ||
        (value == NULL) ||
        (*text < '0') ||
        (*text > '9')) {
        return 0;
    }

    while ((*text >= '0') && (*text <= '9')) {
        uint32_t digit = (uint32_t)(*text - '0');

        if ((parsed > (maximum / 10U)) ||
            ((parsed == (maximum / 10U)) &&
             (digit > (maximum % 10U)))) {
            return 0;
        }

        parsed = (parsed * 10U) + digit;
        text++;
    }

    if ((*text != '\0') || (parsed < minimum)) {
        return 0;
    }

    *value = parsed;
    return 1;
}

static int vision_rx_parse_bounded_u32_field(
    const char **text,
    uint32_t minimum,
    uint32_t maximum,
    uint32_t *value
)
{
    const char *p;
    uint32_t parsed = 0U;

    if ((text == NULL) ||
        (*text == NULL) ||
        (value == NULL)) {
        return 0;
    }

    p = *text;

    if ((*p < '0') || (*p > '9')) {
        return 0;
    }

    while ((*p >= '0') && (*p <= '9')) {
        uint32_t digit = (uint32_t)(*p - '0');

        if ((parsed > (maximum / 10U)) ||
            ((parsed == (maximum / 10U)) &&
             (digit > (maximum % 10U)))) {
            return 0;
        }

        parsed = (parsed * 10U) + digit;
        p++;
    }

    if ((*p != ',') || (parsed < minimum)) {
        return 0;
    }

    *text = p + 1;
    *value = parsed;
    return 1;
}

static int vision_rx_parse_confidence_field(
    const char **text,
    float *confidence
)
{
    const char *p;
    uint32_t hundredths;

    if ((text == NULL) ||
        (*text == NULL) ||
        (confidence == NULL)) {
        return 0;
    }

    p = *text;

    if (((p[0] != '0') && (p[0] != '1')) ||
        (p[1] != '.') ||
        (p[2] < '0') || (p[2] > '9') ||
        (p[3] < '0') || (p[3] > '9') ||
        (p[4] != ',')) {
        return 0;
    }

    hundredths =
        ((uint32_t)(p[0] - '0') * 100U) +
        ((uint32_t)(p[2] - '0') * 10U) +
        (uint32_t)(p[3] - '0');

    if (hundredths > 100U) {
        return 0;
    }

    *text = p + 5;
    *confidence = (float)hundredths / 100.0f;
    return 1;
}

void vision_rx_init(vision_rx_t *rx)
{
    rx->len = 0;
    rx->overflow = 0;
    rx->pending_cr = 0;
}

/* USPARK,<0|1> / USPARK,GET. Kept in its own function (not inline in
 * vision_rx_dispatch) so the dispatcher's large stack frame, which sits under
 * every parser callback on CommTask's small stack, does not grow. Returns 1
 * if the line began with "USPARK," (handled, including a bad frame), 0 if it
 * is not a USPARK line at all. Strict like RTGUARD: digits 0-1 only. */
static int vision_rx_dispatch_us_park(vision_rx_t *rx)
{
    uint32_t enableValue;

    if (strcmp(rx->buf, "USPARK,GET") == 0) {
        on_us_park_get();
        return 1;
    }

    if (strncmp(rx->buf, "USPARK,", 7) != 0) {
        return 0;
    }

    if (vision_rx_parse_bounded_u32(rx->buf + 7, 0U, 1U, &enableValue)) {
        on_us_park_set((int)enableValue);
    } else {
        on_bad_frame(rx->buf);
    }

    return 1;
}

/* IR,GET and USD,GET: exact match only. Its own function for the same reason
 * as the USPARK helper above: it must not add to vision_rx_dispatch()'s frame.
 * Returns 1 if handled. */
static int vision_rx_dispatch_ir(vision_rx_t *rx)
{
    if (strcmp(rx->buf, "IR,GET") == 0) {
        on_ir_get();
        return 1;
    }

    if (strcmp(rx->buf, "USD,GET") == 0) {
        on_usd_get();
        return 1;
    }

    return 0;
}

/* MOV,FIRL,<cm> / MOV,FIRR,<cm> (Phase 5): exact prefix, then 1-999 cm.
 * Its own function for the same reason as the USPARK and IR,GET helpers:
 * vision_rx_dispatch()'s frame sits under every parser callback. Returns 1 if
 * the line began with "MOV,FIRL," or "MOV,FIRR," (handled, including a bad
 * frame), 0 otherwise. Every other MOV,F... form is left to the dispatcher
 * and is rejected there exactly as before. */
static int vision_rx_dispatch_fir(vision_rx_t *rx)
{
    uint32_t amount;
    VisionMoveType moveType;

    if (strncmp(rx->buf, "MOV,FIRL,", 9) == 0) {
        moveType = VISION_MOVE_FORWARD_IR_LEFT;
    } else if (strncmp(rx->buf, "MOV,FIRR,", 9) == 0) {
        moveType = VISION_MOVE_FORWARD_IR_RIGHT;
    } else {
        return 0;
    }

    if (vision_rx_parse_bounded_u32(
            rx->buf + 9,
            1U,
            MOVE_DISTANCE_MAX_CM,
            &amount)) {
        on_move(moveType, (uint16_t)amount);
    } else {
        on_bad_frame(rx->buf);
    }

    return 1;
}

static void vision_rx_dispatch(vision_rx_t *rx)
{
    rx->buf[rx->len] = '\0';

    if (rx->len == 0) {
        return; /* bare newline, ignore */
    }

    if (strcmp(rx->buf, "NUL") == 0) {
        on_no_target();
        return;
    }

    if (strcmp(rx->buf, "RDY") == 0) {
        on_vision_ready();
        return;
    }

    if (strcmp(rx->buf, "ENC,RST") == 0) {
        on_encoder_reset();
        return;
    }

    if (strcmp(rx->buf, "ENC,GET") == 0) {
        on_encoder_get();
        return;
    }

    if (strncmp(rx->buf, "PWM,", 4) == 0) {
        uint32_t duty;

        if (vision_rx_parse_bounded_u32(
                rx->buf + 4,
                0U,
                DIAGNOSTIC_DUTY_MAX,
                &duty)) {
            on_pwm_test((uint8_t)duty);
            return;
        }

        on_bad_frame(rx->buf);
        return;
    }

    if (strncmp(rx->buf, "SPD,", 4) == 0) {
        uint32_t duty;

        if (vision_rx_parse_bounded_u32(
                rx->buf + 4,
                0U,
                DIAGNOSTIC_DUTY_MAX,
                &duty)) {
            on_speed_test((uint8_t)duty);
            return;
        }

        on_bad_frame(rx->buf);
        return;
    }



    if (strcmp(rx->buf, "SERVO,GET") == 0) {
        on_servo_get();
        return;
    }

    if (strncmp(rx->buf, "SERVO,", 6) == 0) {
        uint32_t pulse;

        if (vision_rx_parse_bounded_u32(
                rx->buf + 6,
                SERVO_MIN_US,
                SERVO_MAX_US,
                &pulse)) {
            on_servo_set((uint16_t)pulse);
            return;
        }

        on_bad_frame(rx->buf);
        return;
    }

    if (strcmp(rx->buf, "IMU,GET") == 0) {
        on_imu_get();
        return;
    }

    if (strcmp(rx->buf, "IMU,ZERO") == 0) {
        on_imu_zero();
        return;
    }

    if (strcmp(rx->buf, "IMU,FLIP") == 0) {
        on_imu_flip();
        return;
    }

    if (strcmp(rx->buf, "PID,GET") == 0) {
        on_pid_get();
        return;
    }

    if (strncmp(rx->buf, "PID,", 4) == 0) {
        const char *p = rx->buf + 4;

        char target = *p;

        if ((target != 'L') &&
            (target != 'R') &&
            (target != 'S')) {
            on_bad_frame(rx->buf);
            return;
        }

        p++;

        if (*p != ',') {
            on_bad_frame(rx->buf);
            return;
        }

        p++;

        char *end;

        float kp = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float ki = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float kd = strtof(p, &end);

        if ((end == p) || (*end != '\0')) {
            on_bad_frame(rx->buf);
            return;
        }

        if (!(kp >= 0.0f && kp <= VISION_PID_GAIN_MAX) ||
            !(ki >= 0.0f && ki <= VISION_PID_GAIN_MAX) ||
            !(kd >= 0.0f && kd <= VISION_PID_GAIN_MAX)) {
            on_bad_frame(rx->buf);
            return;
        }

        on_pid_set(
            target,
            kp,
            ki,
            kd
        );

        return;
    }

    if (strcmp(rx->buf, "WSYNC,GET") == 0) {
        on_wheel_sync_get();
        return;
    }

    if (strncmp(rx->buf, "WSYNC,", 6) == 0) {
        const char *p = rx->buf + 6;
        char *end;

        float gain = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float maxDuty = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float enableValue = strtof(p, &end);

        if ((end == p) || (*end != '\0')) {
            on_bad_frame(rx->buf);
            return;
        }

        if (!(gain >= 0.0f && gain <= WSYNC_GAIN_MAX) ||
            !(maxDuty >= 0.0f && maxDuty <= WSYNC_MAX_DUTY_LIMIT) ||
            !((enableValue == 0.0f) || (enableValue == 1.0f))) {
            on_bad_frame(rx->buf);
            return;
        }

        on_wheel_sync_set(
            gain,
            maxDuty,
            (enableValue == 1.0f) ? 1 : 0
        );

        return;
    }

    if (strcmp(rx->buf, "HCTRL,GET") == 0) {
        on_heading_rate_get();
        return;
    }

    if (strncmp(rx->buf, "HCTRL,", 6) == 0) {
        const char *p = rx->buf + 6;
        char *end;

        float krate = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float maxCorrection = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float enableValue = strtof(p, &end);

        if ((end == p) || (*end != '\0')) {
            on_bad_frame(rx->buf);
            return;
        }

        if (!(krate >= 0.0f && krate <= HCTRL_KRATE_MAX) ||
            !(maxCorrection >= 0.0f && maxCorrection <= HCTRL_MAX_CORRECTION_LIMIT) ||
            !((enableValue == 0.0f) || (enableValue == 1.0f))) {
            on_bad_frame(rx->buf);
            return;
        }

        on_heading_rate_set(
            krate,
            maxCorrection,
            (enableValue == 1.0f) ? 1 : 0
        );

        return;
    }

    if (strcmp(rx->buf, "RTGUARD,GET") == 0) {
        on_reverse_torque_guard_get();
        return;
    }

    if (strncmp(rx->buf, "RTGUARD,", 8) == 0) {
        uint32_t enableValue;

        if (vision_rx_parse_bounded_u32(
                rx->buf + 8,
                0U,
                1U,
                &enableValue)) {
            on_reverse_torque_guard_set((int)enableValue);
            return;
        }

        on_bad_frame(rx->buf);
        return;
    }

    if (strcmp(rx->buf, "DPROF,GET") == 0) {
        on_decel_profile_get();
        return;
    }

    if (strncmp(rx->buf, "DPROF,", 6) == 0) {
        const char *p = rx->buf + 6;
        char *end;

        float decelCounts = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float approachDuty = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float enableValue = strtof(p, &end);

        if ((end == p) || (*end != '\0')) {
            on_bad_frame(rx->buf);
            return;
        }

        if (!(decelCounts >= DPROF_DECEL_COUNTS_MIN && decelCounts <= DPROF_DECEL_COUNTS_MAX) ||
            !(approachDuty >= DPROF_APPROACH_DUTY_MIN && approachDuty <= DPROF_APPROACH_DUTY_MAX) ||
            !((enableValue == 0.0f) || (enableValue == 1.0f))) {
            on_bad_frame(rx->buf);
            return;
        }

        on_decel_profile_set(
            decelCounts,
            approachDuty,
            (enableValue == 1.0f) ? 1 : 0
        );

        return;
    }

    if (strcmp(rx->buf, "SPCTRL,GET") == 0) {
        on_speed_control_get();
        return;
    }

    if (strncmp(rx->buf, "SPCTRL,", 7) == 0) {
        const char *p = rx->buf + 7;
        char *end;

        float targetSpeed = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float kp = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float ki = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float enableValue = strtof(p, &end);

        if ((end == p) || (*end != '\0')) {
            on_bad_frame(rx->buf);
            return;
        }

        if (!(targetSpeed >= SPCTRL_TARGET_SPEED_MIN && targetSpeed <= SPCTRL_TARGET_SPEED_MAX) ||
            !(kp >= 0.0f && kp <= SPCTRL_KP_MAX) ||
            !(ki >= 0.0f && ki <= SPCTRL_KI_MAX) ||
            !((enableValue == 0.0f) || (enableValue == 1.0f))) {
            on_bad_frame(rx->buf);
            return;
        }

        on_speed_control_set(
            targetSpeed,
            kp,
            ki,
            (enableValue == 1.0f) ? 1 : 0
        );

        return;
    }

    if (strcmp(rx->buf, "SPPROF,GET") == 0) {
        on_speed_profile_get();
        return;
    }

    if (strncmp(rx->buf, "SPPROF,", 7) == 0) {
        const char *p = rx->buf + 7;
        char *end;

        float decelStartCounts = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float approachSpeed = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float enableValue = strtof(p, &end);

        if ((end == p) || (*end != '\0')) {
            on_bad_frame(rx->buf);
            return;
        }

        if (!(decelStartCounts >= SPPROF_DECEL_COUNTS_MIN && decelStartCounts <= SPPROF_DECEL_COUNTS_MAX) ||
            !(approachSpeed >= SPPROF_APPROACH_SPEED_MIN && approachSpeed <= SPPROF_APPROACH_SPEED_MAX) ||
            !((enableValue == 0.0f) || (enableValue == 1.0f))) {
            on_bad_frame(rx->buf);
            return;
        }

        on_speed_profile_set(
            decelStartCounts,
            approachSpeed,
            (enableValue == 1.0f) ? 1 : 0
        );

        return;
    }

    if (strcmp(rx->buf, "STOPCTRL,GET") == 0) {
        on_stop_control_get();
        return;
    }

    if (strncmp(rx->buf, "STOPCTRL,", 9) == 0) {
        const char *p = rx->buf + 9;
        char *end;

        float brakeLeadCounts = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float brakeEntrySpeed = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float enableValue = strtof(p, &end);

        if ((end == p) || (*end != '\0')) {
            on_bad_frame(rx->buf);
            return;
        }

        if (!(brakeLeadCounts >= STOPCTRL_BRAKE_LEAD_COUNTS_MIN && brakeLeadCounts <= STOPCTRL_BRAKE_LEAD_COUNTS_MAX) ||
            !(brakeEntrySpeed >= STOPCTRL_BRAKE_ENTRY_SPEED_MIN && brakeEntrySpeed <= STOPCTRL_BRAKE_ENTRY_SPEED_MAX) ||
            !((enableValue == 0.0f) || (enableValue == 1.0f))) {
            on_bad_frame(rx->buf);
            return;
        }

        on_stop_control_set(
            brakeLeadCounts,
            brakeEntrySpeed,
            (enableValue == 1.0f) ? 1 : 0
        );

        return;
    }

    if (strcmp(rx->buf, "MOVCAL,GET") == 0) {
        on_distance_cal_get();
        return;
    }

    if (strncmp(rx->buf, "MOVCAL,", 7) == 0) {
        const char *p = rx->buf + 7;
        char *end;

        float slopeCountsPerCm = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float offsetCounts = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float enableValue = strtof(p, &end);

        if ((end == p) || (*end != '\0')) {
            on_bad_frame(rx->buf);
            return;
        }

        if (!(slopeCountsPerCm >= MOVCAL_SLOPE_COUNTS_PER_CM_MIN && slopeCountsPerCm <= MOVCAL_SLOPE_COUNTS_PER_CM_MAX) ||
            !(offsetCounts >= MOVCAL_OFFSET_COUNTS_MIN && offsetCounts <= MOVCAL_OFFSET_COUNTS_MAX) ||
            !((enableValue == 0.0f) || (enableValue == 1.0f))) {
            on_bad_frame(rx->buf);
            return;
        }

        on_distance_cal_set(
            slopeCountsPerCm,
            offsetCounts,
            (enableValue == 1.0f) ? 1 : 0
        );

        return;
    }

    /* Named runtime configuration profile. Four exact literal matches
     * (GET / PRECISION / STANDARD / TASK2TEST), no numeric fields to parse -- any
     * other "PROFILE,..." (bad name, wrong case, trailing garbage, extra
     * fields) falls through to the catch-all strncmp below and is
     * rejected; a bare "PROFILE" with no comma at all falls through to
     * vision_rx_dispatch()'s own final on_bad_frame(), same as every
     * other bare command name. */
    if (strcmp(rx->buf, "PROFILE,GET") == 0) {
        on_motion_profile_get();
        return;
    }

    if (strcmp(rx->buf, "PROFILE,PRECISION") == 0) {
        on_motion_profile_set(VISION_PROFILE_PRECISION);
        return;
    }

    if (strcmp(rx->buf, "PROFILE,STANDARD") == 0) {
        on_motion_profile_set(VISION_PROFILE_STANDARD);
        return;
    }

    if (strcmp(rx->buf, "PROFILE,TASK2TEST") == 0) {
        on_motion_profile_set(VISION_PROFILE_TASK2TEST);
        return;
    }

    if (strncmp(rx->buf, "PROFILE,", 8) == 0) {
        on_bad_frame(rx->buf);
        return;
    }

    if (strcmp(rx->buf, "SFF,GET") == 0) {
        on_servo_feed_forward_get();
        return;
    }

    if (strncmp(rx->buf, "SFF,", 4) == 0) {
        const char *p = rx->buf + 4;

        if (((p[0] == 'F') || (p[0] == 'B')) &&
            (p[1] == ',')) {
            const char *rest = p + 2;
            char *end;

            float startSpeed = strtof(rest, &end);

            if ((end == rest) || (*end != ',')) {
                on_bad_frame(rx->buf);
                return;
            }

            rest = end + 1;

            float fullSpeed = strtof(rest, &end);

            if ((end == rest) || (*end != ',')) {
                on_bad_frame(rx->buf);
                return;
            }

            rest = end + 1;

            float maxUs = strtof(rest, &end);

            if ((end == rest) || (*end != ',')) {
                on_bad_frame(rx->buf);
                return;
            }

            rest = end + 1;

            float enableValue = strtof(rest, &end);

            if ((end == rest) || (*end != '\0')) {
                on_bad_frame(rx->buf);
                return;
            }

            if (!(startSpeed >= SFF_START_SPEED_MIN && startSpeed <= SFF_START_SPEED_MAX) ||
                !(fullSpeed >= SFF_FULL_SPEED_MIN && fullSpeed <= SFF_FULL_SPEED_MAX) ||
                !(maxUs >= SFF_MAX_US_MIN && maxUs <= SFF_MAX_US_MAX) ||
                !((enableValue == 0.0f) || (enableValue == 1.0f))) {
                on_bad_frame(rx->buf);
                return;
            }

            on_servo_feed_forward_set(
                p[0],
                startSpeed,
                fullSpeed,
                maxUs,
                (enableValue == 1.0f) ? 1 : 0
            );

            return;
        }

        on_bad_frame(rx->buf);
        return;
    }

    if (strcmp(rx->buf, "PRESTEER,GET") == 0) {
        on_pre_steer_get();
        return;
    }

    if (strncmp(rx->buf, "PRESTEER,", 9) == 0) {
        const char *p = rx->buf + 9;
        char *end;

        float durationMs = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float enableValue = strtof(p, &end);

        if ((end == p) || (*end != '\0')) {
            on_bad_frame(rx->buf);
            return;
        }

        if (!(durationMs >= PRESTEER_DURATION_MS_MIN && durationMs <= PRESTEER_DURATION_MS_MAX) ||
            !((enableValue == 0.0f) || (enableValue == 1.0f))) {
            on_bad_frame(rx->buf);
            return;
        }

        on_pre_steer_set(
            durationMs,
            (enableValue == 1.0f) ? 1 : 0
        );

        return;
    }

    if (strcmp(rx->buf, "TURNBRAKE,GET") == 0) {
        on_turn_brake_get();
        return;
    }

    if (strncmp(rx->buf, "TURNBRAKE,", 10) == 0) {
        const char *p = rx->buf + 10;
        char *end;

        float lookaheadMs = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float enableValue = strtof(p, &end);

        if ((end == p) || (*end != '\0')) {
            on_bad_frame(rx->buf);
            return;
        }

        if (!(lookaheadMs >= TURNBRAKE_LOOKAHEAD_MS_MIN && lookaheadMs <= TURNBRAKE_LOOKAHEAD_MS_MAX) ||
            !((enableValue == 0.0f) || (enableValue == 1.0f))) {
            on_bad_frame(rx->buf);
            return;
        }

        on_turn_brake_set(
            lookaheadMs,
            (enableValue == 1.0f) ? 1 : 0
        );

        return;
    }

    if (strcmp(rx->buf, "TURNSPEED,GET") == 0) {
        on_turn_speed_get();
        return;
    }

    if (strncmp(rx->buf, "TURNSPEED,", 10) == 0) {
        uint32_t rateValue;

        if (vision_rx_parse_bounded_u32(
                rx->buf + 10,
                TURNSPEED_COUNTS_PER_SEC_MIN,
                TURNSPEED_COUNTS_PER_SEC_MAX,
                &rateValue)) {
            on_turn_speed_set(rateValue);
            return;
        }

        on_bad_frame(rx->buf);
        return;
    }

    if (strcmp(rx->buf, "ACKERMANN,GET") == 0) {
        on_ackermann_get();
        return;
    }

    if (strncmp(rx->buf, "ACKERMANN,", 10) == 0) {
        const char *p = rx->buf + 10;
        char *end;

        float k = strtof(p, &end);

        if ((end == p) || (*end != ',')) {
            on_bad_frame(rx->buf);
            return;
        }

        p = end + 1;

        float enableValue = strtof(p, &end);

        if ((end == p) || (*end != '\0')) {
            on_bad_frame(rx->buf);
            return;
        }

        if (!(k >= ACKERMANN_K_MIN && k <= ACKERMANN_K_MAX) ||
            !((enableValue == 0.0f) || (enableValue == 1.0f))) {
            on_bad_frame(rx->buf);
            return;
        }

        on_ackermann_set(
            k,
            (enableValue == 1.0f) ? 1 : 0
        );

        return;
    }

    if (strncmp(rx->buf, "TUNE,", 5) == 0) {
        const char *p = rx->buf + 5;

        if ((p[0] == 'S') &&
            (p[1] == ',') &&
            ((p[2] == 'F') || (p[2] == 'B')) &&
            (p[3] == ',')) {
            const char *secondsText = p + 4;
            char *end;
            long seconds;

            if ((*secondsText < '0') || (*secondsText > '9')) {
                on_bad_frame(rx->buf);
                return;
            }

            seconds = strtol(secondsText, &end, 10);

            if ((*end == '\0') &&
                (seconds >= 1L) &&
                (seconds <= TUNE_STEERING_MAX_SECONDS)) {
                on_tune_steering(
                    p[2],
                    (uint16_t)seconds
                );
                return;
            }

            on_bad_frame(rx->buf);
            return;
        }

        if (((p[0] == 'L') ||
             (p[0] == 'R') ||
             (p[0] == 'B')) &&
            (p[1] == ',')) {
            const char *countsText = p + 2;
            char *end;
            long counts;

            if ((*countsText != '+') &&
                (*countsText != '-') &&
                ((*countsText < '0') || (*countsText > '9'))) {
                on_bad_frame(rx->buf);
                return;
            }

            counts = strtol(countsText, &end, 10);

            if ((*end == '\0') &&
                (counts != 0L) &&
                (counts >= -TUNE_MOTOR_MAX_ABS_COUNTS) &&
                (counts <= TUNE_MOTOR_MAX_ABS_COUNTS)) {
                on_tune_motor(
                    p[0],
                    (int32_t)counts
                );
                return;
            }

            on_bad_frame(rx->buf);
            return;
        }

        on_bad_frame(rx->buf);
        return;
    }

    if (strncmp(rx->buf, "HDG,RAW,", 8) == 0) {
        uint32_t seconds;

        if (vision_rx_parse_bounded_u32(
                rx->buf + 8,
                HEADING_TEST_MIN_SECONDS,
                HEADING_TEST_MAX_SECONDS,
                &seconds)) {
            on_heading_raw_test((uint16_t)seconds);
            return;
        }

        on_bad_frame(rx->buf);
        return;
    }

    if (strncmp(rx->buf, "HDG,", 4) == 0) {
        const char *p = rx->buf + 4;
        uint32_t seconds;

        if (((p[0] == 'F') || (p[0] == 'B')) &&
            (p[1] == ',') &&
            vision_rx_parse_bounded_u32(
                p + 2,
                HEADING_TEST_MIN_SECONDS,
                HEADING_TEST_MAX_SECONDS,
                &seconds)) {
            on_heading_test(p[0], (uint16_t)seconds);
            return;
        }

        on_bad_frame(rx->buf);
        return;
    }

    if (strncmp(rx->buf, "MOV,", 4) == 0) {
        const char *p = rx->buf + 4;
        const char *amountText;
        uint32_t amount;
        uint32_t maximum;
        VisionMoveType moveType;

        if ((p[0] != 'F') && (p[0] != 'B')) {
            /* Bare L/R (and anything else) is rejected: the final
             * protocol only accepts F, B, FL, FR, BL, BR. */
            on_bad_frame(rx->buf);
            return;
        }

        if (p[1] == ',') {
            maximum = MOVE_DISTANCE_MAX_CM;
            moveType = (p[0] == 'F') ?
                VISION_MOVE_FORWARD : VISION_MOVE_BACKWARD;
            amountText = p + 2;
        } else if (((p[1] == 'L') || (p[1] == 'R')) &&
                   (p[2] == ',')) {
            maximum = MOVE_TURN_MAX_DEGREES;

            if (p[0] == 'F') {
                moveType = (p[1] == 'L') ?
                    VISION_MOVE_FORWARD_LEFT : VISION_MOVE_FORWARD_RIGHT;
            } else {
                moveType = (p[1] == 'L') ?
                    VISION_MOVE_BACKWARD_LEFT : VISION_MOVE_BACKWARD_RIGHT;
            }

            amountText = p + 3;
        } else {
            on_bad_frame(rx->buf);
            return;
        }

        if (vision_rx_parse_bounded_u32(
                amountText,
                0U,
                maximum,
                &amount)) {
            on_move(moveType, (uint16_t)amount);
            return;
        }

        on_bad_frame(rx->buf);
        return;
    }

    if (strncmp(rx->buf, "TGT,", 4) == 0) {
        const char *p = rx->buf + 4;
        uint32_t id;
        uint32_t cx;
        uint32_t cy;
        float confidence;

        if (vision_rx_parse_bounded_u32_field(
                &p,
                0U,
                255U,
                &id) &&
            vision_rx_parse_confidence_field(
                &p,
                &confidence) &&
            vision_rx_parse_bounded_u32_field(
                &p,
                0U,
                9999U,
                &cx) &&
            vision_rx_parse_bounded_u32(
                p,
                0U,
                9999U,
                &cy)) {
            on_target(
                (uint8_t)id,
                confidence,
                (uint16_t)cx,
                (uint16_t)cy
            );
            return;
        }

        on_bad_frame(rx->buf);
        return;
    }

    on_bad_frame(rx->buf);
}

void vision_rx_feed(vision_rx_t *rx, uint8_t byte)
{
    if (byte == '\n') {
        if (rx->overflow) {
            /* An invalid or over-long line is untrustworthy. */
            rx->overflow = 0;
            rx->len = 0;
            rx->pending_cr = 0;
            on_bad_frame("<overflow>");
            return;
        }

        /* A single CR is accepted only when immediately followed by LF. */
        rx->pending_cr = 0;

        /* Early exits for the frames that must not sit under the large
         * vision_rx_dispatch() stack frame (about 360 bytes at -O0) on
         * CommTask's 1 KB stack: the exact priority STOP, which is the
         * deepest firmware chain on the safety path, and the USPARK
         * control. Reached only here, i.e. only for a COMPLETE line that is
         * LF-terminated, not overflowed and not control-character-tainted,
         * so framing, CRLF handling, overflow recovery and parser reset are
         * exactly those of every other command. Matching is the same exact,
         * case-sensitive strcmp the dispatcher used; STOP is no longer
         * checked again inside vision_rx_dispatch() (no duplicate parsing).
         * Anything else -- including "STOP,1", "stop", " STOP", "STOPCTRL,..."
         * -- falls through to the dispatcher unchanged and is rejected or
         * handled exactly as before. */
        rx->buf[rx->len] = '\0';

        if (strcmp(rx->buf, "STOP") == 0) {
            on_stop();
            rx->len = 0;
            return;
        }

        if (vision_rx_dispatch_us_park(rx)) {
            rx->len = 0;
            return;
        }

        if (vision_rx_dispatch_ir(rx)) {
            rx->len = 0;
            return;
        }

        if (vision_rx_dispatch_fir(rx)) {
            rx->len = 0;
            return;
        }

        vision_rx_dispatch(rx);
        rx->len = 0;
        return;
    }

    if (rx->overflow) {
        return;
    }

    if (rx->pending_cr) {
        rx->pending_cr = 0;
        rx->overflow = 1;
        return;
    }

    if (byte == '\r') {
        rx->pending_cr = 1;
        return;
    }

    if ((byte < 0x20U) || (byte > 0x7EU)) {
        rx->overflow = 1;
        return;
    }

    if (rx->len >= VISION_RX_BUF - 1) {
        /* Mark and keep consuming until the newline resynchronises us. */
        rx->overflow = 1;
        return;
    }

    rx->buf[rx->len++] = (char)byte;
}
