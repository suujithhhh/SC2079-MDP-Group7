/* vision_rx.h -- STM32 side of the Pi vision link. See vision_rx.c. */

#ifndef VISION_RX_H
#define VISION_RX_H

#include <stdint.h>

/* Over-long lines are rejected; 32 keeps the receive state compact and
 * leaves headroom for the fixed-format vision and diagnostic commands. */
/* Runtime gains stay finite and printable without printf float support. */
#define VISION_RX_BUF      32
#define VISION_PID_GAIN_MAX 4294967040.0f

typedef struct {
    char    buf[VISION_RX_BUF];
    uint8_t len;
    uint8_t overflow;
    uint8_t pending_cr;
} vision_rx_t;

/* Typed movement kind for on_move(), kept local to the parser so this
 * module does not depend on the firmware's motion-subsystem headers.
 * app_comm.c maps each value onto its own MotionType once. */
typedef enum {
    VISION_MOVE_FORWARD = 0,
    VISION_MOVE_BACKWARD,
    VISION_MOVE_FORWARD_LEFT,
    VISION_MOVE_FORWARD_RIGHT,
    VISION_MOVE_BACKWARD_LEFT,
    VISION_MOVE_BACKWARD_RIGHT
} VisionMoveType;

/* Named runtime configuration profile kind for on_motion_profile_set(),
 * kept local to the parser so this module does not depend on the
 * firmware's motion-subsystem headers. app_comm.c maps each value onto
 * its own AppMotionProfile once. There is no VISION_PROFILE_CUSTOM --
 * CUSTOM is a report-only state (see on_motion_profile_get()), never a
 * settable target. */
typedef enum {
    VISION_PROFILE_PRECISION = 0,
    VISION_PROFILE_STANDARD
} VisionProfileType;

void vision_rx_init(vision_rx_t *rx);

/* Feed one received byte. Calls exactly one of the callbacks below
 * when a complete line arrives. */
void vision_rx_feed(vision_rx_t *rx, uint8_t byte);

/* --- implement these in your firmware --------------------------- */

/* A detection. conf is 0.0-1.0; cx/cy are pixel coords of the centre. */
extern void on_target(uint8_t id, float conf, uint16_t cx, uint16_t cy);

/* A movement command from the Pi. This is the MDP checklist A.1.3 path:
 * the Pi is master, the STM32 is the slave that executes the motion.
 *
 *   VISION_MOVE_FORWARD / VISION_MOVE_BACKWARD          -> amount is a distance in centimetres, 0-999
 *   VISION_MOVE_FORWARD_LEFT / VISION_MOVE_FORWARD_RIGHT
 *   VISION_MOVE_BACKWARD_LEFT / VISION_MOVE_BACKWARD_RIGHT -> amount is a turn in degrees, 0-360
 */
extern void on_move(VisionMoveType type, uint16_t amount);

/* PC/bench encoder diagnostics. These are optional maintenance commands,
 * not part of the Pi mission command set. */
extern void on_encoder_reset(void);
extern void on_encoder_get(void);

/* PC/bench motor PWM sweep: duty is 0-99. */
extern void on_pwm_test(uint8_t duty);

/* PC/bench wheel-speed sample: duty is 0-99. Completion speed values are
 * signed centimetres per second, scaled by 10 (not RPM). */
extern void on_speed_test(uint8_t duty);

/* PC/bench IMU diagnostics and heading-hold test. */
extern void on_imu_get(void);
extern void on_imu_zero(void);
extern void on_imu_flip(void);
extern void on_heading_test(char dir, uint16_t seconds);
extern void on_heading_raw_test(uint16_t seconds);

/* Servo bench calibration. Pulse is 1230-1830 microseconds. */
extern void on_servo_set(uint16_t pulse_us);
extern void on_servo_get(void);

/* Inference ran and found nothing. */
extern void on_no_target(void);

/* The Pi's vision process (re)started. Reset any latched target state. */
extern void on_vision_ready(void);

/* A line arrived that did not parse. Log it or ignore it. */
extern void on_bad_frame(const char *line);

extern void on_pid_set(
    char target,
    float kp,
    float ki,
    float kd
);

extern void on_pid_get(void);

/* Runtime PID tuning tests. Motor counts are signed and relative. */
extern void on_tune_motor(char target, int32_t counts);
extern void on_tune_steering(char dir, uint16_t seconds);

/* Stage A1 rear-wheel synchronization tuning.
 *   WSYNC,<gain>,<maxDuty>,<enable>   set params (idle-only, gated by app_comm)
 *   WSYNC,GET                         report current params
 * gain    : float, 0.0 .. 1.0   (duty per count of progress mismatch)
 * maxDuty : float, 0.0 .. 39.0  (absolute clamp on the sync trim)
 * enable  : 0 or 1
 */
extern void on_wheel_sync_set(float gain, float maxDuty, int enable);
extern void on_wheel_sync_get(void);

/* Stage A2 direct gyro-Z heading rate-damping tuning.
 *   HCTRL,<krate>,<maxCorrection>,<enable>   set params (idle-only, gated by app_comm)
 *   HCTRL,GET                                report current params
 * krate         : float, 0.0 .. 30.0   (us of servo trim per deg/s of yaw rate)
 * maxCorrection : float, 0.0 .. 300.0  (absolute clamp on the rate-damping term)
 * enable        : 0 or 1
 * steeringPID's own Kp/Ki/Kd (PID,S) are untouched by this command.
 */
extern void on_heading_rate_set(float krate, float maxCorrection, int enable);
extern void on_heading_rate_get(void);

/* Stage A3.1 reverse-torque guard, scoped to the existing per-wheel
 * approach zone only.
 *   RTGUARD,<enable>   set (idle-only, gated by app_comm)
 *   RTGUARD,GET        report current state
 * enable : 0 or 1
 */
extern void on_reverse_torque_guard_set(int enable);
extern void on_reverse_torque_guard_get(void);

/* Stage A3.2 common-profile deceleration controller.
 *   DPROF,<decelCounts>,<approachDuty>,<enable>   set params (idle-only, gated by app_comm)
 *   DPROF,GET                                     report current params
 * decelCounts  : float, 300.0 .. 3000.0   (chassis remaining-counts DECEL entry threshold)
 * approachDuty : float, 40.0 .. 90.0      (guaranteed minimum/final commanded magnitude)
 * enable       : 0 or 1
 * leftPID/rightPID gains and WSYNC's own algorithm/settings are untouched by this command.
 */
extern void on_decel_profile_set(float decelCounts, float approachDuty, int enable);
extern void on_decel_profile_get(void);

/* Stage A3.3a fixed-target closed-loop chassis speed controller.
 *   SPCTRL,<target>,<Kp>,<Ki>,<enable>   set params (idle-only, gated by app_comm)
 *   SPCTRL,GET                           report current params
 * target : float, 0.0 .. 8000.0   (chassis counts/s)
 * Kp     : float, 0.0 .. 1.0
 * Ki     : float, 0.0 .. 1.0
 * enable : 0 or 1
 * Mutually exclusive with DPROF; rejected by app_comm.c at SET time in
 * both directions if the other is already enabled.
 */
extern void on_speed_control_set(float targetSpeed, float kp, float ki, int enable);
extern void on_speed_control_get(void);

/* Stage A3.3b distance-based linear speed profile, layered on SPCTRL.
 *   SPPROF,<decelStartCounts>,<approachSpeed>,<enable>   set params (idle-only, gated by app_comm)
 *   SPPROF,GET                                           report current params
 * decelStartCounts : float, 300.0 .. 6000.0   (chassis remaining-counts profile entry threshold)
 * approachSpeed    : float, 1.0 .. 8000.0     (chassis counts/s target near the end of the move)
 * enable           : 0 or 1
 * Only ever has an effect while SPCTRL is enabled -- never claims magnitude
 * ownership itself. approachSpeed must never exceed the currently-configured
 * SPCTRL cruise target; SPPROF,SET and SPCTRL,SET each reject the other's
 * command if it would violate that, rather than silently changing either.
 */
extern void on_speed_profile_set(float decelStartCounts, float approachSpeed, int enable);
extern void on_speed_profile_get(void);

/* Stage A3.3c final controlled stop gate, layered on SPCTRL. ADDITIVE
 * only -- never replaces the existing position-tolerance/target-crossing
 * completion path; when the gate fires it is OR'd alongside that existing
 * path into the same, unmodified active-brake transition.
 *   STOPCTRL,<brakeLeadCounts>,<brakeEntrySpeed>,<enable>   set params (idle-only, gated by app_comm)
 *   STOPCTRL,GET                                            report current params
 * brakeLeadCounts  : float, 20.0 .. 500.0     (per-wheel directed-remaining threshold, counts)
 * brakeEntrySpeed  : float, 500.0 .. 2500.0   (filtered chassis speed magnitude threshold, counts/s)
 * enable           : 0 or 1
 * Only ever has an effect while SPCTRL is enabled -- never claims magnitude
 * ownership itself. No cross-command invariant with SPPROF/SPCTRL is
 * enforced; brakeEntrySpeed relative to SPPROF's approachSpeed is a valid
 * tuning choice, not a configuration error.
 */
extern void on_stop_control_set(float brakeLeadCounts, float brakeEntrySpeed, int enable);
extern void on_stop_control_get(void);

/* Stage A4 distance (counts/cm) calibration, for MOV,F/B target generation
 * only -- never touches WSYNC/HCTRL/SPCTRL/SPPROF/STOPCTRL/DPROF, PID
 * tuning, braking, turns, or the SPEED_TEST diagnostic's own use of
 * ENCODER_COUNTS_PER_CM.
 *   MOVCAL,<slopeCountsPerCm>,<offsetCounts>,<enable>   set params (idle-only, gated by app_comm)
 *   MOVCAL,GET                                          report current params
 * slopeCountsPerCm : float, 50.0 .. 100.0    (counts per commanded centimetre)
 * offsetCounts     : float, -500.0 .. 500.0  (fixed additive correction, counts)
 * enable           : 0 or 1
 * A commanded distance of 0 cm always resolves to exactly 0 target counts
 * regardless of this setting. app_comm.c rejects a SET that would let
 * slopeCountsPerCm+offsetCounts fall to zero or below (checked only when
 * enabling), since that would make even the smallest non-zero command
 * (1 cm) resolve to zero or negative target counts.
 */
extern void on_distance_cal_set(float slopeCountsPerCm, float offsetCounts, int enable);
extern void on_distance_cal_get(void);

/* Named runtime configuration profile, bundling Steering PID (PID,S) +
 * WSYNC + HCTRL + RTGUARD + DPROF + SPCTRL + SPPROF + STOPCTRL + MOVCAL
 * into one atomic apply.
 *   PROFILE,PRECISION   apply the PRECISION profile (idle-only, gated by app_comm)
 *   PROFILE,STANDARD    apply the STANDARD profile (idle-only, gated by app_comm)
 *   PROFILE,GET         report the currently active profile
 * PRECISION is the power-on default. If any individual SET command above
 * (including PID,S) changes one of the profile's own fields after a
 * named profile was loaded, the firmware no longer claims that exact
 * profile is active -- on_motion_profile_get() then reports CUSTOM
 * instead, until the next PROFILE,PRECISION or PROFILE,STANDARD.
 */
extern void on_motion_profile_set(VisionProfileType profile);
extern void on_motion_profile_get(void);

/* Stage SFF1/SFF2 speed-aware servo feed-forward, layered on the
 * existing heading PID + HCTRL output during straight MOV,F/MOV,B only
 * -- independent per direction, same formula. Profile-owned as of
 * Stage SFF2: part of both PROFILE,PRECISION and PROFILE,STANDARD (see
 * AppMotion_ApplyProfile() in app_motion.c) -- a direct SET call marks
 * the active profile CUSTOM, like every other profile-owned command.
 *   SFF,<F|B>,<startSpeed>,<fullSpeed>,<maxUs>,<enable>   set params for one direction (idle-only, gated by app_comm)
 *   SFF,GET                                               report both directions' current params
 * startSpeed : float, 0.0 .. 8000.0    (chassis counts/s where the ramp begins)
 * fullSpeed  : float, 0.0 .. 8000.0    (chassis counts/s where the ramp reaches maxUs; must exceed startSpeed)
 * maxUs      : float, -100.0 .. 100.0  (signed servo-pulse feed-forward at/above fullSpeed)
 * enable     : 0 or 1
 * steeringPID's own Kp/Ki/Kd (PID,S) and HCTRL are untouched by this
 * command.
 */
extern void on_servo_feed_forward_set(char direction, float startSpeed, float fullSpeed, float maxUs, int enable);
extern void on_servo_feed_forward_get(void);

/* PRE_STEER: non-blocking wait ahead of MOV,FL/FR/BL/BR's existing
 * APP_MOTION_TURN logic -- servo is commanded once, directly, to the
 * fixed max-safe turn-servo endpoint (1000us for FL/BR, 2060us for
 * FR/BL), motors are held at zero output (no active braking), wheel
 * targets do not ramp, for durationMs before starting today's
 * unmodified turn logic (which then holds the servo at that same
 * fixed endpoint throughout, not just at entry). Not part of
 * PROFILE,PRECISION or PROFILE,STANDARD.
 *   PRESTEER,<durationMs>,<enable>   set params (idle-only, gated by app_comm)
 *   PRESTEER,GET                     report current params
 * durationMs : float, 0.0 .. 1000.0   (pre-turn wait, milliseconds; 0 is valid and distinct from disabled)
 * enable     : 0 or 1
 * Enabled by default at 150.0ms, the physically validated boot default
 * (see AppMotion_Init() in app_motion.c). Does not touch
 * targetHeadingDeg's sign logic, leftPID/rightPID, the runtime base
 * turn rate (TURNSPEED), or settle/completion tolerance.
 */
extern void on_pre_steer_set(float durationMs, int enable);
extern void on_pre_steer_get(void);

/* TURNSPEED: replaces the previously-hardcoded base turn rate (2000
 * counts/sec) that APP_MOTION_TURN ramps targetLeftCounts/
 * targetRightCounts by. Turn-only (MOV,FL/FR/BL/BR). Not part of
 * PROFILE,PRECISION or PROFILE,STANDARD.
 *   TURNSPEED,<countsPerSec>   set base turn rate (idle-only, gated by app_comm)
 *   TURNSPEED,GET              report current base turn rate
 * countsPerSec : integer, 500 .. 2500   (rear-wheel target-position ramp rate, counts/sec)
 * Default 1700, the physically validated boot default (see
 * AppMotion_Init() in app_motion.c). Does not touch targetHeadingDeg,
 * the fixed turn-servo endpoint (PRESTEER above), leftPID/rightPID's
 * equations, or settle/completion tolerance.
 */
extern void on_turn_speed_set(uint32_t countsPerSec);
extern void on_turn_speed_get(void);

/* Ackermann-v2: redistributes the runtime TURNSPEED base rate between
 * the inner/outer rear wheel targets during APP_MOTION_TURN --
 * innerRate=base*(1-k), outerRate=base*(1+k), average always equals
 * the current TURNSPEED. Turn-only (MOV,FL/FR/BL/BR). Not part of
 * PROFILE,PRECISION or PROFILE,STANDARD.
 *   ACKERMANN,<k>,<enable>   set params (idle-only, gated by app_comm)
 *   ACKERMANN,GET            report current params
 * k      : float, 0.0 .. 0.5   (differential factor)
 * enable : 0 or 1
 * Enabled by default at k=0.270, the physically validated boot default
 * (see AppMotion_Init() in app_motion.c). Does not touch leftPID/
 * rightPID's equations, the motor min-duty floor logic, steeringPID,
 * the fixed turn-servo endpoint, PRE_STEER, or TURNSPEED's own value
 * (only reads it as a base).
 */
extern void on_ackermann_set(float k, int enable);
extern void on_ackermann_get(void);

/*
 * TURNBRAKE,<lookaheadMs>,<enable>
 * TURNBRAKE,GET
 *
 * Turn-only predictive brake trigger, additive to the existing heading-
 * tolerance/5-cycle dwell completion. lookaheadMs range enforced here:
 * [TURNBRAKE_LOOKAHEAD_MS_MIN, TURNBRAKE_LOOKAHEAD_MS_MAX] below (also
 * defensively re-clamped inside AppMotion_SetTurnBrake() itself).
 * Enabled by default at 135.0ms, the physically validated boot default
 * (see AppMotion_Init() in app_motion.c); when disabled, turn
 * completion is byte-identical to before this command existed.
 */
extern void on_turn_brake_set(float lookaheadMs, int enable);
extern void on_turn_brake_get(void);

#endif /* VISION_RX_H */
