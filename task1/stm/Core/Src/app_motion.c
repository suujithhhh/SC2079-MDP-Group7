/*
 * app_motion.c
 *
 *  Created on: 2 Sept 2026
 *      Author: ksuji
 */


#include "app_motion.h"

#include "app_pid.h"
#include "app_motor.h"
#include "app_encoder.h"
#include "app_servo.h"
#include "app_imu.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stddef.h>

#define POSITION_TOLERANCE_COUNTS  15.0f

#define SETTLED_CYCLES_REQUIRED    5U

#define LEFT_MOTOR_MIN_DUTY        60
#define RIGHT_MOTOR_MIN_DUTY  60

#define APPROACH_ZONE_COUNTS        300.0f

/* APP_MOTION_DISTANCE stall recovery only: rescues a motor that has
 * genuinely stopped (near-zero encoder motion) while still materially
 * away from target and under the useful-duty floor. Never engages while
 * the motor is actually moving/braking, so the approach-zone/derivative
 * braking curve above is untouched whenever it is already working.
 * Conservative starting values, NOT verified physical calibration --
 * confirm on the bench (no spurious trigger on a normal F/B braking
 * tail; prompt recovery on a genuine stall) before trusting them. */
#define STALL_MOTION_THRESHOLD_COUNTS   2.0f
#define STALL_DETECT_CYCLES             10U
/* Directed (toward-target) encoder progress a wheel must accumulate
 * since its recovery episode began before it is considered recovered
 * and released back to normal PID for the rest of the episode. Initial
 * bench value derived from the two constants above (one debounce
 * window's worth of real motion); tune on the bench if recovery still
 * releases a wheel too early/late. */
#define STALL_RECOVERY_EXIT_PROGRESS_COUNTS 60.0f


#define TURN_HEADING_TOLERANCE_DEG      2.0f
/* Runtime-overridable base turn rate (TURNSPEED). 2000 was the exact
 * value this originally replaced (the previously-hardcoded
 * TURN_TARGET_RATE_COUNTS_PER_SEC); 1700 is now the boot default, part
 * of the combined turn configuration (PRESTEER=150,1/ACKERMANN=0.270,1/
 * TURNBRAKE=135,1, see AppMotion_Init()) initially tuned from three
 * FL90 bench runs and subsequently physically tested across FL/FR/BL/BR
 * at commanded angles of 10/30/45/60/75/90 degrees. This validation
 * record applies specifically to TURNSPEED=1700 with PRESTEER=150,1,
 * ACKERMANN=0.270,1, and TURNBRAKE=135,1; earlier experimental
 * configurations are outside this validation claim. Range [500,2500] enforced
 * by both the parser (vision_rx.c) and defensively by
 * AppMotion_SetTurnSpeed() itself -- see its own doc comment for the
 * safety reasoning (leftPID/rightPID's own output clamp bounds the
 * motor command independent of how large the target-rate magnitude
 * is). */
#define DEFAULT_TURN_SPEED_COUNTS_PER_SEC 1700U
#define TURN_SPEED_COUNTS_PER_SEC_MIN      500U
#define TURN_SPEED_COUNTS_PER_SEC_MAX     2500U

/* Ackermann-v2: RAM-only, turn-only, NOT profile-owned. Redistributes
 * the runtime TURNSPEED base rate (AppMotion_GetTurnSpeed(), NOT a
 * hardcoded value) between the inner/outer rear wheel targets -- see
 * AppMotion_SelectAckermannRates(). k=0.5 keeps innerRate>=50% and
 * outerRate<=150% of whatever TURNSPEED currently is; confirmed safe
 * via the same reasoning already established for TURNSPEED itself:
 * leftPID/rightPID's own output clamp ([-99,99], AppMotion_Init())
 * bounds the motor command independent of how large the target-rate
 * magnitude is -- a larger k (or a larger TURNSPEED) only makes the
 * setpoint move faster, never pushes the PID output outside its
 * existing clamp. Enabled by default at k=0.270, part of the combined
 * turn configuration (see AppMotion_Init()) initially tuned from three
 * FL90 bench runs and subsequently physically tested across FL/FR/BL/BR
 * at commanded angles of 10/30/45/60/75/90 degrees. This validation
 * record applies specifically to TURNSPEED=1700 with PRESTEER=150,1,
 * ACKERMANN=0.270,1, and TURNBRAKE=135,1; earlier experimental
 * configurations are outside this validation claim. */
#define DEFAULT_ACKERMANN_ENABLED  true
#define DEFAULT_ACKERMANN_K        0.270f
#define ACKERMANN_K_MIN            0.0f
#define ACKERMANN_K_MAX            0.5f

/* TURNBRAKE: RAM-only, turn-only, NOT profile-owned. Predictive brake
 * trigger, additive to (never replacing) the existing heading-tolerance/
 * 5-cycle dwell completion in case APP_MOTION_TURN: -- see
 * AppMotion_SetTurnBrake()'s own doc comment (app_motion.h) for the full
 * eligibility condition and AppMotionTurnBrakeSnapshot's doc comment for
 * the diagnostic mailbox.
 *
 * 135.0ms is an EXPERIMENTAL initial value derived only from three FL90
 * bench runs at TURNSPEED=1700/ACKERMANN=0.270,1/PRESTEER=150,1: those
 * runs showed brake onset around 90.5-91 deg (the existing dwell's own
 * ~40-50ms detection lag) followed by an additional ~6.6 degrees of
 * rotation while the existing MOTION_BRAKE_DURATION_MS (120ms) active
 * brake ran, for a total average overshoot of ~7.3 degrees at a
 * near-target yaw rate of ~52-56 deg/s -- this does NOT show that yaw
 * rate stays constant throughout that 120ms window, only that the
 * complete braking response (whatever its own internal profile) produces
 * that ~6.6-degree residual rotation under this one tested condition.
 * 135ms = roughly 120-125ms (aiming to cover that measured brake-phase
 * travel) plus roughly one extra MotionTask update's worth of margin for
 * the required two-consecutive-sample confirmation below (the trigger
 * cannot fire until a SECOND consecutive qualifying tick, so the
 * earliest possible actual trigger trails the instant the condition
 * first became true by about one update). MotionTask calls
 * AppMotion_Update() from a osDelay(10) loop (main.c) -- ticks are
 * approximately, not exactly, 10ms apart; this value and this comment
 * do not claim exact timing. This same 135.0ms value has since been
 * physically validated across all four turn directions (see below) --
 * it is no longer just a single-direction starting point. Changing
 * speed, steering geometry, tires, driving surface, or braking
 * behavior invalidates this value and requires fresh measurement and
 * validation, not an assumption that 135.0ms still applies -- see
 * app_motion.h.
 *
 * Enabled by default at this 135.0ms value as part of the combined
 * turn configuration finalization (see AppMotion_Init()), alongside
 * TURNSPEED=1700/ACKERMANN=0.270,1/PRESTEER=150,1 -- this combined
 * configuration was subsequently physically tested across FL/FR/BL/BR
 * at commanded angles of 10/30/45/60/75/90 degrees, beyond the three
 * FL90 runs the 135ms value itself was originally derived from above.
 * This validation record applies specifically to TURNSPEED=1700 with
 * PRESTEER=150,1, ACKERMANN=0.270,1, and TURNBRAKE=135,1; earlier
 * experimental configurations are outside this validation claim. */
#define DEFAULT_TURNBRAKE_ENABLED         true
#define DEFAULT_TURNBRAKE_LOOKAHEAD_MS    135.0f
#define TURNBRAKE_LOOKAHEAD_MS_MIN          0.0f
#define TURNBRAKE_LOOKAHEAD_MS_MAX        500.0f
/* Fixed (not runtime-settable) noise-floor guard -- below this measured
 * yaw rate, the predictive condition never qualifies regardless of
 * remaining angle, so gyro bias/noise near a stationary or barely-moving
 * chassis cannot trigger it. Kept out of the TURNBRAKE command surface
 * deliberately, per the requested minimal scope (lookahead + enable
 * only). */
#define TURN_BRAKE_MIN_YAW_RATE_DPS         5.0f
/* Fixed (not runtime-settable) consecutive-sample confirmation
 * requirement, per spec. The qualifying counter is saturated at this
 * value (never allowed to keep incrementing past it) so a uint8_t
 * counter cannot wrap during an arbitrarily long qualifying streak. */
#define TURN_BRAKE_CONFIRM_CYCLES            2U

#define TUNE_STEERING_RATE_COUNTS_PER_SEC 750.0f
#define TUNE_MOTOR_MAX_ABS_COUNTS       7500
#define TUNE_STEERING_MAX_SECONDS       5
/* 7500 counts at 750 counts/s is 10 s; allow 50% settling margin. */
#define TUNE_MOTOR_TIMEOUT_SECONDS      15.0f
/* Motion-local unwrapping leaves the public encoder API unchanged. */
#define RIGHT_ENCODER_MODULUS_COUNTS    65536
#define MOV_DISTANCE_MAX_CM             999
#define MOV_TURN_MAX_DEGREES            360
#define DIAGNOSTIC_RUN_SECONDS          1.0f
#define DIAGNOSTIC_MAX_SECONDS          5
#define HDG_TEST_DUTY                   60U
/* Gyro zeroing is nominally about 1 s; allow ample scheduler/I2C margin. */
#define IMU_REQUEST_TIMEOUT_SECONDS     5.0f
#define MOTION_BRAKE_DURATION_MS        120U
/* 5.0 s base + 0.15 s/cm or 0.10 s/degree, respectively. */
#define MOV_TIMEOUT_BASE_MS             5000U
#define MOV_DISTANCE_TIMEOUT_MS_PER_CM  150U
#define MOV_TURN_TIMEOUT_MS_PER_DEGREE  100U

/* PRE_STEER: boot defaults -- physically validated (bench sweep of
 * 100/150/200/250 ms on FL90/FR90/BL90/BR90): 100 ms was weaker, 150 ms
 * reached the useful response plateau (servo visibly settled before
 * meaningful rear-wheel movement, SP held at the expected 1000/2060 us
 * saturation value throughout PRE with motors stationary), 200/250 ms
 * gave no consistent further benefit. Enabled by default as of this
 * finalization. Runtime-overridable via PRESTEER, range [0,1000]
 * enforced by the parser (vision_rx.c).
 *
 * Fixed-endpoint turn-servo stage: the PRE_STEER seed pulse is no
 * longer derived from a one-shot steeringPID evaluation -- it is
 * commanded directly from AppMotion_TurnServoEndpointUs(), the same
 * fixed 1000/2060 us endpoint the maneuver holds throughout
 * APP_MOTION_TURN. steeringPID is not evaluated at PRE_STEER entry at
 * all any more (see AppMotion_EnterTurnOrPreSteer()). */
#define DEFAULT_PRESTEER_ENABLED        true
#define DEFAULT_PRESTEER_DURATION_MS    150.0f
#define PRESTEER_DURATION_MS_MIN        0.0f
#define PRESTEER_DURATION_MS_MAX        1000.0f

#define APP_MOTION_PID_MASK_LEFT        (1U << 0)
#define APP_MOTION_PID_MASK_RIGHT       (1U << 1)
#define APP_MOTION_PID_MASK_STEERING    (1U << 2)

/* Boot defaults for the physically-validated PRECISION straight-motion
 * configuration (the team's "known-good calibration" plus the
 * SPCTRL/SPPROF/STOPCTRL/MOVCAL stack validated on F120/F10/B10) --
 * STARTUP defaults only, not compile-time-only settings: every runtime
 * setter/getter/UART command below still overrides these freely, with no
 * flash persistence. One named constant per field, referenced by both
 * each static's own initializer and AppMotion_Init(), so the two can
 * never independently drift. */
#define DEFAULT_STEERING_KP                 45.00f
#define DEFAULT_STEERING_KI                  0.00f
#define DEFAULT_STEERING_KD                  0.00f

#define DEFAULT_WSYNC_GAIN                    0.65f
#define DEFAULT_WSYNC_MAX_DUTY               15.0f
#define DEFAULT_WSYNC_ENABLED                true

#define DEFAULT_HCTRL_KRATE                   3.0f
#define DEFAULT_HCTRL_MAX_CORRECTION         60.0f
#define DEFAULT_HCTRL_ENABLED                true

#define DEFAULT_RTGUARD_ENABLED              false

#define DEFAULT_DPROF_DECEL_START_COUNTS   3000.0f
#define DEFAULT_DPROF_APPROACH_DUTY          50.0f
#define DEFAULT_DPROF_ENABLED                false

#define DEFAULT_SPCTRL_TARGET_SPEED        4000.0f
#define DEFAULT_SPCTRL_KP                     0.02f
#define DEFAULT_SPCTRL_KI                     0.03f
#define DEFAULT_SPCTRL_KD                     0.00f
#define DEFAULT_SPCTRL_ENABLED               true

#define DEFAULT_SPPROF_DECEL_START_COUNTS  4500.0f
#define DEFAULT_SPPROF_APPROACH_SPEED      1200.0f
#define DEFAULT_SPPROF_ENABLED               true

#define DEFAULT_STOPCTRL_BRAKE_LEAD_COUNTS  150.0f
#define DEFAULT_STOPCTRL_BRAKE_ENTRY_SPEED 1700.0f
#define DEFAULT_STOPCTRL_ENABLED             true

#define DEFAULT_MOVCAL_SLOPE_COUNTS_PER_CM   75.0f
#define DEFAULT_MOVCAL_OFFSET_COUNTS        100.0f
#define DEFAULT_MOVCAL_ENABLED               true

/* Stage SFF2/SFF3: boot defaults for Stage SFF1's speed-aware servo
 * feed-forward -- both directions now physically validated, so it boots
 * enabled with the validated values for both Forward and Backward,
 * matching PROFILE_PRECISION_SFF_* below exactly, same "two
 * separately-named, numerically-matched constant sets" convention
 * already used for STOPCTRL/SPCTRL/SPPROF/MOVCAL/WSYNC/HCTRL above.
 * Backward's maxUs is a different physical sign from Forward's
 * (+9 vs -12) -- expected, not a bug: each direction's own calibrated
 * correction direction is independent, never auto-inverted (see
 * AppMotion_SetServoFeedForward()/the servoPulse integration). */
#define DEFAULT_SFF_FORWARD_START_SPEED   3200.0f
#define DEFAULT_SFF_FORWARD_FULL_SPEED    4500.0f
#define DEFAULT_SFF_FORWARD_MAX_US         -12.0f
#define DEFAULT_SFF_FORWARD_ENABLED        true
#define DEFAULT_SFF_BACKWARD_START_SPEED  3200.0f
#define DEFAULT_SFF_BACKWARD_FULL_SPEED   4500.0f
#define DEFAULT_SFF_BACKWARD_MAX_US          9.0f
#define DEFAULT_SFF_BACKWARD_ENABLED        true

/* Named runtime configuration profiles (motion-profiles branch),
 * selected via PROFILE,PRECISION / PROFILE,STANDARD / PROFILE,GET (see
 * AppMotion_ApplyProfile()/AppMotion_GetProfile() below and
 * app_comm.c's on_motion_profile_set()/get()). Each profile's full field
 * set is spelled out explicitly and independently here -- PRECISION's
 * values happen to equal the DEFAULT_* boot defaults above (the same
 * physically-validated configuration used at power-on), but the two
 * constant groups are intentionally kept separate: AppMotion_Init()'s
 * boot sequence and AppMotion_ApplyProfile()'s runtime apply are
 * independent code paths that must not silently move in lockstep if one
 * is ever retuned without the other. Only SPCTRL_TARGET_SPEED,
 * SPPROF_DECEL_START_COUNTS and STOPCTRL_BRAKE_ENTRY_SPEED differ between
 * PRECISION and STANDARD -- every other field is intentionally identical
 * between the two profiles. */
#define PROFILE_PRECISION_STEERING_KP                 45.00f
#define PROFILE_PRECISION_STEERING_KI                  0.00f
#define PROFILE_PRECISION_STEERING_KD                  0.00f
#define PROFILE_PRECISION_WSYNC_GAIN                    0.65f
#define PROFILE_PRECISION_WSYNC_MAX_DUTY               15.0f
#define PROFILE_PRECISION_WSYNC_ENABLED                true
#define PROFILE_PRECISION_HCTRL_KRATE                   3.0f
#define PROFILE_PRECISION_HCTRL_MAX_CORRECTION         60.0f
#define PROFILE_PRECISION_HCTRL_ENABLED                true
#define PROFILE_PRECISION_RTGUARD_ENABLED              false
#define PROFILE_PRECISION_DPROF_DECEL_START_COUNTS   3000.0f
#define PROFILE_PRECISION_DPROF_APPROACH_DUTY          50.0f
#define PROFILE_PRECISION_DPROF_ENABLED                false
#define PROFILE_PRECISION_SPCTRL_TARGET_SPEED        4000.0f
#define PROFILE_PRECISION_SPCTRL_KP                     0.02f
#define PROFILE_PRECISION_SPCTRL_KI                     0.03f
#define PROFILE_PRECISION_SPCTRL_ENABLED               true
#define PROFILE_PRECISION_SPPROF_DECEL_START_COUNTS  4500.0f
#define PROFILE_PRECISION_SPPROF_APPROACH_SPEED      1200.0f
#define PROFILE_PRECISION_SPPROF_ENABLED               true
#define PROFILE_PRECISION_STOPCTRL_BRAKE_LEAD_COUNTS  150.0f
#define PROFILE_PRECISION_STOPCTRL_BRAKE_ENTRY_SPEED 1700.0f
#define PROFILE_PRECISION_STOPCTRL_ENABLED             true
#define PROFILE_PRECISION_MOVCAL_SLOPE_COUNTS_PER_CM   75.0f
#define PROFILE_PRECISION_MOVCAL_OFFSET_COUNTS        100.0f
#define PROFILE_PRECISION_MOVCAL_ENABLED               true
#define PROFILE_PRECISION_SFF_FORWARD_START_SPEED    3200.0f
#define PROFILE_PRECISION_SFF_FORWARD_FULL_SPEED     4500.0f
#define PROFILE_PRECISION_SFF_FORWARD_MAX_US          -12.0f
#define PROFILE_PRECISION_SFF_FORWARD_ENABLED         true
#define PROFILE_PRECISION_SFF_BACKWARD_START_SPEED   3200.0f
#define PROFILE_PRECISION_SFF_BACKWARD_FULL_SPEED    4500.0f
#define PROFILE_PRECISION_SFF_BACKWARD_MAX_US           9.0f
#define PROFILE_PRECISION_SFF_BACKWARD_ENABLED        true

#define PROFILE_STANDARD_STEERING_KP                  45.00f
#define PROFILE_STANDARD_STEERING_KI                   0.00f
#define PROFILE_STANDARD_STEERING_KD                   0.00f
#define PROFILE_STANDARD_WSYNC_GAIN                     0.65f
#define PROFILE_STANDARD_WSYNC_MAX_DUTY                15.0f
#define PROFILE_STANDARD_WSYNC_ENABLED                 true
#define PROFILE_STANDARD_HCTRL_KRATE                    3.0f
#define PROFILE_STANDARD_HCTRL_MAX_CORRECTION          60.0f
#define PROFILE_STANDARD_HCTRL_ENABLED                 true
#define PROFILE_STANDARD_RTGUARD_ENABLED               false
#define PROFILE_STANDARD_DPROF_DECEL_START_COUNTS    3000.0f
#define PROFILE_STANDARD_DPROF_APPROACH_DUTY           50.0f
#define PROFILE_STANDARD_DPROF_ENABLED                 false
#define PROFILE_STANDARD_SPCTRL_TARGET_SPEED         6000.0f
#define PROFILE_STANDARD_SPCTRL_KP                      0.02f
#define PROFILE_STANDARD_SPCTRL_KI                      0.03f
#define PROFILE_STANDARD_SPCTRL_ENABLED                true
#define PROFILE_STANDARD_SPPROF_DECEL_START_COUNTS   6000.0f
#define PROFILE_STANDARD_SPPROF_APPROACH_SPEED       1200.0f
#define PROFILE_STANDARD_SPPROF_ENABLED                true
#define PROFILE_STANDARD_STOPCTRL_BRAKE_LEAD_COUNTS   150.0f
#define PROFILE_STANDARD_STOPCTRL_BRAKE_ENTRY_SPEED  1800.0f
#define PROFILE_STANDARD_STOPCTRL_ENABLED              true
#define PROFILE_STANDARD_MOVCAL_SLOPE_COUNTS_PER_CM    75.0f
#define PROFILE_STANDARD_MOVCAL_OFFSET_COUNTS         100.0f
#define PROFILE_STANDARD_MOVCAL_ENABLED                true
#define PROFILE_STANDARD_SFF_FORWARD_START_SPEED     3200.0f
#define PROFILE_STANDARD_SFF_FORWARD_FULL_SPEED      4500.0f
#define PROFILE_STANDARD_SFF_FORWARD_MAX_US           -12.0f
#define PROFILE_STANDARD_SFF_FORWARD_ENABLED          true
#define PROFILE_STANDARD_SFF_BACKWARD_START_SPEED    3200.0f
#define PROFILE_STANDARD_SFF_BACKWARD_FULL_SPEED     4500.0f
#define PROFILE_STANDARD_SFF_BACKWARD_MAX_US            9.0f
#define PROFILE_STANDARD_SFF_BACKWARD_ENABLED         true

static PIDController leftPID;
static PIDController rightPID;
static PIDController steeringPID;

/* Stage A1: rear-wheel synchronization trim (cruise portion of
 * APP_MOTION_DISTANCE only). gain/maxDuty are runtime-tunable via the
 * WSYNC command; maxDuty is hard-limited to 39 by the parser so the
 * leading wheel can never be driven below the existing 60-duty cruise
 * floor (99 - 39 = 60). RAM only; a power cycle restores these.
 *
 * Boot-enabled by default (DEFAULT_WSYNC_*) -- physically validated;
 * still fully runtime-overridable via WSYNC. */
static bool  wheelSyncEnabled  = DEFAULT_WSYNC_ENABLED;
static float wheelSyncGain     = DEFAULT_WSYNC_GAIN;
static float wheelSyncMaxDuty  = DEFAULT_WSYNC_MAX_DUTY;

/* Stage A2: direct gyro-Z heading rate damping, added on top of
 * steeringPID's existing (unmodified) P-only output. maxCorrection is a
 * clamp, not a claim it can never dominate the base term -- near zero
 * heading error the rate term can legitimately exceed it. RAM only; a
 * power cycle restores these.
 *
 * Boot-enabled by default (DEFAULT_HCTRL_*) -- physically validated;
 * still fully runtime-overridable via HCTRL. */
static bool  headingCtrlEnabled       = DEFAULT_HCTRL_ENABLED;
static float headingCtrlKrate         = DEFAULT_HCTRL_KRATE;
static float headingCtrlMaxCorrection = DEFAULT_HCTRL_MAX_CORRECTION;

/* Stage A3.1: reverse-torque guard experiment. Scoped strictly to the
 * existing per-wheel approach zone (POSITION_TOLERANCE_COUNTS < |error|
 * <= APPROACH_ZONE_COUNTS) -- never touches cruise (> APPROACH_ZONE_
 * COUNTS, where WSYNC operates) or the tolerance band (<= POSITION_
 * TOLERANCE_COUNTS, already forced to 0). Disabled by default
 * (DEFAULT_RTGUARD_ENABLED) -- rtGuardEnabled == false means the guard
 * block never executes, so the two existing per-wheel tolerance/floor
 * chains are byte-for-behavior identical to today. RAM only; a power
 * cycle restores this. */
static bool rtGuardEnabled = DEFAULT_RTGUARD_ENABLED;

/* Stage A3.2: common-profile deceleration controller. Disabled by
 * default (DEFAULT_DPROF_ENABLED) -- decelProfileEnabled == false means
 * the profile/enforcement blocks never execute, so leftCommand/
 * rightCommand are computed from leftOutput/rightOutput exactly as today
 * (byte-for-behavior identical). decelStartCounts/decelApproachDuty
 * (DEFAULT_DPROF_*) are runtime-tunable via DPROF so bench tuning never
 * requires a reflash. RAM only; a power cycle restores these. */
static bool  decelProfileEnabled  = DEFAULT_DPROF_ENABLED;
static float decelStartCounts     = DEFAULT_DPROF_DECEL_START_COUNTS;
static float decelApproachDuty    = DEFAULT_DPROF_APPROACH_DUTY;

/* Stage A3.3a: fixed-target closed-loop chassis speed controller.
 * Boot-enabled by default (DEFAULT_SPCTRL_*) -- physically validated
 * (PRECISION config); still fully runtime-overridable via SPCTRL.
 * speedPID is a genuinely new use of PID_Update's Ki path -- every other PID
 * instance in this firmware has Ki=0.00, so this is the first time the
 * existing conditional-integration anti-windup logic is functionally
 * exercised. speedPID's integral limits are NOT a fixed compiled
 * constant (an earlier revision used one and it silently capped IC's
 * steady-state authority well below outputMax, see
 * AppMotion_UpdateSpeedIntegralLimits()); they are recomputed from
 * Ki/outputMax instead so the integral term always has exactly enough
 * authority to supply the controller's full positive output range on
 * its own. SPCTRL_EMA_ALPHA smooths the measured-dt jitter identified
 * in the plan's audit of MP,LV/RV, not general "noise" -- compiled, not
 * runtime-tunable, to keep the command surface minimal. RAM only; a
 * power cycle restores these. */
#define SPCTRL_EMA_ALPHA        0.3f

static PIDController speedPID;
static bool  spctrlEnabled      = DEFAULT_SPCTRL_ENABLED;
static float spctrlTargetSpeed  = DEFAULT_SPCTRL_TARGET_SPEED;

/* Direction-normalized, EMA-filtered common chassis speed (Stage
 * A3.3a). Seeded from the first valid sample after a reset rather than
 * blended from an artificial zero -- see AppMotion_ResetAllPIDs() and
 * the MOTION_FORWARD/MOTION_BACKWARD command-entry resets. */
static float filteredChassisSpeed            = 0.0f;
static bool  filteredChassisSpeedInitialized = false;

/* Stage A3.3b: distance-based linear speed profile, layered on top of
 * the validated Stage A3.3a speed PI -- never claims magnitude ownership
 * itself, only adjusts the target speed fed into speedPID.
 * sprofDecelStartCounts/sprofApproachSpeed (DEFAULT_SPPROF_*) are the
 * physically-validated PRECISION values. Boot-enabled by default -- still
 * fully runtime-overridable via SPPROF, no flash persistence. */
static bool  sprofEnabled          = DEFAULT_SPPROF_ENABLED;
static float sprofDecelStartCounts = DEFAULT_SPPROF_DECEL_START_COUNTS;
static float sprofApproachSpeed    = DEFAULT_SPPROF_APPROACH_SPEED;

/* Stage A3.3c: final controlled stop gate, layered on top of the validated
 * Stage A3.3a speed PI -- ADDITIVE only. Never replaces the existing
 * settledCycles/targetCrossed completion paths; when triggered, it is
 * simply OR'd alongside them into the existing AppMotion_BeginBrake() call,
 * so the already-validated 120 ms active-brake sequence and ACK path are
 * reused completely unchanged. stopctrlBrakeLeadCounts/
 * stopctrlBrakeEntrySpeed (DEFAULT_STOPCTRL_*) are the physically-
 * validated PRECISION values. Boot-enabled by default -- still fully
 * runtime-overridable via STOPCTRL, no flash persistence. */
static bool  stopctrlEnabled          = DEFAULT_STOPCTRL_ENABLED;
static float stopctrlBrakeLeadCounts  = DEFAULT_STOPCTRL_BRAKE_LEAD_COUNTS;
static float stopctrlBrakeEntrySpeed  = DEFAULT_STOPCTRL_BRAKE_ENTRY_SPEED;

/* Stage A4: RAM-only distance (counts/cm) calibration for MOV,F/B target
 * generation only -- never touches WSYNC/HCTRL/SPCTRL/SPPROF/STOPCTRL/DPROF,
 * PID tuning, braking, turns, or the SPEED_TEST diagnostic's own use of
 * ENCODER_COUNTS_PER_CM. When disabled, AppMotion_CalcDistanceCounts()
 * computes distanceCounts exactly as before this stage
 * (cm*ENCODER_COUNTS_PER_CM), byte-identical. movcalSlopeCountsPerCm/
 * movcalOffsetCounts (DEFAULT_MOVCAL_*) are the physically-validated
 * PRECISION values. Boot-enabled by default -- still fully runtime-
 * overridable via MOVCAL, no flash persistence. */
static bool  movcalEnabled          = DEFAULT_MOVCAL_ENABLED;
static float movcalSlopeCountsPerCm = DEFAULT_MOVCAL_SLOPE_COUNTS_PER_CM;
static float movcalOffsetCounts     = DEFAULT_MOVCAL_OFFSET_COUNTS;

/* Named runtime configuration profile tracking (motion-profiles branch).
 * currentProfile starts at PRECISION -- the boot-time DEFAULT_* values
 * above already equal PROFILE_PRECISION_*'s own values, so this static
 * default alone (no boot-time function call, AppMotion_Init() is
 * otherwise untouched) correctly reflects the power-on state.
 * applyingProfile is a one-shot guard AppMotion_ApplyProfile() holds
 * true only while it is calling the profile-tracked Set* functions below
 * on its own behalf, so those calls do not themselves mark the profile
 * CUSTOM -- see each Set* function's own "if (!applyingProfile)" check
 * and AppMotion_ApplyProfile() itself. */
static AppMotionProfile currentProfile = APP_MOTION_PROFILE_PRECISION;
static bool applyingProfile = false;

/* Stage SFF1/SFF2: speed-aware servo feed-forward, layered on top of the
 * validated heading PID + HCTRL output. Applied only during
 * APP_MOTION_DISTANCE (MOV,F/MOV,B) -- never touches FL/FR/BL/BR turns
 * or the bench TUNE,S steering test, which each have their own separate,
 * unmodified servoPulse computation. Independent per direction, same
 * formula: a linear ramp of filteredChassisSpeed (already the
 * direction-normalized, EMA-filtered chassis speed Stage A3.3a computes
 * unconditionally every APP_MOTION_DISTANCE tick) from 0 at
 * startSpeed to maxUs at fullSpeed.
 *
 * Stage SFF2/SFF3: now profile-owned, boot-enabled by default
 * (DEFAULT_SFF_*) with the physically-validated Forward AND Backward
 * values -- both PROFILE_PRECISION_* and PROFILE_STANDARD_* load the
 * same values (see AppMotion_ApplyProfile()); a manual SFF SET now marks
 * the active profile CUSTOM, exactly like every other profile-owned
 * field (see AppMotion_SetServoFeedForward()). Still fully
 * runtime-overridable via SFF, no flash persistence. */
static bool  sffForwardEnabled      = DEFAULT_SFF_FORWARD_ENABLED;
static float sffForwardStartSpeed   = DEFAULT_SFF_FORWARD_START_SPEED;
static float sffForwardFullSpeed    = DEFAULT_SFF_FORWARD_FULL_SPEED;
static float sffForwardMaxUs        = DEFAULT_SFF_FORWARD_MAX_US;

static bool  sffBackwardEnabled     = DEFAULT_SFF_BACKWARD_ENABLED;
static float sffBackwardStartSpeed  = DEFAULT_SFF_BACKWARD_START_SPEED;
static float sffBackwardFullSpeed   = DEFAULT_SFF_BACKWARD_FULL_SPEED;
static float sffBackwardMaxUs       = DEFAULT_SFF_BACKWARD_MAX_US;

/* Command-start encoder positions, captured once per MOV,F/B command.
 * Dedicated statics -- the stall-recovery start-count statics cannot be
 * reused because they are overwritten when a stall episode opens. */
static float distanceLeftStartCounts  = 0.0f;
static float distanceRightStartCounts = 0.0f;

static AppMotionState motionState =
    APP_MOTION_IDLE;

static float targetLeftCounts  = 0.0f;
static float targetRightCounts = 0.0f;
static float targetHeadingDeg  = 0.0f;
static uint8_t settledCycles = 0U;
static float tuneElapsedSeconds = 0.0f;
static float tuneDurationSeconds = 0.0f;
static float tuneSteeringTargetRate = 0.0f;
static int32_t rightEncoderPreviousCounts = 0;
static int32_t rightEncoderUnwrappedCounts = 0;
static float leftDistanceStallPositionCounts = 0.0f;
static float rightDistanceStallPositionCounts = 0.0f;
static uint16_t leftDistanceStallCycles = 0U;
static uint16_t rightDistanceStallCycles = 0U;
static bool distanceStallRecoveryActive = false;
static float leftStallRecoveryStartCounts = 0.0f;
static float rightStallRecoveryStartCounts = 0.0f;
static float leftStallRecoveryDirection = 1.0f;
static float rightStallRecoveryDirection = 1.0f;
static bool leftStallRecovered = false;
static bool rightStallRecovered = false;
static float diagnosticElapsedSeconds = 0.0f;
static float diagnosticDurationSeconds = 0.0f;
static int32_t diagnosticDuty = 0;
static int32_t diagnosticStartLeftCounts = 0;
static int32_t diagnosticStartRightCounts = 0;
static float diagnosticStartHeadingDeg = 0.0f;
static MotionType activeCommand = MOTION_STOP;
static uint32_t awaitedImuEventSequence = 0U;
static float imuWaitElapsedSeconds = 0.0f;
static TickType_t movementDeadlineTick = 0U;
static TickType_t brakeDeadlineTick = 0U;

/* Unified movement telemetry (MM,/EV:) only -- captured at each of the
 * six MOV,F/B/FL/FR/BL/BR command-entry points, read back into
 * tuneTelemetry every APP_MOTION_DISTANCE/APP_MOTION_TURN tick. Never
 * read by any control computation. */
static uint32_t movementStartTick = 0U;
static float movementStartLeftCounts = 0.0f;
static float movementStartRightCounts = 0.0f;
static AppMotionEventType pendingBrakeEvent = APP_MOTION_EVENT_NONE;
static MotionType pendingBrakeCommand = MOTION_STOP;

/* PRE_STEER: RAM-only config (not profile-owned -- i.e. not part of
 * PROFILE_PRECISION_ / PROFILE_STANDARD_ or touched by
 * AppMotion_ApplyProfile(); see AppMotion_SetPreSteer()'s doc comment)
 * plus the one deadline needed to time the wait. Never read by
 * leftPID/rightPID/steeringPID. Explicitly restored to its DEFAULT_
 * value by AppMotion_Init() (see there) so a repeated init is
 * deterministic; the file-scope initializer below only covers the
 * very first boot. */
static bool preSteerEnabled = DEFAULT_PRESTEER_ENABLED;
static float preSteerDurationMs = DEFAULT_PRESTEER_DURATION_MS;
static TickType_t preSteerDeadlineTick = 0U;

/* TURNSPEED: RAM-only, not profile-owned (see
 * AppMotion_SetTurnSpeed()'s doc comment) -- replaces the previously-
 * hardcoded base turn rate. Stored as uint32_t (the wire/API type);
 * cast to float only where it feeds the target-rate ramp
 * (case APP_MOTION_TURN:). Explicitly restored to DEFAULT_
 * TURN_SPEED_COUNTS_PER_SEC by AppMotion_Init() -- see the PRE_STEER
 * static above for why. */
static uint32_t turnSpeedCountsPerSec = DEFAULT_TURN_SPEED_COUNTS_PER_SEC;

/* Ackermann-v2: RAM-only config (not profile-owned, see
 * AppMotion_SetAckermann()'s doc comment). Never read by leftPID/
 * rightPID/steeringPID directly -- only feeds
 * AppMotion_SelectAckermannRates()'s rate selection. Explicitly
 * restored to its DEFAULT_ values by AppMotion_Init() -- see the
 * PRE_STEER static above for why. */
static bool ackermannEnabled = DEFAULT_ACKERMANN_ENABLED;
static float ackermannK = DEFAULT_ACKERMANN_K;

/* TURNBRAKE: RAM-only config (not profile-owned, see
 * AppMotion_SetTurnBrake()'s own doc comment). turnBrakeQualifyingCycles
 * is the per-command, turn-only consecutive-sample counter (see
 * case APP_MOTION_TURN:) -- reset at every point settledCycles itself is
 * reset (new command entry, AppMotion_StopAndReset(),
 * AppMotion_BeginBrake(), AppMotion_Init()), so it never carries a
 * partial streak from one command/abort/timeout into the next.
 * turnBrakeEnabled/turnBrakeLookaheadMs are explicitly restored to
 * their DEFAULT_ values by AppMotion_Init() -- see the PRE_STEER
 * static above for why. */
static bool turnBrakeEnabled = DEFAULT_TURNBRAKE_ENABLED;
static float turnBrakeLookaheadMs = DEFAULT_TURNBRAKE_LOOKAHEAD_MS;
static uint8_t turnBrakeQualifyingCycles = 0U;

static AppMotionTelemetry tuneTelemetry;
static AppMotionTelemetry publishedTelemetry;
static uint32_t motionEventSequence = 0U;
static AppMotionEvent publishedMotionEvent;

/* TURNBRAKE diagnostic snapshot mailbox -- same retained/sequence-gated
 * pattern as publishedMotionEvent above, a second independent instance
 * of it. turnBrakeSnapshotSequence is a separate counter (mirrors
 * motionEventSequence's own role for publishedMotionEvent), incremented
 * and copied into the struct's own `sequence` field at publish time, not
 * read back out of the struct itself. Both this counter and
 * publishedTurnBrakeSnapshot.valid are explicitly (re)initialized in
 * AppMotion_Init(), not left to implicit BSS zero-init. */
static uint32_t turnBrakeSnapshotSequence = 0U;
static AppMotionTurnBrakeSnapshot publishedTurnBrakeSnapshot;

/* Stage A3.3a integral-limit fix: keeps speedPID.integralMin/integralMax
 * self-consistent with its own currently-configured Ki and outputMax,
 * instead of a fixed compiled ceiling. For Ki>0, integralMax =
 * outputMax/Ki so that Ki*integralMax == outputMax exactly -- the
 * integral term regains authority to supply the controller's entire
 * positive output range on its own (required since P=0 at
 * actualSpeed==targetSpeed), and the existing, unmodified output-
 * saturation anti-windup in PID_Update() becomes the effective limiter
 * instead of this raw state ceiling pre-empting it. integralMin is
 * pinned to 0.0f (never negative): speedPID's output is magnitude-only
 * (outputMin=0 is the controller's anti-reversal guarantee), and a
 * negative integral state would only add recovery lag with no benefit,
 * since direction is handled entirely separately by spctrlDirection.
 * For Ki<=0, integralMax=0.0f pins integral at exactly 0 every tick
 * (Ki*integral==0 regardless, matching every other PID instance in this
 * firmware) rather than leaving a division-by-zero or a stale/arbitrary
 * bound. Caller must already hold the critical section, matching how
 * both call sites (AppMotion_Init(), AppMotion_SetSpeedControl())
 * already operate. Does not touch app_pid.c. */
static void AppMotion_UpdateSpeedIntegralLimits(void)
{
    if (speedPID.ki > 0.0f)
    {
        speedPID.integralMin = 0.0f;
        speedPID.integralMax = speedPID.outputMax / speedPID.ki;
    }
    else
    {
        speedPID.integralMin = 0.0f;
        speedPID.integralMax = 0.0f;
    }
}

/* Stage A4: cm -> target-counts conversion, shared by MOTION_FORWARD and
 * MOTION_BACKWARD so the formula exists in exactly one place. distanceCm==0
 * is always exactly 0 counts, regardless of MOVCAL state -- the additive
 * offset must never turn a genuine zero-distance command into motion.
 * MOVCAL disabled preserves the original behavior exactly
 * (cm*ENCODER_COUNTS_PER_CM); enabled applies the calibrated
 * cm*slope+offset model instead. movcalSlopeCountsPerCm/movcalOffsetCounts
 * are validated at SET time (app_comm.c) to keep this always positive for
 * any distanceCm>=1 -- no re-validation needed here. */
static float AppMotion_CalcDistanceCounts(int32_t distanceCm)
{
    if (distanceCm == 0)
    {
        return 0.0f;
    }

    if (movcalEnabled)
    {
        return ((float)distanceCm * movcalSlopeCountsPerCm) + movcalOffsetCounts;
    }

    return (float)distanceCm * (float)ENCODER_COUNTS_PER_CM;
}

static void AppMotion_ResetAllPIDs(void)
{
    PID_Reset(&leftPID);
    PID_Reset(&rightPID);
    PID_Reset(&steeringPID);

    /* Stage A3.3a: covers command completion, abort/error, and
     * controller disable -- a disable can only take effect at the next
     * command anyway (SET is idle-only), which always resets fresh via
     * this same helper or the MOTION_FORWARD/MOTION_BACKWARD entry
     * resets below. */
    PID_Reset(&speedPID);
    filteredChassisSpeed = 0.0f;
    filteredChassisSpeedInitialized = false;
}

/* Seeds the APP_MOTION_DISTANCE stall-recovery tracking from the given
 * encoder positions and clears both stall counters, so no stall state
 * can leak in from a previous command. */
static void AppMotion_ResetDistanceStallState(
    int32_t leftPositionCounts,
    int32_t rightPositionCounts
)
{
    leftDistanceStallPositionCounts = (float)leftPositionCounts;
    rightDistanceStallPositionCounts = (float)rightPositionCounts;
    leftDistanceStallCycles = 0U;
    rightDistanceStallCycles = 0U;

    distanceStallRecoveryActive = false;
    leftStallRecoveryStartCounts = (float)leftPositionCounts;
    rightStallRecoveryStartCounts = (float)rightPositionCounts;
    leftStallRecoveryDirection = 1.0f;
    rightStallRecoveryDirection = 1.0f;
    leftStallRecovered = false;
    rightStallRecovered = false;
}

static bool AppMotion_DeadlineReached(
    TickType_t deadlineTick
)
{
    return (int32_t)(xTaskGetTickCount() - deadlineTick) >= 0;
}

static void AppMotion_SetMovementDeadline(
    uint32_t timeoutMs
)
{
    TickType_t timeoutTicks = pdMS_TO_TICKS(timeoutMs);

    if (timeoutTicks == 0U)
    {
        timeoutTicks = 1U;
    }

    movementDeadlineTick =
        xTaskGetTickCount() + timeoutTicks;
}

static bool AppMotion_PIDHasGains(
    const PIDController *pid
)
{
    return (pid->kp != 0.0f) ||
           (pid->ki != 0.0f) ||
           (pid->kd != 0.0f);
}

static bool AppMotion_RequiredPIDsAreSet(
    uint32_t requiredMask
)
{
    bool areSet = true;

    taskENTER_CRITICAL();

    if (((requiredMask & APP_MOTION_PID_MASK_LEFT) != 0U) &&
        !AppMotion_PIDHasGains(&leftPID))
    {
        areSet = false;
    }

    if (((requiredMask & APP_MOTION_PID_MASK_RIGHT) != 0U) &&
        !AppMotion_PIDHasGains(&rightPID))
    {
        areSet = false;
    }

    if (((requiredMask & APP_MOTION_PID_MASK_STEERING) != 0U) &&
        !AppMotion_PIDHasGains(&steeringPID))
    {
        areSet = false;
    }

    taskEXIT_CRITICAL();
    return areSet;
}

static void AppMotion_RebaseRightEncoder(
    int32_t currentCounts
)
{
    rightEncoderPreviousCounts = currentCounts;
    rightEncoderUnwrappedCounts = currentCounts;
}

static int32_t AppMotion_GetRightPosition(void)
{
    int32_t currentCounts = AppEncoder_Right();
    int32_t delta =
        currentCounts - rightEncoderPreviousCounts;

    if (delta > INT16_MAX)
    {
        delta -= RIGHT_ENCODER_MODULUS_COUNTS;
    }
    else if (delta < INT16_MIN)
    {
        delta += RIGHT_ENCODER_MODULUS_COUNTS;
    }

    rightEncoderPreviousCounts = currentCounts;
    rightEncoderUnwrappedCounts += delta;

    return rightEncoderUnwrappedCounts;
}

static void AppMotion_PublishTelemetry(void)
{
    taskENTER_CRITICAL();
    publishedTelemetry = tuneTelemetry;
    taskEXIT_CRITICAL();
}

static void AppMotion_PublishEvent(
    AppMotionEventType type,
    MotionType command,
    int32_t value,
    int32_t left,
    int32_t right,
    int32_t headingX10
)
{
    AppMotionEvent event;

    motionEventSequence++;

    if (motionEventSequence == 0U)
    {
        motionEventSequence++;
    }

    event.type = type;
    event.command = command;
    event.sequence = motionEventSequence;
    event.value = value;
    event.left = left;
    event.right = right;
    event.headingX10 = headingX10;

    taskENTER_CRITICAL();
    publishedMotionEvent = event;
    taskEXIT_CRITICAL();
}

static void AppMotion_StopAndReset(void)
{
    if (motionState == APP_MOTION_WAIT_IMU)
    {
        (void)AppIMU_CancelPendingRequest();
    }

    AppMotor_Stop();
    AppServo_Center();
    AppMotion_ResetAllPIDs();
    AppMotion_ResetDistanceStallState(0, 0);

    distanceLeftStartCounts = 0.0f;
    distanceRightStartCounts = 0.0f;

    settledCycles = 0U;
    /* TURNBRAKE: reset alongside settledCycles -- every abort/stop
     * (MOTION_STOP) routes through here, so a partial predictive-brake
     * qualifying streak never survives into whatever command comes
     * next. */
    turnBrakeQualifyingCycles = 0U;
    diagnosticElapsedSeconds = 0.0f;
    diagnosticDurationSeconds = 0.0f;
    diagnosticDuty = 0;
    diagnosticStartLeftCounts = 0;
    diagnosticStartRightCounts = 0;
    AppMotion_RebaseRightEncoder(0);
    diagnosticStartHeadingDeg = 0.0f;
    awaitedImuEventSequence = 0U;
    imuWaitElapsedSeconds = 0.0f;
    movementDeadlineTick = 0U;
    brakeDeadlineTick = 0U;
    pendingBrakeEvent = APP_MOTION_EVENT_NONE;
    pendingBrakeCommand = MOTION_STOP;
    activeCommand = MOTION_STOP;
    motionState = APP_MOTION_IDLE;
}

static int32_t AppMotion_ToTenths(float value)
{
    float scaled = value * 10.0f;

    if (scaled >= 0.0f)
    {
        scaled += 0.5f;
    }
    else
    {
        scaled -= 0.5f;
    }

    return (int32_t)scaled;
}

/* TURNBRAKE: publishes the one-shot diagnostic snapshot into its own
 * retained mailbox (see AppMotionTurnBrakeSnapshot's own doc comment,
 * app_motion.h) -- called exactly once per turn, only from
 * case APP_MOTION_TURN: at the instant the predictive-brake condition's
 * second consecutive qualifying tick fires, BEFORE
 * AppMotion_BeginBrake() is called. All values are captured from the
 * SAME tick's already-computed locals (the caller's own headingNow/
 * currentGyroZDps/etc.), not re-read here -- this function performs no
 * sensor access of its own. Uses AppMotion_ToTenths() for every X10
 * field, including configuredLookaheadMsX10 (tenths of a millisecond,
 * preserving turnBrakeLookaheadMs's own float precision rather than
 * truncating it). */
static void AppMotion_PublishTurnBrakeSnapshot(
    MotionType direction,
    float targetHeadingDeg,
    float headingNow,
    float remainingAngleDeg,
    float gyroZDps,
    float predictedStoppingAngleDeg,
    float lookaheadMs,
    uint8_t confirmationCycles
)
{
    AppMotionTurnBrakeSnapshot snapshot;

    turnBrakeSnapshotSequence++;

    if (turnBrakeSnapshotSequence == 0U)
    {
        turnBrakeSnapshotSequence++;
    }

    snapshot.sequence = turnBrakeSnapshotSequence;
    snapshot.valid = 1U;
    snapshot.direction = direction;
    snapshot.targetHeadingX10 = AppMotion_ToTenths(targetHeadingDeg);
    snapshot.currentHeadingX10 = AppMotion_ToTenths(headingNow);
    snapshot.remainingAngleX10 = AppMotion_ToTenths(remainingAngleDeg);
    snapshot.gyroZDpsX10 = AppMotion_ToTenths(gyroZDps);
    snapshot.predictedStoppingAngleX10 =
        AppMotion_ToTenths(predictedStoppingAngleDeg);
    snapshot.configuredLookaheadMsX10 = AppMotion_ToTenths(lookaheadMs);
    snapshot.confirmationCycles = confirmationCycles;

    taskENTER_CRITICAL();
    publishedTurnBrakeSnapshot = snapshot;
    taskEXIT_CRITICAL();
}

/* Fixed max-safe turn servo stage: single source of truth for the
 * turn servo DIRECTION mapping, used by both PRE_STEER entry and
 * every APP_MOTION_TURN tick (see AppMotion_EnterTurnOrPreSteer() and
 * case APP_MOTION_TURN: below) -- the ONLY place this mapping is
 * decided. Explicit switch on MotionType, not a derived boolean --
 * Ackermann geometry (inner/outer wheel reasoning) is intentionally
 * out of scope on this branch; this is purely a turn-servo-direction
 * lookup. FL/BR steer toward SERVO_MIN_US (1000us); FR/BL steer
 * toward SERVO_MAX_US (2060us) -- both physically validated (PRE_STEER
 * bench sweep). Any other/non-turn MotionType defensively returns
 * SERVO_CENTER_US (never actually reached in normal operation -- this
 * function's only callers are already inside a turn-specific code
 * path). This is a fixed lookup, not steeringPID output -- see
 * AppMotion_EnterTurnOrPreSteer()'s and case APP_MOTION_TURN:'s own
 * comments for why steeringPID is not evaluated for servo generation
 * during this experiment. */
static int32_t AppMotion_TurnServoEndpointUs(
    MotionType commandType
)
{
    switch (commandType)
    {
        case MOTION_FORWARD_LEFT:
        case MOTION_BACKWARD_RIGHT:
            return (int32_t)SERVO_MIN_US;

        case MOTION_FORWARD_RIGHT:
        case MOTION_BACKWARD_LEFT:
            return (int32_t)SERVO_MAX_US;

        default:
            return (int32_t)SERVO_CENTER_US;
    }
}

/* Ackermann-v2: single source of truth for the inner/outer rear-wheel
 * target-RATE selection, called both by case APP_MOTION_TURN:'s own
 * ramp computation and by its telemetry write (AK, line), so there is
 * exactly one place this decision is made. Returns RATE MAGNITUDES
 * (counts/sec, always >=0) -- the caller multiplies by dtSeconds
 * itself for the actual per-tick ramp. Deliberately kept independent
 * of AppMotion_TurnServoEndpointUs() above, even though both partition
 * the same four MotionTypes the same way -- not refactored into a
 * shared helper, so a future Ackermann change can never accidentally
 * perturb the validated, frozen servo endpoint logic.
 *
 * Disabled: both wheels get exactly the current runtime TURNSPEED
 * (AppMotion_GetTurnSpeed()'s underlying value) -- not merely
 * numerically equal to it, this is the literal same value with no
 * (1+-k) arithmetic involved at all, so disabled behavior never
 * depends on whatever ackermannK happens to be stored as.
 *
 * Enabled: innerRate = base*(1-k), outerRate = base*(1+k), where base
 * is the CURRENT runtime turnSpeedCountsPerSec (never a hardcoded
 * value) -- (innerRate+outerRate)/2 == base always, by construction,
 * for any k or any TURNSPEED. leftIsInner is the physically-confirmed
 * mapping: MOTION_FORWARD_LEFT and MOTION_BACKWARD_RIGHT are the two
 * primitives whose servo steers toward SERVO_MIN_US (1000us) -- for
 * both, the LEFT rear wheel is the inner wheel. The other two
 * (MOTION_FORWARD_RIGHT, MOTION_BACKWARD_LEFT) steer toward
 * SERVO_MAX_US (2060us) and have the RIGHT rear wheel as inner. This
 * grouping is independent of forward-vs-reverse -- forward/reverse
 * only flips the SIGN applied to these magnitudes (see the caller),
 * never which wheel is inner. */
static void AppMotion_SelectAckermannRates(
    MotionType commandType,
    float *leftRateCountsPerSec,
    float *rightRateCountsPerSec
)
{
    float baseRate = (float)turnSpeedCountsPerSec;

    if (!ackermannEnabled)
    {
        *leftRateCountsPerSec = baseRate;
        *rightRateCountsPerSec = baseRate;
        return;
    }

    float innerRate = baseRate * (1.0f - ackermannK);
    float outerRate = baseRate * (1.0f + ackermannK);

    bool leftIsInner =
        (commandType == MOTION_FORWARD_LEFT) ||
        (commandType == MOTION_BACKWARD_RIGHT);

    *leftRateCountsPerSec = leftIsInner ? innerRate : outerRate;
    *rightRateCountsPerSec = leftIsInner ? outerRate : innerRate;
}

/* Shared by all four turn command-entry cases (only call site) --
 * called strictly after that case's own targetHeadingDeg/
 * targetLeftCounts/targetRightCounts/PID_Reset(leftPID/rightPID/
 * steeringPID) assignments (unchanged), so this only decides HOW the
 * turn begins, never re-touches heading-target math or sign logic.
 *
 * PRESTEER disabled: byte-identical to before PRE_STEER existed --
 * motionState = APP_MOTION_TURN, deadline set immediately.
 *
 * PRESTEER enabled: commands the servo directly to the fixed
 * AppMotion_TurnServoEndpointUs() endpoint -- steeringPID is NOT
 * evaluated at PRE_STEER entry at all (no PID_Update/PID_Reset dance
 * needed here any more; the caller's own PID_Reset(&steeringPID)
 * already leaves it in a clean, dormant state, unread by this
 * function). Motors are held at AppMotor_Stop() (zero PWM duty on all
 * channels, not an active brake) -- per explicit instruction, this
 * stage isolates servo-before-wheel timing only and must not
 * introduce a new electrical/chassis behavior (active braking) that
 * didn't exist at normal turn entry. leftPID/rightPID and the wheel
 * targets are not touched here at all -- they stay exactly as the
 * caller's own PID_Reset()/target-anchor already left them, and
 * APP_MOTION_TURN's deadline is deliberately NOT set yet (see
 * AppMotion_Update()'s APP_MOTION_PRE_STEER case) so PRE_STEER's own
 * wait never steals from the turn-motion timeout budget. */
static void AppMotion_EnterTurnOrPreSteer(
    MotionType commandType,
    int32_t requestedValue
)
{
    if (preSteerEnabled)
    {
        int32_t fixedServoPulse =
            AppMotion_TurnServoEndpointUs(commandType);

        AppServo_SetUs((uint32_t)fixedServoPulse);
        AppMotor_Stop();

        TickType_t preSteerDurationTicks =
            pdMS_TO_TICKS((uint32_t)preSteerDurationMs);

        preSteerDeadlineTick =
            xTaskGetTickCount() + preSteerDurationTicks;

        motionState = APP_MOTION_PRE_STEER;
    }
    else
    {
        motionState = APP_MOTION_TURN;
        AppMotion_SetMovementDeadline(
            MOV_TIMEOUT_BASE_MS +
            ((uint32_t)requestedValue *
             MOV_TURN_TIMEOUT_MS_PER_DEGREE)
        );
    }
}

static void AppMotion_CancelTuneTelemetry(void)
{
    if (tuneTelemetry.active == 0U)
    {
        return;
    }

    tuneTelemetry.active = 0U;
    tuneTelemetry.mode = APP_TUNE_NONE;
    tuneTelemetry.leftOutput = 0;
    tuneTelemetry.rightOutput = 0;
    tuneTelemetry.steeringCorrectionUs = 0;
    tuneTelemetry.servoPulseUs = SERVO_CENTER_US;

    AppMotion_PublishTelemetry();
}

static void AppMotion_PrepareTune(
    AppMotionTuneMode mode,
    int32_t leftNow,
    int32_t rightNow
)
{
    float headingNow = AppIMU_Heading();

    AppMotor_Stop();
    AppServo_Center();
    AppMotion_ResetAllPIDs();

    settledCycles = 0U;
    tuneElapsedSeconds = 0.0f;
    tuneDurationSeconds = 0.0f;
    tuneSteeringTargetRate = 0.0f;

    targetLeftCounts = (float)leftNow;
    targetRightCounts = (float)rightNow;
    targetHeadingDeg = headingNow;
    AppMotion_RebaseRightEncoder(rightNow);

    tuneTelemetry.mode = mode;
    tuneTelemetry.active = 1U;
    tuneTelemetry.runSequence++;

    if (tuneTelemetry.runSequence == 0U)
    {
        tuneTelemetry.runSequence++;
    }

    tuneTelemetry.leftTargetCounts = leftNow;
    tuneTelemetry.leftPositionCounts = leftNow;
    tuneTelemetry.leftErrorCounts = 0;
    tuneTelemetry.leftOutput = 0;

    tuneTelemetry.rightTargetCounts = rightNow;
    tuneTelemetry.rightPositionCounts = rightNow;
    tuneTelemetry.rightErrorCounts = 0;
    tuneTelemetry.rightOutput = 0;

    tuneTelemetry.headingTargetX10 =
        AppMotion_ToTenths(headingNow);
    tuneTelemetry.headingCurrentX10 =
        AppMotion_ToTenths(headingNow);
    tuneTelemetry.headingErrorX10 = 0;
    tuneTelemetry.steeringCorrectionUs = 0;
    tuneTelemetry.servoPulseUs = SERVO_CENTER_US;
}

static void AppMotion_FinishTune(
    AppMotionTuneResult result
)
{
    AppMotionTuneMode completedMode =
        tuneTelemetry.mode;

    AppMotor_Stop();
    AppServo_Center();
    AppMotion_ResetAllPIDs();

    settledCycles = 0U;
    tuneElapsedSeconds = 0.0f;
    tuneDurationSeconds = 0.0f;
    tuneSteeringTargetRate = 0.0f;
    AppMotion_RebaseRightEncoder(0);

    tuneTelemetry.active = 0U;
    tuneTelemetry.mode = APP_TUNE_NONE;
    tuneTelemetry.completedMode = completedMode;
    tuneTelemetry.completedResult = result;
    tuneTelemetry.completionSequence++;

    if (tuneTelemetry.completionSequence == 0U)
    {
        tuneTelemetry.completionSequence++;
    }

    tuneTelemetry.leftOutput = 0;
    tuneTelemetry.rightOutput = 0;
    tuneTelemetry.steeringCorrectionUs = 0;
    tuneTelemetry.servoPulseUs = SERVO_CENTER_US;

    activeCommand = MOTION_STOP;
    motionState = APP_MOTION_IDLE;

    AppMotion_PublishTelemetry();
}

static void AppMotion_PrepareDiagnostic(
    MotionType command,
    int32_t leftNow,
    int32_t rightNow,
    float durationSeconds
)
{
    AppMotor_Stop();
    AppServo_Center();
    AppMotion_ResetAllPIDs();

    diagnosticElapsedSeconds = 0.0f;
    diagnosticDurationSeconds = durationSeconds;
    diagnosticDuty = 0;
    diagnosticStartLeftCounts = leftNow;
    diagnosticStartRightCounts = rightNow;
    AppMotion_RebaseRightEncoder(rightNow);
    diagnosticStartHeadingDeg = AppIMU_Heading();
    targetHeadingDeg = diagnosticStartHeadingDeg;
    activeCommand = command;
}

static void AppMotion_FinishDiagnostic(
    AppMotionEventType eventType,
    int32_t value,
    int32_t left,
    int32_t right,
    int32_t headingX10
)
{
    MotionType completedCommand = activeCommand;

    AppMotion_StopAndReset();
    AppMotion_PublishEvent(
        eventType,
        completedCommand,
        value,
        left,
        right,
        headingX10
    );
}

static void AppMotion_StopWithEvent(
    AppMotionEventType eventType
)
{
    MotionType completedCommand = activeCommand;

    AppMotion_CancelTuneTelemetry();
    AppMotion_StopAndReset();
    AppMotion_PublishEvent(
        eventType,
        completedCommand,
        0,
        0,
        0,
        0
    );
}

static void AppMotion_BeginBrake(
    AppMotionEventType terminalEvent
)
{
	if ((tuneTelemetry.mode == APP_TUNE_MOV_DISTANCE) ||
	    (tuneTelemetry.mode == APP_TUNE_MOV_TURN))
	{
	    /* Unified movement telemetry (MM,) -- one extra frozen snapshot
	     * with ST:BRAKE before the existing clear below. LO/RO are left
	     * exactly as this tick's last regular write left them (the LAST
	     * PRE-BRAKE commands) -- AppMotor_Brake() below takes no
	     * per-wheel value, so no live duty exists to report here; see
	     * movementBraking's doc comment in app_motion.h. SP is set to
	     * SERVO_CENTER_US to match what AppServo_Center() below actually
	     * commands (not a stale pre-brake pulse). */
	    tuneTelemetry.movementBraking = 1U;
	    tuneTelemetry.servoPulseUs = SERVO_CENTER_US;
	    tuneTelemetry.elapsedMs =
	        xTaskGetTickCount() - movementStartTick;
	    AppMotion_PublishTelemetry();

	    tuneTelemetry.active = 0U;
	    tuneTelemetry.mode = APP_TUNE_NONE;
	    tuneTelemetry.movementBraking = 0U;
	    AppMotion_PublishTelemetry();
	}

    TickType_t brakeDurationTicks =
        pdMS_TO_TICKS(MOTION_BRAKE_DURATION_MS);

    if (brakeDurationTicks == 0U)
    {
        brakeDurationTicks = 1U;
    }

    pendingBrakeEvent = terminalEvent;
    pendingBrakeCommand = activeCommand;
    brakeDeadlineTick =
        xTaskGetTickCount() + brakeDurationTicks;
    movementDeadlineTick = 0U;
    settledCycles = 0U;
    /* TURNBRAKE: every call site of AppMotion_BeginBrake() (normal
     * dwell/predictive completion, timeout, IMU-not-ready) routes
     * through here, so this one reset covers all of them. */
    turnBrakeQualifyingCycles = 0U;

    AppMotor_Brake();
    AppServo_Center();
    AppMotion_ResetAllPIDs();
    motionState = APP_MOTION_BRAKING;
}

static void AppMotion_FinishBrake(void)
{
    AppMotionEventType terminalEvent = pendingBrakeEvent;
    MotionType completedCommand = pendingBrakeCommand;

    AppMotion_StopAndReset();
    AppMotion_PublishEvent(
        terminalEvent,
        completedCommand,
        0,
        0,
        0,
        0
    );
}

static void AppMotion_FinishImuRequest(
    AppMotionEventType eventType
)
{
    MotionType completedCommand = activeCommand;

    awaitedImuEventSequence = 0U;
    imuWaitElapsedSeconds = 0.0f;
    activeCommand = MOTION_STOP;
    motionState = APP_MOTION_IDLE;

    AppMotion_PublishEvent(
        eventType,
        completedCommand,
        0,
        0,
        0,
        0
    );
}

void AppMotion_Init(void)
{
    /* Compiled defaults: the runtime PID values validated in physical
     * testing (team notes). PID,L / PID,R / PID,S still override these
     * at runtime via AppMotion_SetPIDGains(); no flash persistence. */
    PID_Init(
        &leftPID,
        0.10f,
        0.00f,
        0.02f,
        -99.0f,
        99.0f,
        -5000.0f,
        5000.0f
    );

    PID_Init(
        &rightPID,
        0.10f,
        0.00f,
        0.02f,
        -99.0f,
        99.0f,
        -5000.0f,
        5000.0f
    );

    PID_Init(
        &steeringPID,
        DEFAULT_STEERING_KP,
        DEFAULT_STEERING_KI,
        DEFAULT_STEERING_KD,
		-530.0f,
		530.0f,
        -1000.0f,
        1000.0f
    );

    /* Stage A3.3a: speedPID -- output range [0, MOTOR_MAX_DUTY] is the
     * entire anti-reversal guarantee (PID_Update's own final clamp
     * makes the returned value structurally >=0 always, see the plan's
     * anti-windup audit). No D term (Kd=0.0f) -- see the plan's dt-
     * jitter-noise reasoning. Must run before AppMotion_ResetAllPIDs()/
     * any SPCTRL reset call, mirroring the existing ordering discipline
     * already used for the other PID instances in this function.
     * integralMin/integralMax are passed as inert 0.0f/0.0f placeholders
     * here -- AppMotion_UpdateSpeedIntegralLimits() immediately below
     * overwrites them from the actual Ki/outputMax, so this call site
     * never shows a fixed ceiling that could drift out of sync with Ki
     * (see the integral-limit-fix plan). Kp/Ki/Kd are now the physically-
     * validated PRECISION boot defaults (DEFAULT_SPCTRL_*), not a
     * first-trial placeholder -- still fully overridable via
     * SPCTRL,<target>,<Kp>,<Ki>,<enable> at runtime. */
    PID_Init(
        &speedPID,
        DEFAULT_SPCTRL_KP,
        DEFAULT_SPCTRL_KI,
        DEFAULT_SPCTRL_KD,
        0.0f,
        (float)MOTOR_MAX_DUTY,
        0.0f,
        0.0f
    );
    AppMotion_UpdateSpeedIntegralLimits();

    /* Stage A1: rear-wheel sync boot defaults -- physically validated,
     * fully runtime-overridable via WSYNC. */
    wheelSyncEnabled = DEFAULT_WSYNC_ENABLED;
    wheelSyncGain = DEFAULT_WSYNC_GAIN;
    wheelSyncMaxDuty = DEFAULT_WSYNC_MAX_DUTY;
    distanceLeftStartCounts = 0.0f;
    distanceRightStartCounts = 0.0f;

    /* Stage A2: heading rate-damping boot defaults -- physically
     * validated, fully runtime-overridable via HCTRL. */
    headingCtrlEnabled = DEFAULT_HCTRL_ENABLED;
    headingCtrlKrate = DEFAULT_HCTRL_KRATE;
    headingCtrlMaxCorrection = DEFAULT_HCTRL_MAX_CORRECTION;

    /* Stage A3.1: reverse-torque guard default -- disabled
     * (DEFAULT_RTGUARD_ENABLED). Inert until enabled over UART via
     * RTGUARD. */
    rtGuardEnabled = DEFAULT_RTGUARD_ENABLED;

    /* Stage A3.2: deceleration profile defaults -- disabled
     * (DEFAULT_DPROF_ENABLED), decelStart/approachDuty (DEFAULT_DPROF_*).
     * Inert until enabled via DPROF. */
    decelProfileEnabled = DEFAULT_DPROF_ENABLED;
    decelStartCounts = DEFAULT_DPROF_DECEL_START_COUNTS;
    decelApproachDuty = DEFAULT_DPROF_APPROACH_DUTY;

    /* Stage A3.3a: speed-control boot defaults -- physically validated,
     * fully runtime-overridable via SPCTRL; speedPID's gains are set by
     * PID_Init above, not here (mirrors how leftPID/rightPID/
     * steeringPID's gains are only ever (re)established by their own
     * PID_Init call, never duplicated as a second assignment). */
    spctrlEnabled = DEFAULT_SPCTRL_ENABLED;
    spctrlTargetSpeed = DEFAULT_SPCTRL_TARGET_SPEED;
    filteredChassisSpeed = 0.0f;
    filteredChassisSpeedInitialized = false;

    /* Stage A3.3b: distance-based linear speed profile boot defaults --
     * physically validated (PRECISION), fully runtime-overridable via
     * SPPROF. */
    sprofEnabled = DEFAULT_SPPROF_ENABLED;
    sprofDecelStartCounts = DEFAULT_SPPROF_DECEL_START_COUNTS;
    sprofApproachSpeed = DEFAULT_SPPROF_APPROACH_SPEED;

    /* Stage A3.3c: final controlled stop gate boot defaults -- physically
     * validated (PRECISION), fully runtime-overridable via STOPCTRL. */
    stopctrlEnabled = DEFAULT_STOPCTRL_ENABLED;
    stopctrlBrakeLeadCounts = DEFAULT_STOPCTRL_BRAKE_LEAD_COUNTS;
    stopctrlBrakeEntrySpeed = DEFAULT_STOPCTRL_BRAKE_ENTRY_SPEED;

    /* Stage A4: distance calibration boot defaults -- physically
     * validated (PRECISION), fully runtime-overridable via MOVCAL. */
    movcalEnabled = DEFAULT_MOVCAL_ENABLED;
    movcalSlopeCountsPerCm = DEFAULT_MOVCAL_SLOPE_COUNTS_PER_CM;
    movcalOffsetCounts = DEFAULT_MOVCAL_OFFSET_COUNTS;

    /* Stage SFF1/SFF2: speed-aware servo feed-forward boot defaults --
     * now the physically-validated PRECISION values, fully runtime-
     * overridable via SFF. */
    sffForwardEnabled = DEFAULT_SFF_FORWARD_ENABLED;
    sffForwardStartSpeed = DEFAULT_SFF_FORWARD_START_SPEED;
    sffForwardFullSpeed = DEFAULT_SFF_FORWARD_FULL_SPEED;
    sffForwardMaxUs = DEFAULT_SFF_FORWARD_MAX_US;
    sffBackwardEnabled = DEFAULT_SFF_BACKWARD_ENABLED;
    sffBackwardStartSpeed = DEFAULT_SFF_BACKWARD_START_SPEED;
    sffBackwardFullSpeed = DEFAULT_SFF_BACKWARD_FULL_SPEED;
    sffBackwardMaxUs = DEFAULT_SFF_BACKWARD_MAX_US;

    /* Turn configuration boot defaults -- PRESTEER/TURNSPEED/ACKERMANN/
     * TURNBRAKE together form the physically validated turn
     * configuration (150.0ms/enabled, 1700 counts/sec, k=0.270/enabled,
     * 135.0ms/enabled respectively; see each DEFAULT_* constant's own
     * comment above for the bench-test basis). These four groups remain
     * deliberately NOT profile-owned -- untouched by
     * AppMotion_ApplyProfile() and PROFILE,PRECISION/PROFILE,STANDARD,
     * same as before -- but are now explicitly restored here (rather
     * than relying solely on their file-scope initializers) so a
     * repeated AppMotion_Init() call is fully deterministic, matching
     * every other boot-default group above. */
    preSteerEnabled = DEFAULT_PRESTEER_ENABLED;
    preSteerDurationMs = DEFAULT_PRESTEER_DURATION_MS;
    preSteerDeadlineTick = 0U;
    turnSpeedCountsPerSec = DEFAULT_TURN_SPEED_COUNTS_PER_SEC;
    ackermannEnabled = DEFAULT_ACKERMANN_ENABLED;
    ackermannK = DEFAULT_ACKERMANN_K;
    turnBrakeEnabled = DEFAULT_TURNBRAKE_ENABLED;
    turnBrakeLookaheadMs = DEFAULT_TURNBRAKE_LOOKAHEAD_MS;

    motionState = APP_MOTION_IDLE;
    settledCycles = 0U;
    /* TURNBRAKE: explicit boot reset, alongside settledCycles, of the
     * per-command qualifying-streak counter and the diagnostic
     * snapshot mailbox (its sequence counter included) -- explicitly
     * (re)initialized here rather than relying only on implicit BSS
     * zero-init, so AppMotion_Init() being called again at runtime
     * (defensive; not currently done anywhere) would still leave
     * `valid`=0 and a known sequence baseline. */
    turnBrakeQualifyingCycles = 0U;
    turnBrakeSnapshotSequence = 0U;
    publishedTurnBrakeSnapshot = (AppMotionTurnBrakeSnapshot){ 0 };
    AppMotion_ResetDistanceStallState(0, 0);
    tuneElapsedSeconds = 0.0f;
    tuneDurationSeconds = 0.0f;
    tuneSteeringTargetRate = 0.0f;
    AppMotion_RebaseRightEncoder(0);
    diagnosticElapsedSeconds = 0.0f;
    diagnosticDurationSeconds = 0.0f;
    diagnosticDuty = 0;
    diagnosticStartLeftCounts = 0;
    diagnosticStartRightCounts = 0;
    diagnosticStartHeadingDeg = 0.0f;
    activeCommand = MOTION_STOP;
    awaitedImuEventSequence = 0U;
    imuWaitElapsedSeconds = 0.0f;
    movementDeadlineTick = 0U;
    brakeDeadlineTick = 0U;
    pendingBrakeEvent = APP_MOTION_EVENT_NONE;
    pendingBrakeCommand = MOTION_STOP;

    tuneTelemetry = (AppMotionTelemetry){0};
    tuneTelemetry.servoPulseUs = SERVO_CENTER_US;
    motionEventSequence = 0U;
    publishedMotionEvent = (AppMotionEvent){0};

    AppMotion_PublishTelemetry();
}


void AppMotion_HandleCommand(
    const MotionCommand *command
)
{
    if (command == NULL)
    {
        return;
    }

    if ((motionState != APP_MOTION_IDLE) &&
        (command->type != MOTION_STOP))
    {
        AppMotion_PublishEvent(
            APP_MOTION_EVENT_BUSY,
            command->type,
            0,
            0,
            0,
            0
        );
        return;
    }

    int32_t leftNow  = AppEncoder_Left();
    int32_t rightNow = AppEncoder_Right();

    switch (command->type)
    {
        case MOTION_STOP:
            AppMotion_CancelTuneTelemetry();
            AppMotion_StopAndReset();
            break;


        case MOTION_FORWARD:
        {
            if ((command->value < 0) ||
                (command->value > MOV_DISTANCE_MAX_CM))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_BAD_COMMAND,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (command->value == 0)
            {
                activeCommand = command->type;
                AppMotion_StopWithEvent(
                    APP_MOTION_EVENT_MOV_DONE
                );
                break;
            }

            if (!AppIMU_IsReady())
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_IMU_NOT_READY,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (!AppMotion_RequiredPIDsAreSet(
                    APP_MOTION_PID_MASK_LEFT |
                    APP_MOTION_PID_MASK_RIGHT |
                    APP_MOTION_PID_MASK_STEERING))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_PID_NOT_SET,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            AppMotion_CancelTuneTelemetry();

            tuneTelemetry.mode = APP_TUNE_MOV_DISTANCE;
            tuneTelemetry.active = 1U;

            tuneTelemetry.runSequence++;
            if (tuneTelemetry.runSequence == 0U)
            {
                tuneTelemetry.runSequence++;
            }

            /* Unified movement telemetry (MM,/EV:START) -- capture only,
             * never read by any control computation. */
            movementStartTick = xTaskGetTickCount();
            movementStartLeftCounts = (float)leftNow;
            movementStartRightCounts = (float)rightNow;
            tuneTelemetry.commandValue = command->value;
            tuneTelemetry.activeDirection = command->type;
            tuneTelemetry.movementStartTick = movementStartTick;

            AppMotion_RebaseRightEncoder(rightNow);

            float distanceCounts =
                AppMotion_CalcDistanceCounts(command->value);

            targetLeftCounts =
                (float)leftNow + distanceCounts;

            targetRightCounts =
                (float)rightNow + distanceCounts;

            targetHeadingDeg =
                AppIMU_Heading();

            PID_Reset(&leftPID);
            PID_Reset(&rightPID);
            PID_Reset(&steeringPID);

            /* Stage A3.3a: new MOV,F -- reset speedPID and the EMA
             * filter state so nothing leaks from a previous command or
             * direction. */
            PID_Reset(&speedPID);
            filteredChassisSpeed = 0.0f;
            filteredChassisSpeedInitialized = false;

            settledCycles = 0U;
            AppMotion_ResetDistanceStallState(leftNow, rightNow);

            /* Stage A1: capture command-start positions for the
             * direction-normalized wheel-sync progress computation. */
            distanceLeftStartCounts = (float)leftNow;
            distanceRightStartCounts = (float)rightNow;

            motionState =
                APP_MOTION_DISTANCE;
            activeCommand = command->type;
            AppMotion_SetMovementDeadline(
                MOV_TIMEOUT_BASE_MS +
                ((uint32_t)command->value *
                 MOV_DISTANCE_TIMEOUT_MS_PER_CM)
            );

            break;
        }


        case MOTION_BACKWARD:
        {
            if ((command->value < 0) ||
                (command->value > MOV_DISTANCE_MAX_CM))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_BAD_COMMAND,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (command->value == 0)
            {
                activeCommand = command->type;
                AppMotion_StopWithEvent(
                    APP_MOTION_EVENT_MOV_DONE
                );
                break;
            }

            if (!AppIMU_IsReady())
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_IMU_NOT_READY,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (!AppMotion_RequiredPIDsAreSet(
                    APP_MOTION_PID_MASK_LEFT |
                    APP_MOTION_PID_MASK_RIGHT |
                    APP_MOTION_PID_MASK_STEERING))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_PID_NOT_SET,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            AppMotion_CancelTuneTelemetry();

            tuneTelemetry.mode = APP_TUNE_MOV_DISTANCE;
            tuneTelemetry.active = 1U;

            tuneTelemetry.runSequence++;
            if (tuneTelemetry.runSequence == 0U)
            {
                tuneTelemetry.runSequence++;
            }

            /* Unified movement telemetry (MM,/EV:START) -- capture only,
             * never read by any control computation. */
            movementStartTick = xTaskGetTickCount();
            movementStartLeftCounts = (float)leftNow;
            movementStartRightCounts = (float)rightNow;
            tuneTelemetry.commandValue = command->value;
            tuneTelemetry.activeDirection = command->type;
            tuneTelemetry.movementStartTick = movementStartTick;

            AppMotion_RebaseRightEncoder(rightNow);
            float distanceCounts =
                AppMotion_CalcDistanceCounts(command->value);

            targetLeftCounts =
                (float)leftNow - distanceCounts;

            targetRightCounts =
                (float)rightNow - distanceCounts;

            targetHeadingDeg =
                AppIMU_Heading();

            PID_Reset(&leftPID);
            PID_Reset(&rightPID);
            PID_Reset(&steeringPID);

            /* Stage A3.3a: new MOV,B -- reset speedPID and the EMA
             * filter state so nothing leaks from a previous command or
             * direction. */
            PID_Reset(&speedPID);
            filteredChassisSpeed = 0.0f;
            filteredChassisSpeedInitialized = false;

            settledCycles = 0U;
            AppMotion_ResetDistanceStallState(leftNow, rightNow);

            /* Stage A1: capture command-start positions for the
             * direction-normalized wheel-sync progress computation. */
            distanceLeftStartCounts = (float)leftNow;
            distanceRightStartCounts = (float)rightNow;

            motionState =
                APP_MOTION_DISTANCE;
            activeCommand = command->type;
            AppMotion_SetMovementDeadline(
                MOV_TIMEOUT_BASE_MS +
                ((uint32_t)command->value *
                 MOV_DISTANCE_TIMEOUT_MS_PER_CM)
            );

            break;
        }


        case MOTION_FORWARD_LEFT:
            if ((command->value < 0) ||
                (command->value > MOV_TURN_MAX_DEGREES))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_BAD_COMMAND,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (command->value == 0)
            {
                activeCommand = command->type;
                AppMotion_StopWithEvent(
                    APP_MOTION_EVENT_MOV_DONE
                );
                break;
            }

            if (!AppIMU_IsReady())
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_IMU_NOT_READY,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (!AppMotion_RequiredPIDsAreSet(
                    APP_MOTION_PID_MASK_LEFT |
                    APP_MOTION_PID_MASK_RIGHT |
                    APP_MOTION_PID_MASK_STEERING))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_PID_NOT_SET,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            AppMotion_CancelTuneTelemetry();

            tuneTelemetry.mode = APP_TUNE_MOV_TURN;
            tuneTelemetry.active = 1U;

            tuneTelemetry.runSequence++;
            if (tuneTelemetry.runSequence == 0U)
            {
                tuneTelemetry.runSequence++;
            }

            /* Unified movement telemetry (MM,/EV:START) -- capture only,
             * never read by any control computation. */
            movementStartTick = xTaskGetTickCount();
            movementStartLeftCounts = (float)leftNow;
            movementStartRightCounts = (float)rightNow;
            tuneTelemetry.commandValue = command->value;
            tuneTelemetry.activeDirection = command->type;
            tuneTelemetry.movementStartTick = movementStartTick;

            AppMotion_RebaseRightEncoder(rightNow);

            activeCommand = command->type;

            {
                float headingNow = AppIMU_Heading();

                targetHeadingDeg =
                    headingNow
                    + (float)command->value;

                targetLeftCounts  = (float)leftNow;
                targetRightCounts = (float)rightNow;

                PID_Reset(&leftPID);
                PID_Reset(&rightPID);
                PID_Reset(&steeringPID);

                settledCycles = 0U;
                /* TURNBRAKE: reset at every new turn command entry, same
                 * as settledCycles. */
                turnBrakeQualifyingCycles = 0U;

                AppMotion_EnterTurnOrPreSteer(command->type, command->value);
            }

            break;


        case MOTION_FORWARD_RIGHT:
            if ((command->value < 0) ||
                (command->value > MOV_TURN_MAX_DEGREES))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_BAD_COMMAND,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (command->value == 0)
            {
                activeCommand = command->type;
                AppMotion_StopWithEvent(
                    APP_MOTION_EVENT_MOV_DONE
                );
                break;
            }

            if (!AppIMU_IsReady())
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_IMU_NOT_READY,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (!AppMotion_RequiredPIDsAreSet(
                    APP_MOTION_PID_MASK_LEFT |
                    APP_MOTION_PID_MASK_RIGHT |
                    APP_MOTION_PID_MASK_STEERING))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_PID_NOT_SET,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            AppMotion_CancelTuneTelemetry();

            tuneTelemetry.mode = APP_TUNE_MOV_TURN;
            tuneTelemetry.active = 1U;

            tuneTelemetry.runSequence++;
            if (tuneTelemetry.runSequence == 0U)
            {
                tuneTelemetry.runSequence++;
            }

            /* Unified movement telemetry (MM,/EV:START) -- capture only,
             * never read by any control computation. */
            movementStartTick = xTaskGetTickCount();
            movementStartLeftCounts = (float)leftNow;
            movementStartRightCounts = (float)rightNow;
            tuneTelemetry.commandValue = command->value;
            tuneTelemetry.activeDirection = command->type;
            tuneTelemetry.movementStartTick = movementStartTick;

            AppMotion_RebaseRightEncoder(rightNow);

            activeCommand = command->type;

            {
                float headingNow = AppIMU_Heading();

                targetHeadingDeg =
                    headingNow
                    - (float)command->value;

                targetLeftCounts  = (float)leftNow;
                targetRightCounts = (float)rightNow;

                PID_Reset(&leftPID);
                PID_Reset(&rightPID);
                PID_Reset(&steeringPID);

                settledCycles = 0U;
                /* TURNBRAKE: reset at every new turn command entry, same
                 * as settledCycles. */
                turnBrakeQualifyingCycles = 0U;

                AppMotion_EnterTurnOrPreSteer(command->type, command->value);
            }

            break;


        case MOTION_BACKWARD_LEFT:
            if ((command->value < 0) ||
                (command->value > MOV_TURN_MAX_DEGREES))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_BAD_COMMAND,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (command->value == 0)
            {
                activeCommand = command->type;
                AppMotion_StopWithEvent(
                    APP_MOTION_EVENT_MOV_DONE
                );
                break;
            }

            if (!AppIMU_IsReady())
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_IMU_NOT_READY,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (!AppMotion_RequiredPIDsAreSet(
                    APP_MOTION_PID_MASK_LEFT |
                    APP_MOTION_PID_MASK_RIGHT |
                    APP_MOTION_PID_MASK_STEERING))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_PID_NOT_SET,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            AppMotion_CancelTuneTelemetry();

            tuneTelemetry.mode = APP_TUNE_MOV_TURN;
            tuneTelemetry.active = 1U;

            tuneTelemetry.runSequence++;
            if (tuneTelemetry.runSequence == 0U)
            {
                tuneTelemetry.runSequence++;
            }

            /* Unified movement telemetry (MM,/EV:START) -- capture only,
             * never read by any control computation. */
            movementStartTick = xTaskGetTickCount();
            movementStartLeftCounts = (float)leftNow;
            movementStartRightCounts = (float)rightNow;
            tuneTelemetry.commandValue = command->value;
            tuneTelemetry.activeDirection = command->type;
            tuneTelemetry.movementStartTick = movementStartTick;

            AppMotion_RebaseRightEncoder(rightNow);

            activeCommand = command->type;

            {
                float headingNow = AppIMU_Heading();

                targetHeadingDeg =
                    headingNow
                    + (float)command->value;

                targetLeftCounts  = (float)leftNow;
                targetRightCounts = (float)rightNow;

                PID_Reset(&leftPID);
                PID_Reset(&rightPID);
                PID_Reset(&steeringPID);

                settledCycles = 0U;
                /* TURNBRAKE: reset at every new turn command entry, same
                 * as settledCycles. */
                turnBrakeQualifyingCycles = 0U;

                AppMotion_EnterTurnOrPreSteer(command->type, command->value);
            }

            break;


        case MOTION_BACKWARD_RIGHT:
            if ((command->value < 0) ||
                (command->value > MOV_TURN_MAX_DEGREES))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_BAD_COMMAND,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (command->value == 0)
            {
                activeCommand = command->type;
                AppMotion_StopWithEvent(
                    APP_MOTION_EVENT_MOV_DONE
                );
                break;
            }

            if (!AppIMU_IsReady())
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_IMU_NOT_READY,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (!AppMotion_RequiredPIDsAreSet(
                    APP_MOTION_PID_MASK_LEFT |
                    APP_MOTION_PID_MASK_RIGHT |
                    APP_MOTION_PID_MASK_STEERING))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_PID_NOT_SET,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            AppMotion_CancelTuneTelemetry();

            tuneTelemetry.mode = APP_TUNE_MOV_TURN;
            tuneTelemetry.active = 1U;

            tuneTelemetry.runSequence++;
            if (tuneTelemetry.runSequence == 0U)
            {
                tuneTelemetry.runSequence++;
            }

            /* Unified movement telemetry (MM,/EV:START) -- capture only,
             * never read by any control computation. */
            movementStartTick = xTaskGetTickCount();
            movementStartLeftCounts = (float)leftNow;
            movementStartRightCounts = (float)rightNow;
            tuneTelemetry.commandValue = command->value;
            tuneTelemetry.activeDirection = command->type;
            tuneTelemetry.movementStartTick = movementStartTick;

            AppMotion_RebaseRightEncoder(rightNow);

            activeCommand = command->type;

            {
                float headingNow = AppIMU_Heading();

                targetHeadingDeg =
                    headingNow
                    - (float)command->value;

                targetLeftCounts  = (float)leftNow;
                targetRightCounts = (float)rightNow;

                PID_Reset(&leftPID);
                PID_Reset(&rightPID);
                PID_Reset(&steeringPID);

                settledCycles = 0U;
                /* TURNBRAKE: reset at every new turn command entry, same
                 * as settledCycles. */
                turnBrakeQualifyingCycles = 0U;

                AppMotion_EnterTurnOrPreSteer(command->type, command->value);
            }

            break;


        case MOTION_TUNE_LEFT:
            if ((command->value == 0) ||
                (command->value < -TUNE_MOTOR_MAX_ABS_COUNTS) ||
                (command->value > TUNE_MOTOR_MAX_ABS_COUNTS))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_BAD_COMMAND,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (!AppMotion_RequiredPIDsAreSet(
                    APP_MOTION_PID_MASK_LEFT))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_PID_NOT_SET,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            AppMotion_PrepareTune(
                APP_TUNE_LEFT,
                leftNow,
                rightNow
            );

            targetLeftCounts +=
                (float)command->value;

            tuneTelemetry.leftTargetCounts =
                (int32_t)targetLeftCounts;
            tuneTelemetry.leftErrorCounts =
                command->value;

            motionState = APP_MOTION_TUNE_LEFT;
            activeCommand = command->type;
            AppMotion_PublishTelemetry();
            break;


        case MOTION_TUNE_RIGHT:
            if ((command->value == 0) ||
                (command->value < -TUNE_MOTOR_MAX_ABS_COUNTS) ||
                (command->value > TUNE_MOTOR_MAX_ABS_COUNTS))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_BAD_COMMAND,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (!AppMotion_RequiredPIDsAreSet(
                    APP_MOTION_PID_MASK_RIGHT))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_PID_NOT_SET,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            AppMotion_PrepareTune(
                APP_TUNE_RIGHT,
                leftNow,
                rightNow
            );

            targetRightCounts +=
                (float)command->value;

            tuneTelemetry.rightTargetCounts =
                (int32_t)targetRightCounts;
            tuneTelemetry.rightErrorCounts =
                command->value;

            motionState = APP_MOTION_TUNE_RIGHT;
            activeCommand = command->type;
            AppMotion_PublishTelemetry();
            break;


        case MOTION_TUNE_BOTH:
            if ((command->value == 0) ||
                (command->value < -TUNE_MOTOR_MAX_ABS_COUNTS) ||
                (command->value > TUNE_MOTOR_MAX_ABS_COUNTS))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_BAD_COMMAND,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (!AppMotion_RequiredPIDsAreSet(
                    APP_MOTION_PID_MASK_LEFT |
                    APP_MOTION_PID_MASK_RIGHT))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_PID_NOT_SET,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            AppMotion_PrepareTune(
                APP_TUNE_BOTH,
                leftNow,
                rightNow
            );

            targetLeftCounts +=
                (float)command->value;
            targetRightCounts +=
                (float)command->value;

            tuneTelemetry.leftTargetCounts =
                (int32_t)targetLeftCounts;
            tuneTelemetry.leftErrorCounts =
                command->value;
            tuneTelemetry.rightTargetCounts =
                (int32_t)targetRightCounts;
            tuneTelemetry.rightErrorCounts =
                command->value;

            motionState = APP_MOTION_TUNE_BOTH;
            activeCommand = command->type;
            AppMotion_PublishTelemetry();
            break;


        case MOTION_TUNE_STEERING_FORWARD:
        case MOTION_TUNE_STEERING_BACKWARD:
            if ((command->value < 1) ||
                (command->value > TUNE_STEERING_MAX_SECONDS))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_BAD_COMMAND,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (!AppIMU_IsReady())
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_IMU_NOT_READY,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (!AppMotion_RequiredPIDsAreSet(
                    APP_MOTION_PID_MASK_LEFT |
                    APP_MOTION_PID_MASK_RIGHT |
                    APP_MOTION_PID_MASK_STEERING))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_PID_NOT_SET,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            AppMotion_PrepareTune(
                APP_TUNE_STEERING,
                leftNow,
                rightNow
            );

            tuneDurationSeconds =
                (float)command->value;

            if (command->type ==
                MOTION_TUNE_STEERING_FORWARD)
            {
                tuneSteeringTargetRate =
                    TUNE_STEERING_RATE_COUNTS_PER_SEC;
            }
            else
            {
                tuneSteeringTargetRate =
                    -TUNE_STEERING_RATE_COUNTS_PER_SEC;
            }

            motionState = APP_MOTION_TUNE_STEERING;
            activeCommand = command->type;
            AppMotion_PublishTelemetry();
            break;

        case MOTION_ENCODER_RESET:
            AppEncoder_Reset();
            AppMotion_RebaseRightEncoder(0);
            AppMotion_PublishEvent(
                APP_MOTION_EVENT_ENCODER_RESET_OK,
                command->type,
                0,
                0,
                0,
                0
            );
            break;

        case MOTION_PWM_TEST:
        case MOTION_SPEED_TEST:
            if ((command->value < 0) ||
                (command->value > (int32_t)MOTOR_MAX_DUTY))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_BAD_COMMAND,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            AppMotion_PrepareDiagnostic(
                command->type,
                leftNow,
                rightNow,
                DIAGNOSTIC_RUN_SECONDS
            );
            diagnosticDuty = command->value;

            if (diagnosticDuty == 0)
            {
                AppMotion_FinishDiagnostic(
                    (command->type == MOTION_PWM_TEST) ?
                        APP_MOTION_EVENT_PWM_DONE :
                        APP_MOTION_EVENT_SPEED_DONE,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            AppMotor_Set(diagnosticDuty, diagnosticDuty);
            motionState =
                (command->type == MOTION_PWM_TEST) ?
                    APP_MOTION_DIAGNOSTIC_PWM :
                    APP_MOTION_DIAGNOSTIC_SPEED;
            break;

        case MOTION_SERVO_SET:
            if ((command->value < (int32_t)SERVO_MIN_US) ||
                (command->value > (int32_t)SERVO_MAX_US))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_BAD_COMMAND,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            AppServo_SetUs((uint32_t)command->value);
            AppMotion_PublishEvent(
                APP_MOTION_EVENT_SERVO_SET,
                command->type,
                (int32_t)AppServo_GetUs(),
                0,
                0,
                0
            );
            break;

        case MOTION_IMU_ZERO:
        case MOTION_IMU_FLIP:
        {
            bool requestAccepted;

            if (command->type == MOTION_IMU_ZERO)
            {
                requestAccepted = AppIMU_RequestZero(
                    &awaitedImuEventSequence
                );
            }
            else
            {
                requestAccepted = AppIMU_RequestFlip(
                    &awaitedImuEventSequence
                );
            }

            if (!requestAccepted)
            {
                AppMotion_PublishEvent(
                    (command->type == MOTION_IMU_ZERO) ?
                        APP_MOTION_EVENT_IMU_ZERO_ERROR :
                        APP_MOTION_EVENT_IMU_FLIP_ERROR,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            activeCommand = command->type;
            imuWaitElapsedSeconds = 0.0f;
            motionState = APP_MOTION_WAIT_IMU;
            break;
        }

        case MOTION_HEADING_RAW:
        case MOTION_HEADING_FORWARD:
        case MOTION_HEADING_BACKWARD:
            if ((command->value < 1) ||
                (command->value > DIAGNOSTIC_MAX_SECONDS))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_BAD_COMMAND,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if (!AppIMU_IsReady())
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_IMU_NOT_READY,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            if ((command->type != MOTION_HEADING_RAW) &&
                !AppMotion_RequiredPIDsAreSet(
                    APP_MOTION_PID_MASK_STEERING))
            {
                AppMotion_PublishEvent(
                    APP_MOTION_EVENT_PID_NOT_SET,
                    command->type,
                    0,
                    0,
                    0,
                    0
                );
                break;
            }

            AppMotion_PrepareDiagnostic(
                command->type,
                leftNow,
                rightNow,
                (float)command->value
            );

            if (command->type == MOTION_HEADING_BACKWARD)
            {
                diagnosticDuty = -(int32_t)HDG_TEST_DUTY;
            }
            else
            {
                diagnosticDuty = (int32_t)HDG_TEST_DUTY;
            }

            AppMotor_Set(diagnosticDuty, diagnosticDuty);

            if (command->type == MOTION_HEADING_RAW)
            {
                motionState = APP_MOTION_DIAGNOSTIC_HEADING_RAW;
            }
            else
            {
                motionState = APP_MOTION_DIAGNOSTIC_HEADING;
            }
            break;

        default:
            AppMotion_PublishEvent(
                APP_MOTION_EVENT_BAD_COMMAND,
                command->type,
                0,
                0,
                0,
                0
            );
            break;
    }
}

void AppMotion_Update(float dtSeconds)
{
    switch (motionState)
    {
        case APP_MOTION_IDLE:
            break;


        case APP_MOTION_DISTANCE:
        {
            if (!AppIMU_IsReady())
            {
                AppMotion_BeginBrake(
                    APP_MOTION_EVENT_IMU_NOT_READY
                );
                break;
            }

            float leftNow =
                (float)AppEncoder_Left();

            float rightNow =
                (float)AppMotion_GetRightPosition();
            float leftError =
                targetLeftCounts - leftNow;

            float rightError =
                targetRightCounts - rightNow;

            float headingNow =
                AppIMU_Heading();

            float leftOutput =
                PID_Update(
                    &leftPID,
                    targetLeftCounts,
                    leftNow,
                    dtSeconds
                );

            float rightOutput =
                PID_Update(
                    &rightPID,
                    targetRightCounts,
                    rightNow,
                    dtSeconds
                );

            float steeringOutput =
                PID_Update(
                    &steeringPID,
                    targetHeadingDeg,
                    headingNow,
                    dtSeconds
                );

            /* -------- Stage A2: direct gyro-Z heading rate damping --------
             * Additional term on top of steeringPID's existing, unmodified
             * P-only output -- NOT a PID derivative. currentGyroZDps is a
             * direct, already-bias-corrected hardware rate measurement
             * (AppIMU_GyroZ()), not a finite difference of the integrated
             * heading error, so it does not amplify inter-task sampling
             * jitter the way a Kd term on error would. Active for the
             * entire APP_MOTION_DISTANCE state (same scope steeringPID
             * itself already runs in -- no approach/stall gating, unlike
             * WSYNC, since this trims the SERVO, not the rear wheels).
             * When headingCtrlEnabled is false, finalSteeringOutput ==
             * steeringOutput unconditionally -- byte-identical to today. */
            float currentGyroZDps = AppIMU_GyroZ();

            float headingRateCorrection = 0.0f;
            float finalSteeringOutput = steeringOutput;

            if (headingCtrlEnabled)
            {
                headingRateCorrection =
                    headingCtrlKrate * currentGyroZDps;

                if (headingRateCorrection > headingCtrlMaxCorrection)
                {
                    headingRateCorrection = headingCtrlMaxCorrection;
                }
                else if (headingRateCorrection < -headingCtrlMaxCorrection)
                {
                    headingRateCorrection = -headingCtrlMaxCorrection;
                }

                finalSteeringOutput =
                    steeringOutput - headingRateCorrection;

                /* Same physical bound steeringPID's own output clamp
                 * already uses; AppServo_SetUs() also hard-clamps the
                 * resulting pulse to [SERVO_MIN_US, SERVO_MAX_US], so
                 * this is defense-in-depth for telemetry sanity, not the
                 * only safety net. */
                if (finalSteeringOutput > 530.0f)
                {
                    finalSteeringOutput = 530.0f;
                }
                else if (finalSteeringOutput < -530.0f)
                {
                    finalSteeringOutput = -530.0f;
                }
            }
            /* -------- end Stage A2 heading rate damping -------- */

            /* speedControllerActive is named/shaped to mirror
             * decelProfileActive; reduces to spctrlEnabled for the whole
             * APP_MOTION_DISTANCE duration once armed. Declared here
             * (moved up from inside the Stage A3.3a block below, where it
             * originally lived) because the Stage A3.2 remaining-distance
             * calculation and the new Stage A3.3b profile block both need
             * it before Stage A3.3a's own PID_Update() runs; semantics
             * are exactly unchanged, this is still the only SPCTRL-active
             * expression in this scope -- not a duplicate. */
            bool speedControllerActive = spctrlEnabled;

            /* -------- Stage A3.2: deceleration profile, remaining distance --------
             * Common (chassis), direction-normalized remaining distance.
             * Hoisted here (originally computed just before the base-
             * command selection further below) so it is available BEFORE
             * Stage A3.3a's speedPID update -- Stage A3.3b's profile
             * needs it there too. Formula and result are byte-identical
             * to before the hoist; dprofDirection is a separately-named
             * local (not WSYNC's own motionDirection, not spctrlDirection
             * below, declared later in this same scope) so no other
             * controller's existing line is ever touched -- now shared,
             * unduplicated, by DPROF's own decelProfileActive/profileDuty
             * below AND by Stage A3.3b. */
            float dprofDirection =
                (activeCommand == MOTION_BACKWARD) ? -1.0f : 1.0f;

            float directedLeftRemaining =
                dprofDirection * leftError;

            float directedRightRemaining =
                dprofDirection * rightError;

            /* Positive while the CHASSIS is still short of the commanded
             * distance, for both F and B. Signed, not fabsf()'d -- an
             * already-crossed (negative) average is excluded by each
             * consumer's own activation gate rather than misread as
             * "still far away" by a magnitude-only test. */
            float chassisRemainingCounts =
                0.5f * (directedLeftRemaining + directedRightRemaining);
            /* -------- end Stage A3.2 remaining distance -------- */

            /* -------- Stage A3.3b: distance-based linear speed profile --------
             * Layered on top of the validated Stage A3.3a speed PI.
             * Disabled by default (sprofEnabled==false): effectiveTarget
             * Speed is then always exactly spctrlTargetSpeed, so
             * PID_Update's call below and its telemetry are byte-
             * identical to A3.3a. spctrlTargetSpeed itself is NEVER
             * overwritten here -- it remains the configured cruise target
             * reported by SPCTRL,GET; effectiveTargetSpeed is a separate,
             * per-tick local. Outside the zone (chassisRemainingCounts >
             * sprofDecelStartCounts) the target is cruise, unchanged;
             * inside it, a continuous linear interpolation down to
             * sprofApproachSpeed, clamped to [sprofApproachSpeed,
             * spctrlTargetSpeed] by construction (app_comm.c's SET-time
             * invariant keeps approachSpeed <= the cruise target at all
             * times). No special-casing at/inside POSITION_TOLERANCE_
             * COUNTS is needed: the ratio naturally clamps to 0 there,
             * and the existing, unmodified tolerance-zero branch further
             * below already zeroes the actual motor command by that
             * point regardless of what the speed PI would otherwise
             * output. */
            float effectiveTargetSpeed = spctrlTargetSpeed;

            if (speedControllerActive && sprofEnabled)
            {
                if (chassisRemainingCounts > sprofDecelStartCounts)
                {
                    effectiveTargetSpeed = spctrlTargetSpeed;
                }
                else
                {
                    float sprofRatio =
                        (chassisRemainingCounts - POSITION_TOLERANCE_COUNTS) /
                        (sprofDecelStartCounts - POSITION_TOLERANCE_COUNTS);

                    if (sprofRatio < 0.0f)
                    {
                        sprofRatio = 0.0f;
                    }
                    else if (sprofRatio > 1.0f)
                    {
                        sprofRatio = 1.0f;
                    }

                    effectiveTargetSpeed =
                        sprofApproachSpeed +
                        (spctrlTargetSpeed - sprofApproachSpeed) * sprofRatio;
                }
            }
            /* -------- end Stage A3.3b distance-based linear speed profile -------- */

            /* -------- Stage A3.3a: chassis speed measurement + speed PI --------
             * Direction-normalized, EMA-filtered common chassis speed.
             * spctrlDirection is yet another separately-named local (not
             * motionDirection, not dprofDirection, both declared
             * elsewhere in this same scope) so no other controller's
             * existing line is ever touched. spctrlLeftDelta/
             * spctrlRightDelta compute the IDENTICAL formula the stall
             * detector's own leftMotionDelta/rightMotionDelta will
             * independently compute further below (leftNow -
             * leftDistanceStallPositionCounts, read before that
             * section's own update of the same static) -- a second,
             * independent read of an unmodified persistent static, not a
             * move or alteration of the stall detector's own later,
             * unchanged computation. No fabsf() anywhere in this chain --
             * a wheel genuinely moving opposite the commanded direction
             * must pull the chassis average down/negative, not be
             * masked. speedControllerActive is declared above (Stage A3.2
             * remaining-distance section), not here -- see that
             * declaration's comment for why. */
            float spctrlDirection =
                (activeCommand == MOTION_BACKWARD) ? -1.0f : 1.0f;

            float spctrlLeftDelta =
                leftNow - leftDistanceStallPositionCounts;

            float spctrlRightDelta =
                rightNow - rightDistanceStallPositionCounts;

            float directedLeftSpeed =
                (dtSeconds > 0.0f) ?
                    (spctrlDirection * (spctrlLeftDelta / dtSeconds)) : 0.0f;

            float directedRightSpeed =
                (dtSeconds > 0.0f) ?
                    (spctrlDirection * (spctrlRightDelta / dtSeconds)) : 0.0f;

            float rawChassisSpeed =
                0.5f * (directedLeftSpeed + directedRightSpeed);

            if (!filteredChassisSpeedInitialized)
            {
                filteredChassisSpeed = rawChassisSpeed;
                filteredChassisSpeedInitialized = true;
            }
            else
            {
                filteredChassisSpeed +=
                    SPCTRL_EMA_ALPHA * (rawChassisSpeed - filteredChassisSpeed);
            }

            float speedError = 0.0f;
            float speedPProportional = 0.0f;
            float speedPIntegralContribution = 0.0f;
            float rawBaseDuty = 0.0f;

            if (speedControllerActive)
            {
                rawBaseDuty =
                    PID_Update(
                        &speedPID,
                        effectiveTargetSpeed,
                        filteredChassisSpeed,
                        dtSeconds
                    );

                /* Telemetry-only re-derivation of the P/I split (does
                 * not feed back into rawBaseDuty, which PID_Update has
                 * already fully computed and clamped to [0,MOTOR_MAX_
                 * DUTY] above). Uses effectiveTargetSpeed -- what
                 * PID_Update actually saw as its setpoint this tick --
                 * NOT spctrlTargetSpeed, so SE/PC/IC stay consistent even
                 * while Stage A3.3b's profile is lowering the effective
                 * target below cruise. */
                speedError = effectiveTargetSpeed - filteredChassisSpeed;
                speedPProportional = speedPID.kp * speedError;
                speedPIntegralContribution = speedPID.ki * speedPID.integral;
            }
            /* -------- end Stage A3.3a chassis speed measurement + speed PI -------- */

            /* -------- Stage A3.3c: final controlled stop gate --------
             * Computed here, deliberately EARLY -- right after the inputs
             * it needs (directedLeftRemaining/directedRightRemaining from
             * the Stage A3.2 remaining-distance section above,
             * filteredChassisSpeed from Stage A3.3a just above,
             * speedControllerActive from even earlier in this scope) all
             * already exist, and well BEFORE AppMotor_Set() and
             * AppMotion_PublishTelemetry() run further below. This is a
             * fresh per-tick local, not a static -- recomputed
             * unconditionally from this tick's own values every time this
             * case runs, so there is no stale value carried between ticks
             * or between moves.
             *
             * ADDITIVE ONLY: this boolean is read (not recomputed) by the
             * existing completion check further below and OR'd alongside
             * the existing settledCycles/targetCrossed conditions -- it
             * never replaces them, and when stopctrlEnabled is false this
             * is always false, making Stage A3.3c behavior byte-identical
             * to Stage A3.3b.
             *
             * Distance test reuses directedLeftRemaining/
             * directedRightRemaining verbatim (no fabsf() -- an
             * already-crossed, negative remaining is correctly treated as
             * "close enough" by construction, not falsely flagged as far
             * away). Speed test uses fabsf(filteredChassisSpeed): unlike
             * remaining distance, filteredChassisSpeed's SIGN is not the
             * signal of interest here (it is direction-normalized, so a
             * momentary negative reading still represents real speed
             * magnitude in the wrong instantaneous sense, not "already
             * done" the way a crossed position is) -- the gate must reject
             * "close but still moving fast in either sense", so the
             * comparison is against the MAGNITUDE of the filtered speed. */
            bool stopctrlTriggered =
                speedControllerActive &&
                stopctrlEnabled &&
                (directedLeftRemaining <= stopctrlBrakeLeadCounts) &&
                (directedRightRemaining <= stopctrlBrakeLeadCounts) &&
                (fabsf(filteredChassisSpeed) <= stopctrlBrakeEntrySpeed);
            /* -------- end Stage A3.3c final controlled stop gate -------- */

            /* -------- Stage A3.2: deceleration profile, base command --------
             * leftPID/rightPID above are NOT modified and keep running
             * every tick unconditionally; leftOutput/rightOutput are kept
             * only for MP, telemetry (LPO/RPO) and are NOT used as motor
             * magnitude while decelProfileActive is true. dprofDirection/
             * directedLeftRemaining/directedRightRemaining/
             * chassisRemainingCounts are computed above (Stage A3.2
             * remaining distance, hoisted for Stage A3.3b); everything
             * below is otherwise unchanged from the original Stage A3.2
             * implementation. */
            bool decelProfileActive =
                decelProfileEnabled &&
                (chassisRemainingCounts > POSITION_TOLERANCE_COUNTS) &&
                (chassisRemainingCounts <= decelStartCounts);

            float profileDuty = 0.0f;

            if (decelProfileActive)
            {
                float ratio =
                    (chassisRemainingCounts - POSITION_TOLERANCE_COUNTS) /
                    (decelStartCounts - POSITION_TOLERANCE_COUNTS);

                if (ratio < 0.0f)
                {
                    ratio = 0.0f;
                }
                else if (ratio > 1.0f)
                {
                    ratio = 1.0f;
                }

                profileDuty =
                    decelApproachDuty +
                    ((float)MOTOR_MAX_DUTY - decelApproachDuty) * ratio;
            }

            int32_t leftCommand;
            int32_t rightCommand;

            /* Three-way, mutually exclusive by construction: DPROF and
             * SPCTRL can never both be enabled at once (enforced at
             * their idle-only SET commands, app_comm.c), so at most one
             * of the first two branches can ever be taken -- this is
             * not a runtime precedence choice. */
            if (decelProfileActive)
            {
                leftCommand  = (int32_t)(dprofDirection * profileDuty);
                rightCommand = (int32_t)(dprofDirection * profileDuty);
            }
            else if (speedControllerActive)
            {
                /* Stage A3.3a: common signed base duty from the speed
                 * PI, BEFORE WSYNC. rawBaseDuty is already in
                 * [0, MOTOR_MAX_DUTY] (PID_Update's own output clamp,
                 * see the plan's anti-windup audit) -- applying the
                 * fixed commanded direction here can only ever produce
                 * a same-direction value, never a reversal, by
                 * construction. */
                leftCommand  = (int32_t)(spctrlDirection * rawBaseDuty);
                rightCommand = (int32_t)(spctrlDirection * rawBaseDuty);
            }
            else
            {
                leftCommand  = (int32_t)leftOutput;
                rightCommand = (int32_t)rightOutput;
            }
            /* -------- end Stage A3.2/A3.3a base command selection -------- */

            /* -------- Stage A1: rear-wheel synchronization trim --------
             * Cross-coupled first-line differential correction on the
             * post-PID motor commands. CRUISE ONLY: skipped once either
             * wheel is within APPROACH_ZONE_COUNTS of target and while a
             * stall-recovery episode is active, so the existing approach
             * / reverse-torque braking and stall recovery are untouched.
             * leftPID/rightPID, their gains and resets, are NOT touched
             * -- this only adds/subtracts an integer duty trim, bounded
             * to +-wheelSyncMaxDuty, computed here BEFORE the existing
             * tolerance / min-duty / stall clamp block below. When
             * wheelSyncEnabled is false the whole block is inert. */
            float motionDirection =
                (activeCommand == MOTION_BACKWARD) ? -1.0f : 1.0f;

            float wheelSyncLeftProgress =
                motionDirection *
                ((float)leftNow - distanceLeftStartCounts);

            float wheelSyncRightProgress =
                motionDirection *
                ((float)rightNow - distanceRightStartCounts);

            /* Positive => LEFT wheel is physically ahead (F or B). */
            float wheelSyncError =
                wheelSyncLeftProgress - wheelSyncRightProgress;

            bool wheelSyncCruiseActive =
                wheelSyncEnabled &&
                (fabsf(leftError) > APPROACH_ZONE_COUNTS) &&
                (fabsf(rightError) > APPROACH_ZONE_COUNTS) &&
                !distanceStallRecoveryActive;

            float wheelSyncCorrection = 0.0f;

            if (wheelSyncCruiseActive)
            {
                wheelSyncCorrection =
                    wheelSyncGain * wheelSyncError;

                if (wheelSyncCorrection > wheelSyncMaxDuty)
                {
                    wheelSyncCorrection = wheelSyncMaxDuty;
                }
                else if (wheelSyncCorrection < -wheelSyncMaxDuty)
                {
                    wheelSyncCorrection = -wheelSyncMaxDuty;
                }

                /* Slow the leading wheel / speed the lagging wheel.
                 * The existing AppMotor_Set() +-99 clamp absorbs the
                 * futile "speed up" half whenever that wheel is already
                 * saturated, leaving only the "slow down" half acting. */
                leftCommand -=
                    (int32_t)(motionDirection * wheelSyncCorrection);
                rightCommand +=
                    (int32_t)(motionDirection * wheelSyncCorrection);
            }
            /* -------- end Stage A1 wheel-sync trim -------- */

            /* -------- Stage A3.2: final per-wheel DECEL enforcement --------
             * Runs AFTER the existing, unmodified WSYNC block above. While
             * a wheel is still short of ITS OWN target (using the FIXED
             * commanded direction dprofDirection, never that wheel's own
             * possibly-overshot error sign), the profile is the
             * authoritative two-sided band: WSYNC may only move a wheel
             * within [decelApproachDuty, profileDuty] (forward) or
             * [-profileDuty, -decelApproachDuty] (backward) -- never
             * outside it. A wheel that has individually finished or
             * overshot is held at exactly 0, NEVER driven opposite the
             * commanded travel direction to correct back -- only the
             * existing AppMotion_BeginBrake() (settle/crossing/timeout,
             * unmodified) acts on it again before the move ends. */
            bool leftDecelEnforced = false;
            bool rightDecelEnforced = false;

            if (decelProfileActive)
            {
                int32_t leftBeforeEnforce = leftCommand;

                if (directedLeftRemaining > POSITION_TOLERANCE_COUNTS)
                {
                    if (dprofDirection > 0.0f)
                    {
                        if (leftCommand < (int32_t)decelApproachDuty)
                        {
                            leftCommand = (int32_t)decelApproachDuty;
                        }
                        else if (leftCommand > (int32_t)profileDuty)
                        {
                            leftCommand = (int32_t)profileDuty;
                        }
                    }
                    else
                    {
                        if (leftCommand > -(int32_t)decelApproachDuty)
                        {
                            leftCommand = -(int32_t)decelApproachDuty;
                        }
                        else if (leftCommand < -(int32_t)profileDuty)
                        {
                            leftCommand = -(int32_t)profileDuty;
                        }
                    }
                }
                else
                {
                    leftCommand = 0;
                }

                leftDecelEnforced = (leftCommand != leftBeforeEnforce);

                int32_t rightBeforeEnforce = rightCommand;

                if (directedRightRemaining > POSITION_TOLERANCE_COUNTS)
                {
                    if (dprofDirection > 0.0f)
                    {
                        if (rightCommand < (int32_t)decelApproachDuty)
                        {
                            rightCommand = (int32_t)decelApproachDuty;
                        }
                        else if (rightCommand > (int32_t)profileDuty)
                        {
                            rightCommand = (int32_t)profileDuty;
                        }
                    }
                    else
                    {
                        if (rightCommand > -(int32_t)decelApproachDuty)
                        {
                            rightCommand = -(int32_t)decelApproachDuty;
                        }
                        else if (rightCommand < -(int32_t)profileDuty)
                        {
                            rightCommand = -(int32_t)profileDuty;
                        }
                    }
                }
                else
                {
                    rightCommand = 0;
                }

                rightDecelEnforced = (rightCommand != rightBeforeEnforce);
            }
            /* -------- end Stage A3.2 final per-wheel DECEL enforcement -------- */

            /* -------- Stage A3.3a: post-WSYNC no-reversal clamp --------
             * Runs AFTER the existing, unmodified WSYNC block. WSYNC's
             * own gate depends on POSITION error, independent of
             * SPCTRL's speed target -- WSYNC can be fully active while
             * rawBaseDuty is legitimately small (a low test target, or
             * early in a gentle ramp-up), and its correction can then
             * exceed rawBaseDuty and flip a wheel's sign. Clamp to
             * [0, MOTOR_MAX_DUTY] (forward) or [-MOTOR_MAX_DUTY, 0]
             * (backward), using the FIXED commanded direction
             * spctrlDirection, never a wheel's own live error sign --
             * same discipline as dprofDirection above. Floor is exactly
             * 0, not a nonzero minimum: forcing a floor here would
             * defeat closed-loop low-speed regulation. A genuinely too-
             * weak output that cannot sustain real motion is caught by
             * the existing, unmodified stall-recovery entry test. */
            if (speedControllerActive)
            {
                if (spctrlDirection > 0.0f)
                {
                    if (leftCommand < 0) { leftCommand = 0; }
                    if (leftCommand > (int32_t)MOTOR_MAX_DUTY) { leftCommand = (int32_t)MOTOR_MAX_DUTY; }
                    if (rightCommand < 0) { rightCommand = 0; }
                    if (rightCommand > (int32_t)MOTOR_MAX_DUTY) { rightCommand = (int32_t)MOTOR_MAX_DUTY; }
                }
                else
                {
                    if (leftCommand > 0) { leftCommand = 0; }
                    if (leftCommand < -(int32_t)MOTOR_MAX_DUTY) { leftCommand = -(int32_t)MOTOR_MAX_DUTY; }
                    if (rightCommand > 0) { rightCommand = 0; }
                    if (rightCommand < -(int32_t)MOTOR_MAX_DUTY) { rightCommand = -(int32_t)MOTOR_MAX_DUTY; }
                }
            }
            /* -------- end Stage A3.3a post-WSYNC no-reversal clamp -------- */

            /* Telemetry-only capture of the common base command as it
             * stands immediately after WSYNC and the no-reversal clamp,
             * before the legacy tolerance/floor chain and RTGUARD can
             * still touch it -- lets SPC,FB be compared against the
             * final MD,LO/RO to see how much those later stages changed
             * it (e.g. to infer stall-recovery engagement). leftCommand
             * is used as the representative value; WSYNC's own trim is
             * bounded (<=maxDuty, bench value 15) so left/right diverge
             * only slightly at this point. */
            float spctrlFinalBaseDuty = (float)leftCommand;

            /* Stage A3.3a: active-this-tick union used to gate the legacy
             * floor below. Deliberately uses only per-tick "active" flags,
             * never the persistent "enabled" flags -- decelProfileEnabled
             * can be true while decelProfileActive is still false (DPROF
             * armed but still in CRUISE), and gating on the enabled flag
             * would incorrectly bypass the legacy floor during that
             * phase. speedControllerActive already mirrors this shape
             * (== spctrlEnabled in A3.3a, no distance-based sub-phase yet;
             * ready for A3.3b to give it real zone logic without a rename). */
            bool magnitudeControllerActive =
                decelProfileActive || speedControllerActive;

            /* Left motor: allow PID/KD to coast/brake inside approach zone. */
            if ((leftError >= -POSITION_TOLERANCE_COUNTS) &&
                (leftError <= POSITION_TOLERANCE_COUNTS))
            {
                leftCommand = 0;
            }
            else if (!magnitudeControllerActive &&
                     (leftError > APPROACH_ZONE_COUNTS) &&
                     (leftCommand > 0) &&
                     (leftCommand < LEFT_MOTOR_MIN_DUTY))
            {
                leftCommand = LEFT_MOTOR_MIN_DUTY;
            }
            else if (!magnitudeControllerActive &&
                     (leftError < -APPROACH_ZONE_COUNTS) &&
                     (leftCommand < 0) &&
                     (leftCommand > -LEFT_MOTOR_MIN_DUTY))
            {
                leftCommand = -LEFT_MOTOR_MIN_DUTY;
            }

            /* -------- Stage A3.1: reverse-torque guard (left) --------
             * Scoped strictly to the existing per-wheel approach zone
             * (POSITION_TOLERANCE_COUNTS < |error| <= APPROACH_ZONE_
             * COUNTS) -- never evaluated in cruise (> APPROACH_ZONE_
             * COUNTS, where WSYNC/the two floor branches above operate)
             * or inside tolerance (already forced to 0 above). LR
             * telemetry captures the command as it stood after WSYNC and
             * the existing tolerance/floor branches, immediately before
             * this suppression. */
            int32_t leftPreGuardCommand = leftCommand;
            bool leftGuardActive = false;

            if (rtGuardEnabled &&
                (fabsf(leftError) > POSITION_TOLERANCE_COUNTS) &&
                (fabsf(leftError) <= APPROACH_ZONE_COUNTS) &&
                (((leftError > 0.0f) && (leftCommand < 0)) ||
                 ((leftError < 0.0f) && (leftCommand > 0))))
            {
                leftCommand = 0;
                leftGuardActive = true;
            }
            /* -------- end Stage A3.1 reverse-torque guard (left) -------- */

            /* Right motor: allow PID/KD to coast/brake inside approach zone. */
            if ((rightError >= -POSITION_TOLERANCE_COUNTS) &&
                (rightError <= POSITION_TOLERANCE_COUNTS))
            {
                rightCommand = 0;
            }
            else if (!magnitudeControllerActive &&
                     (rightError > APPROACH_ZONE_COUNTS) &&
                     (rightCommand > 0) &&
                     (rightCommand < RIGHT_MOTOR_MIN_DUTY))
            {
                rightCommand = RIGHT_MOTOR_MIN_DUTY;
            }
            else if (!magnitudeControllerActive &&
                     (rightError < -APPROACH_ZONE_COUNTS) &&
                     (rightCommand < 0) &&
                     (rightCommand > -RIGHT_MOTOR_MIN_DUTY))
            {
                rightCommand = -RIGHT_MOTOR_MIN_DUTY;
            }

            /* -------- Stage A3.1: reverse-torque guard (right) -------- */
            int32_t rightPreGuardCommand = rightCommand;
            bool rightGuardActive = false;

            if (rtGuardEnabled &&
                (fabsf(rightError) > POSITION_TOLERANCE_COUNTS) &&
                (fabsf(rightError) <= APPROACH_ZONE_COUNTS) &&
                (((rightError > 0.0f) && (rightCommand < 0)) ||
                 ((rightError < 0.0f) && (rightCommand > 0))))
            {
                rightCommand = 0;
                rightGuardActive = true;
            }
            /* -------- end Stage A3.1 reverse-torque guard (right) -------- */

            /* Stall recovery, entry detection: unchanged. Flags a motor
             * that has shown near-zero encoder motion for several ticks
             * while still materially away from target and under the
             * useful-duty floor above. Never trips while a motor is
             * actually moving/braking, so approach-zone/derivative
             * braking above is unaffected whenever it is already
             * working (e.g. the F,40/B,40 braking tail). */
            float leftMotionDelta =
                leftNow - leftDistanceStallPositionCounts;
            leftDistanceStallPositionCounts = leftNow;

            float rightMotionDelta =
                rightNow - rightDistanceStallPositionCounts;
            rightDistanceStallPositionCounts = rightNow;

            if (!distanceStallRecoveryActive)
            {
                if ((fabsf(leftMotionDelta) > STALL_MOTION_THRESHOLD_COUNTS) ||
                    (fabsf(leftError) <= POSITION_TOLERANCE_COUNTS))
                {
                    leftDistanceStallCycles = 0U;
                }
                else if (leftDistanceStallCycles < UINT16_MAX)
                {
                    leftDistanceStallCycles++;
                }

                if ((fabsf(rightMotionDelta) > STALL_MOTION_THRESHOLD_COUNTS) ||
                    (fabsf(rightError) <= POSITION_TOLERANCE_COUNTS))
                {
                    rightDistanceStallCycles = 0U;
                }
                else if (rightDistanceStallCycles < UINT16_MAX)
                {
                    rightDistanceStallCycles++;
                }

                bool leftStalled =
                    (fabsf(leftError) > POSITION_TOLERANCE_COUNTS) &&
                    (leftDistanceStallCycles >= STALL_DETECT_CYCLES) &&
                    (fabsf((float)leftCommand) < (float)LEFT_MOTOR_MIN_DUTY);

                bool rightStalled =
                    (fabsf(rightError) > POSITION_TOLERANCE_COUNTS) &&
                    (rightDistanceStallCycles >= STALL_DETECT_CYCLES) &&
                    (fabsf((float)rightCommand) < (float)RIGHT_MOTOR_MIN_DUTY);

                /* Either wheel stalling opens one shared episode so a
                 * momentary tick of motion on one side can never
                 * silently cancel recovery for the other. */
                if (leftStalled || rightStalled)
                {
                    distanceStallRecoveryActive = true;

                    leftStallRecoveryStartCounts = leftNow;
                    leftStallRecoveryDirection =
                        (leftError >= 0.0f) ? 1.0f : -1.0f;
                    leftStallRecovered = false;

                    rightStallRecoveryStartCounts = rightNow;
                    rightStallRecoveryDirection =
                        (rightError >= 0.0f) ? 1.0f : -1.0f;
                    rightStallRecovered = false;
                }
            }

            /* Stall recovery, per-wheel completion: each wheel is only
             * force-driven while it is both outside tolerance and has
             * not yet made enough directed (toward-its-own-target)
             * progress since the episode began. The moment a wheel
             * clears either condition it is marked recovered and is
             * never forced again for the rest of this episode, even if
             * its own PID output later dips below the floor again (e.g.
             * a legitimate small/negative braking command) -- recovery
             * must not suppress real PID braking once a wheel has
             * genuinely broken free. */
            if (distanceStallRecoveryActive)
            {
                if (!leftStallRecovered)
                {
                    if (fabsf(leftError) <= POSITION_TOLERANCE_COUNTS)
                    {
                        leftStallRecovered = true;
                    }
                    else
                    {
                        float leftDirectedProgress =
                            leftStallRecoveryDirection *
                            (leftNow - leftStallRecoveryStartCounts);

                        if (leftDirectedProgress >=
                            STALL_RECOVERY_EXIT_PROGRESS_COUNTS)
                        {
                            leftStallRecovered = true;
                        }
                        else if (fabsf((float)leftCommand) <
                                 (float)LEFT_MOTOR_MIN_DUTY)
                        {
                            leftCommand = (leftError > 0.0f) ?
                                (int32_t)LEFT_MOTOR_MIN_DUTY :
                                -(int32_t)LEFT_MOTOR_MIN_DUTY;
                        }
                    }
                }

                if (!rightStallRecovered)
                {
                    if (fabsf(rightError) <= POSITION_TOLERANCE_COUNTS)
                    {
                        rightStallRecovered = true;
                    }
                    else
                    {
                        float rightDirectedProgress =
                            rightStallRecoveryDirection *
                            (rightNow - rightStallRecoveryStartCounts);

                        if (rightDirectedProgress >=
                            STALL_RECOVERY_EXIT_PROGRESS_COUNTS)
                        {
                            rightStallRecovered = true;
                        }
                        else if (fabsf((float)rightCommand) <
                                 (float)RIGHT_MOTOR_MIN_DUTY)
                        {
                            rightCommand = (rightError > 0.0f) ?
                                (int32_t)RIGHT_MOTOR_MIN_DUTY :
                                -(int32_t)RIGHT_MOTOR_MIN_DUTY;
                        }
                    }
                }

                if (leftStallRecovered && rightStallRecovered)
                {
                    distanceStallRecoveryActive = false;
                    leftDistanceStallCycles = 0U;
                    rightDistanceStallCycles = 0U;
                }
            }

            AppMotor_Set(leftCommand, rightCommand);

            /* -------- Stage SFF1: speed-aware servo feed-forward --------
             * Additive-only term layered on top of the already-fully-
             * computed finalSteeringOutput (steeringPID + HCTRL, both
             * above, completely unmodified) -- never blended into that
             * math, only added into the final servoPulse assembly below.
             * Keyed on filteredChassisSpeed (Stage A3.3a, already
             * direction-normalized and computed unconditionally every
             * tick above) -- not re-derived here, no new speed
             * measurement. speedForFF is defensively clamped to >=0
             * (filteredChassisSpeed can dip negative only transiently,
             * e.g. at launch or a momentary reversal; a negative input
             * would already fall into the "at/below startSpeed" branch
             * below since startSpeed's own parser floor is >=0, so this
             * clamp changes no real-world behaviour, it only removes any
             * ambiguity). Independent per direction: activeCommand
             * selects exactly one direction's config, mirroring every
             * other F/B-independent controller's own selection pattern
             * in this function (e.g. dprofDirection/spctrlDirection
             * above). Disabled-is-inert: sffOutputUs is exactly 0 for a
             * disabled direction, making servoPulse below byte-identical
             * to before this stage. Deliberately does NOT feed back into
             * finalSteeringOutput/steeringOutput/steeringPID/HCTRL -- SFF
             * complements those, it does not retune them. */
            bool sffActiveEnabled =
                (activeCommand == MOTION_BACKWARD) ?
                    sffBackwardEnabled : sffForwardEnabled;

            float sffStartSpeed =
                (activeCommand == MOTION_BACKWARD) ?
                    sffBackwardStartSpeed : sffForwardStartSpeed;

            float sffFullSpeed =
                (activeCommand == MOTION_BACKWARD) ?
                    sffBackwardFullSpeed : sffForwardFullSpeed;

            float sffMaxUs =
                (activeCommand == MOTION_BACKWARD) ?
                    sffBackwardMaxUs : sffForwardMaxUs;

            int32_t sffOutputUs = 0;

            if (sffActiveEnabled)
            {
                float speedForFF = filteredChassisSpeed;

                if (speedForFF < 0.0f)
                {
                    speedForFF = 0.0f;
                }

                float sffRatio;

                if (speedForFF <= sffStartSpeed)
                {
                    sffRatio = 0.0f;
                }
                else if (speedForFF >= sffFullSpeed)
                {
                    sffRatio = 1.0f;
                }
                else
                {
                    sffRatio =
                        (speedForFF - sffStartSpeed) /
                        (sffFullSpeed - sffStartSpeed);
                }

                sffOutputUs = (int32_t)(sffMaxUs * sffRatio);
            }
            /* -------- end Stage SFF1 speed-aware servo feed-forward -------- */

            int32_t servoPulse;

            if (activeCommand == MOTION_BACKWARD)
            {
                servoPulse =
                    (int32_t)SERVO_CENTER_US
                    + (int32_t)finalSteeringOutput
                    + sffOutputUs;
            }
            else
            {
                servoPulse =
                    (int32_t)SERVO_CENTER_US
                    - (int32_t)finalSteeringOutput
                    + sffOutputUs;
            }

            /* Stage SFF1: explicit signed clamp BEFORE the uint32_t cast
             * (defense-in-depth, same pattern the earlier, since-reverted
             * Stage A5.3a straight-servo-trim work used). Before this
             * stage, finalSteeringOutput's own +-530 clamp kept
             * servoPulse exactly inside [SERVO_MIN_US,SERVO_MAX_US] with
             * no margin left over; sffOutputUs can now push it outside
             * that range, and a negative int32_t would otherwise wrap to
             * a huge value on the (uint32_t) cast below, before
             * AppServo_SetUs()'s own clamp ever saw it -- this clamp is
             * the only thing standing between that wrap and the servo. */
            if (servoPulse < (int32_t)SERVO_MIN_US)
            {
                servoPulse = (int32_t)SERVO_MIN_US;
            }
            else if (servoPulse > (int32_t)SERVO_MAX_US)
            {
                servoPulse = (int32_t)SERVO_MAX_US;
            }

            AppServo_SetUs(
                (uint32_t)servoPulse
            );

            tuneTelemetry.leftTargetCounts =
                (int32_t)targetLeftCounts;

            tuneTelemetry.leftPositionCounts =
                (int32_t)leftNow;

            tuneTelemetry.leftErrorCounts =
                (int32_t)leftError;

            tuneTelemetry.leftOutput =
                leftCommand;

            tuneTelemetry.rightTargetCounts =
                (int32_t)targetRightCounts;

            tuneTelemetry.rightPositionCounts =
                (int32_t)rightNow;

            tuneTelemetry.rightErrorCounts =
                (int32_t)rightError;

            tuneTelemetry.rightOutput =
                rightCommand;

            tuneTelemetry.headingTargetX10 =
                AppMotion_ToTenths(targetHeadingDeg);

            tuneTelemetry.headingCurrentX10 =
                AppMotion_ToTenths(headingNow);

            tuneTelemetry.headingErrorX10 =
                AppMotion_ToTenths(
                    targetHeadingDeg - headingNow
                );

            tuneTelemetry.steeringCorrectionUs =
                (int32_t)steeringOutput;

            tuneTelemetry.servoPulseUs =
                (uint32_t)servoPulse;

            /* Stage A1: wheel-sync tuning visibility (MS, line). SC is
             * the trim as calculated (clamped), BEFORE the final
             * tolerance/min-duty/stall clamp; 0 when sync is inactive. */
            tuneTelemetry.distanceLeftProgressCounts =
                (int32_t)wheelSyncLeftProgress;
            tuneTelemetry.distanceRightProgressCounts =
                (int32_t)wheelSyncRightProgress;
            tuneTelemetry.wheelSyncErrorCounts =
                (int32_t)wheelSyncError;
            tuneTelemetry.wheelSyncCorrectionDuty =
                (int32_t)wheelSyncCorrection;

            /* Stage A2: heading rate-damping tuning visibility (HS, line).
             * headingFinalCorrectionUs == steeringCorrectionUs whenever
             * headingCtrlEnabled is false. */
            tuneTelemetry.headingYawRateX10 =
                AppMotion_ToTenths(currentGyroZDps);
            tuneTelemetry.headingRateCorrectionUs =
                (int32_t)headingRateCorrection;
            tuneTelemetry.headingFinalCorrectionUs =
                (int32_t)finalSteeringOutput;

            /* Stage A3.1: reverse-torque guard tuning visibility (RTG,
             * line). settledCycleCount reports the existing, unmodified
             * settledCycles static as it stands entering this tick (the
             * real increment/reset below is untouched). targetCrossed
             * is duplicated read-only here, purely for telemetry, using
             * the identical expression the unmodified decision further
             * below computes independently -- this does not move or
             * alter that decision. */
            tuneTelemetry.leftPreGuardCommand = leftPreGuardCommand;
            tuneTelemetry.rightPreGuardCommand = rightPreGuardCommand;
            tuneTelemetry.leftGuardActive = leftGuardActive ? 1U : 0U;
            tuneTelemetry.rightGuardActive = rightGuardActive ? 1U : 0U;
            tuneTelemetry.settledCycleCount = (int32_t)settledCycles;

            {
                float telemetryAverageError =
                    (leftError + rightError) * 0.5f;
                bool telemetryTargetCrossed =
                    (activeCommand == MOTION_BACKWARD) ?
                        (telemetryAverageError >= 0.0f) :
                        (telemetryAverageError <= 0.0f);

                tuneTelemetry.targetCrossedFlag =
                    telemetryTargetCrossed ? 1U : 0U;
            }

            /* Stage A3.2: deceleration-profile tuning visibility (MP,
             * line). LPO/RPO are the raw leftPID/rightPID outputs,
             * captured before the profile substitution and before WSYNC
             * -- genuinely "before WSYNC", unlike MD,LO/RO which are the
             * final post-everything commands. LV/RV reuse the
             * already-computed, unfiltered per-tick motion deltas
             * (existing stall-detection inputs) scaled to counts/s --
             * observability only, not a new sensing subsystem. LDR/RDR
             * are the direction-normalized per-wheel remaining counts,
             * so an early-finished wheel (LDR or RDR <= tolerance while
             * the other is still large) is directly visible without
             * needing to know which direction the move is. LF/RF are 1
             * exactly on ticks where the final per-wheel enforcement
             * above changed the post-WSYNC value it was handed. */
            tuneTelemetry.decelProfileActiveFlag =
                decelProfileActive ? 1U : 0U;
            tuneTelemetry.decelProfileDuty =
                (int32_t)profileDuty;
            tuneTelemetry.leftRawPidOutput =
                (int32_t)leftOutput;
            tuneTelemetry.rightRawPidOutput =
                (int32_t)rightOutput;
            tuneTelemetry.leftVelocityCountsPerSec =
                (dtSeconds > 0.0f) ? (int32_t)(leftMotionDelta / dtSeconds) : 0;
            tuneTelemetry.rightVelocityCountsPerSec =
                (dtSeconds > 0.0f) ? (int32_t)(rightMotionDelta / dtSeconds) : 0;
            tuneTelemetry.leftDecelEnforcedFlag =
                leftDecelEnforced ? 1U : 0U;
            tuneTelemetry.rightDecelEnforcedFlag =
                rightDecelEnforced ? 1U : 0U;
            tuneTelemetry.leftDirectedRemainingCounts =
                (int32_t)directedLeftRemaining;
            tuneTelemetry.rightDirectedRemainingCounts =
                (int32_t)directedRightRemaining;

            /* Stage A3.3a: closed-loop chassis speed controller tuning
             * visibility (SPC, line). ASP/RSP are the raw and EMA-
             * filtered chassis speed respectively (RSP is what the PI
             * actually sees) -- UNCHANGED by Stage A3.3b. PC/IC are the
             * speedPID proportional/integral contributions this tick, 0
             * whenever the controller is not active. RB is the raw PID
             * output magnitude (already in [0,MOTOR_MAX_DUTY]); FB is the
             * signed common base as it stood right after WSYNC and the
             * no-reversal clamp, before the legacy floor/RTGUARD. TSP
             * stays the configured cruise target (SPCTRL,GET); Stage
             * A3.3b adds DSP (effectiveTargetSpeed, what PID_Update
             * actually used this tick -- equals TSP whenever SPPROF is
             * disabled or outside the decel zone) and REM
             * (chassisRemainingCounts, signed/direction-normalized). */
            tuneTelemetry.speedControlActiveFlag =
                speedControllerActive ? 1U : 0U;
            tuneTelemetry.speedControlTargetSpeed =
                (int32_t)spctrlTargetSpeed;
            tuneTelemetry.speedControlActualSpeed =
                (int32_t)rawChassisSpeed;
            tuneTelemetry.speedControlFilteredSpeed =
                (int32_t)filteredChassisSpeed;
            tuneTelemetry.speedControlErrorSpeed =
                (int32_t)speedError;
            tuneTelemetry.speedControlProportionalContribution =
                (int32_t)speedPProportional;
            tuneTelemetry.speedControlIntegralContribution =
                (int32_t)speedPIntegralContribution;
            tuneTelemetry.speedControlRawBaseDuty =
                (int32_t)rawBaseDuty;
            tuneTelemetry.speedControlFinalBaseDuty =
                (int32_t)spctrlFinalBaseDuty;
            tuneTelemetry.speedControlDynamicTargetSpeed =
                (int32_t)effectiveTargetSpeed;
            tuneTelemetry.speedControlChassisRemainingCounts =
                (int32_t)chassisRemainingCounts;

            /* Stage A3.3c: final controlled stop gate tuning visibility
             * (STP, line). TriggeredFlag is stopctrlTriggered exactly as
             * computed above -- best-effort delivery: AppMotion_BeginBrake()
             * (unmodified) republishes telemetry with mode/active cleared
             * immediately after this tick's own publish whenever this flag
             * causes a brake, so a bench operator should treat a captured
             * TR:1 as a bonus confirmation, not rely on it exclusively --
             * MP,LDR/RDR and SPC,RSP (continuously published, not one-shot)
             * are the primary tuning evidence. */
            tuneTelemetry.stopControlEnabledFlag =
                stopctrlEnabled ? 1U : 0U;
            tuneTelemetry.stopControlTriggeredFlag =
                stopctrlTriggered ? 1U : 0U;

            /* Stage SFF1: speed-aware servo feed-forward tuning
             * visibility (SFF, line). Direction/EnabledFlag/OutputUs
             * mirror exactly what the servoPulse block above just used
             * this tick. FilteredSpeed duplicates filteredChassisSpeed
             * (already published via SPC,RSP) so RSP and FF can be read
             * from one frame during tuning. */
            tuneTelemetry.sffDirection =
                (uint8_t)((activeCommand == MOTION_BACKWARD) ? 'B' : 'F');
            tuneTelemetry.sffEnabledFlag =
                sffActiveEnabled ? 1U : 0U;
            tuneTelemetry.sffFilteredSpeed =
                (int32_t)filteredChassisSpeed;
            tuneTelemetry.sffOutputUs =
                sffOutputUs;

            /* Unified movement telemetry (MM, line) -- telemetry-only,
             * never read by any control computation. activeDirection was
             * already set at command entry; refreshed here defensively so
             * it always mirrors activeCommand. leftDisplacementCounts/
             * rightDisplacementCounts are RAW SIGNED, NOT direction-
             * normalized -- see the doc comment in app_motion.h. */
            tuneTelemetry.activeDirection = activeCommand;
            tuneTelemetry.elapsedMs =
                xTaskGetTickCount() - movementStartTick;
            tuneTelemetry.leftDisplacementCounts =
                (int32_t)(leftNow - movementStartLeftCounts);
            tuneTelemetry.rightDisplacementCounts =
                (int32_t)(rightNow - movementStartRightCounts);
            tuneTelemetry.movementBraking = 0U;
            tuneTelemetry.movementRecoveryActive =
                distanceStallRecoveryActive ? 1U : 0U;
            tuneTelemetry.movementPreSteering = 0U;

            AppMotion_PublishTelemetry();

            if ((leftError >= -POSITION_TOLERANCE_COUNTS) &&
                (leftError <= POSITION_TOLERANCE_COUNTS) &&
                (rightError >= -POSITION_TOLERANCE_COUNTS) &&
                (rightError <= POSITION_TOLERANCE_COUNTS))
            {
                settledCycles++;
            }
            else
            {
                settledCycles = 0U;
            }

            float averageError =
                (leftError + rightError) * 0.5f;

            bool targetCrossed;

            if (activeCommand == MOTION_BACKWARD)
            {
                targetCrossed = (averageError >= 0.0f);
            }
            else
            {
                targetCrossed = (averageError <= 0.0f);
            }

            /* Stage A3.3c: stopctrlTriggered (computed earlier this tick,
             * see the Stage A3.3c block above) is OR'd in here, ADDITIVE
             * only -- the existing settledCycles/targetCrossed paths are
             * completely unchanged and remain fully active as fallbacks.
             * When stopctrlEnabled is false, stopctrlTriggered is always
             * false and this condition is byte-identical to Stage A3.3b. */
            if ((settledCycles >= SETTLED_CYCLES_REQUIRED) ||
                targetCrossed ||
                stopctrlTriggered)
            {
                AppMotion_BeginBrake(
                    APP_MOTION_EVENT_MOV_DONE
                );
            }
            else if (AppMotion_DeadlineReached(
                         movementDeadlineTick))
            {
                AppMotion_BeginBrake(
                    APP_MOTION_EVENT_MOTION_TIMEOUT
                );
            }

            break;
        }


        case APP_MOTION_PRE_STEER:
        {
            if (!AppIMU_IsReady())
            {
                AppMotion_BeginBrake(
                    APP_MOTION_EVENT_IMU_NOT_READY
                );
                break;
            }

            float leftNow = (float)AppEncoder_Left();
            float rightNow = (float)AppMotion_GetRightPosition();
            float headingNow = AppIMU_Heading();

            /* Unified movement telemetry (MM, line, ST:PRE) --
             * telemetry-only, never read by any control computation.
             * LO/RO are truthfully 0 -- leftPID/rightPID are never
             * updated during PRE_STEER, this is not a placeholder
             * standing in for a real value. LT/RT are the frozen
             * command-entry targets (never ramped here). SP is the
             * actual last-commanded pulse (the one-shot seed, read
             * back via the existing AppServo_GetUs() getter -- no new
             * static needed). Motors are held at AppMotor_Stop()
             * (commanded once, at command entry) -- not re-commanded
             * here, matching the "motors commanded zero" requirement
             * without any per-tick motor call in this case. */
            tuneTelemetry.leftTargetCounts =
                (int32_t)targetLeftCounts;
            tuneTelemetry.leftPositionCounts =
                (int32_t)leftNow;
            tuneTelemetry.leftErrorCounts =
                (int32_t)(targetLeftCounts - leftNow);
            tuneTelemetry.leftOutput = 0;

            tuneTelemetry.rightTargetCounts =
                (int32_t)targetRightCounts;
            tuneTelemetry.rightPositionCounts =
                (int32_t)rightNow;
            tuneTelemetry.rightErrorCounts =
                (int32_t)(targetRightCounts - rightNow);
            tuneTelemetry.rightOutput = 0;

            tuneTelemetry.headingTargetX10 =
                AppMotion_ToTenths(targetHeadingDeg);
            tuneTelemetry.headingCurrentX10 =
                AppMotion_ToTenths(headingNow);
            tuneTelemetry.headingErrorX10 =
                AppMotion_ToTenths(targetHeadingDeg - headingNow);
            tuneTelemetry.headingYawRateX10 =
                AppMotion_ToTenths(AppIMU_GyroZ());
            tuneTelemetry.steeringCorrectionUs = 0;
            tuneTelemetry.servoPulseUs = AppServo_GetUs();

            tuneTelemetry.activeDirection = activeCommand;
            tuneTelemetry.elapsedMs =
                xTaskGetTickCount() - movementStartTick;
            tuneTelemetry.leftDisplacementCounts =
                (int32_t)(leftNow - movementStartLeftCounts);
            tuneTelemetry.rightDisplacementCounts =
                (int32_t)(rightNow - movementStartRightCounts);
            tuneTelemetry.movementBraking = 0U;
            tuneTelemetry.movementRecoveryActive = 0U;
            tuneTelemetry.movementPreSteering = 1U;

            /* Ackermann-v2: no ramp occurs during PRE_STEER (target
             * counts are frozen, see above) -- report 0, not the
             * currently-configured rates, so AK, honestly reflects
             * "inactive" rather than implying a ramp is happening. */
            tuneTelemetry.ackermannLeftRateCountsPerSec = 0;
            tuneTelemetry.ackermannRightRateCountsPerSec = 0;

            AppMotion_PublishTelemetry();

            if (AppMotion_DeadlineReached(preSteerDeadlineTick))
            {
                /* Re-anchor from FRESH encoder reads -- no target jump
                 * caused by the wait, regardless of any tiny physical
                 * settling during PRE_STEER. leftPID/rightPID/
                 * steeringPID are reset again defensively (all three
                 * are already untouched since command entry, so this
                 * is a no-op by construction, not a behavior change --
                 * included so this transition is self-evidently
                 * correct rather than relying on that invariant
                 * silently). The turn-motion timeout is armed HERE,
                 * not at command entry, so PRE_STEER's own wait never
                 * steals from it (see AppMotion_EnterTurnOrPreSteer()'s
                 * doc comment). */
                int32_t freshLeft = AppEncoder_Left();
                int32_t freshRight = AppMotion_GetRightPosition();

                targetLeftCounts = (float)freshLeft;
                targetRightCounts = (float)freshRight;

                PID_Reset(&leftPID);
                PID_Reset(&rightPID);
                PID_Reset(&steeringPID);

                motionState = APP_MOTION_TURN;
                AppMotion_SetMovementDeadline(
                    MOV_TIMEOUT_BASE_MS +
                    ((uint32_t)tuneTelemetry.commandValue *
                     MOV_TURN_TIMEOUT_MS_PER_DEGREE)
                );
            }

            break;
        }


        case APP_MOTION_TURN:
        {
            if (!AppIMU_IsReady())
            {
                AppMotion_BeginBrake(
                    APP_MOTION_EVENT_IMU_NOT_READY
                );
                break;
            }

            float leftNow =
                (float)AppEncoder_Left();

            float rightNow =
                (float)AppMotion_GetRightPosition();

            float headingNow =
                AppIMU_Heading();

            float headingError =
                targetHeadingDeg - headingNow;

            /* TURNBRAKE: single AppIMU_GyroZ() read for this whole tick,
             * reused below both for the predictive-brake condition and
             * for the existing MM,GZ telemetry field (which previously
             * called AppIMU_GyroZ() a second time, separately, later in
             * this same tick) -- one read, two consumers. */
            float currentGyroZDps =
                AppIMU_GyroZ();

            bool isReverseTurn =
                (activeCommand == MOTION_BACKWARD_LEFT) ||
                (activeCommand == MOTION_BACKWARD_RIGHT);

            /* Ackermann-v2 (AppMotion_SelectAckermannRates()): when
             * disabled, both rates are exactly the current runtime
             * TURNSPEED (no (1+-k) math at all), so this is byte-
             * identical to the equal-rate ramp this replaces. When
             * enabled, gives the inner wheel a slower target-rate
             * magnitude and the outer wheel a faster one, averaging
             * back to the same TURNSPEED base -- see that function's
             * own doc comment for the full mapping. */
            float leftRateCountsPerSec;
            float rightRateCountsPerSec;

            AppMotion_SelectAckermannRates(
                activeCommand,
                &leftRateCountsPerSec,
                &rightRateCountsPerSec
            );

            float leftRampCounts =
                leftRateCountsPerSec * dtSeconds;
            float rightRampCounts =
                rightRateCountsPerSec * dtSeconds;

            if (isReverseTurn)
            {
                targetLeftCounts -= leftRampCounts;
                targetRightCounts -= rightRampCounts;
            }
            else
            {
                targetLeftCounts += leftRampCounts;
                targetRightCounts += rightRampCounts;
            }

            float leftError =
                targetLeftCounts - leftNow;

            float rightError =
                targetRightCounts - rightNow;

            float leftOutput =
                PID_Update(
                    &leftPID,
                    targetLeftCounts,
                    leftNow,
                    dtSeconds
                );

            float rightOutput =
                PID_Update(
                    &rightPID,
                    targetRightCounts,
                    rightNow,
                    dtSeconds
                );

            /* Fixed max-safe turn servo stage: steeringPID is
             * deliberately NOT updated this tick -- the servo command
             * comes only from AppMotion_TurnServoEndpointUs() below,
             * so there is no live steeringPID output to compute here.
             * steeringPID's gains/data structure/API are untouched and
             * fully intact for possible future reuse; it is simply not
             * ticked while this experiment holds the servo fixed. */

            int32_t leftCommand =
                (int32_t)leftOutput;

            int32_t rightCommand =
                (int32_t)rightOutput;

            /* Left motor minimum useful duty */
            if ((leftError > POSITION_TOLERANCE_COUNTS) &&
                (leftCommand > 0) &&
                (leftCommand < LEFT_MOTOR_MIN_DUTY))
            {
                leftCommand = LEFT_MOTOR_MIN_DUTY;
            }
            else if ((leftError < -POSITION_TOLERANCE_COUNTS) &&
                     (leftCommand < 0) &&
                     (leftCommand > -LEFT_MOTOR_MIN_DUTY))
            {
                leftCommand = -LEFT_MOTOR_MIN_DUTY;
            }
            else if ((leftError >= -POSITION_TOLERANCE_COUNTS) &&
                     (leftError <= POSITION_TOLERANCE_COUNTS))
            {
                leftCommand = 0;
            }

            /* Right motor minimum useful duty */
            if ((rightError > POSITION_TOLERANCE_COUNTS) &&
                (rightCommand > 0) &&
                (rightCommand < RIGHT_MOTOR_MIN_DUTY))
            {
                rightCommand = RIGHT_MOTOR_MIN_DUTY;
            }
            else if ((rightError < -POSITION_TOLERANCE_COUNTS) &&
                     (rightCommand < 0) &&
                     (rightCommand > -RIGHT_MOTOR_MIN_DUTY))
            {
                rightCommand = -RIGHT_MOTOR_MIN_DUTY;
            }
            else if ((rightError >= -POSITION_TOLERANCE_COUNTS) &&
                     (rightError <= POSITION_TOLERANCE_COUNTS))
            {
                rightCommand = 0;
            }

            AppMotor_Set(leftCommand, rightCommand);


            int32_t servoPulse =
                AppMotion_TurnServoEndpointUs(activeCommand);

            AppServo_SetUs(
                (uint32_t)servoPulse
            );

            /* Unified movement telemetry (MM, line) -- telemetry-only,
             * mirrors exactly what this tick already computed above; no
             * control computation reads any of these fields.
             * currentGyroZDps is read the same way APP_MOTION_DISTANCE's
             * Stage A2 heading-rate-damping block already does (AppIMU_
             * GyroZ()) -- read-only, does not feed steeringPID here.
             * Fixed max-safe turn servo stage: servoPulse (and MM,SP)
             * is always exactly SERVO_MIN_US or SERVO_MAX_US from
             * AppMotion_TurnServoEndpointUs() -- the real commanded
             * value, not derived/estimated. steeringCorrectionUs is 0
             * -- steeringPID is not updated this tick (see above), so
             * there is no live output to report; this is a real
             * dormant-controller state, not a stale/placeholder value.
             * currentGyroZDps is the single read hoisted above, not a
             * second call. */
            {
                tuneTelemetry.leftTargetCounts =
                    (int32_t)targetLeftCounts;
                tuneTelemetry.leftPositionCounts =
                    (int32_t)leftNow;
                tuneTelemetry.leftErrorCounts =
                    (int32_t)leftError;
                tuneTelemetry.leftOutput =
                    leftCommand;

                tuneTelemetry.rightTargetCounts =
                    (int32_t)targetRightCounts;
                tuneTelemetry.rightPositionCounts =
                    (int32_t)rightNow;
                tuneTelemetry.rightErrorCounts =
                    (int32_t)rightError;
                tuneTelemetry.rightOutput =
                    rightCommand;

                tuneTelemetry.headingTargetX10 =
                    AppMotion_ToTenths(targetHeadingDeg);
                tuneTelemetry.headingCurrentX10 =
                    AppMotion_ToTenths(headingNow);
                tuneTelemetry.headingErrorX10 =
                    AppMotion_ToTenths(headingError);
                tuneTelemetry.headingYawRateX10 =
                    AppMotion_ToTenths(currentGyroZDps);
                tuneTelemetry.steeringCorrectionUs = 0;
                tuneTelemetry.servoPulseUs =
                    (uint32_t)servoPulse;

                tuneTelemetry.activeDirection = activeCommand;
                tuneTelemetry.elapsedMs =
                    xTaskGetTickCount() - movementStartTick;
                tuneTelemetry.leftDisplacementCounts =
                    (int32_t)(leftNow - movementStartLeftCounts);
                tuneTelemetry.rightDisplacementCounts =
                    (int32_t)(rightNow - movementStartRightCounts);
                tuneTelemetry.movementBraking = 0U;
                tuneTelemetry.movementRecoveryActive = 0U;
                tuneTelemetry.movementPreSteering = 0U;

                /* Ackermann-v2 (AK, line): the ACTUAL rates selected
                 * and used for this tick's ramp above -- rounded to
                 * nearest integer for telemetry (both are always
                 * >=0), not truncated; the float rates the controller
                 * itself used are untouched by this rounding. */
                tuneTelemetry.ackermannLeftRateCountsPerSec =
                    (int32_t)(leftRateCountsPerSec + 0.5f);
                tuneTelemetry.ackermannRightRateCountsPerSec =
                    (int32_t)(rightRateCountsPerSec + 0.5f);

                AppMotion_PublishTelemetry();
            }


            if ((headingError >= -TURN_HEADING_TOLERANCE_DEG) &&
                (headingError <= TURN_HEADING_TOLERANCE_DEG))
            {
                settledCycles++;
            }
            else
            {
                settledCycles = 0U;
            }


            /* TURNBRAKE: predictive-brake trigger, independent of and
             * additive to the dwell counter above -- see
             * AppMotion_SetTurnBrake()'s own doc comment (app_motion.h)
             * for the full eligibility condition, and
             * DEFAULT_TURNBRAKE_LOOKAHEAD_MS's own comment above for the
             * 135ms starting value's derivation. remainingAngleDeg/
             * predictedStoppingAngleDeg are computed unconditionally
             * (cheap; also needed for the one-time snapshot below) but
             * only ever gate anything when turnBrakeEnabled is true --
             * whenever it is runtime-disabled (enabled is the boot
             * default, see AppMotion_Init()), turnBrakeCandidate is
             * always false and turn completion falls back to the
             * heading-tolerance/5-cycle dwell path alone, unchanged
             * from before TURNBRAKE existed. */
            float remainingAngleDeg = fabsf(headingError);
            float predictedStoppingAngleDeg =
                fabsf(currentGyroZDps) *
                (turnBrakeLookaheadMs / 1000.0f);

            bool turnBrakeCandidate =
                turnBrakeEnabled &&
                ((headingError * currentGyroZDps) > 0.0f) &&
                (fabsf(currentGyroZDps) >= TURN_BRAKE_MIN_YAW_RATE_DPS) &&
                (remainingAngleDeg <= predictedStoppingAngleDeg);

            if (turnBrakeCandidate)
            {
                /* Saturate at TURN_BRAKE_CONFIRM_CYCLES -- never allowed
                 * to keep incrementing past it, so this uint8_t cannot
                 * wrap even across an arbitrarily long qualifying
                 * streak (BeginBrake() below fires and resets it well
                 * before that could matter in practice, but this is not
                 * relied upon for correctness). */
                if (turnBrakeQualifyingCycles < TURN_BRAKE_CONFIRM_CYCLES)
                {
                    turnBrakeQualifyingCycles++;
                }
            }
            else
            {
                turnBrakeQualifyingCycles = 0U;
            }

            bool turnBrakeTriggered =
                turnBrakeEnabled &&
                (turnBrakeQualifyingCycles >= TURN_BRAKE_CONFIRM_CYCLES);

            if (turnBrakeTriggered)
            {
                AppMotion_PublishTurnBrakeSnapshot(
                    activeCommand,
                    targetHeadingDeg,
                    headingNow,
                    remainingAngleDeg,
                    currentGyroZDps,
                    predictedStoppingAngleDeg,
                    turnBrakeLookaheadMs,
                    turnBrakeQualifyingCycles
                );
            }


            if ((settledCycles >= SETTLED_CYCLES_REQUIRED) ||
                turnBrakeTriggered)
            {
                AppMotion_BeginBrake(
                    APP_MOTION_EVENT_MOV_DONE
                );
            }



            else if (AppMotion_DeadlineReached(
                         movementDeadlineTick))
            {
                AppMotion_BeginBrake(
                    APP_MOTION_EVENT_MOTION_TIMEOUT
                );
            }

            break;
        }


        case APP_MOTION_TUNE_LEFT:
        {
            float leftNow =
                (float)AppEncoder_Left();

            float rightNow =
                (float)AppMotion_GetRightPosition();

            tuneElapsedSeconds += dtSeconds;

            float leftError =
                targetLeftCounts - leftNow;

            float leftOutput =
                PID_Update(
                    &leftPID,
                    targetLeftCounts,
                    leftNow,
                    dtSeconds
                );

            int32_t leftCommand =
                (int32_t)leftOutput;

            if ((leftError > POSITION_TOLERANCE_COUNTS) &&
                (leftCommand > 0) &&
                (leftCommand < LEFT_MOTOR_MIN_DUTY))
            {
                leftCommand = LEFT_MOTOR_MIN_DUTY;
            }
            else if ((leftError < -POSITION_TOLERANCE_COUNTS) &&
                     (leftCommand < 0) &&
                     (leftCommand > -LEFT_MOTOR_MIN_DUTY))
            {
                leftCommand = -LEFT_MOTOR_MIN_DUTY;
            }
            else if ((leftError >= -POSITION_TOLERANCE_COUNTS) &&
                     (leftError <= POSITION_TOLERANCE_COUNTS))
            {
                leftCommand = 0;
            }

            AppMotor_SetLeft(leftCommand);
            AppMotor_SetRight(0);
            AppServo_Center();

            tuneTelemetry.leftTargetCounts =
                (int32_t)targetLeftCounts;
            tuneTelemetry.leftPositionCounts =
                (int32_t)leftNow;
            tuneTelemetry.leftErrorCounts =
                (int32_t)leftError;
            tuneTelemetry.leftOutput = leftCommand;
            tuneTelemetry.rightPositionCounts =
                (int32_t)rightNow;
            tuneTelemetry.rightOutput = 0;
            tuneTelemetry.servoPulseUs =
                SERVO_CENTER_US;

            AppMotion_PublishTelemetry();

            if ((leftError >= -POSITION_TOLERANCE_COUNTS) &&
                (leftError <= POSITION_TOLERANCE_COUNTS))
            {
                settledCycles++;
            }
            else
            {
                settledCycles = 0U;
            }

            if (settledCycles >= SETTLED_CYCLES_REQUIRED)
            {
                AppMotion_FinishTune(
                    APP_TUNE_RESULT_DONE
                );
            }
            else if (tuneElapsedSeconds >=
                     TUNE_MOTOR_TIMEOUT_SECONDS)
            {
                AppMotion_FinishTune(
                    APP_TUNE_RESULT_TIMEOUT
                );
            }

            break;
        }


        case APP_MOTION_TUNE_RIGHT:
        {
            float leftNow =
                (float)AppEncoder_Left();

            float rightNow =
                (float)AppMotion_GetRightPosition();

            tuneElapsedSeconds += dtSeconds;

            float rightError =
                targetRightCounts - rightNow;

            float rightOutput =
                PID_Update(
                    &rightPID,
                    targetRightCounts,
                    rightNow,
                    dtSeconds
                );

            int32_t rightCommand =
                (int32_t)rightOutput;

            if ((rightError > POSITION_TOLERANCE_COUNTS) &&
                (rightCommand > 0) &&
                (rightCommand < RIGHT_MOTOR_MIN_DUTY))
            {
                rightCommand = RIGHT_MOTOR_MIN_DUTY;
            }
            else if ((rightError < -POSITION_TOLERANCE_COUNTS) &&
                     (rightCommand < 0) &&
                     (rightCommand > -RIGHT_MOTOR_MIN_DUTY))
            {
                rightCommand = -RIGHT_MOTOR_MIN_DUTY;
            }
            else if ((rightError >= -POSITION_TOLERANCE_COUNTS) &&
                     (rightError <= POSITION_TOLERANCE_COUNTS))
            {
                rightCommand = 0;
            }

            AppMotor_SetLeft(0);
            AppMotor_SetRight(rightCommand);
            AppServo_Center();

            tuneTelemetry.leftPositionCounts =
                (int32_t)leftNow;
            tuneTelemetry.leftOutput = 0;
            tuneTelemetry.rightTargetCounts =
                (int32_t)targetRightCounts;
            tuneTelemetry.rightPositionCounts =
                (int32_t)rightNow;
            tuneTelemetry.rightErrorCounts =
                (int32_t)rightError;
            tuneTelemetry.rightOutput = rightCommand;
            tuneTelemetry.servoPulseUs =
                SERVO_CENTER_US;

            AppMotion_PublishTelemetry();

            if ((rightError >= -POSITION_TOLERANCE_COUNTS) &&
                (rightError <= POSITION_TOLERANCE_COUNTS))
            {
                settledCycles++;
            }
            else
            {
                settledCycles = 0U;
            }

            if (settledCycles >= SETTLED_CYCLES_REQUIRED)
            {
                AppMotion_FinishTune(
                    APP_TUNE_RESULT_DONE
                );
            }
            else if (tuneElapsedSeconds >=
                     TUNE_MOTOR_TIMEOUT_SECONDS)
            {
                AppMotion_FinishTune(
                    APP_TUNE_RESULT_TIMEOUT
                );
            }

            break;
        }


        case APP_MOTION_TUNE_BOTH:
        {
            float leftNow =
                (float)AppEncoder_Left();

            float rightNow =
                (float)AppMotion_GetRightPosition();

            tuneElapsedSeconds += dtSeconds;

            float leftError =
                targetLeftCounts - leftNow;

            float rightError =
                targetRightCounts - rightNow;

            float leftOutput =
                PID_Update(
                    &leftPID,
                    targetLeftCounts,
                    leftNow,
                    dtSeconds
                );

            float rightOutput =
                PID_Update(
                    &rightPID,
                    targetRightCounts,
                    rightNow,
                    dtSeconds
                );

            int32_t leftCommand =
                (int32_t)leftOutput;

            int32_t rightCommand =
                (int32_t)rightOutput;

            /* Left motor minimum useful duty */
            if ((leftError > POSITION_TOLERANCE_COUNTS) &&
                (leftCommand > 0) &&
                (leftCommand < LEFT_MOTOR_MIN_DUTY))
            {
                leftCommand = LEFT_MOTOR_MIN_DUTY;
            }
            else if ((leftError < -POSITION_TOLERANCE_COUNTS) &&
                     (leftCommand < 0) &&
                     (leftCommand > -LEFT_MOTOR_MIN_DUTY))
            {
                leftCommand = -LEFT_MOTOR_MIN_DUTY;
            }
            else if ((leftError >= -POSITION_TOLERANCE_COUNTS) &&
                     (leftError <= POSITION_TOLERANCE_COUNTS))
            {
                leftCommand = 0;
            }

            /* Right motor minimum useful duty */
            if ((rightError > POSITION_TOLERANCE_COUNTS) &&
                (rightCommand > 0) &&
                (rightCommand < RIGHT_MOTOR_MIN_DUTY))
            {
                rightCommand = RIGHT_MOTOR_MIN_DUTY;
            }
            else if ((rightError < -POSITION_TOLERANCE_COUNTS) &&
                     (rightCommand < 0) &&
                     (rightCommand > -RIGHT_MOTOR_MIN_DUTY))
            {
                rightCommand = -RIGHT_MOTOR_MIN_DUTY;
            }
            else if ((rightError >= -POSITION_TOLERANCE_COUNTS) &&
                     (rightError <= POSITION_TOLERANCE_COUNTS))
            {
                rightCommand = 0;
            }


            AppMotor_Set(leftCommand, rightCommand);
            AppServo_Center();

            tuneTelemetry.leftTargetCounts =
                (int32_t)targetLeftCounts;
            tuneTelemetry.leftPositionCounts =
                (int32_t)leftNow;
            tuneTelemetry.leftErrorCounts =
                (int32_t)leftError;
            tuneTelemetry.leftOutput = leftCommand;
            tuneTelemetry.rightTargetCounts =
                (int32_t)targetRightCounts;
            tuneTelemetry.rightPositionCounts =
                (int32_t)rightNow;
            tuneTelemetry.rightErrorCounts =
                (int32_t)rightError;
            tuneTelemetry.rightOutput = rightCommand;
            tuneTelemetry.servoPulseUs =
                SERVO_CENTER_US;

            AppMotion_PublishTelemetry();

            if ((leftError >= -POSITION_TOLERANCE_COUNTS) &&
                (leftError <= POSITION_TOLERANCE_COUNTS) &&
                (rightError >= -POSITION_TOLERANCE_COUNTS) &&
                (rightError <= POSITION_TOLERANCE_COUNTS))
            {
                settledCycles++;
            }
            else
            {
                settledCycles = 0U;
            }

            if (settledCycles >= SETTLED_CYCLES_REQUIRED)
            {
                AppMotion_FinishTune(
                    APP_TUNE_RESULT_DONE
                );
            }
            else if (tuneElapsedSeconds >=
                     TUNE_MOTOR_TIMEOUT_SECONDS)
            {
                AppMotion_FinishTune(
                    APP_TUNE_RESULT_TIMEOUT
                );
            }

            break;
        }


        case APP_MOTION_TUNE_STEERING:
        {
            if (!AppIMU_IsReady())
            {
                AppMotion_StopWithEvent(
                    APP_MOTION_EVENT_IMU_NOT_READY
                );
                break;
            }

            float leftNow =
                (float)AppEncoder_Left();

            float rightNow =
                (float)AppMotion_GetRightPosition();

            float headingNow =
                AppIMU_Heading();

            tuneElapsedSeconds += dtSeconds;

            targetLeftCounts +=
                tuneSteeringTargetRate * dtSeconds;

            targetRightCounts +=
                tuneSteeringTargetRate * dtSeconds;

            float leftError =
                targetLeftCounts - leftNow;

            float rightError =
                targetRightCounts - rightNow;

            float headingError =
                targetHeadingDeg - headingNow;

            float leftOutput =
                PID_Update(
                    &leftPID,
                    targetLeftCounts,
                    leftNow,
                    dtSeconds
                );

            float rightOutput =
                PID_Update(
                    &rightPID,
                    targetRightCounts,
                    rightNow,
                    dtSeconds
                );

            float steeringOutput =
                PID_Update(
                    &steeringPID,
                    targetHeadingDeg,
                    headingNow,
                    dtSeconds
                );

            int32_t leftCommand =
                (int32_t)leftOutput;

            int32_t rightCommand =
                (int32_t)rightOutput;

            /* Left motor minimum useful duty */
            if ((leftError > POSITION_TOLERANCE_COUNTS) &&
                (leftCommand > 0) &&
                (leftCommand < LEFT_MOTOR_MIN_DUTY))
            {
                leftCommand = LEFT_MOTOR_MIN_DUTY;
            }
            else if ((leftError < -POSITION_TOLERANCE_COUNTS) &&
                     (leftCommand < 0) &&
                     (leftCommand > -LEFT_MOTOR_MIN_DUTY))
            {
                leftCommand = -LEFT_MOTOR_MIN_DUTY;
            }
            else if ((leftError >= -POSITION_TOLERANCE_COUNTS) &&
                     (leftError <= POSITION_TOLERANCE_COUNTS))
            {
                leftCommand = 0;
            }

            /* Right motor minimum useful duty */
            if ((rightError > POSITION_TOLERANCE_COUNTS) &&
                (rightCommand > 0) &&
                (rightCommand < RIGHT_MOTOR_MIN_DUTY))
            {
                rightCommand = RIGHT_MOTOR_MIN_DUTY;
            }
            else if ((rightError < -POSITION_TOLERANCE_COUNTS) &&
                     (rightCommand < 0) &&
                     (rightCommand > -RIGHT_MOTOR_MIN_DUTY))
            {
                rightCommand = -RIGHT_MOTOR_MIN_DUTY;
            }
            else if ((rightError >= -POSITION_TOLERANCE_COUNTS) &&
                     (rightError <= POSITION_TOLERANCE_COUNTS))
            {
                rightCommand = 0;
            }

            int32_t steeringCorrection =
                (int32_t)steeringOutput;

            int32_t servoPulse;

            if (tuneSteeringTargetRate < 0.0f)
            {
                /* Backward */
                servoPulse =
                    (int32_t)SERVO_CENTER_US
                    + steeringCorrection;
            }
            else
            {
                /* Forward */
                servoPulse =
                    (int32_t)SERVO_CENTER_US
                    - steeringCorrection;
            }

            AppMotor_Set(leftCommand, rightCommand);
            AppServo_SetUs((uint32_t)servoPulse);

            tuneTelemetry.leftTargetCounts =
                (int32_t)targetLeftCounts;
            tuneTelemetry.leftPositionCounts =
                (int32_t)leftNow;
            tuneTelemetry.leftErrorCounts =
                (int32_t)leftError;
            tuneTelemetry.leftOutput = leftCommand;
            tuneTelemetry.rightTargetCounts =
                (int32_t)targetRightCounts;
            tuneTelemetry.rightPositionCounts =
                (int32_t)rightNow;
            tuneTelemetry.rightErrorCounts =
                (int32_t)rightError;
            tuneTelemetry.rightOutput = rightCommand;
            tuneTelemetry.headingTargetX10 =
                AppMotion_ToTenths(targetHeadingDeg);
            tuneTelemetry.headingCurrentX10 =
                AppMotion_ToTenths(headingNow);
            tuneTelemetry.headingErrorX10 =
                AppMotion_ToTenths(headingError);
            tuneTelemetry.steeringCorrectionUs =
                steeringCorrection;
            tuneTelemetry.servoPulseUs =
                (uint32_t)servoPulse;

            AppMotion_PublishTelemetry();

            if (tuneElapsedSeconds >=
                tuneDurationSeconds)
            {
                AppMotion_FinishTune(
                    APP_TUNE_RESULT_DONE
                );
            }

            break;
        }

        case APP_MOTION_DIAGNOSTIC_PWM:
        case APP_MOTION_DIAGNOSTIC_SPEED:
        {
            int32_t leftNow = AppEncoder_Left();
            int32_t rightNow =
                AppMotion_GetRightPosition();
            int32_t leftDelta =
                leftNow - diagnosticStartLeftCounts;
            int32_t rightDelta =
                rightNow - diagnosticStartRightCounts;

            diagnosticElapsedSeconds += dtSeconds;

            AppMotor_Set(diagnosticDuty, diagnosticDuty);
            AppServo_Center();

            if (diagnosticElapsedSeconds >=
                diagnosticDurationSeconds)
            {
                int32_t completedDuty = diagnosticDuty;

                if (motionState == APP_MOTION_DIAGNOSTIC_PWM)
                {
                    AppMotion_FinishDiagnostic(
                        APP_MOTION_EVENT_PWM_DONE,
                        completedDuty,
                        leftDelta,
                        rightDelta,
                        0
                    );
                }
                else
                {
                    float leftCmPerSecond =
                        (float)leftDelta /
                        ((float)ENCODER_COUNTS_PER_CM *
                         diagnosticElapsedSeconds);
                    float rightCmPerSecond =
                        (float)rightDelta /
                        ((float)ENCODER_COUNTS_PER_CM *
                         diagnosticElapsedSeconds);

                    AppMotion_FinishDiagnostic(
                        APP_MOTION_EVENT_SPEED_DONE,
                        completedDuty,
                        AppMotion_ToTenths(leftCmPerSecond),
                        AppMotion_ToTenths(rightCmPerSecond),
                        0
                    );
                }
            }

            break;
        }

        case APP_MOTION_DIAGNOSTIC_HEADING_RAW:
        {
            if (!AppIMU_IsReady())
            {
                AppMotion_StopWithEvent(
                    APP_MOTION_EVENT_IMU_NOT_READY
                );
                break;
            }

            float headingNow = AppIMU_Heading();

            diagnosticElapsedSeconds += dtSeconds;

            AppMotor_Set(diagnosticDuty, diagnosticDuty);
            AppServo_Center();

            if (diagnosticElapsedSeconds >=
                diagnosticDurationSeconds)
            {
                AppMotion_FinishDiagnostic(
                    APP_MOTION_EVENT_HEADING_RAW_DONE,
                    0,
                    0,
                    0,
                    AppMotion_ToTenths(
                        headingNow - diagnosticStartHeadingDeg
                    )
                );
            }

            break;
        }

        case APP_MOTION_DIAGNOSTIC_HEADING:
        {
            if (!AppIMU_IsReady())
            {
                AppMotion_StopWithEvent(
                    APP_MOTION_EVENT_IMU_NOT_READY
                );
                break;
            }

            float headingNow = AppIMU_Heading();
            float steeringOutput =
                PID_Update(
                    &steeringPID,
                    targetHeadingDeg,
                    headingNow,
                    dtSeconds
                );
            int32_t servoPulse =
                (int32_t)SERVO_CENTER_US -
                (int32_t)steeringOutput;

            diagnosticElapsedSeconds += dtSeconds;

            AppMotor_Set(diagnosticDuty, diagnosticDuty);
            AppServo_SetUs((uint32_t)servoPulse);

            if (diagnosticElapsedSeconds >=
                diagnosticDurationSeconds)
            {
                AppMotion_FinishDiagnostic(
                    APP_MOTION_EVENT_HEADING_DONE,
                    0,
                    0,
                    0,
                    AppMotion_ToTenths(
                        headingNow - targetHeadingDeg
                    )
                );
            }

            break;
        }

        case APP_MOTION_WAIT_IMU:
        {
            AppIMUEvent imuEvent;
            AppMotionEventType resultType;

            AppIMU_GetEvent(&imuEvent);

            if (imuEvent.sequence == awaitedImuEventSequence)
            {
                imuWaitElapsedSeconds += dtSeconds;

                if (imuWaitElapsedSeconds >=
                    IMU_REQUEST_TIMEOUT_SECONDS)
                {
                    MotionType timedOutCommand = activeCommand;

                    AppMotion_StopAndReset();
                    AppMotion_PublishEvent(
                        (timedOutCommand == MOTION_IMU_ZERO) ?
                            APP_MOTION_EVENT_IMU_ZERO_ERROR :
                            APP_MOTION_EVENT_IMU_FLIP_ERROR,
                        timedOutCommand,
                        0,
                        0,
                        0,
                        0
                    );
                }

                break;
            }

            awaitedImuEventSequence = imuEvent.sequence;

            if (activeCommand == MOTION_IMU_ZERO)
            {
                resultType =
                    (imuEvent.type == APP_IMU_EVENT_ZERO_DONE) ?
                        APP_MOTION_EVENT_IMU_ZERO_OK :
                        APP_MOTION_EVENT_IMU_ZERO_ERROR;
            }
            else
            {
                resultType =
                    (imuEvent.type == APP_IMU_EVENT_FLIP_DONE) ?
                        APP_MOTION_EVENT_IMU_FLIP_OK :
                        APP_MOTION_EVENT_IMU_FLIP_ERROR;
            }

            AppMotion_FinishImuRequest(resultType);
            break;
        }

        case APP_MOTION_BRAKING:
            if (AppMotion_DeadlineReached(brakeDeadlineTick))
            {
                AppMotion_FinishBrake();
            }
            break;

        default:
        {
            MotionType rejectedCommand = activeCommand;

            AppMotion_CancelTuneTelemetry();
            AppMotion_StopAndReset();
            AppMotion_PublishEvent(
                APP_MOTION_EVENT_BAD_COMMAND,
                rejectedCommand,
                0,
                0,
                0,
                0
            );
            break;
        }
    }
}

AppMotionState AppMotion_GetState(void)
{
    AppMotionState state;

    taskENTER_CRITICAL();
    state = motionState;
    taskEXIT_CRITICAL();

    return state;
}

bool AppMotion_IsBusy(void)
{
    return AppMotion_GetState() != APP_MOTION_IDLE;
}

void AppMotion_SetPIDGains(
    AppPIDTarget target,
    float kp,
    float ki,
    float kd
)
{
    PIDController *pid;

    switch (target)
    {
        case APP_PID_LEFT:
            pid = &leftPID;
            break;

        case APP_PID_RIGHT:
            pid = &rightPID;
            break;

        case APP_PID_STEERING:
            pid = &steeringPID;
            break;

        default:
            return;
    }

    taskENTER_CRITICAL();
    PID_SetGains(pid, kp, ki, kd);
    if (!applyingProfile && (target == APP_PID_STEERING))
    {
        currentProfile = APP_MOTION_PROFILE_CUSTOM;
    }
    taskEXIT_CRITICAL();
}


void AppMotion_GetPIDGains(
    AppPIDTarget target,
    float *kp,
    float *ki,
    float *kd
)
{
    if ((kp == NULL) ||
        (ki == NULL) ||
        (kd == NULL))
    {
        return;
    }

    switch (target)
    {
        case APP_PID_LEFT:
            *kp = leftPID.kp;
            *ki = leftPID.ki;
            *kd = leftPID.kd;
            break;

        case APP_PID_RIGHT:
            *kp = rightPID.kp;
            *ki = rightPID.ki;
            *kd = rightPID.kd;
            break;

        case APP_PID_STEERING:
            *kp = steeringPID.kp;
            *ki = steeringPID.ki;
            *kd = steeringPID.kd;
            break;

        default:
            *kp = 0.0f;
            *ki = 0.0f;
            *kd = 0.0f;
            break;
    }
}

void AppMotion_SetWheelSync(
    float gain,
    float maxDuty,
    bool enable
)
{
    taskENTER_CRITICAL();
    wheelSyncGain = gain;
    wheelSyncMaxDuty = maxDuty;
    wheelSyncEnabled = enable;
    if (!applyingProfile)
    {
        currentProfile = APP_MOTION_PROFILE_CUSTOM;
    }
    taskEXIT_CRITICAL();
}

void AppMotion_GetWheelSync(
    float *gain,
    float *maxDuty,
    bool *enable
)
{
    if ((gain == NULL) ||
        (maxDuty == NULL) ||
        (enable == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    *gain = wheelSyncGain;
    *maxDuty = wheelSyncMaxDuty;
    *enable = wheelSyncEnabled;
    taskEXIT_CRITICAL();
}

void AppMotion_SetHeadingRateDamping(
    float krate,
    float maxCorrection,
    bool enable
)
{
    taskENTER_CRITICAL();
    headingCtrlKrate = krate;
    headingCtrlMaxCorrection = maxCorrection;
    headingCtrlEnabled = enable;
    if (!applyingProfile)
    {
        currentProfile = APP_MOTION_PROFILE_CUSTOM;
    }
    taskEXIT_CRITICAL();
}

void AppMotion_GetHeadingRateDamping(
    float *krate,
    float *maxCorrection,
    bool *enable
)
{
    if ((krate == NULL) ||
        (maxCorrection == NULL) ||
        (enable == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    *krate = headingCtrlKrate;
    *maxCorrection = headingCtrlMaxCorrection;
    *enable = headingCtrlEnabled;
    taskEXIT_CRITICAL();
}

void AppMotion_SetReverseTorqueGuard(
    bool enable
)
{
    taskENTER_CRITICAL();
    rtGuardEnabled = enable;
    if (!applyingProfile)
    {
        currentProfile = APP_MOTION_PROFILE_CUSTOM;
    }
    taskEXIT_CRITICAL();
}

void AppMotion_GetReverseTorqueGuard(
    bool *enable
)
{
    if (enable == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *enable = rtGuardEnabled;
    taskEXIT_CRITICAL();
}

void AppMotion_SetDecelProfile(
    float decelCounts,
    float approachDuty,
    bool enable
)
{
    taskENTER_CRITICAL();
    decelStartCounts = decelCounts;
    decelApproachDuty = approachDuty;
    decelProfileEnabled = enable;
    if (!applyingProfile)
    {
        currentProfile = APP_MOTION_PROFILE_CUSTOM;
    }
    taskEXIT_CRITICAL();
}

void AppMotion_GetDecelProfile(
    float *decelCounts,
    float *approachDuty,
    bool *enable
)
{
    if ((decelCounts == NULL) ||
        (approachDuty == NULL) ||
        (enable == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    *decelCounts = decelStartCounts;
    *approachDuty = decelApproachDuty;
    *enable = decelProfileEnabled;
    taskEXIT_CRITICAL();
}

void AppMotion_SetSpeedControl(
    float targetSpeed,
    float kp,
    float ki,
    bool enable
)
{
    taskENTER_CRITICAL();
    spctrlTargetSpeed = targetSpeed;
    speedPID.kp = kp;
    speedPID.ki = ki;
    AppMotion_UpdateSpeedIntegralLimits();
    spctrlEnabled = enable;
    if (!applyingProfile)
    {
        currentProfile = APP_MOTION_PROFILE_CUSTOM;
    }
    taskEXIT_CRITICAL();
}

void AppMotion_GetSpeedControl(
    float *targetSpeed,
    float *kp,
    float *ki,
    bool *enable
)
{
    if ((targetSpeed == NULL) ||
        (kp == NULL) ||
        (ki == NULL) ||
        (enable == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    *targetSpeed = spctrlTargetSpeed;
    *kp = speedPID.kp;
    *ki = speedPID.ki;
    *enable = spctrlEnabled;
    taskEXIT_CRITICAL();
}

void AppMotion_SetSpeedProfile(
    float decelStartCounts,
    float approachSpeed,
    bool enable
)
{
    taskENTER_CRITICAL();
    sprofDecelStartCounts = decelStartCounts;
    sprofApproachSpeed = approachSpeed;
    sprofEnabled = enable;
    if (!applyingProfile)
    {
        currentProfile = APP_MOTION_PROFILE_CUSTOM;
    }
    taskEXIT_CRITICAL();
}

void AppMotion_GetSpeedProfile(
    float *decelStartCounts,
    float *approachSpeed,
    bool *enable
)
{
    if ((decelStartCounts == NULL) ||
        (approachSpeed == NULL) ||
        (enable == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    *decelStartCounts = sprofDecelStartCounts;
    *approachSpeed = sprofApproachSpeed;
    *enable = sprofEnabled;
    taskEXIT_CRITICAL();
}

void AppMotion_SetStopControl(
    float brakeLeadCounts,
    float brakeEntrySpeed,
    bool enable
)
{
    taskENTER_CRITICAL();
    stopctrlBrakeLeadCounts = brakeLeadCounts;
    stopctrlBrakeEntrySpeed = brakeEntrySpeed;
    stopctrlEnabled = enable;
    if (!applyingProfile)
    {
        currentProfile = APP_MOTION_PROFILE_CUSTOM;
    }
    taskEXIT_CRITICAL();
}

void AppMotion_GetStopControl(
    float *brakeLeadCounts,
    float *brakeEntrySpeed,
    bool *enable
)
{
    if ((brakeLeadCounts == NULL) ||
        (brakeEntrySpeed == NULL) ||
        (enable == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    *brakeLeadCounts = stopctrlBrakeLeadCounts;
    *brakeEntrySpeed = stopctrlBrakeEntrySpeed;
    *enable = stopctrlEnabled;
    taskEXIT_CRITICAL();
}

void AppMotion_SetDistanceCal(
    float slopeCountsPerCm,
    float offsetCounts,
    bool enable
)
{
    taskENTER_CRITICAL();
    movcalSlopeCountsPerCm = slopeCountsPerCm;
    movcalOffsetCounts = offsetCounts;
    movcalEnabled = enable;
    if (!applyingProfile)
    {
        currentProfile = APP_MOTION_PROFILE_CUSTOM;
    }
    taskEXIT_CRITICAL();
}

void AppMotion_GetDistanceCal(
    float *slopeCountsPerCm,
    float *offsetCounts,
    bool *enable
)
{
    if ((slopeCountsPerCm == NULL) ||
        (offsetCounts == NULL) ||
        (enable == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    *slopeCountsPerCm = movcalSlopeCountsPerCm;
    *offsetCounts = movcalOffsetCounts;
    *enable = movcalEnabled;
    taskEXIT_CRITICAL();
}

/* Stage SFF1/SFF2: speed-aware servo feed-forward, independent per
 * direction. direction must be MOTION_FORWARD or MOTION_BACKWARD
 * (anything else is a no-op, mirroring AppMotion_SetStraightServoTrim()'s
 * own established convention from the earlier, since-reverted Stage
 * A5.3a work) -- the caller (app_comm.c) is responsible for only ever
 * passing one of the two, resolved from the wire protocol's 'F'/'B'
 * token.
 *
 * Stage SFF2: now profile-owned -- marks the active profile CUSTOM via
 * the same "if (!applyingProfile)" check every other profile-tracked
 * Set* function above already uses, once per direction actually written
 * (an invalid direction still touches nothing, including
 * currentProfile). AppMotion_ApplyProfile() below calls this function
 * for both directions while applyingProfile is held true, so its own
 * internal apply never marks CUSTOM -- only a direct SFF SET from
 * app_comm.c (where applyingProfile is always false) does. */
void AppMotion_SetServoFeedForward(
    MotionType direction,
    float startSpeed,
    float fullSpeed,
    float maxUs,
    bool enable
)
{
    taskENTER_CRITICAL();

    if (direction == MOTION_FORWARD)
    {
        sffForwardStartSpeed = startSpeed;
        sffForwardFullSpeed = fullSpeed;
        sffForwardMaxUs = maxUs;
        sffForwardEnabled = enable;

        if (!applyingProfile)
        {
            currentProfile = APP_MOTION_PROFILE_CUSTOM;
        }
    }
    else if (direction == MOTION_BACKWARD)
    {
        sffBackwardStartSpeed = startSpeed;
        sffBackwardFullSpeed = fullSpeed;
        sffBackwardMaxUs = maxUs;
        sffBackwardEnabled = enable;

        if (!applyingProfile)
        {
            currentProfile = APP_MOTION_PROFILE_CUSTOM;
        }
    }

    taskEXIT_CRITICAL();
}

void AppMotion_GetServoFeedForward(
    MotionType direction,
    float *startSpeed,
    float *fullSpeed,
    float *maxUs,
    bool *enable
)
{
    if ((startSpeed == NULL) ||
        (fullSpeed == NULL) ||
        (maxUs == NULL) ||
        (enable == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();

    if (direction == MOTION_FORWARD)
    {
        *startSpeed = sffForwardStartSpeed;
        *fullSpeed = sffForwardFullSpeed;
        *maxUs = sffForwardMaxUs;
        *enable = sffForwardEnabled;
    }
    else if (direction == MOTION_BACKWARD)
    {
        *startSpeed = sffBackwardStartSpeed;
        *fullSpeed = sffBackwardFullSpeed;
        *maxUs = sffBackwardMaxUs;
        *enable = sffBackwardEnabled;
    }
    else
    {
        *startSpeed = 0.0f;
        *fullSpeed = 0.0f;
        *maxUs = 0.0f;
        *enable = false;
    }

    taskEXIT_CRITICAL();
}

/* PRE_STEER: RAM-only, deliberately NOT profile-owned -- see this
 * function's own doc comment in app_motion.h for the full reasoning.
 * Unlike every AppMotion_Set* function above, this one does NOT mark
 * currentProfile CUSTOM. */
void AppMotion_SetPreSteer(
    float durationMs,
    bool enable
)
{
    taskENTER_CRITICAL();
    preSteerDurationMs = durationMs;
    preSteerEnabled = enable;
    taskEXIT_CRITICAL();
}

void AppMotion_GetPreSteer(
    float *durationMs,
    bool *enable
)
{
    if ((durationMs == NULL) || (enable == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    *durationMs = preSteerDurationMs;
    *enable = preSteerEnabled;
    taskEXIT_CRITICAL();
}

/* TURNSPEED: RAM-only, deliberately NOT profile-owned -- see this
 * function's own doc comment in app_motion.h. Does NOT mark
 * currentProfile CUSTOM.
 *
 * Defensively clamps countsPerSec to
 * [TURN_SPEED_COUNTS_PER_SEC_MIN,TURN_SPEED_COUNTS_PER_SEC_MAX] itself
 * (not just relying on the parser's own range check in vision_rx.c),
 * so a future internal caller of this API cannot bypass that safety
 * range. */
void AppMotion_SetTurnSpeed(
    uint32_t countsPerSec
)
{
    uint32_t clamped = countsPerSec;

    if (clamped < TURN_SPEED_COUNTS_PER_SEC_MIN)
    {
        clamped = TURN_SPEED_COUNTS_PER_SEC_MIN;
    }
    else if (clamped > TURN_SPEED_COUNTS_PER_SEC_MAX)
    {
        clamped = TURN_SPEED_COUNTS_PER_SEC_MAX;
    }

    taskENTER_CRITICAL();
    turnSpeedCountsPerSec = clamped;
    taskEXIT_CRITICAL();
}

void AppMotion_GetTurnSpeed(
    uint32_t *countsPerSec
)
{
    if (countsPerSec == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *countsPerSec = turnSpeedCountsPerSec;
    taskEXIT_CRITICAL();
}

/* Ackermann-v2: RAM-only, deliberately NOT profile-owned -- see this
 * function's own doc comment in app_motion.h. Does NOT mark
 * currentProfile CUSTOM.
 *
 * Defensively clamps k to [ACKERMANN_K_MIN,ACKERMANN_K_MAX] itself
 * (not just relying on the parser's own range check in vision_rx.c),
 * so a future internal caller cannot bypass that safety range; enable
 * is stored exactly as given, unaffected by the clamp. */
void AppMotion_SetAckermann(
    float k,
    bool enable
)
{
    float clampedK = k;

    if (clampedK < ACKERMANN_K_MIN)
    {
        clampedK = ACKERMANN_K_MIN;
    }
    else if (clampedK > ACKERMANN_K_MAX)
    {
        clampedK = ACKERMANN_K_MAX;
    }

    taskENTER_CRITICAL();
    ackermannK = clampedK;
    ackermannEnabled = enable;
    taskEXIT_CRITICAL();
}

void AppMotion_GetAckermann(
    float *k,
    bool *enable
)
{
    if ((k == NULL) || (enable == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    *k = ackermannK;
    *enable = ackermannEnabled;
    taskEXIT_CRITICAL();
}

void AppMotion_SetTurnBrake(
    float lookaheadMs,
    bool enable
)
{
    float clampedLookaheadMs = lookaheadMs;

    if (clampedLookaheadMs < TURNBRAKE_LOOKAHEAD_MS_MIN)
    {
        clampedLookaheadMs = TURNBRAKE_LOOKAHEAD_MS_MIN;
    }
    else if (clampedLookaheadMs > TURNBRAKE_LOOKAHEAD_MS_MAX)
    {
        clampedLookaheadMs = TURNBRAKE_LOOKAHEAD_MS_MAX;
    }

    taskENTER_CRITICAL();
    turnBrakeLookaheadMs = clampedLookaheadMs;
    turnBrakeEnabled = enable;
    taskEXIT_CRITICAL();
}

void AppMotion_GetTurnBrake(
    float *lookaheadMs,
    bool *enable
)
{
    if ((lookaheadMs == NULL) || (enable == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    *lookaheadMs = turnBrakeLookaheadMs;
    *enable = turnBrakeEnabled;
    taskEXIT_CRITICAL();
}

/* Named runtime configuration profile apply (motion-profiles branch;
 * Stage SFF2 folded speed-aware servo feed-forward into the bundle).
 * Reuses the exact same Set* functions above -- no control logic is
 * duplicated here, and none of those controllers' own algorithms are
 * touched. The entire sequence runs under one taskENTER_CRITICAL()/
 * taskEXIT_CRITICAL() pair for true atomicity (FreeRTOS critical
 * sections nest safely, so each inner Set* call's own critical section
 * is a cheap no-op re-entry, not a second lock) -- a concurrent
 * AppMotion_GetProfile()/AppMotion_Get*() call can never observe a
 * partially-applied profile. applyingProfile suppresses the CUSTOM-
 * marking check each Set* function otherwise performs, so applying a
 * named profile does not immediately mark itself CUSTOM. Finally resets
 * speedPID's and steeringPID's integral/previousError via the existing,
 * unmodified PID_Reset() -- runtime state only, the PID algorithm itself
 * is untouched; leftPID/rightPID are not part of any profile and are
 * left alone. */
void AppMotion_ApplyProfile(
    AppMotionProfile profile
)
{
    AppMotionProfile appliedProfile =
        (profile == APP_MOTION_PROFILE_STANDARD) ?
            APP_MOTION_PROFILE_STANDARD : APP_MOTION_PROFILE_PRECISION;

    taskENTER_CRITICAL();

    applyingProfile = true;

    if (appliedProfile == APP_MOTION_PROFILE_STANDARD)
    {
        AppMotion_SetPIDGains(
            APP_PID_STEERING,
            PROFILE_STANDARD_STEERING_KP,
            PROFILE_STANDARD_STEERING_KI,
            PROFILE_STANDARD_STEERING_KD
        );
        AppMotion_SetWheelSync(
            PROFILE_STANDARD_WSYNC_GAIN,
            PROFILE_STANDARD_WSYNC_MAX_DUTY,
            PROFILE_STANDARD_WSYNC_ENABLED
        );
        AppMotion_SetHeadingRateDamping(
            PROFILE_STANDARD_HCTRL_KRATE,
            PROFILE_STANDARD_HCTRL_MAX_CORRECTION,
            PROFILE_STANDARD_HCTRL_ENABLED
        );
        AppMotion_SetReverseTorqueGuard(
            PROFILE_STANDARD_RTGUARD_ENABLED
        );
        AppMotion_SetDecelProfile(
            PROFILE_STANDARD_DPROF_DECEL_START_COUNTS,
            PROFILE_STANDARD_DPROF_APPROACH_DUTY,
            PROFILE_STANDARD_DPROF_ENABLED
        );
        AppMotion_SetSpeedControl(
            PROFILE_STANDARD_SPCTRL_TARGET_SPEED,
            PROFILE_STANDARD_SPCTRL_KP,
            PROFILE_STANDARD_SPCTRL_KI,
            PROFILE_STANDARD_SPCTRL_ENABLED
        );
        AppMotion_SetSpeedProfile(
            PROFILE_STANDARD_SPPROF_DECEL_START_COUNTS,
            PROFILE_STANDARD_SPPROF_APPROACH_SPEED,
            PROFILE_STANDARD_SPPROF_ENABLED
        );
        AppMotion_SetStopControl(
            PROFILE_STANDARD_STOPCTRL_BRAKE_LEAD_COUNTS,
            PROFILE_STANDARD_STOPCTRL_BRAKE_ENTRY_SPEED,
            PROFILE_STANDARD_STOPCTRL_ENABLED
        );
        AppMotion_SetDistanceCal(
            PROFILE_STANDARD_MOVCAL_SLOPE_COUNTS_PER_CM,
            PROFILE_STANDARD_MOVCAL_OFFSET_COUNTS,
            PROFILE_STANDARD_MOVCAL_ENABLED
        );
        AppMotion_SetServoFeedForward(
            MOTION_FORWARD,
            PROFILE_STANDARD_SFF_FORWARD_START_SPEED,
            PROFILE_STANDARD_SFF_FORWARD_FULL_SPEED,
            PROFILE_STANDARD_SFF_FORWARD_MAX_US,
            PROFILE_STANDARD_SFF_FORWARD_ENABLED
        );
        AppMotion_SetServoFeedForward(
            MOTION_BACKWARD,
            PROFILE_STANDARD_SFF_BACKWARD_START_SPEED,
            PROFILE_STANDARD_SFF_BACKWARD_FULL_SPEED,
            PROFILE_STANDARD_SFF_BACKWARD_MAX_US,
            PROFILE_STANDARD_SFF_BACKWARD_ENABLED
        );
    }
    else
    {
        AppMotion_SetPIDGains(
            APP_PID_STEERING,
            PROFILE_PRECISION_STEERING_KP,
            PROFILE_PRECISION_STEERING_KI,
            PROFILE_PRECISION_STEERING_KD
        );
        AppMotion_SetWheelSync(
            PROFILE_PRECISION_WSYNC_GAIN,
            PROFILE_PRECISION_WSYNC_MAX_DUTY,
            PROFILE_PRECISION_WSYNC_ENABLED
        );
        AppMotion_SetHeadingRateDamping(
            PROFILE_PRECISION_HCTRL_KRATE,
            PROFILE_PRECISION_HCTRL_MAX_CORRECTION,
            PROFILE_PRECISION_HCTRL_ENABLED
        );
        AppMotion_SetReverseTorqueGuard(
            PROFILE_PRECISION_RTGUARD_ENABLED
        );
        AppMotion_SetDecelProfile(
            PROFILE_PRECISION_DPROF_DECEL_START_COUNTS,
            PROFILE_PRECISION_DPROF_APPROACH_DUTY,
            PROFILE_PRECISION_DPROF_ENABLED
        );
        AppMotion_SetSpeedControl(
            PROFILE_PRECISION_SPCTRL_TARGET_SPEED,
            PROFILE_PRECISION_SPCTRL_KP,
            PROFILE_PRECISION_SPCTRL_KI,
            PROFILE_PRECISION_SPCTRL_ENABLED
        );
        AppMotion_SetSpeedProfile(
            PROFILE_PRECISION_SPPROF_DECEL_START_COUNTS,
            PROFILE_PRECISION_SPPROF_APPROACH_SPEED,
            PROFILE_PRECISION_SPPROF_ENABLED
        );
        AppMotion_SetStopControl(
            PROFILE_PRECISION_STOPCTRL_BRAKE_LEAD_COUNTS,
            PROFILE_PRECISION_STOPCTRL_BRAKE_ENTRY_SPEED,
            PROFILE_PRECISION_STOPCTRL_ENABLED
        );
        AppMotion_SetDistanceCal(
            PROFILE_PRECISION_MOVCAL_SLOPE_COUNTS_PER_CM,
            PROFILE_PRECISION_MOVCAL_OFFSET_COUNTS,
            PROFILE_PRECISION_MOVCAL_ENABLED
        );
        AppMotion_SetServoFeedForward(
            MOTION_FORWARD,
            PROFILE_PRECISION_SFF_FORWARD_START_SPEED,
            PROFILE_PRECISION_SFF_FORWARD_FULL_SPEED,
            PROFILE_PRECISION_SFF_FORWARD_MAX_US,
            PROFILE_PRECISION_SFF_FORWARD_ENABLED
        );
        AppMotion_SetServoFeedForward(
            MOTION_BACKWARD,
            PROFILE_PRECISION_SFF_BACKWARD_START_SPEED,
            PROFILE_PRECISION_SFF_BACKWARD_FULL_SPEED,
            PROFILE_PRECISION_SFF_BACKWARD_MAX_US,
            PROFILE_PRECISION_SFF_BACKWARD_ENABLED
        );
    }

    PID_Reset(&speedPID);
    PID_Reset(&steeringPID);
    currentProfile = appliedProfile;
    applyingProfile = false;

    taskEXIT_CRITICAL();
}

AppMotionProfile AppMotion_GetProfile(void)
{
    AppMotionProfile profile;

    taskENTER_CRITICAL();
    profile = currentProfile;
    taskEXIT_CRITICAL();

    return profile;
}

void AppMotion_GetTelemetry(
    AppMotionTelemetry *telemetry
)
{
    if (telemetry == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *telemetry = publishedTelemetry;
    taskEXIT_CRITICAL();
}

void AppMotion_GetEvent(
    AppMotionEvent *event
)
{
    if (event == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *event = publishedMotionEvent;
    taskEXIT_CRITICAL();
}

void AppMotion_GetTurnBrakeSnapshot(
    AppMotionTurnBrakeSnapshot *snapshot
)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = publishedTurnBrakeSnapshot;
    taskEXIT_CRITICAL();
}
