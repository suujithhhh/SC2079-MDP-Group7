/*
 * app_motion.h
 *
 *  Created on: 1 Sept 2026
 *      Author: ksuji
 */

#ifndef APP_MOTION_H
#define APP_MOTION_H

#include <stdint.h>
#include <stdbool.h>

#include "app_ir.h"

typedef enum
{
    MOTION_STOP = 0,
    MOTION_FORWARD,
    MOTION_BACKWARD,
    MOTION_FORWARD_LEFT,
    MOTION_FORWARD_RIGHT,
    MOTION_BACKWARD_LEFT,
    MOTION_BACKWARD_RIGHT,
    MOTION_TUNE_LEFT,
    MOTION_TUNE_RIGHT,
    MOTION_TUNE_BOTH,
    MOTION_TUNE_STEERING_FORWARD,
    MOTION_TUNE_STEERING_BACKWARD,
    MOTION_ENCODER_RESET,
    MOTION_PWM_TEST,
    MOTION_SPEED_TEST,
    MOTION_SERVO_SET,
    MOTION_IMU_ZERO,
    MOTION_IMU_FLIP,
    MOTION_HEADING_RAW,
    MOTION_HEADING_FORWARD,
    MOTION_HEADING_BACKWARD,
    /* Phase 5: forward movement terminated by the far edge of an obstacle
     * seen by the LEFT / RIGHT side IR sensor (MOV,FIRL / MOV,FIRR; value is
     * the maximum travel in cm). Needs APP_IR_ENABLE; see AppMotion_ServiceIrEdge. */
    MOTION_FORWARD_IR_LEFT,
    MOTION_FORWARD_IR_RIGHT
} MotionType;

typedef struct
{
    MotionType type;
    int32_t value;
} MotionCommand;

typedef enum
{
    APP_MOTION_IDLE = 0,
    APP_MOTION_DISTANCE,
    /* PRE_STEER: non-blocking wait ahead of APP_MOTION_TURN -- servo is
     * commanded once to the existing steering controller's initial
     * output, motors are held at zero output (AppMotor_Stop(), no
     * active braking), wheel targets do not ramp. See
     * AppMotion_SetPreSteer()'s doc comment. Enabled by default
     * (physically validated 150.0ms, see AppMotion_Init()); when
     * disabled, MOV,FL/FR/BL/BR go straight to APP_MOTION_TURN,
     * byte-identical to before this state existed. */
    APP_MOTION_PRE_STEER,
    APP_MOTION_TURN,
    APP_MOTION_TUNE_LEFT,
    APP_MOTION_TUNE_RIGHT,
    APP_MOTION_TUNE_BOTH,
    APP_MOTION_TUNE_STEERING,
    APP_MOTION_DIAGNOSTIC_PWM,
    APP_MOTION_DIAGNOSTIC_SPEED,
    APP_MOTION_DIAGNOSTIC_HEADING_RAW,
    APP_MOTION_DIAGNOSTIC_HEADING,
    APP_MOTION_WAIT_IMU,
    APP_MOTION_BRAKING,
    /* Phase 5: MOV,FIRL/FIRR accepted, motors stopped, waiting (bounded) for
     * the selected IR sensor to deliver usable scans. No movement telemetry
     * and no EV:START until driving begins. */
    APP_MOTION_IR_WAIT
} AppMotionState;

typedef enum
{
    APP_PID_LEFT = 0,
    APP_PID_RIGHT,
    APP_PID_STEERING
} AppPIDTarget;

/* Named runtime configuration profile (motion-profiles branch). The named
 * profiles are PRECISION, STANDARD and TASK2TEST (a physically tuned Task 2
 * test profile, not a production-final one). PRECISION is the power-on
 * default (see AppMotion_Init()) and CUSTOM is never a settable target --
 * it is the state AppMotion_ApplyProfile() moves AWAY from and every other
 * profile-tracked Set* function moves back TO the instant it is called
 * outside of an AppMotion_ApplyProfile() call (see AppMotion_GetProfile()).
 * CUSTOM stays last; existing values are unchanged except CUSTOM, which
 * moves up by one. */
typedef enum
{
    APP_MOTION_PROFILE_PRECISION = 0,
    APP_MOTION_PROFILE_STANDARD,
    APP_MOTION_PROFILE_TASK2TEST,
    APP_MOTION_PROFILE_CUSTOM
} AppMotionProfile;

typedef enum
{
    APP_TUNE_NONE = 0,
    APP_TUNE_LEFT,
    APP_TUNE_RIGHT,
    APP_TUNE_BOTH,
    APP_TUNE_STEERING,
    APP_TUNE_MOV_DISTANCE,
    /* Telemetry-only: MOV,FL/FR/BL/BR now arm this mode (mirroring how
     * MOV,F/B arm APP_TUNE_MOV_DISTANCE) so the unified MM, line (see
     * AppMotionTelemetry below) is populated/published for turns.
     * Carries no control-behavior implication -- APP_MOTION_TURN's own
     * PID/servo logic is entirely unaffected by this mode existing. */
    APP_TUNE_MOV_TURN
} AppMotionTuneMode;

typedef enum
{
    APP_TUNE_RESULT_NONE = 0,
    APP_TUNE_RESULT_DONE,
    APP_TUNE_RESULT_TIMEOUT
} AppMotionTuneResult;

typedef struct
{
    AppMotionTuneMode mode;
    AppMotionTuneMode completedMode;
    AppMotionTuneResult completedResult;
    uint32_t runSequence;
    uint32_t completionSequence;
    uint8_t active;

    int32_t leftTargetCounts;
    int32_t leftPositionCounts;
    int32_t leftErrorCounts;
    int32_t leftOutput;

    int32_t rightTargetCounts;
    int32_t rightPositionCounts;
    int32_t rightErrorCounts;
    int32_t rightOutput;

    int32_t headingTargetX10;
    int32_t headingCurrentX10;
    int32_t headingErrorX10;
    int32_t steeringCorrectionUs;
    uint32_t servoPulseUs;

    /* Stage A1: rear-wheel synchronization tuning visibility (MS, line).
     * distanceLeft/RightProgressCounts are direction-normalized (>=0 as
     * the move executes). wheelSyncCorrectionDuty is the signed sync
     * trim as calculated (clamped to +-maxDuty), BEFORE the existing
     * final tolerance/min-duty/stall clamp behaviour; 0 whenever sync is
     * inactive (disabled, non-cruise, or stall recovery active). */
    int32_t distanceLeftProgressCounts;
    int32_t distanceRightProgressCounts;
    int32_t wheelSyncErrorCounts;
    int32_t wheelSyncCorrectionDuty;

    /* Stage A2: direct gyro-Z heading rate-damping tuning visibility
     * (HS, line). headingYawRateX10 is AppIMU_GyroZ(), tenths deg/s.
     * headingRateCorrectionUs is the clamped Krate*gyroZ term (signed
     * us), 0 whenever the controller is disabled.
     * headingFinalCorrectionUs is the combined base+rate output actually
     * used for the servo pulse (== steeringCorrectionUs when disabled). */
    int32_t headingYawRateX10;
    int32_t headingRateCorrectionUs;
    int32_t headingFinalCorrectionUs;

    /* Stage A3.1: reverse-torque guard tuning visibility (RTG, line).
     * left/rightPreGuardCommand are the final motor commands after
     * WSYNC and the existing tolerance/min-duty processing, immediately
     * BEFORE the new suppression (== leftOutput/rightOutput in MD, when
     * the guard does not fire). left/rightGuardActive is 1 only on a
     * tick where the guard actually zeroed that wheel's command.
     * targetCrossedFlag and settledCycleCount are the existing,
     * unmodified completion signals, newly exposed for visibility. */
    int32_t leftPreGuardCommand;
    int32_t rightPreGuardCommand;
    uint8_t leftGuardActive;
    uint8_t rightGuardActive;
    uint8_t targetCrossedFlag;
    int32_t settledCycleCount;

    /* Stage A3.2: deceleration-profile tuning visibility (MP, line).
     * leftRawPidOutput/rightRawPidOutput are leftPID/rightPID's raw
     * output BEFORE the profile substitution and BEFORE WSYNC (unlike
     * MD,LO/RO, which are the final post-everything commands).
     * left/rightVelocityCountsPerSec reuse the existing, unfiltered
     * per-tick motion deltas (stall-detection inputs) scaled to
     * counts/s -- observability only. decelProfileActiveFlag is the
     * chassis-level DECEL gate this tick; decelProfileDuty is the
     * shared ceiling (profileDuty); decelApproachDuty, the floor, is
     * static while enabled and available via DPROF,GET, not resent
     * every tick. left/rightDecelEnforcedFlag are 1 only on a tick
     * where the final per-wheel enforcement changed the post-WSYNC
     * value. left/rightDirectedRemainingCounts are the direction-
     * normalized (positive-while-short, for both F and B) per-wheel
     * remaining counts -- lets an early-finished wheel (near/at 0
     * while the other wheel's value is still large) be identified
     * directly, regardless of move direction. */
    int32_t leftRawPidOutput;
    int32_t rightRawPidOutput;
    int32_t leftVelocityCountsPerSec;
    int32_t rightVelocityCountsPerSec;
    uint8_t decelProfileActiveFlag;
    int32_t decelProfileDuty;
    uint8_t leftDecelEnforcedFlag;
    uint8_t rightDecelEnforcedFlag;
    int32_t leftDirectedRemainingCounts;
    int32_t rightDirectedRemainingCounts;

    /* Stage A3.3a: closed-loop chassis speed controller tuning
     * visibility (SPC, line). speedControlActiveFlag is the per-tick
     * "active" gate (== spctrlEnabled). TargetSpeed is the CONFIGURED
     * cruise target (SPCTRL,GET) -- unchanged by Stage A3.3b.
     * ActualSpeed is the raw (unfiltered) directed chassis speed;
     * FilteredSpeed is the EMA value actually fed to the speed PI --
     * both unchanged by Stage A3.3b. ProportionalContribution/
     * IntegralContribution are speedPID's own kp*error / ki*integral
     * terms this tick, 0 whenever inactive. RawBaseDuty is speedPID's
     * output magnitude, already clamped to [0,MOTOR_MAX_DUTY].
     * FinalBaseDuty is the signed common base command as it stood right
     * after WSYNC and the no-reversal clamp, before the legacy floor
     * chain and RTGUARD.
     *
     * Stage A3.3b: DynamicTargetSpeed is effectiveTargetSpeed, the
     * setpoint PID_Update actually used this tick -- equals TargetSpeed
     * whenever the Stage A3.3b profile (SPPROF) is disabled or outside
     * its decel zone, and falls continuously toward the configured
     * approach speed inside it. ChassisRemainingCounts is the signed,
     * direction-normalized common chassis remaining distance the
     * profile is driven from (same quantity DPROF's own per-wheel
     * LDR/RDR are built from, chassis-averaged). */
    uint8_t speedControlActiveFlag;
    int32_t speedControlTargetSpeed;
    int32_t speedControlActualSpeed;
    int32_t speedControlFilteredSpeed;
    int32_t speedControlErrorSpeed;
    int32_t speedControlProportionalContribution;
    int32_t speedControlIntegralContribution;
    int32_t speedControlRawBaseDuty;
    int32_t speedControlFinalBaseDuty;
    int32_t speedControlDynamicTargetSpeed;
    int32_t speedControlChassisRemainingCounts;

    /* Stage A3.3c: final controlled stop gate tuning visibility (STP,
     * line). EnabledFlag is the persistent stopctrlEnabled flag
     * (STOPCTRL,GET). TriggeredFlag is the per-tick "close AND slow" gate
     * result -- 1 only on the tick it causes AppMotion_BeginBrake() to
     * fire; best-effort to observe on the wire (the shared telemetry
     * mailbox is republished with mode/active cleared by
     * AppMotion_BeginBrake() itself immediately afterward, on the same
     * tick), so treat a captured 1 as a bonus confirmation, not the
     * primary tuning signal -- MP,LDR/RDR and SPC,RSP (continuously
     * published) are. */
    uint8_t stopControlEnabledFlag;
    uint8_t stopControlTriggeredFlag;

    /* Stage SFF1: speed-aware servo feed-forward tuning visibility (SFF,
     * line). Direction is the literal 'F'/'B' character of the command
     * this tick belongs to (matches STR,DIR's own convention from the
     * earlier, since-reverted Stage A5.3a work). EnabledFlag is that
     * direction's own persistent enable flag (SFF,GET). FilteredSpeed is
     * filteredChassisSpeed as read this tick -- duplicated here
     * (already published continuously via SPC,RSP) specifically so a
     * bench operator can correlate actual speed against the applied
     * feed-forward from one single frame, without cross-referencing SPC,.
     * OutputUs is sffOutputUs exactly as added into servoPulse this tick
     * (0 whenever that direction's SFF is disabled). The resulting final
     * servo pulse is NOT duplicated here -- already available,
     * automatically inclusive of this term, via the existing HS,SP. */
    uint8_t sffDirection;
    uint8_t sffEnabledFlag;
    int32_t sffFilteredSpeed;
    int32_t sffOutputUs;

    /* Unified movement telemetry (MM, line + EV:START/EV:END markers in
     * app_comm.c), covers all six MOV primitives (F/B/FL/FR/BL/BR).
     * Telemetry-only -- none of these fields are read by any control
     * computation. activeDirection is the activeCommand MotionType this
     * movement belongs to. commandValue is the raw requested cm/degree
     * value, captured at command entry (EV:START's V:). movementStartTick
     * is the tick this movement began (xTaskGetTickCount()), used to
     * derive elapsedMs here and EV:END's T: in app_comm.c.
     * left/rightDisplacementCounts are RAW SIGNED
     * (currentCounts - startCounts), deliberately NOT direction-
     * normalized -- F/FL/FR read positive, B/BL/BR read negative, by
     * design, to expose actual encoder direction/sign correctness. This
     * is a different quantity from leftPositionCounts/rightPositionCounts
     * above (MD's LP/RP, absolute position) -- MD's own fields and
     * format are completely unchanged by this addition.
     * movementBraking is 1 only on the one frozen snapshot
     * AppMotion_BeginBrake() publishes (ST:BRAKE) -- on that frame,
     * leftOutput/rightOutput are the LAST PRE-BRAKE commands, not a live
     * AppMotor_Set() duty (AppMotor_Brake() takes no per-wheel value).
     * movementRecoveryActive mirrors the existing, unmodified
     * distanceStallRecoveryActive flag (APP_MOTION_DISTANCE-only,
     * ST:RECOVERY). */
    MotionType activeDirection;
    int32_t commandValue;
    uint32_t movementStartTick;
    uint32_t elapsedMs;
    int32_t leftDisplacementCounts;
    int32_t rightDisplacementCounts;
    uint8_t movementBraking;
    uint8_t movementRecoveryActive;

    /* PRE_STEER (ST:PRE): 1 only during APP_MOTION_PRE_STEER ticks, 0
     * everywhere else including once real APP_MOTION_TURN begins.
     * During PRE, leftOutput/rightOutput are truthfully 0 (leftPID/
     * rightPID are never updated during PRE, not a placeholder), and
     * servoPulseUs reflects the one-shot seed pulse actually commanded
     * to the servo at PRE entry (via AppServo_GetUs()). */
    uint8_t movementPreSteering;

    /* Ackermann-v2 (AK, line): the actual inner/outer target-rate
     * MAGNITUDES (counts/sec, always >=0, rounded to nearest integer
     * -- NOT truncated, unlike most count fields in this struct, since
     * k's small [0,0.5] range makes a 1-unit truncation error visually
     * confusing at this precision) selected for THIS tick by
     * AppMotion_SelectAckermannRates() and actually used to ramp
     * targetLeftCounts/targetRightCounts this tick during
     * APP_MOTION_TURN. The float rates the controller itself uses are
     * never rounded -- only this telemetry copy is. Both read 0 during
     * APP_MOTION_PRE_STEER (no ramp occurs yet). */
    int32_t ackermannLeftRateCountsPerSec;
    int32_t ackermannRightRateCountsPerSec;
} AppMotionTelemetry;

typedef enum
{
    APP_MOTION_EVENT_NONE = 0,
    APP_MOTION_EVENT_MOV_DONE,
    APP_MOTION_EVENT_ENCODER_RESET_OK,
    APP_MOTION_EVENT_PWM_DONE,
    APP_MOTION_EVENT_SPEED_DONE,
    APP_MOTION_EVENT_SERVO_SET,
    APP_MOTION_EVENT_IMU_ZERO_OK,
    APP_MOTION_EVENT_IMU_ZERO_ERROR,
    APP_MOTION_EVENT_IMU_FLIP_OK,
    APP_MOTION_EVENT_IMU_FLIP_ERROR,
    APP_MOTION_EVENT_IMU_NOT_READY,
    APP_MOTION_EVENT_HEADING_RAW_DONE,
    APP_MOTION_EVENT_HEADING_DONE,
    APP_MOTION_EVENT_MOTION_TIMEOUT,
    APP_MOTION_EVENT_PID_NOT_SET,
    APP_MOTION_EVENT_BUSY,
    APP_MOTION_EVENT_BAD_COMMAND,
    /* Terminal event of an internal MOTION_STOP (see
     * AppMotion_HandleStop() in app_motion.c). command is the interrupted
     * MotionType, or MOTION_STOP if STOP arrived while already idle;
     * value/left/right/headingX10 are always 0. Published once per stop
     * operation: immediately for an idle STOP or STOP during
     * APP_MOTION_WAIT_IMU, otherwise when the active-brake window ends
     * and the motors are released. It supersedes (the superseded event is
     * never published) any pending MOV_DONE/MOTION_TIMEOUT/IMU_NOT_READY
     * or diagnostic completion. Means "brake phase complete", not "zero
     * velocity verified". On the wire it is reported by app_comm.c as
     * "MM,EV:END,...,RESULT:STOPPED" (only if a START was sent) followed
     * by "ACK,STOPPED"; see PROTOCOL.md. */
    APP_MOTION_EVENT_STOPPED,
    /* Phase 5 terminal errors of MOV,FIRL/FIRR (PROTOCOL.md): the maximum
     * travel was reached without ever confirming the obstacle / without
     * confirming its far edge, or the IR sensor was not usable. */
    APP_MOTION_EVENT_IR_NO_OBSTACLE,
    APP_MOTION_EVENT_IR_NO_EDGE,
    APP_MOTION_EVENT_IR_SENSOR
} AppMotionEventType;

typedef struct
{
    AppMotionEventType type;
    MotionType command;
    uint32_t sequence;
    int32_t value;
    int32_t left;
    int32_t right;
    int32_t headingX10;
} AppMotionEvent;

/* TURNBRAKE: retained, sequence-gated one-shot diagnostic snapshot --
 * same mailbox pattern as AppMotionEvent/AppMotion_GetEvent() above, a
 * second independent instance of it, not a new queue/task. Published
 * exactly once per turn, only when the predictive-brake condition (see
 * AppMotion_SetTurnBrake()'s own doc comment) actually fires -- never
 * touched by the ordinary dwell/timeout completion paths. `valid` is 0
 * until the first trigger ever occurs (explicitly initialized in
 * AppMotion_Init(), not left to implicit BSS zero-init). The heading/
 * angle/rate X10 fields are tenths of a degree (or degrees/second for
 * gyroZDpsX10), matching this file's existing headingTargetX10/
 * headingCurrentX10/headingErrorX10 convention; configuredLookaheadMsX10
 * is tenths of a millisecond (see its own comment) -- a different unit,
 * same X10-for-one-decimal-place convention. */
typedef struct
{
    uint32_t sequence;
    uint8_t valid;
    MotionType direction;
    int32_t targetHeadingX10;
    int32_t currentHeadingX10;
    int32_t remainingAngleX10;
    int32_t gyroZDpsX10;
    int32_t predictedStoppingAngleX10;
    /* Tenths of a millisecond -- preserves turnBrakeLookaheadMs's own
     * fractional precision (it is stored/settable as a float, same as
     * PRESTEER's durationMs), consistent with this file's existing X10
     * convention for fractional telemetry values rather than truncating
     * to whole milliseconds. */
    int32_t configuredLookaheadMsX10;
    uint8_t confirmationCycles;
} AppMotionTurnBrakeSnapshot;

void AppMotion_Init(void);

void AppMotion_HandleCommand(
    const MotionCommand *command
);

void AppMotion_Update(float dtSeconds);

/* Read-only, atomic movement identity + measured speed for CommTask's
 * ultrasonic parking protection (USPARK). All four fields are copied inside
 * ONE critical section, so identity and speed always belong together:
 *   state        motionState (APP_MOTION_DISTANCE while a straight MOV,F/B
 *                runs, APP_MOTION_BRAKING while braking, ...)
 *   command      activeCommand (MOTION_FORWARD for MOV,F)
 *   movementSeq  tuneTelemetry.runSequence: increments on every MOV/TUNE
 *                command that starts executing (never 0, skips 0 on wrap),
 *                so a new straight-forward movement always has a new value
 *   filteredSpeedCountsPerSec  filteredChassisSpeed, the encoder-measured,
 *                direction-normalised EMA (counts/s), updated every
 *                APP_MOTION_DISTANCE tick whether or not SPCTRL is enabled.
 *                DIAGNOSTIC ONLY in this version: nothing may use it for a
 *                stopping decision until stopping-distance evidence exists.
 * No setter exists and no internal state is exposed beyond these copies. */
typedef struct
{
    AppMotionState state;
    MotionType command;
    uint32_t movementSeq;
    int32_t filteredSpeedCountsPerSec;
} AppMotionMovementStatus;

void AppMotion_GetMovementStatus(AppMotionMovementStatus *status);

AppMotionState AppMotion_GetState(void);

#if APP_IR_ENABLE
/* Phase 5 (MOV,FIRL / MOV,FIRR). One-shot diagnostic of the edge decision,
 * same retained-mailbox pattern as AppMotionTurnBrakeSnapshot: published
 * exactly once per FIR movement, when the decision is made (edge confirmed,
 * maximum travel reached, sensor failure); never on STOP. Counts are
 * chassis displacement (mean of both wheels, encoder counts) from the start
 * of driving at the confirming scan, -1 if that never happened. */
typedef enum
{
    APP_MOTION_IR_RESULT_EDGE = 0,
    APP_MOTION_IR_RESULT_NO_OBSTACLE,
    APP_MOTION_IR_RESULT_NO_EDGE,
    APP_MOTION_IR_RESULT_SENSOR
} AppMotionIrEdgeResult;

typedef struct
{
    uint32_t sequence;          /* 0 = never published                      */
    uint8_t valid;
    MotionType direction;
    uint8_t result;             /* AppMotionIrEdgeResult                     */
    uint8_t startedVisible;     /* the obstacle was already beside the robot */
    uint8_t failReason;         /* AppIrEdgeFailReason, SENSOR only          */
    int32_t seenCounts;         /* -1 = obstacle never confirmed             */
    int32_t clearCounts;        /* -1 = edge never confirmed                 */
    uint16_t seenRaw;
    uint16_t clearRaw;
    uint16_t missedScans;
    uint16_t unusableScans;
    uint16_t ambiguousScans;
    uint32_t readyMs;           /* command -> sensor ready (IR_WAIT time)    */
} AppMotionIrEdgeSnapshot;

/* True from the acceptance of MOV,FIRL/FIRR until the movement ends.
 * MotionTask only: while true it calls AppMotion_ServiceIrEdge() every loop,
 * BEFORE AppMotion_Update() (not inside it, to keep Update's stack frame
 * out of the IR call chain). */
bool AppMotion_IrEdgeArmed(void);

/* Feeds the newest raw IR publication to the edge detector and acts on its
 * outcome: start driving when the sensor is ready, pull the target in when
 * the edge is confirmed, brake with ERR,IR_SENSOR when it fails. A no-op
 * unless a FIR movement is in IR_WAIT or driving. */
void AppMotion_ServiceIrEdge(const AppIrSnapshot *snapshot, uint32_t nowTick);

void AppMotion_GetIrEdgeSnapshot(AppMotionIrEdgeSnapshot *snapshot);
#endif

bool AppMotion_IsBusy(void);

void AppMotion_SetPIDGains(
    AppPIDTarget target,
    float kp,
    float ki,
    float kd
);

void AppMotion_GetPIDGains(
    AppPIDTarget target,
    float *kp,
    float *ki,
    float *kd
);

void AppMotion_GetTelemetry(
    AppMotionTelemetry *telemetry
);

void AppMotion_GetEvent(
    AppMotionEvent *event
);

/* Retained one-shot TURNBRAKE diagnostic snapshot -- see
 * AppMotionTurnBrakeSnapshot's own doc comment above. Available at any
 * time (matches AppMotion_GetEvent()'s own semantics); callers detect a
 * new trigger via the `sequence` field, same as AppMotion_GetEvent(). */
void AppMotion_GetTurnBrakeSnapshot(
    AppMotionTurnBrakeSnapshot *snapshot
);

/* Stage A1: RAM-only rear-wheel synchronization trim parameters. Applied
 * only during the cruise portion of APP_MOTION_DISTANCE (MOV,F / MOV,B).
 * Disabled by default; no flash persistence. The caller (app_comm.c)
 * is responsible for gating SET to an idle motion system. */
void AppMotion_SetWheelSync(
    float gain,
    float maxDuty,
    bool enable
);

void AppMotion_GetWheelSync(
    float *gain,
    float *maxDuty,
    bool *enable
);

/* Stage A2: RAM-only direct gyro-Z heading rate-damping trim parameters.
 * Applied throughout APP_MOTION_DISTANCE (MOV,F / MOV,B) as an
 * additional term on top of steeringPID's existing (unmodified) output.
 * Disabled by default; no flash persistence. The caller (app_comm.c) is
 * responsible for gating SET to an idle motion system. */
void AppMotion_SetHeadingRateDamping(
    float krate,
    float maxCorrection,
    bool enable
);

void AppMotion_GetHeadingRateDamping(
    float *krate,
    float *maxCorrection,
    bool *enable
);

/* Stage A3.1: RAM-only reverse-torque guard enable flag. Applied only
 * inside the existing per-wheel approach zone of APP_MOTION_DISTANCE
 * (MOV,F / MOV,B); never touches cruise or tolerance. Disabled by
 * default; no flash persistence. The caller (app_comm.c) is responsible
 * for gating SET to an idle motion system. */
void AppMotion_SetReverseTorqueGuard(
    bool enable
);

void AppMotion_GetReverseTorqueGuard(
    bool *enable
);

/* Stage A3.2: RAM-only common-profile deceleration controller
 * parameters. Applied only during APP_MOTION_DISTANCE (MOV,F / MOV,B).
 * Disabled by default; no flash persistence. The caller (app_comm.c) is
 * responsible for gating SET to an idle motion system. */
void AppMotion_SetDecelProfile(
    float decelCounts,
    float approachDuty,
    bool enable
);

void AppMotion_GetDecelProfile(
    float *decelCounts,
    float *approachDuty,
    bool *enable
);

/* Stage A3.3a: RAM-only fixed-target closed-loop chassis speed
 * controller. Applied throughout APP_MOTION_DISTANCE (MOV,F / MOV,B) as
 * the common base duty, in place of the legacy leftPID/rightPID output,
 * whenever enabled; WSYNC still applies L/R trim on top. Mutually
 * exclusive with the Stage A3.2 deceleration profile -- rejected by the
 * caller (app_comm.c) at SET time in both directions. Disabled by
 * default; no flash persistence. The caller is responsible for gating
 * SET to an idle motion system. */
void AppMotion_SetSpeedControl(
    float targetSpeed,
    float kp,
    float ki,
    bool enable
);

void AppMotion_GetSpeedControl(
    float *targetSpeed,
    float *kp,
    float *ki,
    bool *enable
);

/* Stage A3.3b: RAM-only distance-based linear speed profile, layered on
 * top of the Stage A3.3a speed controller. Only ever has an effect while
 * that controller is active (spctrlEnabled) -- it never claims magnitude
 * ownership itself, it only adjusts the target speed fed into speedPID.
 * Applied throughout APP_MOTION_DISTANCE (MOV,F / MOV,B). Disabled by
 * default; no flash persistence. The caller (app_comm.c) is responsible
 * for gating SET to an idle motion system, and for enforcing that
 * approachSpeed never exceeds the currently-configured SPCTRL cruise
 * target in either direction (SPPROF SET and SPCTRL SET each check the
 * other). */
void AppMotion_SetSpeedProfile(
    float decelStartCounts,
    float approachSpeed,
    bool enable
);

void AppMotion_GetSpeedProfile(
    float *decelStartCounts,
    float *approachSpeed,
    bool *enable
);

/* Stage A3.3c: RAM-only final controlled stop gate, layered on top of the
 * Stage A3.3a speed controller. ADDITIVE only -- never replaces the
 * existing position-tolerance/target-crossing completion path, only OR's
 * an early trigger alongside it into the existing, unmodified
 * AppMotion_BeginBrake() call. Only ever has an effect while SPCTRL is
 * active (spctrlEnabled) -- it never claims magnitude ownership itself.
 * Applied throughout APP_MOTION_DISTANCE (MOV,F / MOV,B). Disabled by
 * default; no flash persistence. The caller (app_comm.c) is responsible
 * for gating SET to an idle motion system. */
void AppMotion_SetStopControl(
    float brakeLeadCounts,
    float brakeEntrySpeed,
    bool enable
);

void AppMotion_GetStopControl(
    float *brakeLeadCounts,
    float *brakeEntrySpeed,
    bool *enable
);

/* Stage A4: RAM-only distance (counts/cm) calibration for MOV,F/B target
 * generation. Disabled by default -- when disabled, target-count generation
 * is exactly ENCODER_COUNTS_PER_CM-based, byte-identical to before this
 * stage; when enabled, distanceCounts = cm*slopeCountsPerCm + offsetCounts
 * (cm==0 always maps to exactly 0 counts regardless of this setting). Does
 * not affect WSYNC/HCTRL/SPCTRL/SPPROF/STOPCTRL/DPROF, PID tuning, braking,
 * turns, or the SPEED_TEST diagnostic's own use of ENCODER_COUNTS_PER_CM.
 * No flash persistence. The caller (app_comm.c) is responsible for gating
 * SET to an idle motion system, and for rejecting a SET that would let
 * slopeCountsPerCm+offsetCounts fall to zero or below. */
void AppMotion_SetDistanceCal(
    float slopeCountsPerCm,
    float offsetCounts,
    bool enable
);

void AppMotion_GetDistanceCal(
    float *slopeCountsPerCm,
    float *offsetCounts,
    bool *enable
);

/* Stage SFF1/SFF2/SFF3: RAM-only speed-aware servo feed-forward,
 * independent per direction. Applied only during APP_MOTION_DISTANCE
 * (MOV,F / MOV,B), added after steeringPID+HCTRL's finalSteeringOutput is
 * fully computed -- never mixed into that math -- and keyed on the
 * existing filteredChassisSpeed (Stage A3.3a), not a new speed
 * measurement. direction must be MOTION_FORWARD or MOTION_BACKWARD.
 * Boot-enabled by default for BOTH directions (DEFAULT_SFF_FORWARD_ and
 * DEFAULT_SFF_BACKWARD_ constants, both physically validated --
 * Backward's maxUs is independently signed from Forward's, never
 * auto-inverted); no flash persistence.
 *
 * Stage SFF2: now profile-owned, part of PROFILE_PRECISION_*,
 * PROFILE_STANDARD_* and PROFILE_TASK2TEST_* (see AppMotion_ApplyProfile()) -- a direct SET call
 * marks the active profile CUSTOM, exactly like every other
 * profile-owned Set* function below; AppMotion_ApplyProfile() calling
 * this on its own behalf does not (see its own doc comment). The caller
 * (app_comm.c) is responsible for gating SET to an idle motion system. */
void AppMotion_SetServoFeedForward(
    MotionType direction,
    float startSpeed,
    float fullSpeed,
    float maxUs,
    bool enable
);

void AppMotion_GetServoFeedForward(
    MotionType direction,
    float *startSpeed,
    float *fullSpeed,
    float *maxUs,
    bool *enable
);

/* PRE_STEER: RAM-only, turn-only (MOV,FL/FR/BL/BR), non-blocking
 * pre-turn wait. When enabled, turn command entry commands the servo
 * ONCE, directly, to the fixed max-safe turn-servo endpoint
 * (AppMotion_TurnServoEndpointUs() in app_motion.c -- SERVO_MIN_US for
 * FL/BR, SERVO_MAX_US for FR/BL; steeringPID is not evaluated for this
 * any more), holds the motors at zero output (AppMotor_Stop(), no
 * active braking), and does not ramp the wheel targets, for
 * durationMs before starting today's unmodified APP_MOTION_TURN logic
 * (which re-anchors wheel targets from fresh encoder reads at that
 * transition and also holds the servo at that same fixed endpoint
 * throughout, not just at entry). durationMs is clamped to [0,1000]
 * by the caller (app_comm.c/vision_rx.c); 0 is a valid, distinct-
 * from-disabled state (enters PRE_STEER, transitions to TURN on the
 * very next tick). Disabled by default -- when disabled, turn command
 * entry is byte-identical to before this feature existed. Does not
 * change targetHeadingDeg's sign logic, leftPID/rightPID, the runtime
 * base turn rate (TURNSPEED), settle/completion tolerance, or
 * braking. Deliberately NOT profile-owned (unlike every other Set*
 * above) -- does not mark the active profile CUSTOM, and is not part
 * of the PROFILE_PRECISION_ / PROFILE_STANDARD_ / PROFILE_TASK2TEST_
 * constant bundles or
 * AppMotion_ApplyProfile(); this is an orthogonal staging delay, not a
 * tuned control parameter, and marking CUSTOM
 * would fight the runtime sweep workflow this feature exists for. The
 * caller (app_comm.c) is responsible for gating SET to an idle motion
 * system; GET is available at any time. */
void AppMotion_SetPreSteer(
    float durationMs,
    bool enable
);

void AppMotion_GetPreSteer(
    float *durationMs,
    bool *enable
);

/* TURNSPEED: RAM-only, turn-only (MOV,FL/FR/BL/BR), replaces the
 * previously-hardcoded base turn rate (2000 counts/sec) that
 * case APP_MOTION_TURN: ramps targetLeftCounts/targetRightCounts by.
 * countsPerSec is an integer (the wire/API type throughout, cast to
 * float only where it feeds the target-rate ramp in app_motion.c) and
 * is defensively clamped to [500,2500] inside AppMotion_SetTurnSpeed()
 * itself (not just relying on the parser's own range check in
 * vision_rx.c), so a future internal caller cannot bypass that safety
 * range. That range is a tuning-quality bound, not a hardware-safety
 * one -- leftPID/rightPID's own output clamp ([-99,99],
 * AppMotion_Init()) bounds the motor command independent of how large
 * the target-rate magnitude is; a larger rate only makes the setpoint
 * move faster, never pushes the PID output outside its existing
 * clamp. Default 1700, the physically validated boot default (see
 * AppMotion_Init()) -- a fresh boot (or GET before any SET) reports
 * this value, so unconfigured behavior is recognizable.
 * Does not change targetHeadingDeg, the fixed turn-servo endpoint
 * (PRE_STEER above), settle/completion tolerance, or braking.
 * Deliberately NOT profile-owned, same reasoning as PRE_STEER above.
 * The caller (app_comm.c) is responsible for gating SET to an idle
 * motion system; GET is available at any time. */
void AppMotion_SetTurnSpeed(
    uint32_t countsPerSec
);

void AppMotion_GetTurnSpeed(
    uint32_t *countsPerSec
);

/* Ackermann-v2: RAM-only, turn-only (MOV,FL/FR/BL/BR), redistributes
 * the runtime TURNSPEED base rate (AppMotion_GetTurnSpeed(), never a
 * hardcoded value) between the inner/outer rear wheel targets --
 * innerRate=base*(1-k), outerRate=base*(1+k), so the average always
 * equals the current TURNSPEED regardless of k. See
 * AppMotion_SelectAckermannRates() (app_motion.c) for the exact
 * inner/outer mapping. Does not touch leftPID/rightPID's equations,
 * the motor min-duty floor logic, steeringPID (still dormant), the
 * fixed turn-servo endpoint, or PRE_STEER. Does not change the
 * heading-based turn-completion CRITERION (still driven purely by
 * headingError/settledCycles, unmodified) -- but because it changes
 * rear-wheel motion, it may change the elapsed TIME needed to reach
 * that same, unmodified criterion. k is defensively clamped to
 * [0.0,0.5] inside this setter (not just relying on the parser's own
 * range check in vision_rx.c), so a future internal caller cannot
 * bypass that range; enable semantics are not affected by the clamp.
 * Enabled by default at k=0.270, the physically validated boot default
 * (see AppMotion_Init()). Deliberately NOT profile-owned, same
 * reasoning as PRESTEER/TURNSPEED above. The caller (app_comm.c) is
 * responsible for gating SET to an idle motion system; GET is
 * available at any time. */
void AppMotion_SetAckermann(
    float k,
    bool enable
);

void AppMotion_GetAckermann(
    float *k,
    bool *enable
);

/* TURNBRAKE: RAM-only, turn-only (MOV,FL/FR/BL/BR), predictive brake
 * trigger. Independent of, and a fallback-preserving ADDITION to, the
 * existing heading-tolerance/5-cycle dwell completion in
 * case APP_MOTION_TURN: -- when disabled (enabled is the boot default,
 * see AppMotion_Init()), turn completion is byte-identical to today's
 * heading-tolerance/5-cycle dwell path alone. When enabled, each turn tick
 * estimates predictedStoppingAngleDeg = |AppIMU_GyroZ()| * (lookaheadMs
 * / 1000) and, once the chassis is confirmed still approaching the
 * target (headingError and yaw rate sharing sign) at or above a fixed
 * 5.0 deg/s noise floor, with the remaining angle already at or below
 * that predicted stopping angle, on two CONSECUTIVE turn ticks (not
 * cumulative/latched across a miss), calls the existing, unmodified
 * AppMotion_BeginBrake(APP_MOTION_EVENT_MOV_DONE) immediately -- the
 * same terminal path and the same 120ms active-brake duration the
 * dwell path already uses; nothing about braking itself, servo
 * endpoints, Ackermann, or TURNSPEED is changed by this feature.
 * lookaheadMs is defensively clamped to [0.0,500.0] inside
 * AppMotion_SetTurnBrake() itself (not just relying on the parser's own
 * range check in vision_rx.c); 0.0 is a valid, distinct-from-disabled
 * value (the predictive condition becomes practically unreachable, same
 * "0 distinct from disabled" precedent as PRE_STEER above). Initial
 * stored default is 135.0ms, originally derived from three FL90 bench
 * measurements (see app_motion.c); the combined turn configuration
 * (TURNSPEED=1700/ACKERMANN=0.270,1/PRESTEER=150,1/TURNBRAKE=135,1) was
 * subsequently physically tested across FL/FR/BL/BR at commanded
 * angles of 10/30/45/60/75/90 degrees. This validation record applies
 * specifically to TURNSPEED=1700 with PRESTEER=150,1, ACKERMANN=0.270,1,
 * and TURNBRAKE=135,1; earlier experimental configurations are outside
 * this validation claim. Enabled by default at this value, the
 * physically validated boot default (see AppMotion_Init()).
 * Deliberately NOT profile-owned, same reasoning as PRESTEER/TURNSPEED/
 * ACKERMANN above. The caller (app_comm.c) is responsible for gating
 * SET to an idle motion system; GET is available at any time. */
void AppMotion_SetTurnBrake(
    float lookaheadMs,
    bool enable
);

void AppMotion_GetTurnBrake(
    float *lookaheadMs,
    bool *enable
);

/* Named runtime configuration profile (motion-profiles branch; Stage
 * SFF2 folded speed-aware servo feed-forward into the bundle): bundles
 * Steering PID (PID,S) + WSYNC + HCTRL + RTGUARD + DPROF + SPCTRL +
 * SPPROF + STOPCTRL + MOVCAL + SFF (both directions) into one atomic
 * apply. Implemented entirely
 * by calling the existing Set* functions above under one critical
 * section -- no control logic is duplicated, and none of those
 * controllers' own algorithms are touched. Also resets speedPID's and
 * steeringPID's integral/previousError via the existing, unmodified
 * PID_Reset() (runtime state only; the PID algorithm itself is
 * untouched) -- leftPID/rightPID are not part of any profile and are
 * left alone. PRECISION is the power-on default; no flash persistence.
 * The caller (app_comm.c) is responsible for gating this to an idle
 * motion system, mirroring every other Set* function above.
 *
 * Calling this with anything other than APP_MOTION_PROFILE_STANDARD or
 * APP_MOTION_PROFILE_TASK2TEST applies PRECISION (defensive default --
 * app_comm.c only ever passes PRECISION, STANDARD or TASK2TEST; CUSTOM
 * has no corresponding SET form). */
void AppMotion_ApplyProfile(
    AppMotionProfile profile
);

/* Returns the profile most recently applied via AppMotion_ApplyProfile(),
 * or APP_MOTION_PROFILE_CUSTOM if any individual Set* call above (or
 * PID,S) has changed one of the profile's own fields since -- see each
 * Set* function's own doc comment. Available at any time, including
 * during active motion (matches every other GET in this API). */
AppMotionProfile AppMotion_GetProfile(void);

#endif
