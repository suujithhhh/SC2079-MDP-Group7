/*
 * app_comm.c
 *
 *  Created on: 1 Sept 2026
 *      Author: ksuji
 */


#include "app_comm.h"
#include "app_encoder.h"
#include "app_imu.h"
#include "app_motion.h"
#include "app_servo.h"
#include "vision_rx.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define APP_COMM_TX_TIMEOUT_MS     100U
#define APP_COMM_GAIN_SCALE        1000000.0f
#define APP_COMM_GAIN_SCALE_DIGITS 6
#define APP_COMM_TELEMETRY_RATE_HZ 10U
#define APP_COMM_SERVICE_RATE_HZ   100U
#define APP_COMM_TELEMETRY_BUFFER_SIZE 160U
/* Unified movement telemetry (MM,/EV:) line buffer. Worst case: "MM,"
 * + 16 fields (labels D,T,ST,TH,H,HE,GZ,SP,LT,LP,LE,LO,RT,RP,RE,RO,
 * each "label:" + value + separator), signed 32-bit values up to
 * "-2147483648" (11 chars), "RECOVERY" (8 chars, longest ST value),
 * "FL"/"FR"/"BL"/"BR" (2 chars, longest D value) + '\n' + NUL ~= 227
 * bytes worst case -- sized with margin, not reusing the shorter
 * 160-byte buffer above (MD,/MS,/etc. individually stay well under
 * it, unchanged). */
#define APP_COMM_MOVEMENT_BUFFER_SIZE  256U
#define APP_COMM_UART_HAL_ERROR_MASK  \
    (HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | \
     HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE)
#define APP_COMM_UART_STATUS_ERROR_MASK \
    (UART_FLAG_PE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_ORE)

typedef enum
{
    APP_COMM_RX_SYNCHRONIZED = 0,
    APP_COMM_RX_RESET_PENDING,
    APP_COMM_RX_DISCARD_TO_NEWLINE
} AppCommRxRecoveryState;

extern UART_HandleTypeDef huart3;

extern osMessageQueueId_t CommRxQueueHandle;
extern osMessageQueueId_t MotionQueueHandle;

static vision_rx_t visionRx;
static uint8_t uart3RxByte;
static uint32_t lastTelemetryServiceTick;
static uint32_t lastTelemetryTick;
static uint32_t lastCompletionSequence;
static uint32_t lastMotionEventSequence;
static uint32_t lastTurnBrakeSnapshotSequence;
static uint8_t stateCommandReserved;
static volatile AppCommRxRecoveryState uart3RxRecoveryState;
static volatile uint8_t uart3RxRecoveryRequested;
static volatile uint8_t uart3RxBoundarySeen;
static volatile uint32_t uart3RxErrorCount;
static volatile uint32_t uart3RxDroppedCount;
static char tuneTelemetryResponse[
    APP_COMM_TELEMETRY_BUFFER_SIZE
];
static char movementSummaryResponse[
    APP_COMM_MOVEMENT_BUFFER_SIZE
];

/* Unified movement telemetry (MM,/EV:) CommTask-local bookkeeping only
 * -- never read by app_motion.c, never affects any control computation.
 * lastTelemetryRunSequence detects a newly-armed MOV,F/B/FL/FR/BL/BR
 * command (tuneTelemetry.runSequence already increments at every
 * command entry; TUNE_LEFT/RIGHT/BOTH/STEERING also bump it but are
 * excluded by the mode check at the call site, see
 * AppComm_ServiceTelemetry()). lastKnownDirection/lastKnownStartTick
 * cache the values needed to build MM,EV:END even after
 * tuneTelemetry.active has already been cleared by the time the
 * existing (unmodified) terminal AppMotionEventType arrives.
 * movementStartEmitted is true only between a sent MM,EV:START and its
 * matching MM,EV:END -- gates EV:END so a pre-start rejection (BUSY/
 * BAD_CMD/IMU_NOT_READY/PID_NOT_SET fired before any movement began)
 * never produces a spurious EV:END. */
static uint32_t lastTelemetryRunSequence;
static MotionType lastKnownDirection;
static uint32_t lastKnownStartTick;
static bool movementStartEmitted;

static void AppComm_IncrementRxCounter(
    volatile uint32_t *counter
)
{
    if ((counter != NULL) && (*counter < UINT32_MAX))
    {
        (*counter)++;
    }
}

static void AppComm_MarkRxDesynchronized(void)
{
    if (uart3RxRecoveryState ==
        APP_COMM_RX_SYNCHRONIZED)
    {
        uart3RxBoundarySeen = 0U;
        uart3RxRecoveryState =
            APP_COMM_RX_RESET_PENDING;
    }
}

static void AppComm_ClearUartErrorFlags(
    UART_HandleTypeDef *huart
)
{
    if ((huart != NULL) &&
        ((READ_REG(huart->Instance->SR) &
          APP_COMM_UART_STATUS_ERROR_MASK) != 0U))
    {
        /* STM32F4 clears PE/FE/NE/ORE with an SR read then a DR read. */
        __HAL_UART_CLEAR_PEFLAG(huart);
    }
}

static void AppComm_ServiceRxRecovery(void)
{
    HAL_StatusTypeDef abortStatus;
    HAL_StatusTypeDef armStatus;

    if ((uart3RxRecoveryState !=
         APP_COMM_RX_RESET_PENDING) &&
        (uart3RxRecoveryRequested == 0U))
    {
        return;
    }

    /* USART3 is at the maximum RTOS syscall priority, so this short
     * task critical section also excludes its queue-producing ISR. */
    taskENTER_CRITICAL();

    if (uart3RxRecoveryState ==
        APP_COMM_RX_RESET_PENDING)
    {
        if (osMessageQueueReset(CommRxQueueHandle) == osOK)
        {
            vision_rx_init(&visionRx);
            uart3RxRecoveryState =
                (uart3RxBoundarySeen != 0U) ?
                    APP_COMM_RX_SYNCHRONIZED :
                    APP_COMM_RX_DISCARD_TO_NEWLINE;
            uart3RxBoundarySeen = 0U;
        }
    }

    if (uart3RxRecoveryRequested != 0U)
    {
        abortStatus = HAL_UART_AbortReceive(&huart3);
        AppComm_ClearUartErrorFlags(&huart3);

        if (abortStatus == HAL_OK)
        {
            armStatus = HAL_UART_Receive_IT(
                &huart3,
                &uart3RxByte,
                1U
            );
        }
        else
        {
            armStatus = HAL_ERROR;
        }

        if (armStatus == HAL_OK)
        {
            uart3RxRecoveryRequested = 0U;
        }
        else
        {
            AppComm_MarkRxDesynchronized();
            uart3RxRecoveryRequested = 1U;
        }
    }

    taskEXIT_CRITICAL();
}

static void AppComm_Send(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    (void)HAL_UART_Transmit(
        &huart3,
        (uint8_t *)text,
        (uint16_t)strlen(text),
        APP_COMM_TX_TIMEOUT_MS
    );
}

static int32_t AppComm_ToTenths(float value)
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

static void AppComm_FormatGain(
    float gain,
    char *text,
    size_t textSize
)
{
    unsigned long whole;
    unsigned long fraction;

    if ((text == NULL) || (textSize == 0U))
    {
        return;
    }

    if (!(gain >= 0.0f) ||
        (gain > VISION_PID_GAIN_MAX))
    {
        (void)snprintf(text, textSize, "ERR");
        return;
    }

    whole = (unsigned long)gain;
    fraction = (unsigned long)(
        ((gain - (float)whole) * APP_COMM_GAIN_SCALE) + 0.5f
    );

    if (fraction >= (unsigned long)APP_COMM_GAIN_SCALE)
    {
        whole++;
        fraction = 0UL;
    }

    (void)snprintf(
        text,
        textSize,
        "%lu.%0*lu",
        whole,
        APP_COMM_GAIN_SCALE_DIGITS,
        fraction
    );
}

static void AppComm_SendPIDGains(
    char target,
    AppPIDTarget pidTarget
)
{
    float kp;
    float ki;
    float kd;
    char kpText[24];
    char kiText[24];
    char kdText[24];
    char response[96];

    AppMotion_GetPIDGains(
        pidTarget,
        &kp,
        &ki,
        &kd
    );

    AppComm_FormatGain(kp, kpText, sizeof(kpText));
    AppComm_FormatGain(ki, kiText, sizeof(kiText));
    AppComm_FormatGain(kd, kdText, sizeof(kdText));

    (void)snprintf(
        response,
        sizeof(response),
        "PID,%c,%s,%s,%s\n",
        target,
        kpText,
        kiText,
        kdText
    );

    AppComm_Send(response);
}

/* Stage A1: idle/busy predicate, identical in meaning to the inline
 * check AppComm_QueueMotionCommand() uses -- a motion command is
 * reserved, the state machine is non-idle (including braking), or a
 * command is sitting in MotionQueueHandle not yet dequeued. Used to gate
 * WSYNC,SET so wheel-sync parameters cannot mutate mid-command. Kept as
 * a separate helper so the proven MOV enqueue path stays byte-for-byte
 * unchanged. */
static bool AppComm_MotionBusy(void)
{
    return (stateCommandReserved != 0U) ||
           AppMotion_IsBusy() ||
           (osMessageQueueGetCount(MotionQueueHandle) != 0U);
}

static bool AppComm_QueueMotionCommand(
    MotionType type,
    int32_t value
)
{
    MotionCommand command;
    osStatus_t status;

    if ((stateCommandReserved != 0U) ||
        AppMotion_IsBusy() ||
        (osMessageQueueGetCount(MotionQueueHandle) != 0U))
    {
        AppComm_Send("ERR,BUSY\n");
        return false;
    }

    command.type = type;
    command.value = value;

    status = osMessageQueuePut(
        MotionQueueHandle,
        &command,
        0U,
        0U
    );

    if (status != osOK)
    {
        AppComm_Send("ERR,BUSY\n");
        return false;
    }

    stateCommandReserved = 1U;
    return true;
}

static char AppComm_TuneModeCharacter(
    AppMotionTuneMode mode
)
{
    switch (mode)
    {
        case APP_TUNE_LEFT:
            return 'L';

        case APP_TUNE_RIGHT:
            return 'R';

        case APP_TUNE_BOTH:
            return 'B';

        case APP_TUNE_STEERING:
            return 'S';

        default:
            return '\0';
    }
}

static void AppComm_SendTuneResult(
    AppMotionTuneMode mode,
    AppMotionTuneResult result
)
{
    char response[20];
    char modeCharacter =
        AppComm_TuneModeCharacter(mode);
    const char *resultText;
    int length;

    if (modeCharacter == '\0')
    {
        return;
    }

    switch (result)
    {
        case APP_TUNE_RESULT_DONE:
            resultText = "DONE";
            break;

        case APP_TUNE_RESULT_TIMEOUT:
            resultText = "TIMEOUT";
            break;

        default:
            return;
    }

    length = snprintf(
        response,
        sizeof(response),
        "TUNE,%c,%s\n",
        modeCharacter,
        resultText
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

static void AppComm_SendMotionEvent(
    const AppMotionEvent *event
)
{
    char response[96];
    int length = -1;

    if (event == NULL)
    {
        return;
    }

    switch (event->type)
    {
        case APP_MOTION_EVENT_MOV_DONE:
            AppComm_Send("ACK\n");
            return;

        case APP_MOTION_EVENT_ENCODER_RESET_OK:
            AppComm_Send("ENC,RST,OK\n");
            return;

        case APP_MOTION_EVENT_PWM_DONE:
            length = snprintf(
                response,
                sizeof(response),
                "PWM,DONE,D:%ld,L:%ld,R:%ld\n",
                (long)event->value,
                (long)event->left,
                (long)event->right
            );
            break;

        case APP_MOTION_EVENT_SPEED_DONE:
            /* L/R are signed centimetres per second, scaled by 10. */
            length = snprintf(
                response,
                sizeof(response),
                "SPD,D:%ld,L:%ld,R:%ld\n",
                (long)event->value,
                (long)event->left,
                (long)event->right
            );
            break;

        case APP_MOTION_EVENT_SERVO_SET:
            length = snprintf(
                response,
                sizeof(response),
                "SERVO,SET,%ld\n",
                (long)event->value
            );
            break;

        case APP_MOTION_EVENT_IMU_ZERO_OK:
            AppComm_Send("IMU,ZERO,OK\n");
            return;

        case APP_MOTION_EVENT_IMU_ZERO_ERROR:
            AppComm_Send("ERR,IMU_ZERO\n");
            return;

        case APP_MOTION_EVENT_IMU_FLIP_OK:
            AppComm_Send("IMU,FLIP,OK\n");
            return;

        case APP_MOTION_EVENT_IMU_FLIP_ERROR:
            AppComm_Send("ERR,IMU_FLIP\n");
            return;

        case APP_MOTION_EVENT_IMU_NOT_READY:
            AppComm_Send("ERR,IMU_NOT_READY\n");
            return;

        case APP_MOTION_EVENT_MOTION_TIMEOUT:
            AppComm_Send("ERR,MOTION_TIMEOUT\n");
            return;

        case APP_MOTION_EVENT_PID_NOT_SET:
            AppComm_Send("ERR,PID_NOT_SET\n");
            return;

        case APP_MOTION_EVENT_HEADING_RAW_DONE:
            length = snprintf(
                response,
                sizeof(response),
                "HDG,RAW,DONE,Y:%ld\n",
                (long)event->headingX10
            );
            break;

        case APP_MOTION_EVENT_HEADING_DONE:
        {
            char direction =
                (event->command == MOTION_HEADING_BACKWARD) ?
                    'B' : 'F';

            length = snprintf(
                response,
                sizeof(response),
                "HDG,DONE,D:%c,Y:%ld\n",
                direction,
                (long)event->headingX10
            );
            break;
        }

        case APP_MOTION_EVENT_BUSY:
            AppComm_Send("ERR,BUSY\n");
            return;

        case APP_MOTION_EVENT_BAD_COMMAND:
            AppComm_Send("ERR,BAD_CMD\n");
            return;

        default:
            return;
    }

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

/* Unified movement telemetry (MM,/EV:) formatting helpers. Pure
 * outbound formatting -- read tuneTelemetry/AppMotionEvent snapshots
 * already handed to app_comm.c by the existing mailboxes, never touch
 * any control state. */

static const char *AppComm_MovementDirectionString(
    MotionType direction
)
{
    switch (direction)
    {
        case MOTION_FORWARD:        return "F";
        case MOTION_BACKWARD:       return "B";
        case MOTION_FORWARD_LEFT:   return "FL";
        case MOTION_FORWARD_RIGHT:  return "FR";
        case MOTION_BACKWARD_LEFT:  return "BL";
        case MOTION_BACKWARD_RIGHT: return "BR";
        default:                    return "?";
    }
}

/* Periodic MM, snapshot -- sent for both APP_TUNE_MOV_DISTANCE (in
 * addition to the existing MD,/MS,/HS,/RTG,/MP,/SPC,/STP,/SFF,
 * cascade, which this does not touch) and APP_TUNE_MOV_TURN (as the
 * only periodic line, since none of the other controllers run during
 * a turn). LP/RP are RAW SIGNED displacement from command start (see
 * app_motion.h's doc comment on leftDisplacementCounts) -- a different
 * quantity from MD's own LP/RP (absolute position), which this does
 * not touch. */
static void AppComm_SendMovementSummary(
    const AppMotionTelemetry *telemetry
)
{
    const char *state;
    int length;

    if (telemetry->movementBraking != 0U)
    {
        state = "BRAKE";
    }
    else if (telemetry->movementPreSteering != 0U)
    {
        state = "PRE";
    }
    else if (telemetry->movementRecoveryActive != 0U)
    {
        state = "RECOVERY";
    }
    else if (telemetry->mode == APP_TUNE_MOV_TURN)
    {
        state = "TURN";
    }
    else
    {
        state = "DIST";
    }

    length = snprintf(
        movementSummaryResponse,
        sizeof(movementSummaryResponse),
        "MM,D:%s,T:%lu,ST:%s,TH:%ld,H:%ld,HE:%ld,GZ:%ld,SP:%lu,"
        "LT:%ld,LP:%ld,LE:%ld,LO:%ld,RT:%ld,RP:%ld,RE:%ld,RO:%ld\n",
        AppComm_MovementDirectionString(telemetry->activeDirection),
        (unsigned long)telemetry->elapsedMs,
        state,
        (long)telemetry->headingTargetX10,
        (long)telemetry->headingCurrentX10,
        (long)telemetry->headingErrorX10,
        (long)telemetry->headingYawRateX10,
        (unsigned long)telemetry->servoPulseUs,
        (long)telemetry->leftTargetCounts,
        (long)telemetry->leftDisplacementCounts,
        (long)telemetry->leftErrorCounts,
        (long)telemetry->leftOutput,
        (long)telemetry->rightTargetCounts,
        (long)telemetry->rightDisplacementCounts,
        (long)telemetry->rightErrorCounts,
        (long)telemetry->rightOutput
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(movementSummaryResponse)))
    {
        AppComm_Send(movementSummaryResponse);
    }
}

/* AK, line -- Ackermann-v2 rear-wheel target-rate control, turn-only.
 * Sent immediately after MM, (same buffer, reused sequentially, same
 * established pattern MD,/MS,/HS,/etc. and MM, itself already use).
 * E/K/B are static config, fetched directly via AppMotion_GetAckermann()/
 * AppMotion_GetTurnSpeed() rather than threaded through the telemetry
 * mailbox -- safe to read this way because ACKERMANN,/TURNSPEED, SET
 * are both idle-only/busy-gated (on_ackermann_set()/on_turn_speed_set()),
 * so neither can change while a turn's PRE_STEER/TURN telemetry is
 * active. LR/RR are the ACTUAL rates MotionTask selected/used this
 * tick (telemetry->ackermannLeft/RightRateCountsPerSec, already
 * rounded in app_motion.c) -- both read 0 during APP_MOTION_PRE_STEER,
 * never independently recomputed here. K is scaled x10000 and rounded
 * (k is always >=0, so plain +0.5 rounding is exact) -- a different
 * representation from ACKERMANN,GET's AppComm_FormatGain() 6-decimal
 * string, deliberately: this is the same integer-scaled-for-hot-path
 * convention the rest of this file's periodic telemetry already uses
 * (e.g. headingTargetX10), not a new one invented for this line. */
static void AppComm_SendAckermannSummary(
    const AppMotionTelemetry *telemetry
)
{
    float k;
    bool enabled;
    uint32_t baseRate;
    int length;

    AppMotion_GetAckermann(&k, &enabled);
    AppMotion_GetTurnSpeed(&baseRate);

    length = snprintf(
        movementSummaryResponse,
        sizeof(movementSummaryResponse),
        "AK,D:%s,E:%u,K:%ld,B:%lu,LR:%ld,RR:%ld\n",
        AppComm_MovementDirectionString(telemetry->activeDirection),
        enabled ? 1U : 0U,
        (long)((k * 10000.0f) + 0.5f),
        (unsigned long)baseRate,
        (long)telemetry->ackermannLeftRateCountsPerSec,
        (long)telemetry->ackermannRightRateCountsPerSec
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(movementSummaryResponse)))
    {
        AppComm_Send(movementSummaryResponse);
    }
}

/* MM,EV:START -- derived entirely from tuneTelemetry.runSequence
 * changing (see the call site in AppComm_ServiceTelemetry()), never
 * from AppMotion_PublishEvent()/AppMotionEventType, which this stage
 * does not touch. */
static void AppComm_SendMovementStartMarker(
    const AppMotionTelemetry *telemetry
)
{
    int length = snprintf(
        movementSummaryResponse,
        sizeof(movementSummaryResponse),
        "MM,EV:START,D:%s,T:0,V:%ld\n",
        AppComm_MovementDirectionString(telemetry->activeDirection),
        (long)telemetry->commandValue
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(movementSummaryResponse)))
    {
        AppComm_Send(movementSummaryResponse);
    }
}

/* MM,EV:END -- sent alongside (never instead of) the existing,
 * unmodified terminal ACK/ERR dispatch in AppComm_SendMotionEvent().
 * Gated on movementStartEmitted so a pre-start rejection (BUSY/
 * BAD_CMD/IMU_NOT_READY/PID_NOT_SET fired before any MOV,
 * F/B/FL/FR/BL/BR command actually started, i.e. no matching
 * MM,EV:START was ever sent) never produces a spurious EV:END; only
 * the four event types that can actually terminate an in-progress
 * movement are mapped, everything else (PWM_DONE, SERVO_SET,
 * IMU_ZERO_OK, HDG_DONE, ...) is ignored here without touching
 * movementStartEmitted, since a real movement (if any) is still
 * pending its own END. */
static void AppComm_SendMovementEndMarker(
    const AppMotionEvent *event
)
{
    const char *result;
    uint32_t nowTick;
    uint32_t elapsedMs;
    int length;

    if (!movementStartEmitted)
    {
        return;
    }

    switch (event->type)
    {
        case APP_MOTION_EVENT_MOV_DONE:       result = "OK";            break;
        case APP_MOTION_EVENT_MOTION_TIMEOUT: result = "TIMEOUT";       break;
        case APP_MOTION_EVENT_IMU_NOT_READY:  result = "IMU_NOT_READY"; break;
        case APP_MOTION_EVENT_PID_NOT_SET:    result = "PID_NOT_SET";   break;
        default:
            return;
    }

    nowTick = xTaskGetTickCount();
    elapsedMs = nowTick - lastKnownStartTick;

    length = snprintf(
        movementSummaryResponse,
        sizeof(movementSummaryResponse),
        "MM,EV:END,D:%s,T:%lu,RESULT:%s\n",
        AppComm_MovementDirectionString(lastKnownDirection),
        (unsigned long)elapsedMs,
        result
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(movementSummaryResponse)))
    {
        AppComm_Send(movementSummaryResponse);
    }

    movementStartEmitted = false;
}

/* TB, line -- TURNBRAKE one-shot diagnostic snapshot. Sent exactly once
 * per predictive-brake trigger, from the retained mailbox (see
 * AppMotionTurnBrakeSnapshot's own doc comment, app_motion.h) -- never
 * part of the periodic MM,/AK, cascade, and does not touch either of
 * their format strings. TH/H/REM/GZ/PRED are tenths of a degree (or
 * degrees/second for GZ), same scaled-integer convention as MM,'s own
 * TH:/H:/HE:/GZ:. LA is tenths of a millisecond (configuredLookaheadMsX10
 * already preserves turnBrakeLookaheadMs's own fractional precision --
 * see that field's own comment). CONF is the confirmation-cycle count
 * that actually fired the trigger (always TURN_BRAKE_CONFIRM_CYCLES by
 * construction today, reported rather than assumed constant). Guarded
 * on `snapshot->valid` defensively, even though the caller only invokes
 * this on a genuine sequence change (which never happens before the
 * first real trigger, since AppMotion_Init() explicitly zeroes the
 * mailbox's sequence). */
static void AppComm_SendTurnBrakeSnapshot(
    const AppMotionTurnBrakeSnapshot *snapshot
)
{
    int length;

    if (snapshot->valid == 0U)
    {
        return;
    }

    length = snprintf(
        movementSummaryResponse,
        sizeof(movementSummaryResponse),
        "TB,D:%s,TH:%ld,H:%ld,REM:%ld,GZ:%ld,PRED:%ld,LA:%ld,CONF:%u\n",
        AppComm_MovementDirectionString(snapshot->direction),
        (long)snapshot->targetHeadingX10,
        (long)snapshot->currentHeadingX10,
        (long)snapshot->remainingAngleX10,
        (long)snapshot->gyroZDpsX10,
        (long)snapshot->predictedStoppingAngleX10,
        (long)snapshot->configuredLookaheadMsX10,
        snapshot->confirmationCycles
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(movementSummaryResponse)))
    {
        AppComm_Send(movementSummaryResponse);
    }
}

static void AppComm_SendTuneTelemetry(
    const AppMotionTelemetry *telemetry
)
{
    int length = -1;

    if (telemetry == NULL)
    {
        return;
    }

    switch (telemetry->mode)
    {
        case APP_TUNE_LEFT:
            length = snprintf(
                tuneTelemetryResponse,
                sizeof(tuneTelemetryResponse),
                "TL,T:%ld,P:%ld,E:%ld,O:%ld\n",
                (long)telemetry->leftTargetCounts,
                (long)telemetry->leftPositionCounts,
                (long)telemetry->leftErrorCounts,
                (long)telemetry->leftOutput
            );
            break;

        case APP_TUNE_RIGHT:
            length = snprintf(
                tuneTelemetryResponse,
                sizeof(tuneTelemetryResponse),
                "TR,T:%ld,P:%ld,E:%ld,O:%ld\n",
                (long)telemetry->rightTargetCounts,
                (long)telemetry->rightPositionCounts,
                (long)telemetry->rightErrorCounts,
                (long)telemetry->rightOutput
            );
            break;

        case APP_TUNE_BOTH:
            length = snprintf(
                tuneTelemetryResponse,
                sizeof(tuneTelemetryResponse),
                "TB,LT:%ld,LP:%ld,LE:%ld,LO:%ld,"
                "RT:%ld,RP:%ld,RE:%ld,RO:%ld\n",
                (long)telemetry->leftTargetCounts,
                (long)telemetry->leftPositionCounts,
                (long)telemetry->leftErrorCounts,
                (long)telemetry->leftOutput,
                (long)telemetry->rightTargetCounts,
                (long)telemetry->rightPositionCounts,
                (long)telemetry->rightErrorCounts,
                (long)telemetry->rightOutput
            );
            break;

        case APP_TUNE_STEERING:
            length = snprintf(
                tuneTelemetryResponse,
                sizeof(tuneTelemetryResponse),
                "TS,H:%ld,Y:%ld,E:%ld,O:%ld,S:%lu\n",
                (long)telemetry->headingTargetX10,
                (long)telemetry->headingCurrentX10,
                (long)telemetry->headingErrorX10,
                (long)telemetry->steeringCorrectionUs,
                (unsigned long)telemetry->servoPulseUs
            );
            break;

        case APP_TUNE_MOV_TURN:
            /* Telemetry-only: no MS,/HS,/RTG,/MP,/SPC,/STP,/SFF,
             * equivalent for turns -- those controllers don't run
             * during APP_MOTION_TURN. MM, plus the turn-only AK,
             * (Ackermann-v2 rear-wheel target-rate) line are the only
             * periodic lines for turns. */
            AppComm_SendMovementSummary(telemetry);
            AppComm_SendAckermannSummary(telemetry);
            return;

        case APP_TUNE_MOV_DISTANCE:
            /* Unified movement telemetry: MM, first, then the existing
             * MD,/MS,/HS,/RTG,/MP,/SPC,/STP,/SFF, cascade below,
             * completely unchanged. */
            AppComm_SendMovementSummary(telemetry);

            length = snprintf(
                tuneTelemetryResponse,
                sizeof(tuneTelemetryResponse),
                "MD,LT:%ld,LP:%ld,LE:%ld,LO:%ld,"
                "RT:%ld,RP:%ld,RE:%ld,RO:%ld,"
                "H:%ld,Y:%ld,HE:%ld,S:%lu\n",
                (long)telemetry->leftTargetCounts,
                (long)telemetry->leftPositionCounts,
                (long)telemetry->leftErrorCounts,
                (long)telemetry->leftOutput,
                (long)telemetry->rightTargetCounts,
                (long)telemetry->rightPositionCounts,
                (long)telemetry->rightErrorCounts,
                (long)telemetry->rightOutput,
                (long)telemetry->headingTargetX10,
                (long)telemetry->headingCurrentX10,
                (long)telemetry->headingErrorX10,
                (unsigned long)telemetry->servoPulseUs
            );

            /* MD, format is unchanged; send it now, then fall through to
             * emit the Stage A1 MS, line as a second frame using the
             * same buffer (keeps MD, byte-identical to the baseline). */
            if ((length > 0) &&
                ((size_t)length < sizeof(tuneTelemetryResponse)))
            {
                AppComm_Send(tuneTelemetryResponse);
            }

            length = snprintf(
                tuneTelemetryResponse,
                sizeof(tuneTelemetryResponse),
                "MS,LPR:%ld,RPR:%ld,SE:%ld,SC:%ld\n",
                (long)telemetry->distanceLeftProgressCounts,
                (long)telemetry->distanceRightProgressCounts,
                (long)telemetry->wheelSyncErrorCounts,
                (long)telemetry->wheelSyncCorrectionDuty
            );

            /* MS, format is unchanged; send it now, then fall through to
             * emit the Stage A2 HS, line as a third frame using the same
             * buffer (keeps MD, and MS, byte-identical to the baseline). */
            if ((length > 0) &&
                ((size_t)length < sizeof(tuneTelemetryResponse)))
            {
                AppComm_Send(tuneTelemetryResponse);
            }

            length = snprintf(
                tuneTelemetryResponse,
                sizeof(tuneTelemetryResponse),
                "HS,HE:%ld,GZ:%ld,BC:%ld,RC:%ld,SO:%ld,SP:%lu\n",
                (long)telemetry->headingErrorX10,
                (long)telemetry->headingYawRateX10,
                (long)telemetry->steeringCorrectionUs,
                (long)telemetry->headingRateCorrectionUs,
                (long)telemetry->headingFinalCorrectionUs,
                (unsigned long)telemetry->servoPulseUs
            );

            /* HS, format is unchanged; send it now, then fall through to
             * emit the Stage A3.1 RTG, line as a fourth frame using the
             * same buffer (keeps MD,/MS,/HS, byte-identical to the
             * baseline). */
            if ((length > 0) &&
                ((size_t)length < sizeof(tuneTelemetryResponse)))
            {
                AppComm_Send(tuneTelemetryResponse);
            }

            length = snprintf(
                tuneTelemetryResponse,
                sizeof(tuneTelemetryResponse),
                "RTG,LG:%u,RG:%u,LR:%ld,RR:%ld,TC:%u,SC:%ld\n",
                (unsigned int)telemetry->leftGuardActive,
                (unsigned int)telemetry->rightGuardActive,
                (long)telemetry->leftPreGuardCommand,
                (long)telemetry->rightPreGuardCommand,
                (unsigned int)telemetry->targetCrossedFlag,
                (long)telemetry->settledCycleCount
            );

            /* RTG, format is unchanged; send it now, then fall through to
             * emit the Stage A3.2 MP, line as a fifth frame using the
             * same buffer (keeps MD,/MS,/HS,/RTG, byte-identical to the
             * baseline). */
            if ((length > 0) &&
                ((size_t)length < sizeof(tuneTelemetryResponse)))
            {
                AppComm_Send(tuneTelemetryResponse);
            }

            length = snprintf(
                tuneTelemetryResponse,
                sizeof(tuneTelemetryResponse),
                "MP,PA:%u,PD:%ld,LPO:%ld,RPO:%ld,LV:%ld,RV:%ld,"
                "LF:%u,RF:%u,LDR:%ld,RDR:%ld\n",
                (unsigned int)telemetry->decelProfileActiveFlag,
                (long)telemetry->decelProfileDuty,
                (long)telemetry->leftRawPidOutput,
                (long)telemetry->rightRawPidOutput,
                (long)telemetry->leftVelocityCountsPerSec,
                (long)telemetry->rightVelocityCountsPerSec,
                (unsigned int)telemetry->leftDecelEnforcedFlag,
                (unsigned int)telemetry->rightDecelEnforcedFlag,
                (long)telemetry->leftDirectedRemainingCounts,
                (long)telemetry->rightDirectedRemainingCounts
            );

            /* MP, format is unchanged; send it now, then fall through to
             * emit the Stage A3.3a SPC, line as a sixth frame using the
             * same buffer (keeps MD,/MS,/HS,/RTG,/MP, byte-identical to
             * the baseline). */
            if ((length > 0) &&
                ((size_t)length < sizeof(tuneTelemetryResponse)))
            {
                AppComm_Send(tuneTelemetryResponse);
            }

            length = snprintf(
                tuneTelemetryResponse,
                sizeof(tuneTelemetryResponse),
                "SPC,E:%u,TSP:%ld,DSP:%ld,ASP:%ld,RSP:%ld,SE:%ld,"
                "PC:%ld,IC:%ld,RB:%ld,FB:%ld,REM:%ld\n",
                (unsigned int)telemetry->speedControlActiveFlag,
                (long)telemetry->speedControlTargetSpeed,
                (long)telemetry->speedControlDynamicTargetSpeed,
                (long)telemetry->speedControlActualSpeed,
                (long)telemetry->speedControlFilteredSpeed,
                (long)telemetry->speedControlErrorSpeed,
                (long)telemetry->speedControlProportionalContribution,
                (long)telemetry->speedControlIntegralContribution,
                (long)telemetry->speedControlRawBaseDuty,
                (long)telemetry->speedControlFinalBaseDuty,
                (long)telemetry->speedControlChassisRemainingCounts
            );

            /* SPC, format is unchanged; send it now, then fall through to
             * emit the Stage A3.3c STP, line as a seventh frame using the
             * same buffer (keeps MD,/MS,/HS,/RTG,/MP,/SPC, byte-identical
             * to the baseline). */
            if ((length > 0) &&
                ((size_t)length < sizeof(tuneTelemetryResponse)))
            {
                AppComm_Send(tuneTelemetryResponse);
            }

            length = snprintf(
                tuneTelemetryResponse,
                sizeof(tuneTelemetryResponse),
                "STP,E:%u,TR:%u\n",
                (unsigned int)telemetry->stopControlEnabledFlag,
                (unsigned int)telemetry->stopControlTriggeredFlag
            );

            /* STP, format is unchanged; send it now, then fall through to
             * emit the Stage SFF1 SFF, line as a new trailing frame using
             * the same buffer (keeps MD,/MS,/HS,/RTG,/MP,/SPC,/STP,
             * byte-identical to the baseline). */
            if ((length > 0) &&
                ((size_t)length < sizeof(tuneTelemetryResponse)))
            {
                AppComm_Send(tuneTelemetryResponse);
            }

            length = snprintf(
                tuneTelemetryResponse,
                sizeof(tuneTelemetryResponse),
                "SFF,D:%c,E:%u,RSP:%ld,FF:%ld\n",
                (char)telemetry->sffDirection,
                (unsigned int)telemetry->sffEnabledFlag,
                (long)telemetry->sffFilteredSpeed,
                (long)telemetry->sffOutputUs
            );
            break;

        default:
            return;
    }

    if ((length > 0) &&
        ((size_t)length < sizeof(tuneTelemetryResponse)))
    {
        AppComm_Send(tuneTelemetryResponse);
    }
}

/* Start parser + first 1-byte UART interrupt reception. */
void AppComm_Init(void)
{
    vision_rx_init(&visionRx);

    lastTelemetryServiceTick =
        osKernelGetTickCount();
    lastTelemetryTick = 0U;
    lastCompletionSequence = 0U;
    lastMotionEventSequence = 0U;
    lastTurnBrakeSnapshotSequence = 0U;
    lastTelemetryRunSequence = 0U;
    lastKnownDirection = MOTION_STOP;
    lastKnownStartTick = 0U;
    movementStartEmitted = false;
    stateCommandReserved = 0U;
    uart3RxRecoveryState =
        APP_COMM_RX_SYNCHRONIZED;
    uart3RxRecoveryRequested = 0U;
    uart3RxBoundarySeen = 0U;
    uart3RxErrorCount = 0U;
    uart3RxDroppedCount = 0U;

    if (HAL_UART_Receive_IT(
            &huart3,
            &uart3RxByte,
            1U) != HAL_OK)
    {
        AppComm_MarkRxDesynchronized();
        uart3RxRecoveryRequested = 1U;
    }
}

void AppComm_ProcessByte(uint8_t byte)
{
    if (uart3RxRecoveryState ==
        APP_COMM_RX_SYNCHRONIZED)
    {
        vision_rx_feed(&visionRx, byte);
    }
}

void AppComm_GetRxStatus(AppCommRxStatus *status)
{
    if (status == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();

    status->errorCount = uart3RxErrorCount;
    status->droppedCount = uart3RxDroppedCount;
    status->recovering =
        ((uart3RxRecoveryState !=
          APP_COMM_RX_SYNCHRONIZED) ||
         (uart3RxRecoveryRequested != 0U)) ? 1U : 0U;

    taskEXIT_CRITICAL();
}

void AppComm_ServiceTelemetry(void)
{
    AppMotionTelemetry telemetry;
    AppMotionEvent motionEvent;
    uint32_t now;
    uint32_t servicePeriodTicks;
    uint32_t periodTicks;

    /* RX recovery is task-owned and must not be telemetry-rate gated. */
    AppComm_ServiceRxRecovery();

    now = osKernelGetTickCount();
    servicePeriodTicks =
        osKernelGetTickFreq() /
        APP_COMM_SERVICE_RATE_HZ;

    if (servicePeriodTicks == 0U)
    {
        servicePeriodTicks = 1U;
    }

    if ((now - lastTelemetryServiceTick) <
        servicePeriodTicks)
    {
        return;
    }

    lastTelemetryServiceTick = now;

    /* TURNBRAKE: independent one-shot diagnostic snapshot, polled the
     * same way the terminal motion event is (sequence-number diff), via
     * its own separate retained mailbox -- never touches or is touched
     * by the motionEvent/telemetry checks below. */
    {
        AppMotionTurnBrakeSnapshot turnBrakeSnapshot;

        AppMotion_GetTurnBrakeSnapshot(&turnBrakeSnapshot);

        if (turnBrakeSnapshot.sequence != lastTurnBrakeSnapshotSequence)
        {
            lastTurnBrakeSnapshotSequence = turnBrakeSnapshot.sequence;

            AppComm_SendTurnBrakeSnapshot(&turnBrakeSnapshot);
            return;
        }
    }

    AppMotion_GetEvent(&motionEvent);

    if (motionEvent.sequence !=
        lastMotionEventSequence)
    {
        lastMotionEventSequence =
            motionEvent.sequence;
        stateCommandReserved = 0U;

        /* Unified movement telemetry: MM,EV:END alongside (never in
         * place of) the existing, unmodified ACK/ERR dispatch below --
         * internally gated on movementStartEmitted, a no-op for any
         * event that isn't a real movement's terminal outcome. */
        AppComm_SendMovementEndMarker(&motionEvent);
        AppComm_SendMotionEvent(&motionEvent);
        return;
    }

    AppMotion_GetTelemetry(&telemetry);

    /* Unified movement telemetry: MM,EV:START, derived purely from
     * tuneTelemetry.runSequence changing while a MOV,F/B/FL/FR/BL/BR
     * command is newly active (APP_TUNE_MOV_DISTANCE/APP_TUNE_MOV_TURN
     * only -- TUNE_LEFT/RIGHT/BOTH/STEERING also bump runSequence but
     * are excluded here). lastTelemetryRunSequence always advances
     * below regardless, so a later genuine MOV arming is still
     * detected as a change even after an intervening TUNE_* run. */
    if ((telemetry.runSequence != lastTelemetryRunSequence) &&
        (telemetry.active != 0U) &&
        ((telemetry.mode == APP_TUNE_MOV_DISTANCE) ||
         (telemetry.mode == APP_TUNE_MOV_TURN)))
    {
        lastTelemetryRunSequence = telemetry.runSequence;
        lastKnownDirection = telemetry.activeDirection;
        lastKnownStartTick = telemetry.movementStartTick;
        movementStartEmitted = true;

        AppComm_SendMovementStartMarker(&telemetry);
        return;
    }

    lastTelemetryRunSequence = telemetry.runSequence;

    if (telemetry.completionSequence !=
        lastCompletionSequence)
    {
        lastCompletionSequence =
            telemetry.completionSequence;
        stateCommandReserved = 0U;

        AppComm_SendTuneResult(
            telemetry.completedMode,
            telemetry.completedResult
        );

        return;
    }

    if (telemetry.active == 0U)
    {
        return;
    }

    periodTicks =
        osKernelGetTickFreq() /
        APP_COMM_TELEMETRY_RATE_HZ;

    if (periodTicks == 0U)
    {
        periodTicks = 1U;
    }

    if ((now - lastTelemetryTick) < periodTicks)
    {
        return;
    }

    lastTelemetryTick = now;

    AppComm_SendTuneTelemetry(&telemetry);
}

/* Called by HAL when USART3 receives one byte. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART3))
    {
        if ((huart->ErrorCode &
             APP_COMM_UART_HAL_ERROR_MASK) != 0U)
        {
            /* Do not admit a byte carrying a hardware RX error. The HAL
             * error callback leaves re-arming to CommTask. */
            AppComm_IncrementRxCounter(
                &uart3RxDroppedCount
            );
            uart3RxBoundarySeen = 0U;
            AppComm_MarkRxDesynchronized();
            uart3RxRecoveryRequested = 1U;
            return;
        }

        if (uart3RxRecoveryState ==
            APP_COMM_RX_SYNCHRONIZED)
        {
            /* ISR -> RTOS byte queue. Timeout MUST be 0 inside ISR. */
            if (osMessageQueuePut(
                    CommRxQueueHandle,
                    &uart3RxByte,
                    0U,
                    0U) != osOK)
            {
                AppComm_IncrementRxCounter(
                    &uart3RxDroppedCount
                );
                AppComm_MarkRxDesynchronized();

                if (uart3RxByte == (uint8_t)'\n')
                {
                    uart3RxBoundarySeen = 1U;
                }
            }
        }
        else
        {
            AppComm_IncrementRxCounter(
                &uart3RxDroppedCount
            );

            if (uart3RxByte == (uint8_t)'\n')
            {
                if (uart3RxRecoveryState ==
                    APP_COMM_RX_DISCARD_TO_NEWLINE)
                {
                    /* The parser and queued prefix were already reset by
                     * CommTask; this physical boundary makes RX safe again. */
                    uart3RxBoundarySeen = 0U;
                    uart3RxRecoveryState =
                        APP_COMM_RX_SYNCHRONIZED;
                }
                else
                {
                    /* Preserve a boundary that arrives before CommTask can
                     * atomically purge the stale parser/queue prefix. */
                    uart3RxBoundarySeen = 1U;
                }
            }
            else if (uart3RxRecoveryState ==
                     APP_COMM_RX_RESET_PENDING)
            {
                /* A byte after the remembered newline starts a new
                 * partial frame, so recovery must wait for its boundary. */
                uart3RxBoundarySeen = 0U;
            }
        }

        /* Preserve the normal one-byte interrupt receive model. */
        if (HAL_UART_Receive_IT(
                &huart3,
                &uart3RxByte,
                1U) != HAL_OK)
        {
            AppComm_MarkRxDesynchronized();
            uart3RxRecoveryRequested = 1U;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART3))
    {
        AppComm_IncrementRxCounter(&uart3RxErrorCount);
        uart3RxBoundarySeen = 0U;
        AppComm_MarkRxDesynchronized();
        uart3RxRecoveryRequested = 1U;

        /* Clear any F4 receive-error flags the HAL has not already
         * consumed. CommTask owns abort and re-arm. */
        AppComm_ClearUartErrorFlags(huart);
    }
}

/* MOV,F/B/FL/FR/BL/BR callback from vision_rx.c */
void on_move(VisionMoveType type, uint16_t amount)
{
    MotionType motionType;

    switch (type)
    {
        case VISION_MOVE_FORWARD:
            motionType = MOTION_FORWARD;
            break;

        case VISION_MOVE_BACKWARD:
            motionType = MOTION_BACKWARD;
            break;

        case VISION_MOVE_FORWARD_LEFT:
            motionType = MOTION_FORWARD_LEFT;
            break;

        case VISION_MOVE_FORWARD_RIGHT:
            motionType = MOTION_FORWARD_RIGHT;
            break;

        case VISION_MOVE_BACKWARD_LEFT:
            motionType = MOTION_BACKWARD_LEFT;
            break;

        case VISION_MOVE_BACKWARD_RIGHT:
            motionType = MOTION_BACKWARD_RIGHT;
            break;

        default:
            AppComm_Send("ERR,BAD_CMD\n");
            return;
    }

    (void)AppComm_QueueMotionCommand(
        motionType,
        (int32_t)amount
    );
}

/* Vision callbacks — we'll use these later. */
void on_target(uint8_t id, float conf, uint16_t cx, uint16_t cy)
{
    (void)id;
    (void)conf;
    (void)cx;
    (void)cy;
}

void on_no_target(void)
{
}

void on_vision_ready(void)
{
}

void on_bad_frame(const char *line)
{
    (void)line;
    AppComm_Send("ERR,BAD_CMD\n");
}


void on_encoder_reset(void)
{
    (void)AppComm_QueueMotionCommand(
        MOTION_ENCODER_RESET,
        0
    );
}

void on_encoder_get(void)
{
    AppEncoderSnapshot snapshot =
        AppEncoder_GetSnapshot();
    char response[64];
    int length = snprintf(
        response,
        sizeof(response),
        "ENC,L:%ld,R:%ld\n",
        (long)snapshot.left,
        (long)snapshot.right
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

void on_pwm_test(uint8_t duty)
{
    (void)AppComm_QueueMotionCommand(
        MOTION_PWM_TEST,
        (int32_t)duty
    );
}

void on_speed_test(uint8_t duty)
{
    (void)AppComm_QueueMotionCommand(
        MOTION_SPEED_TEST,
        (int32_t)duty
    );
}

void on_servo_get(void)
{
    char response[32];
    int length = snprintf(
        response,
        sizeof(response),
        "SERVO,%lu\n",
        (unsigned long)AppServo_GetUs()
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

void on_servo_set(uint16_t pulse_us)
{
    (void)AppComm_QueueMotionCommand(
        MOTION_SERVO_SET,
        (int32_t)pulse_us
    );
}

void on_imu_get(void)
{
    AppIMUSnapshot snapshot;
    char response[64];
    int length;

    AppIMU_GetSnapshot(&snapshot);

    if (snapshot.ready == 0U)
    {
        AppComm_Send("ERR,IMU\n");
        return;
    }

    length = snprintf(
        response,
        sizeof(response),
        "IMU,Y:%ld,GZ:%ld,SIGN:%+d\n",
        (long)AppComm_ToTenths(snapshot.headingDeg),
        (long)AppComm_ToTenths(snapshot.gyroZDps),
        (int)snapshot.headingSign
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

void on_imu_zero(void)
{
    (void)AppComm_QueueMotionCommand(
        MOTION_IMU_ZERO,
        0
    );
}

void on_imu_flip(void)
{
    (void)AppComm_QueueMotionCommand(
        MOTION_IMU_FLIP,
        0
    );
}

void on_heading_raw_test(uint16_t seconds)
{
    (void)AppComm_QueueMotionCommand(
        MOTION_HEADING_RAW,
        (int32_t)seconds
    );
}

void on_heading_test(char dir, uint16_t seconds)
{
    MotionType type;

    switch (dir)
    {
        case 'F':
            type = MOTION_HEADING_FORWARD;
            break;

        case 'B':
            type = MOTION_HEADING_BACKWARD;
            break;

        default:
            AppComm_Send("ERR,BAD_CMD\n");
            return;
    }

    (void)AppComm_QueueMotionCommand(
        type,
        (int32_t)seconds
    );
}

void on_pid_set(
    char target,
    float kp,
    float ki,
    float kd
)
{
    AppPIDTarget pidTarget;
    char response[] = "PID,L,OK\n";

    switch (target)
    {
        case 'L':
            pidTarget = APP_PID_LEFT;
            break;

        case 'R':
            pidTarget = APP_PID_RIGHT;
            break;

        case 'S':
            pidTarget = APP_PID_STEERING;
            break;

        default:
            AppComm_Send("PID,ERR\n");
            return;
    }

    AppMotion_SetPIDGains(
        pidTarget,
        kp,
        ki,
        kd
    );

    response[4] = target;
    AppComm_Send(response);
}

void on_pid_get(void)
{
    AppComm_SendPIDGains('L', APP_PID_LEFT);
    AppComm_SendPIDGains('R', APP_PID_RIGHT);
    AppComm_SendPIDGains('S', APP_PID_STEERING);
}

/* Stage A1: rear-wheel sync SET is idle-only. Parser (vision_rx.c) has
 * already range-checked gain (0..1), maxDuty (0..39) and enable (0/1). */
void on_wheel_sync_set(float gain, float maxDuty, int enable)
{
    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    AppMotion_SetWheelSync(gain, maxDuty, (enable != 0));
    AppComm_Send("WSYNC,OK\n");
}

/* GET remains available while moving. */
void on_wheel_sync_get(void)
{
    float gain;
    float maxDuty;
    bool enable;
    char gainText[24];
    char maxDutyText[24];
    char response[80];
    int length;

    AppMotion_GetWheelSync(&gain, &maxDuty, &enable);

    AppComm_FormatGain(gain, gainText, sizeof(gainText));
    AppComm_FormatGain(maxDuty, maxDutyText, sizeof(maxDutyText));

    length = snprintf(
        response,
        sizeof(response),
        "WSYNC,G:%s,M:%s,E:%u\n",
        gainText,
        maxDutyText,
        enable ? 1U : 0U
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

/* Stage A2: heading rate-damping SET is idle-only. Parser (vision_rx.c)
 * has already range-checked krate (0..30), maxCorrection (0..300) and
 * enable (0/1). steeringPID's own Kp/Ki/Kd (PID,S) are untouched here. */
void on_heading_rate_set(float krate, float maxCorrection, int enable)
{
    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    AppMotion_SetHeadingRateDamping(krate, maxCorrection, (enable != 0));
    AppComm_Send("HCTRL,OK\n");
}

/* GET remains available while moving. */
void on_heading_rate_get(void)
{
    float krate;
    float maxCorrection;
    bool enable;
    char krateText[24];
    char maxCorrectionText[24];
    char response[80];
    int length;

    AppMotion_GetHeadingRateDamping(&krate, &maxCorrection, &enable);

    AppComm_FormatGain(krate, krateText, sizeof(krateText));
    AppComm_FormatGain(maxCorrection, maxCorrectionText, sizeof(maxCorrectionText));

    length = snprintf(
        response,
        sizeof(response),
        "HCTRL,K:%s,M:%s,E:%u\n",
        krateText,
        maxCorrectionText,
        enable ? 1U : 0U
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

/* Stage A3.1: reverse-torque guard SET is idle-only. Parser (vision_rx.c)
 * has already range-checked enable (0/1). Scoped strictly to the
 * existing per-wheel approach zone in app_motion.c -- WSYNC, HCTRL,
 * steeringPID and leftPID/rightPID gains are untouched by this command. */
void on_reverse_torque_guard_set(int enable)
{
    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    AppMotion_SetReverseTorqueGuard(enable != 0);
    AppComm_Send("RTGUARD,OK\n");
}

/* GET remains available while moving. */
void on_reverse_torque_guard_get(void)
{
    bool enable;
    char response[32];
    int length;

    AppMotion_GetReverseTorqueGuard(&enable);

    length = snprintf(
        response,
        sizeof(response),
        "RTGUARD,E:%u\n",
        enable ? 1U : 0U
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

/* Stage A3.2: deceleration-profile SET is idle-only. Parser (vision_rx.c)
 * has already range-checked decelCounts (300..3000), approachDuty
 * (40..90) and enable (0/1). leftPID/rightPID gains and WSYNC's own
 * algorithm/settings are untouched by this command. Mutually exclusive
 * with Stage A3.3a's SPCTRL -- rejected here if SPCTRL is already
 * enabled, both controllers own the same common base-duty magnitude and
 * can never run together. */
void on_decel_profile_set(float decelCounts, float approachDuty, int enable)
{
    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    if (enable != 0)
    {
        float spctrlTarget;
        float spctrlKp;
        float spctrlKi;
        bool  spctrlEnable;

        AppMotion_GetSpeedControl(&spctrlTarget, &spctrlKp, &spctrlKi, &spctrlEnable);

        if (spctrlEnable)
        {
            AppComm_Send("DPROF,ERR\n");
            return;
        }
    }

    AppMotion_SetDecelProfile(decelCounts, approachDuty, (enable != 0));
    AppComm_Send("DPROF,OK\n");
}

/* GET remains available while moving. */
void on_decel_profile_get(void)
{
    float decelCounts;
    float approachDuty;
    bool enable;
    char decelCountsText[24];
    char approachDutyText[24];
    char response[80];
    int length;

    AppMotion_GetDecelProfile(&decelCounts, &approachDuty, &enable);

    AppComm_FormatGain(decelCounts, decelCountsText, sizeof(decelCountsText));
    AppComm_FormatGain(approachDuty, approachDutyText, sizeof(approachDutyText));

    length = snprintf(
        response,
        sizeof(response),
        "DPROF,D:%s,A:%s,E:%u\n",
        decelCountsText,
        approachDutyText,
        enable ? 1U : 0U
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

/* Stage A3.3a: closed-loop chassis speed control SET is idle-only.
 * Parser (vision_rx.c) has already range-checked targetSpeed
 * (0..8000), Kp (0..1.0), Ki (0..1.0) and enable (0/1). Mutually
 * exclusive with Stage A3.2's DPROF -- rejected here if DPROF is
 * already enabled (symmetric with the check added to
 * on_decel_profile_set() above). leftPID/rightPID gains, WSYNC's own
 * algorithm/settings and steeringPID/HCTRL are untouched by this
 * command.
 *
 * Stage A3.3b: also rejected here if Stage A3.3b's SPPROF is currently
 * enabled and the incoming targetSpeed would fall below its configured
 * approachSpeed -- explicit rejection, never a silent profile change
 * (symmetric with the check added to on_speed_profile_set() below). */
void on_speed_control_set(float targetSpeed, float kp, float ki, int enable)
{
    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    if (enable != 0)
    {
        float decelCounts;
        float approachDuty;
        bool  decelEnable;

        AppMotion_GetDecelProfile(&decelCounts, &approachDuty, &decelEnable);

        if (decelEnable)
        {
            AppComm_Send("SPCTRL,ERR\n");
            return;
        }

        float sprofDecelStartCounts;
        float sprofApproachSpeed;
        bool  sprofEnable;

        AppMotion_GetSpeedProfile(&sprofDecelStartCounts, &sprofApproachSpeed, &sprofEnable);

        if (sprofEnable && (targetSpeed < sprofApproachSpeed))
        {
            AppComm_Send("SPCTRL,ERR\n");
            return;
        }
    }

    AppMotion_SetSpeedControl(targetSpeed, kp, ki, (enable != 0));
    AppComm_Send("SPCTRL,OK\n");
}

/* GET remains available while moving. */
void on_speed_control_get(void)
{
    float targetSpeed;
    float kp;
    float ki;
    bool enable;
    char targetSpeedText[24];
    char kpText[24];
    char kiText[24];
    char response[96];
    int length;

    AppMotion_GetSpeedControl(&targetSpeed, &kp, &ki, &enable);

    AppComm_FormatGain(targetSpeed, targetSpeedText, sizeof(targetSpeedText));
    AppComm_FormatGain(kp, kpText, sizeof(kpText));
    AppComm_FormatGain(ki, kiText, sizeof(kiText));

    length = snprintf(
        response,
        sizeof(response),
        "SPCTRL,T:%s,P:%s,I:%s,E:%u\n",
        targetSpeedText,
        kpText,
        kiText,
        enable ? 1U : 0U
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

/* Stage A3.3b: distance-based speed profile SET is idle-only. Parser
 * (vision_rx.c) has already range-checked decelStartCounts (300..6000),
 * approachSpeed (1..8000) and enable (0/1). approachSpeed must never
 * exceed the currently-configured SPCTRL cruise target -- rejected here,
 * checked only when enabling (disabling is always allowed), explicit
 * rejection rather than a silent clamp/rewrite (symmetric with the check
 * added to on_speed_control_set() above). Not gated on SPCTRL's current
 * enabled state: SPPROF has no magnitude of its own to fight over, so it
 * is safe to arm before SPCTRL is -- it simply has no effect until
 * SPCTRL is also enabled (app_motion.c's own runtime gate). */
void on_speed_profile_set(float decelStartCounts, float approachSpeed, int enable)
{
    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    if (enable != 0)
    {
        float cruiseTarget;
        float cruiseKp;
        float cruiseKi;
        bool  cruiseEnable;

        AppMotion_GetSpeedControl(&cruiseTarget, &cruiseKp, &cruiseKi, &cruiseEnable);

        if (approachSpeed > cruiseTarget)
        {
            AppComm_Send("SPPROF,ERR\n");
            return;
        }
    }

    AppMotion_SetSpeedProfile(decelStartCounts, approachSpeed, (enable != 0));
    AppComm_Send("SPPROF,OK\n");
}

/* GET remains available while moving. */
void on_speed_profile_get(void)
{
    float decelStartCounts;
    float approachSpeed;
    bool enable;
    char decelStartCountsText[24];
    char approachSpeedText[24];
    char response[80];
    int length;

    AppMotion_GetSpeedProfile(&decelStartCounts, &approachSpeed, &enable);

    AppComm_FormatGain(decelStartCounts, decelStartCountsText, sizeof(decelStartCountsText));
    AppComm_FormatGain(approachSpeed, approachSpeedText, sizeof(approachSpeedText));

    length = snprintf(
        response,
        sizeof(response),
        "SPPROF,D:%s,A:%s,E:%u\n",
        decelStartCountsText,
        approachSpeedText,
        enable ? 1U : 0U
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

/* Stage A3.3c: final controlled stop gate SET is idle-only. Parser
 * (vision_rx.c) has already range-checked brakeLeadCounts (20..500),
 * brakeEntrySpeed (500..2500) and enable (0/1). No cross-command
 * invariant check is needed: STOPCTRL owns no magnitude and its gate is
 * purely additive to the existing completion path (app_motion.c OR's it
 * alongside settledCycles/targetCrossed), so it cannot conflict with
 * DPROF or SPPROF. Not gated on SPCTRL's current enabled state, for the
 * same reason as SPPROF above -- safe to arm independently, simply has no
 * effect until SPCTRL is also enabled. */
void on_stop_control_set(float brakeLeadCounts, float brakeEntrySpeed, int enable)
{
    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    AppMotion_SetStopControl(brakeLeadCounts, brakeEntrySpeed, (enable != 0));
    AppComm_Send("STOPCTRL,OK\n");
}

/* GET remains available while moving. */
void on_stop_control_get(void)
{
    float brakeLeadCounts;
    float brakeEntrySpeed;
    bool enable;
    char brakeLeadCountsText[24];
    char brakeEntrySpeedText[24];
    char response[80];
    int length;

    AppMotion_GetStopControl(&brakeLeadCounts, &brakeEntrySpeed, &enable);

    AppComm_FormatGain(brakeLeadCounts, brakeLeadCountsText, sizeof(brakeLeadCountsText));
    AppComm_FormatGain(brakeEntrySpeed, brakeEntrySpeedText, sizeof(brakeEntrySpeedText));

    length = snprintf(
        response,
        sizeof(response),
        "STOPCTRL,L:%s,S:%s,E:%u\n",
        brakeLeadCountsText,
        brakeEntrySpeedText,
        enable ? 1U : 0U
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

/* PRE_STEER SET is idle-only. Parser (vision_rx.c) has already range-
 * checked durationMs (0..1000) and enable (0/1). No cross-command
 * invariant check is needed -- PRE_STEER owns no magnitude and only
 * affects MOV,FL/FR/BL/BR's own entry sequencing. */
void on_pre_steer_set(float durationMs, int enable)
{
    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    AppMotion_SetPreSteer(durationMs, (enable != 0));
    AppComm_Send("PRESTEER,OK\n");
}

/* GET remains available while moving. */
void on_pre_steer_get(void)
{
    float durationMs;
    bool enable;
    char durationMsText[24];
    char response[48];
    int length;

    AppMotion_GetPreSteer(&durationMs, &enable);

    AppComm_FormatGain(durationMs, durationMsText, sizeof(durationMsText));

    length = snprintf(
        response,
        sizeof(response),
        "PRESTEER,D:%s,E:%u\n",
        durationMsText,
        enable ? 1U : 0U
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

/* TURNBRAKE SET is idle-only. Parser (vision_rx.c) has already range-
 * checked lookaheadMs (0..500) and enable (0/1). No cross-command
 * invariant check is needed -- TURNBRAKE owns no magnitude and only
 * affects APP_MOTION_TURN's own predictive-brake trigger, additive to
 * the existing dwell completion. */
void on_turn_brake_set(float lookaheadMs, int enable)
{
    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    AppMotion_SetTurnBrake(lookaheadMs, (enable != 0));
    AppComm_Send("TURNBRAKE,OK\n");
}

/* GET remains available while moving. */
void on_turn_brake_get(void)
{
    float lookaheadMs;
    bool enable;
    char lookaheadMsText[24];
    char response[48];
    int length;

    AppMotion_GetTurnBrake(&lookaheadMs, &enable);

    AppComm_FormatGain(lookaheadMs, lookaheadMsText, sizeof(lookaheadMsText));

    length = snprintf(
        response,
        sizeof(response),
        "TURNBRAKE,L:%s,E:%u\n",
        lookaheadMsText,
        enable ? 1U : 0U
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

/* TURNSPEED SET is idle-only. Parser (vision_rx.c) has already range-
 * checked countsPerSec (500..2500) via vision_rx_parse_bounded_u32();
 * AppMotion_SetTurnSpeed() itself defensively re-clamps, see its own
 * doc comment. No cross-command invariant check is needed --
 * TURNSPEED owns no magnitude and only affects APP_MOTION_TURN's own
 * rear-wheel target-rate ramp. */
void on_turn_speed_set(uint32_t countsPerSec)
{
    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    AppMotion_SetTurnSpeed(countsPerSec);
    AppComm_Send("TURNSPEED,OK\n");
}

/* GET remains available while moving. */
void on_turn_speed_get(void)
{
    uint32_t countsPerSec;
    char response[32];
    int length;

    AppMotion_GetTurnSpeed(&countsPerSec);

    length = snprintf(
        response,
        sizeof(response),
        "TURNSPEED,R:%lu\n",
        (unsigned long)countsPerSec
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

/* Ackermann-v2 SET is idle-only. Parser (vision_rx.c) has already
 * range-checked k (0.0..0.5) and enable (0/1);
 * AppMotion_SetAckermann() itself defensively re-clamps, see its own
 * doc comment. No cross-command invariant check is needed --
 * Ackermann owns no magnitude and only affects APP_MOTION_TURN's own
 * rear-wheel target-rate ramp. */
void on_ackermann_set(float k, int enable)
{
    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    AppMotion_SetAckermann(k, (enable != 0));
    AppComm_Send("ACKERMANN,OK\n");
}

/* GET remains available while moving. */
void on_ackermann_get(void)
{
    float k;
    bool enable;
    char kText[24];
    char response[48];
    int length;

    AppMotion_GetAckermann(&k, &enable);

    AppComm_FormatGain(k, kText, sizeof(kText));

    length = snprintf(
        response,
        sizeof(response),
        "ACKERMANN,K:%s,E:%u\n",
        kText,
        enable ? 1U : 0U
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

/* Stage A4: distance (counts/cm) calibration SET is idle-only. Parser
 * (vision_rx.c) has already range-checked slopeCountsPerCm (50..100),
 * offsetCounts (-500..500) and enable (0/1) independently. This command's
 * own configuration invariant -- slopeCountsPerCm+offsetCounts>0 -- is
 * checked here, not in vision_rx.c and not in any other controller's code:
 * it guards against the smallest possible non-zero command (1 cm)
 * resolving to zero or negative target counts (e.g. slope=50,offset=-500
 * would give 1*50-500=-450). Checked only when enabling (disabling is
 * always allowed, exactly like every other Stage-A invariant this
 * session, since a disabled MOVCAL's slope/offset are inert and never
 * read by AppMotion_CalcDistanceCounts()). Rejected before
 * AppMotion_SetDistanceCal() is ever called, so the active configuration
 * is left completely unchanged on rejection. */
void on_distance_cal_set(float slopeCountsPerCm, float offsetCounts, int enable)
{
    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    if ((enable != 0) &&
        ((slopeCountsPerCm + offsetCounts) <= 0.0f))
    {
        AppComm_Send("MOVCAL,ERR\n");
        return;
    }

    AppMotion_SetDistanceCal(slopeCountsPerCm, offsetCounts, (enable != 0));
    AppComm_Send("MOVCAL,OK\n");
}

/* GET remains available while moving. */
void on_distance_cal_get(void)
{
    float slopeCountsPerCm;
    float offsetCounts;
    bool enable;
    char slopeCountsPerCmText[24];
    char offsetCountsText[32];
    char offsetMagnitudeText[24];
    char response[80];
    int length;

    AppMotion_GetDistanceCal(&slopeCountsPerCm, &offsetCounts, &enable);

    AppComm_FormatGain(slopeCountsPerCm, slopeCountsPerCmText, sizeof(slopeCountsPerCmText));

    /* AppComm_FormatGain() rejects negative input (formats it as the
     * literal string "ERR") -- every other GET reply in this codebase
     * only ever formats a non-negative gain/count, so this is the first
     * field that needs a negative-capable path. Handled locally here,
     * not by changing the shared helper (which every other controller's
     * GET reply also uses): format the magnitude, then prepend '-' by
     * hand if the original value was negative. */
    if (offsetCounts < 0.0f)
    {
        AppComm_FormatGain(-offsetCounts, offsetMagnitudeText, sizeof(offsetMagnitudeText));
        (void)snprintf(offsetCountsText, sizeof(offsetCountsText), "-%s", offsetMagnitudeText);
    }
    else
    {
        AppComm_FormatGain(offsetCounts, offsetCountsText, sizeof(offsetCountsText));
    }

    length = snprintf(
        response,
        sizeof(response),
        "MOVCAL,M:%s,B:%s,E:%u\n",
        slopeCountsPerCmText,
        offsetCountsText,
        enable ? 1U : 0U
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

/* Named runtime configuration profile SET is idle-only, mirroring every
 * other SET command in this file -- rejected up front if the robot is
 * moving, exactly like AppMotion_ApplyProfile() atomically rewriting
 * nine live control parameters mid-move would be. Parser (vision_rx.c)
 * has already rejected any name other than PRECISION/STANDARD before
 * this is ever called (CUSTOM has no corresponding SET form), so the
 * switch below has no invalid-value case to reject; it only maps the
 * parser's own local VisionProfileType onto AppMotionProfile.
 * AppMotion_ApplyProfile() does the actual atomic apply + relevant PID
 * integral reset. */
void on_motion_profile_set(VisionProfileType profile)
{
    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    if (profile == VISION_PROFILE_STANDARD)
    {
        AppMotion_ApplyProfile(APP_MOTION_PROFILE_STANDARD);
    }
    else
    {
        AppMotion_ApplyProfile(APP_MOTION_PROFILE_PRECISION);
    }

    AppComm_Send("PROFILE,OK\n");
}

/* GET remains available while moving, matching every other GET in this
 * file. */
void on_motion_profile_get(void)
{
    switch (AppMotion_GetProfile())
    {
        case APP_MOTION_PROFILE_STANDARD:
            AppComm_Send("PROFILE,STANDARD\n");
            break;

        case APP_MOTION_PROFILE_PRECISION:
            AppComm_Send("PROFILE,PRECISION\n");
            break;

        case APP_MOTION_PROFILE_CUSTOM:
        default:
            AppComm_Send("PROFILE,CUSTOM\n");
            break;
    }
}

/* Stage SFF1: speed-aware servo feed-forward SET is idle-only. Parser
 * (vision_rx.c) has already range-checked startSpeed/fullSpeed
 * (0..8000), maxUs (-100..100) and enable (0/1) independently, and
 * already restricted direction to 'F'/'B'. This command's own
 * configuration invariant -- fullSpeed>startSpeed -- is checked here,
 * not in vision_rx.c: it guards against a degenerate/inverted ramp
 * (zero or negative-width denominator in the ratio computed in
 * app_motion.c). Checked only when enabling (disabling is always
 * allowed), explicit rejection rather than a silent clamp/reorder.
 * Deliberately has no relationship to PROFILE -- see
 * AppMotion_SetServoFeedForward()'s own doc comment; this command never
 * marks the active profile CUSTOM and is never touched by
 * PROFILE,PRECISION/STANDARD. */
void on_servo_feed_forward_set(char direction, float startSpeed, float fullSpeed, float maxUs, int enable)
{
    MotionType motionDirection =
        (direction == 'B') ? MOTION_BACKWARD : MOTION_FORWARD;

    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    if ((enable != 0) &&
        (fullSpeed <= startSpeed))
    {
        AppComm_Send("SFF,ERR\n");
        return;
    }

    AppMotion_SetServoFeedForward(motionDirection, startSpeed, fullSpeed, maxUs, (enable != 0));
    AppComm_Send("SFF,OK\n");
}

/* GET remains available while moving. Reports both directions in one
 * reply -- mirrors the earlier, since-reverted Stage A5.3a STRIM,GET's
 * own "both directions in one line" shape. maxUs is signed, so it needs
 * the same negative-capable local formatting on_distance_cal_get() uses
 * for MOVCAL's offsetCounts (AppComm_FormatGain() itself rejects
 * negative input) -- handled locally here, not by changing the shared
 * helper every other GET reply also uses. */
static void AppComm_FormatSignedGain(float value, char *text, size_t textSize)
{
    char magnitudeText[24];

    if (value < 0.0f)
    {
        AppComm_FormatGain(-value, magnitudeText, sizeof(magnitudeText));
        (void)snprintf(text, textSize, "-%s", magnitudeText);
    }
    else
    {
        AppComm_FormatGain(value, text, textSize);
    }
}

void on_servo_feed_forward_get(void)
{
    float forwardStartSpeed;
    float forwardFullSpeed;
    float forwardMaxUs;
    bool  forwardEnable;
    float backwardStartSpeed;
    float backwardFullSpeed;
    float backwardMaxUs;
    bool  backwardEnable;
    char forwardStartSpeedText[24];
    char forwardFullSpeedText[24];
    char forwardMaxUsText[32];
    char backwardStartSpeedText[24];
    char backwardFullSpeedText[24];
    char backwardMaxUsText[32];
    char response[160];
    int length;

    AppMotion_GetServoFeedForward(MOTION_FORWARD, &forwardStartSpeed, &forwardFullSpeed, &forwardMaxUs, &forwardEnable);
    AppMotion_GetServoFeedForward(MOTION_BACKWARD, &backwardStartSpeed, &backwardFullSpeed, &backwardMaxUs, &backwardEnable);

    AppComm_FormatGain(forwardStartSpeed, forwardStartSpeedText, sizeof(forwardStartSpeedText));
    AppComm_FormatGain(forwardFullSpeed, forwardFullSpeedText, sizeof(forwardFullSpeedText));
    AppComm_FormatSignedGain(forwardMaxUs, forwardMaxUsText, sizeof(forwardMaxUsText));
    AppComm_FormatGain(backwardStartSpeed, backwardStartSpeedText, sizeof(backwardStartSpeedText));
    AppComm_FormatGain(backwardFullSpeed, backwardFullSpeedText, sizeof(backwardFullSpeedText));
    AppComm_FormatSignedGain(backwardMaxUs, backwardMaxUsText, sizeof(backwardMaxUsText));

    length = snprintf(
        response,
        sizeof(response),
        "SFF,FS:%s,FF:%s,FM:%s,FE:%u,BS:%s,BF:%s,BM:%s,BE:%u\n",
        forwardStartSpeedText,
        forwardFullSpeedText,
        forwardMaxUsText,
        forwardEnable ? 1U : 0U,
        backwardStartSpeedText,
        backwardFullSpeedText,
        backwardMaxUsText,
        backwardEnable ? 1U : 0U
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(response)))
    {
        AppComm_Send(response);
    }
}

void on_tune_motor(
    char target,
    int32_t counts
)
{
    MotionType type;

    switch (target)
    {
        case 'L':
            type = MOTION_TUNE_LEFT;
            break;

        case 'R':
            type = MOTION_TUNE_RIGHT;
            break;

        case 'B':
            type = MOTION_TUNE_BOTH;
            break;

        default:
            AppComm_Send("ERR,BAD_CMD\n");
            return;
    }

    (void)AppComm_QueueMotionCommand(type, counts);
}

void on_tune_steering(
    char dir,
    uint16_t seconds
)
{
    MotionType type;

    switch (dir)
    {
        case 'F':
            type = MOTION_TUNE_STEERING_FORWARD;
            break;

        case 'B':
            type = MOTION_TUNE_STEERING_BACKWARD;
            break;

        default:
            AppComm_Send("ERR,BAD_CMD\n");
            return;
    }

    (void)AppComm_QueueMotionCommand(
        type,
        (int32_t)seconds
    );
}
