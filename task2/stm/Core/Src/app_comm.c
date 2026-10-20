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
#include "app_ultrasonic.h"
#include "app_us_park.h"
#include "app_park_trace.h"
#include "app_ir.h"
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
#if APP_ULTRASONIC_BENCH_LOG
static uint32_t lastUltrasonicLogTick;
#endif
#if APP_IR_BENCH_LOG
static uint32_t lastIrLogTick;
#endif
#if APP_IR_ENABLE
/* IR,GET replies still owed (see on_ir_get()), requests discarded because the
 * limit was reached, and the most replies ever owed at once. CommTask-only. */
static uint8_t irGetPending;
static uint16_t irGetDropped;
static uint8_t irGetPendingHighWater;
/* MOV,FIRL/FIRR decision line (IREDGE,): the last mailbox sequence handled and
 * a CommTask-only copy (a file static, not a stack local). */
static uint32_t lastIrEdgeSequence;
static AppMotionIrEdgeSnapshot irEdgeService;
#endif
/* USD,GET replies still owed (see on_usd_get()), requests discarded because
 * the limit was reached, and the most replies ever owed at once. CommTask-only. */
static uint8_t usdGetPending;
static uint16_t usdGetDropped;
static uint8_t usdGetPendingHighWater;
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

/* Priority STOP (Stage 2). All four are private to this file.
 * stopOutstanding: CommTask-owned; true from acceptance of a STOP until
 * its APP_MOTION_EVENT_STOPPED has been processed. Gates new commands and
 * coalesces duplicate STOPs.
 * priorityStopPending: one-shot latch set by on_stop() (CommTask) and
 * consumed exactly once by AppComm_TakeMotionCommand() (MotionTask).
 * motionQueueFenced: while set, MotionTask never reads MotionQueueHandle
 * and no new command is admitted; cleared only after the queue has been
 * verified empty.
 * stopQueueDrainFaults: debugger-visible count of queue-drain anomalies
 * (no wire output).
 *
 * Safety of the hand-off relies on MotionTask (osPriorityAboveNormal)
 * outranking CommTask (osPriorityNormal) on a single core, and on every
 * set/take being done inside a critical section. */
static bool stopOutstanding;
static volatile uint8_t priorityStopPending;
static volatile uint8_t motionQueueFenced;
static volatile uint32_t stopQueueDrainFaults;

/* Ultrasonic parking protection (USPARK). The detector is a pure module
 * (app_us_park.c) owned and evaluated ONLY by CommTask: it is touched from
 * the parser callbacks and AppComm_ServiceParking(), both CommTask. It
 * never stops anything itself -- an automatic STOP goes through the same
 * AppComm_RequestPriorityStop() as the manual STOP command. */
static AppUsPark usPark;

/* CommTask-only scratch for large structures. CommTask has a 1 KB stack and
 * several chains through it are deep; these used to be automatic variables
 * (AppMotionTelemetry alone is ~200 bytes). They are only ever touched from
 * CommTask, each user is non-reentrant and never nests inside another user
 * of the SAME scratch, and every use first overwrites the whole structure
 * through the AppMotion_Get*() call, so this is behaviour-neutral. */
static AppMotionEvent stopEvent;
static AppMotionTelemetry stopTelemetry;
static AppMotionEvent serviceMotionEvent;
static AppMotionTelemetry serviceTelemetry;
static AppUltrasonicSnapshot parkSnapshot;

typedef struct
{
    uint32_t movementSeq;   /* the forward movement the request is for */
} AppCommStopGuard;

static bool AppComm_RequestPriorityStop(const AppCommStopGuard *guard);
static bool AppComm_StopGuardRefuses(const AppCommStopGuard *guard);
static uint32_t lastParkingServiceTick;

#if APP_USPARK_TRACE
/* Measurement-only USPARK trace (see app_park_trace.h). Producers hand fixed
 * binary records to the trace ring; this file only builds them and, in
 * AppComm_ServiceParkTrace(), formats and sends them from CommTask once the
 * stop is over and the link is idle. Nothing here is read by any control
 * decision. The scratch records are statics (not stack locals) to keep the
 * CommTask / MotionTask stacks as they were. */
static AppParkTraceRecord parkTraceRec;         /* CommTask-only scratch    */
static AppParkTraceRecord parkTraceTakeRec;     /* MotionTask-only scratch  */
static AppMotionEvent parkTraceEvent;           /* CommTask-only scratch    */
static uint32_t parkTraceLastUsSeq;
static bool parkTraceSeqValid;
static uint32_t parkTraceFormatFailures;        /* debugger-visible only    */

_Static_assert((unsigned)APP_USPARK_CAUSE_FAULT    == APP_PARK_CAUSE_FAULT &&
               (unsigned)APP_USPARK_CAUSE_NOUPDATE == APP_PARK_CAUSE_NOUPDATE &&
               (unsigned)APP_USPARK_CAUSE_INVALID  == APP_PARK_CAUSE_INVALID &&
               (unsigned)APP_USPARK_CAUSE_NEAR     == APP_PARK_CAUSE_NEAR,
               "trace cause codes must equal AppUsParkCause");

static void AppComm_TraceParkingStep(const AppUsParkInputs *in,
                                     AppUsParkCause cause,
                                     bool armedBefore,
                                     uint8_t nearBefore);
static void AppComm_TraceRequest(const AppUltrasonicSnapshot *snapshot,
                                 AppUsParkCause cause);
static void AppComm_TraceAccepted(void);
static void AppComm_TraceTake(void);
#endif

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

/* True while a STOP operation is outstanding, its latch has not been
 * consumed yet, or the motion queue fence is still raised. Every
 * admission path (motion queue, idle-only SETs, PID SET) treats this as
 * "busy". */
static bool AppComm_StopGateActive(void)
{
    return stopOutstanding ||
           (priorityStopPending != 0U) ||
           (motionQueueFenced != 0U);
}

/* Empties MotionQueueHandle and reports whether it is verifiably empty.
 * The status of osMessageQueueReset() is checked, but never trusted on
 * its own (the CMSIS wrapper discards xQueueReset()'s result): after the
 * reset a bounded get loop removes anything left, and success is decided
 * only by the final queue count. Any anomaly bumps stopQueueDrainFaults.
 * Never blocks (all timeouts are 0). */
static bool AppComm_DrainMotionQueue(void)
{
    MotionCommand discarded;
    uint32_t capacity;
    uint32_t attempts;
    bool resetOk;
    bool empty;

    if (osMessageQueueGetCount(MotionQueueHandle) == 0U)
    {
        return true;
    }

    resetOk = (osMessageQueueReset(MotionQueueHandle) == osOK);
    capacity = osMessageQueueGetCapacity(MotionQueueHandle);

    for (attempts = 0U;
         (attempts < capacity) &&
         (osMessageQueueGetCount(MotionQueueHandle) != 0U);
         attempts++)
    {
        (void)osMessageQueueGet(
            MotionQueueHandle,
            &discarded,
            NULL,
            0U
        );
    }

    empty = (osMessageQueueGetCount(MotionQueueHandle) == 0U);

    if (!resetOk || !empty)
    {
        taskENTER_CRITICAL();
        stopQueueDrainFaults++;
        taskEXIT_CRITICAL();
    }

    return empty;
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
           AppComm_StopGateActive() ||
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
        AppComm_StopGateActive() ||
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

        case APP_MOTION_EVENT_IR_NO_OBSTACLE:
            AppComm_Send("ERR,IR_NO_OBSTACLE\n");
            return;

        case APP_MOTION_EVENT_IR_NO_EDGE:
            AppComm_Send("ERR,IR_NO_EDGE\n");
            return;

        case APP_MOTION_EVENT_IR_SENSOR:
            AppComm_Send("ERR,IR_SENSOR\n");
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

        case APP_MOTION_EVENT_STOPPED:
            /* Only ever sent for the Stage 1 STOPPED event (never from
             * parsing or queue dispatch). */
#if APP_IR_ENABLE
            /* A stopped FIR movement reports no edge decision: drop one that
             * was published but not yet sent so it cannot follow the reply. */
            AppMotion_GetIrEdgeSnapshot(&irEdgeService);
            lastIrEdgeSequence = irEdgeService.sequence;
#endif
            AppComm_Send("ACK,STOPPED\n");
            return;

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
        case MOTION_FORWARD_IR_LEFT:  return "FIRL";
        case MOTION_FORWARD_IR_RIGHT: return "FIRR";
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
        case APP_MOTION_EVENT_STOPPED:        result = "STOPPED";       break;
        case APP_MOTION_EVENT_IR_NO_OBSTACLE: result = "IR_NO_OBSTACLE"; break;
        case APP_MOTION_EVENT_IR_NO_EDGE:     result = "IR_NO_EDGE";     break;
        case APP_MOTION_EVENT_IR_SENSOR:      result = "IR_SENSOR";      break;
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

/* Shared by ServiceTelemetry() and the pre-STOP flush: the movement END
 * marker (when a START was sent) immediately followed by the terminal
 * reply. */
static void AppComm_ReportMotionEvent(
    const AppMotionEvent *event
)
{
    AppComm_SendMovementEndMarker(event);
    AppComm_SendMotionEvent(event);
}

/* Sends MM,EV:START if a MOV,F/B/FL/FR/BL/BR newly became active since
 * the last one (runSequence changed, MOV mode only; TUNE_* also bump
 * runSequence but are excluded). Returns true if a START was sent. */
static bool AppComm_TrySendMovementStart(
    const AppMotionTelemetry *telemetry
)
{
    if ((telemetry->runSequence != lastTelemetryRunSequence) &&
        (telemetry->active != 0U) &&
        ((telemetry->mode == APP_TUNE_MOV_DISTANCE) ||
         (telemetry->mode == APP_TUNE_MOV_TURN)))
    {
        lastTelemetryRunSequence = telemetry->runSequence;
        lastKnownDirection = telemetry->activeDirection;
        lastKnownStartTick = telemetry->movementStartTick;
        movementStartEmitted = true;

        AppComm_SendMovementStartMarker(telemetry);
        return true;
    }

    return false;
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

#if APP_ULTRASONIC_BENCH_LOG
/* ---------------------------------------------------------------------- *
 * Ultrasonic bench log (APP_ULTRASONIC_BENCH_LOG, default 0).
 *
 * One "US," line, at most APP_COMM_ULTRASONIC_LOG_PERIOD_MS apart (5 Hz),
 * built in CommTask from the driver's published snapshot -- never from an
 * interrupt, and with no new incoming command. Like the TB, line it reuses
 * movementSummaryResponse, which is only ever used one line at a time from
 * CommTask.
 *
 *   US,ST:<state>,R:<result>,PW:<us|NA>,RAW:<mm|NA>,FLT:<mm|NA>,V:<0|1>,
 *      F:<0|1>,AGE:<ms|NA>,SEQ:<n>,CYC:<n>,OK:<n>,NR:<n>,NF:<n>,NEAR:<n>,
 *      FAR:<n>,EDG:<n>,LATE:<n>,ES:<n>,STL:<n>,SRS:<n>,SPU:<n>,SCP:<n>,
 *      FM:<hex>
 *
 * V = latest cycle valid, F = filtered value fresh (not stale). A value
 * field is "NA" whenever it is not meaningful, so 0 is never a sentinel.
 * PW is reported for any cycle that captured both edges (including
 * NEAR/FAR) to help calibration. Counters saturate at 65535.
 * ---------------------------------------------------------------------- */
#define APP_COMM_ULTRASONIC_LOG_PERIOD_MS 200U

static const char *AppComm_UltrasonicStateString(
    AppUltrasonicState state
)
{
    switch (state)
    {
        case APP_US_STATE_IDLE:         return "IDLE";
        case APP_US_STATE_TRIGGER_HIGH: return "TRIG";
        case APP_US_STATE_WAIT_RISE:    return "RISE";
        case APP_US_STATE_WAIT_FALL:    return "FALL";
        case APP_US_STATE_DONE:         return "DONE";
        case APP_US_STATE_FAULT:        return "FAULT";
        default:                        return "?";
    }
}

static const char *AppComm_UltrasonicResultString(
    AppUltrasonicResult result
)
{
    switch (result)
    {
        case APP_US_RESULT_NONE:        return "NONE";
        case APP_US_RESULT_VALID:       return "OK";
        case APP_US_RESULT_NO_RISE:     return "NORISE";
        case APP_US_RESULT_NO_FALL:     return "NOFALL";
        case APP_US_RESULT_TOO_NEAR:    return "NEAR";
        case APP_US_RESULT_TOO_FAR:     return "FAR";
        case APP_US_RESULT_EDGE_ERROR:  return "EDGE";
        case APP_US_RESULT_LATE_RISE:   return "LATE";
        case APP_US_RESULT_ECHO_STUCK:  return "STUCK";
        case APP_US_RESULT_STALL:       return "STALL";
        case APP_US_RESULT_FAULT:       return "FAULT";
        default:                        return "?";
    }
}

static void AppComm_UltrasonicField(
    char *out,
    size_t outSize,
    bool present,
    uint32_t value
)
{
    if (present)
    {
        (void)snprintf(out, outSize, "%lu", (unsigned long)value);
    }
    else
    {
        (void)snprintf(out, outSize, "NA");
    }
}

static bool AppComm_TrySendUltrasonicLog(uint32_t now)
{
    AppUltrasonicSnapshot snapshot;
    uint32_t periodTicks;
    uint32_t ticksPerMs;
    uint32_t ageTicks;
    char pulse[12];
    char raw[12];
    char filtered[12];
    char age[12];
    int length;

    periodTicks =
        (osKernelGetTickFreq() * APP_COMM_ULTRASONIC_LOG_PERIOD_MS) /
        1000U;

    if (periodTicks == 0U)
    {
        periodTicks = 1U;
    }

    if ((now - lastUltrasonicLogTick) < periodTicks)
    {
        return false;
    }

    lastUltrasonicLogTick = now;

    AppUltrasonic_GetSnapshot(&snapshot);

    ticksPerMs = osKernelGetTickFreq() / 1000U;

    if (ticksPerMs == 0U)
    {
        ticksPerMs = 1U;
    }

    ageTicks = AppUltrasonic_AgeTicks(&snapshot, now);

    AppComm_UltrasonicField(
        pulse, sizeof(pulse), snapshot.pulseCaptured != 0U, snapshot.pulseUs);
    AppComm_UltrasonicField(
        raw, sizeof(raw), snapshot.rawValid != 0U, snapshot.rawMm);
    AppComm_UltrasonicField(
        filtered, sizeof(filtered),
        snapshot.filteredValid != 0U, snapshot.filteredMm);
    AppComm_UltrasonicField(
        age, sizeof(age), ageTicks != UINT32_MAX, ageTicks / ticksPerMs);

    length = snprintf(
        movementSummaryResponse,
        sizeof(movementSummaryResponse),
        "US,ST:%s,R:%s,PW:%s,RAW:%s,FLT:%s,V:%u,F:%u,AGE:%s,"
        "SEQ:%lu,CYC:%lu,OK:%u,NR:%u,NF:%u,NEAR:%u,FAR:%u,EDG:%u,"
        "LATE:%u,ES:%u,STL:%u,SRS:%u,SPU:%u,SCP:%u,FM:%02X\n",
        AppComm_UltrasonicStateString(snapshot.state),
        AppComm_UltrasonicResultString(snapshot.result),
        pulse,
        raw,
        filtered,
        (unsigned)snapshot.rawValid,
        (unsigned)(AppUltrasonic_IsFresh(&snapshot, now) ? 1U : 0U),
        age,
        (unsigned long)snapshot.sequence,
        (unsigned long)snapshot.cycles,
        (unsigned)snapshot.counters.valid,
        (unsigned)snapshot.counters.noRise,
        (unsigned)snapshot.counters.noFall,
        (unsigned)snapshot.counters.tooNear,
        (unsigned)snapshot.counters.tooFar,
        (unsigned)snapshot.counters.edgeError,
        (unsigned)snapshot.counters.lateRise,
        (unsigned)snapshot.counters.echoStuck,
        (unsigned)snapshot.counters.stall,
        (unsigned)snapshot.counters.staleResult,
        (unsigned)snapshot.counters.spurious,
        (unsigned)snapshot.counters.staleCompare,
        (unsigned)snapshot.faultMask
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(movementSummaryResponse)))
    {
        AppComm_Send(movementSummaryResponse);
    }

    return true;
}
#endif /* APP_ULTRASONIC_BENCH_LOG */

/* ---------------------------------------------------------------------- *
 * USD,GET reply (front ultrasonic, read-only telemetry; root PROTOCOL.md §4.8).
 *
 *   USD,S:<seq>,ST:<OK|WARM|STALE|ERR|FAULT>,D:<mm|NEAR|FAR|NA>,AGE:<ms|NA>,F:<hex>
 *
 * Built and sent only by CommTask from AppComm_ServiceTelemetry(), exactly
 * like the IR,GET reply: on_usd_get() only records that a reply is owed, so
 * the reply never delays or overtakes ACK,STOPPED, MM,EV:END and its terminal
 * reply, or any ACK / ERR due in the same pass. Always built (the ultrasonic
 * driver is part of every build); independent of APP_ULTRASONIC_BENCH_LOG.
 *
 * ST from the driver's published snapshot, first match wins:
 *   FAULT  driver state or result FAULT, or a non-zero fault mask (F = mask | 40)
 *   STALE  nothing published yet, or the newest result older than
 *          APP_US_STALE_TICKS_RTOS (200 ms)                          (F:40)
 *   OK     newest result VALID and the 3-sample median full: D = filtered mm
 *          from the transducer face (below 10 mm: NEAR)              (F:00)
 *   WARM   newest result VALID, median not full yet                  (F:00)
 *   OK     newest result TOO_NEAR: D:NEAR                            (F:00)
 *   OK     newest result TOO_FAR, or NO_FALL (the echo line never fell
 *          before the timeout: the HC-SR04's no-echo case): D:FAR    (F:00)
 *   ERR    any other result (NO_RISE, EDGE, LATE, STUCK, STALL, NONE) (F:40)
 * D is NA unless ST is OK. S = the snapshot's publication sequence (0 =
 * nothing published). AGE = milliseconds since the newest publication (the
 * latest VALID one for a distance), capped at 65535, NA if none.
 *
 * Buffer bound: the longest reply, "USD,S:4294967295,ST:STALE,D:NA,AGE:65535,
 * F:FFFF" (synthetic), is 47 characters + '\n' (host test).
 * ---------------------------------------------------------------------- */
#define APP_COMM_USD_GET_MAX_PENDING 4U
#define APP_COMM_USD_AGE_CAP_MS      65535UL
#define APP_COMM_USD_NEAR_BELOW_MM   10U
#define APP_COMM_USD_NOT_FRESH       0x40U

static void AppComm_UsdAgeField(
    char *out, size_t size, bool present, uint32_t ageTicks)
{
    uint32_t ticksPerMs = osKernelGetTickFreq() / 1000U;
    unsigned long ageMs;

    if (!present)
    {
        (void)snprintf(out, size, "NA");
        return;
    }

    if (ticksPerMs == 0U)
    {
        ticksPerMs = 1U;
    }

    ageMs = (unsigned long)(ageTicks / ticksPerMs);

    if (ageMs > APP_COMM_USD_AGE_CAP_MS)
    {
        ageMs = APP_COMM_USD_AGE_CAP_MS;
    }

    (void)snprintf(out, size, "%lu", ageMs);
}

static bool AppComm_TrySendUsdGet(uint32_t now)
{
    AppUltrasonicSnapshot snapshot;
    const char *state;
    char distance[8];
    char age[8];
    unsigned flags = 0U;
    uint32_t ageTicks;
    bool published;
    int length;

    if (usdGetPending == 0U)
    {
        return false;
    }

    usdGetPending--;

    AppUltrasonic_GetSnapshot(&snapshot);

    published = (snapshot.sequence != 0U);
    ageTicks = (uint32_t)(now - snapshot.lastSampleTick);
    (void)snprintf(distance, sizeof(distance), "NA");

    if ((snapshot.state == APP_US_STATE_FAULT) ||
        (snapshot.result == APP_US_RESULT_FAULT) ||
        (snapshot.faultMask != 0U))
    {
        state = "FAULT";
        flags = (unsigned)snapshot.faultMask | APP_COMM_USD_NOT_FRESH;
    }
    else if (!published || (ageTicks > APP_US_STALE_TICKS_RTOS))
    {
        state = "STALE";
        flags = APP_COMM_USD_NOT_FRESH;
    }
    else if (snapshot.result == APP_US_RESULT_VALID)
    {
        if ((snapshot.filteredValid != 0U) &&
            (snapshot.filterSamples >= APP_US_FILTER_DEPTH))
        {
            state = "OK";
            ageTicks = (uint32_t)(now - snapshot.lastValidTick);

            if (snapshot.filteredMm < APP_COMM_USD_NEAR_BELOW_MM)
            {
                (void)snprintf(distance, sizeof(distance), "NEAR");
            }
            else
            {
                (void)snprintf(distance, sizeof(distance), "%u",
                               (unsigned)snapshot.filteredMm);
            }
        }
        else
        {
            state = "WARM";
        }
    }
    else if (snapshot.result == APP_US_RESULT_TOO_NEAR)
    {
        state = "OK";
        (void)snprintf(distance, sizeof(distance), "NEAR");
    }
    else if ((snapshot.result == APP_US_RESULT_TOO_FAR) ||
             (snapshot.result == APP_US_RESULT_NO_FALL))
    {
        state = "OK";
        (void)snprintf(distance, sizeof(distance), "FAR");
    }
    else
    {
        state = "ERR";
        flags = APP_COMM_USD_NOT_FRESH;
    }

    AppComm_UsdAgeField(age, sizeof(age), published, ageTicks);

    length = snprintf(
        movementSummaryResponse,
        sizeof(movementSummaryResponse),
        "USD,S:%lu,ST:%s,D:%s,AGE:%s,F:%02X\n",
        (unsigned long)snapshot.sequence,
        state,
        distance,
        age,
        flags
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(movementSummaryResponse)))
    {
        AppComm_Send(movementSummaryResponse);
    }

    return true;
}

#if APP_IR_BENCH_LOG
/* ---------------------------------------------------------------------- *
 * IR raw bench log (APP_IR_BENCH_LOG, default 0; implies APP_IR_ENABLE).
 *
 * One "IRRAW," line, at most APP_COMM_IR_LOG_PERIOD_MS apart (5 Hz), built
 * in CommTask from the published IR snapshot -- never from IRTask or an
 * interrupt, and with no incoming command. Same placement and gating as the
 * "US," line, and it reuses movementSummaryResponse the same way.
 *
 *   IRRAW,S:<seq>,L:<0-4095|NA>,R:<0-4095|NA>,F:<hex>
 *
 * S = publication sequence (0 = nothing published yet). Acquisition runs at
 *     about 50 Hz, so S normally advances by about 10 per line: the line
 *     shows one raw sample, it never averages or filters.
 * L / R = raw 12-bit ADC counts of the LEFT (PC0) / RIGHT (PC1) sensor from
 *     that publication. NOT distance, NOT calibrated. "NA" means acquisition
 *     unavailable, failed or stale (AppIr_IsFresh() false) -- never that a
 *     raw value looks implausible: 0 and 4095 are printed as-is.
 * F = AppIr_ReportMask(): APP_IR_FAULT_* bits plus STALE (0x40), at least
 *     two hex digits (more only for latched faults). It is always 00 when
 *     L/R carry values.
 * Longest line "IRRAW,S:4294967295,L:4095,R:4095,F:00" = 37 chars + '\n'.
 * ---------------------------------------------------------------------- */
#define APP_COMM_IR_LOG_PERIOD_MS 200U

static bool AppComm_TrySendIrLog(uint32_t now)
{
    AppIrSnapshot snapshot;
    uint32_t periodTicks;
    int length;

    periodTicks =
        (osKernelGetTickFreq() * APP_COMM_IR_LOG_PERIOD_MS) / 1000U;

    if (periodTicks == 0U)
    {
        periodTicks = 1U;
    }

    if ((now - lastIrLogTick) < periodTicks)
    {
        return false;
    }

    lastIrLogTick = now;

    AppIr_GetSnapshot(&snapshot);

    if (AppIr_IsFresh(&snapshot, now))
    {
        length = snprintf(
            movementSummaryResponse,
            sizeof(movementSummaryResponse),
            "IRRAW,S:%lu,L:%u,R:%u,F:%02X\n",
            (unsigned long)snapshot.sequence,
            (unsigned)snapshot.leftRaw,
            (unsigned)snapshot.rightRaw,
            (unsigned)AppIr_ReportMask(&snapshot, now)
        );
    }
    else
    {
        length = snprintf(
            movementSummaryResponse,
            sizeof(movementSummaryResponse),
            "IRRAW,S:%lu,L:NA,R:NA,F:%02X\n",
            (unsigned long)snapshot.sequence,
            (unsigned)AppIr_ReportMask(&snapshot, now)
        );
    }

    if ((length > 0) &&
        ((size_t)length < sizeof(movementSummaryResponse)))
    {
        AppComm_Send(movementSummaryResponse);
    }

    return true;
}
#endif /* APP_IR_BENCH_LOG */

#if APP_IR_ENABLE
/* ---------------------------------------------------------------------- *
 * IR,GET reply (Phase 3, telemetry only; always built with APP_IR_ENABLE).
 *
 *   IR,S:<seq>,ST:<OK|WARM|STALE|ERR|FAULT>,L:<mm|NEAR|FAR|NA>,R:<mm|NEAR|FAR|NA>,AGE:<ms|NA>,F:<hex>
 *
 * Built and sent only by CommTask, from AppComm_ServiceTelemetry(), NEVER
 * from the parser callback: on_ir_get() only records that a reply is owed.
 * That keeps the reply behind every terminal line, because AppComm_ServiceTelemetry() returns after each STOPPED / event
 * / MM,EV:START / completion / turn-brake line it sends, and this reply is
 * considered only when none of those was due in the pass and no STOP is
 * outstanding. At most one line per pass.
 *
 * ST = AppIrAcqState. L and R are millimetres FROM THE SENSOR FACE (the
 * 45 mm face-to-tyre offset is not applied) inside the calibrated 100-400 mm
 * domain (a +-5 mm guard band is clamped onto 100 / 400); NEAR = closer than
 * about 95 mm or saturated; FAR = farther than about 405 mm
 * or no target. L and R are NA unless ST is OK. AGE = milliseconds since
 * the newest publication (capped at 65535; NA if nothing was published).
 * F = AppIr_ReportMask() (fault bits plus 0x40 whenever the newest publication
 * is not a fresh good one, i.e. for STALE, ERR and FAULT), at least two digits.
 *
 * Buffer bound: no reply exceeds 52 characters + '\n' (the host test formats a
 * SYNTHETIC worst case, "IR,S:4294967295,ST:OK,L:NEAR,R:NEAR,AGE:65535,F:FFFF",
 * to prove this; the driver can never produce that combination).
 * ---------------------------------------------------------------------- */
#define APP_COMM_IR_GET_MAX_PENDING 4U
#define APP_COMM_IR_AGE_CAP_MS      65535UL

static const char *AppComm_IrAcqString(uint8_t acq)
{
    switch (acq)
    {
        case APP_IR_ACQ_OK:     return "OK";
        case APP_IR_ACQ_WARMUP: return "WARM";
        case APP_IR_ACQ_STALE:  return "STALE";
        case APP_IR_ACQ_ERROR:  return "ERR";
        case APP_IR_ACQ_FAULT:  return "FAULT";
        default:                return "?";
    }
}

static void AppComm_IrDistanceField(
    char *out, size_t size, const AppIrReading *reading, uint8_t sensor)
{
    if ((reading->acq != (uint8_t)APP_IR_ACQ_OK) ||
        (reading->range[sensor] == (uint8_t)APP_IR_RANGE_INVALID))
    {
        (void)snprintf(out, size, "NA");
    }
    else if (reading->range[sensor] == (uint8_t)APP_IR_RANGE_NEAR)
    {
        (void)snprintf(out, size, "NEAR");
    }
    else if (reading->range[sensor] == (uint8_t)APP_IR_RANGE_FAR)
    {
        (void)snprintf(out, size, "FAR");
    }
    else
    {
        (void)snprintf(out, size, "%u", (unsigned)reading->distanceMm[sensor]);
    }
}

/* IREDGE, -- one line per MOV,FIRL/FIRR decision (edge confirmed, maximum
 * travel reached, sensor failure), sent from the telemetry service after every
 * STOP / event / START / completion line, never while a STOP is outstanding,
 * one line per pass. SC/CC are chassis displacement counts at the confirming
 * scan and SR/CR its raw count; -1 = that never happened. W is the sensor
 * failure reason (NA otherwise). */
static const char *AppComm_IrEdgeResultString(uint8_t result)
{
    switch (result)
    {
        case APP_MOTION_IR_RESULT_EDGE:         return "EDGE";
        case APP_MOTION_IR_RESULT_NO_OBSTACLE:  return "NO_OBSTACLE";
        case APP_MOTION_IR_RESULT_NO_EDGE:      return "NO_EDGE";
        default:                                return "SENSOR";
    }
}

static const char *AppComm_IrEdgeFailString(const AppMotionIrEdgeSnapshot *s)
{
    if (s->result != (uint8_t)APP_MOTION_IR_RESULT_SENSOR)
    {
        return "NA";
    }

    switch (s->failReason)
    {
        case 1U:  return "FAULT";       /* APP_IR_EDGE_FAIL_FAULT         */
        case 2U:  return "READY";       /* ... READY_TIMEOUT              */
        case 3U:  return "DATA";        /* ... DATA_LOST                  */
        default:  return "CFG";
    }
}

static bool AppComm_TrySendIrEdge(void)
{
    int length;

    AppMotion_GetIrEdgeSnapshot(&irEdgeService);

    if ((irEdgeService.valid == 0U) ||
        (irEdgeService.sequence == lastIrEdgeSequence))
    {
        return false;
    }

    lastIrEdgeSequence = irEdgeService.sequence;

    length = snprintf(
        movementSummaryResponse,
        sizeof(movementSummaryResponse),
        "IREDGE,D:%s,R:%s,SA:%u,SC:%ld,SR:%ld,CC:%ld,CR:%ld,"
        "UN:%u,AMB:%u,MISS:%u,RDY:%lu,W:%s\n",
        AppComm_MovementDirectionString(irEdgeService.direction),
        AppComm_IrEdgeResultString(irEdgeService.result),
        (unsigned)irEdgeService.startedVisible,
        (long)irEdgeService.seenCounts,
        (irEdgeService.seenCounts < 0) ? -1L : (long)irEdgeService.seenRaw,
        (long)irEdgeService.clearCounts,
        (irEdgeService.clearCounts < 0) ? -1L : (long)irEdgeService.clearRaw,
        (unsigned)irEdgeService.unusableScans,
        (unsigned)irEdgeService.ambiguousScans,
        (unsigned)irEdgeService.missedScans,
        (unsigned long)irEdgeService.readyMs,
        AppComm_IrEdgeFailString(&irEdgeService)
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(movementSummaryResponse)))
    {
        AppComm_Send(movementSummaryResponse);
    }

    return true;
}

static bool AppComm_TrySendIrGet(uint32_t now)
{
    AppIrReading reading;
    uint32_t ticksPerMs;
    unsigned long ageMs;
    char left[8];
    char right[8];
    char age[8];
    int length;

    if (irGetPending == 0U)
    {
        return false;
    }

    irGetPending--;

    AppIr_GetReading(&reading, now);

    AppComm_IrDistanceField(left, sizeof(left), &reading, APP_IR_SENSOR_LEFT);
    AppComm_IrDistanceField(right, sizeof(right), &reading, APP_IR_SENSOR_RIGHT);

    ticksPerMs = osKernelGetTickFreq() / 1000U;

    if (ticksPerMs == 0U)
    {
        ticksPerMs = 1U;
    }

    if (reading.ageTicks == UINT32_MAX)
    {
        (void)snprintf(age, sizeof(age), "NA");
    }
    else
    {
        ageMs = (unsigned long)(reading.ageTicks / ticksPerMs);

        if (ageMs > APP_COMM_IR_AGE_CAP_MS)
        {
            ageMs = APP_COMM_IR_AGE_CAP_MS;
        }

        (void)snprintf(age, sizeof(age), "%lu", ageMs);
    }

    length = snprintf(
        movementSummaryResponse,
        sizeof(movementSummaryResponse),
        "IR,S:%lu,ST:%s,L:%s,R:%s,AGE:%s,F:%02X\n",
        (unsigned long)reading.sequence,
        AppComm_IrAcqString(reading.acq),
        left,
        right,
        age,
        (unsigned)reading.faultMask
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(movementSummaryResponse)))
    {
        AppComm_Send(movementSummaryResponse);
    }

    return true;
}
#endif /* APP_IR_ENABLE */

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
#if APP_ULTRASONIC_BENCH_LOG
    lastUltrasonicLogTick = lastTelemetryServiceTick;
#endif
#if APP_IR_BENCH_LOG
    lastIrLogTick = lastTelemetryServiceTick;
#endif
#if APP_IR_ENABLE
    irGetPending = 0U;
    irGetDropped = 0U;
    irGetPendingHighWater = 0U;
    lastIrEdgeSequence = 0U;
#endif
    usdGetPending = 0U;
    usdGetDropped = 0U;
    usdGetPendingHighWater = 0U;
    lastTelemetryRunSequence = 0U;
    lastKnownDirection = MOTION_STOP;
    lastKnownStartTick = 0U;
    movementStartEmitted = false;
    stateCommandReserved = 0U;
    stopOutstanding = false;
    priorityStopPending = 0U;
    motionQueueFenced = 0U;
    stopQueueDrainFaults = 0U;
    AppUsPark_Init(&usPark);
    lastParkingServiceTick = osKernelGetTickCount();
#if APP_USPARK_TRACE
    AppParkTrace_Init();
    parkTraceSeqValid = false;
    parkTraceFormatFailures = 0U;
#endif
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

/* MotionTask command source (replaces the bare osMessageQueueGet() in
 * StartMotionTask). Priority order:
 *   1. a pending priority STOP is consumed (exactly once) and returned as
 *      { MOTION_STOP, 0 } -- the normal queue is not read this loop;
 *   2. while the queue fence is raised the queue is drained again and is
 *      NOT read; the fence drops only once the queue is verified empty;
 *   3. otherwise the normal queue is read exactly as before.
 * Called only from MotionTask. Never blocks. */
bool AppComm_TakeMotionCommand(MotionCommand *command)
{
    uint8_t stopPending;
    uint8_t fenced;

    if (command == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    stopPending = priorityStopPending;
    priorityStopPending = 0U;
    fenced = motionQueueFenced;
    taskEXIT_CRITICAL();

    if ((stopPending != 0U) || (fenced != 0U))
    {
        bool drained = AppComm_DrainMotionQueue();

        taskENTER_CRITICAL();

        if (drained && (priorityStopPending == 0U))
        {
            motionQueueFenced = 0U;
        }

        taskEXIT_CRITICAL();

        if (stopPending != 0U)
        {
#if APP_USPARK_TRACE
            /* Recorded only for an automatic stop the trace saw accepted. */
            AppComm_TraceTake();
#endif
            command->type = MOTION_STOP;
            command->value = 0;
            return true;
        }

        return false;
    }

    return (osMessageQueueGet(
                MotionQueueHandle,
                command,
                NULL,
                0U) == osOK);
}

void AppComm_ServiceTelemetry(void)
{
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

    AppMotion_GetEvent(&serviceMotionEvent);

    if (serviceMotionEvent.sequence !=
        lastMotionEventSequence)
    {
        lastMotionEventSequence =
            serviceMotionEvent.sequence;

        if (stopOutstanding)
        {
            /* A STOP is outstanding: only its own STOPPED event is
             * reported. Anything else is superseded and is neither sent
             * nor allowed to clear the STOP reservation. */
            if (serviceMotionEvent.type == APP_MOTION_EVENT_STOPPED)
            {
                AppComm_ReportMotionEvent(&serviceMotionEvent);
                stopOutstanding = false;
                stateCommandReserved = 0U;
            }

            return;
        }

        stateCommandReserved = 0U;

        /* Unified movement telemetry: MM,EV:END alongside (never in
         * place of) the existing, unmodified ACK/ERR dispatch below --
         * internally gated on movementStartEmitted, a no-op for any
         * event that isn't a real movement's terminal outcome. */
        AppComm_ReportMotionEvent(&serviceMotionEvent);
        return;
    }

    AppMotion_GetTelemetry(&serviceTelemetry);

    /* Unified movement telemetry: MM,EV:START, derived purely from
     * tuneTelemetry.runSequence changing while a MOV,F/B/FL/FR/BL/BR
     * command is newly active (APP_TUNE_MOV_DISTANCE/APP_TUNE_MOV_TURN
     * only -- TUNE_LEFT/RIGHT/BOTH/STEERING also bump runSequence but
     * are excluded here). lastTelemetryRunSequence always advances
     * below regardless, so a later genuine MOV arming is still
     * detected as a change even after an intervening TUNE_* run. */
    if (!stopOutstanding &&
        AppComm_TrySendMovementStart(&serviceTelemetry))
    {
        return;
    }

    lastTelemetryRunSequence = serviceTelemetry.runSequence;

    if (serviceTelemetry.completionSequence !=
        lastCompletionSequence)
    {
        lastCompletionSequence =
            serviceTelemetry.completionSequence;

        if (!stopOutstanding)
        {
            stateCommandReserved = 0U;

            AppComm_SendTuneResult(
                serviceTelemetry.completedMode,
                serviceTelemetry.completedResult
            );
        }

        return;
    }

#if APP_IR_ENABLE
    /* IR,GET reply (Phase 3): after every STOP / event / START / completion
     * branch above, so a terminal reply that is ready in this pass always
     * goes out first and the reply simply waits one more pass. Never while a
     * STOP is outstanding (it follows ACK,STOPPED), never between MM,EV:END
     * and its reply (those two are sent back to back by the event branch).
     * At most one line per pass; the bench lines below wait their turn.
     * The one-shot IREDGE, decision line of MOV,FIRL/FIRR goes first, under
     * the same gating. */
    if (!stopOutstanding && AppComm_TrySendIrEdge())
    {
        return;
    }

    if (!stopOutstanding && AppComm_TrySendIrGet(now))
    {
        return;
    }
#endif

    /* USD,GET reply: same gating and slot as the IR,GET reply above (after
     * it, ahead of the bench lines). */
    if (!stopOutstanding && AppComm_TrySendUsdGet(now))
    {
        return;
    }

#if APP_ULTRASONIC_BENCH_LOG
    /* Bench-only "US," line: after every STOP / event / START / completion
     * branch above (so it can never fall between MM,EV:END and
     * ACK,STOPPED), and not at all while a STOP is outstanding. */
    if (!stopOutstanding && AppComm_TrySendUltrasonicLog(now))
    {
        return;
    }
#endif

#if APP_IR_BENCH_LOG
    /* Bench-only "IRRAW," line: same placement and gating as "US," above
     * (never while a STOP is outstanding, never between MM,EV:END and its
     * terminal reply, never ahead of an event/START/completion due in this
     * pass), and at most one bench line per pass. */
    if (!stopOutstanding && AppComm_TrySendIrLog(now))
    {
        return;
    }
#endif

    if (serviceTelemetry.active == 0U)
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

    AppComm_SendTuneTelemetry(&serviceTelemetry);
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

/* STOP callback from vision_rx.c. Runs in CommTask. Parsing only told us
 * the line was exactly "STOP"; everything else happens here:
 *  - a STOP while one is already outstanding is coalesced (no effect);
 *  - in one critical section MotionTask cannot run, so the unread event /
 *    telemetry snapshot, the gate, the priority latch, the queue fence and
 *    the first queue drain are one atomic step;
 *  - MotionTask later dispatches { MOTION_STOP, 0 } through
 *    AppComm_TakeMotionCommand() before any queued command. This function
 *    never calls AppMotion_HandleCommand() and never touches the motors;
 *  - results already published before the snapshot are still reported,
 *    before the eventual ACK,STOPPED. */
/* The single place a priority STOP is requested. Used by the manual STOP
 * command (guard == NULL, behaviour byte-identical to before) and by the
 * USPARK automatic stop (guard != NULL). Returns true only if THIS call
 * raised the stop; a coalesced duplicate, or a guarded request that no
 * longer applies, returns false and changes nothing.
 *
 * A guarded request is additionally refused when a stop latch/fence is
 * already up, or when -- re-read inside the same critical section -- the
 * motion is no longer the same straight-forward movement the detector
 * decided about (it finished or started braking in between). That closes
 * the race that would otherwise produce an ACK followed by an unsolicited
 * ACK,STOPPED. */
/* Guard for an automatic stop, called inside the request's critical
 * section. Kept out of AppComm_RequestPriorityStop() on purpose: a local
 * here lives only for this short call, so the stop helper's own frame (the
 * deepest part of CommTask's stack on the manual STOP path) does not grow.
 * Refuses when a stop latch/fence is already up, or when the motion is no
 * longer the SAME straight-forward movement the detector decided about
 * (it finished, started braking, or was replaced: movementSeq differs). */
static bool AppComm_StopGuardRefuses(const AppCommStopGuard *guard)
{
    AppMotionMovementStatus status;

    AppMotion_GetMovementStatus(&status);

    return (priorityStopPending != 0U) ||
           (motionQueueFenced != 0U) ||
           (status.state != APP_MOTION_DISTANCE) ||
           (status.command != MOTION_FORWARD) ||
           (status.movementSeq != guard->movementSeq);
}

static inline __attribute__((always_inline))
bool AppComm_PriorityStopImpl(const AppCommStopGuard *guard)
{
    taskENTER_CRITICAL();

    if (stopOutstanding)
    {
        taskEXIT_CRITICAL();
        return false;
    }

    if ((guard != NULL) && AppComm_StopGuardRefuses(guard))
    {
        taskEXIT_CRITICAL();
        return false;
    }

    AppMotion_GetEvent(&stopEvent);
    AppMotion_GetTelemetry(&stopTelemetry);

    stopOutstanding = true;
    stateCommandReserved = 1U;
    priorityStopPending = 1U;
    motionQueueFenced = 1U;

    /* First cancellation attempt. A failure is recorded inside the helper
     * and is retried by MotionTask; the latch and fence hold either way. */
    (void)AppComm_DrainMotionQueue();

#if APP_USPARK_TRACE
    /* Automatic (guarded) stops only; recorded INSIDE the critical section so
     * MotionTask's TAKE can never be ordered before this ACC. */
    if (guard != NULL)
    {
        AppComm_TraceAccepted();
    }
#endif

    taskEXIT_CRITICAL();

    /* Pre-STOP reports, outside the critical section, in this order: the
     * unread terminal event (with its END marker) or else a pending START,
     * then an unread tune completion. */
    if (stopEvent.sequence != lastMotionEventSequence)
    {
        lastMotionEventSequence = stopEvent.sequence;
        AppComm_ReportMotionEvent(&stopEvent);
    }
    else
    {
        (void)AppComm_TrySendMovementStart(&stopTelemetry);
    }

    if (stopTelemetry.completionSequence != lastCompletionSequence)
    {
        lastCompletionSequence = stopTelemetry.completionSequence;
        AppComm_SendTuneResult(
            stopTelemetry.completedMode,
            stopTelemetry.completedResult
        );
    }

    return true;
}

/* Automatic (USPARK) entry: an ordinary call around the SAME implementation,
 * so the code exists once in the source. */
static bool AppComm_RequestPriorityStop(const AppCommStopGuard *guard)
{
    return AppComm_PriorityStopImpl(guard);
}

/* Manual STOP. The implementation is inlined here (always_inline) so the
 * parser -> on_stop chain has no extra wrapper frame. */
void on_stop(void)
{
    (void)AppComm_PriorityStopImpl(NULL);
}

/* ---- USPARK: ultrasonic parking protection -------------------------- */

static const char *AppComm_UsParkStateString(AppUsParkState state)
{
    switch (state)
    {
        case APP_USPARK_DISABLED: return "DISABLED";
        case APP_USPARK_UNARMED:  return "UNARMED";
        case APP_USPARK_ARMED:    return "ARMED";
        case APP_USPARK_NEAR1:    return "NEAR1";
        case APP_USPARK_NEAR2:    return "NEAR2";
        case APP_USPARK_LATCHED:  return "LATCHED";
        default:                  return "?";
    }
}

static const char *AppComm_UsParkCauseString(AppUsParkCause cause)
{
    switch (cause)
    {
        case APP_USPARK_CAUSE_FAULT:    return "FAULT";
        case APP_USPARK_CAUSE_NOUPDATE: return "NOUPDATE";
        case APP_USPARK_CAUSE_INVALID:  return "INVALID";
        case APP_USPARK_CAUSE_NEAR:     return "NEAR";
        default:                        return "?";
    }
}

static uint32_t AppComm_TicksToMs(uint32_t ticks)
{
    uint32_t ticksPerMs = osKernelGetTickFreq() / 1000U;

    if (ticksPerMs == 0U)
    {
        ticksPerMs = 1U;
    }

    return ticks / ticksPerMs;
}

/* Evaluated by CommTask on every 100 Hz service pass. Reads one atomic
 * motion status and one atomic ultrasonic snapshot, runs the pure detector
 * and, on a trigger, requests the SAME priority STOP as the manual command.
 * It never touches the motors and never enqueues anything. The
 * USPARK,STOP cause line is sent only if the request was accepted, after
 * the pre-STOP flush the helper performs, so it always precedes
 * MM,EV:END,...,RESULT:STOPPED and ACK,STOPPED.
 *
 * Called by StartCommTask() right after AppComm_ServiceTelemetry(), as its
 * OWN call rather than from inside the telemetry service: it then does not
 * sit under that function's large frame on CommTask's small stack. Returns
 * immediately while USPARK is disabled (the boot default), so nothing else
 * in the system changes. Self-gated to the same 100 Hz service rate. */
void AppComm_ServiceParking(void)
{
    AppUsParkInputs in;
    AppUsParkOutput out;
    uint32_t nowTick;
    uint32_t servicePeriodTicks;
    int length;

    if (AppUsPark_GetState(&usPark) == APP_USPARK_DISABLED)
    {
        return;
    }

    nowTick = osKernelGetTickCount();
    servicePeriodTicks = osKernelGetTickFreq() / APP_COMM_SERVICE_RATE_HZ;

    if (servicePeriodTicks == 0U)
    {
        servicePeriodTicks = 1U;
    }

    if ((nowTick - lastParkingServiceTick) < servicePeriodTicks)
    {
        return;
    }

    lastParkingServiceTick = nowTick;

    AppMotion_GetMovementStatus(&in.motion);
    AppUltrasonic_GetSnapshot(&parkSnapshot);

    in.nowTick = nowTick;
    in.nowMs = AppComm_TicksToMs(nowTick);
    in.stopGateActive = AppComm_StopGateActive();
    in.snapshot = &parkSnapshot;

#if APP_USPARK_TRACE
    {
        bool traceArmedBefore = usPark.armed;
        uint8_t traceNearBefore = usPark.nearCount;

        out = AppUsPark_Step(&usPark, &in);
        AppComm_TraceParkingStep(&in, out.cause, traceArmedBefore,
                                 traceNearBefore);
    }
#else
    out = AppUsPark_Step(&usPark, &in);
#endif

    if (out.notifyFault)
    {
        length = snprintf(
            movementSummaryResponse,
            sizeof(movementSummaryResponse),
            "USPARK,FAULT,FM:%02X\n",
            (unsigned)parkSnapshot.faultMask
        );

        if ((length > 0) &&
            ((size_t)length < sizeof(movementSummaryResponse)))
        {
            AppComm_Send(movementSummaryResponse);
        }
    }

    if (out.cause != APP_USPARK_CAUSE_NONE)
    {
        AppCommStopGuard guard;

        guard.movementSeq = in.motion.movementSeq;

#if APP_USPARK_TRACE
        AppComm_TraceRequest(&parkSnapshot, out.cause);
#endif

#ifdef APP_COMM_TEST_HOOK_BEFORE_PARK_STOP
        extern void APP_COMM_TEST_HOOK_BEFORE_PARK_STOP(void);
        /* Host tests ONLY (never defined in firmware): lets a test change
         * the motion state between the detector's decision and the request,
         * the window the guard in AppComm_RequestPriorityStop() closes. */
        APP_COMM_TEST_HOOK_BEFORE_PARK_STOP();
#endif

        if (AppComm_RequestPriorityStop(&guard))
        {
            char distance[12];

            AppUsPark_StopAccepted(&usPark, out.cause);

            if (parkSnapshot.filteredValid != 0U)
            {
                (void)snprintf(distance, sizeof(distance), "%lu",
                               (unsigned long)parkSnapshot.filteredMm);
            }
            else
            {
                (void)snprintf(distance, sizeof(distance), "NA");
            }

            length = snprintf(
                movementSummaryResponse,
                sizeof(movementSummaryResponse),
                "USPARK,STOP,R:%s,D:%s,V:%ld,FM:%02X\n",
                AppComm_UsParkCauseString(out.cause),
                distance,
                (long)in.motion.filteredSpeedCountsPerSec,
                (unsigned)parkSnapshot.faultMask
            );

            if ((length > 0) &&
                ((size_t)length < sizeof(movementSummaryResponse)))
            {
                AppComm_Send(movementSummaryResponse);
            }
        }
#if APP_USPARK_TRACE
        else
        {
            AppParkTrace_Refused(osKernelGetTickCount());
        }
#endif
    }
}

/* USPARK,<0|1>. Enable is idle-only (same gate as every idle-only SET) and
 * refused with ERR,US_FAULT while the ultrasonic driver is faulted; disable
 * is accepted at any time and cannot affect a STOP already accepted (the
 * detector has no access to the stop latch/fence). */
void on_us_park_set(int enable)
{
    if (enable == 0)
    {
        AppUsPark_Disable(&usPark);
#if APP_USPARK_TRACE
        parkTraceSeqValid = false;
        AppParkTrace_Disabled(osKernelGetTickCount());
#endif
        AppComm_Send("USPARK,OK\n");
        return;
    }

    if (AppComm_MotionBusy())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

    AppUltrasonic_GetSnapshot(&parkSnapshot);

    if ((parkSnapshot.state == APP_US_STATE_FAULT) ||
        (parkSnapshot.faultMask != 0U))
    {
        AppComm_Send("ERR,US_FAULT\n");
        return;
    }

    AppUsPark_Enable(
        &usPark,
        parkSnapshot.sequence,
        AppComm_TicksToMs(osKernelGetTickCount())
    );
#if APP_USPARK_TRACE
    /* Publications older than this enable are never replayed as US records. */
    parkTraceLastUsSeq = parkSnapshot.sequence;
    parkTraceSeqValid = true;
#endif
    AppComm_Send("USPARK,OK\n");
}

#if APP_IR_ENABLE
uint16_t AppComm_IrGetDroppedCount(void)
{
    return irGetDropped;
}

uint8_t AppComm_IrGetPendingHighWater(void)
{
    return irGetPendingHighWater;
}
#endif

/* IR,GET: telemetry request. The parser calls this from CommTask; it only
 * records that one reply is owed. The reply is built and sent later by
 * AppComm_ServiceTelemetry() so it can never delay or overtake a STOP
 * acknowledgement, MM,EV:END and its terminal reply, or any ACK / ERR that
 * is ready in the same pass.
 *
 * Overflow: up to APP_COMM_IR_GET_MAX_PENDING (4) replies may be owed at
 * once; a request beyond that is DISCARDED, without any reply (it is counted
 * in irGetDropped). It is deliberately not answered ERR,BUSY: the RPi reads
 * every ERR line as the terminal reply of the movement in flight, so a generic
 * ERR,BUSY caused by a telemetry poll would be mistaken for a movement
 * failure. A client that keeps ONE request outstanding (sends the next only
 * after the previous reply) never reaches the limit: the high-water mark stays
 * at 1 and nothing is dropped (proved by tests/comm/app_comm_ir_get_test.c).
 * With the IR feature compiled out the line is an unknown command. */
void on_ir_get(void)
{
#if APP_IR_ENABLE
    if (irGetPending < APP_COMM_IR_GET_MAX_PENDING)
    {
        irGetPending++;

        if (irGetPending > irGetPendingHighWater)
        {
            irGetPendingHighWater = irGetPending;
        }
    }
    else if (irGetDropped < UINT16_MAX)
    {
        irGetDropped++;         /* discarded: see the comment above */
    }
#else
    AppComm_Send("ERR,BAD_CMD\n");
#endif
}

uint16_t AppComm_UsdGetDroppedCount(void)
{
    return usdGetDropped;
}

uint8_t AppComm_UsdGetPendingHighWater(void)
{
    return usdGetPendingHighWater;
}

/* USD,GET: front ultrasonic read request (PROTOCOL.md §4.8). The parser calls
 * this from CommTask; it only records that one reply is owed, exactly like
 * on_ir_get() (same limit of APP_COMM_USD_GET_MAX_PENDING owed replies, a
 * request beyond it DISCARDED without any reply and counted, never ERR,BUSY). */
void on_usd_get(void)
{
    if (usdGetPending < APP_COMM_USD_GET_MAX_PENDING)
    {
        usdGetPending++;

        if (usdGetPending > usdGetPendingHighWater)
        {
            usdGetPendingHighWater = usdGetPending;
        }
    }
    else if (usdGetDropped < UINT16_MAX)
    {
        usdGetDropped++;        /* discarded: see on_ir_get() */
    }
}

/* GET is always available, including while moving or stopping. */
void on_us_park_get(void)
{
    int length;

    AppUltrasonic_GetSnapshot(&parkSnapshot);

    length = snprintf(
        movementSummaryResponse,
        sizeof(movementSummaryResponse),
        "USPARK,E:%u,ST:%s,CN:%u,FM:%02X,TH:%u,RA:%u,AR:%u\n",
        usPark.enabled ? 1U : 0U,
        AppComm_UsParkStateString(AppUsPark_GetState(&usPark)),
        usPark.confirmedNear ? 1U : 0U,
        (unsigned)parkSnapshot.faultMask,
        (unsigned)APP_USPARK_STOP_MM,
        (unsigned)APP_USPARK_REARM_MM,
        (unsigned)APP_USPARK_ARM_MM
    );

    if ((length > 0) &&
        ((size_t)length < sizeof(movementSummaryResponse)))
    {
        AppComm_Send(movementSummaryResponse);
    }
}

#if APP_USPARK_TRACE
/* ---- USPARK trace (measurement only, bench builds) ---------------------- *
 * Everything below builds fixed binary records and hands them to the trace
 * ring (app_park_trace.c) -- no formatting, no UART, no waiting -- except
 * AppComm_ServiceParkTrace(), which is CommTask's only transmitter of trace
 * lines and runs only when the link is idle. None of it feeds a decision.
 * ------------------------------------------------------------------------- */

static uint16_t AppComm_TraceClamp(uint32_t value, uint16_t max)
{
    return (value > max) ? max : (uint16_t)value;
}

/* Tick difference clamped for display; a "negative" difference is 0. */
static uint16_t AppComm_TraceSince(uint32_t now, uint32_t then, uint16_t max)
{
    uint32_t d = (uint32_t)(now - then);

    return ((int32_t)d < 0) ? 0U : AppComm_TraceClamp(d, max);
}

/* One new ultrasonic publication (US), and ARM / NEAR when the detector just
 * armed or counted a near sample. Called by AppComm_ServiceParking() right
 * after AppUsPark_Step(). The encoder counts on a US record are read HERE,
 * when CommTask OBSERVES the publication, so they are observation-time
 * counts (observation tick = tick + obs), not capture-time counts. */
static void AppComm_TraceParkingStep(const AppUsParkInputs *in,
                                     AppUsParkCause cause,
                                     bool armedBefore,
                                     uint8_t nearBefore)
{
    const AppUltrasonicSnapshot *s = in->snapshot;
    int32_t el = AppEncoder_Left();
    int32_t er = AppEncoder_Right();
    uint8_t nearRun = 0U;

    if (!parkTraceSeqValid)
    {
        parkTraceLastUsSeq = s->sequence;
        parkTraceSeqValid = true;
    }
    else if (s->sequence != parkTraceLastUsSeq)
    {
        parkTraceLastUsSeq = s->sequence;

        AppParkTrace_InitRecord(&parkTraceRec, APP_PARK_EV_US,
                                s->lastSampleTick, el, er);
        parkTraceRec.seq = (uint16_t)s->sequence;
        parkTraceRec.obs = AppComm_TraceSince(in->nowTick, s->lastSampleTick,
                                              9999U);

        if (s->rawValid != 0U)
        {
            parkTraceRec.raw = AppComm_TraceClamp(s->rawMm, 9999U);
        }

        if (s->filteredValid != 0U)
        {
            parkTraceRec.filt = AppComm_TraceClamp(s->filteredMm, 9999U);
            parkTraceRec.age = AppComm_TraceClamp(
                AppComm_TicksToMs(AppUltrasonic_AgeTicks(s, in->nowTick)),
                9999U);
        }

        if (s->pulseCaptured != 0U)
        {
            parkTraceRec.pulse = AppComm_TraceClamp(s->pulseUs, 0xFFFEU);
        }

        (void)AppParkTrace_Submit(&parkTraceRec);
    }

    if (!armedBefore && usPark.armed)
    {
        AppParkTrace_InitRecord(&parkTraceRec, APP_PARK_EV_ARM,
                                in->nowTick, el, er);
        parkTraceRec.w = (uint16_t)in->motion.movementSeq;
        parkTraceRec.seq = (uint16_t)s->sequence;
        parkTraceRec.fm = s->faultMask;
        parkTraceRec.arg = usPark.confirmedNear ? 1U : 0U;
        parkTraceRec.flags = (uint8_t)AppMotion_GetProfile();

        if (s->filteredValid != 0U)
        {
            parkTraceRec.filt = AppComm_TraceClamp(s->filteredMm, 9999U);
        }

        (void)AppParkTrace_Submit(&parkTraceRec);
    }

    if (cause == APP_USPARK_CAUSE_NEAR)
    {
        nearRun = (uint8_t)(nearBefore + 1U);
    }
    else if (usPark.nearCount > nearBefore)
    {
        nearRun = usPark.nearCount;
    }

    if (nearRun != 0U)
    {
        AppParkTrace_InitRecord(&parkTraceRec, APP_PARK_EV_NEAR,
                                in->nowTick, el, er);
        parkTraceRec.arg = nearRun;
        parkTraceRec.seq = (uint16_t)s->sequence;

        if (s->filteredValid != 0U)
        {
            parkTraceRec.filt = AppComm_TraceClamp(s->filteredMm, 9999U);
        }

        (void)AppParkTrace_Submit(&parkTraceRec);
    }
}

/* The automatic STOP is about to be requested (REQ). obs carries PD: the age
 * of the latest publication at the moment of the request. */
static void AppComm_TraceRequest(const AppUltrasonicSnapshot *snapshot,
                                 AppUsParkCause cause)
{
    uint32_t now = osKernelGetTickCount();

    AppParkTrace_InitRecord(&parkTraceRec, APP_PARK_EV_REQ, now,
                            AppEncoder_Left(), AppEncoder_Right());
    parkTraceRec.arg = (uint8_t)cause;
    parkTraceRec.seq = (uint16_t)snapshot->sequence;

    if (snapshot->sequence != 0U)
    {
        parkTraceRec.obs = AppComm_TraceSince(now, snapshot->lastSampleTick,
                                              9999U);
    }

    if (snapshot->filteredValid != 0U)
    {
        parkTraceRec.filt = AppComm_TraceClamp(snapshot->filteredMm, 9999U);
    }

    (void)AppParkTrace_Submit(&parkTraceRec);
}

/* ACC: the guarded priority STOP was accepted. Called from inside the
 * critical section of AppComm_PriorityStopImpl(), automatic stops only. */
static void AppComm_TraceAccepted(void)
{
    AppParkTrace_InitRecord(&parkTraceRec, APP_PARK_EV_ACC,
                            osKernelGetTickCount(),
                            AppEncoder_Left(), AppEncoder_Right());
    (void)AppParkTrace_Submit(&parkTraceRec);
}

/* TAKE: MotionTask consumed the STOP latch. Runs in MotionTask, so it has its
 * own scratch record; ignored by the trace unless this was the traced
 * automatic stop. */
static void AppComm_TraceTake(void)
{
    AppParkTrace_InitRecord(&parkTraceTakeRec, APP_PARK_EV_TAKE,
                            osKernelGetTickCount(),
                            AppEncoder_Left(), AppEncoder_Right());
    (void)AppParkTrace_Submit(&parkTraceTakeRec);
}

/* "Communication idle": nothing the trace transmitter could delay. */
static bool AppComm_TraceLinkIdle(void)
{
    AppMotion_GetEvent(&parkTraceEvent);

    return (AppMotion_GetState() == APP_MOTION_IDLE) &&
           !AppComm_StopGateActive() &&
           (stateCommandReserved == 0U) &&
           (parkTraceEvent.sequence == lastMotionEventSequence) &&
           (osMessageQueueGetCount(MotionQueueHandle) == 0U) &&
           (osMessageQueueGetCount(CommRxQueueHandle) == 0U) &&
           (visionRx.len == 0U) &&
           (visionRx.pending_cr == 0U) &&
           (visionRx.overflow == 0U) &&
           (uart3RxRecoveryState == APP_COMM_RX_SYNCHRONIZED);
}

void AppComm_ServiceParkTrace(void)
{
    uint32_t now = osKernelGetTickCount();
    AppParkTraceState state = AppParkTrace_GetState();
    int length;

    /* The traced movement is over without MotionTask having taken a stop:
     * close the window. The trace judges the state again under its lock, so a
     * stop MotionTask has already taken is never cut short here. */
    if ((state == APP_PARK_STATE_ARMED) ||
        (state == APP_PARK_STATE_REQ) ||
        (state == APP_PARK_STATE_ACC))
    {
        AppMotionMovementStatus status;

        AppMotion_GetMovementStatus(&status);

        if (!((status.state == APP_MOTION_DISTANCE) &&
              (status.command == MOTION_FORWARD) &&
              ((uint16_t)status.movementSeq == AppParkTrace_GetWindow())))
        {
            AppParkTrace_MovementEnded(now);
        }
    }

    AppParkTrace_Poll(now);

    if (!AppParkTrace_DrainReady() || !AppComm_TraceLinkIdle())
    {
        return;
    }

    /* At most one line per pass. */
    if (AppParkTrace_Pop(&parkTraceRec))
    {
        length = AppParkTrace_Format(
            &parkTraceRec,
            movementSummaryResponse,
            sizeof(movementSummaryResponse));

        if (length > 0)
        {
            AppComm_Send(movementSummaryResponse);
        }
        else if (parkTraceFormatFailures < UINT32_MAX)
        {
            parkTraceFormatFailures++;      /* skipped, never retried */
        }
    }
}
#endif /* APP_USPARK_TRACE */

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

#if APP_IR_ENABLE
        case VISION_MOVE_FORWARD_IR_LEFT:
            motionType = MOTION_FORWARD_IR_LEFT;
            break;

        case VISION_MOVE_FORWARD_IR_RIGHT:
            motionType = MOTION_FORWARD_IR_RIGHT;
            break;
#endif

        default:
            /* includes MOV,FIRL/FIRR when the IR feature is compiled out */
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

    /* STOP-specific gate: PID SET stays ungated during ordinary motion
     * (existing behaviour) but is refused while a stop is outstanding. */
    if (AppComm_StopGateActive())
    {
        AppComm_Send("ERR,BUSY\n");
        return;
    }

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
 * has already rejected any name other than PRECISION/STANDARD/TASK2TEST
 * before this is ever called (CUSTOM has no corresponding SET form), so the
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

    if (profile == VISION_PROFILE_TASK2TEST)
    {
        AppMotion_ApplyProfile(APP_MOTION_PROFILE_TASK2TEST);
    }
    else if (profile == VISION_PROFILE_STANDARD)
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

        case APP_MOTION_PROFILE_TASK2TEST:
            AppComm_Send("PROFILE,TASK2TEST\n");
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
 * PROFILE,PRECISION/STANDARD/TASK2TEST. */
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
