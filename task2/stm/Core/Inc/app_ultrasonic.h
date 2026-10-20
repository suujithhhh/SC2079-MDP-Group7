/*
 * app_ultrasonic.h
 *
 * Non-blocking HC-SR04-compatible ultrasonic ranging driver.
 *
 * Hardware (all pre-existing CubeMX configuration, none of it changed here):
 *   PC6 = US_TRIG, plain GPIO push-pull output (never an alternate function).
 *   PC7 = TIM8_CH2 Echo input capture (Echo is divided down to 3.3 V).
 *   TIM8 = free-running 16-bit counter at 1 MHz (PSC 167, ARR 65535).
 *   TIM8_CC interrupt priority 5 (masked by FreeRTOS critical sections).
 *   TIM8_CH3 = internal output compare with NO pin: it ends the 10 us
 *   Trigger pulse and then serves as the hardware no-Echo timeout.
 *
 * Division of labour:
 *   - Two bounded, integer-only interrupt entry points
 *     (AppUltrasonic_OnCapture / AppUltrasonic_OnCompare) only timestamp
 *     edges and move the state machine.
 *   - AppUltrasonic_Service() (UltrasonicTask) does all triggering,
 *     classification, conversion, filtering and publication.
 *   - Other tasks read a consistent snapshot with AppUltrasonic_GetSnapshot().
 *
 * Nothing here is connected to motion control or the stopping logic.
 */

#ifndef INC_APP_ULTRASONIC_H_
#define INC_APP_ULTRASONIC_H_

#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------- *
 * Bench logging.
 *
 * Single compile-time flag. 0 (the default) removes the whole "US," line
 * from the build. To enable calibration logging change the 0 on the next
 * line to 1 (or build with -DAPP_ULTRASONIC_BENCH_LOG=1).
 * ---------------------------------------------------------------------- */
#ifndef APP_ULTRASONIC_BENCH_LOG
#define APP_ULTRASONIC_BENCH_LOG 0
#endif

/* ---------------------------------------------------------------------- *
 * Timing and conversion constants.
 *
 * PROVISIONAL: every value below assumes a generic HC-SR04-compatible
 * sensor and has NOT been bench-validated. Calibrate on hardware before
 * relying on any of them (see the calibration procedure in the plan /
 * CODEBASE_INDEX.md). Times are TIM8 ticks == microseconds unless a name
 * says _TICKS_RTOS / _MS (RTOS ticks are 1 ms, configTICK_RATE_HZ = 1000).
 * ---------------------------------------------------------------------- */

/* The Trigger pulse is ended by a TIM8_CH3 compare event this many timer
 * ticks after PC6 goes high. The counter is read after PC6 is raised and
 * the compare is "now + 11", so the pulse is always >= 10 us (plus ISR
 * latency). Sensors accept a longer pulse; 10 us is a minimum. */
#define APP_US_TRIG_PULSE_US            11U

/* Minimum spacing between triggers, in RTOS ticks. 1 ms tick granularity
 * means "60" could be 59 ms in the worst case, so 61 guarantees >= 60 ms. */
#define APP_US_MIN_TRIGGER_TICKS_RTOS   61U

/* Hardware no-Echo timeout, measured from the START of the Trigger pulse.
 * Must stay below 65536 us (16-bit wrap ambiguity) and below the minimum
 * trigger spacing. Typical "nothing in range" Echo is ~38 ms long. */
#define APP_US_ECHO_TIMEOUT_US          45000U

/* A rising Echo must appear within this window after the END of the
 * Trigger pulse; later is LATE_RISE, earlier is EDGE_ERROR. */
#define APP_US_RISE_MAX_DELAY_US        5000U

/* Plausible pulse-width window (after the edges were captured).
 * APP_US_MIN_VALID_PULSE_US_PRODUCTION is the real firmware limit and is
 * what the compile-time guarantees in app_ultrasonic.c are checked against.
 * Only a host-test build may override APP_US_MIN_VALID_PULSE_US (to reach
 * the "result <= 0 -> TOO_NEAR" branch, which the production minimum makes
 * unreachable); firmware builds never define it. */
#define APP_US_MIN_VALID_PULSE_US_PRODUCTION 100U   /* 7 mm after offset  */
#ifndef APP_US_MIN_VALID_PULSE_US
#define APP_US_MIN_VALID_PULSE_US       APP_US_MIN_VALID_PULSE_US_PRODUCTION
#endif
#define APP_US_MAX_VALID_PULSE_US       23500U  /* 4088 mm after offset  */

/* ACCEPTED BENCH CALIBRATION (all 8 cm points excluded; every other
 * calibration distance accepted). Distance is measured to the FRONT FACES
 * of the two ultrasonic transducers; no bumper or mounting offset is added.
 *
 *   mm = ((pulse_us * 1744 + 5000) / 10000) + (-10)
 *
 * Signed 32-bit arithmetic: the numerator is never negative (floor
 * division), the offset may make the result <= 0 for tiny pulses, and a
 * result <= 0 is never VALID (TOO_NEAR). Pulses 1..60 us give <= 0 mm,
 * 60 gives 0, 61 gives 1, 100 (the production minimum) gives 7. */
#define APP_US_MM_PER_US_X10000         1744
#define APP_US_OFFSET_MM                (-10)

/* The single definition of the conversion above (pure, signed). Exposed so
 * the host tests can check it without touching production limits. */
int32_t AppUltrasonic_PulseToMm(uint16_t pulseUs);

/* A measurement is "stale" when no VALID sample was published for this
 * many RTOS ticks. */
#define APP_US_STALE_TICKS_RTOS         200U

/* Task-side watchdogs (RTOS ticks). */
#define APP_US_TRIGGER_STALL_TICKS_RTOS 2U      /* PC6 high this long: force low */
#define APP_US_CAPTURE_STALL_TICKS_RTOS 50U     /* no DONE this long: abort cycle */

/* Median filter depth (valid samples only). Do not change without
 * updating the host tests. */
#define APP_US_FILTER_DEPTH             3U

/* UltrasonicTask service period (RTOS ticks). */
#define APP_US_SERVICE_PERIOD_TICKS_RTOS 10U

/* ---------------------------------------------------------------------- *
 * Types
 * ---------------------------------------------------------------------- */

/* ISR-visible measurement state machine. */
typedef enum
{
    APP_US_STATE_IDLE = 0,      /* ready to be triggered by the task        */
    APP_US_STATE_TRIGGER_HIGH,  /* PC6 high, waiting for the CH3 compare     */
    APP_US_STATE_WAIT_RISE,     /* Trigger done, waiting for Echo rising     */
    APP_US_STATE_WAIT_FALL,     /* rising captured, waiting for Echo falling */
    APP_US_STATE_DONE,          /* outcome latched, waiting for the task     */
    APP_US_STATE_FAULT          /* start-up configuration check failed       */
} AppUltrasonicState;

/* Outcome of the most recent measurement cycle. Validity is carried HERE,
 * never by the distance value: a distance of 0 is never a sentinel. */
typedef enum
{
    APP_US_RESULT_NONE = 0,     /* nothing has been published yet            */
    APP_US_RESULT_VALID,        /* plausible edges, width inside the window  */
    APP_US_RESULT_NO_RISE,      /* hardware timeout, no rising edge          */
    APP_US_RESULT_NO_FALL,      /* rose but never fell / fell after timeout  */
    APP_US_RESULT_TOO_NEAR,     /* width below APP_US_MIN_VALID_PULSE_US     */
    APP_US_RESULT_TOO_FAR,      /* width above APP_US_MAX_VALID_PULSE_US     */
    APP_US_RESULT_EDGE_ERROR,   /* overcapture, or edge earlier than allowed */
    APP_US_RESULT_LATE_RISE,    /* rising edge after APP_US_RISE_MAX_DELAY   */
    APP_US_RESULT_ECHO_STUCK,   /* Echo already high; trigger not issued     */
    APP_US_RESULT_STALL,        /* task watchdog aborted a cycle             */
    APP_US_RESULT_FAULT         /* driver refused to run (see faultMask)     */
} AppUltrasonicResult;

/* Start-up configuration-check failure bits (AppUltrasonicSnapshot.faultMask). */
#define APP_US_FAULT_CLOCK      0x01U   /* TIM8 tick is not 1 MHz           */
#define APP_US_FAULT_ARR        0x02U   /* TIM8 ARR is not 65535            */
#define APP_US_FAULT_NVIC_PRIO  0x04U   /* TIM8_CC not maskable by RTOS     */
#define APP_US_FAULT_NVIC_OFF   0x08U   /* TIM8_CC interrupt not enabled    */
#define APP_US_FAULT_CH2        0x10U   /* CH2 not direct-TI2 / DIV1 / filter 4 */
#define APP_US_FAULT_CH3        0x20U   /* CH3 compare setup failed/unsafe  */
#define APP_US_FAULT_START      0x40U   /* HAL_TIM_IC_Start failed          */
#define APP_US_FAULT_TASK       0x80U   /* UltrasonicTask was not created   */
#define APP_US_FAULT_SELFTEST   0x100U  /* CC3IF did not set with CC3E off  */

/* Error counters saturate at UINT16_MAX. */
typedef struct
{
    uint16_t valid;
    uint16_t noRise;
    uint16_t noFall;
    uint16_t tooNear;
    uint16_t tooFar;
    uint16_t edgeError;         /* overcapture or edge before the Trigger end */
    uint16_t lateRise;
    uint16_t echoStuck;
    uint16_t stall;             /* task watchdog aborts (trigger + capture)   */
    uint16_t staleResult;       /* DONE record for a cycle no longer awaited  */
    uint16_t spurious;          /* ISR: capture event in a state without CC2  */
    uint16_t staleCompare;      /* ISR: compare event in a state without CC3  */
} AppUltrasonicCounters;

typedef struct
{
    uint32_t sequence;          /* 0 = never published; 0 skipped on wrap     */
    uint32_t cycles;            /* results published so far (any result)      */
    uint32_t lastSampleTick;    /* RTOS tick the latest result was published  */
    uint32_t lastValidTick;     /* RTOS tick of the latest VALID result       */
    AppUltrasonicResult result; /* latest cycle's outcome                     */
    AppUltrasonicState state;   /* LIVE ISR state at the time of the read     */
    uint16_t pulseUs;           /* meaningful only if pulseCaptured           */
    uint16_t rawMm;             /* meaningful only if rawValid                */
    uint16_t filteredMm;        /* meaningful only if filteredValid           */
    uint8_t pulseCaptured;      /* both edges were captured this cycle        */
    uint8_t rawValid;           /* latest cycle was VALID                     */
    uint8_t filteredValid;      /* a filtered value exists (see age/stale)    */
    uint8_t filterSamples;      /* valid samples currently in the filter      */
    uint16_t faultMask;         /* APP_US_FAULT_* when state is FAULT         */
    AppUltrasonicCounters counters;
} AppUltrasonicSnapshot;

/* ---------------------------------------------------------------------- *
 * Task API (UltrasonicTask only, except GetSnapshot which is any task)
 * ---------------------------------------------------------------------- */

/* Runs the configuration checks and arms the capture channel. Call once
 * from UltrasonicTask before the first Service(). On a failed check the
 * driver enters FAULT and never drives the Trigger pin. */
void AppUltrasonic_Init(void);

/* Call from main() when osThreadNew(StartUltrasonicTask) returned NULL. The
 * task that would have published results does not exist, so this publishes
 * a consistent FAULT snapshot (state FAULT, result FAULT, faultMask =
 * APP_US_FAULT_TASK, sequence 1) that every consumer and the bench log can
 * observe, forces PC6 low, and does NOT touch TIM8 (no capture is started).
 *
 * Pre-scheduler only: it deliberately uses no critical section (FreeRTOS's
 * critical nesting is not initialised before osKernelStart) and writes the
 * snapshot with plain stores, which is safe because nothing else runs yet.
 * Servicing is never moved into another task. */
void AppUltrasonic_ReportTaskCreateFailure(void);

/* Call periodically (APP_US_SERVICE_PERIOD_TICKS_RTOS) from UltrasonicTask
 * with osKernelGetTickCount(). Never blocks. */
void AppUltrasonic_Service(uint32_t nowTick);

/* Atomic copy of the latest published snapshot plus the live driver state
 * and ISR-owned counters. Safe from any task. */
void AppUltrasonic_GetSnapshot(AppUltrasonicSnapshot *snapshot);

/* Ticks since the latest VALID sample, or UINT32_MAX if there is none.
 * Wrap-safe. */
static inline uint32_t AppUltrasonic_AgeTicks(
    const AppUltrasonicSnapshot *snapshot,
    uint32_t nowTick)
{
    if ((snapshot == 0) || (snapshot->filteredValid == 0U))
    {
        return UINT32_MAX;
    }

    return (uint32_t)(nowTick - snapshot->lastValidTick);
}

/* True only when a filtered value exists AND it is not stale. A stale,
 * invalid, timed-out or never-measured reading is never "fresh", and the
 * distance field is never consulted for this decision. */
static inline bool AppUltrasonic_IsFresh(
    const AppUltrasonicSnapshot *snapshot,
    uint32_t nowTick)
{
    uint32_t age = AppUltrasonic_AgeTicks(snapshot, nowTick);

    return (age != UINT32_MAX) && (age <= APP_US_STALE_TICKS_RTOS);
}

/* ---------------------------------------------------------------------- *
 * Interrupt entry points. Called ONLY from the HAL callback dispatchers
 * in main.c (USER CODE region). Bounded, integer-only, no RTOS calls.
 * ---------------------------------------------------------------------- */
void AppUltrasonic_OnCapture(void);     /* TIM8 CH2 input capture event */
void AppUltrasonic_OnCompare(void);     /* TIM8 CH3 output compare event */

#endif /* INC_APP_ULTRASONIC_H_ */
