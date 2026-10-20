/*
 * app_us_park.h
 *
 * Ultrasonic parking-protection detector (USPARK). PURE: no HAL, no RTOS,
 * no I/O. CommTask feeds it one set of inputs per service pass and acts on
 * the result; it never stops anything itself.
 *
 * Scope: only for the slow FINAL Task 2 parking approach. The 300 mm
 * trigger distance is measured from the front faces of the transducers and
 * is NOT guaranteed clearance: three consecutive qualifying samples plus
 * task scheduling plus the 120 ms brake mean substantial travel after the
 * obstacle first reaches 300 mm. Enabling USPARK does not reduce speed.
 * The 300/320/500 mm values are the thresholds validated on the robot.
 * The measured chassis speed is carried only for diagnostics.
 *
 * Disabled after every boot. When disabled, ultrasonic data has no effect.
 */

#ifndef INC_APP_US_PARK_H_
#define INC_APP_US_PARK_H_

#include <stdint.h>
#include <stdbool.h>

#include "app_motion.h"
#include "app_ultrasonic.h"

/* Fixed thresholds (compile time). Distances are filtered millimetres. */
#define APP_USPARK_STOP_MM          300U    /* qualifying: <= this         */
#define APP_USPARK_REARM_MM         320U    /* confirmedNear clears: > this */
#define APP_USPARK_ARM_MM           500U    /* final approach arms: <= this */
#define APP_USPARK_NEAR_SAMPLES     3U      /* consecutive near publications */
#define APP_USPARK_INVALID_SAMPLES  3U      /* consecutive invalid, armed    */
#define APP_USPARK_NOUPDATE_MS      250U    /* armed: no publication > this  */

/* Why an automatic STOP is requested. Priority when several hold on the
 * same pass: FAULT > NOUPDATE > INVALID/NEAR (the last two come from one
 * publication and are mutually exclusive). */
typedef enum
{
    APP_USPARK_CAUSE_NONE = 0,
    APP_USPARK_CAUSE_FAULT,
    APP_USPARK_CAUSE_NOUPDATE,
    APP_USPARK_CAUSE_INVALID,
    APP_USPARK_CAUSE_NEAR
} AppUsParkCause;

typedef enum
{
    APP_USPARK_DISABLED = 0,
    APP_USPARK_UNARMED,     /* enabled; no forward movement, or not armed  */
    APP_USPARK_ARMED,       /* forward movement, final approach armed      */
    APP_USPARK_NEAR1,       /* nearCount == 1                              */
    APP_USPARK_NEAR2,       /* nearCount == 2                              */
    APP_USPARK_LATCHED      /* a stop was requested/superseded this movement */
} AppUsParkState;

typedef struct
{
    bool enabled;
    bool armed;                     /* final approach armed, CURRENT movement */
    bool confirmedNear;             /* persists after a NEAR stop until a valid
                                       reading above REARM_MM, disable, enable */
    bool faultReported;             /* USPARK,FAULT already emitted           */
    uint8_t nearCount;              /* consecutive near publications (0..3)   */
    uint8_t invalidCount;           /* consecutive invalid publications, armed */
    uint32_t lastProcessedUsSeq;    /* last ultrasonic sequence consumed      */
    uint32_t lastNewUsMs;           /* ms of the last new publication, or of
                                       the movement start if later            */
    uint32_t trackedMovementSeq;    /* active forward movement (0 = none)     */
    uint32_t latchedMovementSeq;    /* movement a stop was requested/superseded
                                       for (0 = none)                         */
} AppUsPark;

typedef struct
{
    uint32_t nowTick;               /* RTOS ticks (for AppUltrasonic_IsFresh)  */
    uint32_t nowMs;                 /* milliseconds, same instant              */
    AppMotionMovementStatus motion; /* ONE atomic read                         */
    bool stopGateActive;            /* manual/auto STOP outstanding or fenced  */
    const AppUltrasonicSnapshot *snapshot;
} AppUsParkInputs;

typedef struct
{
    AppUsParkCause cause;           /* != NONE: request exactly one STOP now   */
    bool notifyFault;               /* emit one USPARK,FAULT line              */
} AppUsParkOutput;

void AppUsPark_Init(AppUsPark *park);

/* Enable: clears every counter, latch and flag (including confirmedNear) and
 * initialises lastProcessedUsSeq so older samples can never count. The CALLER
 * enforces the idle-only and not-faulted preconditions. */
void AppUsPark_Enable(AppUsPark *park, uint32_t usSequence, uint32_t nowMs);

/* Disable: clears everything. Never touches any STOP in progress (the
 * detector has no access to the stop machinery). */
void AppUsPark_Disable(AppUsPark *park);

/* One evaluation. Returns at most one cause per call. When disabled it
 * returns immediately without reading the inputs. When a cause is returned
 * the movement is already latched and the counters cleared; the caller must
 * then attempt the guarded STOP and, if it was accepted, call
 * AppUsPark_StopAccepted(). */
AppUsParkOutput AppUsPark_Step(AppUsPark *park, const AppUsParkInputs *in);

/* The guarded STOP request for `cause` was accepted. A NEAR stop sets
 * confirmedNear. */
void AppUsPark_StopAccepted(AppUsPark *park, AppUsParkCause cause);

AppUsParkState AppUsPark_GetState(const AppUsPark *park);

#endif /* INC_APP_US_PARK_H_ */
