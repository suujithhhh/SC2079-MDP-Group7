/*
 * app_park_trace.h
 *
 * Measurement-only trace of AUTOMATIC ultrasonic parking stops (USPARK) during
 * an enabled forward-distance movement. Compile-time gated and OFF in
 * production:
 *
 *     #ifndef APP_USPARK_TRACE
 *     #define APP_USPARK_TRACE 0
 *     #endif
 *
 * Enable it only for a bench build with a compiler definition
 * (-DAPP_USPARK_TRACE=1). With the flag at 0 this header declares nothing,
 * app_park_trace.c / app_park_trace_fmt.c compile to empty objects and every
 * hook in app_comm.c / app_motion.c / main.c is preprocessed away.
 *
 * It records and reports; it never decides anything. No PWM, PID, motor
 * direction, timing, threshold or brake behaviour is touched, and nothing here
 * can fail in a way that reaches the control path.
 *
 * ---------------------------------------------------------------------------
 * Data flow
 * ---------------------------------------------------------------------------
 *  Producers (CommTask and MotionTask) fill a fixed-size AppParkTraceRecord
 *  and hand it to AppParkTrace_Submit(). Submit() copies it into a 48-entry
 *  ring inside one short critical section. Producers never print, allocate,
 *  wait or block; a full ring (or a per-class cap) just increments a
 *  dropped-record counter. The two events raised deep inside
 *  AppMotion_Update() on MotionTask's small stack (BRK, BEND) do not even
 *  call Submit(): they LATCH tick and encoder counts into a few static words
 *  (AppParkTrace_Latch()) and the next AppParkTrace_Sample() call, made by
 *  MotionTask right after AppMotion_Update(), submits them.
 *
 *  ONLY CommTask drains the ring (AppComm_ServiceParkTrace() in app_comm.c),
 *  formats with app_park_trace_fmt.c and transmits at most one line per
 *  eligible pass. Records are not poppable until the window's DONE record
 *  exists, and CommTask additionally requires the link to be idle (motion
 *  idle, no STOP outstanding, no pending command, no RX activity).
 *
 * ---------------------------------------------------------------------------
 * One window = one automatic parking stop
 * ---------------------------------------------------------------------------
 *   IDLE -ARM-> ARMED -REQ-> REQ -ACC-> ACC -TAKE-> TAKE -BRK-> BRAKE -> DRAIN
 *
 *  Each of ARM, REQ, ACC, TAKE, BRK, BEND, ZL, ZR, ZERO, DONE is accepted at
 *  most once per window and only from its own predecessor state, so a manual
 *  STOP (no REQ/ACC), a natural completion, a reverse movement or a turn never
 *  produces BRK/BEND/Z* records. DONE is written when BOTH the zero-speed
 *  verdict (ZERO) and the end of the existing 120 ms brake (BEND) are known,
 *  because zero speed may be confirmed before OR after BEND.
 *
 * ---------------------------------------------------------------------------
 * Capacity proof (RING_SIZE = 48)
 * ---------------------------------------------------------------------------
 *   PRE  (copy of the last valid publications at ARM)  <= 3
 *   ARM, REQ, ACC, TAKE, BRK, BEND, ZL, ZR, ZERO, DONE    10  (once each)
 *   NEAR (capped)                                          6
 *   US   (capped, after ARM)                              29
 *   ---------------------------------------------------------
 *   total                                                 48   (checked by _Static_assert)
 *
 *  The ring is only refilled when the previous window has been drained
 *  completely (a window that would start earlier is skipped and counted), so
 *  a valid session can never overflow the ring. Anything over a cap is
 *  counted in the window's DR (dropped) figure, never silently lost.
 *
 * ---------------------------------------------------------------------------
 * Time and encoders
 * ---------------------------------------------------------------------------
 *  All ticks are RTOS ticks (1 tick = 1 ms, configTICK_RATE_HZ = 1000) and all
 *  differences are wrap-safe unsigned 32-bit subtractions. Left encoder counts
 *  are the raw 32-bit TIM2 value; right counts are AppEncoder_Right() (the
 *  16-bit TIM3 value, sign-corrected) and every right-hand difference is taken
 *  modulo 2^16.
 *
 *  US / PRE records: "tick" is the PUBLICATION tick (snapshot.lastSampleTick).
 *  Their encoder counts were read when CommTask OBSERVED the publication
 *  (observation tick = tick + obs) and are therefore OBSERVATION-TIME counts,
 *  not capture-time counts. They are labelled OEL/OER on the wire for exactly
 *  that reason. Nothing in this module captures counts at publication time.
 *
 * ---------------------------------------------------------------------------
 * Zero-speed definition (MEASUREMENT ONLY, never used to release the brake)
 * ---------------------------------------------------------------------------
 *  From BRK, MotionTask samples both encoders every cycle. A wheel is
 *  "stationary" the first time its counts vary by at most one count over a
 *  50 ms rolling window that is fully covered by samples taken at or after BRK.
 *  ZL/ZR report the confirmation tick of each wheel (so the physical stop is
 *  up to 50 ms earlier); ZERO is reported when both are confirmed. If both
 *  are not confirmed within 2000 ms of BRK, ZERO is reported with the
 *  timeout flag. Motor behaviour is unaffected in every case.
 */

#ifndef INC_APP_PARK_TRACE_H_
#define INC_APP_PARK_TRACE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifndef APP_USPARK_TRACE
#define APP_USPARK_TRACE 0
#endif

#if (APP_USPARK_TRACE != 0) && (APP_USPARK_TRACE != 1)
#error "APP_USPARK_TRACE must be defined as 0 or 1"
#endif

#if APP_USPARK_TRACE

/* ---- sizes and limits ------------------------------------------------- */

#define APP_PARK_TRACE_RING_SIZE     48U
#define APP_PARK_TRACE_PRE_COUNT      3U    /* valid publications copied at ARM */
#define APP_PARK_TRACE_FIXED_COUNT   10U    /* ARM REQ ACC TAKE BRK BEND ZL ZR ZERO DONE */

#ifndef APP_PARK_TRACE_NEAR_CAP
#define APP_PARK_TRACE_NEAR_CAP       6U
#endif
#ifndef APP_PARK_TRACE_US_CAP
#define APP_PARK_TRACE_US_CAP        29U
#endif

/* Only a host test may override the caps to deliberately break the proof. */
#ifndef APP_PARK_TRACE_ALLOW_OVERCOMMIT
_Static_assert(APP_PARK_TRACE_PRE_COUNT + APP_PARK_TRACE_FIXED_COUNT +
               APP_PARK_TRACE_NEAR_CAP + APP_PARK_TRACE_US_CAP <=
               APP_PARK_TRACE_RING_SIZE,
               "worst valid window must fit the ring");
#endif

#define APP_PARK_TRACE_ZERO_WINDOW_MS     50U    /* rolling window            */
#define APP_PARK_TRACE_ZERO_MAX_COUNTS     1     /* max movement in window    */
#define APP_PARK_TRACE_TIMEOUT_MS       2000U    /* BRK (or REQ) to give up   */
#define APP_PARK_TRACE_BEND_GRACE_MS     500U    /* BEND missing after timeout */
#define APP_PARK_TRACE_LINE_MAX          120U    /* every line is < this      */

/* Unavailable markers (0 is never a sentinel). */
#define APP_PARK_TRACE_NA_U16      0xFFFFU
#define APP_PARK_TRACE_NA_V        ((int16_t)INT16_MIN)

/* ---- events ----------------------------------------------------------- */

typedef enum
{
    APP_PARK_EV_NONE = 0,
    APP_PARK_EV_US,         /* filtered ultrasonic publication               */
    APP_PARK_EV_PRE,        /* replay of a pre-ARM publication (at ARM)      */
    APP_PARK_EV_ARM,        /* parking protection arms                       */
    APP_PARK_EV_NEAR,       /* qualifying near sample (arg = run number)     */
    APP_PARK_EV_REQ,        /* automatic STOP requested (arg = cause)        */
    APP_PARK_EV_ACC,        /* priority STOP accepted by CommTask            */
    APP_PARK_EV_TAKE,       /* MotionTask consumed the STOP                  */
    APP_PARK_EV_BRK,        /* electrical brake begins                       */
    APP_PARK_EV_BEND,       /* the existing 120 ms brake interval ended      */
    APP_PARK_EV_ZL,         /* left wheel stationary (confirmation tick)     */
    APP_PARK_EV_ZR,         /* right wheel stationary                        */
    APP_PARK_EV_ZERO,       /* both stationary, or timeout (flag bit 0)      */
    APP_PARK_EV_DONE        /* complete trace ready                          */
} AppParkTraceEvent;

/* REQ.arg: numerically equal to AppUsParkCause (app_comm.c asserts it). */
#define APP_PARK_CAUSE_FAULT      1U
#define APP_PARK_CAUSE_NOUPDATE   2U
#define APP_PARK_CAUSE_INVALID    3U
#define APP_PARK_CAUSE_NEAR       4U

/* DONE.arg */
typedef enum
{
    APP_PARK_RES_OK = 0,    /* BEND and ZERO both seen, no timeout           */
    APP_PARK_RES_TIMEOUT,   /* a bounded wait expired                        */
    APP_PARK_RES_NOREQ,     /* movement ended without an automatic stop      */
    APP_PARK_RES_NOACC,     /* REQ made but the movement ended before ACC    */
    APP_PARK_RES_REFUSED,   /* the guarded STOP request was refused          */
    APP_PARK_RES_ABORT      /* USPARK disabled while armed                   */
} AppParkTraceResult;

#define APP_PARK_FLAG_ZERO_TIMEOUT  0x01U   /* ZERO / DONE                    */

/* ---- the fixed-size binary record (36 bytes) -------------------------- */

typedef struct
{
    uint32_t tick;      /* event tick (US/PRE: publication tick)             */
    int32_t  el;        /* left encoder (US/PRE: at observation time)        */
    int32_t  er;        /* right encoder, AppEncoder_Right() value           */
    uint16_t seq;       /* ultrasonic publication sequence, low 16 bits      */
    uint16_t w;         /* window id = movementSeq low 16 bits (ARM, DONE)   */
    uint16_t raw;       /* raw mm | NA            ; DONE: dropped (window)   */
    uint16_t filt;      /* filtered mm | NA       ; DONE: ring-full drops    */
    uint16_t age;       /* ms since last valid sample | NA                   */
    uint16_t pulse;     /* echo pulse width us | NA                          */
    uint16_t obs;       /* US/PRE: observation delay ms; REQ: PD ms          */
    uint16_t fm;        /* ARM: ultrasonic fault mask; DONE: skipped windows */
    int16_t  vl;        /* signed counts/s (REQ, BRK) | NA                   */
    int16_t  vr;
    uint8_t  n;         /* index of the record within its window, from 1     */
    uint8_t  ev;        /* AppParkTraceEvent                                 */
    uint8_t  arg;       /* NEAR run / REQ cause / ARM CN / DONE result       */
    uint8_t  flags;     /* ARM: profile (0..3); ZERO/DONE: bit 0 = timeout   */
} AppParkTraceRecord;

_Static_assert(sizeof(AppParkTraceRecord) == 36U, "trace record is 36 bytes");

typedef enum
{
    APP_PARK_STATE_IDLE = 0,
    APP_PARK_STATE_ARMED,
    APP_PARK_STATE_REQ,
    APP_PARK_STATE_ACC,
    APP_PARK_STATE_TAKE,
    APP_PARK_STATE_BRAKE,
    APP_PARK_STATE_DRAIN
} AppParkTraceState;

typedef struct
{
    AppParkTraceState state;
    uint16_t window;            /* id of the open window                     */
    uint8_t  stored;            /* records currently in the ring             */
    uint16_t windowDropped;     /* records not stored this window (any cause)*/
    uint16_t windowRingFull;    /* ... of which ring-full                    */
    uint32_t lifetimeDropped;   /* since boot / Init                         */
    uint16_t windowsSkipped;    /* ARM refused: previous trace not drained   */
} AppParkTraceStats;

/* ---- API --------------------------------------------------------------- */

void AppParkTrace_Init(void);

/* Initialises *r for event `ev`: every field zero except the unavailable
 * markers (NA) on the optional fields. Pure, no locking. */
void AppParkTrace_InitRecord(AppParkTraceRecord *r, uint8_t ev, uint32_t tick,
                             int32_t el, int32_t er);

/* Hands one record to the trace. Which events are accepted, when, how often
 * and into which window is decided here (see the state machine above). For
 * REQ and BRK the signed wheel velocities are filled in from the module's own
 * recent encoder samples. Never blocks, never prints; the return value is
 * informational only and must not influence any control decision. */
bool AppParkTrace_Submit(const AppParkTraceRecord *r);

/* MotionTask only. For BRK / BEND, which are raised deep inside
 * AppMotion_Update() on MotionTask's small stack: records tick and both
 * encoder counts in a few static words and returns at once (no lock, no
 * call, no record). The next AppParkTrace_Sample() submits them, BRK before
 * BEND, so they are in the ring before that cycle's own sample is processed. */
void AppParkTrace_Latch(uint8_t ev, uint32_t tick, int32_t el, int32_t er);

/* MotionTask, once per cycle after AppMotion_Update(). Submits latched BRK /
 * BEND events, then (while a window is open) feeds the velocity estimate and,
 * from BRK, the zero-speed detector. Returns at once when nothing is open. */
void AppParkTrace_Sample(uint32_t nowTick, int32_t el, int32_t er);

/* CommTask, every pass: expires bounded waits (REQ..TAKE, and BEND/ZERO). */
void AppParkTrace_Poll(uint32_t nowTick);

/* CommTask observed that the traced movement is no longer the active forward
 * distance movement. Closes the window with NOREQ (never requested a stop) or
 * NOACC (requested, not accepted before the movement ended). Judged against
 * the trace state under the lock, so a stop that MotionTask has already
 * taken (TAKE/BRAKE) is never cut short by a stale observation. */
void AppParkTrace_MovementEnded(uint32_t nowTick);

/* The guarded automatic STOP request was refused (closes only from REQ). */
void AppParkTrace_Refused(uint32_t nowTick);

/* USPARK was disabled: closes the window only while it is merely ARMED (an
 * accepted STOP in progress is never interrupted). */
void AppParkTrace_Disabled(uint32_t nowTick);

/* Consumer (CommTask). Records become visible only once DONE exists. */
bool AppParkTrace_DrainReady(void);
bool AppParkTrace_Pop(AppParkTraceRecord *out);

AppParkTraceState AppParkTrace_GetState(void);
uint16_t AppParkTrace_GetWindow(void);
void AppParkTrace_GetStats(AppParkTraceStats *stats);

/* Pure helpers (also used by the tests). Wrap-safe differences. */
static inline uint32_t AppParkTrace_TickDelta(uint32_t later, uint32_t earlier)
{
    return (uint32_t)(later - earlier);
}
static inline int32_t AppParkTrace_Delta32(int32_t from, int32_t to)
{
    return (int32_t)((uint32_t)to - (uint32_t)from);
}
static inline int16_t AppParkTrace_Delta16(int32_t from, int32_t to)
{
    return (int16_t)((uint16_t)((uint32_t)to) - (uint16_t)((uint32_t)from));
}

/* ---- formatter (app_park_trace_fmt.c, no I/O) ------------------------- */

/* Formats one record as a compact "USPARK,TRACE,EV:..." line ending in '\n'.
 * Returns the length (excluding the NUL) or -1 if it would not fit in `size`
 * or would not stay below APP_PARK_TRACE_LINE_MAX. Integers and fixed strings
 * only: no floating point. */
int AppParkTrace_Format(const AppParkTraceRecord *r, char *buf, size_t size);

#endif /* APP_USPARK_TRACE */

#endif /* INC_APP_PARK_TRACE_H_ */
