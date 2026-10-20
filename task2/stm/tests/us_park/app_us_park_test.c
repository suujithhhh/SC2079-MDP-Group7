/* Host test of the pure ultrasonic parking-protection detector
 * (Core/Src/app_us_park.c). No HAL, no RTOS, no motion code: the test feeds
 * AppUsPark_Step() hand-built movement statuses and ultrasonic snapshots and
 * checks every state, counter, latch and reset rule of the corrected state
 * machine (see Core/Inc/app_us_park.h for the rules).
 *
 * Time is in milliseconds; one ultrasonic publication every 70 ms.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -o tests/us_park/app_us_park_test.exe \
 *       tests/us_park/app_us_park_test.c Core/Src/app_us_park.c
 *   ./tests/us_park/app_us_park_test.exe
 */

#include "app_us_park.h"

#include <stdio.h>
#include <string.h>

static unsigned checks;
static unsigned failures;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        failures++; \
        printf("FAIL line %d: %s\n", __LINE__, #expr); \
    } \
} while (0)

#define CHECK_EQ(actual, expected) do { \
    long a_ = (long)(actual); \
    long e_ = (long)(expected); \
    checks++; \
    if (a_ != e_) { \
        failures++; \
        printf("FAIL line %d: %s == %ld, expected %ld\n", \
               __LINE__, #actual, a_, e_); \
    } \
} while (0)

#define NONE   APP_USPARK_CAUSE_NONE
#define FAULT  APP_USPARK_CAUSE_FAULT
#define NOUPD  APP_USPARK_CAUSE_NOUPDATE
#define INVAL  APP_USPARK_CAUSE_INVALID
#define NEARC  APP_USPARK_CAUSE_NEAR

static AppUsPark park;
static AppUltrasonicSnapshot snap;
static AppMotionMovementStatus motion;
static bool gate;
static uint32_t nowMs;
static uint32_t usSeq;

static AppUsParkOutput Poll(void)
{
    AppUsParkInputs in;

    in.nowTick = nowMs;
    in.nowMs = nowMs;
    in.motion = motion;
    in.stopGateActive = gate;
    in.snapshot = &snap;
    return AppUsPark_Step(&park, &in);
}

static void Begin(void)
{
    AppUsPark_Init(&park);
    memset(&snap, 0, sizeof(snap));
    memset(&motion, 0, sizeof(motion));
    snap.state = APP_US_STATE_IDLE;
    nowMs = 100000U;
    usSeq = 100U;
    snap.sequence = usSeq;
    motion.state = APP_MOTION_IDLE;
    motion.command = MOTION_STOP;
    gate = false;
    AppUsPark_Enable(&park, snap.sequence, nowMs);
}

static void SetMotion(AppMotionState state, MotionType command, uint32_t seq)
{
    motion.state = state;
    motion.command = command;
    motion.movementSeq = seq;
}

/* A straight-forward movement; the first Poll() only starts tracking it. */
static void StartForward(uint32_t seq)
{
    SetMotion(APP_MOTION_DISTANCE, MOTION_FORWARD, seq);
    {
        AppUsParkOutput out = Poll();
        CHECK(out.cause == NONE);
    }
}

/* New publication (fresh), then one evaluation. */
static AppUsParkOutput PublishValid(uint16_t mm)
{
    nowMs += 70U;
    usSeq++;
    snap.sequence = usSeq;
    snap.result = APP_US_RESULT_VALID;
    snap.filteredValid = 1U;
    snap.filteredMm = mm;
    snap.rawValid = 1U;
    snap.rawMm = mm;
    snap.lastValidTick = nowMs;
    snap.faultMask = 0U;
    snap.state = APP_US_STATE_IDLE;
    return Poll();
}

/* New publication that is not a valid reading (no Echo). */
static AppUsParkOutput PublishInvalid(void)
{
    nowMs += 70U;
    usSeq++;
    snap.sequence = usSeq;
    snap.result = APP_US_RESULT_NO_RISE;
    snap.rawValid = 0U;
    /* The filter still holds an old value; it must not matter. */
    snap.faultMask = 0U;
    snap.state = APP_US_STATE_IDLE;
    return Poll();
}

/* New publication that says VALID but whose sample is already stale. */
static AppUsParkOutput PublishValidButStale(uint16_t mm)
{
    AppUsParkOutput out;

    nowMs += 70U;
    usSeq++;
    snap.sequence = usSeq;
    snap.result = APP_US_RESULT_VALID;
    snap.filteredValid = 1U;
    snap.filteredMm = mm;
    snap.lastValidTick = nowMs - (APP_US_STALE_TICKS_RTOS + 50U);
    snap.faultMask = 0U;
    out = Poll();
    return out;
}

static void ExpectState(AppUsParkState state)
{
    CHECK_EQ(AppUsPark_GetState(&park), state);
}

/* ------------------------------------------------------------------ */

static void test_disabled(void)
{
    unsigned i;

    Begin();
    AppUsPark_Disable(&park);
    ExpectState(APP_USPARK_DISABLED);

    SetMotion(APP_MOTION_DISTANCE, MOTION_FORWARD, 7U);

    for (i = 0U; i < 10U; i++)
    {
        AppUsParkOutput out = PublishValid(150U);
        CHECK(out.cause == NONE);
        CHECK(!out.notifyFault);
    }

    snap.faultMask = 0x05U;
    {
        AppUsParkOutput out = Poll();
        CHECK(out.cause == NONE);
        CHECK(!out.notifyFault);                /* silent when disabled */
    }

    CHECK_EQ(park.nearCount, 0);
    CHECK_EQ(park.trackedMovementSeq, 0);
    ExpectState(APP_USPARK_DISABLED);
}

static void test_near_threshold_and_debounce(void)
{
    AppUsParkOutput out;

    /* 301 mm never counts; it only arms. */
    Begin();
    StartForward(1U);
    ExpectState(APP_USPARK_UNARMED);
    out = PublishValid(301U);
    CHECK(out.cause == NONE);
    ExpectState(APP_USPARK_ARMED);
    CHECK_EQ(park.nearCount, 0);
    out = PublishValid(301U);
    CHECK(out.cause == NONE);
    CHECK_EQ(park.nearCount, 0);

    /* Exactly 300 mm counts; the third consecutive one triggers. */
    out = PublishValid(300U);
    CHECK(out.cause == NONE);
    ExpectState(APP_USPARK_NEAR1);
    out = PublishValid(300U);
    CHECK(out.cause == NONE);
    ExpectState(APP_USPARK_NEAR2);
    out = PublishValid(150U);
    CHECK(out.cause == NEARC);
    ExpectState(APP_USPARK_LATCHED);
    CHECK_EQ(park.nearCount, 0);

    /* Never earlier than the third, with several starting distances. */
    Begin();
    StartForward(1U);
    out = PublishValid(10U);
    CHECK(out.cause == NONE);
    out = PublishValid(10U);
    CHECK(out.cause == NONE);
    out = PublishValid(10U);
    CHECK(out.cause == NEARC);
}

static void test_same_sequence_never_counts_twice(void)
{
    unsigned i;
    AppUsParkOutput out;

    Begin();
    StartForward(1U);
    out = PublishValid(190U);
    CHECK(out.cause == NONE);
    CHECK_EQ(park.nearCount, 1);

    for (i = 0U; i < 5U; i++)
    {
        nowMs += 10U;
        out = Poll();                           /* same publication */
        CHECK(out.cause == NONE);
    }

    CHECK_EQ(park.nearCount, 1);
    ExpectState(APP_USPARK_NEAR1);
}

static void test_alternating_is_not_consecutive(void)
{
    AppUsParkOutput out;

    /* 190, invalid, 190, invalid, 190 must NOT trigger NEAR. */
    Begin();
    StartForward(1U);
    out = PublishValid(190U);
    CHECK(out.cause == NONE);
    CHECK_EQ(park.nearCount, 1);
    out = PublishInvalid();
    CHECK(out.cause == NONE);
    CHECK_EQ(park.nearCount, 0);
    CHECK_EQ(park.invalidCount, 1);
    out = PublishValid(190U);
    CHECK(out.cause == NONE);
    CHECK_EQ(park.nearCount, 1);
    CHECK_EQ(park.invalidCount, 0);             /* valid resets invalid */
    out = PublishInvalid();
    CHECK(out.cause == NONE);
    CHECK_EQ(park.nearCount, 0);
    out = PublishValid(190U);
    CHECK(out.cause == NONE);
    CHECK_EQ(park.nearCount, 1);
    ExpectState(APP_USPARK_NEAR1);

    /* Stale-but-VALID samples break the run too. */
    Begin();
    StartForward(1U);
    out = PublishValid(190U);
    out = PublishValid(190U);
    CHECK_EQ(park.nearCount, 2);
    out = PublishValidButStale(190U);
    CHECK(out.cause == NONE);
    CHECK_EQ(park.nearCount, 0);
}

static void test_valid_above_threshold_resets_debounce(void)
{
    AppUsParkOutput out;

    Begin();
    StartForward(1U);
    out = PublishValid(190U);
    out = PublishValid(190U);
    CHECK_EQ(park.nearCount, 2);
    out = PublishValid(301U);
    CHECK(out.cause == NONE);
    CHECK_EQ(park.nearCount, 0);

    out = PublishValid(190U);
    CHECK(out.cause == NONE);
    out = PublishValid(190U);
    CHECK(out.cause == NONE);
    out = PublishValid(190U);
    CHECK(out.cause == NEARC);                  /* three NEW consecutive */
}

static void test_arming_and_invalid_failsafe(void)
{
    AppUsParkOutput out;
    unsigned i;

    /* Far readings do not arm; invalid samples before arming never stop. */
    Begin();
    StartForward(1U);
    out = PublishValid(900U);
    ExpectState(APP_USPARK_UNARMED);
    CHECK(!park.armed);

    for (i = 0U; i < 20U; i++)
    {
        out = PublishInvalid();
        CHECK(out.cause == NONE);
    }

    CHECK_EQ(park.invalidCount, 0);

    /* 501 does not arm, 500 does. */
    out = PublishValid(501U);
    CHECK(!park.armed);
    out = PublishValid(500U);
    CHECK(park.armed);
    ExpectState(APP_USPARK_ARMED);

    /* Armed: valid readings above 500 disarm again. */
    out = PublishValid(550U);
    CHECK(!park.armed);
    out = PublishValid(499U);
    CHECK(park.armed);

    /* Armed: the third consecutive invalid publication triggers INVALID. */
    out = PublishInvalid();
    CHECK(out.cause == NONE);
    CHECK_EQ(park.invalidCount, 1);
    out = PublishInvalid();
    CHECK(out.cause == NONE);
    CHECK_EQ(park.invalidCount, 2);
    out = PublishInvalid();
    CHECK(out.cause == INVAL);
    ExpectState(APP_USPARK_LATCHED);

    /* A valid publication in between restarts the invalid count. */
    Begin();
    StartForward(1U);
    out = PublishValid(400U);
    out = PublishInvalid();
    out = PublishInvalid();
    CHECK_EQ(park.invalidCount, 2);
    out = PublishValid(400U);
    CHECK_EQ(park.invalidCount, 0);
    out = PublishInvalid();
    out = PublishInvalid();
    CHECK(out.cause == NONE);
    out = PublishInvalid();
    CHECK(out.cause == INVAL);

    /* A valid-but-stale publication is an invalid one for this purpose. */
    Begin();
    StartForward(1U);
    out = PublishValid(400U);
    out = PublishValidButStale(400U);
    out = PublishValidButStale(400U);
    CHECK(out.cause == NONE);
    out = PublishValidButStale(400U);
    CHECK(out.cause == INVAL);
}

static void test_stale_poll_does_not_trigger(void)
{
    AppUsParkOutput out;
    unsigned i;

    Begin();
    StartForward(1U);
    out = PublishValid(400U);                   /* armed */

    /* The sample ages past the freshness limit, but no NEW publication and
     * less than 250 ms: repeated polls change nothing and never trigger. */
    for (i = 0U; i < 24U; i++)
    {
        nowMs += 10U;
        out = Poll();
        CHECK(out.cause == NONE);
    }

    CHECK_EQ(park.invalidCount, 0);
    CHECK_EQ(park.nearCount, 0);
}

static void test_noupdate_boundaries(void)
{
    AppUsParkOutput out;

    Begin();
    StartForward(1U);
    out = PublishValid(400U);                   /* armed; last new = now */
    CHECK(out.cause == NONE);

    nowMs += 249U;
    out = Poll();
    CHECK(out.cause == NONE);                   /* 249 ms */
    nowMs += 1U;
    out = Poll();
    CHECK(out.cause == NONE);                   /* exactly 250 ms */
    nowMs += 1U;
    out = Poll();
    CHECK(out.cause == NOUPD);                  /* 251 ms: trigger once */
    ExpectState(APP_USPARK_LATCHED);

    nowMs += 500U;
    out = Poll();
    CHECK(out.cause == NONE);                   /* latched: only once */

    /* Before arming, no publication for ages never stops. */
    Begin();
    StartForward(1U);
    out = PublishValid(900U);                   /* valid but far: unarmed */
    nowMs += 100000U;
    out = Poll();
    CHECK(out.cause == NONE);

    /* The 250 ms clock starts at the movement start, not at an old
     * publication from before the movement. */
    Begin();
    out = PublishValid(400U);                   /* published while idle */
    nowMs += 5000U;
    StartForward(1U);                           /* tracking starts here */
    park.armed = true;                          /* force armed for timing */
    nowMs += 249U;
    out = Poll();
    CHECK(out.cause == NONE);
    nowMs += 2U;
    out = Poll();
    CHECK(out.cause == NOUPD);

    /* Wrap-safe millisecond arithmetic. */
    Begin();
    nowMs = 0xFFFFFF00U;
    park.lastNewUsMs = nowMs;
    StartForward(1U);
    out = PublishValid(400U);
    nowMs += 250U;                              /* wraps past zero */
    out = Poll();
    CHECK(out.cause == NONE);
    nowMs += 1U;
    out = Poll();
    CHECK(out.cause == NOUPD);
}

static void test_fault(void)
{
    AppUsParkOutput out;

    /* A diagnosed fault stops forward motion even when not armed, notifies
     * exactly once, and wins over every other cause. */
    Begin();
    SetMotion(APP_MOTION_DISTANCE, MOTION_FORWARD, 1U);
    snap.faultMask = 0x05U;
    snap.state = APP_US_STATE_FAULT;
    out = Poll();                               /* movement start pass */
    CHECK(out.cause == NONE);
    CHECK(out.notifyFault);
    out = Poll();
    CHECK(out.cause == FAULT);
    CHECK(!out.notifyFault);
    out = Poll();
    CHECK(out.cause == NONE);                   /* one stop per movement */
    CHECK(!out.notifyFault);                    /* not repeated */

    /* The same persisting fault stops the NEXT movement too, still with a
     * single notification. */
    SetMotion(APP_MOTION_DISTANCE, MOTION_FORWARD, 2U);
    out = Poll();
    CHECK(out.cause == NONE);
    CHECK(!out.notifyFault);
    out = Poll();
    CHECK(out.cause == FAULT);
    CHECK(!out.notifyFault);

    /* A healthy, fault-mask-zero publication ends the episode; a later
     * fault is reported again. */
    SetMotion(APP_MOTION_IDLE, MOTION_STOP, 2U);
    out = Poll();
    snap.faultMask = 0U;
    snap.state = APP_US_STATE_IDLE;
    out = PublishValid(900U);
    CHECK(!out.notifyFault);
    CHECK(!park.faultReported);
    snap.faultMask = 0x10U;
    out = Poll();
    CHECK(out.notifyFault);

    /* Fault while NOT in forward motion: reported, but nothing to stop. */
    Begin();
    snap.faultMask = 0x01U;
    out = Poll();
    CHECK(out.cause == NONE);
    CHECK(out.notifyFault);

    /* FAULT wins over NOUPDATE and over a would-be NEAR/INVALID. */
    Begin();
    StartForward(1U);
    out = PublishValid(400U);
    nowMs += 400U;
    snap.faultMask = 0x20U;
    out = Poll();
    CHECK(out.cause == FAULT);

    Begin();
    StartForward(1U);
    out = PublishValid(150U);
    out = PublishValid(150U);
    nowMs += 70U;
    usSeq++;
    snap.sequence = usSeq;                      /* would be the 3rd near */
    snap.faultMask = 0x20U;
    out = Poll();
    CHECK(out.cause == FAULT);

    /* A fault-state snapshot with a zero mask is still a fault. */
    Begin();
    StartForward(1U);
    snap.state = APP_US_STATE_FAULT;
    out = Poll();
    CHECK(out.cause == FAULT);
}

static void test_one_stop_per_movement_and_new_movement(void)
{
    AppUsParkOutput out;

    Begin();
    StartForward(1U);
    (void)PublishValid(150U);
    (void)PublishValid(150U);
    out = PublishValid(150U);
    CHECK(out.cause == NEARC);
    AppUsPark_StopAccepted(&park, NEARC);

    /* Same movement: nothing more, however near. */
    out = PublishValid(100U);
    CHECK(out.cause == NONE);
    out = PublishValid(100U);
    out = PublishValid(100U);
    out = PublishValid(100U);
    CHECK(out.cause == NONE);
    ExpectState(APP_USPARK_LATCHED);

    /* New forward movement: latch and counters cleared, so it can stop
     * again once three NEW consecutive near samples arrive. */
    SetMotion(APP_MOTION_DISTANCE, MOTION_FORWARD, 2U);
    out = Poll();
    CHECK(out.cause == NONE);
    CHECK_EQ(park.latchedMovementSeq, 0);
    CHECK_EQ(park.nearCount, 0);
    CHECK_EQ(park.invalidCount, 0);
    out = PublishValid(100U);
    out = PublishValid(100U);
    CHECK(out.cause == NONE);
    out = PublishValid(100U);
    CHECK(out.cause == NEARC);
}

static void test_manual_stop_outstanding_wins(void)
{
    AppUsParkOutput out;

    Begin();
    StartForward(1U);
    (void)PublishValid(150U);
    (void)PublishValid(150U);
    gate = true;                                /* a STOP is outstanding */
    out = PublishValid(150U);
    CHECK(out.cause == NONE);                   /* manual STOP wins */
    ExpectState(APP_USPARK_LATCHED);

    gate = false;                               /* even once it is clear */
    out = PublishValid(150U);
    CHECK(out.cause == NONE);                   /* same movement stays latched */
}

static void test_only_forward_is_protected(void)
{
    AppUsParkOutput out;
    unsigned i;
    struct { AppMotionState state; MotionType command; } others[] = {
        { APP_MOTION_IDLE, MOTION_STOP },
        { APP_MOTION_DISTANCE, MOTION_BACKWARD },
        { APP_MOTION_PRE_STEER, MOTION_FORWARD_LEFT },
        { APP_MOTION_TURN, MOTION_FORWARD_LEFT },
        { APP_MOTION_PRE_STEER, MOTION_FORWARD_RIGHT },
        { APP_MOTION_TURN, MOTION_FORWARD_RIGHT },
        { APP_MOTION_PRE_STEER, MOTION_BACKWARD_LEFT },
        { APP_MOTION_TURN, MOTION_BACKWARD_LEFT },
        { APP_MOTION_PRE_STEER, MOTION_BACKWARD_RIGHT },
        { APP_MOTION_TURN, MOTION_BACKWARD_RIGHT },
        { APP_MOTION_BRAKING, MOTION_FORWARD },     /* braking after completion */
        { APP_MOTION_TUNE_LEFT, MOTION_TUNE_LEFT },
        { APP_MOTION_TUNE_BOTH, MOTION_TUNE_BOTH },
        { APP_MOTION_TUNE_STEERING, MOTION_TUNE_STEERING_FORWARD },
        { APP_MOTION_DIAGNOSTIC_PWM, MOTION_PWM_TEST },
        { APP_MOTION_DIAGNOSTIC_HEADING, MOTION_HEADING_FORWARD },
        { APP_MOTION_WAIT_IMU, MOTION_IMU_ZERO },
        { APP_MOTION_DISTANCE, MOTION_FORWARD }     /* seq 0 = no identity */
    };
    unsigned n = (unsigned)(sizeof(others) / sizeof(others[0]));

    for (i = 0U; i < n; i++)
    {
        unsigned k;

        Begin();
        SetMotion(others[i].state, others[i].command, 0U);
        (void)Poll();

        if (i + 1U < n)
        {
            motion.movementSeq = 9U;            /* identity present */
        }

        for (k = 0U; k < 6U; k++)
        {
            out = PublishValid(100U);
            CHECK(out.cause == NONE);
            out = PublishInvalid();
            CHECK(out.cause == NONE);
        }

        CHECK_EQ(park.nearCount, 0);
        CHECK_EQ(park.invalidCount, 0);
        CHECK(!park.armed);
    }
}

static void test_counters_reset_at_movement_boundaries(void)
{
    AppUsParkOutput out;

    /* Two invalid in movement N, one in N+1 (switched with no idle gap):
     * must not trigger. */
    Begin();
    StartForward(1U);
    (void)PublishValid(400U);
    (void)PublishInvalid();
    (void)PublishInvalid();
    CHECK_EQ(park.invalidCount, 2);

    SetMotion(APP_MOTION_DISTANCE, MOTION_FORWARD, 2U);
    out = Poll();
    CHECK(out.cause == NONE);
    CHECK_EQ(park.invalidCount, 0);             /* reset at the boundary */
    CHECK_EQ(park.nearCount, 0);
    (void)PublishValid(400U);                   /* arm movement N+1 */
    out = PublishInvalid();
    CHECK(out.cause == NONE);
    CHECK_EQ(park.invalidCount, 1);

    /* Leaving forward motion clears every numeric counter. */
    Begin();
    StartForward(1U);
    (void)PublishValid(400U);
    (void)PublishInvalid();
    (void)PublishInvalid();
    (void)PublishValid(150U);
    SetMotion(APP_MOTION_BRAKING, MOTION_FORWARD, 1U);
    (void)Poll();
    CHECK_EQ(park.nearCount, 0);
    CHECK_EQ(park.invalidCount, 0);
    CHECK(!park.armed);
    ExpectState(APP_USPARK_UNARMED);

    /* Back to idle, then a new movement: nothing carried over. */
    SetMotion(APP_MOTION_IDLE, MOTION_STOP, 1U);
    (void)Poll();
    StartForward(2U);
    (void)PublishInvalid();
    CHECK_EQ(park.invalidCount, 0);             /* not armed in movement 2 */
    CHECK_EQ(park.nearCount, 0);

    /* A sample published BEFORE the movement never counts toward it. */
    Begin();
    (void)PublishValid(150U);                   /* seq published while idle */
    SetMotion(APP_MOTION_DISTANCE, MOTION_FORWARD, 1U);
    out = Poll();                               /* tracking starts */
    CHECK(out.cause == NONE);
    CHECK_EQ(park.nearCount, 0);
    out = Poll();                               /* same old publication */
    CHECK_EQ(park.nearCount, 0);
    out = PublishValid(150U);                   /* first NEW one counts */
    CHECK_EQ(park.nearCount, 1);
}

static void test_confirmed_near(void)
{
    AppUsParkOutput out;

    Begin();
    StartForward(1U);
    (void)PublishValid(150U);
    (void)PublishValid(150U);
    out = PublishValid(150U);
    CHECK(out.cause == NEARC);

    /* Not set until the guarded STOP is actually accepted. */
    CHECK(!park.confirmedNear);
    AppUsPark_StopAccepted(&park, NEARC);
    CHECK(park.confirmedNear);

    /* Survives leaving forward motion, a reverse and a turn. */
    SetMotion(APP_MOTION_BRAKING, MOTION_FORWARD, 1U);
    (void)Poll();
    SetMotion(APP_MOTION_IDLE, MOTION_STOP, 1U);
    (void)Poll();
    SetMotion(APP_MOTION_DISTANCE, MOTION_BACKWARD, 2U);
    (void)Poll();
    SetMotion(APP_MOTION_TURN, MOTION_FORWARD_LEFT, 3U);
    (void)Poll();
    CHECK(park.confirmedNear);
    CHECK(!park.armed);                         /* per-movement armed cleared */

    /* 320 mm keeps it; 321 mm clears it (valid, fresh), at any time. The
     * NEAR threshold itself (300) and the 301 mm band do not clear it. */
    SetMotion(APP_MOTION_IDLE, MOTION_STOP, 3U);
    (void)PublishValid(320U);
    CHECK(park.confirmedNear);
    (void)PublishValid(150U);
    CHECK(park.confirmedNear);
    (void)PublishValid(300U);
    CHECK(park.confirmedNear);
    (void)PublishValid(301U);
    CHECK(park.confirmedNear);
    (void)PublishValid(321U);
    CHECK(!park.confirmedNear);

    /* An invalid or stale reading never clears it. */
    park.confirmedNear = true;
    (void)PublishInvalid();
    (void)PublishValidButStale(900U);
    CHECK(park.confirmedNear);

    /* A new forward movement starts ARMED from confirmedNear: its first
     * three consecutive invalid publications fail safe. */
    SetMotion(APP_MOTION_DISTANCE, MOTION_FORWARD, 4U);
    out = Poll();
    CHECK(out.cause == NONE);
    CHECK(park.armed);
    ExpectState(APP_USPARK_ARMED);
    (void)PublishInvalid();
    (void)PublishInvalid();
    out = PublishInvalid();
    CHECK(out.cause == INVAL);

    /* Without confirmedNear the next movement starts unarmed. */
    park.confirmedNear = false;
    SetMotion(APP_MOTION_DISTANCE, MOTION_FORWARD, 5U);
    out = Poll();
    CHECK(!park.armed);

    /* A rejected STOP request (movement ended first) sets nothing. */
    Begin();
    StartForward(1U);
    (void)PublishValid(150U);
    (void)PublishValid(150U);
    out = PublishValid(150U);
    CHECK(out.cause == NEARC);
    CHECK(!park.confirmedNear);                 /* StopAccepted not called */
    AppUsPark_StopAccepted(&park, INVAL);
    CHECK(!park.confirmedNear);                 /* only NEAR sets it */
}

static void test_enable_disable(void)
{
    AppUsParkOutput out;

    /* Enable remembers the current sequence: an old sample cannot count. */
    Begin();
    snap.sequence = 50U;
    snap.result = APP_US_RESULT_VALID;
    snap.filteredValid = 1U;
    snap.filteredMm = 100U;
    snap.lastValidTick = nowMs;
    usSeq = 50U;
    AppUsPark_Enable(&park, 50U, nowMs);
    StartForward(1U);
    out = Poll();
    CHECK_EQ(park.nearCount, 0);

    /* Disable mid-approach clears everything. */
    (void)PublishValid(150U);
    (void)PublishValid(150U);
    CHECK_EQ(park.nearCount, 2);
    park.confirmedNear = true;
    AppUsPark_Disable(&park);
    CHECK(!park.enabled);
    CHECK_EQ(park.nearCount, 0);
    CHECK_EQ(park.invalidCount, 0);
    CHECK(!park.armed);
    CHECK(!park.confirmedNear);
    CHECK_EQ(park.trackedMovementSeq, 0);
    CHECK_EQ(park.latchedMovementSeq, 0);
    ExpectState(APP_USPARK_DISABLED);
    out = PublishValid(100U);
    CHECK(out.cause == NONE);

    /* Re-enabling starts clean, including confirmedNear and faultReported. */
    AppUsPark_Enable(&park, usSeq, nowMs);
    CHECK(park.enabled);
    CHECK(!park.confirmedNear);
    CHECK(!park.faultReported);
    CHECK_EQ(park.lastProcessedUsSeq, usSeq);
}

/* The ground-tested baseline values are pinned both at compile time and in
 * the behaviour checks above/below: a threshold edit must fail loudly. */
_Static_assert(APP_USPARK_STOP_MM == 300U, "STOP threshold is 300 mm");
_Static_assert(APP_USPARK_REARM_MM == 320U, "REARM threshold is 320 mm");
_Static_assert(APP_USPARK_ARM_MM == 500U, "ARM threshold is 500 mm");
_Static_assert(APP_USPARK_NEAR_SAMPLES == 3U, "three near samples");
_Static_assert(APP_USPARK_INVALID_SAMPLES == 3U, "three invalid samples");
_Static_assert(APP_USPARK_NOUPDATE_MS == 250U, "250 ms no-update limit");

static void test_threshold_constants(void)
{
    CHECK_EQ(APP_USPARK_STOP_MM, 300);
    CHECK_EQ(APP_USPARK_REARM_MM, 320);
    CHECK_EQ(APP_USPARK_ARM_MM, 500);
    CHECK(APP_USPARK_STOP_MM < APP_USPARK_REARM_MM);
    CHECK(APP_USPARK_REARM_MM < APP_USPARK_ARM_MM);
}

/* An armed reading around 400 mm (above the 300 mm STOP threshold, inside
 * the 500 mm arming band) must stay armed and must never count as NEAR, no
 * matter how many consecutive publications arrive. */
static void test_armed_400_never_near(void)
{
    AppUsParkOutput out;
    unsigned i;
    static const uint16_t band[] = { 400U, 399U, 350U, 320U, 310U, 301U };
    unsigned b;

    for (b = 0U; b < (unsigned)(sizeof(band) / sizeof(band[0])); b++)
    {
        Begin();
        StartForward(1U);

        for (i = 0U; i < 12U; i++)
        {
            out = PublishValid(band[b]);
            CHECK(out.cause == NONE);
            CHECK(park.armed);
            CHECK_EQ(park.nearCount, 0);
            CHECK_EQ(park.invalidCount, 0);
            ExpectState(APP_USPARK_ARMED);
        }

        /* Approaching: the run still needs three NEW readings <= 300. */
        out = PublishValid(300U);
        CHECK(out.cause == NONE);
        ExpectState(APP_USPARK_NEAR1);
        out = PublishValid(band[b]);                /* breaks the run */
        CHECK(out.cause == NONE);
        CHECK_EQ(park.nearCount, 0);
        CHECK(park.armed);
    }

    /* Armed at 400 mm, then three invalid publications still fail safe. */
    Begin();
    StartForward(1U);
    out = PublishValid(400U);
    CHECK(park.armed);
    out = PublishInvalid();
    out = PublishInvalid();
    CHECK(out.cause == NONE);
    out = PublishInvalid();
    CHECK(out.cause == INVAL);
}

/* Exact 300/301 and 500/501 boundaries on a fresh movement each time. */
static void test_threshold_boundaries(void)
{
    AppUsParkOutput out;

    Begin();
    StartForward(1U);
    (void)PublishValid(300U);
    (void)PublishValid(300U);
    out = PublishValid(300U);
    CHECK(out.cause == NEARC);

    Begin();
    StartForward(1U);
    (void)PublishValid(301U);
    (void)PublishValid(301U);
    out = PublishValid(301U);
    CHECK(out.cause == NONE);
    CHECK_EQ(park.nearCount, 0);
    CHECK(park.armed);

    Begin();
    StartForward(1U);
    (void)PublishValid(500U);
    CHECK(park.armed);

    Begin();
    StartForward(1U);
    (void)PublishValid(501U);
    CHECK(!park.armed);
}

int main(void)
{
    test_threshold_constants();
    test_armed_400_never_near();
    test_threshold_boundaries();
    test_disabled();
    test_near_threshold_and_debounce();
    test_same_sequence_never_counts_twice();
    test_alternating_is_not_consecutive();
    test_valid_above_threshold_resets_debounce();
    test_arming_and_invalid_failsafe();
    test_stale_poll_does_not_trigger();
    test_noupdate_boundaries();
    test_fault();
    test_one_stop_per_movement_and_new_movement();
    test_manual_stop_outstanding_wins();
    test_only_forward_is_protected();
    test_counters_reset_at_movement_boundaries();
    test_confirmed_near();
    test_enable_disable();

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
