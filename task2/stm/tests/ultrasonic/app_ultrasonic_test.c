/* Host-buildable regression test for the ultrasonic driver core
 * (Core/Src/app_ultrasonic.c), run against a simulated TIM8
 * (tests/ultrasonic/fakes). The real ISR entry points and the real task
 * service step are exercised together: the simulator steps the 1 MHz
 * counter, raises capture/compare flags and dispatches them the way
 * HAL_TIM_IRQHandler() does (one SR/DIER sample, CC2 before CC3).
 *
 * Time model: RTOS ticks are 1 ms; RunMs() advances 1000 us of simulated
 * hardware per tick and calls AppUltrasonic_Service() every 10 ticks, like
 * UltrasonicTask. Interrupt latency is zero in the simulator.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -Itests/ultrasonic/fakes \
 *       -Itests/motion/fakes -o tests/ultrasonic/app_ultrasonic_test.exe \
 *       tests/ultrasonic/app_ultrasonic_test.c Core/Src/app_ultrasonic.c \
 *       tests/ultrasonic/fakes/ultrasonic_hw_fake.c
 *   ./tests/ultrasonic/app_ultrasonic_test.exe
 */

#include "app_ultrasonic.h"
#include "ultrasonic_hw_fake.h"

#include <stdio.h>
#include <stdlib.h>

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

static uint32_t tick;
static uint32_t sinceStart;

/* Independent re-statement of the documented conversion. */
static unsigned ExpectedMm(unsigned pulseUs)
{
    /* mm = ((pulse * 1744 + 5000) / 10000) - 10, restated here independently
     * of the firmware macros so this also pins the accepted calibration. */
    return (unsigned)((((unsigned long)pulseUs * 1744UL + 5000UL) / 10000UL) - 10UL);
}

static AppUltrasonicSnapshot Snap(void)
{
    AppUltrasonicSnapshot snapshot;

    AppUltrasonic_GetSnapshot(&snapshot);
    return snapshot;
}

static void Begin(uint32_t tickBase, uint16_t setupFaultMask)
{
    FakeUs_Reset();
    FakeUs_SetSetupFaultMask(setupFaultMask);
    tick = tickBase;
    sinceStart = 0U;
    AppUltrasonic_Init();
}

/* Pin invariant: PC6 is high only while the driver is in TRIGGER_HIGH. */
static void CheckInvariants(void)
{
    AppUltrasonicSnapshot snapshot = Snap();

    if (snapshot.state != APP_US_STATE_TRIGGER_HIGH)
    {
        CHECK(!FakeUs_TriggerPin());
    }
}

static void RunMs(unsigned milliseconds)
{
    while (milliseconds-- > 0U)
    {
        if ((sinceStart % 10U) == 0U)
        {
            AppUltrasonic_Service(tick);
        }

        FakeUs_AdvanceUs(1000U);
        tick++;
        sinceStart++;
        CheckInvariants();
    }
}

/* Run until the published sequence advances beyond `before`. */
static AppUltrasonicSnapshot RunToNextPublish(unsigned limitMs)
{
    uint32_t before = Snap().sequence;
    unsigned ms = 0U;

    while ((Snap().sequence == before) && (ms < limitMs))
    {
        RunMs(1U);
        ms++;
    }

    return Snap();
}

static AppUltrasonicSnapshot RunCycle(uint32_t riseDelayUs, uint32_t widthUs)
{
    FakeUs_EchoPulse(riseDelayUs, widthUs);
    return RunToNextPublish(300U);
}

/* Every Trigger pulse that completed must be >= 10 us (and not absurdly
 * long); a pulse that never completed (width 0) is skipped. */
static void CheckPulseWidths(void)
{
    unsigned i;

    for (i = 0U; i < FakeUs_TriggerCount(); i++)
    {
        uint32_t width = FakeUs_TriggerWidthUs(i);

        if (width != 0U)
        {
            CHECK(width >= 10U);
            CHECK(width <= 12U);
        }
    }
}

static void CheckIdleHardware(void)
{
    CHECK(!FakeUs_TriggerPin());
    CHECK(!FakeUs_CompareIrqEnabled());
    CHECK(!FakeUs_CaptureIrqEnabled());
    CHECK(!FakeUs_CapturePolarityFalling());
    CHECK(!FakeUs_CaptureFlagsPending());
}

/* Manual-stepping start: trigger issued, 20 us on, Trigger pulse ended,
 * driver waiting for the rising Echo. No echo is scheduled. */
static uint16_t BeginWaitingForRise(void)
{
    Begin(0U, 0U);
    FakeUs_EchoNone();
    AppUltrasonic_Service(0U);
    FakeUs_AdvanceUs(20U);
    CHECK_EQ(Snap().state, APP_US_STATE_WAIT_RISE);
    return FakeUs_TriggerStartCount();
}

/* ------------------------------------------------------------------ */

static void test_init_and_fault(void)
{
    AppUltrasonicSnapshot snapshot;
    unsigned i;

    Begin(0U, 0U);
    snapshot = Snap();

    CHECK_EQ(FakeUs_SetupCallCount(), 1);
    CHECK_EQ(snapshot.sequence, 0);
    CHECK_EQ(snapshot.result, APP_US_RESULT_NONE);
    CHECK_EQ(snapshot.state, APP_US_STATE_IDLE);
    CHECK_EQ(snapshot.rawValid, 0);
    CHECK_EQ(snapshot.filteredValid, 0);
    CHECK_EQ(snapshot.pulseCaptured, 0);
    CHECK_EQ(FakeUs_TriggerCount(), 0);
    CheckIdleHardware();
    CHECK(!AppUltrasonic_IsFresh(&snapshot, 0U));
    CHECK_EQ(AppUltrasonic_AgeTicks(&snapshot, 5U), (long)UINT32_MAX);

    AppUltrasonic_GetSnapshot(NULL);   /* must not crash */

    /* A failed configuration check: the driver must refuse to run and
     * never drive the Trigger pin. */
    Begin(0U, APP_US_FAULT_CLOCK | APP_US_FAULT_NVIC_PRIO);
    CHECK_EQ(Snap().state, APP_US_STATE_FAULT);

    for (i = 0U; i < 50U; i++)
    {
        RunMs(10U);
    }

    snapshot = Snap();
    CHECK_EQ(FakeUs_TriggerCount(), 0);
    CHECK(!FakeUs_TriggerPin());
    CHECK_EQ(snapshot.state, APP_US_STATE_FAULT);
    CHECK_EQ(snapshot.result, APP_US_RESULT_FAULT);
    CHECK_EQ(snapshot.sequence, 1);          /* published exactly once */
    CHECK_EQ(snapshot.faultMask, APP_US_FAULT_CLOCK | APP_US_FAULT_NVIC_PRIO);
    CHECK_EQ(snapshot.rawValid, 0);
    CHECK_EQ(snapshot.filteredValid, 0);
    CHECK(!AppUltrasonic_IsFresh(&snapshot, tick));
}

static void test_trigger_pulse(void)
{
    /* Counter starts near the 16-bit wrap: the compare value must wrap. */
    Begin(0U, 0U);
    FakeUs_EchoNone();
    FakeUs_SetCounter(65530U);
    AppUltrasonic_Service(0U);

    CHECK(FakeUs_TriggerPin());
    CHECK_EQ(Snap().state, APP_US_STATE_TRIGGER_HIGH);
    CHECK_EQ(FakeUs_TriggerStartCount(), 65530);
    CHECK_EQ(FakeUs_CompareValue(), (uint16_t)(65530U + 11U));
    CHECK(FakeUs_CompareIrqEnabled());
    CHECK(!FakeUs_CaptureIrqEnabled());

    FakeUs_AdvanceUs(10U);
    CHECK(FakeUs_TriggerPin());                 /* still inside the pulse */
    FakeUs_AdvanceUs(1U);
    CHECK(!FakeUs_TriggerPin());                /* ended by the CH3 event */
    CHECK_EQ(FakeUs_TriggerWidthUs(0U), 11);

    /* The same channel is now the no-Echo timeout, wrapped mod 65536. */
    CHECK_EQ(Snap().state, APP_US_STATE_WAIT_RISE);
    CHECK_EQ(FakeUs_CompareValue(), (uint16_t)(65530U + 45000U));
    CHECK(FakeUs_CompareIrqEnabled());
    CHECK(FakeUs_CaptureIrqEnabled());
    CHECK(!FakeUs_CapturePolarityFalling());
    CHECK(!FakeUs_CaptureFlagsPending());
}

static void test_valid_measurement(void)
{
    AppUltrasonicSnapshot snapshot;

    Begin(0U, 0U);
    snapshot = RunCycle(500U, 1166U);

    CHECK_EQ(snapshot.sequence, 1);
    CHECK_EQ(snapshot.result, APP_US_RESULT_VALID);
    CHECK_EQ(snapshot.pulseCaptured, 1);
    CHECK_EQ(snapshot.pulseUs, 1166);
    CHECK_EQ(snapshot.rawValid, 1);
    CHECK_EQ(snapshot.rawMm, ExpectedMm(1166U));
    CHECK_EQ(ExpectedMm(1166U), 193);
    CHECK_EQ(snapshot.filteredValid, 1);
    CHECK_EQ(snapshot.filteredMm, ExpectedMm(1166U));
    CHECK_EQ(snapshot.filterSamples, 1);
    CHECK_EQ(snapshot.counters.valid, 1);
    CHECK_EQ(snapshot.state, APP_US_STATE_IDLE);
    CHECK(AppUltrasonic_IsFresh(&snapshot, tick));
    CheckIdleHardware();
    CheckPulseWidths();
}

static void test_counter_wrap(void)
{
    AppUltrasonicSnapshot snapshot;

    /* Rising edge at CNT 65000, falling edge at CNT 1000 after the wrap:
     * 1536 us, not 1000 - 65000. */
    Begin(0U, 0U);
    FakeUs_SetCounter(64000U);
    snapshot = RunCycle(989U, 1536U);

    CHECK_EQ(snapshot.result, APP_US_RESULT_VALID);
    CHECK_EQ(snapshot.pulseUs, 1536);
    CHECK_EQ(snapshot.rawMm, ExpectedMm(1536U));
    CHECK_EQ(snapshot.rawMm, 258);

    /* Trigger start itself just below the wrap. */
    Begin(0U, 0U);
    FakeUs_SetCounter(65500U);
    snapshot = RunCycle(300U, 2000U);

    CHECK_EQ(snapshot.result, APP_US_RESULT_VALID);
    CHECK_EQ(snapshot.pulseUs, 2000);
}

static void test_trigger_spacing_and_tick_wrap(void)
{
    unsigned i;
    AppUltrasonicSnapshot snapshot;

    /* The RTOS tick counter wraps 256 ms into this run. */
    Begin(0xFFFFFF00UL, 0U);
    FakeUs_EchoPulse(500U, 1000U);
    RunMs(1000U);

    CHECK(FakeUs_TriggerCount() >= 13U);
    CHECK_EQ(FakeUs_TriggerTimeUs(0U), 0);

    for (i = 1U; i < FakeUs_TriggerCount(); i++)
    {
        uint32_t spacing =
            FakeUs_TriggerTimeUs(i) - FakeUs_TriggerTimeUs(i - 1U);

        CHECK(spacing >= 60000U);       /* never faster than 60 ms        */
        CHECK(spacing >= 61000U);       /* 61 ticks, the approved gate    */
        CHECK(spacing <= 71000U);       /* 10 ms service grid, no drift   */
    }

    snapshot = Snap();
    CHECK(snapshot.counters.valid >= 12U);
    CHECK_EQ(snapshot.counters.noRise, 0);
    CHECK_EQ(snapshot.counters.edgeError, 0);
    CHECK(AppUltrasonic_IsFresh(&snapshot, tick));
    CheckPulseWidths();
}

static void test_no_rise_timeout_restores_capture(void)
{
    AppUltrasonicSnapshot snapshot;

    Begin(0U, 0U);
    FakeUs_EchoNone();
    RunMs(55U);                 /* timeout at 45 ms, published at 50 ms */
    snapshot = Snap();

    CHECK_EQ(snapshot.sequence, 1);
    CHECK_EQ(snapshot.result, APP_US_RESULT_NO_RISE);
    CHECK_EQ(snapshot.counters.noRise, 1);
    CHECK_EQ(snapshot.rawValid, 0);
    CHECK_EQ(snapshot.pulseCaptured, 0);
    CHECK_EQ(snapshot.filteredValid, 0);
    CHECK_EQ(snapshot.state, APP_US_STATE_IDLE);
    CHECK(!AppUltrasonic_IsFresh(&snapshot, tick));
    CheckIdleHardware();

    /* The next cycle works from the restored state. */
    snapshot = RunCycle(300U, 1166U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_VALID);
    CHECK_EQ(snapshot.rawMm, ExpectedMm(1166U));
    CheckPulseWidths();
}

static void test_no_fall_then_echo_stuck(void)
{
    AppUltrasonicSnapshot snapshot;
    unsigned triggers;

    Begin(0U, 0U);
    FakeUs_EchoPulse(300U, 0U);         /* rises, never falls */
    RunMs(55U);
    snapshot = Snap();

    CHECK_EQ(snapshot.result, APP_US_RESULT_NO_FALL);
    CHECK_EQ(snapshot.counters.noFall, 1);
    CHECK_EQ(snapshot.pulseCaptured, 0);
    CHECK_EQ(snapshot.rawValid, 0);
    /* Capture channel restored to rising even though Echo is still high. */
    CHECK(!FakeUs_CapturePolarityFalling());
    CHECK(!FakeUs_CaptureIrqEnabled());
    CHECK(!FakeUs_CompareIrqEnabled());
    CHECK(!FakeUs_TriggerPin());

    /* Echo still high: the next trigger is skipped and reported. */
    triggers = FakeUs_TriggerCount();
    snapshot = RunToNextPublish(200U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_ECHO_STUCK);
    CHECK_EQ(snapshot.counters.echoStuck, 1);
    CHECK_EQ(FakeUs_TriggerCount(), triggers);
    CHECK_EQ(snapshot.rawValid, 0);

    /* Echo released: normal operation resumes. */
    FakeUs_ReleaseEcho();
    snapshot = RunCycle(500U, 1166U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_VALID);
    CHECK_EQ(FakeUs_TriggerCount(), triggers + 1U);
    CheckPulseWidths();
}

static void test_rise_window(void)
{
    AppUltrasonicSnapshot snapshot;

    Begin(0U, 0U);

    snapshot = RunCycle(5000U, 1000U);          /* last accepted delay */
    CHECK_EQ(snapshot.result, APP_US_RESULT_VALID);

    snapshot = RunCycle(5001U, 1000U);          /* one microsecond late */
    CHECK_EQ(snapshot.result, APP_US_RESULT_LATE_RISE);
    CHECK_EQ(snapshot.counters.lateRise, 1);
    CHECK_EQ(snapshot.rawValid, 0);
    CHECK_EQ(snapshot.pulseCaptured, 0);
    CheckIdleHardware();

    snapshot = RunCycle(6000U, 1000U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_LATE_RISE);
    CHECK_EQ(snapshot.counters.lateRise, 2);
    CHECK_EQ(snapshot.counters.valid, 1);
}

static void test_pulse_range_gates(void)
{
    AppUltrasonicSnapshot snapshot;

    Begin(0U, 0U);

    snapshot = RunCycle(200U, 99U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_TOO_NEAR);
    CHECK_EQ(snapshot.pulseCaptured, 1);        /* kept for calibration */
    CHECK_EQ(snapshot.pulseUs, 99);
    CHECK_EQ(snapshot.rawValid, 0);
    CHECK_EQ(snapshot.filteredValid, 0);        /* filter untouched */

    snapshot = RunCycle(200U, 100U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_VALID);
    CHECK_EQ(snapshot.rawMm, ExpectedMm(100U));
    CHECK_EQ(snapshot.rawMm, 7);

    snapshot = RunCycle(200U, 23500U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_VALID);
    CHECK_EQ(snapshot.rawMm, ExpectedMm(23500U));
    CHECK_EQ(snapshot.rawMm, 4088);

    snapshot = RunCycle(200U, 23501U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_TOO_FAR);
    CHECK_EQ(snapshot.rawValid, 0);
    CHECK_EQ(snapshot.counters.tooFar, 1);
    CHECK_EQ(snapshot.counters.tooNear, 1);
    CHECK_EQ(snapshot.filterSamples, 2);        /* only valid samples */

    /* A typical "nothing in range" Echo (~38 ms) is captured but far. */
    snapshot = RunCycle(200U, 38000U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_TOO_FAR);
    CHECK_EQ(snapshot.pulseUs, 38000);
    CheckPulseWidths();
}

static void test_edge_rejection(void)
{
    AppUltrasonicSnapshot snapshot;
    uint16_t start;

    /* Edge earlier than the end of the Trigger pulse. */
    start = BeginWaitingForRise();
    FakeUs_InjectCapture((uint16_t)(start + 5U), false);
    CHECK_EQ(Snap().state, APP_US_STATE_DONE);
    CheckIdleHardware();
    AppUltrasonic_Service(10U);
    snapshot = Snap();
    CHECK_EQ(snapshot.result, APP_US_RESULT_EDGE_ERROR);
    CHECK_EQ(snapshot.counters.edgeError, 1);
    CHECK_EQ(snapshot.state, APP_US_STATE_IDLE);

    /* Overcapture while waiting for the rising edge. */
    start = BeginWaitingForRise();
    FakeUs_InjectCapture((uint16_t)(start + 500U), true);
    AppUltrasonic_Service(10U);
    CHECK_EQ(Snap().result, APP_US_RESULT_EDGE_ERROR);
    CheckIdleHardware();

    /* Overcapture while waiting for the falling edge. */
    start = BeginWaitingForRise();
    FakeUs_InjectCapture((uint16_t)(start + 500U), false);
    CHECK_EQ(Snap().state, APP_US_STATE_WAIT_FALL);
    CHECK(FakeUs_CapturePolarityFalling());
    FakeUs_InjectCapture((uint16_t)(start + 1500U), true);
    AppUltrasonic_Service(10U);
    snapshot = Snap();
    CHECK_EQ(snapshot.result, APP_US_RESULT_EDGE_ERROR);
    CHECK_EQ(snapshot.rawValid, 0);
    CHECK_EQ(snapshot.pulseCaptured, 0);
    CheckIdleHardware();

    /* Duplicate edge: a "fall" identical to the rise is a zero-width
     * pulse. It is rejected, never reported as a valid 0 mm. */
    start = BeginWaitingForRise();
    FakeUs_InjectCapture((uint16_t)(start + 500U), false);
    FakeUs_InjectCapture((uint16_t)(start + 500U), false);
    AppUltrasonic_Service(10U);
    snapshot = Snap();
    CHECK_EQ(snapshot.result, APP_US_RESULT_TOO_NEAR);
    CHECK_EQ(snapshot.pulseCaptured, 1);
    CHECK_EQ(snapshot.pulseUs, 0);
    CHECK_EQ(snapshot.rawValid, 0);
    CHECK_EQ(snapshot.filteredValid, 0);

    /* A falling edge after the timeout deadline is a late edge. */
    start = BeginWaitingForRise();
    FakeUs_InjectCapture((uint16_t)(start + 500U), false);
    FakeUs_InjectCapture((uint16_t)(start + 45010U), false);
    AppUltrasonic_Service(10U);
    CHECK_EQ(Snap().result, APP_US_RESULT_NO_FALL);
    CHECK_EQ(Snap().rawValid, 0);

    /* Exactly at the deadline is still accepted as an edge. */
    start = BeginWaitingForRise();
    FakeUs_InjectCapture((uint16_t)(start + 500U), false);
    FakeUs_InjectCapture((uint16_t)(start + 45000U), false);
    AppUltrasonic_Service(10U);
    snapshot = Snap();
    CHECK_EQ(snapshot.result, APP_US_RESULT_TOO_FAR);
    CHECK_EQ(snapshot.pulseUs, 44500);
}

static void test_spurious_and_stale_events(void)
{
    AppUltrasonicSnapshot snapshot;
    uint16_t start;

    /* Capture event while idle (after a finished cycle). */
    Begin(0U, 0U);
    (void)RunCycle(500U, 1166U);
    AppUltrasonic_OnCapture();
    snapshot = Snap();
    CHECK_EQ(snapshot.counters.spurious, 1);
    CHECK_EQ(snapshot.state, APP_US_STATE_IDLE);
    CheckIdleHardware();

    /* Compare event while idle. */
    AppUltrasonic_OnCompare();
    snapshot = Snap();
    CHECK_EQ(snapshot.counters.staleCompare, 1);
    CHECK_EQ(snapshot.state, APP_US_STATE_IDLE);
    CHECK(!FakeUs_CompareIrqEnabled());

    /* Capture event during the Trigger pulse: ignored, cycle continues. */
    Begin(0U, 0U);
    FakeUs_EchoPulse(500U, 1166U);
    AppUltrasonic_Service(0U);
    AppUltrasonic_OnCapture();
    CHECK_EQ(Snap().counters.spurious, 1);
    CHECK_EQ(Snap().state, APP_US_STATE_TRIGGER_HIGH);
    CHECK(FakeUs_TriggerPin());
    snapshot = RunToNextPublish(300U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_VALID);
    CHECK_EQ(snapshot.rawMm, ExpectedMm(1166U));

    /* Stale capture flags from before the pulse never become an edge. */
    Begin(0U, 0U);
    FakeUs_SetStaleCaptureFlags();
    snapshot = RunCycle(500U, 1166U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_VALID);
    CHECK_EQ(snapshot.pulseUs, 1166);
    CHECK_EQ(snapshot.counters.spurious, 0);

    (void)start;
}

static void test_capture_and_compare_in_one_entry(void)
{
    AppUltrasonicSnapshot snapshot;

    /* Falling edge exactly on the timeout count: both flags are pending in
     * one interrupt entry. CC2 is serviced first, the edge is accepted, and
     * the compare that follows finds a finished cycle (counted, ignored). */
    Begin(0U, 0U);
    snapshot = RunCycle(100U, 44889U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_TOO_FAR);
    CHECK_EQ(snapshot.pulseUs, 44889);
    CHECK_EQ(snapshot.counters.staleCompare, 1);
    CheckIdleHardware();

    /* One microsecond later the timeout wins: no falling edge. */
    Begin(0U, 0U);
    snapshot = RunCycle(100U, 44890U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_NO_FALL);
    CHECK_EQ(snapshot.counters.staleCompare, 0);
    CHECK_EQ(snapshot.counters.spurious, 0);
    CheckIdleHardware();
}

static void test_echo_stuck_at_start_and_filter_reset(void)
{
    AppUltrasonicSnapshot snapshot;

    /* A good sample first, then the line sticks high. */
    Begin(0U, 0U);
    snapshot = RunCycle(500U, 1166U);
    CHECK_EQ(snapshot.filteredValid, 1);

    FakeUs_EchoStuckHigh(true);
    snapshot = RunToNextPublish(200U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_ECHO_STUCK);
    CHECK_EQ(snapshot.filteredValid, 0);            /* history dropped */
    CHECK_EQ(snapshot.filterSamples, 0);
    CHECK(!AppUltrasonic_IsFresh(&snapshot, tick));

    /* Stuck from power-up: never triggers, reports at most every 61 ticks. */
    Begin(0U, 0U);
    FakeUs_EchoStuckHigh(true);
    RunMs(300U);
    snapshot = Snap();
    CHECK_EQ(FakeUs_TriggerCount(), 0);
    CHECK(snapshot.counters.echoStuck >= 4U);
    CHECK(snapshot.counters.echoStuck <= 5U);
    CHECK(!FakeUs_TriggerPin());

    FakeUs_EchoStuckHigh(false);
    snapshot = RunCycle(500U, 1166U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_VALID);
    CHECK_EQ(FakeUs_TriggerCount(), 1);
}

static void test_watchdogs(void)
{
    AppUltrasonicSnapshot snapshot;

    /* Lost compare interrupt: PC6 must not stay high. */
    Begin(0U, 0U);
    FakeUs_DropCompareInterrupts(true);
    RunMs(5U);
    CHECK(FakeUs_TriggerPin());                 /* still inside the grace */
    CHECK_EQ(Snap().state, APP_US_STATE_TRIGGER_HIGH);
    RunMs(6U);                                  /* service at 10 ticks */
    snapshot = Snap();
    CHECK(!FakeUs_TriggerPin());
    CHECK_EQ(snapshot.state, APP_US_STATE_IDLE);
    CHECK_EQ(snapshot.result, APP_US_RESULT_STALL);
    CHECK_EQ(snapshot.counters.stall, 1);
    CHECK_EQ(snapshot.rawValid, 0);
    CheckIdleHardware();

    FakeUs_DropCompareInterrupts(false);
    snapshot = RunCycle(500U, 1166U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_VALID);
    CHECK(tick >= 61U);

    /* Lost timeout interrupt while waiting for the Echo. */
    Begin(0U, 0U);
    FakeUs_EchoNone();
    RunMs(1U);                                  /* trigger + Trigger end */
    CHECK_EQ(Snap().state, APP_US_STATE_WAIT_RISE);
    FakeUs_DropCompareInterrupts(true);
    RunMs(60U);
    snapshot = Snap();
    CHECK_EQ(snapshot.result, APP_US_RESULT_STALL);
    CHECK_EQ(snapshot.counters.stall, 1);
    CHECK_EQ(snapshot.state, APP_US_STATE_IDLE);
    CheckIdleHardware();
    CheckPulseWidths();
}

static void test_filter_and_staleness(void)
{
    AppUltrasonicSnapshot snapshot;
    unsigned guard;

    Begin(0U, 0U);

    snapshot = RunCycle(300U, 1166U);               /* 200 mm */
    CHECK_EQ(snapshot.filteredMm, ExpectedMm(1166U));
    CHECK_EQ(snapshot.filterSamples, 1);

    snapshot = RunCycle(300U, 1749U);               /* 300 mm */
    CHECK_EQ(snapshot.rawMm, ExpectedMm(1749U));
    CHECK_EQ(snapshot.filteredMm,
             (ExpectedMm(1166U) + ExpectedMm(1749U)) / 2U);             /* floor of the mean */
    CHECK_EQ(snapshot.filterSamples, 2);

    snapshot = RunCycle(300U, 5832U);               /* 1000 mm outlier */
    CHECK_EQ(snapshot.rawMm, ExpectedMm(5832U));
    CHECK_EQ(snapshot.filteredMm, ExpectedMm(1749U));             /* median of 3 */
    CHECK_EQ(snapshot.filterSamples, 3);

    snapshot = RunCycle(300U, 1807U);               /* 310 mm */
    CHECK_EQ(snapshot.rawMm, ExpectedMm(1807U));
    CHECK_EQ(snapshot.filteredMm, ExpectedMm(1807U));             /* {305,295,1007} */
    CHECK_EQ(snapshot.filterSamples, 3);

    /* Invalid cycles never enter the filter or disturb its value. */
    FakeUs_EchoNone();
    snapshot = RunToNextPublish(300U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_NO_RISE);
    CHECK_EQ(snapshot.rawValid, 0);
    CHECK_EQ(snapshot.filteredMm, ExpectedMm(1807U));
    CHECK_EQ(snapshot.filterSamples, 3);
    CHECK_EQ(snapshot.filteredValid, 1);
    CHECK(AppUltrasonic_IsFresh(&snapshot, tick));

    /* Keep failing until the last valid sample is stale. */
    for (guard = 0U; guard < 20U; guard++)
    {
        snapshot = RunToNextPublish(300U);
        if (!AppUltrasonic_IsFresh(&snapshot, tick))
        {
            break;
        }
    }

    CHECK(!AppUltrasonic_IsFresh(&snapshot, tick));
    CHECK(AppUltrasonic_AgeTicks(&snapshot, tick) > APP_US_STALE_TICKS_RTOS);
    CHECK_EQ(snapshot.filteredValid, 1);            /* value kept, but stale */
    CHECK_EQ(snapshot.filteredMm, ExpectedMm(1807U));

    /* The next valid sample starts a fresh history (no old/new mixing). */
    snapshot = RunCycle(300U, 1166U);
    CHECK_EQ(snapshot.result, APP_US_RESULT_VALID);
    CHECK_EQ(snapshot.filterSamples, 1);
    CHECK_EQ(snapshot.filteredMm, ExpectedMm(1166U));
    CHECK(AppUltrasonic_IsFresh(&snapshot, tick));
}

static void test_two_sample_mean(void)
{
    AppUltrasonicSnapshot snapshot;

    /* 193 mm then 296 mm: two valid samples average to floor(244.5) = 244,
     * and the third sample switches to the median. */
    Begin(0U, 0U);
    snapshot = RunCycle(300U, 1166U);
    CHECK_EQ(snapshot.rawMm, ExpectedMm(1166U));
    CHECK_EQ(snapshot.filterSamples, 1);
    CHECK_EQ(snapshot.filteredMm, ExpectedMm(1166U));

    snapshot = RunCycle(300U, 1755U);
    CHECK_EQ(snapshot.rawMm, ExpectedMm(1755U));
    CHECK_EQ(ExpectedMm(1755U), 296);
    CHECK_EQ(snapshot.filterSamples, 2);
    CHECK_EQ(snapshot.filteredMm,
             (ExpectedMm(1166U) + ExpectedMm(1755U)) / 2U);   /* floor(244.5) */

    snapshot = RunCycle(300U, 1807U);               /* 310 mm */
    CHECK_EQ(snapshot.filterSamples, 3);
    CHECK_EQ(snapshot.filteredMm, ExpectedMm(1755U));             /* median of 193,296,305 */

    /* Equal pair and a large pair: no overflow in the 16-bit mean. */
    Begin(0U, 0U);
    (void)RunCycle(300U, 23500U);                   /* 4030 mm */
    snapshot = RunCycle(300U, 23500U);
    CHECK_EQ(snapshot.filterSamples, 2);
    CHECK_EQ(snapshot.filteredMm, 4088);
}

static void test_task_create_failure(void)
{
    AppUltrasonicSnapshot snapshot;
    unsigned i;

    /* UltrasonicTask was never created, so AppUltrasonic_Init()/Service()
     * never run. main() calls the reporter instead (pre-scheduler). */
    FakeUs_Reset();
    AppUltrasonic_ReportTaskCreateFailure();
    snapshot = Snap();

    CHECK_EQ(snapshot.sequence, 1);
    CHECK_EQ(snapshot.state, APP_US_STATE_FAULT);
    CHECK_EQ(snapshot.result, APP_US_RESULT_FAULT);
    CHECK_EQ(snapshot.faultMask, APP_US_FAULT_TASK);
    CHECK_EQ(snapshot.cycles, 1);
    CHECK_EQ(snapshot.rawValid, 0);
    CHECK_EQ(snapshot.filteredValid, 0);
    CHECK_EQ(snapshot.pulseCaptured, 0);
    CHECK_EQ(snapshot.filterSamples, 0);
    CHECK(!AppUltrasonic_IsFresh(&snapshot, 0U));
    CHECK(!AppUltrasonic_IsFresh(&snapshot, 12345U));
    CHECK_EQ(AppUltrasonic_AgeTicks(&snapshot, 5U), (long)UINT32_MAX);

    /* PC6 forced low, TIM8 capture never started, nothing armed. */
    CHECK(FakeUs_TriggerLowCallCount() >= 1U);
    CHECK(!FakeUs_TriggerPin());
    CHECK_EQ(FakeUs_SetupCallCount(), 0);           /* no TIM8 start-up */
    CHECK(!FakeUs_CompareIrqEnabled());
    CHECK(!FakeUs_CaptureIrqEnabled());
    CHECK_EQ(FakeUs_TriggerCount(), 0);

    /* Even if something did call Service(), the fault is sticky: no
     * trigger, no extra publication, no TIM8 start-up. */
    tick = 0U;
    sinceStart = 0U;

    for (i = 0U; i < 50U; i++)
    {
        RunMs(10U);
    }

    snapshot = Snap();
    CHECK_EQ(FakeUs_TriggerCount(), 0);
    CHECK_EQ(FakeUs_SetupCallCount(), 0);
    CHECK_EQ(snapshot.sequence, 1);
    CHECK_EQ(snapshot.state, APP_US_STATE_FAULT);
    CHECK_EQ(snapshot.faultMask, APP_US_FAULT_TASK);
    CHECK(!FakeUs_TriggerPin());

    /* Stray interrupt events are counted and ignored, never acted on. */
    AppUltrasonic_OnCompare();
    AppUltrasonic_OnCapture();
    snapshot = Snap();
    CHECK_EQ(snapshot.state, APP_US_STATE_FAULT);
    CHECK_EQ(snapshot.counters.spurious, 1);
    CHECK_EQ(snapshot.counters.staleCompare, 1);
    CHECK(!FakeUs_TriggerPin());
}

static void test_sequence_and_snapshot_helpers(void)
{
    AppUltrasonicSnapshot snapshot = {0};
    uint32_t last;
    unsigned i;

    /* Each publication advances the sequence by exactly one. */
    Begin(0U, 0U);
    FakeUs_EchoPulse(500U, 1166U);
    last = 0U;

    for (i = 0U; i < 5U; i++)
    {
        snapshot = RunToNextPublish(300U);
        CHECK_EQ(snapshot.sequence, last + 1U);
        last = snapshot.sequence;
    }

    CHECK_EQ(snapshot.cycles, 5);

    /* Age / freshness are wrap-safe and never depend on the distance. */
    snapshot = (AppUltrasonicSnapshot){0};
    snapshot.filteredValid = 1U;
    snapshot.filteredMm = 0U;                    /* a 0 value, still "valid" */
    snapshot.lastValidTick = 0xFFFFFFF0UL;
    CHECK_EQ(AppUltrasonic_AgeTicks(&snapshot, 0x10UL), 0x20);
    CHECK(AppUltrasonic_IsFresh(&snapshot, 0x10UL));
    CHECK(AppUltrasonic_IsFresh(&snapshot,
          0xFFFFFFF0UL + APP_US_STALE_TICKS_RTOS));
    CHECK(!AppUltrasonic_IsFresh(&snapshot,
          0xFFFFFFF0UL + APP_US_STALE_TICKS_RTOS + 1U));

    snapshot.filteredValid = 0U;
    snapshot.filteredMm = 1234U;                 /* nonzero, still invalid */
    CHECK(!AppUltrasonic_IsFresh(&snapshot, 0x10UL));
    CHECK_EQ(AppUltrasonic_AgeTicks(&snapshot, 0x10UL), (long)UINT32_MAX);
    CHECK_EQ(AppUltrasonic_AgeTicks(NULL, 0U), (long)UINT32_MAX);
}

int main(void)
{
    test_init_and_fault();
    test_trigger_pulse();
    test_valid_measurement();
    test_counter_wrap();
    test_trigger_spacing_and_tick_wrap();
    test_no_rise_timeout_restores_capture();
    test_no_fall_then_echo_stuck();
    test_rise_window();
    test_pulse_range_gates();
    test_edge_rejection();
    test_spurious_and_stale_events();
    test_capture_and_compare_in_one_entry();
    test_echo_stuck_at_start_and_filter_reset();
    test_watchdogs();
    test_filter_and_staleness();
    test_two_sample_mean();
    test_task_create_failure();
    test_sequence_and_snapshot_helpers();

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
