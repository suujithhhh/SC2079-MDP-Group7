/* Host test of the Phase 1 IR acquisition core (Core/Src/app_ir.c): scan
 * scheduling, one-scan-at-a-time, channel mapping, outcome classification,
 * timeout / start / busy / DMA / ADC / data errors, recovery and the latched
 * faults, publication sequence and tick wrap-around, freshness, counter
 * saturation and critical-section use. The ADC/DMA port is replaced by
 * tests/ir/fakes/ir_hw_fake.c; critical sections are COUNTED by
 * tests/park_trace/fakes/FreeRTOS.h.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -Werror -DAPP_IR_ENABLE=1 -DAPP_IR_TEST_HOOKS \
 *       -Itests/ir/fakes -Itests/park_trace/fakes -ICore/Inc \
 *       -o tests/ir/app_ir_test.exe tests/ir/app_ir_test.c \
 *       Core/Src/app_ir.c Core/Src/app_ir_dist.c tests/ir/fakes/ir_hw_fake.c
 *   ./tests/ir/app_ir_test.exe
 * (app_ir_dist.c is the Phase 3 filter app_ir.c now links against; the Phase 1
 * assertions below are unchanged.)
 */

#include "app_ir.h"
#include "ir_hw_fake.h"

#include <stdio.h>
#include <string.h>

int FakeCrit_Depth;
int FakeCrit_MaxDepth;
unsigned FakeCrit_Enters;
unsigned FakeCrit_Underflows;

void FakeCrit_Enter(void)
{
    FakeCrit_Depth++;
    FakeCrit_Enters++;
    if (FakeCrit_Depth > FakeCrit_MaxDepth) { FakeCrit_MaxDepth = FakeCrit_Depth; }
}

void FakeCrit_Exit(void)
{
    if (FakeCrit_Depth == 0) { FakeCrit_Underflows++; }
    else { FakeCrit_Depth--; }
}

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
    long long a_ = (long long)(actual); \
    long long e_ = (long long)(expected); \
    checks++; \
    if (a_ != e_) { \
        failures++; \
        printf("FAIL line %d: %s == %lld, expected %lld\n", \
               __LINE__, #actual, a_, e_); \
    } \
} while (0)

/* HAL_ADC_ERROR_* values (stm32f4xx_hal_adc.h). */
#define HAL_ADC_ERROR_OVR_VALUE  0x02U
#define HAL_ADC_ERROR_DMA_VALUE  0x04U

static uint32_t now;

static void Svc(void) { AppIr_Service(now); }

static AppIrSnapshot Snap(void)
{
    AppIrSnapshot s;
    AppIr_GetSnapshot(&s);
    return s;
}

static void CritReset(void)
{
    FakeCrit_Depth = 0;
    FakeCrit_MaxDepth = 0;
    FakeCrit_Enters = 0U;
    FakeCrit_Underflows = 0U;
}

static void CritBalanced(int line)
{
    checks++;
    if ((FakeCrit_Depth != 0) || (FakeCrit_MaxDepth > 1) ||
        (FakeCrit_Underflows != 0U))
    {
        failures++;
        printf("FAIL line %d: critical sections unbalanced/nested "
               "(depth %d, max %d, underflows %u)\n", line,
               FakeCrit_Depth, FakeCrit_MaxDepth, FakeCrit_Underflows);
    }
}

static void Boot(uint32_t startTick)
{
    IrHwFake_Reset();
    CritReset();
    now = startTick;
    AppIr_Init();
}

/* Start at `now` (must be due), complete with (l, r), collect 10 ticks later.
 * Leaves `now` at the collection tick. */
static void GoodCycle(uint16_t l, uint16_t r)
{
    Svc();
    IrHwFake_Complete(l, r);
    now += 10U;
    Svc();
}

/* ---------------------------------------------------------------------- */

static void test_init_and_first_scan(void)
{
    AppIrSnapshot s;

    Boot(1000U);
    CHECK_EQ(irHw.checkCalls, 1);
    CHECK_EQ(irHw.startCalls, 0);           /* nothing before the first Service */

    s = Snap();
    CHECK_EQ(s.sequence, 0);
    CHECK_EQ(s.status, APP_IR_STATUS_NONE);
    CHECK_EQ(s.faultMask, 0);
    CHECK(!AppIr_IsFresh(&s, now));
    CHECK_EQ(AppIr_ReportMask(&s, now), APP_IR_FAULT_STALE);

    Svc();                                   /* first service starts a scan */
    CHECK_EQ(irHw.startCalls, 1);
    CHECK_EQ(irHw.readyCalls, 1);
    CHECK(irHw.buffer != NULL);
    /* Sentinel written into BOTH words before the start. */
    CHECK_EQ(irHw.bufferAtStart[0], APP_IR_DMA_SENTINEL);
    CHECK_EQ(irHw.bufferAtStart[1], APP_IR_DMA_SENTINEL);
    CHECK_EQ(Snap().sequence, 0);            /* nothing published yet */
    CritBalanced(__LINE__);
}

static void test_mapping_and_valid_values(void)
{
    AppIrSnapshot s;

    Boot(1000U);
    GoodCycle(1820U, 1764U);                 /* element 0 = LEFT, 1 = RIGHT */
    s = Snap();
    CHECK_EQ(s.sequence, 1);
    CHECK_EQ(s.status, APP_IR_STATUS_VALID);
    CHECK_EQ(s.leftRaw, 1820);
    CHECK_EQ(s.rightRaw, 1764);
    CHECK_EQ(s.publishTick, 1010);
    CHECK_EQ(s.lastValidTick, 1010);
    CHECK_EQ(s.faultMask, 0);
    CHECK_EQ(s.consecutiveFailures, 0);
    CHECK_EQ(s.counters.valid, 1);
    CHECK(AppIr_IsFresh(&s, now));
    CHECK_EQ(AppIr_ReportMask(&s, now), 0);

    /* 0 and 4095 are valid raw conversions in Phase 1 (no saturation or
     * plausibility rule), and so is any value in between. */
    now += 10U;
    GoodCycle(0U, APP_IR_ADC_MAX);
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_VALID);
    CHECK_EQ(s.leftRaw, 0);
    CHECK_EQ(s.rightRaw, 4095);

    now += 10U;
    GoodCycle(APP_IR_ADC_MAX, 0U);
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_VALID);
    CHECK_EQ(s.leftRaw, 4095);
    CHECK_EQ(s.rightRaw, 0);
    CHECK_EQ(s.sequence, 3);
    CHECK_EQ(irHw.recoverCalls, 0);          /* no recovery after success */
    CHECK_EQ(irHw.overlaps, 0);
    CritBalanced(__LINE__);
}

static void test_cadence_and_no_overlap(void)
{
    unsigned i;

    Boot(5000U);
    Svc();                                   /* start @5000 */
    CHECK_EQ(irHw.startCalls, 1);

    /* Woken early and often: an in-flight, not-yet-timed-out scan is never
     * collected, re-started or overlapped. */
    now += 1U; Svc();
    now += 1U; Svc();
    CHECK_EQ(irHw.startCalls, 1);
    CHECK_EQ(Snap().sequence, 0);

    IrHwFake_Complete(100U, 200U);
    now = 5010U; Svc();                      /* collect */
    CHECK_EQ(Snap().sequence, 1);
    CHECK_EQ(irHw.startCalls, 1);            /* next start not due until 5020 */
    now = 5019U; Svc();
    CHECK_EQ(irHw.startCalls, 1);
    now = 5020U; Svc();
    CHECK_EQ(irHw.startCalls, 2);

    /* Steady 10-tick service: starts every 20 ticks, publication 10 ticks
     * after each start, never two scans in flight. */
    for (i = 0U; i < 100U; i++)
    {
        unsigned startsBefore = irHw.startCalls;
        IrHwFake_Complete((uint16_t)i, (uint16_t)(4095U - i));
        now += 10U; Svc();                   /* collect */
        CHECK_EQ(irHw.startCalls, startsBefore);
        CHECK_EQ(Snap().publishTick, now);
        CHECK_EQ(Snap().leftRaw, i);
        now += 10U; Svc();                   /* start */
        CHECK_EQ(irHw.startCalls, startsBefore + 1U);
    }
    CHECK_EQ(irHw.overlaps, 0);
    CHECK_EQ(Snap().counters.valid, 101);
    CritBalanced(__LINE__);
}

static void test_completion_inside_start(void)
{
    AppIrSnapshot s;

    /* The scan finishes before HAL_ADC_Start_DMA() returns (IRTask
     * pre-empted after SWSTART): the ISR record was armed first, so the
     * result is collected, not counted as spurious. */
    Boot(1000U);
    irHw.inside = IRFAKE_INSIDE_COMPLETE;
    irHw.insideLeft = 1234U;
    irHw.insideRight = 2345U;
    Svc();
    now += 10U; Svc();
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_VALID);
    CHECK_EQ(s.leftRaw, 1234);
    CHECK_EQ(s.rightRaw, 2345);
    CHECK_EQ(s.counters.spurious, 0);

    /* Same for an error raised inside the start. */
    Boot(1000U);
    irHw.inside = IRFAKE_INSIDE_ERROR;
    irHw.insideErrorCode = HAL_ADC_ERROR_DMA_VALUE;
    Svc();
    now += 10U; Svc();
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_ERROR);
    CHECK_EQ(s.faultMask, APP_IR_FAULT_DMA);
    CHECK_EQ(s.counters.spurious, 0);
    CHECK_EQ(irHw.recoverCalls, 1);
    CritBalanced(__LINE__);
}

static void test_data_integrity(void)
{
    AppIrSnapshot s;

    /* NDTR not zero after "complete" (DMA did not transfer both items). */
    Boot(1000U);
    Svc();
    IrHwFake_Complete(100U, 200U);
    irHw.remaining = 1U;
    now += 10U; Svc();
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_ERROR);
    CHECK_EQ(s.faultMask, APP_IR_FAULT_DATA);
    CHECK_EQ(s.counters.dataError, 1);
    CHECK_EQ(s.leftRaw, 0);
    CHECK_EQ(s.rightRaw, 0);
    CHECK_EQ(irHw.recoverCalls, 1);
    CHECK(!AppIr_IsFresh(&s, now));

    /* A word above 12 bits on either channel. */
    Boot(1000U);
    GoodCycle(4096U, 10U);
    CHECK_EQ(Snap().faultMask, APP_IR_FAULT_DATA);
    Boot(1000U);
    GoodCycle(10U, 4096U);
    CHECK_EQ(Snap().faultMask, APP_IR_FAULT_DATA);

    /* Completion without the DMA writing: the sentinel is still there. */
    Boot(1000U);
    Svc();
    IrHwFake_CompleteNoWrite();
    now += 10U; Svc();
    CHECK_EQ(Snap().status, APP_IR_STATUS_ERROR);
    CHECK_EQ(Snap().faultMask, APP_IR_FAULT_DATA);

    /* The next scan re-writes the sentinel, so stale data from an earlier
     * scan can never pass as a new one. */
    Boot(1000U);
    GoodCycle(111U, 222U);
    now += 10U; Svc();                       /* start #2 */
    CHECK_EQ(irHw.bufferAtStart[0], APP_IR_DMA_SENTINEL);
    CHECK_EQ(irHw.bufferAtStart[1], APP_IR_DMA_SENTINEL);
    IrHwFake_CompleteNoWrite();
    now += 10U; Svc();
    CHECK_EQ(Snap().faultMask, APP_IR_FAULT_DATA);
    CritBalanced(__LINE__);
}

static void test_timeout(void)
{
    AppIrSnapshot s;

    Boot(1000U);
    Svc();                                   /* start @1000 */
    now = 1002U; Svc();                      /* 2 ticks: not yet lost */
    CHECK_EQ(Snap().sequence, 0);
    CHECK_EQ(irHw.recoverCalls, 0);

    now = 1003U; Svc();                      /* 3 ticks: timeout */
    s = Snap();
    CHECK_EQ(s.sequence, 1);
    CHECK_EQ(s.status, APP_IR_STATUS_ERROR);
    CHECK_EQ(s.faultMask, APP_IR_FAULT_TIMEOUT);
    CHECK_EQ(s.counters.timeout, 1);
    CHECK_EQ(s.counters.recoveries, 1);
    CHECK_EQ(s.consecutiveFailures, 1);
    CHECK_EQ(s.publishTick, 1003);
    CHECK_EQ(irHw.recoverCalls, 1);
    CHECK_EQ(irHw.startCalls, 1);            /* no immediate restart */
    CHECK_EQ(irHw.overrunCalls, 1);

    /* A completion arriving after the timeout is only spurious. */
    IrHwFake_Complete(5U, 6U);
    s = Snap();
    CHECK_EQ(s.counters.spurious, 1);
    CHECK_EQ(s.sequence, 1);

    now = 1010U; Svc();
    CHECK_EQ(Snap().sequence, 1);            /* nothing new */
    CHECK_EQ(irHw.startCalls, 1);
    now = 1020U; Svc();                      /* on schedule */
    CHECK_EQ(irHw.startCalls, 2);
    IrHwFake_Complete(7U, 8U);
    now = 1030U; Svc();
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_VALID);
    CHECK_EQ(s.leftRaw, 7);
    CHECK_EQ(s.consecutiveFailures, 0);
    CHECK_EQ(s.counters.spurious, 1);

    /* Timeout with the ADC overrun flag set: TIMEOUT | ADC, flag cleared. */
    Boot(1000U);
    Svc();
    irHw.overrun = true;
    now = 1010U; Svc();
    s = Snap();
    CHECK_EQ(s.faultMask, APP_IR_FAULT_TIMEOUT | APP_IR_FAULT_ADC);
    CHECK_EQ(s.counters.adcError, 1);
    CHECK(!irHw.overrun);

    /* A long task delay: the timeout is measured from the start tick. */
    Boot(1000U);
    Svc();
    now = 1500U; Svc();
    CHECK_EQ(Snap().faultMask, APP_IR_FAULT_TIMEOUT);
    CHECK_EQ(irHw.startCalls, 2);            /* overdue start right after */
    CHECK_EQ(irHw.overlaps, 0);
    CritBalanced(__LINE__);
}

static void test_start_and_busy_failures(void)
{
    AppIrSnapshot s;

    /* HAL start refused (or ADC not enabled): START, recovery, retry on the
     * next period. */
    Boot(1000U);
    irHw.startResult = false;
    Svc();
    s = Snap();
    CHECK_EQ(s.sequence, 1);
    CHECK_EQ(s.status, APP_IR_STATUS_ERROR);
    CHECK_EQ(s.faultMask, APP_IR_FAULT_START);
    CHECK_EQ(s.counters.startFail, 1);
    CHECK_EQ(irHw.recoverCalls, 1);
    now = 1010U; Svc();
    CHECK_EQ(irHw.startCalls, 1);            /* not in flight, not due */
    CHECK_EQ(Snap().sequence, 1);
    irHw.startResult = true;
    now = 1020U; Svc();
    CHECK_EQ(irHw.startCalls, 2);
    IrHwFake_Complete(9U, 10U);
    now = 1030U; Svc();
    CHECK_EQ(Snap().status, APP_IR_STATUS_VALID);

    /* A late callback after a refused start is spurious, never published. */
    Boot(1000U);
    irHw.startResult = false;
    Svc();
    AppIr_OnConvComplete();
    CHECK_EQ(Snap().counters.spurious, 1);
    CHECK_EQ(Snap().sequence, 1);

    /* Handles not ready: no HAL start at all (HAL would still set SWSTART). */
    Boot(1000U);
    irHw.ready = false;
    Svc();
    s = Snap();
    CHECK_EQ(irHw.startCalls, 0);
    CHECK_EQ(s.status, APP_IR_STATUS_ERROR);
    CHECK_EQ(s.faultMask, APP_IR_FAULT_BUSY);
    CHECK_EQ(s.counters.busy, 1);
    CHECK_EQ(irHw.recoverCalls, 1);
    irHw.ready = true;
    now = 1020U; Svc();
    CHECK_EQ(irHw.startCalls, 1);
    CritBalanced(__LINE__);
}

static void test_dma_and_adc_errors(void)
{
    AppIrSnapshot s;

    Boot(1000U);
    Svc();
    IrHwFake_Error(HAL_ADC_ERROR_DMA_VALUE);
    now += 10U; Svc();
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_ERROR);
    CHECK_EQ(s.faultMask, APP_IR_FAULT_DMA);
    CHECK_EQ(s.counters.dmaError, 1);
    CHECK_EQ(irHw.recoverCalls, 1);

    Boot(1000U);
    Svc();
    IrHwFake_Error(HAL_ADC_ERROR_OVR_VALUE);
    now += 10U; Svc();
    s = Snap();
    CHECK_EQ(s.faultMask, APP_IR_FAULT_ADC);
    CHECK_EQ(s.counters.adcError, 1);

    /* DMA error with the overrun flag also set: both bits. */
    Boot(1000U);
    Svc();
    irHw.overrun = true;
    IrHwFake_Error(HAL_ADC_ERROR_DMA_VALUE);
    now += 10U; Svc();
    CHECK_EQ(Snap().faultMask, APP_IR_FAULT_DMA | APP_IR_FAULT_ADC);
    CHECK(!irHw.overrun);

    /* ADC error with the overrun flag: one ADC bit, counted once. */
    Boot(1000U);
    Svc();
    irHw.overrun = true;
    IrHwFake_Error(HAL_ADC_ERROR_OVR_VALUE);
    now += 10U; Svc();
    CHECK_EQ(Snap().faultMask, APP_IR_FAULT_ADC);
    CHECK_EQ(Snap().counters.adcError, 1);

    /* Error, then the HAL's "ConvCplt in error state" path calls the error
     * callback again (or a completion follows): one publication, the rest
     * spurious. */
    Boot(1000U);
    Svc();
    IrHwFake_Error(HAL_ADC_ERROR_DMA_VALUE);
    IrHwFake_Error(HAL_ADC_ERROR_DMA_VALUE);
    IrHwFake_CompleteNoWrite();
    now += 10U; Svc();
    s = Snap();
    CHECK_EQ(s.sequence, 1);
    CHECK_EQ(s.faultMask, APP_IR_FAULT_DMA);
    CHECK_EQ(s.counters.spurious, 2);
    CHECK_EQ(s.counters.dmaError, 1);

    /* Errors are transient: the next scan on schedule is VALID again. */
    now += 10U;
    GoodCycle(321U, 654U);
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_VALID);
    CHECK_EQ(s.leftRaw, 321);
    CHECK_EQ(s.faultMask, 0);
    CritBalanced(__LINE__);
}

static void test_no_consecutive_failure_latch(void)
{
    AppIrSnapshot s;
    unsigned i;

    /* 300 timeouts in a row never latch FAULT; the counter saturates at 255
     * and resets on the first VALID scan. */
    Boot(1000U);
    for (i = 0U; i < 300U; i++)
    {
        Svc();                               /* start */
        now += 10U; Svc();                   /* timeout */
        now += 10U;
    }
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_ERROR);
    CHECK_EQ(s.consecutiveFailures, 255);
    CHECK_EQ(s.counters.timeout, 300);
    CHECK_EQ(irHw.startCalls, 300);
    GoodCycle(1U, 2U);
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_VALID);
    CHECK_EQ(s.consecutiveFailures, 0);
    CHECK_EQ(s.counters.recoveries, 300);
    CritBalanced(__LINE__);
}

static void test_recovery_failure_latches(void)
{
    AppIrSnapshot s;
    unsigned i;
    uint32_t seq;

    Boot(1000U);
    irHw.recoverResult = false;
    Svc();
    now = 1010U; Svc();                      /* timeout -> recover fails */
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_FAULT);
    CHECK_EQ(s.faultMask, APP_IR_FAULT_TIMEOUT | APP_IR_FAULT_RECOVERY);
    CHECK_EQ(s.counters.recoveries, 0);
    seq = s.sequence;

    /* Never touches the ADC again, even if the hardware "comes back". */
    irHw.recoverResult = true;
    for (i = 0U; i < 200U; i++) { now += 10U; Svc(); }
    s = Snap();
    CHECK_EQ(s.sequence, seq);
    CHECK_EQ(s.status, APP_IR_STATUS_FAULT);
    CHECK_EQ(irHw.startCalls, 1);
    CHECK_EQ(irHw.readyCalls, 1);
    CHECK_EQ(irHw.recoverCalls, 1);
    CHECK(!AppIr_IsFresh(&s, now));
    CHECK_EQ(AppIr_ReportMask(&s, now),
             APP_IR_FAULT_TIMEOUT | APP_IR_FAULT_RECOVERY | APP_IR_FAULT_STALE);

    /* Recovery failing after a busy / start failure latches the same way. */
    Boot(1000U);
    irHw.ready = false;
    irHw.recoverResult = false;
    Svc();
    CHECK_EQ(Snap().faultMask, APP_IR_FAULT_BUSY | APP_IR_FAULT_RECOVERY);
    CHECK_EQ(Snap().status, APP_IR_STATUS_FAULT);

    Boot(1000U);
    irHw.startResult = false;
    irHw.recoverResult = false;
    Svc();
    CHECK_EQ(Snap().faultMask, APP_IR_FAULT_START | APP_IR_FAULT_RECOVERY);
    now += 20U; Svc();
    CHECK_EQ(irHw.startCalls, 1);
    CritBalanced(__LINE__);
}

static void test_config_faults(void)
{
    static const uint16_t cases[] = {
        APP_IR_FAULT_CFG_ADC,
        APP_IR_FAULT_CFG_DMA,
        APP_IR_FAULT_CFG_NVIC,
        APP_IR_FAULT_CFG_ADC | APP_IR_FAULT_CFG_DMA | APP_IR_FAULT_CFG_NVIC
    };
    unsigned c;
    unsigned i;

    for (c = 0U; c < sizeof(cases) / sizeof(cases[0]); c++)
    {
        AppIrSnapshot s;

        IrHwFake_Reset();
        CritReset();
        irHw.cfgFault = cases[c];
        now = 2000U;
        AppIr_Init();
        CHECK_EQ(Snap().sequence, 0);        /* published by the first Service */

        Svc();
        s = Snap();
        CHECK_EQ(s.sequence, 1);
        CHECK_EQ(s.status, APP_IR_STATUS_FAULT);
        CHECK_EQ(s.faultMask, cases[c]);
        CHECK_EQ(s.publishTick, 2000);

        for (i = 0U; i < 500U; i++) { now += 10U; Svc(); }
        s = Snap();
        CHECK_EQ(s.sequence, 1);             /* published exactly once */
        CHECK_EQ(irHw.readyCalls, 0);        /* fail closed: ADC never used */
        CHECK_EQ(irHw.startCalls, 0);
        CHECK_EQ(irHw.recoverCalls, 0);
        CHECK(!AppIr_IsFresh(&s, now));
        CHECK_EQ(AppIr_ReportMask(&s, now), cases[c] | APP_IR_FAULT_STALE);
        CritBalanced(__LINE__);
    }
}

static void test_task_create_failure(void)
{
    AppIrSnapshot s;
    unsigned i;

    Boot(1000U);                             /* zero state, as after reset */
    IrHwFake_Reset();
    CritReset();

    AppIr_ReportTaskCreateFailure();
    CHECK_EQ(FakeCrit_Enters, 0);            /* pre-scheduler: no crit sect */
    CHECK_EQ(irHw.checkCalls + irHw.readyCalls + irHw.startCalls +
             irHw.recoverCalls, 0);          /* no hardware touched */

    s = Snap();
    CHECK_EQ(s.sequence, 1);
    CHECK_EQ(s.status, APP_IR_STATUS_FAULT);
    CHECK_EQ(s.faultMask, APP_IR_FAULT_TASK);
    CHECK_EQ(AppIr_ReportMask(&s, 0U), APP_IR_FAULT_TASK | APP_IR_FAULT_STALE);

    /* Even if Service() were called, nothing would happen. */
    for (i = 0U; i < 10U; i++) { now += 10U; Svc(); }
    CHECK_EQ(Snap().sequence, 1);
    CHECK_EQ(irHw.startCalls, 0);
}

static void test_sequence_wrap(void)
{
    Boot(1000U);
    AppIr_TestSetSequence(0xFFFFFFFEU);
    GoodCycle(1U, 1U);
    CHECK_EQ(Snap().sequence, 0xFFFFFFFFU);
    now += 10U;
    GoodCycle(2U, 2U);
    CHECK_EQ(Snap().sequence, 1);            /* 0 is skipped on wrap */
    CHECK_EQ(Snap().leftRaw, 2);
}

static void test_tick_wrap(void)
{
    AppIrSnapshot s;

    Boot(0xFFFFFFF0U);
    Svc();                                   /* start @FFFFFFF0 */
    IrHwFake_Complete(42U, 43U);
    now = 0xFFFFFFFAU; Svc();
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_VALID);
    CHECK_EQ(s.publishTick, 0xFFFFFFFAU);

    now = 0x00000003U; Svc();                /* 19 ticks after start */
    CHECK_EQ(irHw.startCalls, 1);
    now = 0x00000004U; Svc();                /* 20 ticks: start across wrap */
    CHECK_EQ(irHw.startCalls, 2);
    now = 0x00000006U; Svc();                /* 2 ticks: still in flight */
    CHECK_EQ(Snap().sequence, 1);
    now = 0x00000007U; Svc();                /* 3 ticks: timeout */
    CHECK_EQ(Snap().faultMask, APP_IR_FAULT_TIMEOUT);

    /* Freshness across the wrap (publishTick FFFFFFFA). */
    CHECK(AppIr_IsFresh(&s, 0xFFFFFFFAU + APP_IR_STALE_TICKS));
    CHECK(!AppIr_IsFresh(&s, 0xFFFFFFFAU + APP_IR_STALE_TICKS + 1U));
    CritBalanced(__LINE__);
}

static void test_freshness_helpers(void)
{
    AppIrSnapshot s;

    Boot(1000U);
    GoodCycle(10U, 20U);                     /* published @1010 */
    s = Snap();
    CHECK(AppIr_IsFresh(&s, 1010U));
    CHECK(AppIr_IsFresh(&s, 1010U + APP_IR_STALE_TICKS));
    CHECK(!AppIr_IsFresh(&s, 1010U + APP_IR_STALE_TICKS + 1U));
    CHECK_EQ(AppIr_ReportMask(&s, 1010U + APP_IR_STALE_TICKS), 0);
    CHECK_EQ(AppIr_ReportMask(&s, 1010U + APP_IR_STALE_TICKS + 1U),
             APP_IR_FAULT_STALE);
    CHECK_EQ(s.faultMask & APP_IR_FAULT_STALE, 0);   /* never stored */

    /* An ERROR publication is never fresh, even at age 0. */
    now += 10U; Svc();
    now += 10U; Svc();                       /* timeout */
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_ERROR);
    CHECK(!AppIr_IsFresh(&s, s.publishTick));
    CHECK_EQ(AppIr_ReportMask(&s, s.publishTick),
             APP_IR_FAULT_TIMEOUT | APP_IR_FAULT_STALE);
    CHECK_EQ(s.lastValidTick, 1010);         /* last good one remembered */

    CHECK(!AppIr_IsFresh(NULL, 0U));
    CHECK_EQ(AppIr_ReportMask(NULL, 0U), APP_IR_FAULT_STALE);
}

static void test_counter_saturation(void)
{
    AppIrSnapshot s;
    unsigned i;

    Boot(1000U);
    for (i = 0U; i < 70000U; i++)
    {
        GoodCycle(1U, 1U);
        now += 10U;
    }
    s = Snap();
    CHECK_EQ(s.counters.valid, UINT16_MAX);
    CHECK_EQ(s.sequence, 70000);

    for (i = 0U; i < 70000U; i++) { AppIr_OnConvComplete(); }  /* not armed */
    CHECK_EQ(Snap().counters.spurious, UINT16_MAX);
    CHECK_EQ(Snap().sequence, 70000);
    CHECK_EQ(irHw.overlaps, 0);
}

static void test_critical_sections(void)
{
    unsigned before;

    Boot(1000U);
    Svc();

    /* The ISR entry points never take a critical section. */
    before = FakeCrit_Enters;
    IrHwFake_Complete(1U, 2U);
    AppIr_OnConvComplete();
    AppIr_OnError(HAL_ADC_ERROR_DMA_VALUE);
    CHECK_EQ(FakeCrit_Enters, before);

    /* GetSnapshot: exactly one balanced section. */
    before = FakeCrit_Enters;
    (void)Snap();
    CHECK_EQ(FakeCrit_Enters, before + 1U);
    CHECK_EQ(FakeCrit_Depth, 0);

    /* The HAL port is never called inside a critical section (all paths). */
    now += 10U; Svc();
    irHw.startResult = false;
    now += 10U; Svc();
    irHw.startResult = true;
    irHw.ready = false;
    now += 20U; Svc();
    irHw.ready = true;
    now += 20U; Svc();
    now += 10U; Svc();                       /* timeout + recover */
    CHECK_EQ(irHw.halInCritical, 0);
    CritBalanced(__LINE__);
}

/* Randomised wake jitter and outcomes: the invariants must always hold. */
static void test_randomised_invariants(void)
{
    uint32_t rng = 12345U;
    uint32_t lastStart = 0U;
    bool haveStart = false;
    uint32_t lastSeq = 0U;
    unsigned i;

    Boot(0xFFFF0000U);                       /* crosses the tick wrap too */

    for (i = 0U; i < 200000U; i++)
    {
        unsigned startsBefore = irHw.startCalls;
        AppIrSnapshot s;
        uint32_t r;

        rng = rng * 1103515245U + 12345U;
        r = (rng >> 16) & 0x7FFFU;

        irHw.ready = (r % 50U) != 0U;
        irHw.startResult = (r % 37U) != 0U;
        irHw.inside = ((r % 23U) == 0U) ? IRFAKE_INSIDE_COMPLETE : IRFAKE_INSIDE_NONE;
        irHw.insideLeft = (uint16_t)(r & 0xFFFU);
        irHw.insideRight = (uint16_t)((r >> 3) & 0xFFFU);

        Svc();

        if (irHw.startCalls != startsBefore)
        {
            CHECK(!haveStart || ((uint32_t)(now - lastStart) >= APP_IR_SAMPLE_PERIOD_TICKS));
            lastStart = now;
            haveStart = true;
        }

        if (irHw.inFlight)
        {
            switch (r % 7U)
            {
                case 0: case 1: case 2: case 3:
                    IrHwFake_Complete((uint16_t)(r & 0xFFFU), (uint16_t)((r * 7U) & 0xFFFU));
                    break;
                case 4:
                    IrHwFake_Error(((r >> 4) & 1U) ? HAL_ADC_ERROR_DMA_VALUE : HAL_ADC_ERROR_OVR_VALUE);
                    break;
                default:
                    break;                   /* left to time out */
            }
        }

        s = Snap();
        CHECK(s.sequence >= lastSeq);
        lastSeq = s.sequence;
        if (s.status == APP_IR_STATUS_VALID)
        {
            if ((s.faultMask != 0U) || (s.leftRaw > APP_IR_ADC_MAX) ||
                (s.rightRaw > APP_IR_ADC_MAX))
            {
                CHECK(0);
                break;
            }
        }
        else if (s.status == APP_IR_STATUS_ERROR)
        {
            CHECK(s.faultMask != 0U);
            CHECK((s.faultMask & (APP_IR_FAULT_LATCHED_MASK | APP_IR_FAULT_STALE)) == 0U);
        }

        now += 1U + ((rng >> 8) % 25U);
    }

    CHECK_EQ(irHw.overlaps, 0);
    CHECK_EQ(irHw.halInCritical, 0);
    CHECK(Snap().counters.valid > 1000U);
    CritBalanced(__LINE__);
}

int main(void)
{
    test_init_and_first_scan();
    test_mapping_and_valid_values();
    test_cadence_and_no_overlap();
    test_completion_inside_start();
    test_data_integrity();
    test_timeout();
    test_start_and_busy_failures();
    test_dma_and_adc_errors();
    test_no_consecutive_failure_latch();
    test_recovery_failure_latches();
    test_config_faults();
    test_task_create_failure();
    test_sequence_wrap();
    test_tick_wrap();
    test_freshness_helpers();
    test_counter_saturation();
    test_critical_sections();
    test_randomised_invariants();

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
