/* Host test of the Phase 3 IR reading (Core/Src/app_ir.c + app_ir_dist.c):
 * the filter fed by real acquisition cycles, start-up hold-off, warm-up,
 * the OK / WARMUP / STALE / ERROR / FAULT decision of AppIr_GetReading(),
 * LEFT/RIGHT mapping, 0 / 4095, freshness boundaries, filter survival of a
 * single failed scan, reset after a gap, latched faults, tick wrap-around,
 * critical-section use, and a differential random run against an independent
 * model of the filter rules. The ADC/DMA port is replaced by
 * tests/ir/fakes/ir_hw_fake.c, as in app_ir_test.c.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -Werror -DAPP_IR_ENABLE=1 -DAPP_IR_TEST_HOOKS \
 *       -Itests/ir/fakes -Itests/park_trace/fakes -ICore/Inc \
 *       -o tests/ir/app_ir_reading_test.exe tests/ir/app_ir_reading_test.c \
 *       Core/Src/app_ir.c Core/Src/app_ir_dist.c tests/ir/fakes/ir_hw_fake.c
 *   ./tests/ir/app_ir_reading_test.exe
 * It follows the filter constants, so the alternative builds listed in
 * app_ir_dist_test.c (-DAPP_IR_FILTER_WINDOW / -DAPP_IR_FILTER_RANK) pass too.
 */

#include "app_ir.h"
#include "ir_hw_fake.h"

#include <stdio.h>
#include <stdlib.h>
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

#define HAL_ADC_ERROR_DMA_VALUE  0x04U

static uint32_t now;

static void Svc(void) { AppIr_Service(now); }

static AppIrSnapshot Snap(void)
{
    AppIrSnapshot s;
    AppIr_GetSnapshot(&s);
    return s;
}

static AppIrReading Read(void)
{
    AppIrReading r;
    AppIr_GetReading(&r, now);
    return r;
}

static void Boot(uint32_t startTick)
{
    IrHwFake_Reset();
    FakeCrit_Depth = 0;
    FakeCrit_MaxDepth = 0;
    FakeCrit_Enters = 0U;
    FakeCrit_Underflows = 0U;
    now = startTick;
    AppIr_Init();
}

/* One VALID scan: start at `now` (must be due), complete, collect 10 ticks
 * later, then move to the next start (20 ticks after this one's start). */
static void GoodCycle(uint16_t l, uint16_t r)
{
    Svc();
    IrHwFake_Complete(l, r);
    now += 10U;
    Svc();
    now += 10U;
}

/* One scan that never completes: collected as a timeout 10 ticks later. */
static void TimeoutCycle(void)
{
    Svc();
    now += 10U;
    Svc();
    now += 10U;
}

/* Number of publications at 10 + 20k ticks after the first start that are
 * inside the start-up hold-off. */
static unsigned HoldoffCycles(void)
{
    unsigned k = 0U;

    while ((10U + 20U * k) < APP_IR_DIST_STARTUP_TICKS) { k++; }
    return k;
}

static void CheckNoNumbers(const AppIrReading *r)
{
    unsigned i;

    for (i = 0U; i < 2U; i++)
    {
        CHECK_EQ(r->range[i], APP_IR_RANGE_INVALID);
        CHECK_EQ(r->distanceMm[i], 0);
        CHECK_EQ(r->filteredRaw[i], 0);
    }
}

/* Runs good cycles until the filter is full; returns the number run. */
static unsigned FillFilter(uint16_t l, uint16_t r)
{
    unsigned n = 0U;

    while (Snap().filterSamples < APP_IR_FILTER_WINDOW)
    {
        GoodCycle(l, r);
        n++;
        if (n > 100U) { break; }
    }
    return n;
}

/* ---------------------------------------------------------------------- */

static void test_never_published(void)
{
    AppIrReading r;

    Boot(1000U);
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_STALE);
    CHECK_EQ(r.sequence, 0);
    CHECK_EQ(r.ageTicks, UINT32_MAX);
    CHECK_EQ(r.faultMask, APP_IR_FAULT_STALE);
    CheckNoNumbers(&r);
}

static void test_holdoff_and_warmup(void)
{
    unsigned k;
    unsigned cycles;
    AppIrReading r;
    AppIrSnapshot s;
    unsigned hold = HoldoffCycles();

    Boot(1000U);

    /* During the hold-off the raw value is published (VALID) but not
     * filtered: the reading is WARMUP with no numbers. */
    for (k = 0U; k < hold; k++)
    {
        GoodCycle(2000U, 2000U);
        s = Snap();
        CHECK_EQ(s.status, APP_IR_STATUS_VALID);
        CHECK_EQ(s.leftRaw, 2000);
        CHECK_EQ(s.filterSamples, 0);
        r = Read();
        CHECK_EQ(r.acq, APP_IR_ACQ_WARMUP);
        CheckNoNumbers(&r);
        CHECK_EQ(r.faultMask, 0);           /* warm-up is not a fault */
    }

    /* Then one sample per cycle until the window is full. */
    for (k = 1U; k < APP_IR_FILTER_WINDOW; k++)
    {
        GoodCycle(2000U, 2000U);
        s = Snap();
        CHECK_EQ(s.filterSamples, k);
        CHECK_EQ(Read().acq, APP_IR_ACQ_WARMUP);
    }

    cycles = 1U;
    GoodCycle(2000U, 2000U);                /* the window-th filtered sample */
    s = Snap();
    CHECK_EQ(s.filterSamples, APP_IR_FILTER_WINDOW);
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_OK);
    CHECK_EQ(r.filteredRaw[0], 2000);
    CHECK_EQ(r.filteredRaw[1], 2000);
    CHECK_EQ(r.range[0], APP_IR_RANGE_OK);
    CHECK_EQ(r.range[1], APP_IR_RANGE_OK);
    CHECK_EQ(r.faultMask, 0);
    CHECK(cycles == 1U);
    /* Constant 2000 counts: each sensor's own curve. */
    CHECK_EQ(r.distanceMm[0], 154);     /* 31785.22973379/2026.48617844 - 0.29510522 = 15.390 cm */
    CHECK_EQ(r.distanceMm[1], 162);     /* 32831.45/1905.3176 - 1.02464 = 16.207 cm */
}

static void test_channel_mapping_and_median(void)
{
    AppIrReading r;
    unsigned i;
    uint16_t mm;

    Boot(1000U);
    (void)FillFilter(1500U, 2500U);         /* LEFT 1500, RIGHT 2500 */
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_OK);
    CHECK_EQ(r.filteredRaw[0], 1500);
    CHECK_EQ(r.filteredRaw[1], 2500);
    (void)AppIrDist_Convert(APP_IR_SENSOR_LEFT, 1500U, &mm);
    CHECK_EQ(r.distanceMm[0], mm);
    (void)AppIrDist_Convert(APP_IR_SENSOR_RIGHT, 2500U, &mm);
    CHECK_EQ(r.distanceMm[1], mm);

    /* An isolated upward spike on each sensor never reaches the output of a
     * median; with other ranks it may, so only check the default. */
#if (APP_IR_FILTER_WINDOW == 9U) && (APP_IR_FILTER_RANK == 4U)
    for (i = 0U; i < 4U; i++)
    {
        GoodCycle(3500U, 3900U);            /* 4 spikes in the window */
        r = Read();
        CHECK_EQ(r.filteredRaw[0], 1500);
        CHECK_EQ(r.filteredRaw[1], 2500);
    }
    GoodCycle(3500U, 3900U);                /* 5th: the median moves */
    r = Read();
    CHECK_EQ(r.filteredRaw[0], 3500);
    CHECK_EQ(r.filteredRaw[1], 3900);
#else
    (void)i;
#endif
}

static void test_range_states_and_saturation(void)
{
    AppIrReading r;
    AppIrSnapshot s;

    /* Filtered 0 on LEFT is FAR, filtered 4095 on RIGHT is NEAR: still an OK
     * acquisition, no fault bit, no distance. */
    Boot(1000U);
    (void)FillFilter(0U, APP_IR_ADC_MAX);
    r = Read();
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_VALID);
    CHECK_EQ(s.faultMask, 0);
    CHECK_EQ(r.acq, APP_IR_ACQ_OK);
    CHECK_EQ(r.faultMask, 0);
    CHECK_EQ(r.range[0], APP_IR_RANGE_FAR);
    CHECK_EQ(r.range[1], APP_IR_RANGE_NEAR);
    CHECK_EQ(r.distanceMm[0], 0);
    CHECK_EQ(r.distanceMm[1], 0);
    CHECK_EQ(r.filteredRaw[0], 0);
    CHECK_EQ(r.filteredRaw[1], 4095);

    /* Swapped. */
    Boot(1000U);
    (void)FillFilter(APP_IR_ADC_MAX, 0U);
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_OK);
    CHECK_EQ(r.range[0], APP_IR_RANGE_NEAR);
    CHECK_EQ(r.range[1], APP_IR_RANGE_FAR);

    /* Domain edges, each sensor on its own bounds. */
    Boot(1000U);
    (void)FillFilter(APP_IR_LEFT_ADC_NEAR_LIMIT, APP_IR_RIGHT_ADC_FAR_LIMIT);
    r = Read();
    CHECK_EQ(r.range[0], APP_IR_RANGE_OK);
    CHECK_EQ(r.distanceMm[0], 100);
    CHECK_EQ(r.range[1], APP_IR_RANGE_OK);
    CHECK_EQ(r.distanceMm[1], 400);

    Boot(1000U);
    (void)FillFilter(APP_IR_LEFT_ADC_NEAR_LIMIT + 1U, APP_IR_RIGHT_ADC_FAR_LIMIT - 1U);
    r = Read();
    CHECK_EQ(r.range[0], APP_IR_RANGE_NEAR);
    CHECK_EQ(r.range[1], APP_IR_RANGE_FAR);
}

/* The calibration medians at the endpoints: RIGHT 40 cm (893) sits 1 mm
 * beyond its curve's 400 mm point, RIGHT 10 cm (3063) and LEFT 10 cm (3026)
 * at about 100 mm. All are converted (guard band), none is NEAR/FAR. */
static void test_calibrated_endpoints(void)
{
    AppIrReading r;

    Boot(1000U);
    (void)FillFilter(776U, 893U);                   /* LEFT 40 cm, RIGHT 40 cm medians */
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_OK);
    CHECK_EQ(r.range[0], APP_IR_RANGE_OK);
    CHECK_EQ(r.distanceMm[0], 393);                 /* the LEFT fit residual, not a bound issue */
    CHECK_EQ(r.range[1], APP_IR_RANGE_OK);
    CHECK_EQ(r.distanceMm[1], 400);                 /* 401.0 mm on the curve, clamped */

    Boot(1000U);
    (void)FillFilter(3026U, 3063U);                 /* LEFT 10 cm, RIGHT 10 cm medians */
    r = Read();
    CHECK_EQ(r.range[0], APP_IR_RANGE_OK);
    CHECK_EQ(r.distanceMm[0], 101);
    CHECK_EQ(r.range[1], APP_IR_RANGE_OK);
    CHECK_EQ(r.distanceMm[1], 100);
}

static void test_freshness_boundary(void)
{
    AppIrReading r;
    uint32_t publish;

    Boot(1000U);
    (void)FillFilter(2000U, 2000U);
    publish = Snap().publishTick;
    now = publish + APP_IR_STALE_TICKS;         /* exactly 100 ticks old */
    r = Read();
    CHECK_EQ(r.ageTicks, APP_IR_STALE_TICKS);
    CHECK_EQ(r.acq, APP_IR_ACQ_OK);
    CHECK_EQ(r.faultMask, 0);

    now = publish + APP_IR_STALE_TICKS + 1U;    /* 101: stale */
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_STALE);
    CHECK_EQ(r.faultMask, APP_IR_FAULT_STALE);
    CHECK_EQ(r.ageTicks, APP_IR_STALE_TICKS + 1U);
    CheckNoNumbers(&r);
}

static void test_single_error_keeps_filter(void)
{
    AppIrReading r;
    AppIrSnapshot s;

    Boot(1000U);
    (void)FillFilter(2000U, 2000U);

    TimeoutCycle();                              /* one failed scan */
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_ERROR);
    CHECK_EQ(s.filterSamples, 0);                /* nothing carried in an ERROR */
    CHECK_EQ(s.filteredRaw[0], 0);
    r = Read();
    now -= 10U;                                  /* read at the collection tick */
    r = Read();
    now += 10U;
    CHECK_EQ(r.acq, APP_IR_ACQ_ERROR);
    CHECK(r.faultMask & APP_IR_FAULT_TIMEOUT);
    CheckNoNumbers(&r);

    /* The next good scan is OK at once: the window survived. */
    GoodCycle(2000U, 2000U);
    s = Snap();
    CHECK_EQ(s.filterSamples, APP_IR_FILTER_WINDOW);
    CHECK_EQ(Read().acq, APP_IR_ACQ_OK);
}

static void test_gap_resets_filter(void)
{
    AppIrSnapshot s;
    unsigned i;

    /* Last VALID at T. Errors at T+20..T+80 (4 of them), next VALID at
     * T+100: gap exactly APP_IR_STALE_TICKS -> the window survives. */
    Boot(1000U);
    (void)FillFilter(2000U, 2000U);
    for (i = 0U; i < 4U; i++) { TimeoutCycle(); }
    GoodCycle(2000U, 2000U);
    s = Snap();
    CHECK_EQ((uint32_t)(s.lastValidTick), (uint32_t)(s.publishTick));
    CHECK_EQ(s.filterSamples, APP_IR_FILTER_WINDOW);

    /* One more error: gap 120 > 100 -> reset; the next VALID starts again. */
    Boot(1000U);
    (void)FillFilter(2000U, 2000U);
    for (i = 0U; i < 5U; i++) { TimeoutCycle(); }
    GoodCycle(2100U, 2100U);
    s = Snap();
    CHECK_EQ(s.status, APP_IR_STATUS_VALID);
    CHECK_EQ(s.filterSamples, 1);
    CHECK_EQ(Read().acq, (APP_IR_FILTER_WINDOW == 1U) ? APP_IR_ACQ_OK : APP_IR_ACQ_WARMUP);
    (void)FillFilter(2100U, 2100U);
    CHECK_EQ(Read().filteredRaw[0], 2100);       /* no trace of the old window */
}

/* The priority FAULT > STALE > ERR > WARMUP > range state:
 *   - a FRESH failed acquisition reports ERR (age <= 100 ticks);
 *   - the same publication, once older than 100 ticks, reports STALE (IRTask
 *     stopped publishing), keeping its transient bit in the mask;
 *   - a latched fault reports FAULT at every age, and recovery never happens. */
static void test_state_priority_err_stale_fault(void)
{
    AppIrSnapshot s;
    AppIrReading r;
    uint32_t publish;

    Boot(1000U);
    (void)FillFilter(2000U, 2000U);
    TimeoutCycle();                                 /* one failed scan */
    s = Snap();
    publish = s.publishTick;
    CHECK_EQ(s.status, APP_IR_STATUS_ERROR);

    now = publish;                                  /* fresh */
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_ERROR);
    CHECK_EQ(r.ageTicks, 0);
    CHECK(r.faultMask & APP_IR_FAULT_TIMEOUT);

    now = publish + APP_IR_STALE_TICKS;             /* exactly at the limit */
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_ERROR);
    CHECK_EQ(r.ageTicks, APP_IR_STALE_TICKS);

    now = publish + APP_IR_STALE_TICKS + 1U;        /* one tick older */
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_STALE);
    CHECK(r.faultMask & APP_IR_FAULT_TIMEOUT);      /* the cause is still visible */
    CHECK(r.faultMask & APP_IR_FAULT_STALE);
    CheckNoNumbers(&r);

    now = publish + 600000U;
    CHECK_EQ(Read().acq, APP_IR_ACQ_STALE);

    /* Latched fault: FAULT at every age, never STALE or ERR. */
    Boot(1000U);
    (void)FillFilter(2000U, 2000U);
    irHw.recoverResult = false;
    TimeoutCycle();
    s = Snap();
    publish = s.publishTick;
    CHECK_EQ(s.status, APP_IR_STATUS_FAULT);

    now = publish;
    CHECK_EQ(Read().acq, APP_IR_ACQ_FAULT);
    now = publish + APP_IR_STALE_TICKS;
    CHECK_EQ(Read().acq, APP_IR_ACQ_FAULT);
    now = publish + APP_IR_STALE_TICKS + 1U;
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_FAULT);
    CHECK(r.faultMask & APP_IR_FAULT_RECOVERY);
    now = publish + 0x7FFFFFFFU;
    CHECK_EQ(Read().acq, APP_IR_ACQ_FAULT);
}

static void test_latched_faults(void)
{
    AppIrReading r;
    uint32_t t;

    /* Configuration fault: published once by the first Service. */
    Boot(1000U);
    irHw.cfgFault = APP_IR_FAULT_CFG_ADC;
    AppIr_Init();
    Svc();
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_FAULT);
    CHECK(r.faultMask & APP_IR_FAULT_CFG_ADC);
    CheckNoNumbers(&r);
    CHECK_EQ(irHw.startCalls, 0);

    /* A latched FAULT stays a FAULT however old it is. */
    now += 60000U;
    CHECK_EQ(Read().acq, APP_IR_ACQ_FAULT);

    /* IRTask could not be created. */
    Boot(1000U);
    AppIr_ReportTaskCreateFailure();
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_FAULT);
    CHECK(r.faultMask & APP_IR_FAULT_TASK);
    CheckNoNumbers(&r);

    /* Recovery failure after good readings: FAULT for good. */
    Boot(1000U);
    (void)FillFilter(2000U, 2000U);
    CHECK_EQ(Read().acq, APP_IR_ACQ_OK);
    irHw.recoverResult = false;
    TimeoutCycle();
    t = now;
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_FAULT);
    CHECK(r.faultMask & APP_IR_FAULT_RECOVERY);
    CheckNoNumbers(&r);

    GoodCycle(2000U, 2000U);                     /* nothing starts any more */
    CHECK_EQ(Read().acq, APP_IR_ACQ_FAULT);
    now = t + 100000U;
    Svc();
    CHECK_EQ(Read().acq, APP_IR_ACQ_FAULT);
}

static void test_tick_wrap(void)
{
    AppIrReading r;
    unsigned n;

    Boot(0xFFFFFF00U);                           /* wraps after 256 ticks */
    n = FillFilter(1800U, 1700U);
    CHECK(n >= HoldoffCycles());
    CHECK(now < 0x10000000U || now > 0xF0000000U);
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_OK);
    CHECK_EQ(r.filteredRaw[0], 1800);
    CHECK_EQ(r.filteredRaw[1], 1700);
    CHECK(r.ageTicks <= 20U);

    /* Cross the wrap with the filter running. */
    for (n = 0U; n < 40U; n++) { GoodCycle(1800U, 1700U); }
    CHECK(now < 0xFFFFFF00U);                    /* we did wrap */
    r = Read();
    CHECK_EQ(r.acq, APP_IR_ACQ_OK);
    CHECK_EQ(r.filteredRaw[0], 1800);
}

static void test_critical_sections(void)
{
    AppIrReading r;

    Boot(1000U);
    (void)FillFilter(2000U, 2000U);
    FakeCrit_Depth = 0;
    FakeCrit_MaxDepth = 0;
    FakeCrit_Enters = 0U;
    FakeCrit_Underflows = 0U;

    r = Read();
    (void)r;
    CHECK_EQ(FakeCrit_Enters, 1);                /* exactly one snapshot copy */
    CHECK_EQ(FakeCrit_Depth, 0);
    CHECK_EQ(FakeCrit_MaxDepth, 1);
    CHECK_EQ(FakeCrit_Underflows, 0);

    AppIr_GetReading(0, now);                    /* null is ignored */
    CHECK_EQ(FakeCrit_Enters, 1);
}

/* ---- differential random run against a model of the filter rules ----- */

static uint32_t rng = 20261012U;

static uint32_t Rnd(void)
{
    rng = rng * 1664525U + 1013904223U;
    return rng >> 8;
}

static int CmpU16(const void *a, const void *b)
{
    unsigned x = *(const uint16_t *)a;
    unsigned y = *(const uint16_t *)b;

    return (x > y) - (x < y);
}

static uint16_t RandomCount(void)
{
    switch (Rnd() % 8U)
    {
        case 0:  return 0U;
        case 1:  return APP_IR_ADC_MAX;
        case 2:  return (uint16_t)(700U + Rnd() % 100U);
        case 3:  return (uint16_t)(3000U + Rnd() % 100U);
        default: return (uint16_t)(Rnd() % 4096U);
    }
}

/* State, age and fault mask are always mutually consistent: this is what makes
 * every documented protocol example a state the driver can really produce. */
static void CheckConsistent(const AppIrReading *r, const AppIrSnapshot *s)
{
    /* State, age and fault mask are always mutually consistent: this
     * is what makes every documented protocol example a state the
     * driver can really produce. */
    switch (r->acq)
    {
        case APP_IR_ACQ_OK:
        case APP_IR_ACQ_WARMUP:
            CHECK(r->ageTicks <= APP_IR_STALE_TICKS);
            CHECK_EQ(r->faultMask, 0);       /* fresh and healthy: F:00 */
            break;
        case APP_IR_ACQ_ERROR:
            CHECK(r->ageTicks <= APP_IR_STALE_TICKS);
            CHECK((r->faultMask & APP_IR_FAULT_STALE) != 0U);
            CHECK((r->faultMask & 0x3FU) != 0U);     /* a transient bit */
            break;
        case APP_IR_ACQ_STALE:
            CHECK((r->sequence == 0U) || (r->ageTicks > APP_IR_STALE_TICKS) ||
                  (s->status == APP_IR_STATUS_NONE));
            CHECK((r->faultMask & APP_IR_FAULT_STALE) != 0U);
            break;
        default:
            CHECK((r->faultMask & APP_IR_FAULT_LATCHED_MASK) != 0U);
            CHECK((r->faultMask & APP_IR_FAULT_STALE) != 0U);
            break;
    }
}

static void test_random_differential(void)
{
    unsigned step;
    uint16_t hist[2][8192];
    unsigned histLen = 0U;
    uint32_t firstStart = 0U;
    int haveFirstStart = 0;
    uint32_t lastValid = 0U;
    int haveLastValid = 0;
    uint32_t lastSeq = 0U;
    unsigned okReadings = 0U;
    unsigned warm = 0U;
    unsigned stale = 0U;
    unsigned err = 0U;

    Boot(0xFFFFF000U + (Rnd() % 4096U));
    memset(hist, 0, sizeof(hist));

    for (step = 0U; step < 200000U; step++)
    {
        AppIrSnapshot s;
        AppIrReading r;
        unsigned dice;

        /* One service step at `now`; the fake starts a scan when due. */
        {
            unsigned before = irHw.startCalls;

            Svc();
            if (!haveFirstStart && (irHw.startCalls > before))
            {
                firstStart = now;
                haveFirstStart = 1;
            }
        }

        if (irHw.inFlight)
        {
            dice = Rnd() % 20U;
            if (dice < 16U) { IrHwFake_Complete(RandomCount(), RandomCount()); }
            else if (dice < 18U) { /* leave it: times out */ }
            else if (dice < 19U) { IrHwFake_Error(HAL_ADC_ERROR_DMA_VALUE); }
            else { IrHwFake_Error(0x02U); }
        }

        now += 10U;
        Svc();

        s = Snap();
        if (s.sequence != lastSeq)
        {
            lastSeq = s.sequence;

            if (s.status == APP_IR_STATUS_VALID)
            {
                if (haveLastValid && ((uint32_t)(s.publishTick - lastValid) > APP_IR_STALE_TICKS))
                {
                    histLen = 0U;
                }
                lastValid = s.publishTick;
                haveLastValid = 1;

                if ((uint32_t)(s.publishTick - firstStart) >= APP_IR_DIST_STARTUP_TICKS)
                {
                    hist[0][histLen & 8191U] = s.leftRaw;
                    hist[1][histLen & 8191U] = s.rightRaw;
                    histLen++;
                }

                /* Model output vs the firmware's. */
                {
                    unsigned sensor;
                    unsigned have = (histLen < APP_IR_FILTER_WINDOW) ? histLen : APP_IR_FILTER_WINDOW;

                    CHECK_EQ(s.filterSamples, have);

                    for (sensor = 0U; sensor < 2U; sensor++)
                    {
                        uint16_t win[APP_IR_FILTER_WINDOW];
                        unsigned j;

                        if (have < APP_IR_FILTER_WINDOW)
                        {
                            CHECK_EQ(s.filteredRaw[sensor], 0);
                            continue;
                        }

                        for (j = 0U; j < APP_IR_FILTER_WINDOW; j++)
                        {
                            win[j] = hist[sensor][(histLen - APP_IR_FILTER_WINDOW + j) & 8191U];
                        }
                        qsort(win, APP_IR_FILTER_WINDOW, sizeof(win[0]), CmpU16);
                        CHECK_EQ(s.filteredRaw[sensor], win[APP_IR_FILTER_RANK]);
                    }
                }
            }
        }

        /* Reading invariants, at the current time and a little later. */
        r = Read();
        {
            unsigned sensor;

            CHECK(r.acq <= APP_IR_ACQ_FAULT);

            CheckConsistent(&r, &s);

            if (r.acq == APP_IR_ACQ_OK)
            {
                okReadings++;
                CHECK_EQ(s.status, APP_IR_STATUS_VALID);
                CHECK(r.ageTicks <= APP_IR_STALE_TICKS);
                CHECK_EQ(r.faultMask & APP_IR_FAULT_STALE, 0);
                CHECK_EQ(s.filterSamples, APP_IR_FILTER_WINDOW);

                for (sensor = 0U; sensor < 2U; sensor++)
                {
                    uint16_t mm = 0xFFFFU;
                    AppIrRange range = AppIrDist_Convert((AppIrSensor)sensor, s.filteredRaw[sensor], &mm);

                    CHECK_EQ(r.range[sensor], range);
                    CHECK_EQ(r.distanceMm[sensor], mm);
                    CHECK_EQ(r.filteredRaw[sensor], s.filteredRaw[sensor]);
                    CHECK(r.range[sensor] != APP_IR_RANGE_INVALID);

                    if (r.range[sensor] == APP_IR_RANGE_OK)
                    {
                        CHECK(r.distanceMm[sensor] >= APP_IR_DIST_MIN_MM);
                        CHECK(r.distanceMm[sensor] <= APP_IR_DIST_MAX_MM);
                    }
                    else
                    {
                        CHECK_EQ(r.distanceMm[sensor], 0);
                    }
                }
            }
            else
            {
                if (r.acq == APP_IR_ACQ_WARMUP) { warm++; }
                if (r.acq == APP_IR_ACQ_STALE) { stale++; }
                if (r.acq == APP_IR_ACQ_ERROR) { err++; }
                CheckNoNumbers(&r);
            }
        }

        /* The same publication read later (up to 250 ticks on) must still be
         * consistent: this is where STALE appears. */
        {
            AppIrReading late;
            uint32_t saved = now;

            now += Rnd() % 250U;
            late = Read();
            now = saved;
            CheckConsistent(&late, &s);
            if (late.acq == APP_IR_ACQ_STALE) { stale++; }
        }

        /* Irregular time: mostly one period, sometimes a long pause. */
        dice = Rnd() % 50U;
        now += (dice == 0U) ? (60U + Rnd() % 200U) : (dice < 5U ? Rnd() % 10U : 10U);
    }

    /* The run actually exercised every branch. */
    CHECK(okReadings > 1000U);
    CHECK((warm > 100U) || (APP_IR_FILTER_WINDOW == 1U));
    CHECK(err > 100U);
    CHECK_EQ(FakeCrit_Depth, 0);
    CHECK_EQ(FakeCrit_Underflows, 0);
    printf("random run: %u OK, %u WARMUP, %u ERROR, %u STALE readings; "
           "%u scan starts, %u overlaps\n",
           okReadings, warm, err, stale, irHw.startCalls, irHw.overlaps);
    CHECK_EQ(irHw.overlaps, 0);
}

int main(void)
{
    printf("filter: window %u, rank %u\n", (unsigned)APP_IR_FILTER_WINDOW,
           (unsigned)APP_IR_FILTER_RANK);

    test_never_published();
    test_holdoff_and_warmup();
    test_channel_mapping_and_median();
    test_range_states_and_saturation();
    test_calibrated_endpoints();
    test_freshness_boundary();
    test_single_error_keeps_filter();
    test_gap_resets_filter();
    test_state_priority_err_stale_fault();
    test_latched_faults();
    test_tick_wrap();
    test_critical_sections();
    test_random_differential();

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
