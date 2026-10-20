/* Host test of the accepted ultrasonic calibration:
 *
 *   mm = ((pulse_us * 1744 + 5000) / 10000) + (-10)
 *
 * measured to the FRONT FACES of the two transducers (no bumper offset).
 * Built twice from this one file:
 *
 *   production build  - the firmware limits: minimum valid pulse 100 us.
 *                       Pins the constants, the signed conversion, the
 *                       rounding boundaries, the 65535 us no-overflow case
 *                       and the end-to-end classification of 99/100/23500/
 *                       23501 us through the real driver.
 *   -DAPP_US_MIN_VALID_PULSE_US=1  - host-test override ONLY. The minimum
 *                       valid pulse is lowered to 1 us so the otherwise
 *                       unreachable "result <= 0 -> TOO_NEAR" branch can be
 *                       exercised: pulses 1..60 us give <= 0 mm (60 gives
 *                       exactly 0) and are TOO_NEAR; 61 us gives 1 mm and is
 *                       VALID. The compile-time guarantees in
 *                       app_ultrasonic.c are checked against the PRODUCTION
 *                       limit, so this override still compiles.
 *
 * Build (from repo root), production:
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -Itests/ultrasonic/fakes \
 *       -Itests/motion/fakes -o tests/ultrasonic/app_ultrasonic_calibration_test.exe \
 *       tests/ultrasonic/app_ultrasonic_calibration_test.c Core/Src/app_ultrasonic.c \
 *       tests/ultrasonic/fakes/ultrasonic_hw_fake.c
 * Override build: add -DAPP_US_MIN_VALID_PULSE_US=1
 */

#include "app_ultrasonic.h"
#include "ultrasonic_hw_fake.h"

#include <stdio.h>

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

/* Final production constants, pinned literally. */
#if APP_ULTRASONIC_BENCH_LOG != 0
#error "production firmware must have APP_ULTRASONIC_BENCH_LOG 0"
#endif
#if APP_US_MM_PER_US_X10000 != 1744
#error "accepted calibration slope is 1744"
#endif
#if APP_US_OFFSET_MM != (-10)
#error "accepted calibration offset is -10"
#endif
#if APP_US_MIN_VALID_PULSE_US_PRODUCTION != 100U
#error "production minimum valid pulse is 100 us"
#endif

static uint32_t tick;
static uint32_t sinceStart;

static AppUltrasonicSnapshot Snap(void)
{
    AppUltrasonicSnapshot snapshot;

    AppUltrasonic_GetSnapshot(&snapshot);
    return snapshot;
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
    }
}

static AppUltrasonicSnapshot RunCycle(uint32_t widthUs)
{
    uint32_t before = Snap().sequence;
    unsigned ms = 0U;

    FakeUs_EchoPulse(300U, widthUs);

    while ((Snap().sequence == before) && (ms < 300U))
    {
        RunMs(1U);
        ms++;
    }

    return Snap();
}

static void Begin(void)
{
    FakeUs_Reset();
    FakeUs_SetSetupFaultMask(0U);
    tick = 0U;
    sinceStart = 0U;
    AppUltrasonic_Init();
}

/* The formula, restated independently of the firmware macros. */
static long Reference(long pulse)
{
    return ((pulse * 1744L + 5000L) / 10000L) - 10L;
}

static void test_constants_and_conversion(void)
{
    CHECK_EQ(APP_US_MM_PER_US_X10000, 1744);
    CHECK_EQ(APP_US_OFFSET_MM, -10);
    CHECK_EQ(APP_ULTRASONIC_BENCH_LOG, 0);
    CHECK_EQ(APP_US_MIN_VALID_PULSE_US_PRODUCTION, 100);
    CHECK_EQ(APP_US_MAX_VALID_PULSE_US, 23500);

    /* The signed range below the production minimum. */
    CHECK_EQ(AppUltrasonic_PulseToMm(0U), -10);
    CHECK_EQ(AppUltrasonic_PulseToMm(1U), -10);
    CHECK_EQ(AppUltrasonic_PulseToMm(30U), -5);
    CHECK_EQ(AppUltrasonic_PulseToMm(54U), -1);
    CHECK_EQ(AppUltrasonic_PulseToMm(55U), 0);       /* 55..60 all give 0 */
    CHECK_EQ(AppUltrasonic_PulseToMm(59U), 0);
    CHECK_EQ(AppUltrasonic_PulseToMm(60U), 0);       /* exactly zero */
    CHECK_EQ(AppUltrasonic_PulseToMm(61U), 1);       /* first positive */
    CHECK_EQ(AppUltrasonic_PulseToMm(100U), 7);      /* production minimum */

    /* Rounding boundaries around the 200 mm stop and 220 / 400 mm limits. */
    CHECK_EQ(AppUltrasonic_PulseToMm(1201U), 199);
    CHECK_EQ(AppUltrasonic_PulseToMm(1202U), 200);
    CHECK_EQ(AppUltrasonic_PulseToMm(1206U), 200);
    CHECK_EQ(AppUltrasonic_PulseToMm(1207U), 201);
    CHECK_EQ(AppUltrasonic_PulseToMm(1208U), 201);
    CHECK_EQ(AppUltrasonic_PulseToMm(1315U), 219);
    CHECK_EQ(AppUltrasonic_PulseToMm(1316U), 220);
    CHECK_EQ(AppUltrasonic_PulseToMm(1321U), 220);
    CHECK_EQ(AppUltrasonic_PulseToMm(1322U), 221);
    CHECK_EQ(AppUltrasonic_PulseToMm(2348U), 399);
    CHECK_EQ(AppUltrasonic_PulseToMm(2349U), 400);

    /* Maximum valid pulse and the absolute 16-bit maximum: no overflow. */
    CHECK_EQ(AppUltrasonic_PulseToMm(23500U), 4088);
    CHECK_EQ(AppUltrasonic_PulseToMm(65535U), 11419);

    /* Exhaustive agreement with the independent restatement. */
    {
        uint32_t pulse;
        unsigned mismatches = 0U;

        for (pulse = 0U; pulse <= 65535U; pulse++)
        {
            if (AppUltrasonic_PulseToMm((uint16_t)pulse) !=
                Reference((long)pulse))
            {
                mismatches++;
            }
        }

        CHECK_EQ(mismatches, 0);
    }

    /* Monotonic non-decreasing in the pulse width. */
    {
        uint32_t pulse;
        unsigned decreases = 0U;

        for (pulse = 1U; pulse <= 65535U; pulse++)
        {
            if (AppUltrasonic_PulseToMm((uint16_t)pulse) <
                AppUltrasonic_PulseToMm((uint16_t)(pulse - 1U)))
            {
                decreases++;
            }
        }

        CHECK_EQ(decreases, 0);
    }
}

#if APP_US_MIN_VALID_PULSE_US == 100U

/* Production limits end to end through the real driver. */
static void test_production_classification(void)
{
    AppUltrasonicSnapshot s;

    Begin();

    s = RunCycle(99U);
    CHECK(s.result == APP_US_RESULT_TOO_NEAR);
    CHECK_EQ(s.pulseCaptured, 1);
    CHECK_EQ(s.rawValid, 0);

    s = RunCycle(100U);
    CHECK(s.result == APP_US_RESULT_VALID);
    CHECK_EQ(s.rawMm, 7);
    CHECK_EQ(s.rawValid, 1);

    s = RunCycle(1202U);
    CHECK(s.result == APP_US_RESULT_VALID);
    CHECK_EQ(s.rawMm, 200);

    s = RunCycle(1208U);
    CHECK_EQ(s.rawMm, 201);

    s = RunCycle(23500U);
    CHECK(s.result == APP_US_RESULT_VALID);
    CHECK_EQ(s.rawMm, 4088);

    s = RunCycle(23501U);
    CHECK(s.result == APP_US_RESULT_TOO_FAR);
    CHECK_EQ(s.rawValid, 0);
}

#else /* APP_US_MIN_VALID_PULSE_US overridden to 1 (host test only) */

static void test_signed_branch_override(void)
{
    AppUltrasonicSnapshot s;
    uint32_t width;

    CHECK_EQ(APP_US_MIN_VALID_PULSE_US, 1);

    Begin();

    /* 1..60 us convert to <= 0 mm: TOO_NEAR, never VALID, never 0 mm. */
    for (width = 1U; width <= 60U; width++)
    {
        s = RunCycle(width);
        CHECK(s.result == APP_US_RESULT_TOO_NEAR);
        CHECK_EQ(s.rawValid, 0);
        CHECK_EQ(s.pulseCaptured, 1);
        CHECK_EQ(s.pulseUs, width);
    }

    CHECK_EQ(Snap().filteredValid, 0);          /* nothing reached the filter */
    CHECK_EQ(Snap().counters.valid, 0);
    CHECK_EQ(Snap().counters.tooNear, 60);

    /* 61 us is exactly 1 mm and is the first VALID pulse. */
    s = RunCycle(61U);
    CHECK(s.result == APP_US_RESULT_VALID);
    CHECK_EQ(s.rawMm, 1);
    CHECK_EQ(s.rawValid, 1);
    CHECK_EQ(s.filteredMm, 1);
}

#endif

int main(void)
{
    test_constants_and_conversion();

#if APP_US_MIN_VALID_PULSE_US == 100U
    printf("build: production limits\n");
    test_production_classification();
#else
    printf("build: APP_US_MIN_VALID_PULSE_US override\n");
    test_signed_branch_override();
#endif

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
