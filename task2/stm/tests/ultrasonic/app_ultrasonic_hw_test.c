/* Host test of the REAL Core/Src/app_ultrasonic_hw.c start-up checks and
 * register operations, run against a fake TIM8 register block and HAL
 * (tests/ultrasonic/fakes_hw). It proves the driver is fail-closed:
 * when any assumed configuration differs, AppUltrasonicHw_Setup() reports
 * the right fault bit WITHOUT touching PC6, the timer registers, the CH3
 * compare channel or the CH2 capture channel, and without starting HAL
 * capture. It also checks that, on a good configuration, the CH3 frozen
 * compare setup leaves CH3 output disabled and TIM8_CH2 exactly as
 * CubeMX configured it.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -Itests/ultrasonic/fakes_hw -ICore/Inc \
 *       -o tests/ultrasonic/app_ultrasonic_hw_test.exe \
 *       tests/ultrasonic/app_ultrasonic_hw_test.c Core/Src/app_ultrasonic_hw.c \
 *       tests/ultrasonic/fakes_hw/fake_hal.c
 *   ./tests/ultrasonic/app_ultrasonic_hw_test.exe
 */

#include "app_ultrasonic.h"
#include "app_ultrasonic_hw.h"
#include "fake_hal.h"

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

#define CHECK_HEX(actual, expected) do { \
    unsigned long a_ = (unsigned long)(actual); \
    unsigned long e_ = (unsigned long)(expected); \
    checks++; \
    if (a_ != e_) { \
        failures++; \
        printf("FAIL line %d: %s == 0x%lX, expected 0x%lX\n", \
               __LINE__, #actual, a_, e_); \
    } \
} while (0)

#define GOLDEN_CCMR1 (TIM_CCMR1_CC2S_0 | (4U << TIM_CCMR1_IC2F_Pos))

/* Registers, SR and PC6 must be exactly as they were: nothing was touched. */
static void ExpectUntouched(const TIM_TypeDef *before, uint32_t srBefore)
{
    const TIM_TypeDef *now = FakeHal_Regs();

    CHECK_HEX(now->CR2, before->CR2);
    CHECK_HEX(now->DIER, before->DIER);
    CHECK_HEX(now->CCMR1, before->CCMR1);
    CHECK_HEX(now->CCMR2, before->CCMR2);
    CHECK_HEX(now->CCER, before->CCER);
    CHECK_HEX(now->CCR2, before->CCR2);
    CHECK_HEX(now->CCR3, before->CCR3);
    CHECK_HEX(now->PSC, before->PSC);
    CHECK_HEX(now->ARR, before->ARR);
    CHECK_HEX(now->CNT, before->CNT);
    CHECK_HEX(FakeHal_Sr(), srBefore);
    CHECK_HEX(fake_gpioc.BSRR, 0U);             /* PC6 never driven */
    CHECK(FakeHal_OcConfigCalls() == 0U);       /* CH3 not configured */
    CHECK(FakeHal_IcStartCalls() == 0U);        /* CH2 capture not started */
    CHECK(!FakeHal_TimerEnabled());
}

/* Run Setup() on whatever configuration is loaded and require a fail-closed
 * result with exactly `expectedFault`. */
static void ExpectFailClosed(uint16_t expectedFault, int line)
{
    TIM_TypeDef before = *FakeHal_Regs();
    uint32_t srBefore = FakeHal_Sr();
    uint16_t fault = AppUltrasonicHw_Setup();

    checks++;
    if (fault != expectedFault)
    {
        failures++;
        printf("FAIL line %d: Setup() == 0x%X, expected 0x%X\n",
               line, (unsigned)fault, (unsigned)expectedFault);
    }

    ExpectUntouched(&before, srBefore);
}

#define FAIL_CLOSED(fault) ExpectFailClosed((fault), __LINE__)

static void test_good_configuration(void)
{
    TIM_TypeDef before;
    const TIM_TypeDef *regs;

    FakeHal_Reset();
    before = *FakeHal_Regs();
    CHECK_HEX(AppUltrasonicHw_Setup(), 0U);

    regs = FakeHal_Regs();

    /* CH3 is a frozen-mode compare with its output disabled. */
    CHECK(FakeHal_OcConfigCalls() == 1U);
    CHECK(FakeHal_OcConfigLastChannel() == TIM_CHANNEL_3);
    CHECK_HEX(regs->CCER & TIM_CCER_CC3E, 0U);
    CHECK_HEX(regs->CCMR2 & TIM_CCMR2_CC3S, 0U);
    CHECK_HEX(regs->CCMR2 & TIM_CCMR2_OC3M, 0U);
    CHECK_HEX(regs->CCMR2 & TIM_CCMR2_OC3PE, 0U);

    /* TIM8_CH2 untouched except the CC2E enable done by HAL_TIM_IC_Start. */
    CHECK_HEX(regs->CCMR1, GOLDEN_CCMR1);
    CHECK_HEX(regs->CCMR1, before.CCMR1);
    CHECK_HEX(regs->CCR2, before.CCR2);
    CHECK_HEX(regs->CCER & ~TIM_CCER_CC2E, before.CCER);
    CHECK_HEX(regs->CCER & TIM_CCER_CC2E, TIM_CCER_CC2E);
    CHECK(FakeHal_IcStartCalls() == 1U);
    CHECK(FakeHal_IcStartLastChannel() == TIM_CHANNEL_2);
    CHECK(FakeHal_TimerEnabled());

    /* Clock tree and counter configuration were only verified. */
    CHECK_HEX(regs->PSC, 167U);
    CHECK_HEX(regs->ARR, 0xFFFFU);

    /* No interrupts left enabled; self-test flag cleared; PC6 driven low. */
    CHECK_HEX(regs->DIER, 0U);
    CHECK_HEX(FakeHal_Sr() & (TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC2OF), 0U);
    CHECK_HEX(fake_gpioc.BSRR, ((uint32_t)US_TRIG_Pin) << 16U);

    /* The self-test saw a real CC3IF with CC3E disabled. */
    CHECK(FakeHal_Cc3ifSetEvents() >= 1U);
}

static void test_ch2_startup_checks(void)
{
    uint32_t selection;
    uint32_t prescaler;
    uint32_t filter;

    /* Input selection (CC2S): only 01 = direct TI2 is acceptable. */
    for (selection = 0U; selection < 4U; selection++)
    {
        FakeHal_Reset();
        FakeHal_Regs()->CCMR1 =
            (selection << 8U) | (4U << TIM_CCMR1_IC2F_Pos);
        if (selection == 1U)
        {
            CHECK_HEX(AppUltrasonicHw_Setup(), 0U);
        }
        else
        {
            FAIL_CLOSED(APP_US_FAULT_CH2);
        }
    }

    /* Input-capture prescaler (IC2PSC): only DIV1 (00) is acceptable. */
    for (prescaler = 0U; prescaler < 4U; prescaler++)
    {
        FakeHal_Reset();
        FakeHal_Regs()->CCMR1 =
            TIM_CCMR1_CC2S_0 | (prescaler << 10U) | (4U << TIM_CCMR1_IC2F_Pos);
        if (prescaler == 0U)
        {
            CHECK_HEX(AppUltrasonicHw_Setup(), 0U);
        }
        else
        {
            FAIL_CLOSED(APP_US_FAULT_CH2);
        }
    }

    /* Input filter (IC2F): only 4 is acceptable, all 16 values tried. */
    for (filter = 0U; filter < 16U; filter++)
    {
        FakeHal_Reset();
        FakeHal_Regs()->CCMR1 =
            TIM_CCMR1_CC2S_0 | (filter << TIM_CCMR1_IC2F_Pos);
        if (filter == 4U)
        {
            CHECK_HEX(AppUltrasonicHw_Setup(), 0U);
        }
        else
        {
            FAIL_CLOSED(APP_US_FAULT_CH2);
        }
    }

    /* Bits that belong to CH1 (low byte of CCMR1) are not CH2's business. */
    FakeHal_Reset();
    FakeHal_Regs()->CCMR1 = GOLDEN_CCMR1 | 0x00F1U;
    CHECK_HEX(AppUltrasonicHw_Setup(), 0U);
    CHECK_HEX(FakeHal_Regs()->CCMR1, GOLDEN_CCMR1 | 0x00F1U);
}

static void test_clock_nvic_checks(void)
{
    FakeHal_Reset();
    FakeHal_Regs()->PSC = 83U;                      /* 2 MHz */
    FAIL_CLOSED(APP_US_FAULT_CLOCK);

    FakeHal_Reset();
    FakeHal_Regs()->PSC = 168U;
    FAIL_CLOSED(APP_US_FAULT_CLOCK);

    FakeHal_Reset();
    FakeHal_SetPclk2(42000000U);                    /* clock tree changed */
    FAIL_CLOSED(APP_US_FAULT_CLOCK);

    /* APB2 prescaler 1: the timer clock equals PCLK2, not twice it. */
    FakeHal_Reset();
    fake_rcc.CFGR = 0U;
    FakeHal_SetPclk2(168000000U);
    CHECK_HEX(AppUltrasonicHw_Setup(), 0U);

    FakeHal_Reset();
    fake_rcc.CFGR = 0U;                             /* DIV1 but 84 MHz */
    FAIL_CLOSED(APP_US_FAULT_CLOCK);

    FakeHal_Reset();
    FakeHal_Regs()->ARR = 9999U;
    FAIL_CLOSED(APP_US_FAULT_ARR);

    /* NVIC priority 4 is above the RTOS ceiling: critical sections would
     * not mask it. 5 and below-priority (numerically larger) are fine. */
    FakeHal_Reset();
    FakeHal_SetNvic(4U, true);
    FAIL_CLOSED(APP_US_FAULT_NVIC_PRIO);

    FakeHal_Reset();
    FakeHal_SetNvic(0U, true);
    FAIL_CLOSED(APP_US_FAULT_NVIC_PRIO);

    FakeHal_Reset();
    FakeHal_SetNvic(6U, true);
    CHECK_HEX(AppUltrasonicHw_Setup(), 0U);

    FakeHal_Reset();
    FakeHal_SetNvic(5U, false);
    FAIL_CLOSED(APP_US_FAULT_NVIC_OFF);

    /* Several wrong assumptions are all reported, and still fail closed. */
    FakeHal_Reset();
    FakeHal_Regs()->PSC = 99U;
    FakeHal_Regs()->ARR = 100U;
    FakeHal_Regs()->CCMR1 = 0U;
    FakeHal_SetNvic(2U, false);
    FAIL_CLOSED(APP_US_FAULT_CLOCK | APP_US_FAULT_ARR | APP_US_FAULT_NVIC_PRIO |
                APP_US_FAULT_NVIC_OFF | APP_US_FAULT_CH2);
}

static void test_ch3_safety_checks(void)
{
    TIM_TypeDef before;
    uint32_t srBefore;

    /* PC8 (TIM8_CH3's pin) in alternate-function mode: refuse. The HAL OC
     * configuration has already run by then, but CH3 output stays off and
     * the CH2 capture channel is never started. */
    FakeHal_Reset();
    fake_gpioc.MODER = 0x2U << 16U;
    CHECK_HEX(AppUltrasonicHw_Setup(), APP_US_FAULT_CH3);
    CHECK(FakeHal_IcStartCalls() == 0U);
    CHECK(!FakeHal_TimerEnabled());
    CHECK_HEX(FakeHal_Regs()->CCER & TIM_CCER_CC3E, 0U);
    CHECK_HEX(FakeHal_Regs()->DIER, 0U);

    /* PC8 as a plain output/input is fine. */
    FakeHal_Reset();
    fake_gpioc.MODER = 0x1U << 16U;
    CHECK_HEX(AppUltrasonicHw_Setup(), 0U);

    /* The HAL reports an error. */
    FakeHal_Reset();
    FakeHal_OcConfigFails(true);
    CHECK_HEX(AppUltrasonicHw_Setup(), APP_US_FAULT_CH3);
    CHECK(FakeHal_IcStartCalls() == 0U);

    /* CC3E already set before Setup(): the real HAL RESTORES it (it writes
     * back the CCER value it read before clearing the bit), so the output
     * would stay enabled. Setup() must catch that and refuse. */
    FakeHal_Reset();
    FakeHal_Regs()->CCER = TIM_CCER_CC3E;
    CHECK_HEX(AppUltrasonicHw_Setup(), APP_US_FAULT_CH3);
    CHECK(FakeHal_IcStartCalls() == 0U);
    CHECK(!FakeHal_TimerEnabled());

    /* OC3PE is not cleared by the HAL: preload would delay CCR3 updates. */
    FakeHal_Reset();
    FakeHal_Regs()->CCMR2 = TIM_CCMR2_OC3PE;
    CHECK_HEX(AppUltrasonicHw_Setup(), APP_US_FAULT_CH3);
    CHECK(FakeHal_IcStartCalls() == 0U);

    /* Stale OC3M / CC3S bits ARE cleared/rewritten by the HAL, so a dirty
     * CCMR2 from before is normalised to frozen compare. */
    FakeHal_Reset();
    FakeHal_Regs()->CCMR2 = TIM_CCMR2_OC3M | TIM_CCMR2_CC3S;
    CHECK_HEX(AppUltrasonicHw_Setup(), 0U);
    CHECK_HEX(FakeHal_Regs()->CCMR2 & (TIM_CCMR2_OC3M | TIM_CCMR2_CC3S), 0U);

    /* The HAL call must not alter CH2 even with other CCER bits set. */
    FakeHal_Reset();
    FakeHal_Regs()->CCER = TIM_CCER_CC2P;           /* falling polarity */
    before = *FakeHal_Regs();
    srBefore = FakeHal_Sr();
    (void)srBefore;
    CHECK_HEX(AppUltrasonicHw_Setup(), 0U);
    CHECK_HEX(FakeHal_Regs()->CCMR1, before.CCMR1);
    CHECK_HEX(FakeHal_Regs()->CCR2, before.CCR2);
}

static void test_start_and_selftest(void)
{
    /* HAL_TIM_IC_Start fails: reported, no self-test side effects. */
    FakeHal_Reset();
    FakeHal_IcStartFails(true);
    CHECK_HEX(AppUltrasonicHw_Setup(), APP_US_FAULT_START);
    CHECK_HEX(FakeHal_Regs()->DIER, 0U);

    /* Silicon that did NOT set CC3IF with CC3E disabled would make the
     * pin-less compare useless: the self-test must catch it, bounded. */
    FakeHal_Reset();
    FakeHal_Cc3ifNeedsCc3e(true);
    CHECK_HEX(AppUltrasonicHw_Setup(), APP_US_FAULT_SELFTEST);
    CHECK_HEX(FakeHal_Regs()->DIER, 0U);
    CHECK_HEX(FakeHal_Regs()->CCER & TIM_CCER_CC3E, 0U);
    CHECK_HEX(FakeHal_Sr() & TIM_SR_CC3IF, 0U);

    /* A counter that never runs: the poll is iteration-bounded, not an
     * endless wait. */
    FakeHal_Reset();
    FakeHal_IcStartDoesNotRunCounter(true);
    CHECK_HEX(AppUltrasonicHw_Setup(), APP_US_FAULT_SELFTEST);
    CHECK_HEX(FakeHal_Regs()->DIER, 0U);
    /* 5000 polls x at most 3 register accesses, plus the fixed setup. */
    CHECK(FakeHal_AccessCount() < 16000UL);
}

static void test_selftest_arming_is_atomic(void)
{
    /* Pre-emption right after CCR3 is armed: the compare condition is true
     * at the very next register access. The old order (arm CCR3, THEN
     * clear CC3IF) erases that successful flag and falsely reports
     * APP_US_FAULT_SELFTEST; the correct order (clear, read counter, arm)
     * leaves it set. */
    FakeHal_Reset();
    FakeHal_MatchRightAfterArm(true);
    CHECK_HEX(AppUltrasonicHw_Setup(), 0U);
    CHECK(FakeHal_Cc3ifSetEvents() >= 1U);
    CHECK_HEX(FakeHal_Sr() & TIM_SR_CC3IF, 0U);     /* cleaned up afterwards */

    /* The arming critical section: exactly one, balanced, covering only the
     * three register operations (clear flag, read CNT, write CCR3) and
     * never the polling loop. */
    CHECK(FakeHal_CsEnterCount() == 1U);
    CHECK(FakeHal_CsExitCount() == 1U);
    CHECK(FakeHal_CsMaxDepth() == 1U);
    CHECK(FakeHal_AccessesInsideCs() == 3UL);

    /* CC3IE stays disabled for the whole self-test. */
    CHECK(!FakeHal_Cc3ieEverSet());

    /* Same race, normal timing (match ~20 us later) still passes, and the
     * critical-section shape is identical. */
    FakeHal_Reset();
    CHECK_HEX(AppUltrasonicHw_Setup(), 0U);
    CHECK(FakeHal_CsEnterCount() == 1U);
    CHECK(FakeHal_CsExitCount() == 1U);
    CHECK(FakeHal_AccessesInsideCs() == 3UL);
    CHECK(!FakeHal_Cc3ieEverSet());

    /* A stale CC3IF from before the test must not count as success: the
     * old flag is cleared first. (Counter not running, so no real match.) */
    FakeHal_Reset();
    FakeHal_IcStartDoesNotRunCounter(true);
    FakeHal_SetSrFlags(TIM_SR_CC3IF);
    CHECK_HEX(AppUltrasonicHw_Setup(), APP_US_FAULT_SELFTEST);

    /* Fail-closed paths never reach the self-test, so they never enter a
     * critical section. */
    FakeHal_Reset();
    FakeHal_Regs()->ARR = 100U;
    CHECK_HEX(AppUltrasonicHw_Setup(), APP_US_FAULT_ARR);
    CHECK(FakeHal_CsEnterCount() == 0U);
}

static void test_register_operations(void)
{
    TIM_TypeDef *regs;

    FakeHal_Reset();
    regs = FakeHal_Regs();
    regs->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E;     /* unrelated enables */

    /* Polarity touches CC2P/CC2NP only. */
    AppUltrasonicHw_SetCapturePolarity(true);
    CHECK_HEX(regs->CCER, TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC2P);
    AppUltrasonicHw_SetCapturePolarity(false);
    CHECK_HEX(regs->CCER, TIM_CCER_CC2E | TIM_CCER_CC3E);

    /* Interrupt enables are independent single bits. */
    AppUltrasonicHw_EnableCompareIrq(true);
    CHECK_HEX(regs->DIER, TIM_DIER_CC3IE);
    AppUltrasonicHw_EnableCaptureIrq(true);
    CHECK_HEX(regs->DIER, TIM_DIER_CC3IE | TIM_DIER_CC2IE);
    AppUltrasonicHw_EnableCompareIrq(false);
    CHECK_HEX(regs->DIER, TIM_DIER_CC2IE);
    AppUltrasonicHw_EnableCaptureIrq(false);
    CHECK_HEX(regs->DIER, 0U);

    /* Flag clears are rc_w0: only the named flags go away. */
    FakeHal_SetSrFlags(TIM_SR_CC2IF | TIM_SR_CC2OF | TIM_SR_CC3IF | 0x1U);
    AppUltrasonicHw_ClearCaptureFlags();
    CHECK_HEX(FakeHal_Sr(), TIM_SR_CC3IF | 0x1U);
    FakeHal_SetSrFlags(TIM_SR_CC2IF);
    AppUltrasonicHw_ClearCompareFlag();
    CHECK_HEX(FakeHal_Sr(), TIM_SR_CC2IF | 0x1U);

    /* Overrun is reported once and cleared without disturbing CC2IF. */
    FakeHal_SetSrFlags(TIM_SR_CC2OF);
    CHECK(AppUltrasonicHw_TakeCaptureOverrun());
    CHECK(!AppUltrasonicHw_TakeCaptureOverrun());
    CHECK_HEX(FakeHal_Sr() & TIM_SR_CC2IF, TIM_SR_CC2IF);

    /* Trigger pin: atomic set/reset via BSRR; Echo read from IDR bit 7. */
    AppUltrasonicHw_TriggerHigh();
    CHECK_HEX(fake_gpioc.BSRR, (uint32_t)US_TRIG_Pin);
    AppUltrasonicHw_TriggerLow();
    CHECK_HEX(fake_gpioc.BSRR, ((uint32_t)US_TRIG_Pin) << 16U);

    fake_gpioc.IDR = 0U;
    CHECK(!AppUltrasonicHw_EchoHigh());
    fake_gpioc.IDR = GPIO_PIN_7;
    CHECK(AppUltrasonicHw_EchoHigh());
    fake_gpioc.IDR = (uint32_t)~GPIO_PIN_7;
    CHECK(!AppUltrasonicHw_EchoHigh());

    regs->CCR2 = 0xABCDU;
    CHECK_HEX(AppUltrasonicHw_ReadCapture(), 0xABCDU);
    AppUltrasonicHw_SetCompare(0x1234U);
    CHECK_HEX(regs->CCR3, 0x1234U);
}

int main(void)
{
    test_good_configuration();
    test_ch2_startup_checks();
    test_clock_nvic_checks();
    test_ch3_safety_checks();
    test_start_and_selftest();
    test_selftest_arming_is_atomic();
    test_register_operations();

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
