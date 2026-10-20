/*
 * app_ultrasonic_hw.c
 *
 * TIM8 / GPIO port for the ultrasonic driver. Each function is a single
 * register operation so it is safe and bounded inside TIM8_CC_IRQHandler.
 * See app_ultrasonic_hw.h for the ownership / masking rules.
 *
 * Nothing in this file changes a pin, clock, prescaler, ARR or NVIC
 * priority: AppUltrasonicHw_Setup() only VERIFIES them and refuses to run
 * (returns fault bits) if they differ from what the driver assumes.
 */

#include "app_ultrasonic_hw.h"
#include "app_ultrasonic.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

extern TIM_HandleTypeDef htim8;

/* PC7 is TIM8_CH2 (AF3). IDR still reflects the pin level in AF mode. The
 * CubeMX project defines no label for it, and main.h is generated, so the
 * port/pin are named here instead. */
#define US_ECHO_GPIO_Port   GPIOC
#define US_ECHO_Pin         GPIO_PIN_7

/* PC8 is TIM8_CH3's pin. It must stay a non-alternate pin so the internal
 * compare can never drive anything. */
#define US_CH3_GPIO_Port    GPIOC
#define US_CH3_MODER_SHIFT  (8U * 2U)

static uint32_t Tim8ClockHz(void)
{
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

    /* Timer kernel clock is 2 x PCLK2 whenever the APB2 prescaler is not 1. */
    if ((RCC->CFGR & RCC_CFGR_PPRE2) == RCC_CFGR_PPRE2_DIV1)
    {
        return pclk2;
    }

    return 2U * pclk2;
}

/* One-time start-up proof that CC3IF sets on a CNT == CCR3 match while CC3E
 * is disabled (the whole point of the pin-less compare). The HAL source only
 * shows that nothing here enables the output and that HAL_TIM_IRQHandler()
 * gates on SR/DIER, never CCER; that the silicon sets the flag with CC3E=0
 * is reference-manual behaviour the HAL cannot demonstrate, so it is
 * verified rather than assumed.
 *
 * Bounded twice: by 200 us of timer time and by a hard iteration cap (in
 * case the counter is not running). The flag is tested BEFORE the time
 * limit each pass and it is sticky, so a pre-emption during the test cannot
 * cause a false failure. Init-only; interrupts for CC2/CC3 are still off. */
#define US_SELFTEST_AHEAD_TICKS     20U
#define US_SELFTEST_LIMIT_TICKS     200U
#define US_SELFTEST_MAX_POLLS       5000U

static bool CompareFlagSelfTest(void)
{
    uint16_t start;
    uint32_t polls;
    bool seen = false;

    /* Arming must be atomic and ordered "clear old flag, read counter, set
     * CCR3": this task (BelowNormal) can be pre-empted at any point, and if
     * CCR3 were armed first and the flag cleared second, a match during the
     * pre-emption would be erased and a healthy timer reported as failed.
     * The critical section covers ONLY these three register operations,
     * never the polling below. CC3IE stays disabled throughout. */
    taskENTER_CRITICAL();
    TIM8->SR = ~(uint32_t)TIM_SR_CC3IF;
    start = (uint16_t)TIM8->CNT;
    TIM8->CCR3 = (uint16_t)(start + US_SELFTEST_AHEAD_TICKS);
    taskEXIT_CRITICAL();

    for (polls = 0U; polls < US_SELFTEST_MAX_POLLS; polls++)
    {
        if ((TIM8->SR & TIM_SR_CC3IF) != 0U)
        {
            seen = true;
            break;
        }

        if ((uint16_t)((uint16_t)TIM8->CNT - start) > US_SELFTEST_LIMIT_TICKS)
        {
            break;
        }
    }

    TIM8->SR = ~(uint32_t)TIM_SR_CC3IF;
    return seen;
}

/* TIM8_CH2 as configured by CubeMX (RTOSV.ioc / MX_TIM8_Init): direct TI2,
 * input-capture prescaler DIV1, input filter 4. Verified, never changed. */
#define US_CH2_EXPECTED_FILTER      4U

static bool Ch2ConfigMatches(void)
{
    uint32_t ccmr1 = TIM8->CCMR1;

    return ((ccmr1 & TIM_CCMR1_CC2S) == TIM_CCMR1_CC2S_0) &&        /* direct TI2 */
           ((ccmr1 & TIM_CCMR1_IC2PSC) == 0U) &&                    /* DIV1       */
           (((ccmr1 & TIM_CCMR1_IC2F) >> TIM_CCMR1_IC2F_Pos) ==
            US_CH2_EXPECTED_FILTER);
}

uint16_t AppUltrasonicHw_Setup(void)
{
    uint16_t fault = 0U;
    uint32_t divider = (uint32_t)TIM8->PSC + 1U;
    uint32_t clock = Tim8ClockHz();
    TIM_OC_InitTypeDef oc = {0};

    /* --- verify, never change ---------------------------------------- */
    if (((clock % divider) != 0U) || ((clock / divider) != 1000000U))
    {
        fault |= APP_US_FAULT_CLOCK;
    }

    if (TIM8->ARR != 0xFFFFU)
    {
        fault |= APP_US_FAULT_ARR;
    }

    /* NVIC_PRIORITYGROUP_4: the field is the preemption priority. The
     * driver's atomicity relies on critical sections masking this IRQ. */
    if (NVIC_GetPriority(TIM8_CC_IRQn) <
        (uint32_t)configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
    {
        fault |= APP_US_FAULT_NVIC_PRIO;
    }

    if (NVIC_GetEnableIRQ(TIM8_CC_IRQn) == 0U)
    {
        fault |= APP_US_FAULT_NVIC_OFF;
    }

    if (!Ch2ConfigMatches())
    {
        fault |= APP_US_FAULT_CH2;
    }

    if (fault != 0U)
    {
        return fault;       /* refuse to run: nothing touched */
    }

    /* --- quiet state -------------------------------------------------- */
    CLEAR_BIT(TIM8->DIER, TIM_DIER_CC2IE | TIM_DIER_CC3IE);
    AppUltrasonicHw_TriggerLow();

    /* --- TIM8_CH3: frozen-mode output compare, no preload, no output --- */
    oc.OCMode = TIM_OCMODE_TIMING;
    oc.Pulse = 0U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_OC_ConfigChannel(&htim8, &oc, TIM_CHANNEL_3) != HAL_OK)
    {
        fault |= APP_US_FAULT_CH3;
    }

    /* The event must be a pure internal flag: output disabled, compare (not
     * capture) channel, frozen mode, immediate CCR3 updates, and PC8 not in
     * alternate-function mode. */
    if (((TIM8->CCER & TIM_CCER_CC3E) != 0U) ||
        ((TIM8->CCMR2 & TIM_CCMR2_CC3S) != 0U) ||
        ((TIM8->CCMR2 & TIM_CCMR2_OC3M) != 0U) ||
        ((TIM8->CCMR2 & TIM_CCMR2_OC3PE) != 0U) ||
        (((US_CH3_GPIO_Port->MODER >> US_CH3_MODER_SHIFT) & 0x3U) == 0x2U))
    {
        fault |= APP_US_FAULT_CH3;
    }

    if (fault != 0U)
    {
        return fault;
    }

    /* --- CH2 capture on, interrupts still off ------------------------- */
    if (HAL_TIM_IC_Start(&htim8, TIM_CHANNEL_2) != HAL_OK)
    {
        return APP_US_FAULT_START;
    }

    CLEAR_BIT(TIM8->DIER, TIM_DIER_CC2IE | TIM_DIER_CC3IE);

    if (!CompareFlagSelfTest())
    {
        return APP_US_FAULT_SELFTEST;
    }

    AppUltrasonicHw_SetCapturePolarity(false);
    AppUltrasonicHw_ClearCaptureFlags();
    AppUltrasonicHw_ClearCompareFlag();

    return 0U;
}

uint16_t AppUltrasonicHw_Counter(void)
{
    return (uint16_t)TIM8->CNT;
}

void AppUltrasonicHw_TriggerHigh(void)
{
    US_TRIG_GPIO_Port->BSRR = (uint32_t)US_TRIG_Pin;
}

void AppUltrasonicHw_TriggerLow(void)
{
    US_TRIG_GPIO_Port->BSRR = ((uint32_t)US_TRIG_Pin) << 16U;
}

bool AppUltrasonicHw_EchoHigh(void)
{
    return (US_ECHO_GPIO_Port->IDR & US_ECHO_Pin) != 0U;
}

void AppUltrasonicHw_SetCompare(uint16_t ticks)
{
    TIM8->CCR3 = ticks;
}

void AppUltrasonicHw_EnableCompareIrq(bool enable)
{
    if (enable)
    {
        SET_BIT(TIM8->DIER, TIM_DIER_CC3IE);
    }
    else
    {
        CLEAR_BIT(TIM8->DIER, TIM_DIER_CC3IE);
    }
}

void AppUltrasonicHw_ClearCompareFlag(void)
{
    /* SR bits are rc_w0: writing 0 clears, writing 1 leaves unchanged. */
    TIM8->SR = ~(uint32_t)TIM_SR_CC3IF;
}

void AppUltrasonicHw_EnableCaptureIrq(bool enable)
{
    if (enable)
    {
        SET_BIT(TIM8->DIER, TIM_DIER_CC2IE);
    }
    else
    {
        CLEAR_BIT(TIM8->DIER, TIM_DIER_CC2IE);
    }
}

void AppUltrasonicHw_ClearCaptureFlags(void)
{
    TIM8->SR = ~((uint32_t)(TIM_SR_CC2IF | TIM_SR_CC2OF));
}

void AppUltrasonicHw_SetCapturePolarity(bool falling)
{
    /* One read-modify-write store. CC2P=0,CC2NP=0 rising; CC2P=1 falling. */
    uint32_t ccer = TIM8->CCER & ~(TIM_CCER_CC2P | TIM_CCER_CC2NP);

    if (falling)
    {
        ccer |= TIM_CCER_CC2P;
    }

    TIM8->CCER = ccer;
}

uint16_t AppUltrasonicHw_ReadCapture(void)
{
    return (uint16_t)TIM8->CCR2;
}

bool AppUltrasonicHw_TakeCaptureOverrun(void)
{
    if ((TIM8->SR & TIM_SR_CC2OF) != 0U)
    {
        TIM8->SR = ~(uint32_t)TIM_SR_CC2OF;
        return true;
    }

    return false;
}
