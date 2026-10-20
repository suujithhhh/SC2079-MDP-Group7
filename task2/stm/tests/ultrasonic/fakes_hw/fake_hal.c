/* Fake TIM8 register block + HAL entry points for the host test of the REAL
 * Core/Src/app_ultrasonic_hw.c. See main.h in this directory.
 *
 * FakeTim8() is invoked on every TIM8->register access. It (1) interprets a
 * changed SR as an rc_w0 write (new = hw & written), (2) advances the 1 MHz
 * counter one tick if the timer is running, setting CC3IF when CNT == CCR3,
 * and (3) publishes the hardware SR back into the register.
 *
 * The HAL replicas mirror the real STM32F4 HAL v1.8 code that was read for
 * this project (Drivers/.../stm32f4xx_hal_tim.c):
 *   TIM_OC3_SetConfig: tmpccer = CCER; CCER &= ~CC3E; tmpccmr = CCMR2 with
 *     OC3M and CC3S cleared, OCMode ORed in; tmpccer CC3P replaced; CR2
 *     rewritten; CCMR2 = tmpccmr; CCR3 = Pulse; CCER = tmpccer  (so a
 *     previously set CC3E is RESTORED, OC3PE is left alone, CH2 untouched).
 *   HAL_TIM_IC_Start: channel-state check, CCxE set via TIM_CCxChannelCmd,
 *     then __HAL_TIM_ENABLE (no MOE for input capture). */

#include "fake_hal.h"

#include <string.h>

GPIO_TypeDef fake_gpioc;
RCC_TypeDef fake_rcc;

static TIM_TypeDef regs;
static uint32_t srHw;
static uint32_t lastSr;
static bool running;

static uint32_t pclk2;
static uint32_t nvicPriority;
static bool nvicEnabled;
static bool ocFails;
static bool icFails;
static bool cc3ifNeedsCc3e;
static bool icStartNoCounter;
static unsigned ocCalls;
static uint32_t ocChannel;
static unsigned icCalls;
static uint32_t icChannel;
static unsigned cc3ifEvents;
static unsigned long accessCount;
static bool matchOnArm;
static uint32_t lastCcr3;
static unsigned csEnter, csExit, csDepth, csMaxDepth;
static unsigned long insideCs;
static bool cc3ieSeen;

void FakeHal_Reset(void)
{
    memset(&regs, 0, sizeof(regs));
    memset(&fake_gpioc, 0, sizeof(fake_gpioc));
    memset(&fake_rcc, 0, sizeof(fake_rcc));
    srHw = 0U;
    lastSr = 0U;
    running = false;
    ocFails = false;
    icFails = false;
    cc3ifNeedsCc3e = false;
    icStartNoCounter = false;
    ocCalls = 0U;
    icCalls = 0U;
    cc3ifEvents = 0U;
    accessCount = 0U;
    matchOnArm = false;
    lastCcr3 = 0U;
    csEnter = csExit = csDepth = csMaxDepth = 0U;
    insideCs = 0U;
    cc3ieSeen = false;

    regs.PSC = 167U;
    regs.ARR = 0xFFFFU;
    regs.CNT = 1000U;
    regs.CCMR1 = TIM_CCMR1_CC2S_0 | (4U << TIM_CCMR1_IC2F_Pos);
    pclk2 = 84000000U;
    fake_rcc.CFGR = 0x00008000U;            /* APB2 divider 2 */
    nvicPriority = 5U;
    nvicEnabled = true;
}

void FakeHal_SetPclk2(uint32_t hz) { pclk2 = hz; }
void FakeHal_SetNvic(uint32_t priority, bool enabled)
{
    nvicPriority = priority;
    nvicEnabled = enabled;
}
void FakeHal_OcConfigFails(bool fails) { ocFails = fails; }
void FakeHal_IcStartFails(bool fails) { icFails = fails; }
void FakeHal_Cc3ifNeedsCc3e(bool needs) { cc3ifNeedsCc3e = needs; }
void FakeHal_IcStartDoesNotRunCounter(bool v) { icStartNoCounter = v; }

unsigned FakeHal_OcConfigCalls(void) { return ocCalls; }
uint32_t FakeHal_OcConfigLastChannel(void) { return ocChannel; }
unsigned FakeHal_IcStartCalls(void) { return icCalls; }
uint32_t FakeHal_IcStartLastChannel(void) { return icChannel; }
bool FakeHal_TimerEnabled(void) { return running; }
unsigned FakeHal_Cc3ifSetEvents(void) { return cc3ifEvents; }
unsigned long FakeHal_AccessCount(void) { return accessCount; }
void FakeHal_MatchRightAfterArm(bool enable) { matchOnArm = enable; }
unsigned FakeHal_CsEnterCount(void) { return csEnter; }
unsigned FakeHal_CsExitCount(void) { return csExit; }
unsigned FakeHal_CsMaxDepth(void) { return csMaxDepth; }
unsigned long FakeHal_AccessesInsideCs(void) { return insideCs; }
bool FakeHal_Cc3ieEverSet(void) { return cc3ieSeen; }

void FakeCs_Enter(void)
{
    csEnter++;
    csDepth++;
    if (csDepth > csMaxDepth) { csMaxDepth = csDepth; }
}

void FakeCs_Exit(void)
{
    csExit++;
    if (csDepth > 0U) { csDepth--; }
}

TIM_TypeDef *FakeHal_Regs(void) { return &regs; }
void FakeHal_SetSrFlags(uint32_t flags) { srHw |= flags; regs.SR = lastSr = srHw; }
uint32_t FakeHal_Sr(void)
{
    if (regs.SR != lastSr)                  /* apply a pending rc_w0 write, no tick */
    {
        srHw &= regs.SR;
        regs.SR = srHw;
        lastSr = srHw;
    }

    return srHw;
}

TIM_TypeDef *FakeTim8(void)
{
    accessCount++;

    if (csDepth > 0U)
    {
        insideCs++;
    }

    if ((regs.DIER & TIM_DIER_CC3IE) != 0U)
    {
        cc3ieSeen = true;
    }

    if (regs.SR != lastSr)
    {
        srHw &= regs.SR;                    /* rc_w0: written 0 clears */
    }

    if (regs.CCR3 != lastCcr3)
    {
        lastCcr3 = regs.CCR3;

        if (matchOnArm)
        {
            /* The task was pre-empted for longer than the compare lead: the
             * match has happened AND the counter is already past CCR3, so
             * it cannot match again until the 16-bit wrap (65 ms). A flag
             * erased after this point is gone for good. */
            srHw |= TIM_SR_CC3IF;
            cc3ifEvents++;
            regs.CNT = (regs.CCR3 + 1U) & 0xFFFFU;
        }
    }

    if (running)
    {
        regs.CNT = (regs.CNT + 1U) & 0xFFFFU;

        if ((regs.CNT == regs.CCR3) &&
            (!cc3ifNeedsCc3e || ((regs.CCER & TIM_CCER_CC3E) != 0U)))
        {
            srHw |= TIM_SR_CC3IF;
            cc3ifEvents++;
        }
    }

    regs.SR = srHw;
    lastSr = srHw;
    return &regs;
}

uint32_t HAL_RCC_GetPCLK2Freq(void) { return pclk2; }
uint32_t NVIC_GetPriority(IRQn_Type irq) { (void)irq; return nvicPriority; }
uint32_t NVIC_GetEnableIRQ(IRQn_Type irq) { (void)irq; return nvicEnabled ? 1U : 0U; }

HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim,
    const TIM_OC_InitTypeDef *cfg, uint32_t channel)
{
    uint32_t tmpccer;
    uint32_t tmpccmr;
    uint32_t tmpcr2;

    (void)htim;
    ocCalls++;
    ocChannel = channel;

    if (ocFails || (channel != TIM_CHANNEL_3))
    {
        return HAL_ERROR;
    }

    /* TIM_OC3_SetConfig(), as read from the HAL source. Raw register
     * access (no timer-model ticking) like a real function body. */
    tmpccer = regs.CCER;
    regs.CCER &= ~TIM_CCER_CC3E;
    tmpcr2 = regs.CR2;
    tmpccmr = regs.CCMR2;
    tmpccmr &= ~TIM_CCMR2_OC3M;
    tmpccmr &= ~TIM_CCMR2_CC3S;
    tmpccmr |= cfg->OCMode;
    tmpccer &= ~TIM_CCER_CC3P;
    tmpccer |= (cfg->OCPolarity << 8U);
    regs.CR2 = tmpcr2;
    regs.CCMR2 = tmpccmr;
    regs.CCR3 = cfg->Pulse;
    regs.CCER = tmpccer;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)htim;
    icCalls++;
    icChannel = channel;

    if (icFails)
    {
        return HAL_ERROR;
    }

    if (channel == TIM_CHANNEL_2)
    {
        regs.CCER |= TIM_CCER_CC2E;
    }

    running = !icStartNoCounter;
    return HAL_OK;
}

TIM_HandleTypeDef htim8 = { 0 };
