/* Host-test fake of main.h / the HAL for the REAL Core/Src/app_ultrasonic_hw.c.
 *
 * Provides just the register blocks, bit masks and HAL entry points the port
 * touches. Bit-mask values were taken from the real STM32F407 device header
 * (extracted with the ARM preprocessor) so they cannot drift:
 *   TIM_CCMR1_CC2S 0x300  CC2S_0 0x100  IC2PSC 0xC00  IC2F 0xF000 (pos 12)
 *   TIM_CCMR2_CC3S 0x3  OC3M 0x70  OC3PE 0x8
 *   TIM_CCER_CC2E 0x10  CC3E 0x100  CC2P 0x20  CC2NP 0x80
 *   TIM_DIER_CC2IE 0x4  CC3IE 0x8   TIM_SR_CC2IF 0x4  CC3IF 0x8  CC2OF 0x400
 *   RCC_CFGR_PPRE2 0xE000   GPIO_PIN_6 0x40  GPIO_PIN_7 0x80
 *
 * TIM8 is a FUNCTION-LIKE macro: every register access (TIM8->X) calls
 * FakeTim8(), which advances a tiny timer model one step, so polling loops
 * make progress and SR behaves like rc_w0 flags (see fake_hal.c). */
#ifndef FAKE_HW_MAIN_H
#define FAKE_HW_MAIN_H

#include <stdbool.h>
#include <stdint.h>

typedef struct { volatile uint32_t CR2, SMCR, DIER, SR, CCMR1, CCMR2, CCER,
                 CNT, PSC, ARR, CCR2, CCR3; } TIM_TypeDef;
typedef struct { volatile uint32_t MODER, IDR, BSRR; } GPIO_TypeDef;
typedef struct { volatile uint32_t CFGR; } RCC_TypeDef;

extern GPIO_TypeDef fake_gpioc;
extern RCC_TypeDef fake_rcc;
TIM_TypeDef *FakeTim8(void);

#define TIM8  FakeTim8()
#define GPIOC (&fake_gpioc)
#define RCC   (&fake_rcc)

#define TIM_CCMR1_CC2S      0x00000300U
#define TIM_CCMR1_CC2S_0    0x00000100U
#define TIM_CCMR1_IC2PSC    0x00000C00U
#define TIM_CCMR1_IC2F      0x0000F000U
#define TIM_CCMR1_IC2F_Pos  12U
#define TIM_CCMR2_CC3S      0x00000003U
#define TIM_CCMR2_OC3M      0x00000070U
#define TIM_CCMR2_OC3PE     0x00000008U
#define TIM_CCER_CC2E       0x00000010U
#define TIM_CCER_CC3E       0x00000100U
#define TIM_CCER_CC2P       0x00000020U
#define TIM_CCER_CC2NP      0x00000080U
#define TIM_CCER_CC3P       0x00000200U
#define TIM_DIER_CC2IE      0x00000004U
#define TIM_DIER_CC3IE      0x00000008U
#define TIM_SR_CC2IF        0x00000004U
#define TIM_SR_CC3IF        0x00000008U
#define TIM_SR_CC2OF        0x00000400U
#define RCC_CFGR_PPRE2      0x0000E000U
#define RCC_CFGR_PPRE2_DIV1 0x00000000U
#define GPIO_PIN_6          0x0040U
#define GPIO_PIN_7          0x0080U
#define US_TRIG_Pin         GPIO_PIN_6
#define US_TRIG_GPIO_Port   GPIOC

#define SET_BIT(REG, BIT)   ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))

#define TIM_CHANNEL_2   0x00000004U
#define TIM_CHANNEL_3   0x00000008U
#define TIM_OCMODE_TIMING        0x00000000U
#define TIM_OCPOLARITY_HIGH      0x00000000U
#define TIM_OCNPOLARITY_HIGH     0x00000000U
#define TIM_OCFAST_DISABLE       0x00000000U
#define TIM_OCIDLESTATE_RESET    0x00000000U
#define TIM_OCNIDLESTATE_RESET   0x00000000U

typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef enum { TIM8_CC_IRQn = 46 } IRQn_Type;

typedef struct
{
    uint32_t OCMode, Pulse, OCPolarity, OCNPolarity, OCFastMode,
             OCIdleState, OCNIdleState;
} TIM_OC_InitTypeDef;

typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;

uint32_t HAL_RCC_GetPCLK2Freq(void);
uint32_t NVIC_GetPriority(IRQn_Type irq);
uint32_t NVIC_GetEnableIRQ(IRQn_Type irq);
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim,
    const TIM_OC_InitTypeDef *cfg, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t channel);

#endif
