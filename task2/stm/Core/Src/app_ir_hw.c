/*
 * app_ir_hw.c
 *
 * ADC1 / DMA2 Stream0 port for the IR acquisition core. See app_ir_hw.h for
 * the ownership rules: IRTask is the only caller, nothing here runs in an
 * ISR, and nothing here changes the CubeMX configuration -- the start-up
 * check only VERIFIES it and makes the core fail closed on a mismatch.
 *
 * HAL behaviour this port relies on / works around (STM32F4 HAL, verified in
 * Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c and _dma.c):
 *   - HAL_ADC_Start_DMA() clears and re-sets CR2.DMA before every start,
 *     which DDS = 0 (DMAContinuousRequests DISABLE) requires for each new
 *     scan in DMA NORMAL mode; then starts the stream and sets SWSTART.
 *   - It returns HAL_OK even when ADON could not be set (it only flags
 *     HAL_ADC_STATE_ERROR_INTERNAL), so AppIrHw_Start() checks the state.
 *   - If the DMA handle is not READY, HAL_DMA_Start_IT() returns HAL_BUSY but
 *     HAL_ADC_Start_DMA() still sets SWSTART (-> overrun), so the core calls
 *     AppIrHw_Ready() before every start.
 *   - ADC_DMAError() OVERWRITES hadc->State with HAL_ADC_STATE_ERROR_DMA, and
 *     neither HAL_ADC_Start_DMA() nor HAL_ADC_Stop_DMA() ever clears that
 *     bit. Left alone, every later completion would be routed to
 *     HAL_ADC_ErrorCallback() and acquisition could never recover. That is
 *     the ONLY reason AppIrHw_Recover() writes hadc1.State / ErrorCode
 *     directly, after HAL_ADC_Stop_DMA() has stopped the ADC (ADON = 0) and
 *     the DMA handle is READY, i.e. when no transfer or HAL call can race it.
 *   - HAL_ADC_Stop_DMA() aborts a busy stream with HAL_DMA_Abort(), which
 *     waits (bounded, HAL_TIMEOUT_DMA_ABORT = 5 ms) for EN to clear. That
 *     wait happens only on this recovery path, in IRTask.
 */

#include "app_ir_hw.h"

#if APP_IR_ENABLE

#include "main.h"
#include "FreeRTOS.h"

#include <stdint.h>

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

/* Expected register contents (all from the CubeMX configuration). */
#define IR_ADC_CHANNEL_LEFT       10U   /* PC0, rank 1 -> DMA element 0 */
#define IR_ADC_CHANNEL_RIGHT      11U   /* PC1, rank 2 -> DMA element 1 */
#define IR_ADC_SQR_FIELD_MASK     0x1FU
#define IR_ADC_SMP_FIELD_MASK     0x7U
#define IR_ADC_SMP_84_CYCLES      0x4U  /* ADC_SAMPLETIME_84CYCLES */
#define IR_ADC_SMPR1_SMP10_POS    0U
#define IR_ADC_SMPR1_SMP11_POS    3U
#define IR_DMA_SIZE_HALFWORD      0x1U
#define IR_DMA_DIR_P2M            0x0U

static bool AdcConfigMatches(void)
{
    uint32_t cr1;
    uint32_t cr2;
    uint32_t sqr3;
    uint32_t smpr1;

    if ((hadc1.Instance != ADC1) || (hadc1.DMA_Handle != &hdma_adc1))
    {
        return false;
    }

    cr1 = ADC1->CR1;
    cr2 = ADC1->CR2;
    sqr3 = ADC1->SQR3;
    smpr1 = ADC1->SMPR1;

    return ((cr1 & ADC_CR1_RES) == 0U) &&                   /* 12-bit          */
           ((cr1 & ADC_CR1_SCAN) != 0U) &&                  /* scan            */
           ((cr1 & ADC_CR1_DISCEN) == 0U) &&
           ((cr2 & ADC_CR2_CONT) == 0U) &&                  /* single scan     */
           ((cr2 & ADC_CR2_ALIGN) == 0U) &&                 /* right aligned   */
           ((cr2 & ADC_CR2_EXTEN) == 0U) &&                 /* software start  */
           ((cr2 & ADC_CR2_DDS) == 0U) &&                   /* one DMA burst   */
           ((cr2 & ADC_CR2_EOCS) == 0U) &&                  /* EOC at sequence */
           (((ADC1->SQR1 & ADC_SQR1_L) >> ADC_SQR1_L_Pos) == 1U) &&   /* 2 ranks */
           (((sqr3 >> ADC_SQR3_SQ1_Pos) & IR_ADC_SQR_FIELD_MASK) ==
                IR_ADC_CHANNEL_LEFT) &&
           (((sqr3 >> ADC_SQR3_SQ2_Pos) & IR_ADC_SQR_FIELD_MASK) ==
                IR_ADC_CHANNEL_RIGHT) &&
           (((smpr1 >> IR_ADC_SMPR1_SMP10_POS) & IR_ADC_SMP_FIELD_MASK) ==
                IR_ADC_SMP_84_CYCLES) &&
           (((smpr1 >> IR_ADC_SMPR1_SMP11_POS) & IR_ADC_SMP_FIELD_MASK) ==
                IR_ADC_SMP_84_CYCLES);
}

static bool DmaConfigMatches(void)
{
    uint32_t cr;

    if ((hdma_adc1.Instance != DMA2_Stream0) ||
        (hdma_adc1.Parent != (void *)&hadc1))
    {
        return false;
    }

    cr = DMA2_Stream0->CR;

    return (((cr & DMA_SxCR_CHSEL) >> DMA_SxCR_CHSEL_Pos) == 0U) &&
           (((cr & DMA_SxCR_DIR) >> DMA_SxCR_DIR_Pos) == IR_DMA_DIR_P2M) &&
           ((cr & DMA_SxCR_CIRC) == 0U) &&                  /* NORMAL mode     */
           ((cr & DMA_SxCR_DBM) == 0U) &&
           ((cr & DMA_SxCR_PINC) == 0U) &&
           ((cr & DMA_SxCR_MINC) != 0U) &&
           (((cr & DMA_SxCR_PSIZE) >> DMA_SxCR_PSIZE_Pos) == IR_DMA_SIZE_HALFWORD) &&
           (((cr & DMA_SxCR_MSIZE) >> DMA_SxCR_MSIZE_Pos) == IR_DMA_SIZE_HALFWORD) &&
           ((cr & DMA_SxCR_EN) == 0U) &&                    /* idle at start   */
           ((DMA2_Stream0->FCR & DMA_SxFCR_DMDIS) == 0U);   /* direct mode     */
}

uint16_t AppIrHw_CheckConfig(void)
{
    uint16_t fault = 0U;

    if (!AdcConfigMatches())
    {
        fault |= APP_IR_FAULT_CFG_ADC;
    }

    if (!DmaConfigMatches())
    {
        fault |= APP_IR_FAULT_CFG_DMA;
    }

    /* NVIC_PRIORITYGROUP_4: the field is the preemption priority. The core's
     * atomicity relies on taskENTER_CRITICAL() masking this IRQ. */
    if ((NVIC_GetEnableIRQ(DMA2_Stream0_IRQn) == 0U) ||
        (NVIC_GetPriority(DMA2_Stream0_IRQn) <
         (uint32_t)configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY))
    {
        fault |= APP_IR_FAULT_CFG_NVIC;
    }

    return fault;
}

bool AppIrHw_Ready(void)
{
    const uint32_t adcNotReady =
        HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_ERROR_INTERNAL |
        HAL_ADC_STATE_ERROR_CONFIG | HAL_ADC_STATE_ERROR_DMA;

    return (hdma_adc1.State == HAL_DMA_STATE_READY) &&
           ((DMA2_Stream0->CR & DMA_SxCR_EN) == 0U) &&
           (hadc1.Lock == HAL_UNLOCKED) &&
           ((hadc1.State & adcNotReady) == 0U);
}

bool AppIrHw_Start(volatile uint16_t *buffer)
{
    /* The DMA writes the buffer behind the compiler's back; the core only
     * reads it after the completion was observed under a critical section. */
    if (HAL_ADC_Start_DMA(&hadc1,
                          (uint32_t *)(uintptr_t)buffer,
                          APP_IR_DMA_COUNT) != HAL_OK)
    {
        return false;
    }

    return ((hadc1.State & HAL_ADC_STATE_ERROR_INTERNAL) == 0U) &&
           ((ADC1->CR2 & ADC_CR2_ADON) != 0U);
}

uint16_t AppIrHw_Remaining(void)
{
    return (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc1);
}

bool AppIrHw_TakeOverrun(void)
{
    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR) != 0U)
    {
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
        return true;
    }

    return false;
}

bool AppIrHw_ErrorIsDma(uint32_t halAdcErrorCode)
{
    return (halAdcErrorCode & HAL_ADC_ERROR_DMA) != 0U;
}

bool AppIrHw_Recover(void)
{
    /* ADON = 0, CR2.DMA = 0, OVR interrupt off, busy stream aborted. */
    (void)HAL_ADC_Stop_DMA(&hadc1);

    if ((hdma_adc1.State != HAL_DMA_STATE_READY) ||
        ((DMA2_Stream0->CR & DMA_SxCR_EN) != 0U) ||
        ((ADC1->CR2 & ADC_CR2_ADON) != 0U) ||
        (hadc1.Lock != HAL_UNLOCKED))
    {
        return false;   /* the core latches APP_IR_FAULT_RECOVERY */
    }

    /* HAL quirk (see the file header): clear a sticky ERROR_DMA/INTERNAL
     * state that no HAL ADC call would ever clear. Safe here: the ADC is
     * off, the stream is disabled and IRTask is the only HAL caller. */
    hadc1.State = HAL_ADC_STATE_READY;
    hadc1.ErrorCode = HAL_ADC_ERROR_NONE;
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR | ADC_FLAG_EOC | ADC_FLAG_STRT);

    return true;
}

#endif /* APP_IR_ENABLE */
