/*
 * app_ir_hw.h
 *
 * ADC1 / DMA2 Stream0 port for the IR acquisition core (app_ir.c). This is
 * the ONLY module that touches hadc1 / hdma_adc1 after CubeMX initialisation,
 * and every function here is called from IRTask only (never from an ISR), so
 * the HAL handle lock is never contended.
 *
 * Nothing here changes a pin, clock, rank, sample time, DMA setting or NVIC
 * priority: AppIrHw_CheckConfig() only VERIFIES them.
 *
 * Host tests replace this module with tests/ir/fakes/ir_hw_fake.c.
 */

#ifndef INC_APP_IR_HW_H_
#define INC_APP_IR_HW_H_

#include <stdint.h>
#include <stdbool.h>

#include "app_ir.h"

#if APP_IR_ENABLE

/* Returns APP_IR_FAULT_CFG_* bits (0 = configuration as expected). */
uint16_t AppIrHw_CheckConfig(void);

/* True when a scan may be started: the DMA handle is READY and the ADC handle
 * is neither busy nor in an error state. Checked before every start because
 * HAL_ADC_Start_DMA() still issues SWSTART when the DMA refuses to start. */
bool AppIrHw_Ready(void);

/* Starts one two-channel scan into buffer[0..1]. True only if the HAL
 * accepted it AND the ADC really is enabled (HAL_ADC_Start_DMA() returns
 * HAL_OK even when ADON failed to set; it flags that in the handle state). */
bool AppIrHw_Start(volatile uint16_t *buffer);

/* DMA items still to transfer (NDTR); 0 after a complete scan. */
uint16_t AppIrHw_Remaining(void);

/* Reads and clears the ADC overrun flag. ADC_IRQn is disabled in this
 * project, so an overrun is only ever found by polling. */
bool AppIrHw_TakeOverrun(void);

/* True if a HAL ADC error code (hadc->ErrorCode) reports a DMA error. */
bool AppIrHw_ErrorIsDma(uint32_t halAdcErrorCode);

/* Stops the ADC and aborts the DMA, then returns both HAL handles to READY.
 * True when the DMA handle is READY afterwards. See app_ir_hw.c for why the
 * ADC handle state has to be reset directly. */
bool AppIrHw_Recover(void);

#endif /* APP_IR_ENABLE */

#endif /* INC_APP_IR_HW_H_ */
