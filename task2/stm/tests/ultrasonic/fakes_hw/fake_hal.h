/* Control/observation knobs for fake_hal.c (test-only). */
#ifndef FAKE_HAL_H
#define FAKE_HAL_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

/* Golden CubeMX configuration: PSC 167, ARR 65535, PCLK2 84 MHz with an APB2
 * divider of 2, TIM8_CC priority 5 and enabled, CH2 = direct TI2 / DIV1 /
 * filter 4, everything else zero, timer stopped. */
void FakeHal_Reset(void);

void FakeHal_SetPclk2(uint32_t hz);
void FakeHal_SetNvic(uint32_t priority, bool enabled);
void FakeHal_OcConfigFails(bool fails);
void FakeHal_IcStartFails(bool fails);
/* Silicon model knobs. */
void FakeHal_Cc3ifNeedsCc3e(bool needs);     /* flag only sets if CCER.CC3E */
void FakeHal_IcStartDoesNotRunCounter(bool); /* counter never advances      */

/* Observation. */
unsigned FakeHal_OcConfigCalls(void);
uint32_t FakeHal_OcConfigLastChannel(void);
unsigned FakeHal_IcStartCalls(void);
uint32_t FakeHal_IcStartLastChannel(void);
bool FakeHal_TimerEnabled(void);
unsigned FakeHal_Cc3ifSetEvents(void);
unsigned long FakeHal_AccessCount(void);     /* TIM8 register accesses so far */

/* Raw register access that does NOT advance the timer model. */
TIM_TypeDef *FakeHal_Regs(void);
void FakeHal_SetSrFlags(uint32_t flags);     /* set hardware status flags */
uint32_t FakeHal_Sr(void);


/* Preemption model: when set, the compare condition becomes true (CC3IF
 * latches) at the very next register access after CCR3 changes, i.e. "the
 * task was pre-empted right after arming and the match happened". */
void FakeHal_MatchRightAfterArm(bool enable);

/* Critical-section tracking (see task.h). */
unsigned FakeHal_CsEnterCount(void);
unsigned FakeHal_CsExitCount(void);
unsigned FakeHal_CsMaxDepth(void);
unsigned long FakeHal_AccessesInsideCs(void);
bool FakeHal_Cc3ieEverSet(void);        /* DIER.CC3IE seen set at any access */

#endif
