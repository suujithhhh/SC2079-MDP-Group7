/*
 * app_ultrasonic_hw.h
 *
 * Thin TIM8 / GPIO port for app_ultrasonic.c. Every function is a single
 * register operation (or a start-up check), so the driver core has no HAL
 * dependency and is host-testable against a simulated timer
 * (tests/ultrasonic/fakes).
 *
 * Ownership: only app_ultrasonic.c calls these. The *Irq / *Flag /
 * SetCompare / SetCapturePolarity functions are called either from the
 * TIM8_CC interrupt or from the task inside taskENTER_CRITICAL(), which
 * masks that interrupt (priority 5 is not above
 * configMAX_SYSCALL_INTERRUPT_PRIORITY) -- the read-modify-writes on
 * DIER / CCER / SR therefore never race.
 */

#ifndef INC_APP_ULTRASONIC_HW_H_
#define INC_APP_ULTRASONIC_HW_H_

#include <stdint.h>
#include <stdbool.h>

/* Start-up (task context, once). Returns a bitmask of APP_US_FAULT_* bits
 * (0 = everything verified). It never changes the pin, clock, prescaler or
 * NVIC priority: it only verifies them (clock, ARR, NVIC, and CH2 = direct
 * TI2 / IC prescaler DIV1 / filter 4), failing closed before touching
 * anything, then configures TIM8_CH3 as a pin-less frozen-mode output
 * compare, starts the CH2 capture channel (counter on, CC2/CC3 interrupts
 * left disabled, PC6 left low) and runs a bounded one-time self-test that
 * CC3IF really sets on a CNT==CCR3 match with CC3E disabled. */
uint16_t AppUltrasonicHw_Setup(void);

uint16_t AppUltrasonicHw_Counter(void);                 /* TIM8->CNT          */

void AppUltrasonicHw_TriggerHigh(void);                 /* PC6 high (BSRR)    */
void AppUltrasonicHw_TriggerLow(void);                  /* PC6 low  (BSRR)    */
bool AppUltrasonicHw_EchoHigh(void);                    /* PC7 input level    */

void AppUltrasonicHw_SetCompare(uint16_t ticks);        /* TIM8->CCR3         */
void AppUltrasonicHw_EnableCompareIrq(bool enable);     /* DIER.CC3IE         */
void AppUltrasonicHw_ClearCompareFlag(void);            /* SR.CC3IF           */

void AppUltrasonicHw_EnableCaptureIrq(bool enable);     /* DIER.CC2IE         */
void AppUltrasonicHw_ClearCaptureFlags(void);           /* SR.CC2IF + CC2OF   */
void AppUltrasonicHw_SetCapturePolarity(bool falling);  /* CCER.CC2P (1 store)*/
uint16_t AppUltrasonicHw_ReadCapture(void);             /* TIM8->CCR2         */
/* Returns true and clears SR.CC2OF if a second capture overwrote CCR2. */
bool AppUltrasonicHw_TakeCaptureOverrun(void);

#endif /* INC_APP_ULTRASONIC_HW_H_ */
