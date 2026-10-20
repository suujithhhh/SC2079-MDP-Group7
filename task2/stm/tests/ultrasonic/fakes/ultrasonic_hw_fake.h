/* Host-test fake of the TIM8 / GPIO port (Core/Inc/app_ultrasonic_hw.h).
 *
 * A 1 MHz TIM8 simulation just detailed enough to exercise the real
 * Core/Src/app_ultrasonic.c end to end:
 *   - a free-running 16-bit counter, stepped one microsecond at a time;
 *   - CH3 compare flag set when CNT == CCR3 (whether or not enabled, like
 *     the hardware), CH2 capture with polarity, CC2IF and CC2OF;
 *   - interrupt dispatch that mimics HAL_TIM_IRQHandler(): SR and DIER are
 *     sampled ONCE, then CC2 is serviced before CC3, each cleared before
 *     its callback runs, so "both pending in one entry" is reproduced;
 *   - a programmable HC-SR04-like Echo that starts a configurable time
 *     after the Trigger falls;
 *   - bookkeeping of every Trigger pulse (start time, width).
 * Interrupt latency is zero in this model. */
#ifndef ULTRASONIC_HW_FAKE_H
#define ULTRASONIC_HW_FAKE_H

#include <stdbool.h>
#include <stdint.h>

#define FAKEUS_MAX_TRIGGERS 256U

void FakeUs_Reset(void);

/* Value AppUltrasonicHw_Setup() will return (APP_US_FAULT_* bits). */
void FakeUs_SetSetupFaultMask(uint16_t mask);
unsigned FakeUs_SetupCallCount(void);
unsigned FakeUs_TriggerLowCallCount(void);   /* AppUltrasonicHw_TriggerLow() calls */

/* Echo model, applied to the NEXT Trigger pulse and every one after it:
 * no echo at all / rises `riseDelayUs` after the Trigger falls and stays
 * high `widthUs` (widthUs == 0 means it rises and never falls). */
void FakeUs_EchoNone(void);
void FakeUs_EchoPulse(uint32_t riseDelayUs, uint32_t widthUs);
void FakeUs_EchoStuckHigh(bool stuck);   /* pin physically held high */
void FakeUs_ReleaseEcho(void);           /* Echo line goes low, schedule off */

/* Pretend the TIM8_CC interrupt is lost (never dispatched). */
void FakeUs_DropCompareInterrupts(bool drop);
void FakeUs_DropCaptureInterrupts(bool drop);

void FakeUs_SetCounter(uint16_t value);
uint16_t FakeUs_Counter(void);

/* Advance simulated time, dispatching interrupts like the HAL would. */
void FakeUs_AdvanceUs(uint32_t microseconds);

/* Raise a capture event with arbitrary data and dispatch it (if CC2IE). */
void FakeUs_InjectCapture(uint16_t capture, bool overrun);
/* Pre-load stale CC2 flags (as if edges happened while IRQs were off). */
void FakeUs_SetStaleCaptureFlags(void);

/* Observation. */
bool FakeUs_TriggerPin(void);
bool FakeUs_CompareIrqEnabled(void);
bool FakeUs_CaptureIrqEnabled(void);
bool FakeUs_CapturePolarityFalling(void);
bool FakeUs_CaptureFlagsPending(void);
uint16_t FakeUs_CompareValue(void);
uint16_t FakeUs_TriggerStartCount(void);   /* CNT when PC6 last went high */
unsigned FakeUs_TriggerCount(void);
uint32_t FakeUs_TriggerTimeUs(unsigned index);      /* abs sim time, high */
uint32_t FakeUs_TriggerWidthUs(unsigned index);     /* 0 if still high    */
uint32_t FakeUs_NowUs(void);

#endif /* ULTRASONIC_HW_FAKE_H */
