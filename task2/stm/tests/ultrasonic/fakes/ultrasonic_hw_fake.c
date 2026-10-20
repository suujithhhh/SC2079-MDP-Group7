/* Host-test fake of the TIM8 / GPIO port. See ultrasonic_hw_fake.h. */

#include "ultrasonic_hw_fake.h"

#include "app_ultrasonic.h"
#include "app_ultrasonic_hw.h"

#include <string.h>

typedef enum { ECHO_MODEL_NONE = 0, ECHO_MODEL_PULSE } EchoModel;

static struct
{
    uint32_t nowUs;
    uint16_t cnt;
    uint16_t ccr2;
    uint16_t ccr3;
    bool cc2if;
    bool cc2of;
    bool cc3if;
    bool cc2ie;
    bool cc3ie;
    bool polFalling;

    bool trigPin;
    uint16_t trigStartCnt;
    unsigned trigCount;
    uint32_t trigAtUs[FAKEUS_MAX_TRIGGERS];
    uint32_t trigWidthUs[FAKEUS_MAX_TRIGGERS];

    EchoModel model;
    uint32_t modelRiseDelay;
    uint32_t modelWidth;
    bool echo;
    bool echoForcedHigh;
    bool scheduled;
    bool hasFall;
    uint32_t riseAtUs;
    uint32_t fallAtUs;

    bool dropCompare;
    bool dropCapture;

    uint16_t setupFaultMask;
    unsigned triggerLowCalls;
    unsigned setupCalls;
} s;

void FakeUs_Reset(void)
{
    memset(&s, 0, sizeof(s));
}

void FakeUs_SetSetupFaultMask(uint16_t mask) { s.setupFaultMask = mask; }
unsigned FakeUs_TriggerLowCallCount(void) { return s.triggerLowCalls; }
unsigned FakeUs_SetupCallCount(void) { return s.setupCalls; }

void FakeUs_EchoNone(void) { s.model = ECHO_MODEL_NONE; }

void FakeUs_EchoPulse(uint32_t riseDelayUs, uint32_t widthUs)
{
    s.model = ECHO_MODEL_PULSE;
    s.modelRiseDelay = riseDelayUs;
    s.modelWidth = widthUs;
}

void FakeUs_EchoStuckHigh(bool stuck) { s.echoForcedHigh = stuck; }
void FakeUs_ReleaseEcho(void) { s.echo = false; s.scheduled = false; }
void FakeUs_DropCompareInterrupts(bool drop) { s.dropCompare = drop; }
void FakeUs_DropCaptureInterrupts(bool drop) { s.dropCapture = drop; }
void FakeUs_SetCounter(uint16_t value) { s.cnt = value; }
uint16_t FakeUs_Counter(void) { return s.cnt; }

/* A capture happens in hardware regardless of CC2IE. */
static void Capture(uint16_t value, bool overrun)
{
    if (s.cc2if)
    {
        s.cc2of = true;     /* previous capture was not yet serviced */
    }

    if (overrun)
    {
        s.cc2of = true;
    }

    s.ccr2 = value;
    s.cc2if = true;
}

static void Edge(bool rising)
{
    if (rising != s.polFalling)
    {
        Capture(s.cnt, false);
    }
}

/* HAL_TIM_IRQHandler(): SR and DIER sampled once; CC2 before CC3; the
 * flag is cleared before its callback runs. */
static void Dispatch(void)
{
    bool flag2 = s.cc2if;
    bool flag3 = s.cc3if;
    bool source2 = s.cc2ie;
    bool source3 = s.cc3ie;

    if (flag2 && source2 && !s.dropCapture)
    {
        s.cc2if = false;
        AppUltrasonic_OnCapture();
    }

    if (flag3 && source3 && !s.dropCompare)
    {
        s.cc3if = false;
        AppUltrasonic_OnCompare();
    }
}

static void Step(void)
{
    s.nowUs++;
    s.cnt++;

    if (s.cnt == s.ccr3)
    {
        s.cc3if = true;     /* set on every match, enabled or not */
    }

    if (s.scheduled)
    {
        if (s.nowUs == s.riseAtUs)
        {
            s.echo = true;
            Edge(true);
        }

        if (s.hasFall && (s.nowUs == s.fallAtUs))
        {
            s.echo = false;
            Edge(false);
        }
    }

    Dispatch();
}

void FakeUs_AdvanceUs(uint32_t microseconds)
{
    while (microseconds-- > 0U)
    {
        Step();
    }
}

void FakeUs_InjectCapture(uint16_t capture, bool overrun)
{
    s.cc2if = false;
    Capture(capture, overrun);
    Dispatch();
}

void FakeUs_SetStaleCaptureFlags(void)
{
    s.cc2if = true;
    s.cc2of = true;
}

bool FakeUs_TriggerPin(void) { return s.trigPin; }
bool FakeUs_CompareIrqEnabled(void) { return s.cc3ie; }
bool FakeUs_CaptureIrqEnabled(void) { return s.cc2ie; }
bool FakeUs_CapturePolarityFalling(void) { return s.polFalling; }
bool FakeUs_CaptureFlagsPending(void) { return s.cc2if || s.cc2of; }
uint16_t FakeUs_CompareValue(void) { return s.ccr3; }
uint16_t FakeUs_TriggerStartCount(void) { return s.trigStartCnt; }
unsigned FakeUs_TriggerCount(void) { return s.trigCount; }
uint32_t FakeUs_TriggerTimeUs(unsigned index) { return s.trigAtUs[index]; }
uint32_t FakeUs_TriggerWidthUs(unsigned index) { return s.trigWidthUs[index]; }
uint32_t FakeUs_NowUs(void) { return s.nowUs; }

/* --- the port ------------------------------------------------------- */

uint16_t AppUltrasonicHw_Setup(void)
{
    s.setupCalls++;

    if (s.setupFaultMask == 0U)
    {
        s.trigPin = false;
        s.cc2ie = false;
        s.cc3ie = false;
        s.polFalling = false;
        s.cc2if = false;
        s.cc2of = false;
        s.cc3if = false;
    }

    return s.setupFaultMask;
}

uint16_t AppUltrasonicHw_Counter(void) { return s.cnt; }

void AppUltrasonicHw_TriggerHigh(void)
{
    if (!s.trigPin && (s.trigCount < FAKEUS_MAX_TRIGGERS))
    {
        s.trigPin = true;
        s.trigStartCnt = s.cnt;
        s.trigAtUs[s.trigCount] = s.nowUs;
        s.trigWidthUs[s.trigCount] = 0U;
        s.trigCount++;
    }
}

void AppUltrasonicHw_TriggerLow(void)
{
    s.triggerLowCalls++;

    if (s.trigPin)
    {
        s.trigPin = false;
        s.trigWidthUs[s.trigCount - 1U] =
            s.nowUs - s.trigAtUs[s.trigCount - 1U];

        if (s.model == ECHO_MODEL_PULSE)
        {
            s.scheduled = true;
            s.riseAtUs = s.nowUs + s.modelRiseDelay;
            s.hasFall = (s.modelWidth != 0U);
            s.fallAtUs = s.riseAtUs + s.modelWidth;
        }
        else
        {
            s.scheduled = false;
        }
    }
}

bool AppUltrasonicHw_EchoHigh(void) { return s.echo || s.echoForcedHigh; }

void AppUltrasonicHw_SetCompare(uint16_t ticks) { s.ccr3 = ticks; }
void AppUltrasonicHw_EnableCompareIrq(bool enable) { s.cc3ie = enable; }
void AppUltrasonicHw_ClearCompareFlag(void) { s.cc3if = false; }

void AppUltrasonicHw_EnableCaptureIrq(bool enable) { s.cc2ie = enable; }

void AppUltrasonicHw_ClearCaptureFlags(void)
{
    s.cc2if = false;
    s.cc2of = false;
}

void AppUltrasonicHw_SetCapturePolarity(bool falling) { s.polFalling = falling; }
uint16_t AppUltrasonicHw_ReadCapture(void) { return s.ccr2; }

bool AppUltrasonicHw_TakeCaptureOverrun(void)
{
    bool overrun = s.cc2of;

    s.cc2of = false;
    return overrun;
}
