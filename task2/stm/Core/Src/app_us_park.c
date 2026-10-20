/*
 * app_us_park.c
 *
 * Pure ultrasonic parking-protection detector. See app_us_park.h.
 *
 * Evaluation order in AppUsPark_Step() (every step is deterministic):
 *   1. disabled -> return, inputs not read
 *   2. consume a NEW ultrasonic publication (sequence differs) and classify
 *      it: Q (valid, fresh, fault-free, filtered <= 300), V (same, > 300) or
 *      I (anything else). Re-polling the same sequence changes nothing.
 *   3. a V sample above 320 mm clears confirmedNear (any time, even idle)
 *   4. report a fault once per fault episode
 *   5. not an active straight-forward movement -> clear counters, disarm
 *   6. a new forward movement -> clear counters and the old latch, arm only
 *      from confirmedNear, restart lastProcessed/lastNew, evaluate nothing
 *   7. already latched this movement -> nothing
 *   8. a STOP is already outstanding -> latch (superseded), nothing
 *   9. causes: FAULT > NOUPDATE > per-sample (NEAR / valid / INVALID)
 */

#include "app_us_park.h"

static bool IsFault(const AppUltrasonicSnapshot *s)
{
    return (s->state == APP_US_STATE_FAULT) || (s->faultMask != 0U);
}

void AppUsPark_Init(AppUsPark *park)
{
    park->enabled = false;
    park->armed = false;
    park->confirmedNear = false;
    park->faultReported = false;
    park->nearCount = 0U;
    park->invalidCount = 0U;
    park->lastProcessedUsSeq = 0U;
    park->lastNewUsMs = 0U;
    park->trackedMovementSeq = 0U;
    park->latchedMovementSeq = 0U;
}

void AppUsPark_Enable(AppUsPark *park, uint32_t usSequence, uint32_t nowMs)
{
    AppUsPark_Init(park);
    park->enabled = true;
    park->lastProcessedUsSeq = usSequence;
    park->lastNewUsMs = nowMs;
}

void AppUsPark_Disable(AppUsPark *park)
{
    AppUsPark_Init(park);
}

void AppUsPark_StopAccepted(AppUsPark *park, AppUsParkCause cause)
{
    if (cause == APP_USPARK_CAUSE_NEAR)
    {
        park->confirmedNear = true;
        park->armed = true;
    }
}

AppUsParkOutput AppUsPark_Step(AppUsPark *park, const AppUsParkInputs *in)
{
    AppUsParkOutput out = { APP_USPARK_CAUSE_NONE, false };
    const AppUltrasonicSnapshot *s;
    bool fault;
    bool newSample;
    bool qualifies = false;
    bool isNear = false;
    bool forwardActive;

    if (!park->enabled)
    {
        return out;
    }

    s = in->snapshot;
    fault = IsFault(s);
    newSample = (s->sequence != park->lastProcessedUsSeq);

    /* 2. consume a new publication */
    if (newSample)
    {
        park->lastProcessedUsSeq = s->sequence;
        park->lastNewUsMs = in->nowMs;

        qualifies = (s->result == APP_US_RESULT_VALID) &&
                    (s->filteredValid != 0U) &&
                    AppUltrasonic_IsFresh(s, in->nowTick) &&
                    !fault;
        isNear = qualifies && (s->filteredMm <= APP_USPARK_STOP_MM);

        /* A healthy, fault-free publication ends a fault episode. */
        if (!fault)
        {
            park->faultReported = false;
        }

        /* 3. valid reading above the rearm boundary clears confirmedNear */
        if (qualifies && (s->filteredMm > APP_USPARK_REARM_MM))
        {
            park->confirmedNear = false;
        }
    }

    /* 4. one USPARK,FAULT notification per fault episode */
    if (fault && !park->faultReported)
    {
        park->faultReported = true;
        out.notifyFault = true;
    }

    /* 5. only an active straight-forward movement is protected */
    forwardActive = (in->motion.state == APP_MOTION_DISTANCE) &&
                    (in->motion.command == MOTION_FORWARD) &&
                    (in->motion.movementSeq != 0U);

    if (!forwardActive)
    {
        park->nearCount = 0U;
        park->invalidCount = 0U;
        park->armed = false;
        park->trackedMovementSeq = 0U;
        return out;
    }

    /* 6. a genuinely new forward movement starts from a clean slate */
    if (in->motion.movementSeq != park->trackedMovementSeq)
    {
        park->trackedMovementSeq = in->motion.movementSeq;
        park->nearCount = 0U;
        park->invalidCount = 0U;
        park->latchedMovementSeq = 0U;
        park->armed = park->confirmedNear;
        park->lastProcessedUsSeq = s->sequence;
        park->lastNewUsMs = in->nowMs;
        return out;
    }

    /* 7. at most one automatic STOP per movement */
    if (park->latchedMovementSeq == park->trackedMovementSeq)
    {
        return out;
    }

    /* 8. a STOP is already outstanding: it wins, no automatic request */
    if (in->stopGateActive)
    {
        park->latchedMovementSeq = park->trackedMovementSeq;
        return out;
    }

    /* 9. causes, in priority order */
    if (fault)
    {
        out.cause = APP_USPARK_CAUSE_FAULT;
    }
    else if (park->armed &&
             ((uint32_t)(in->nowMs - park->lastNewUsMs) >
              APP_USPARK_NOUPDATE_MS))
    {
        out.cause = APP_USPARK_CAUSE_NOUPDATE;
    }
    else if (newSample)
    {
        if (isNear)
        {
            park->armed = true;
            park->invalidCount = 0U;
            park->nearCount++;

            if (park->nearCount >= APP_USPARK_NEAR_SAMPLES)
            {
                out.cause = APP_USPARK_CAUSE_NEAR;
            }
        }
        else if (qualifies)
        {
            park->nearCount = 0U;
            park->invalidCount = 0U;
            park->armed = (s->filteredMm <= APP_USPARK_ARM_MM);
        }
        else
        {
            /* Any new non-qualifying publication breaks the near run. */
            park->nearCount = 0U;

            if (park->armed)
            {
                park->invalidCount++;

                if (park->invalidCount >= APP_USPARK_INVALID_SAMPLES)
                {
                    out.cause = APP_USPARK_CAUSE_INVALID;
                }
            }
        }
    }

    if (out.cause != APP_USPARK_CAUSE_NONE)
    {
        park->latchedMovementSeq = park->trackedMovementSeq;
        park->nearCount = 0U;
        park->invalidCount = 0U;
    }

    return out;
}

AppUsParkState AppUsPark_GetState(const AppUsPark *park)
{
    if (!park->enabled)
    {
        return APP_USPARK_DISABLED;
    }

    if (park->trackedMovementSeq == 0U)
    {
        return APP_USPARK_UNARMED;
    }

    if (park->latchedMovementSeq == park->trackedMovementSeq)
    {
        return APP_USPARK_LATCHED;
    }

    if (park->nearCount == 2U)
    {
        return APP_USPARK_NEAR2;
    }

    if (park->nearCount == 1U)
    {
        return APP_USPARK_NEAR1;
    }

    return park->armed ? APP_USPARK_ARMED : APP_USPARK_UNARMED;
}
