/*
 * app_park_trace_fmt.c
 *
 * Text formatter for the measurement-only USPARK trace. Pure: no I/O, no HAL,
 * no RTOS, integers and fixed strings only (no floating point). Kept apart
 * from app_park_trace.c so the producer side of the trace has no reference to
 * snprintf or to the UART at all (checked on the object file in the tests).
 * Only CommTask ever calls it.
 *
 * Every line starts "USPARK,TRACE,EV:<event>,N:<index>,T:<tick>" and ends in
 * '\n'. A worst-case line (every field at its maximum width) is 117 bytes;
 * AppParkTrace_Format() refuses anything of APP_PARK_TRACE_LINE_MAX (120) or
 * more rather than sending it.
 *
 *   US/PRE  ...,OD:<ms>,SEQ:<n>,RAW:<mm|NA>,FILT:<mm|NA>,AGE:<ms|NA>,
 *               PU:<us|NA>,OEL:<count>,OER:<count>
 *           T = publication tick; OD = observation delay (observation tick =
 *           T + OD); OEL/OER are OBSERVATION-time encoder counts.
 *   ARM     ...,W:<movSeq>,SEQ:<n>,FILT:<mm|NA>,CN:<0|1>,P:<profile>,
 *               FM:<hex>,EL:<count>,ER:<count>
 *   NEAR    ...,K:<1..3>,SEQ:<n>,FILT:<mm|NA>,EL:,ER:
 *   REQ     ...,R:<cause>,SEQ:<n>,PD:<ms|NA>,EL:,ER:,VL:<cps|NA>,VR:<cps|NA>
 *           PD = request tick minus the latest publication's tick.
 *   ACC/TAKE/BEND/ZL/ZR  ...,EL:,ER:
 *   BRK     ...,EL:,ER:,VL:,VR:
 *   ZERO    ...,EL:,ER:,TO:<0|1>
 *   DONE    ...,W:<movSeq>,R:<result>,DR:<dropped>,RF:<ring-full>,ZT:<0|1>,
 *               SK:<skipped windows>
 */

#include "app_park_trace.h"

#if APP_USPARK_TRACE

#include <stdio.h>

#define CLAMP_MS   999U
#define CLAMP_MM   9999U
#define CLAMP_PD   9999U

static const char *U16Text(char *buf, size_t size, uint16_t v, uint16_t clamp)
{
    if (v == APP_PARK_TRACE_NA_U16)
    {
        (void)snprintf(buf, size, "NA");
    }
    else
    {
        (void)snprintf(buf, size, "%u", (unsigned)((v > clamp) ? clamp : v));
    }

    return buf;
}

static const char *VelText(char *buf, size_t size, int16_t v)
{
    if (v == APP_PARK_TRACE_NA_V)
    {
        (void)snprintf(buf, size, "NA");
    }
    else
    {
        int d = v;

        if (d > 9999) { d = 9999; }
        if (d < -9999) { d = -9999; }
        (void)snprintf(buf, size, "%d", d);
    }

    return buf;
}

static const char *EventName(uint8_t ev)
{
    switch (ev)
    {
        case APP_PARK_EV_US:   return "US";
        case APP_PARK_EV_PRE:  return "PRE";
        case APP_PARK_EV_ARM:  return "ARM";
        case APP_PARK_EV_NEAR: return "NEAR";
        case APP_PARK_EV_REQ:  return "REQ";
        case APP_PARK_EV_ACC:  return "ACC";
        case APP_PARK_EV_TAKE: return "TAKE";
        case APP_PARK_EV_BRK:  return "BRK";
        case APP_PARK_EV_BEND: return "BEND";
        case APP_PARK_EV_ZL:   return "ZL";
        case APP_PARK_EV_ZR:   return "ZR";
        case APP_PARK_EV_ZERO: return "ZERO";
        case APP_PARK_EV_DONE: return "DONE";
        default:               return NULL;
    }
}

static const char *CauseName(uint8_t cause)
{
    switch (cause)
    {
        case APP_PARK_CAUSE_FAULT:    return "FAULT";
        case APP_PARK_CAUSE_NOUPDATE: return "NOUPDATE";
        case APP_PARK_CAUSE_INVALID:  return "INVALID";
        case APP_PARK_CAUSE_NEAR:     return "NEAR";
        default:                      return "?";
    }
}

static const char *ResultName(uint8_t result)
{
    switch (result)
    {
        case APP_PARK_RES_OK:      return "OK";
        case APP_PARK_RES_TIMEOUT: return "TO";
        case APP_PARK_RES_NOREQ:   return "NOREQ";
        case APP_PARK_RES_NOACC:   return "NOACC";
        case APP_PARK_RES_REFUSED: return "REFUSED";
        case APP_PARK_RES_ABORT:   return "ABORT";
        default:                   return "?";
    }
}

static const char *ProfileName(uint8_t profile)
{
    switch (profile)
    {
        case 0U:  return "PREC";
        case 1U:  return "STD";
        case 2U:  return "T2T";
        default:  return "CUST";
    }
}

int AppParkTrace_Format(const AppParkTraceRecord *r, char *buf, size_t size)
{
    const char *name;
    char a[8];
    char b[8];
    char c[8];
    char d[8];
    char e[8];
    int len;

    if ((r == NULL) || (buf == NULL) || (size == 0U))
    {
        return -1;
    }

    name = EventName(r->ev);

    if (name == NULL)
    {
        return -1;
    }

    switch (r->ev)
    {
        case APP_PARK_EV_US:
        case APP_PARK_EV_PRE:
            len = snprintf(buf, size,
                "USPARK,TRACE,EV:%s,N:%u,T:%lu,OD:%s,SEQ:%u,RAW:%s,FILT:%s,"
                "AGE:%s,PU:%s,OEL:%ld,OER:%ld\n",
                name, (unsigned)r->n, (unsigned long)r->tick,
                U16Text(a, sizeof(a), r->obs, CLAMP_MS),
                (unsigned)r->seq,
                U16Text(b, sizeof(b), r->raw, CLAMP_MM),
                U16Text(c, sizeof(c), r->filt, CLAMP_MM),
                U16Text(d, sizeof(d), r->age, CLAMP_MS),
                U16Text(e, sizeof(e), r->pulse,
                        (uint16_t)(APP_PARK_TRACE_NA_U16 - 1U)),
                (long)r->el, (long)r->er);
            break;

        case APP_PARK_EV_ARM:
            len = snprintf(buf, size,
                "USPARK,TRACE,EV:ARM,N:%u,T:%lu,W:%u,SEQ:%u,FILT:%s,CN:%u,"
                "P:%s,FM:%X,EL:%ld,ER:%ld\n",
                (unsigned)r->n, (unsigned long)r->tick, (unsigned)r->w,
                (unsigned)r->seq,
                U16Text(a, sizeof(a), r->filt, CLAMP_MM),
                (unsigned)(r->arg != 0U ? 1U : 0U),
                ProfileName(r->flags), (unsigned)r->fm,
                (long)r->el, (long)r->er);
            break;

        case APP_PARK_EV_NEAR:
            len = snprintf(buf, size,
                "USPARK,TRACE,EV:NEAR,N:%u,T:%lu,K:%u,SEQ:%u,FILT:%s,"
                "EL:%ld,ER:%ld\n",
                (unsigned)r->n, (unsigned long)r->tick, (unsigned)r->arg,
                (unsigned)r->seq,
                U16Text(a, sizeof(a), r->filt, CLAMP_MM),
                (long)r->el, (long)r->er);
            break;

        case APP_PARK_EV_REQ:
            len = snprintf(buf, size,
                "USPARK,TRACE,EV:REQ,N:%u,T:%lu,R:%s,SEQ:%u,PD:%s,"
                "EL:%ld,ER:%ld,VL:%s,VR:%s\n",
                (unsigned)r->n, (unsigned long)r->tick, CauseName(r->arg),
                (unsigned)r->seq,
                U16Text(a, sizeof(a), r->obs, CLAMP_PD),
                (long)r->el, (long)r->er,
                VelText(b, sizeof(b), r->vl),
                VelText(c, sizeof(c), r->vr));
            break;

        case APP_PARK_EV_BRK:
            len = snprintf(buf, size,
                "USPARK,TRACE,EV:BRK,N:%u,T:%lu,EL:%ld,ER:%ld,VL:%s,VR:%s\n",
                (unsigned)r->n, (unsigned long)r->tick,
                (long)r->el, (long)r->er,
                VelText(a, sizeof(a), r->vl),
                VelText(b, sizeof(b), r->vr));
            break;

        case APP_PARK_EV_ZERO:
            len = snprintf(buf, size,
                "USPARK,TRACE,EV:ZERO,N:%u,T:%lu,EL:%ld,ER:%ld,TO:%u\n",
                (unsigned)r->n, (unsigned long)r->tick,
                (long)r->el, (long)r->er,
                (unsigned)(((r->flags & APP_PARK_FLAG_ZERO_TIMEOUT) != 0U) ?
                           1U : 0U));
            break;

        case APP_PARK_EV_DONE:
            len = snprintf(buf, size,
                "USPARK,TRACE,EV:DONE,N:%u,T:%lu,W:%u,R:%s,DR:%u,RF:%u,"
                "ZT:%u,SK:%u\n",
                (unsigned)r->n, (unsigned long)r->tick, (unsigned)r->w,
                ResultName(r->arg), (unsigned)r->raw, (unsigned)r->filt,
                (unsigned)(((r->flags & APP_PARK_FLAG_ZERO_TIMEOUT) != 0U) ?
                           1U : 0U),
                (unsigned)r->fm);
            break;

        default:    /* ACC, TAKE, BEND, ZL, ZR */
            len = snprintf(buf, size,
                "USPARK,TRACE,EV:%s,N:%u,T:%lu,EL:%ld,ER:%ld\n",
                name, (unsigned)r->n, (unsigned long)r->tick,
                (long)r->el, (long)r->er);
            break;
    }

    if ((len < 0) || ((size_t)len >= size) ||
        ((unsigned)len >= APP_PARK_TRACE_LINE_MAX))
    {
        return -1;
    }

    return len;
}

#endif /* APP_USPARK_TRACE */
