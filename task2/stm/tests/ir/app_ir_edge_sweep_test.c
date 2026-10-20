/* Timing sweep for the Phase 5 IR edge detector: does the REAL detector
 * (Core/Src/app_ir_edge.c) detect the smallest Task 2 obstacle face at the
 * intended sampling rate and speed?
 *
 *   gcc -std=c11 -Wall -Wextra -Werror -DAPP_IR_ENABLE=1 -ICore/Inc \
 *       -o tests/ir/app_ir_edge_sweep_test.exe tests/ir/app_ir_edge_sweep_test.c \
 *       Core/Src/app_ir_edge.c Core/Src/app_ir_dist.c -lm
 *   ./tests/ir/app_ir_edge_sweep_test.exe
 *
 * Smallest obstacle face: 10 cm (rules PDF, Task 2: obstacle 1 is 10 x 10 cm,
 * obstacle 2 is 10 cm wide and at least 30 cm long).
 *
 * The sensor model (a model, not a measurement of this robot):
 *   - Sharp GP2Y0A21YK: one measurement per cycle T in [28.7, 47.9] ms (datasheet,
 *     user-verified), its value on the output up to 5 ms after the cycle ends,
 *     held until the next measurement;
 *   - a measurement wholly inside the beam-on window reads VISIBLE, wholly
 *     outside reads CLEAR; one straddling an edge may read either, so three
 *     assignments are run: A all straddlers CLEAR (worst for "seen"), B all
 *     VISIBLE (worst for "clear"), C leading edge VISIBLE / trailing edge
 *     CLEAR (the optimistic mix, which is what proves nothing ever clears
 *     before the beam left);
 *   - the ADC scans every 20 ms (any phase), a scan is published 10 ms later
 *     (+0..2 ms), MotionTask polls every 10 ms (any phase) and the detector
 *     sees the newest publication.
 * The obstacle passes the sensor at the chassis speed of each SPCTRL target
 * (MOVCAL 75 counts/cm). Beam-on time = face length / speed.
 *
 * What is asserted:
 *   - 10 cm face at 3000 and 4500 counts/s: SEEN once, then CLEARED once, for
 *     every cycle / phase / delay combination, in all three assignments;
 *   - 30 cm face at every speed up to 8000: the same;
 *   - a 155 ms window (just above the model's guaranteed minimum) is always
 *     detected, a 100 ms window is not (the test is not vacuous);
 *   - never CLEARED before the beam left the obstacle, in either assignment of
 *     the edge measurements, at any speed (including 6000 / 8000 on a 10 cm
 *     face, where the face may be MISSED but is never wrongly cleared);
 *   - clear latency (beam left -> CLEARED) never exceeds 200 ms (adversarial).
 * It prints the measured table.
 */

#include "app_ir_edge.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned checks;
static unsigned failures;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
    } \
} while (0)

#define VIS_RAW 2100U
#define CLR_RAW 200U

typedef struct
{
    unsigned runs;
    unsigned detected;          /* SEEN and CLEARED once each                       */
    unsigned missed;            /* neither (the face went by unseen)                */
    unsigned other;             /* anything else (must stay 0 for the guaranteed cases) */
    unsigned falseClear;        /* CLEARED before the beam left                     */
    double worstClearLatency;
    double worstSeenAfterEnter;
} Stats;

/* Beam-on window [0, w]. `adversarial` picks how straddling measurements read. */
static int VoVisible(double t, double T, double phi, double delta, double w, int mode)
{
    double k = floor((t - phi - delta) / T) - 1.0;
    double a = phi + k * T;
    double b = a + T;

    if ((a >= 0.0) && (b <= w))
    {
        return 1;                           /* wholly inside */
    }
    if ((b <= 0.0) || (a >= w))
    {
        return 0;                           /* wholly outside */
    }
    if (mode == 0)
    {
        return 0;                           /* A: every straddler reads CLEAR   */
    }
    if (mode == 1)
    {
        return 1;                           /* B: every straddler reads VISIBLE */
    }
    return (a < 0.0) ? 1 : 0;               /* C: leading VISIBLE, trailing CLEAR */
}

static void OneRun(Stats *st, AppIrEdgeSide side, double w, double T, double phi, double delta,
                   int psi, int pubExtra, int rho, int mode)
{
    AppIrEdgeDetector d;
    AppIrSnapshot s;
    int startTick = -2000;
    int endTick = (int)w + 800;
    int tick;
    int seen = 0, cleared = 0, other = 0;
    int seenTick = 0, clearTick = 0;
    int order = 0;

    memset(&s, 0, sizeof(s));
    (void)AppIrEdge_Start(&d, side, (uint32_t)(startTick + 100000));

    for (tick = startTick; tick <= endTick; tick++)
    {
        int j;
        AppIrEdgeOutcome o;

        if ((((tick - rho) % 10) + 10) % 10 != 0)
        {
            continue;
        }

        /* newest publication at this tick: scan j sampled at psi + 20 j, published 10+pubExtra later */
        j = (int)floor((tick - 10 - pubExtra - psi) / 20.0);
        {
            double scanTime = (double)(psi + 20 * j);
            int vis = VoVisible(scanTime, T, phi, delta, w, mode);
            uint16_t sel = vis ? VIS_RAW : CLR_RAW;
            uint16_t opp = vis ? CLR_RAW : VIS_RAW;

            s.status = (uint8_t)APP_IR_STATUS_VALID;
            s.sequence = (uint32_t)(j + 100000);
            s.publishTick = (uint32_t)(psi + 20 * j + 10 + pubExtra + 100000);
            s.leftRaw = (side == APP_IR_EDGE_SIDE_LEFT) ? sel : opp;
            s.rightRaw = (side == APP_IR_EDGE_SIDE_LEFT) ? opp : sel;
        }

        o = AppIrEdge_Step(&d, &s, (uint32_t)(tick + 100000));

        if (o == APP_IR_EDGE_OUT_SEEN)
        {
            seen++;
            seenTick = tick;
            if (order == 0) { order = 1; }
        }
        else if (o == APP_IR_EDGE_OUT_CLEARED)
        {
            cleared++;
            clearTick = tick;
            if (order == 1) { order = 2; }
        }
        else if (o == APP_IR_EDGE_OUT_FAILED)
        {
            other++;
        }
    }

    st->runs++;

    if ((seen == 1) && (cleared == 1) && (order == 2) && (other == 0))
    {
        st->detected++;

        if ((double)clearTick < w)
        {
            st->falseClear++;
        }
        else if (mode == 1)
        {
            double lat = (double)clearTick - w;

            if (lat > st->worstClearLatency)
            {
                st->worstClearLatency = lat;
            }
        }

        if ((double)seenTick > st->worstSeenAfterEnter)
        {
            st->worstSeenAfterEnter = (double)seenTick;
        }
    }
    else if ((seen == 0) && (cleared == 0) && (other == 0))
    {
        st->missed++;
    }
    else
    {
        st->other++;
        if (cleared != 0)
        {
            if ((double)clearTick < w)
            {
                st->falseClear++;
            }
        }
    }
}

static void Sweep(Stats *st, double w, int mode)
{
    int side;
    int ti, pi, di, psi, pe, rho;

    memset(st, 0, sizeof(*st));

    for (side = 0; side < 2; side++)
    {
        for (ti = 0; ti < 13; ti++)
        {
            double T = 28.7 + 1.6 * ti;                 /* 28.7 .. 47.9 */

            for (pi = 0; pi < 5; pi++)
            {
                double phi = T * pi / 5.0;

                for (di = 0; di < 2; di++)
                {
                    double delta = 5.0 * di;

                    for (psi = 0; psi < 20; psi += 4)
                    {
                        for (pe = 0; pe < 3; pe += 2)
                        {
                            for (rho = 0; rho < 10; rho += 3)
                            {
                                OneRun(st, (AppIrEdgeSide)side, w, T, phi, delta, psi, pe, rho,
                                       mode);
                            }
                        }
                    }
                }
            }
        }
    }
}

typedef struct
{
    const char *name;
    int counts;
    double cmPerSec;
} Speed;

int main(void)
{
    static const Speed speeds[] = {
        { "3000", 3000, 3000.0 / 75.0 },
        { "4500", 4500, 4500.0 / 75.0 },
        { "6000", 6000, 6000.0 / 75.0 },
        { "8000", 8000, 8000.0 / 75.0 },
    };
    static const double faces[] = { 10.0, 30.0 };
    size_t f;
    size_t i;
    Stats st;

    printf("face   speed  cm/s   beam-on  mode  runs  detected  missed  other  falseClear  worstClearLat\n");

    for (f = 0U; f < 2U; f++)
    {
        for (i = 0U; i < 4U; i++)
        {
            double w = faces[f] / speeds[i].cmPerSec * 1000.0;
            int mode;

            for (mode = 0; mode < 3; mode++)
            {
                Sweep(&st, w, mode);

                printf("%4.0fcm %5s %5.1f %7.1f ms   %c %6u %8u %7u %6u %10u %9.0f ms\n",
                       faces[f], speeds[i].name, speeds[i].cmPerSec, w, 'A' + mode, st.runs,
                       st.detected, st.missed, st.other, st.falseClear, st.worstClearLatency);

                CHECK(st.falseClear == 0U);
                CHECK(st.worstClearLatency <= 200.0);
                CHECK(st.other == 0U);

                if (((f == 0U) && (i <= 1U)) || (f == 1U))
                {
                    /* guaranteed cases: always seen and cleared exactly once */
                    CHECK(st.detected == st.runs);
                }
                else
                {
                    /* 10 cm at 6000 / 8000: may be missed, must not misbehave */
                    CHECK(st.detected + st.missed == st.runs);

                    if (mode == 0)
                    {
                        CHECK(st.missed > 0U);      /* the worst case really does miss */
                    }
                }
            }
        }
    }

    /* just above the model's guaranteed minimum (150.1 ms) is always detected */
    {
        int mode;

        for (mode = 0; mode < 3; mode++)
        {
            Sweep(&st, 155.0, mode);
            CHECK(st.detected == st.runs);
            printf("155 ms window, mode %c: %u/%u detected\n", 'A' + mode, st.detected, st.runs);
        }
    }

    /* and a window well below it is not (the sweep is not vacuous) */
    Sweep(&st, 100.0, 0);
    CHECK(st.missed > 0U);
    CHECK(st.falseClear == 0U);
    printf("100 ms window, mode A: %u/%u detected, %u missed\n", st.detected, st.runs, st.missed);

    printf("%u checks, %u failures\n", checks, failures);
    return failures == 0U ? 0 : 1;
}
