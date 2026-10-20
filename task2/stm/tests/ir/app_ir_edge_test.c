/* Host test for the pure Phase 5 IR edge detector (Core/Src/app_ir_edge.c).
 *
 *   gcc -std=c11 -Wall -Wextra -Werror -DAPP_IR_ENABLE=1 -ICore/Inc \
 *       -o tests/ir/app_ir_edge_test.exe tests/ir/app_ir_edge_test.c \
 *       Core/Src/app_ir_edge.c Core/Src/app_ir_dist.c
 *   ./tests/ir/app_ir_edge_test.exe
 *
 * The test follows the configuration constants, so these must pass too:
 *   ... -DAPP_IR_EDGE_SEEN_SCANS=5U -DAPP_IR_EDGE_CLEAR_SCANS=6U
 *   ... -DAPP_IR_EDGE_VISIBLE_MAX_MM=180U -DAPP_IR_EDGE_CLEAR_MIN_MM=300U
 *   ... -DAPP_IR_EDGE_READY_SCANS=3U
 * and these must FAIL to compile (a single held sensor measurement could
 * confirm): -DAPP_IR_EDGE_SEEN_SCANS=3U, -DAPP_IR_EDGE_CLEAR_SCANS=3U.
 *
 * What is proved: the raw thresholds equal the Phase 3 conversion for every
 * count 0..4095 of both sensors; LEFT/RIGHT mapping; FAR -> visible -> clear;
 * already visible at start; FAR forever; noise and dropouts; a held sensor
 * measurement (3 identical scans) never confirms; NEAR is visible and never
 * clear; ERROR / stale / FAULT / no data; readiness and both timeouts at their
 * exact boundaries; each publication consumed once; sequence gaps and wrap;
 * tick wrap; one-shot outcomes; and a randomised differential run against an
 * independently written model.
 */

#include "app_ir_edge.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

_Static_assert((APP_IR_EDGE_READY_SCANS >= 2U) && (APP_IR_EDGE_READY_SCANS < APP_IR_EDGE_SEEN_SCANS),
               "test assumes 2 <= READY_SCANS < SEEN_SCANS");

static unsigned checks;
static unsigned failures;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
    } \
} while (0)

#define V APP_IR_EDGE_CLASS_VISIBLE
#define C APP_IR_EDGE_CLASS_CLEAR
#define A APP_IR_EDGE_CLASS_AMBIGUOUS

/* ---------------------------------------------------------------------- */
/* Helpers                                                                */
/* ---------------------------------------------------------------------- */

static AppIrEdgeDetector det;
static AppIrSnapshot snap;
static uint32_t nowTick;
static uint32_t seqNo;
static uint16_t visRaw[2], clrRaw[2], ambRaw[2];

static uint32_t NextSeq(uint32_t s)
{
    s++;
    return (s == 0U) ? 1U : s;
}

static void ComputeSamples(void)
{
    AppIrEdgeDetector d;
    int side;

    for (side = 0; side < 2; side++)
    {
        (void)AppIrEdge_Start(&d, (AppIrEdgeSide)side, 0U);
        visRaw[side] = (uint16_t)(d.visibleMinAdc + 50U);
        clrRaw[side] = (uint16_t)(d.clearMaxAdc - 50U);
        ambRaw[side] = (uint16_t)((d.visibleMinAdc + d.clearMaxAdc) / 2U);
    }
}

static void ResetWorld(AppIrEdgeSide side, uint32_t startTick, uint32_t firstSeq)
{
    memset(&snap, 0, sizeof(snap));
    nowTick = startTick;
    seqNo = firstSeq;                   /* the next published sequence */
    CHECK(AppIrEdge_Start(&det, side, startTick));
}

/* One raw count for `cls` on the selected side; the OTHER sensor shows the
 * opposite class so a mapping error is visible. */
static void FillRaw(AppIrEdgeSide side, int cls)
{
    int other = (side == APP_IR_EDGE_SIDE_LEFT) ? 1 : 0;
    uint16_t sel = (cls == V) ? visRaw[side] : (cls == C) ? clrRaw[side] : ambRaw[side];
    uint16_t opp = (cls == V) ? clrRaw[other] : visRaw[other];

    snap.leftRaw = (side == APP_IR_EDGE_SIDE_LEFT) ? sel : opp;
    snap.rightRaw = (side == APP_IR_EDGE_SIDE_LEFT) ? opp : sel;
}

/* A raw count given explicitly (selected side), other sensor opposite. */
static void FillRawValue(AppIrEdgeSide side, uint16_t value)
{
    snap.leftRaw = (side == APP_IR_EDGE_SIDE_LEFT) ? value : 4000U;
    snap.rightRaw = (side == APP_IR_EDGE_SIDE_LEFT) ? 4000U : value;
}

/* Publishes the next scan 20 ms after the previous one and polls it. */
static AppIrEdgeOutcome Scan(int cls)
{
    nowTick += 20U;
    snap.sequence = seqNo;
    snap.publishTick = nowTick;
    snap.lastValidTick = nowTick;
    snap.status = (uint8_t)APP_IR_STATUS_VALID;
    FillRaw((AppIrEdgeSide)det.side, cls);
    seqNo = NextSeq(seqNo);
    return AppIrEdge_Step(&det, &snap, nowTick);
}

static AppIrEdgeOutcome ScanRaw(uint16_t value)
{
    nowTick += 20U;
    snap.sequence = seqNo;
    snap.publishTick = nowTick;
    snap.lastValidTick = nowTick;
    snap.status = (uint8_t)APP_IR_STATUS_VALID;
    FillRawValue((AppIrEdgeSide)det.side, value);
    seqNo = NextSeq(seqNo);
    return AppIrEdge_Step(&det, &snap, nowTick);
}

static AppIrEdgeOutcome ScanError(void)
{
    nowTick += 20U;
    snap.sequence = seqNo;
    snap.publishTick = nowTick;
    snap.status = (uint8_t)APP_IR_STATUS_ERROR;
    seqNo = NextSeq(seqNo);
    return AppIrEdge_Step(&det, &snap, nowTick);
}

/* Polls the SAME snapshot again `ms` later. */
static AppIrEdgeOutcome Poll(uint32_t ms)
{
    nowTick += ms;
    return AppIrEdge_Step(&det, &snap, nowTick);
}

/* Runs n scans of one class, returns the first non-NONE outcome (or NONE),
 * and counts every non-NONE outcome into *count. */
static AppIrEdgeOutcome Run(int cls, unsigned n, unsigned *count)
{
    AppIrEdgeOutcome first = APP_IR_EDGE_OUT_NONE;
    unsigned i;

    for (i = 0U; i < n; i++)
    {
        AppIrEdgeOutcome o = Scan(cls);

        if (o != APP_IR_EDGE_OUT_NONE)
        {
            if (first == APP_IR_EDGE_OUT_NONE)
            {
                first = o;
            }
            if (count != 0)
            {
                (*count)++;
            }
        }
    }

    return first;
}

/* READY_SCANS scans of one class: NONE until the last, which reports READY. */
static void ReadyWith(int cls)
{
    unsigned i;

    for (i = 1U; i < APP_IR_EDGE_READY_SCANS; i++)
    {
        CHECK(Scan(cls) == APP_IR_EDGE_OUT_NONE);
    }
    CHECK(Scan(cls) == APP_IR_EDGE_OUT_READY);
}

/* Brings the detector to WAIT_OBSTACLE with clear scans. */
static void ToWaitObstacle(AppIrEdgeSide side)
{
    ResetWorld(side, 1000U, 1U);
    ReadyWith(C);
    CHECK(det.phase == (uint8_t)APP_IR_EDGE_PHASE_WAIT_OBSTACLE);
}

static void ToWaitClear(AppIrEdgeSide side)
{
    unsigned i;

    ToWaitObstacle(side);
    for (i = 1U; i < APP_IR_EDGE_SEEN_SCANS; i++)
    {
        CHECK(Scan(V) == APP_IR_EDGE_OUT_NONE);
    }
    CHECK(Scan(V) == APP_IR_EDGE_OUT_SEEN);
    CHECK(det.phase == (uint8_t)APP_IR_EDGE_PHASE_WAIT_CLEAR);
}

/* ---------------------------------------------------------------------- */
/* Tests                                                                  */
/* ---------------------------------------------------------------------- */

/* Thresholds equal the Phase 3 conversion for EVERY count of both sensors. */
static void TestThresholds(void)
{
    int side;

    for (side = 0; side < 2; side++)
    {
        AppIrEdgeDetector d;
        AppIrSensor sensor = (side == 0) ? APP_IR_SENSOR_LEFT : APP_IR_SENSOR_RIGHT;
        unsigned c;
        unsigned lastMm = 0xFFFFU;
        bool seenVisible = false;
        bool leftClear = false;

        CHECK(AppIrEdge_Start(&d, (AppIrEdgeSide)side, 0U));
        CHECK(d.clearMaxAdc < d.visibleMinAdc);

        for (c = 0U; c <= APP_IR_ADC_MAX; c++)
        {
            uint16_t mm = 0U;
            AppIrRange r = AppIrDist_Convert(sensor, (uint16_t)c, &mm);
            AppIrEdgeClass expect;

            /* The definition, straight from the conversion. */
            bool vis = (r == APP_IR_RANGE_NEAR) ||
                       ((r == APP_IR_RANGE_OK) && (mm <= APP_IR_EDGE_VISIBLE_MAX_MM));
            bool clr = (r == APP_IR_RANGE_FAR) ||
                       ((r == APP_IR_RANGE_OK) && (mm >= APP_IR_EDGE_CLEAR_MIN_MM));

            CHECK(!(vis && clr));
            expect = vis ? V : (clr ? C : A);
            CHECK(AppIrEdge_ClassifyRaw(&d, (uint16_t)c) == expect);

            /* monotone: mm never rises with the count; classes only go C -> A -> V */
            if (r == APP_IR_RANGE_OK)
            {
                CHECK(mm <= lastMm);
                lastMm = mm;
            }

            if (vis)
            {
                seenVisible = true;
            }
            if (seenVisible)
            {
                CHECK(vis);         /* once visible, every higher count is visible */
            }
            if (!clr)
            {
                leftClear = true;
            }
            if (leftClear)
            {
                CHECK(!clr);        /* once not clear, every higher count is not clear */
            }
        }

        /* the NEAR counts and 4095 are visible, 0 is clear */
        CHECK(AppIrEdge_ClassifyRaw(&d, 4095U) == V);
        CHECK(AppIrEdge_ClassifyRaw(&d, 0U) == C);
        printf("  side %d: visible >= %u, clear <= %u\n", side,
               (unsigned)d.visibleMinAdc, (unsigned)d.clearMaxAdc);
    }

    {
        AppIrEdgeDetector d;
        CHECK(!AppIrEdge_Start(&d, (AppIrEdgeSide)2, 0U));
        CHECK(d.phase == (uint8_t)APP_IR_EDGE_PHASE_FAILED);
        CHECK(!AppIrEdge_Start(0, APP_IR_EDGE_SIDE_LEFT, 0U));
    }
}

/* FAR -> visible -> clear; and the other sensor never matters. */
static void TestNormalPass(AppIrEdgeSide side)
{
    unsigned n = 0U;
    unsigned i;

    ResetWorld(side, 5000U, 100U);
    CHECK(det.phase == (uint8_t)APP_IR_EDGE_PHASE_READY);

    ReadyWith(C);
    CHECK(det.startedVisible == 0U);

    for (i = 0U; i < 20U; i++)
    {
        CHECK(Scan(C) == APP_IR_EDGE_OUT_NONE);     /* FAR before the obstacle */
    }
    CHECK(det.phase == (uint8_t)APP_IR_EDGE_PHASE_WAIT_OBSTACLE);

    for (i = 1U; i < APP_IR_EDGE_SEEN_SCANS; i++)
    {
        CHECK(Scan(V) == APP_IR_EDGE_OUT_NONE);
    }
    CHECK(Scan(V) == APP_IR_EDGE_OUT_SEEN);
    CHECK(det.seenRaw == visRaw[side]);

    for (i = 0U; i < 10U; i++)
    {
        CHECK(Scan(V) == APP_IR_EDGE_OUT_NONE);     /* still beside it */
    }
    for (i = 1U; i < APP_IR_EDGE_CLEAR_SCANS; i++)
    {
        CHECK(Scan(C) == APP_IR_EDGE_OUT_NONE);
    }
    CHECK(Scan(C) == APP_IR_EDGE_OUT_CLEARED);
    CHECK(det.clearRaw == clrRaw[side]);
    CHECK(det.phase == (uint8_t)APP_IR_EDGE_PHASE_CLEARED);

    /* exactly once: nothing more, whatever follows */
    (void)Run(C, 50U, &n);
    (void)Run(V, 50U, &n);
    CHECK(n == 0U);
    CHECK(Poll(1000U) == APP_IR_EDGE_OUT_NONE);
}

static void TestAlreadyVisible(void)
{
    unsigned n = 0U;
    unsigned i;

    ResetWorld(APP_IR_EDGE_SIDE_RIGHT, 0U, 1U);
    ReadyWith(V);
    CHECK(det.startedVisible == 1U);

    /* the readiness scans count toward the "seen" streak */
    for (i = APP_IR_EDGE_READY_SCANS + 1U; i < APP_IR_EDGE_SEEN_SCANS; i++)
    {
        CHECK(Scan(V) == APP_IR_EDGE_OUT_NONE);
    }
    CHECK(Scan(V) == APP_IR_EDGE_OUT_SEEN);
    CHECK(Run(C, APP_IR_EDGE_CLEAR_SCANS, &n) == APP_IR_EDGE_OUT_CLEARED);
    CHECK(n == 1U);
}

/* Open space forever: ready, but never seen and never cleared. */
static void TestFarForever(void)
{
    unsigned n = 0U;

    ResetWorld(APP_IR_EDGE_SIDE_LEFT, 0U, 1U);
    CHECK(Run(C, 3000U, &n) == APP_IR_EDGE_OUT_READY);
    CHECK(n == 1U);
    CHECK(det.phase == (uint8_t)APP_IR_EDGE_PHASE_WAIT_OBSTACLE);
}

/* Noise and dropouts never confirm. */
static void TestNoise(void)
{
    unsigned i;
    unsigned n = 0U;

    /* "seen": N-1 visible scans, one clear, repeated: never confirmed */
    ToWaitObstacle(APP_IR_EDGE_SIDE_LEFT);
    for (i = 0U; i < 40U; i++)
    {
        (void)Run(V, APP_IR_EDGE_SEEN_SCANS - 1U, &n);
        (void)Run(C, 1U, &n);
    }
    CHECK(n == 0U);
    /* ... an ambiguous scan breaks the streak too */
    for (i = 0U; i < 40U; i++)
    {
        (void)Run(V, APP_IR_EDGE_SEEN_SCANS - 1U, &n);
        (void)Run(A, 1U, &n);
    }
    CHECK(n == 0U);
    CHECK(det.ambiguousScans >= 40U);
    /* a single visible spike in open space */
    for (i = 0U; i < 100U; i++)
    {
        (void)Run(C, 5U, &n);
        (void)Run(V, 1U, &n);
    }
    CHECK(n == 0U);
    /* the real thing still works afterwards */
    CHECK(Run(V, APP_IR_EDGE_SEEN_SCANS, &n) == APP_IR_EDGE_OUT_SEEN);

    /* "clear": N-1 clear scans, one visible, repeated: never confirmed */
    n = 0U;
    for (i = 0U; i < 40U; i++)
    {
        (void)Run(C, APP_IR_EDGE_CLEAR_SCANS - 1U, &n);
        (void)Run(V, 1U, &n);
    }
    CHECK(n == 0U);
    for (i = 0U; i < 40U; i++)
    {
        (void)Run(C, APP_IR_EDGE_CLEAR_SCANS - 1U, &n);
        (void)Run(A, 1U, &n);
    }
    CHECK(n == 0U);
    CHECK(Run(C, APP_IR_EDGE_CLEAR_SCANS, &n) == APP_IR_EDGE_OUT_CLEARED);
}

/* One held sensor measurement is at most 3 identical scans (47.9 ms cycle,
 * 20 ms scans): it must never confirm on its own. */
static void TestHeldMeasurement(void)
{
    unsigned n = 0U;
    unsigned held;

    CHECK(((APP_IR_EDGE_SEEN_SCANS - 1U) * 20U) > 47U);

    for (held = 1U; held <= 3U; held++)
    {
        CHECK(held < APP_IR_EDGE_SEEN_SCANS);
        CHECK(held < APP_IR_EDGE_CLEAR_SCANS);

        ToWaitObstacle(APP_IR_EDGE_SIDE_RIGHT);
        (void)Run(V, held, &n);                     /* held VISIBLE value in open space */
        (void)Run(C, 10U, &n);
        CHECK(n == 0U);

        ToWaitClear(APP_IR_EDGE_SIDE_RIGHT);
        (void)Run(C, held, &n);                     /* held CLEAR value mid-obstacle */
        (void)Run(V, 10U, &n);
        CHECK(n == 0U);
    }
}

/* NEAR is visible and never clear. */
static void TestNear(void)
{
    unsigned n = 0U;

    ToWaitObstacle(APP_IR_EDGE_SIDE_LEFT);
    {
        unsigned i;
        AppIrEdgeOutcome o = APP_IR_EDGE_OUT_NONE;

        for (i = 0U; i < APP_IR_EDGE_SEEN_SCANS; i++)
        {
            o = ScanRaw(4095U);
        }
        CHECK(o == APP_IR_EDGE_OUT_SEEN);
    }
    for (n = 0U; n < 500U; n++)
    {
        CHECK(ScanRaw(4095U) == APP_IR_EDGE_OUT_NONE);
        CHECK(ScanRaw(3600U) == APP_IR_EDGE_OUT_NONE);      /* the NEAR region, not folded back */
    }
    CHECK(det.phase == (uint8_t)APP_IR_EDGE_PHASE_WAIT_CLEAR);
}

/* ERROR / stale / FAULT / no data. */
static void TestUnusable(void)
{
    unsigned n = 0U;
    unsigned i;

    /* ERROR publications reset the streak and never count as clear */
    ToWaitClear(APP_IR_EDGE_SIDE_LEFT);
    for (i = 0U; i < 5U; i++)
    {
        (void)Run(C, APP_IR_EDGE_CLEAR_SCANS - 1U, &n);
        CHECK(ScanError() == APP_IR_EDGE_OUT_NONE);
    }
    CHECK(n == 0U);
    CHECK(det.unusableScans == 5U);
    CHECK(Run(C, APP_IR_EDGE_CLEAR_SCANS, &n) == APP_IR_EDGE_OUT_CLEARED);

    /* a stream of ERRORs in WAIT_CLEAR fails with DATA_LOST, never clears */
    ToWaitClear(APP_IR_EDGE_SIDE_RIGHT);
    {
        AppIrEdgeOutcome o = APP_IR_EDGE_OUT_NONE;

        for (i = 0U; (i < 50U) && (o == APP_IR_EDGE_OUT_NONE); i++)
        {
            o = ScanError();
        }
        CHECK(o == APP_IR_EDGE_OUT_FAILED);
        CHECK(det.failReason == (uint8_t)APP_IR_EDGE_FAIL_DATA_LOST);
        CHECK(det.phase == (uint8_t)APP_IR_EDGE_PHASE_FAILED);
        CHECK(Scan(C) == APP_IR_EDGE_OUT_NONE);             /* once only */
        CHECK(Poll(5000U) == APP_IR_EDGE_OUT_NONE);
    }

    /* FAULT fails at once in every phase, whatever its age */
    {
        int phase;

        for (phase = 0; phase < 3; phase++)
        {
            if (phase == 0)
            {
                ResetWorld(APP_IR_EDGE_SIDE_LEFT, 0U, 1U);
            }
            else if (phase == 1)
            {
                ToWaitObstacle(APP_IR_EDGE_SIDE_LEFT);
            }
            else
            {
                ToWaitClear(APP_IR_EDGE_SIDE_LEFT);
            }

            snap.status = (uint8_t)APP_IR_STATUS_FAULT;
            snap.sequence = seqNo;
            snap.publishTick = nowTick - 100000U;           /* ancient */
            CHECK(AppIrEdge_Step(&det, &snap, nowTick) == APP_IR_EDGE_OUT_FAILED);
            CHECK(det.failReason == (uint8_t)APP_IR_EDGE_FAIL_FAULT);
            CHECK(AppIrEdge_Step(&det, &snap, nowTick + 10U) == APP_IR_EDGE_OUT_NONE);
        }
    }

    /* a stale VALID publication is unusable and never counts */
    ResetWorld(APP_IR_EDGE_SIDE_LEFT, 1000U, 1U);
    snap.status = (uint8_t)APP_IR_STATUS_VALID;
    snap.sequence = 1U;
    snap.publishTick = 1000U;
    FillRaw(APP_IR_EDGE_SIDE_LEFT, C);
    CHECK(AppIrEdge_Step(&det, &snap, 1000U + APP_IR_STALE_TICKS + 1U) == APP_IR_EDGE_OUT_NONE);
    CHECK(det.unusableScans == 1U);
    CHECK(det.readyCount == 0U);
}

/* Readiness: motors-stopped period with exact timeout boundaries. */
static void TestReadiness(void)
{
    /* nothing ever published: fails strictly after the timeout */
    ResetWorld(APP_IR_EDGE_SIDE_LEFT, 40000U, 1U);
    CHECK(AppIrEdge_Step(&det, &snap, 40000U) == APP_IR_EDGE_OUT_NONE);
    CHECK(AppIrEdge_Step(&det, &snap, 40000U + APP_IR_EDGE_READY_TIMEOUT_TICKS) == APP_IR_EDGE_OUT_NONE);
    CHECK(AppIrEdge_Step(&det, &snap, 40000U + APP_IR_EDGE_READY_TIMEOUT_TICKS + 1U) == APP_IR_EDGE_OUT_FAILED);
    CHECK(det.failReason == (uint8_t)APP_IR_EDGE_FAIL_READY_TIMEOUT);

    /* ERROR scans never make it ready; the timeout still applies */
    ResetWorld(APP_IR_EDGE_SIDE_RIGHT, 0U, 1U);
    {
        unsigned i;
        AppIrEdgeOutcome o = APP_IR_EDGE_OUT_NONE;

        for (i = 0U; (i < 40U) && (o == APP_IR_EDGE_OUT_NONE); i++)
        {
            o = ScanError();
        }
        CHECK(o == APP_IR_EDGE_OUT_FAILED);
        CHECK(det.failReason == (uint8_t)APP_IR_EDGE_FAIL_READY_TIMEOUT);
        CHECK(nowTick <= APP_IR_EDGE_READY_TIMEOUT_TICKS + 20U);
    }

    /* an ERROR between two good scans restarts the readiness count */
    ResetWorld(APP_IR_EDGE_SIDE_LEFT, 0U, 1U);
    {
        unsigned i;

        for (i = 0U; i + 1U < APP_IR_EDGE_READY_SCANS; i++)
        {
            CHECK(Scan(C) == APP_IR_EDGE_OUT_NONE);
        }
        CHECK(ScanError() == APP_IR_EDGE_OUT_NONE);
        for (i = 0U; i + 1U < APP_IR_EDGE_READY_SCANS; i++)
        {
            CHECK(Scan(C) == APP_IR_EDGE_OUT_NONE);
        }
        CHECK(Scan(C) == APP_IR_EDGE_OUT_READY);
    }

    /* READY with the last good scan exactly at the boundary */
    ResetWorld(APP_IR_EDGE_SIDE_LEFT, 0U, 1U);
    nowTick = APP_IR_EDGE_READY_TIMEOUT_TICKS - 20U * (APP_IR_EDGE_READY_SCANS - 1U) - 20U;
    {
        unsigned i;
        AppIrEdgeOutcome o = APP_IR_EDGE_OUT_NONE;

        for (i = 0U; i < APP_IR_EDGE_READY_SCANS; i++)
        {
            o = Scan(C);
        }
        CHECK(o == APP_IR_EDGE_OUT_READY);
    }

    /* the unavailable limit: exactly at the limit is fine, one over fails */
    ToWaitObstacle(APP_IR_EDGE_SIDE_LEFT);
    {
        uint32_t last = nowTick;

        CHECK(AppIrEdge_Step(&det, &snap, last + APP_IR_EDGE_UNAVAILABLE_LIMIT_TICKS) == APP_IR_EDGE_OUT_NONE);
        CHECK(AppIrEdge_Step(&det, &snap, last + APP_IR_EDGE_UNAVAILABLE_LIMIT_TICKS + 1U) == APP_IR_EDGE_OUT_FAILED);
        CHECK(det.failReason == (uint8_t)APP_IR_EDGE_FAIL_DATA_LOST);
    }
}

/* Each publication once; gaps; sequence 0; wrap; ticks wrap; old scans. */
static void TestSequences(void)
{
    unsigned n = 0U;
    unsigned i;

    /* polling the same publication many times counts it once */
    ToWaitObstacle(APP_IR_EDGE_SIDE_LEFT);
    for (i = 0U; i < APP_IR_EDGE_SEEN_SCANS - 1U; i++)
    {
        (void)Scan(V);
        CHECK(Poll(5U) == APP_IR_EDGE_OUT_NONE);
        CHECK(Poll(5U) == APP_IR_EDGE_OUT_NONE);
        CHECK(Poll(1U) == APP_IR_EDGE_OUT_NONE);
    }
    CHECK(det.streak == APP_IR_EDGE_SEEN_SCANS - 1U);
    CHECK(Scan(V) == APP_IR_EDGE_OUT_SEEN);

    /* a missed scan (gap) resets the streak */
    ToWaitObstacle(APP_IR_EDGE_SIDE_RIGHT);
    (void)Run(V, APP_IR_EDGE_SEEN_SCANS - 1U, &n);
    seqNo = NextSeq(seqNo);                         /* one publication lost */
    CHECK(Scan(V) == APP_IR_EDGE_OUT_NONE);
    CHECK(det.missedScans == 1U);
    CHECK(det.streak == 1U);
    CHECK(Run(V, APP_IR_EDGE_SEEN_SCANS - 1U, &n) == APP_IR_EDGE_OUT_SEEN);

    /* a gap during readiness restarts it */
    ResetWorld(APP_IR_EDGE_SIDE_LEFT, 0U, 1U);
    {
        unsigned k;

        for (k = 0U; k + 1U < APP_IR_EDGE_READY_SCANS; k++)
        {
            CHECK(Scan(C) == APP_IR_EDGE_OUT_NONE);
        }
        seqNo = NextSeq(seqNo);
        CHECK(Scan(C) == APP_IR_EDGE_OUT_NONE);
        CHECK(det.readyCount == 1U);
    }

    /* sequence 0 means "never published": never consumed */
    ResetWorld(APP_IR_EDGE_SIDE_LEFT, 0U, 1U);
    snap.sequence = 0U;
    snap.publishTick = 10U;
    snap.status = (uint8_t)APP_IR_STATUS_VALID;
    FillRaw(APP_IR_EDGE_SIDE_LEFT, C);
    CHECK(AppIrEdge_Step(&det, &snap, 20U) == APP_IR_EDGE_OUT_NONE);
    CHECK(det.readyCount == 0U);
    CHECK(det.haveSequence == 0U);

    /* sequence wrap 0xFFFFFFFE -> 0xFFFFFFFF -> 1 (0 skipped) is continuous */
    ResetWorld(APP_IR_EDGE_SIDE_RIGHT, 100U, 0xFFFFFFFEU);
    {
        unsigned k;

        /* the readiness run crosses 0xFFFFFFFF -> 1 whatever READY_SCANS is */
        for (k = 0U; k < APP_IR_EDGE_READY_SCANS; k++)
        {
            (void)Scan(C);
        }
        CHECK(det.phase == (uint8_t)APP_IR_EDGE_PHASE_WAIT_OBSTACLE);
        (void)Scan(C);
        (void)Scan(C);
        CHECK(snap.sequence >= 1U);
    }
    CHECK(det.missedScans == 0U);
    CHECK(Run(V, APP_IR_EDGE_SEEN_SCANS, &n) == APP_IR_EDGE_OUT_SEEN);
    CHECK(det.missedScans == 0U);

    /* a jump over the wrap (0xFFFFFFFF -> 2) is a gap */
    ResetWorld(APP_IR_EDGE_SIDE_RIGHT, 100U, 0xFFFFFFFFU);
    (void)Scan(C);
    seqNo = 2U;
    (void)Scan(C);
    CHECK(det.missedScans == 1U);

    /* the tick counter wraps in the middle of a pass */
    ResetWorld(APP_IR_EDGE_SIDE_LEFT, 0xFFFFFF00U, 1U);
    n = 0U;
    CHECK(Run(C, 5U, &n) == APP_IR_EDGE_OUT_READY);
    CHECK(Run(V, APP_IR_EDGE_SEEN_SCANS, &n) == APP_IR_EDGE_OUT_SEEN);
    CHECK(Run(C, APP_IR_EDGE_CLEAR_SCANS, &n) == APP_IR_EDGE_OUT_CLEARED);
    CHECK(n == 3U);
    CHECK(nowTick < 0xFFFFFF00U);                   /* it really wrapped */

    /* publications made before the command started are ignored */
    ResetWorld(APP_IR_EDGE_SIDE_LEFT, 5000U, 1U);
    snap.status = (uint8_t)APP_IR_STATUS_VALID;
    snap.sequence = 777U;
    snap.publishTick = 4990U;                       /* 10 ms BEFORE the start */
    FillRaw(APP_IR_EDGE_SIDE_LEFT, V);
    CHECK(AppIrEdge_Step(&det, &snap, 5001U) == APP_IR_EDGE_OUT_NONE);
    CHECK(det.readyCount == 0U);
    CHECK(det.missedScans == 0U);
    seqNo = 778U;                                   /* the successor counts and is no gap */
    nowTick = 5001U;
    CHECK(Scan(C) == APP_IR_EDGE_OUT_NONE);
    CHECK(det.missedScans == 0U);
    CHECK(det.readyCount == 1U);
}

/* ---------------------------------------------------------------------- */
/* Randomised differential run against an independently written model     */
/* ---------------------------------------------------------------------- */

#define MAXH 4096

typedef struct
{
    uint32_t startTick;
    uint32_t lastUsable;
    uint32_t lastSeq;
    int haveSeq;
    int phase;                  /* 0 ready 1 wait-obstacle 2 wait-clear 3 done 4 failed */
    int hist[MAXH];             /* class: 0 V 1 C 2 A, -1 break */
    int nh;
} Model;

static uint32_t rngState = 12345U;
static uint32_t Rnd(void)
{
    rngState = rngState * 1664525U + 1013904223U;
    return rngState >> 8;
}

static int ModelClass(AppIrSensor sensor, uint16_t raw)
{
    uint16_t mm = 0U;
    AppIrRange r = AppIrDist_Convert(sensor, raw, &mm);

    if ((r == APP_IR_RANGE_NEAR) || ((r == APP_IR_RANGE_OK) && (mm <= APP_IR_EDGE_VISIBLE_MAX_MM)))
    {
        return 0;
    }
    if ((r == APP_IR_RANGE_FAR) || ((r == APP_IR_RANGE_OK) && (mm >= APP_IR_EDGE_CLEAR_MIN_MM)))
    {
        return 1;
    }
    return 2;
}

static int TrailingRun(const Model *m, int cls)
{
    int k = 0;
    int i;

    for (i = m->nh - 1; i >= 0; i--)
    {
        if (m->hist[i] != cls)
        {
            break;
        }
        k++;
    }
    return k;
}

static int TrailingUsable(const Model *m)
{
    int k = 0;
    int i;

    for (i = m->nh - 1; (i >= 0) && (m->hist[i] >= 0); i--)
    {
        k++;
    }
    return k;
}

static void ModelPush(Model *m, int v)
{
    if (m->nh < MAXH)
    {
        m->hist[m->nh++] = v;
    }
}

/* returns 0 none,1 ready,2 seen,3 cleared,4 failed */
static int ModelStep(Model *m, AppIrSensor sensor, const AppIrSnapshot *s, uint32_t now)
{
    int newPub;

    if ((m->phase >= 3))
    {
        return 0;
    }
    if (s->status == (uint8_t)APP_IR_STATUS_FAULT)
    {
        m->phase = 4;
        return 4;
    }

    newPub = (s->sequence != 0U) && (!m->haveSeq || (s->sequence != m->lastSeq));

    if (newPub)
    {
        int before = ((int32_t)(s->publishTick - m->startTick) < 0);
        uint32_t expect = m->lastSeq + 1U;

        if (expect == 0U)
        {
            expect = 1U;
        }

        if (!before && m->haveSeq && (s->sequence != expect))
        {
            ModelPush(m, -1);
        }

        m->lastSeq = s->sequence;
        m->haveSeq = 1;

        if (!before)
        {
            int usable = (s->status == (uint8_t)APP_IR_STATUS_VALID) &&
                         ((uint32_t)(now - s->publishTick) <= APP_IR_STALE_TICKS);

            if (!usable)
            {
                ModelPush(m, -1);
            }
            else
            {
                int cls = ModelClass(sensor,
                    (sensor == APP_IR_SENSOR_RIGHT) ? s->rightRaw : s->leftRaw);

                m->lastUsable = now;
                ModelPush(m, cls);

                if (m->phase == 0)
                {
                    if (TrailingUsable(m) >= (int)APP_IR_EDGE_READY_SCANS)
                    {
                        m->phase = 1;
                        return 1;
                    }
                }
                else if (m->phase == 1)
                {
                    if (TrailingRun(m, 0) >= (int)APP_IR_EDGE_SEEN_SCANS)
                    {
                        m->phase = 2;
                        m->nh = 0;
                        return 2;
                    }
                }
                else if (m->phase == 2)
                {
                    if (TrailingRun(m, 1) >= (int)APP_IR_EDGE_CLEAR_SCANS)
                    {
                        m->phase = 3;
                        return 3;
                    }
                }
            }
        }
    }

    if (m->phase == 0)
    {
        if ((uint32_t)(now - m->startTick) > APP_IR_EDGE_READY_TIMEOUT_TICKS)
        {
            m->phase = 4;
            return 4;
        }
    }
    else if ((uint32_t)(now - m->lastUsable) > APP_IR_EDGE_UNAVAILABLE_LIMIT_TICKS)
    {
        m->phase = 4;
        return 4;
    }
    return 0;
}

static void TestDifferential(void)
{
    unsigned run;
    unsigned divergences = 0U;
    unsigned reached[5] = {0, 0, 0, 0, 0};

    for (run = 0U; run < 6000U; run++)
    {
        AppIrEdgeSide side = (AppIrEdgeSide)(Rnd() & 1U);
        AppIrSensor sensor = (side == APP_IR_EDGE_SIDE_LEFT) ? APP_IR_SENSOR_LEFT : APP_IR_SENSOR_RIGHT;
        AppIrEdgeDetector d;
        AppIrSnapshot s;
        Model m;
        uint32_t tick = (Rnd() & 1U) ? 0xFFFFFE00U : (Rnd() & 0xFFFFU);
        uint32_t seq = (Rnd() & 1U) ? 0xFFFFFFF0U : (Rnd() & 0xFFFFU) + 1U;
        unsigned step;
        int mode = 0;               /* what the "world" looks like: 0 clear 1 visible 2 amb 3 near */
        unsigned modeLeft = 1U + (Rnd() % 12U);
        int dead = 0;
        unsigned errRate = (Rnd() % 3U == 0U) ? 8U : 0U;
        unsigned dropRate = (Rnd() % 3U == 0U) ? 6U : 0U;

        memset(&s, 0, sizeof(s));
        memset(&m, 0, sizeof(m));
        (void)AppIrEdge_Start(&d, side, tick);
        m.startTick = tick;
        m.lastUsable = tick;

        for (step = 0U; step < 500U; step++)
        {
            int out;
            int expect;
            uint16_t raw;

            tick += 10U;

            if ((step & 1U) == 0U)
            {
                /* a new scan every second poll (20 ms), sometimes dropped / errored / late */
                seq = NextSeq(seq);

                if (modeLeft == 0U)
                {
                    mode = (int)(Rnd() % 4U);
                    modeLeft = 1U + (Rnd() % 10U);
                }
                modeLeft--;

                if ((dropRate != 0U) && (Rnd() % dropRate == 0U))
                {
                    seq = NextSeq(seq);                 /* one publication lost */
                }

                switch (mode)
                {
                    case 0: raw = clrRaw[side]; break;
                    case 1: raw = visRaw[side]; break;
                    case 2: raw = ambRaw[side]; break;
                    default: raw = (uint16_t)(3300U + (Rnd() % 796U)); break;   /* NEAR */
                }
                if ((Rnd() % 7U) == 0U)
                {
                    raw = (uint16_t)(Rnd() % 4096U);    /* any count at all */
                }

                s.sequence = seq;
                s.publishTick = tick - (Rnd() % 11U);
                s.status = (uint8_t)APP_IR_STATUS_VALID;

                if ((errRate != 0U) && (Rnd() % errRate == 0U))
                {
                    s.status = (uint8_t)APP_IR_STATUS_ERROR;
                }
                if (Rnd() % 300U == 0U)
                {
                    s.publishTick = tick - 150U;        /* a stale one */
                }
                if (Rnd() % 4000U == 0U)
                {
                    s.status = (uint8_t)APP_IR_STATUS_FAULT;
                }
                if (Rnd() % 50U == 0U)
                {
                    s.publishTick = (uint32_t)(m.startTick - 5U);   /* before the start */
                }
                s.leftRaw = (side == APP_IR_EDGE_SIDE_LEFT) ? raw : (uint16_t)(Rnd() % 4096U);
                s.rightRaw = (side == APP_IR_EDGE_SIDE_LEFT) ? (uint16_t)(Rnd() % 4096U) : raw;
            }

            out = (int)AppIrEdge_Step(&d, &s, tick);
            expect = ModelStep(&m, sensor, &s, tick);

            if (out != expect)
            {
                divergences++;
            }
            if (out != 0)
            {
                reached[out]++;
            }
            if (d.phase != (uint8_t)m.phase)
            {
                divergences++;
            }
            if (out == 4)
            {
                dead = 1;
            }
            if (dead || (d.phase == (uint8_t)APP_IR_EDGE_PHASE_CLEARED))
            {
                break;
            }
        }
    }

    CHECK(divergences == 0U);
    CHECK(reached[1] > 100U);                       /* the run mix reaches every outcome */
    CHECK(reached[2] > 100U);
    CHECK(reached[3] > 20U);
    CHECK(reached[4] > 100U);
    printf("  differential: ready %u seen %u cleared %u failed %u divergences %u\n",
           reached[1], reached[2], reached[3], reached[4], divergences);
}

int main(void)
{
    ComputeSamples();

    TestThresholds();
    TestNormalPass(APP_IR_EDGE_SIDE_LEFT);
    TestNormalPass(APP_IR_EDGE_SIDE_RIGHT);
    TestAlreadyVisible();
    TestFarForever();
    TestNoise();
    TestHeldMeasurement();
    TestNear();
    TestUnusable();
    TestReadiness();
    TestSequences();
    TestDifferential();

    printf("%u checks, %u failures\n", checks, failures);
    return failures == 0U ? 0 : 1;
}
