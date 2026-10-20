/* Host tool (not a pass/fail test): replays the RECORDED calibration samples
 * through the REAL firmware filter (Core/Src/app_ir_dist.c) and conversion and
 * prints, per sensor and calibration distance, the error of the firmware's
 * millimetre output against the nominal distance. It answers "does the
 * implementation behave as the calibration data predicts?", in-sample only.
 *
 * The recorded samples are the `IRRAW` lines of the calibration runs, about
 * 200 ms apart (one scan in ten), so the filter window slides over samples
 * that are 200 ms apart, not 20 ms: a window of 9 here spans 1.8 s. That is
 * the best available replay, not a model of the 50 Hz stream, and it is
 * in-sample (the same data the curves were fitted to). Nominal distances are
 * ruler/target placements (+-5 mm).
 *
 * After the per-distance table it prints, for the 10 cm and 40 cm endpoints,
 * how the guard band (APP_IR_DIST_GUARD_MM) changes the classification
 * compared with a strict 100..400 mm cutoff, and the ADC cutoffs with their
 * fitted distances.
 *
 * Input: tests/ir/calibration/ir_left_calibration.csv and
 *        tests/ir/calibration/ir_right_calibration_clean.csv.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -Werror -DAPP_IR_ENABLE=1 -ICore/Inc \
 *       -o tests/ir/app_ir_replay.exe tests/ir/app_ir_replay.c \
 *       Core/Src/app_ir_dist.c
 *   ./tests/ir/app_ir_replay.exe tests/ir/calibration/ir_left_calibration.csv \
 *                                tests/ir/calibration/ir_right_calibration_clean.csv
 */

#include "app_ir.h"
#include "app_ir_dist.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SAMPLES 4096

typedef struct
{
    int distanceCm;
    uint16_t adc[MAX_SAMPLES];
    int n;
} Group;

/* Report curves in millimetres (tool-local copy, for the cutoff table only). */
static double RefMm(int sensor, int adc)
{
    if (sensor == 0)
    {
        return (31785.22973379 / ((double)adc + 26.48617844) - 0.29510522) * 10.0;
    }
    return (32831.45 / ((double)adc - 94.6824) - 1.02464) * 10.0;
}

/* Lowest / highest count whose curve value lies in [loMm, hiMm]. */
static void Bounds(int sensor, double loMm, double hiMm, int *lo, int *hi)
{
    int adc;

    *lo = 99999;
    *hi = -1;
    for (adc = (sensor == 1) ? 96 : 0; adc <= 4095; adc++)
    {
        double mm = RefMm(sensor, adc);

        if ((mm >= loMm) && (mm <= hiMm))
        {
            if (adc < *lo) { *lo = adc; }
            if (adc > *hi) { *hi = adc; }
        }
    }
}

static int CmpInt(const void *a, const void *b)
{
    int x = *(const int *)a;
    int y = *(const int *)b;

    return (x > y) - (x < y);
}

/* Splits the first 12 comma-separated fields in place. The 13th (raw_line) is
 * quoted and contains commas; it is not needed. */
static int SplitFields(char *line, char **fields, int maxFields)
{
    int n = 0;
    char *p = line;

    while ((n < maxFields) && (p != NULL))
    {
        fields[n++] = p;
        p = strchr(p, ',');
        if (p != NULL) { *p++ = '\0'; }
    }
    return n;
}

static int strictLo;
static int strictHi;

static int Replay(const char *path, int sensor, const char *name)
{
    FILE *f = fopen(path, "r");
    static Group groups[64];
    int ng = 0;
    char line[1024];
    int g;

    if (f == NULL)
    {
        printf("cannot open %s\n", path);
        return 1;
    }

    (void)fgets(line, sizeof(line), f);                 /* header */
    while (fgets(line, sizeof(line), f) != NULL)
    {
        char *fields[13];
        int cm;
        int adc;
        int i;

        if (SplitFields(line, fields, 13) < 12) { continue; }
        if (strcmp(fields[11], "1") != 0) { continue; }  /* parse_ok */
        cm = atoi(fields[3]);
        adc = atoi(sensor == 0 ? fields[8] : fields[9]);

        for (i = 0; i < ng; i++) { if (groups[i].distanceCm == cm) { break; } }
        if (i == ng) { groups[ng].distanceCm = cm; groups[ng].n = 0; ng++; }
        if (groups[i].n < MAX_SAMPLES) { groups[i].adc[groups[i].n++] = (uint16_t)adc; }
    }
    fclose(f);

    {
        int gLo, gHi;
        const double guard = (double)APP_IR_DIST_GUARD_MM;

        Bounds(sensor, (double)APP_IR_DIST_MIN_MM - guard, (double)APP_IR_DIST_MAX_MM + guard, &gLo, &gHi);
        Bounds(sensor, (double)APP_IR_DIST_MIN_MM, (double)APP_IR_DIST_MAX_MM, &strictLo, &strictHi);
        printf("== %s cutoffs: guard band counts %d..%d (%.2f mm .. %.2f mm), strict counts %d..%d (%.2f mm .. %.2f mm); "
               "firmware uses %u..%u\n", name,
               gLo, gHi, RefMm(sensor, gLo), RefMm(sensor, gHi),
               strictLo, strictHi, RefMm(sensor, strictLo), RefMm(sensor, strictHi),
               (unsigned)(sensor == 0 ? APP_IR_LEFT_ADC_FAR_LIMIT : APP_IR_RIGHT_ADC_FAR_LIMIT),
               (unsigned)(sensor == 0 ? APP_IR_LEFT_ADC_NEAR_LIMIT : APP_IR_RIGHT_ADC_NEAR_LIMIT));
    }
    printf("== %s (filter: window %u, rank %u)\n", name,
           (unsigned)APP_IR_FILTER_WINDOW, (unsigned)APP_IR_FILTER_RANK);
    printf("  dist  samples outputs   OK  NEAR  FAR   median err   p95 |err|   max |err|  (mm)\n");

    for (g = 0; g < ng; g++)
    {
        AppIrFilter filter;
        int errs[MAX_SAMPLES];
        int abserrs[MAX_SAMPLES];
        int ne = 0;
        int outputs = 0;
        int near = 0;
        int far = 0;
        int strictOk = 0;
        int mmList[MAX_SAMPLES];
        int nmm = 0;
        int i;

        AppIrFilter_Reset(&filter);
        for (i = 0; i < groups[g].n; i++)
        {
            uint16_t out;
            uint16_t mm;
            AppIrRange range;

            AppIrFilter_Push(&filter, groups[g].adc[i]);
            if (!AppIrFilter_Output(&filter, &out)) { continue; }

            outputs++;
            if (((int)out >= strictLo) && ((int)out <= strictHi)) { strictOk++; }
            range = AppIrDist_Convert((AppIrSensor)sensor, out, &mm);
            if (range == APP_IR_RANGE_NEAR) { near++; }
            else if (range == APP_IR_RANGE_FAR) { far++; }
            else if (range == APP_IR_RANGE_OK)
            {
                errs[ne] = (int)mm - groups[g].distanceCm * 10;
                abserrs[ne] = abs(errs[ne]);
                mmList[nmm++] = (int)mm;
                ne++;
            }
        }

        if ((groups[g].distanceCm == 10) || (groups[g].distanceCm == 40))
        {
            int med = 0;

            if (nmm > 0)
            {
                qsort(mmList, (size_t)nmm, sizeof(int), CmpInt);
                med = mmList[nmm / 2];
            }
            printf("  ENDPOINT %s %2d cm: guard band OK %3d%%, NEAR/FAR %3d%% (strict cutoffs: OK %3d%%), "
                   "median reported %d mm\n", name, groups[g].distanceCm,
                   outputs ? (100 * ne) / outputs : 0, outputs ? (100 * (near + far)) / outputs : 0,
                   outputs ? (100 * strictOk) / outputs : 0, med);
        }

        if (ne > 0)
        {
            qsort(errs, (size_t)ne, sizeof(int), CmpInt);
            qsort(abserrs, (size_t)ne, sizeof(int), CmpInt);
            printf("  %4d  %7d %7d %4d %5d %4d   %+9d   %9d   %9d\n",
                   groups[g].distanceCm, groups[g].n, outputs, ne, near, far,
                   errs[ne / 2], abserrs[(ne * 95) / 100 < ne ? (ne * 95) / 100 : ne - 1],
                   abserrs[ne - 1]);
        }
        else
        {
            printf("  %4d  %7d %7d %4d %5d %4d   (no in-domain output)\n",
                   groups[g].distanceCm, groups[g].n, outputs, ne, near, far);
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    int rc = 0;

    if (argc != 3)
    {
        printf("usage: %s <left csv> <right csv>\n", argv[0]);
        return 2;
    }

    rc |= Replay(argv[1], 0, "LEFT");
    rc |= Replay(argv[2], 1, "RIGHT");
    return rc;
}
