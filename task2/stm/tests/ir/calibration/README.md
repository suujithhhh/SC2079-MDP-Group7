# IR sensor calibration data (measurement data only)

Raw and processed measurements for the two Sharp GP2Y0A21YK sensors, collected on
2026-10-12 from the firmware's `IRRAW,S:<seq>,L:<adc>,R:<adc>,F:<hex>` bench line
(about 5 lines per second, one raw 12-bit ADC scan per line). The files are
**measurement data, not implementation instructions**: the firmware's curves are
typed into `Core/Src/app_ir_dist.c` and checked against these reports by
`tests/ir/app_ir_dist_test.c`, never read from here at run time.

| Sensor | Pin / channel | Files |
|---|---|---|
| LEFT | PC0 / ADC1 channel 10 / DMA element 0 | `ir_left_calibration.csv` (all samples), `ir_left_calibration_runs.csv` (one row per run), `ir_left_calibration_analysis.csv` (per-distance statistics), `ir_left_calibration_report.txt` (fit) |
| RIGHT | PC1 / ADC1 channel 11 / DMA element 1 | `ir_right_calibration_clean.csv` (all samples; one run with a wrong 25 cm label removed), `ir_right_calibration_runs_clean.csv`, `ir_right_calibration_summary.csv`, `ir_right_fit_report.txt` |

## What was measured (user-confirmed 2026-10-12)

- Every `distance_cm` is the distance from the **sensor face**, along the beam, to the
  target. Both sensors face sideways. Each sensor face is 45 mm inboard of the
  corresponding tyre's outer lateral reference, along the beam; the firmware does not
  apply that offset (`PROTOCOL.md` §2, §4.5).
- LEFT: 15 distances (10-20 cm in 1 cm steps, then 25, 30, 35, 40 cm), 100 samples each.
  RIGHT: 20 distances (5-20 cm in 1 cm steps, then 25, 30, 35, 40 cm), 100 samples each;
  5-9 cm were measured but are excluded from the fit.
- Target placement uncertainty: take +-5 mm unless the operator supplies a better value.
  The firmware's guard band at the 10 and 40 cm ends (`APP_IR_DIST_GUARD_MM`) is this
  value: the 40 cm RIGHT median (ADC 893) is 401.0 mm on its own curve.

## Fitted curves (provisional, in-sample)

```
LEFT : distance_cm = 31785.22973379 / (ADC_L + 26.48617844) - 0.29510522
RIGHT: distance_cm = 32831.45       / (ADC_R - 94.6824)     - 1.02464
```

Fitted to the per-distance medians over 10-40 cm only. Reported fit quality: LEFT R2
0.99929, RMSE 2.32 mm, largest median residual 6.87 mm (40 cm); RIGHT R2 0.99981, RMSE
1.19 mm, largest median residual 2.81 mm. The two curves are not interchangeable.

## Limitations

- One run per distance, no repeats; no temperature, supply voltage, target material or
  lighting recorded.
- Nothing below 5 cm (RIGHT) or 10 cm (LEFT). The sensor output is not monotonic below
  about 6 cm, so a very close object can read like a valid distance: the firmware
  documents a blind region below 10 cm and does not detect it.
- The recorded samples are 200 ms apart (one scan in ten), so they say nothing about
  the correlation of consecutive 20 ms scans.
- LEFT is 2-3 times noisier than RIGHT (one-sided upward spikes in 25-37 % of samples
  against 10-21 %).

## Reproducing the firmware replay

From `RTOSV/`:

```
gcc -std=c11 -Wall -Wextra -Werror -DAPP_IR_ENABLE=1 -ICore/Inc \
    -o tests/ir/app_ir_replay.exe tests/ir/app_ir_replay.c Core/Src/app_ir_dist.c
./tests/ir/app_ir_replay.exe tests/ir/calibration/ir_left_calibration.csv \
                             tests/ir/calibration/ir_right_calibration_clean.csv
```

It feeds the recorded samples through the real firmware filter and conversion and prints
the error against the nominal distance (in-sample; see the header of `app_ir_replay.c`).
