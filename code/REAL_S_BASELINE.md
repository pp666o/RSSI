# Real `s-only` RSSI Baseline

This baseline is for the robot project, not for IMUWiFine trajectory `s`.

## Expected CSV format

One row = one already-aggregated reference sample or one online observation window.

```csv
s,timestamp,beacon_1_mean,beacon_1_std,beacon_2_mean,beacon_2_std,...,beacon_10_mean,beacon_10_std
```

Notes:

- `s` is the true global path progress defined by your own centerline.
- `timestamp` is optional.
- Missing packets can be stored as `-999` in the raw logger output.
- The script converts `-999` into an internal weak RSSI value like `-95`.

## Scheme A

Pure RSSI fingerprint + KNN:

```powershell
.\.venv\Scripts\python.exe .\robot_s_baseline.py `
  --train .\your_train.csv `
  --test .\your_test.csv `
  --k 3 `
  --max-jump-s 0 `
  --smooth-alpha 0 `
  --out baseline_outputs\robot_predictions_A.csv `
  --map-out baseline_outputs\robot_map_A.csv
```

## Scheme B

RSSI fingerprint + local path constraint + optional smoothing:

```powershell
.\.venv\Scripts\python.exe .\robot_s_baseline.py `
  --train .\your_train.csv `
  --test .\your_test.csv `
  --k 3 `
  --max-jump-s 5 `
  --smooth-alpha 0.4 `
  --out baseline_outputs\robot_predictions_B.csv `
  --map-out baseline_outputs\robot_map_B.csv
```

## Output files

- `*_map_*.csv`: fingerprint library indexed by true `s`
- `*_predictions_*.csv`: per-row predicted `s`
- `*_by_file.csv`: per-trajectory `s_error` summary

## Included demo

The repo includes a tiny demo dataset:

- `examples/robot_s_demo/train.csv`
- `examples/robot_s_demo/test.csv`

These files are only for checking that the pipeline runs end-to-end.
