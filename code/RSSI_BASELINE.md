# RSSI Fingerprint Baseline

This baseline implements a traditional, non-neural path-tracking pipeline:

1. Load labelled CSV trajectories.
2. Infer AP/RSSI columns by excluding `timestamp`, IMU columns, and `x/y/z`.
3. Smooth RSSI per trajectory with a rolling mean.
4. Build a fingerprint map by grouping nearby labelled reference points.
5. Match test RSSI vectors using weighted k-nearest-neighbour distance.
6. Apply a simple path continuity constraint with `--max-jump-m`.
7. Save predictions and, when test labels exist, report path-distance error.

Recommended project mode:

- Use `--target-mode s` for robot path tracking.
- Treat `s` as the primary output sent to the robot controller.
- Keep `x/y/z` only as optional debug fields, not as the main control target.

Example smoke test:

```powershell
.\.venv\Scripts\python.exe .\rssi_fingerprint_baseline.py `
  --target-mode s `
  --train "..\dataset\datasets--issai--IMUWiFine\snapshots\0d43350cb8ce12e10ff605c6c89b6062a0d58d7e\IMU_DATA\raw_IUMIWiFi\oppo\train\csv_files_processed\DATA_02112020_160427.csv" `
  --test "..\dataset\datasets--issai--IMUWiFine\snapshots\0d43350cb8ce12e10ff605c6c89b6062a0d58d7e\IMU_DATA\raw_IUMIWiFi\oppo\val\csv_files_processed\DATA_20112020_153104.csv" `
  --out "baseline_outputs\smoke_predictions.csv" `
  --map-out "baseline_outputs\smoke_fingerprint_map.csv"
```

Example directory run:

```powershell
.\.venv\Scripts\python.exe .\rssi_fingerprint_baseline.py `
  --target-mode s `
  --train "..\dataset\datasets--issai--IMUWiFine\snapshots\0d43350cb8ce12e10ff605c6c89b6062a0d58d7e\IMU_DATA\raw_IUMIWiFi\oppo\train\csv_files_processed" `
  --test "..\dataset\datasets--issai--IMUWiFine\snapshots\0d43350cb8ce12e10ff605c6c89b6062a0d58d7e\IMU_DATA\raw_IUMIWiFi\oppo\val\csv_files_processed" `
  --k 3 `
  --rssi-window 5 `
  --train-row-stride 5 `
  --test-row-stride 5 `
  --s-bin-m 1.0 `
  --max-jump-m 3.0 `
  --out "baseline_outputs\val_predictions.csv" `
  --map-out "baseline_outputs\fingerprint_map.csv"
```

Important notes:

- `0` RSSI values are treated as missing and converted to `-90` by default.
- AP weights are derived from fingerprint standard deviation: stable APs get higher weights.
- Large trajectory directories can be run quickly at first with `--train-row-stride 5 --test-row-stride 5`.
- In `s` mode, the preferred training label is an explicit `s` column. If `s` is missing but `x/y` exist, the script approximates a local arc length from cumulative `x/y` movement per file.
- If the CSV does not include `s` or enough geometry to derive it, the script cannot build an `s`-only fingerprint map.
- `--position-bin-m` is only used by legacy `xyz` mode. For robot path tracking, tune `--s-bin-m` instead.

Suggested field data collection for `s`:

- Mark one path centerline.
- Place reference points along the centerline with denser spacing than 10 m if possible.
- For each reference point, record:
  - `s`
  - current RSSI vector from all beacons
  - optional repeated samples for averaging

Suggested extra collection for future `e`:

- Keep the same centerline.
- At each chosen `s`, sample not only the centerline but also left/right offset points.
- Record the signed lateral offset as `e`.
- This gives training pairs like `(RSSI -> s, e)` instead of only `(RSSI -> s)`.
