# A/B Path-Tracking Plan

## Key correction about current IMUWiFine `s` handling

- `samsung_s_fingerprint_map.csv` should **not** be interpreted as one continuous path.
- In the current IMUWiFine baseline, `s` is derived independently inside each CSV trajectory.
- Therefore, sorting all reference points by `s` only gives a table ordered by local trajectory progress, not a single global robot path.
- For the real robot project, the correct setup is:
  - define one global path centerline
  - assign a unified global `s` to every reference sample
  - build one fingerprint library indexed by this global `s`
  - evaluate each test trajectory independently against that library

## Target for the real project

- Input: RSSI vector from 10 beacons
- Output: path progress `s`
- Optional future output: lateral deviation `e`

## Scheme A

Pure RSSI fingerprint + KNN

- Offline:
  - collect repeated RSSI samples for each reference point `s`
  - average over a short window (for example 5 seconds)
  - store one fingerprint per reference point
- Online:
  - get current RSSI vector
  - compare to fingerprint library
  - estimate current `s`

## Scheme B

RSSI fingerprint + path constraint + temporal smoothing

- Everything in Scheme A
- Plus:
  - only search in a local `s` range around previous estimate
  - limit maximum jump between adjacent estimates
  - smooth output `s`
  - keep a confidence indicator from match distance

## Evaluation metrics

- Main metric:
  - absolute path error `|s_pred - s_gt|`
- Auxiliary metrics:
  - median error
  - p75 / p90 error
  - jump count
  - runtime per update

## Wednesday implementation checklist

1. Freeze the real-project data format.
   - one row per reference sample
   - columns: `timestamp`, `s`, `beacon_1 ... beacon_10`
   - optional: `e`, `x`, `y`

2. Freeze the collection rule.
   - one path centerline
   - denser than 10 m if possible
   - repeated RSSI sampling per point
   - aggregate mean/std over 5 s

3. Freeze the train/test protocol.
   - build one fingerprint library from training reference points
   - validate on one test trajectory at a time
   - do not mix test trajectories into one pseudo-path

4. Implement Scheme A.
   - build `s` fingerprint library
   - KNN inference
   - report `s_error`

5. Implement Scheme B.
   - add local `s` search window
   - add max-jump limit
   - add simple smoothing

6. Produce first comparison table.
   - Scheme A vs Scheme B
   - mean / median / p75 / p90 `s_error`

## Data collection suggestion for future `e`

- Mark one visible centerline on the ground.
- Put reference points on the centerline with known `s`.
- For each chosen `s`, additionally collect left/right offset points.
- Store signed offset as `e`.
- Then the training target can become `(s, e)` instead of only `s`.
