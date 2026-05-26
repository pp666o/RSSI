"""RSSI fingerprint baseline for path-constrained localization.

This script is intentionally independent from the PyTorch training pipeline.
It builds a lightweight fingerprint map from labelled CSV files, then matches
new RSSI scans with weighted k-nearest-neighbour search plus simple temporal
path constraints.
"""

from __future__ import annotations

import argparse
import glob
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


IMU_COLUMNS = {"gx", "gy", "gz", "ax", "ay", "az", "mx", "my", "mz"}
POSITION_COLUMNS = {"x", "y", "z"}
META_COLUMNS = {
    "",
    "timestamp",
    "num_visible_APs",
    "Unnamed: 0",
    "Unnamed: 0.1",
    "Unnamed: 0.1.1",
}


def ensure_s_column(data: pd.DataFrame) -> pd.DataFrame:
    """Ensure a path arc-length column exists, deriving it from x/y if needed."""
    if "s" in data.columns:
        data = data.copy()
        data["s"] = pd.to_numeric(data["s"], errors="coerce")
        return data
    return add_path_s(data)


def expand_inputs(inputs: Iterable[str]) -> list[Path]:
    """Expand directories and glob patterns into CSV file paths."""
    files: list[Path] = []
    for item in inputs:
        path = Path(item)
        if path.is_dir():
            files.extend(sorted(path.rglob("*.csv")))
            continue

        matches = sorted(Path(p) for p in glob.glob(item))
        if matches:
            files.extend(matches)
        elif path.is_file():
            files.append(path)

    unique_files = sorted({p.resolve() for p in files})
    if not unique_files:
        raise FileNotFoundError(f"No CSV files found from inputs: {list(inputs)}")
    return unique_files


def infer_rssi_columns(columns: Iterable[str]) -> list[str]:
    """Infer AP/RSSI columns by excluding known IMU, position, and metadata."""
    excluded = IMU_COLUMNS | POSITION_COLUMNS | META_COLUMNS | {"s", "e", "file"}
    return [col for col in columns if col not in excluded]


def load_csvs(
    files: list[Path],
    zero_is_missing: bool,
    missing_rssi: float,
    row_stride: int,
) -> pd.DataFrame:
    frames: list[pd.DataFrame] = []
    for file in files:
        df = pd.read_csv(file).copy()
        if row_stride > 1:
            df = df.iloc[::row_stride].copy()
        df["file"] = str(file)
        frames.append(df)

    data = pd.concat(frames, ignore_index=True)
    rssi_cols = infer_rssi_columns(data.columns)
    data[rssi_cols] = data[rssi_cols].apply(pd.to_numeric, errors="coerce")
    data[rssi_cols] = data[rssi_cols].fillna(missing_rssi)
    if zero_is_missing:
        data[rssi_cols] = data[rssi_cols].replace(0.0, missing_rssi)

    for col in sorted(POSITION_COLUMNS & set(data.columns)):
        data[col] = pd.to_numeric(data[col], errors="coerce")

    if "timestamp" in data.columns:
        data["timestamp"] = pd.to_numeric(data["timestamp"], errors="coerce")

    return data


def smooth_rssi(data: pd.DataFrame, rssi_cols: list[str], window: int) -> pd.DataFrame:
    if window <= 1:
        return data

    smoothed = data.copy()
    smoothed[rssi_cols] = (
        smoothed.groupby("file", sort=False)[rssi_cols]
        .rolling(window=window, min_periods=1)
        .mean()
        .reset_index(level=0, drop=True)
    )
    return smoothed


def add_path_s(data: pd.DataFrame) -> pd.DataFrame:
    """Add an approximate path arc-length coordinate from x/y trajectory order."""
    if "s" in data.columns:
        data = data.copy()
        data["s"] = pd.to_numeric(data["s"], errors="coerce")
        return data

    if not {"x", "y"}.issubset(data.columns):
        return data

    data = data.copy()
    data["s"] = np.nan
    for _, idx in data.groupby("file", sort=False).groups.items():
        xy = data.loc[idx, ["x", "y"]].to_numpy(dtype=float)
        deltas = np.diff(xy, axis=0, prepend=xy[:1])
        step = np.linalg.norm(deltas, axis=1)
        data.loc[idx, "s"] = np.cumsum(step)
    return data


def build_fingerprint_map(
    train: pd.DataFrame,
    rssi_cols: list[str],
    target_mode: str,
    position_bin_m: float,
    s_bin_m: float,
    min_std: float,
) -> pd.DataFrame:
    train = ensure_s_column(train)
    if target_mode == "xyz":
        required = {"x", "y", "z"}
        if not required.issubset(train.columns):
            missing = ", ".join(sorted(required - set(train.columns)))
            raise ValueError(
                f"Training data is missing supervised position columns: {missing}. "
                "XYZ mode needs x/y/z labels."
            )

        working = train.dropna(subset=["x", "y", "z"]).copy()
        if working.empty:
            raise ValueError("Training data has no rows with valid x/y/z labels.")

        for col in ("x", "y", "z"):
            working[f"{col}_bin"] = np.round(working[col] / position_bin_m).astype(int)
        group_cols = ["x_bin", "y_bin", "z_bin"]
        pose_cols = ["x", "y", "z", "s"]
    else:
        working = train.dropna(subset=["s"]).copy()
        if working.empty:
            raise ValueError("Training data has no rows with valid s labels.")

        working["s_bin"] = np.round(working["s"] / s_bin_m).astype(int)
        group_cols = ["s_bin"]
        pose_cols = ["s"]
        for col in ("x", "y", "z"):
            if col in working.columns:
                pose_cols.append(col)

    grouped = working.groupby(group_cols, sort=False)
    rssi_mean = grouped[rssi_cols].mean()
    rssi_std = grouped[rssi_cols].std().fillna(min_std).clip(lower=min_std)
    pose = grouped[pose_cols].mean()
    counts = grouped.size().rename("count")

    fingerprint_map = pd.concat(
        [
            pose,
            counts,
            rssi_mean.add_prefix("mean__"),
            rssi_std.add_prefix("std__"),
        ],
        axis=1,
    ).reset_index(drop=True)
    fingerprint_map["state_id"] = np.arange(len(fingerprint_map))
    return fingerprint_map.sort_values("s").reset_index(drop=True)


def make_weights(fingerprint_map: pd.DataFrame, rssi_cols: list[str], min_std: float) -> np.ndarray:
    std = fingerprint_map[[f"std__{col}" for col in rssi_cols]].to_numpy(dtype=float)
    return 1.0 / np.square(np.maximum(std, min_std))


def predict_positions(
    test: pd.DataFrame,
    fingerprint_map: pd.DataFrame,
    rssi_cols: list[str],
    target_mode: str,
    k: int,
    min_std: float,
    max_jump_m: float | None,
) -> pd.DataFrame:
    means = fingerprint_map[[f"mean__{col}" for col in rssi_cols]].to_numpy(dtype=float)
    weights = make_weights(fingerprint_map, rssi_cols, min_std)
    if target_mode == "xyz":
        state_cols = ["state_id", "x", "y", "z", "s"]
    else:
        state_cols = ["state_id", "s"]
    states = fingerprint_map[state_cols].to_numpy(dtype=float)

    rows: list[dict[str, float | str]] = []
    prev_s: float | None = None
    prev_xy: np.ndarray | None = None
    prev_file: str | None = None
    for row_idx, row in test.iterrows():
        cur_file = str(row["file"])
        if prev_file is None or cur_file != prev_file:
            prev_s = None
            prev_xy = None
            prev_file = cur_file
        obs = row[rssi_cols].to_numpy(dtype=float)
        candidates = np.arange(len(fingerprint_map))
        if max_jump_m is not None:
            if target_mode == "xyz" and prev_xy is not None:
                xy_vals = states[:, 1:3]
                constrained = np.flatnonzero(np.linalg.norm(xy_vals - prev_xy, axis=1) <= max_jump_m)
                if len(constrained) > 0:
                    candidates = constrained
            elif target_mode != "xyz" and prev_s is not None:
                s_vals = states[:, -1]
                constrained = np.flatnonzero(np.abs(s_vals - prev_s) <= max_jump_m)
                if len(constrained) > 0:
                    candidates = constrained

        diff = means[candidates] - obs
        dist = np.sqrt(np.sum(weights[candidates] * np.square(diff), axis=1))
        nn_count = min(k, len(candidates))
        top_local = np.argpartition(dist, nn_count - 1)[:nn_count]
        top = candidates[top_local]
        top_dist = dist[top_local]
        inv = 1.0 / np.maximum(top_dist, 1e-9)
        inv /= inv.sum()

        out = {
            "file": row["file"],
            "row": int(row_idx),
            "state_id": int(states[top[np.argmin(top_dist)], 0]),
            "distance": float(np.min(top_dist)),
        }
        if target_mode == "xyz":
            pred = np.sum(states[top, 1:5] * inv[:, None], axis=0)
            prev_s = float(pred[3])
            prev_xy = pred[0:2]
            out.update(
                {
                    "x_pred": float(pred[0]),
                    "y_pred": float(pred[1]),
                    "z_pred": float(pred[2]),
                    "s_pred": float(pred[3]),
                }
            )
        else:
            pred_s = float(np.sum(states[top, 1] * inv))
            prev_s = pred_s
            out["s_pred"] = pred_s

        for col in ("timestamp", "x", "y", "z", "s"):
            if col in test.columns:
                out[col] = row[col]
        rows.append(out)

    predictions = pd.DataFrame(rows)
    if target_mode == "xyz" and {"x", "y", "z"}.issubset(predictions.columns):
        err = predictions[["x_pred", "y_pred", "z_pred"]].to_numpy() - predictions[["x", "y", "z"]].to_numpy()
        predictions["error_m"] = np.linalg.norm(err, axis=1)
    if "s_pred" in predictions.columns and "s" in predictions.columns:
        predictions["s_error"] = np.abs(predictions["s_pred"] - predictions["s"])
    return predictions


def summarize_by_file(predictions: pd.DataFrame) -> pd.DataFrame:
    metric_col = None
    if "s_error" in predictions.columns:
        metric_col = "s_error"
    elif "error_m" in predictions.columns:
        metric_col = "error_m"
    if metric_col is None or "file" not in predictions.columns:
        return pd.DataFrame()

    grouped = predictions.groupby("file")[metric_col]
    summary = pd.DataFrame(
        {
            "count": grouped.count(),
            "mean": grouped.mean(),
            "median": grouped.median(),
            "p75": grouped.quantile(0.75),
            "p90": grouped.quantile(0.90),
        }
    ).reset_index()
    summary = summary.sort_values("median").reset_index(drop=True)
    return summary


def summarize_errors(predictions: pd.DataFrame) -> dict[str, float]:
    metric_col = None
    if "s_error" in predictions.columns:
        metric_col = "s_error"
    elif "error_m" in predictions.columns:
        metric_col = "error_m"
    if metric_col is None:
        return {}
    errors = predictions[metric_col].dropna().to_numpy()
    return {
        "count": float(len(errors)),
        "mean_m": float(np.mean(errors)),
        "median_m": float(np.median(errors)),
        "p75_m": float(np.percentile(errors, 75)),
        "p90_m": float(np.percentile(errors, 90)),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RSSI fingerprint path-tracking baseline.")
    parser.add_argument("--train", nargs="+", required=True, help="Training CSV files, directories, or glob patterns.")
    parser.add_argument("--test", nargs="+", required=True, help="Test CSV files, directories, or glob patterns.")
    parser.add_argument("--target-mode", choices=["s", "xyz"], default="s", help="Predict path arc-length s, or keep legacy xyz+s mode.")
    parser.add_argument("--out", default="baseline_outputs/predictions.csv", help="Prediction CSV output path.")
    parser.add_argument("--map-out", default="baseline_outputs/fingerprint_map.csv", help="Fingerprint map CSV output path.")
    parser.add_argument("--k", type=int, default=3, help="K for weighted k-nearest-neighbour matching.")
    parser.add_argument("--rssi-window", type=int, default=5, help="Per-file RSSI rolling mean window.")
    parser.add_argument("--train-row-stride", type=int, default=1, help="Use every Nth training row.")
    parser.add_argument("--test-row-stride", type=int, default=1, help="Use every Nth test row.")
    parser.add_argument("--position-bin-m", type=float, default=1.0, help="Reference point quantization size in meters.")
    parser.add_argument("--s-bin-m", type=float, default=1.0, help="Reference point quantization size along path s.")
    parser.add_argument("--missing-rssi", type=float, default=-90.0, help="RSSI value for missing APs.")
    parser.add_argument("--min-std", type=float, default=2.0, help="Lower bound for AP std used in weighted distance.")
    parser.add_argument("--max-jump-m", type=float, default=3.0, help="Max allowed path-s jump per row; use <=0 to disable.")
    parser.add_argument("--keep-zero", action="store_true", help="Keep 0 RSSI values instead of treating them as missing.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    train_files = expand_inputs(args.train)
    test_files = expand_inputs(args.test)

    train = load_csvs(
        train_files,
        zero_is_missing=not args.keep_zero,
        missing_rssi=args.missing_rssi,
        row_stride=max(args.train_row_stride, 1),
    )
    test = load_csvs(
        test_files,
        zero_is_missing=not args.keep_zero,
        missing_rssi=args.missing_rssi,
        row_stride=max(args.test_row_stride, 1),
    )
    train_rssi_cols = infer_rssi_columns(train.columns)
    test_rssi_cols = infer_rssi_columns(test.columns)
    rssi_cols = [col for col in train_rssi_cols if col in test_rssi_cols]
    if not rssi_cols:
        raise ValueError("No shared RSSI/AP columns found between train and test data.")

    train = smooth_rssi(train, rssi_cols, args.rssi_window)
    test = smooth_rssi(test, rssi_cols, args.rssi_window)
    if args.target_mode == "s":
        test = ensure_s_column(test)
    fingerprint_map = build_fingerprint_map(
        train=train,
        rssi_cols=rssi_cols,
        target_mode=args.target_mode,
        position_bin_m=args.position_bin_m,
        s_bin_m=args.s_bin_m,
        min_std=args.min_std,
    )

    max_jump_m = args.max_jump_m if args.max_jump_m > 0 else None
    predictions = predict_positions(
        test=test,
        fingerprint_map=fingerprint_map,
        rssi_cols=rssi_cols,
        target_mode=args.target_mode,
        k=args.k,
        min_std=args.min_std,
        max_jump_m=max_jump_m,
    )

    out_path = Path(args.out)
    map_path = Path(args.map_out)
    summary_path = out_path.with_name(out_path.stem + "_by_file.csv")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    map_path.parent.mkdir(parents=True, exist_ok=True)
    predictions.to_csv(out_path, index=False)
    fingerprint_map.to_csv(map_path, index=False)
    file_summary = summarize_by_file(predictions)
    if not file_summary.empty:
        file_summary.to_csv(summary_path, index=False)

    print(f"train files: {len(train_files)}")
    print(f"test files: {len(test_files)}")
    print(f"shared RSSI columns: {len(rssi_cols)}")
    print(f"fingerprint states: {len(fingerprint_map)}")
    print(f"predictions: {out_path}")
    print(f"fingerprint map: {map_path}")
    if not file_summary.empty:
        print(f"per-file summary: {summary_path}")
    summary = summarize_errors(predictions)
    if summary:
        metric_name = "s_error" if "s_error" in predictions.columns else "error_m"
        print(
            f"{metric_name}: "
            f"mean={summary['mean_m']:.3f}, "
            f"median={summary['median_m']:.3f}, "
            f"p75={summary['p75_m']:.3f}, "
            f"p90={summary['p90_m']:.3f}, "
            f"count={int(summary['count'])}"
        )
    else:
        print("No compatible labels were found in test data, so error metrics were not computed.")


if __name__ == "__main__":
    main()
