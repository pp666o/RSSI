"""Convert raw RSSI packet logs to robot CSVs with trimmed-mean RSSI features."""

from __future__ import annotations

import argparse
import ast
import re
from pathlib import Path

import numpy as np
import pandas as pd


ACTIVE_BEACONS = (1, 2, 3, 4, 6, 9)
DISTANCE_RE = re.compile(r"distance=([-+]?\d+(?:\.\d+)?)\s+timestamp=(.*)")
BEACON_RE = re.compile(r"beacon(\d+)\s+\([^)]+\):\s+count=(\d+)\s+raw_rssi=(\[.*\])")


def trimmed_values(values: list[float], trim_ratio: float) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return arr

    drop = int(np.floor(arr.size * trim_ratio))
    if drop <= 0 or arr.size <= 2 * drop:
        return arr

    return np.sort(arr)[drop : arr.size - drop]


def summarize(values: list[float], trim_ratio: float, missing_marker: float) -> tuple[float, float]:
    trimmed = trimmed_values(values, trim_ratio)
    if trimmed.size == 0:
        return missing_marker, missing_marker
    return float(np.mean(trimmed)), float(np.std(trimmed, ddof=0))


def parse_raw_file(path: Path, trim_ratio: float, missing_marker: float) -> pd.DataFrame:
    rows: list[dict[str, float | str]] = []
    current: dict[str, float | str | list[float]] | None = None

    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        distance_match = DISTANCE_RE.search(line)
        if distance_match:
            if current is not None:
                rows.append(finalize_row(current, trim_ratio, missing_marker))
            current = {
                "s": float(distance_match.group(1)),
                "timestamp": distance_match.group(2).strip(),
            }
            for beacon in ACTIVE_BEACONS:
                current[f"beacon_{beacon}_raw"] = []
            continue

        beacon_match = BEACON_RE.search(line)
        if current is None or beacon_match is None:
            continue

        beacon_id = int(beacon_match.group(1))
        if beacon_id not in ACTIVE_BEACONS:
            continue

        current[f"beacon_{beacon_id}_raw"] = ast.literal_eval(beacon_match.group(3))

    if current is not None:
        rows.append(finalize_row(current, trim_ratio, missing_marker))

    return pd.DataFrame(rows)


def finalize_row(
    row: dict[str, float | str | list[float]],
    trim_ratio: float,
    missing_marker: float,
) -> dict[str, float | str]:
    out: dict[str, float | str] = {
        "s": row["s"],
        "timestamp": row["timestamp"],
    }
    for beacon in ACTIVE_BEACONS:
        mean, std = summarize(row.get(f"beacon_{beacon}_raw", []), trim_ratio, missing_marker)  # type: ignore[arg-type]
        out[f"beacon_{beacon}_mean"] = mean
        out[f"beacon_{beacon}_std"] = std
    return out


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build trimmed-mean robot RSSI CSVs from raw packet logs.")
    parser.add_argument("--input", nargs="+", required=True, help="Raw packet txt files.")
    parser.add_argument("--out-dir", required=True, help="Directory for converted CSV files.")
    parser.add_argument("--trim-ratio", type=float, default=0.1, help="Fraction removed from each tail.")
    parser.add_argument("--missing-marker", type=float, default=-999.0, help="Missing RSSI marker.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for item in args.input:
        input_path = Path(item)
        df = parse_raw_file(input_path, args.trim_ratio, args.missing_marker)
        output_path = out_dir / f"{input_path.stem}_trim10_robot.csv"
        df.to_csv(output_path, index=False)
        print(f"{input_path} -> {output_path} ({len(df)} rows)")


if __name__ == "__main__":
    main()
