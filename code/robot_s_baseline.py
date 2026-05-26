"""RSSI path-progress baseline for pre-aggregated robot data.

Expected columns follow the user's planned collection format:

- s
- timestamp (optional)
- beacon_1_mean, beacon_1_std
- beacon_2_mean, beacon_2_std
- ...

Training rows form a fingerprint library indexed by the true global path s.
Testing rows are matched against that library to predict s.
"""

from __future__ import annotations

import argparse
import glob
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


def expand_inputs(inputs: Iterable[str]) -> list[Path]:
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


def infer_beacon_ids(columns: Iterable[str]) -> list[str]:
    beacon_ids = sorted({col[:-5] for col in columns if col.endswith("_mean")})
    if not beacon_ids:
        raise ValueError("No beacon mean columns were found. Expected columns like beacon_1_mean.")
    return beacon_ids


def load_csvs(files: list[Path], missing_marker: float, missing_rssi: float) -> pd.DataFrame:
    frames: list[pd.DataFrame] = []
    for file in files:
        df = pd.read_csv(file).copy()
        df["file"] = str(file)
        frames.append(df)

    data = pd.concat(frames, ignore_index=True)
    beacon_ids = infer_beacon_ids(data.columns)

    mean_cols = [f"{beacon}_mean" for beacon in beacon_ids]
    std_cols = [f"{beacon}_std" for beacon in beacon_ids if f"{beacon}_std" in data.columns]

    data["s"] = pd.to_numeric(data["s"], errors="coerce")
    if "timestamp" in data.columns:
        data["timestamp"] = pd.to_numeric(data["timestamp"], errors="coerce")

    data[mean_cols] = data[mean_cols].apply(pd.to_numeric, errors="coerce")

    for beacon in beacon_ids:
        mean_col = f"{beacon}_mean"
        data[f"{beacon}_observed"] = data[mean_col].notna() & (data[mean_col] != missing_marker)

    data[mean_cols] = data[mean_cols].replace(missing_marker, missing_rssi).fillna(missing_rssi)

    if std_cols:
        data[std_cols] = data[std_cols].apply(pd.to_numeric, errors="coerce")
        data[std_cols] = data[std_cols].replace(missing_marker, np.nan)

    return data


def load_imu_csv(path: str | None) -> pd.DataFrame | None:
    if not path:
        return None

    imu = pd.read_csv(Path(path)).copy()
    if "elapsed_sec" not in imu.columns:
        raise ValueError("IMU CSV must contain elapsed_sec.")

    imu["elapsed_sec"] = pd.to_numeric(imu["elapsed_sec"], errors="coerce")
    required = [
        "acc_x_g",
        "acc_y_g",
        "acc_z_g",
        "gyro_x_dps",
        "gyro_y_dps",
        "gyro_z_dps",
    ]
    missing = [col for col in required if col not in imu.columns]
    if missing:
        raise ValueError(f"IMU CSV is missing required columns: {missing}")

    for col in required:
        imu[col] = pd.to_numeric(imu[col], errors="coerce")
    imu = imu.dropna(subset=["elapsed_sec", *required]).sort_values("elapsed_sec")
    imu["acc_norm"] = np.sqrt(np.square(imu[["acc_x_g", "acc_y_g", "acc_z_g"]]).sum(axis=1))
    imu["acc_dynamic"] = np.abs(imu["acc_norm"] - 1.0)
    imu["gyro_norm"] = np.sqrt(np.square(imu[["gyro_x_dps", "gyro_y_dps", "gyro_z_dps"]]).sum(axis=1))
    return imu


def attach_imu_features(
    data: pd.DataFrame,
    imu: pd.DataFrame | None,
    window_sec: float,
    static_acc_threshold: float,
    static_gyro_threshold: float,
) -> pd.DataFrame:
    data = data.copy()
    if imu is None or "elapsed_sec" not in data.columns:
        data["imu_acc_dynamic_mean"] = np.nan
        data["imu_gyro_norm_mean"] = np.nan
        data["imu_motion_score"] = np.nan
        data["imu_is_static"] = False
        return data

    half_window = max(window_sec, 1e-6) / 2.0
    elapsed = pd.to_numeric(data["elapsed_sec"], errors="coerce").to_numpy(dtype=float)
    imu_elapsed = imu["elapsed_sec"].to_numpy(dtype=float)
    acc_dynamic = imu["acc_dynamic"].to_numpy(dtype=float)
    gyro_norm = imu["gyro_norm"].to_numpy(dtype=float)

    acc_means: list[float] = []
    gyro_means: list[float] = []
    for t in elapsed:
        if not np.isfinite(t):
            acc_means.append(np.nan)
            gyro_means.append(np.nan)
            continue
        start = np.searchsorted(imu_elapsed, t - half_window, side="left")
        end = np.searchsorted(imu_elapsed, t + half_window, side="right")
        if end <= start:
            acc_means.append(np.nan)
            gyro_means.append(np.nan)
            continue
        acc_means.append(float(np.nanmean(acc_dynamic[start:end])))
        gyro_means.append(float(np.nanmean(gyro_norm[start:end])))

    data["imu_acc_dynamic_mean"] = acc_means
    data["imu_gyro_norm_mean"] = gyro_means
    acc_score = data["imu_acc_dynamic_mean"] / max(static_acc_threshold, 1e-9)
    gyro_score = data["imu_gyro_norm_mean"] / max(static_gyro_threshold, 1e-9)
    data["imu_motion_score"] = np.clip(np.maximum(acc_score, gyro_score), 0.0, 1.0)
    data["imu_is_static"] = (
        (data["imu_acc_dynamic_mean"] < static_acc_threshold)
        & (data["imu_gyro_norm_mean"] < static_gyro_threshold)
    )
    return data


def build_fingerprint_map(
    train: pd.DataFrame,
    beacon_ids: list[str],
    min_std: float,
    missing_rssi: float,
) -> pd.DataFrame:
    working = train.dropna(subset=["s"]).copy()
    if working.empty:
        raise ValueError("Training data has no valid s labels.")

    grouped = working.groupby("s", sort=True)
    fingerprint = pd.DataFrame(
        {
            "s": grouped["s"].mean(),
            "count": grouped.size(),
        }
    ).reset_index(drop=True)

    for beacon in beacon_ids:
        mean_col = f"{beacon}_mean"
        std_col = f"{beacon}_std"
        observed_col = f"{beacon}_observed"
        if observed_col not in working.columns:
            working[observed_col] = working[mean_col].notna()

        observed_mean_col = f"_observed_mean__{beacon}"
        observed_std_col = f"_observed_std__{beacon}"
        working[observed_mean_col] = working[mean_col].where(working[observed_col])
        fingerprint[f"mean__{beacon}"] = grouped[observed_mean_col].mean().fillna(missing_rssi).to_numpy()
        fingerprint[f"observed_rate__{beacon}"] = grouped[observed_col].mean().to_numpy()

        if std_col in working.columns:
            working[observed_std_col] = working[std_col].where(working[observed_col])
            # min_std only prevents over-trusting tiny/zero packet spread; missing beacons are masked later.
            fingerprint[f"std__{beacon}"] = grouped[observed_std_col].mean().fillna(min_std).clip(lower=min_std).to_numpy()
        else:
            fingerprint[f"std__{beacon}"] = min_std

    fingerprint["state_id"] = np.arange(len(fingerprint))
    return fingerprint.sort_values("s").reset_index(drop=True)


def make_weights(
    fingerprint_map: pd.DataFrame,
    beacon_ids: list[str],
    min_std: float,
    missing_rssi: float,
    strong_rssi: float,
) -> np.ndarray:
    means = fingerprint_map[[f"mean__{beacon}" for beacon in beacon_ids]].to_numpy(dtype=float)
    std = fingerprint_map[[f"std__{beacon}" for beacon in beacon_ids]].to_numpy(dtype=float)
    stability_weight = 1.0 / np.square(np.maximum(std, min_std))
    strength_span = max(strong_rssi - missing_rssi, 1e-9)
    strength_weight = np.clip((means - missing_rssi) / strength_span, 0.0, 1.0)
    return strength_weight * stability_weight


def make_strength(values: np.ndarray, missing_rssi: float, strong_rssi: float) -> np.ndarray:
    strength_span = max(strong_rssi - missing_rssi, 1e-9)
    return np.clip((values - missing_rssi) / strength_span, 0.0, 1.0)


def transform_rssi(
    values: np.ndarray,
    representation: str,
    missing_rssi: float,
    strong_rssi: float,
) -> np.ndarray:
    if representation == "raw":
        return values
    if representation == "normalized":
        return make_strength(values, missing_rssi, strong_rssi)
    raise ValueError(f"Unsupported RSSI representation: {representation}")


def compute_distances(
    reference: np.ndarray,
    observation: np.ndarray,
    weights: np.ndarray,
    metric: str,
) -> tuple[np.ndarray, np.ndarray]:
    weight_sum = np.sum(weights, axis=1)
    valid = weight_sum > 0.0
    distances = np.full(len(reference), np.nan, dtype=float)
    if not np.any(valid):
        return distances, weight_sum

    ref = reference[valid]
    w = weights[valid]
    obs = observation[np.newaxis, :]

    if metric == "euclidean":
        weighted_sq_error = np.sum(w * np.square(ref - obs), axis=1)
        distances[valid] = np.sqrt(weighted_sq_error / weight_sum[valid])
    elif metric == "manhattan":
        weighted_abs_error = np.sum(w * np.abs(ref - obs), axis=1)
        distances[valid] = weighted_abs_error / weight_sum[valid]
    elif metric == "sorensen":
        numerator = np.sum(w * np.abs(ref - obs), axis=1)
        denominator = np.sum(w * (np.abs(ref) + np.abs(obs)), axis=1)
        distances[valid] = numerator / np.maximum(denominator, 1e-12)
    elif metric == "cosine":
        numerator = np.sum(w * ref * obs, axis=1)
        ref_norm = np.sqrt(np.sum(w * np.square(ref), axis=1))
        obs_norm = np.sqrt(np.sum(w * np.square(obs), axis=1))
        cosine_similarity = numerator / np.maximum(ref_norm * obs_norm, 1e-12)
        distances[valid] = 1.0 - np.clip(cosine_similarity, -1.0, 1.0)
    else:
        raise ValueError(f"Unsupported distance metric: {metric}")

    return distances, weight_sum


def compute_confidence(
    observed_dims: int,
    best_distance: float,
    second_distance: float,
    top_s: np.ndarray,
    top_weights: np.ndarray,
    pred_s: float,
    min_observed_dims: int,
    confidence_distance_scale: float,
    confidence_spread_scale: float,
) -> tuple[float, float, float, float, float]:
    observed_factor = min(observed_dims / max(min_observed_dims, 1), 1.0)
    distance_factor = 1.0 / (1.0 + best_distance / max(confidence_distance_scale, 1e-9))
    margin = max(second_distance - best_distance, 0.0)
    margin_factor = margin / (margin + 1.0)
    s_spread = float(np.sum(top_weights * np.abs(top_s - pred_s)))
    spread_factor = 1.0 / (1.0 + s_spread / max(confidence_spread_scale, 1e-9))
    confidence = (
        0.4 * observed_factor
        + 0.2 * distance_factor
        + 0.2 * margin_factor
        + 0.2 * spread_factor
    )
    return (
        float(np.clip(confidence, 0.0, 1.0)),
        float(observed_factor),
        float(distance_factor),
        float(margin_factor),
        float(spread_factor),
    )


def estimate_imu_delta_s(row: pd.Series, expected_step_s: float) -> float:
    if "imu_delta_s" in row and pd.notna(row["imu_delta_s"]):
        return float(row["imu_delta_s"])

    motion_score = float(row.get("imu_motion_score", np.nan))
    if np.isfinite(motion_score):
        return float(expected_step_s * np.clip(motion_score, 0.0, 1.0))

    return float(expected_step_s)


def predict_s(
    test: pd.DataFrame,
    fingerprint_map: pd.DataFrame,
    beacon_ids: list[str],
    k: int,
    min_std: float,
    missing_rssi: float,
    strong_rssi: float,
    use_mask: bool,
    missing_mismatch_weight: float,
    rssi_representation: str,
    distance_metric: str,
    continuity_lambda: float,
    expected_step_s: float,
    min_observed_dims: int,
    confidence_distance_scale: float,
    confidence_spread_scale: float,
    confidence_blend: bool,
    post_filter: str,
    filter_window: int,
    kalman_process_var: float,
    kalman_measurement_var: float,
    max_jump_s: float | None,
    smooth_alpha: float | None,
    imu_motion_gate: bool,
    imu_static_alpha: float,
    imu_dynamic_alpha: float,
    imu_static_max_jump_s: float,
    imu_dynamic_max_jump_s: float,
) -> pd.DataFrame:
    raw_means = fingerprint_map[[f"mean__{beacon}" for beacon in beacon_ids]].to_numpy(dtype=float)
    means = transform_rssi(raw_means, rssi_representation, missing_rssi, strong_rssi)
    weights = make_weights(fingerprint_map, beacon_ids, min_std, missing_rssi, strong_rssi)
    train_strength = make_strength(raw_means, missing_rssi, strong_rssi)
    observed_rates = fingerprint_map[[f"observed_rate__{beacon}" for beacon in beacon_ids]].to_numpy(dtype=float)
    states = fingerprint_map[["state_id", "s"]].to_numpy(dtype=float)

    rows: list[dict[str, float | str]] = []
    prev_s: float | None = None
    prev_file: str | None = None
    filter_history: list[float] = []
    kalman_x: np.ndarray | None = None
    kalman_p: np.ndarray | None = None
    for row_idx, row in test.iterrows():
        cur_file = str(row["file"])
        if prev_file is None or cur_file != prev_file:
            prev_s = None
            prev_file = cur_file
            filter_history = []
            kalman_x = None
            kalman_p = None

        raw_obs = row[[f"{beacon}_mean" for beacon in beacon_ids]].to_numpy(dtype=float)
        obs = transform_rssi(raw_obs, rssi_representation, missing_rssi, strong_rssi)
        obs_mask = row[[f"{beacon}_observed" for beacon in beacon_ids]].to_numpy(dtype=bool)
        obs_strength = make_strength(raw_obs, missing_rssi, strong_rssi)
        candidates = np.arange(len(fingerprint_map))
        imu_motion_score = float(row.get("imu_motion_score", np.nan))
        imu_is_static = bool(row.get("imu_is_static", False))
        effective_max_jump_s = max_jump_s
        if imu_motion_gate and prev_s is not None and np.isfinite(imu_motion_score):
            effective_max_jump_s = float(
                imu_static_max_jump_s
                + imu_motion_score * (imu_dynamic_max_jump_s - imu_static_max_jump_s)
            )
        if effective_max_jump_s is not None and prev_s is not None:
            constrained = np.flatnonzero(np.abs(states[:, 1] - prev_s) <= effective_max_jump_s)
            if len(constrained) > 0:
                candidates = constrained

        candidate_weights = weights[candidates]
        if use_mask:
            train_mask = observed_rates[candidates] > 0.0
            test_mask = obs_mask[np.newaxis, :]
            both_observed = train_mask & test_mask
            mismatch = train_mask ^ test_mask
            mismatch_strength = np.maximum(train_strength[candidates], obs_strength[np.newaxis, :])
            mismatch_weights = missing_mismatch_weight * mismatch_strength / np.square(min_std)
            candidate_weights = np.where(both_observed, candidate_weights, 0.0)
            candidate_weights = candidate_weights + np.where(mismatch, mismatch_weights, 0.0)

        dist, weight_sum = compute_distances(means[candidates], obs, candidate_weights, distance_metric)
        valid_candidates = np.isfinite(dist)
        if not np.any(valid_candidates):
            pred_s = prev_s if prev_s is not None else float(np.median(states[:, 1]))
            raw_pred_s = pred_s
            out = {
                "file": row["file"],
                "row": int(row_idx),
                "state_id": -1,
                "distance": np.nan,
                "rssi_distance": np.nan,
                "continuity_penalty": np.nan,
                "observed_dims": int(np.sum(obs_mask)),
                "weight_sum": 0.0,
                "confidence": 0.0,
                "confidence_observed": 0.0,
                "confidence_distance": 0.0,
                "confidence_margin": 0.0,
                "confidence_spread": 0.0,
                "s_pred_raw": raw_pred_s,
                "s_pred_measurement": pred_s,
                "s_pred": pred_s,
                "ekf_imu_delta_s": np.nan,
                "ekf_state_s": np.nan,
                "ekf_state_v": np.nan,
            }
            if "timestamp" in test.columns:
                out["timestamp"] = row["timestamp"]
            if "s" in test.columns:
                out["s"] = row["s"]
            out["imu_acc_dynamic_mean"] = float(row.get("imu_acc_dynamic_mean", np.nan))
            out["imu_gyro_norm_mean"] = float(row.get("imu_gyro_norm_mean", np.nan))
            out["imu_motion_score"] = float(row.get("imu_motion_score", np.nan))
            out["imu_is_static"] = bool(row.get("imu_is_static", False))
            out["effective_max_jump_s"] = float(effective_max_jump_s) if effective_max_jump_s is not None else np.nan
            rows.append(out)
            prev_s = pred_s
            continue

        valid_candidate_indices = np.flatnonzero(valid_candidates)
        valid_candidates_global = candidates[valid_candidate_indices]
        valid_dist = dist[valid_candidates]
        continuity_penalty = np.zeros_like(valid_dist)
        if continuity_lambda > 0.0 and prev_s is not None:
            expected_s = prev_s + expected_step_s
            continuity_penalty = continuity_lambda * np.abs(states[valid_candidates_global, 1] - expected_s)
        ranking_dist = valid_dist + continuity_penalty
        nn_count = min(k, len(candidates))
        nn_count = min(nn_count, len(valid_candidates_global))
        top_local = np.argpartition(ranking_dist, nn_count - 1)[:nn_count]
        top = valid_candidates_global[top_local]
        top_dist = valid_dist[top_local]
        top_ranking_dist = ranking_dist[top_local]
        top_continuity_penalty = continuity_penalty[top_local]
        top_weight_sum = weight_sum[valid_candidate_indices][top_local]
        inv = 1.0 / np.maximum(top_ranking_dist, 1e-9)
        inv /= inv.sum()

        pred_s = float(np.sum(states[top, 1] * inv))
        raw_pred_s = pred_s
        sorted_ranking = np.sort(top_ranking_dist)
        second_distance = float(sorted_ranking[1]) if len(sorted_ranking) > 1 else float(sorted_ranking[0])
        (
            confidence,
            confidence_observed,
            confidence_distance,
            confidence_margin,
            confidence_spread,
        ) = compute_confidence(
            observed_dims=int(np.sum(obs_mask)),
            best_distance=float(np.min(top_ranking_dist)),
            second_distance=second_distance,
            top_s=states[top, 1],
            top_weights=inv,
            pred_s=raw_pred_s,
            min_observed_dims=min_observed_dims,
            confidence_distance_scale=confidence_distance_scale,
            confidence_spread_scale=confidence_spread_scale,
        )
        if confidence_blend and prev_s is not None:
            expected_s = prev_s + expected_step_s
            pred_s = float(confidence * pred_s + (1.0 - confidence) * expected_s)
        if imu_motion_gate and prev_s is not None and np.isfinite(imu_motion_score):
            imu_alpha = float(
                imu_static_alpha + imu_motion_score * (imu_dynamic_alpha - imu_static_alpha)
            )
            imu_alpha = float(np.clip(imu_alpha, 0.0, 1.0))
            pred_s = float(imu_alpha * pred_s + (1.0 - imu_alpha) * prev_s)
        if smooth_alpha is not None and prev_s is not None:
            pred_s = float(smooth_alpha * pred_s + (1.0 - smooth_alpha) * prev_s)
        measurement_s = pred_s

        ekf_imu_delta_s = np.nan
        ekf_state_s = np.nan
        ekf_state_v = np.nan
        if post_filter == "sliding_mean":
            filter_history.append(measurement_s)
            filter_history = filter_history[-max(filter_window, 1) :]
            pred_s = float(np.mean(filter_history))
        elif post_filter == "sliding_median":
            filter_history.append(measurement_s)
            filter_history = filter_history[-max(filter_window, 1) :]
            pred_s = float(np.median(filter_history))
        elif post_filter == "kalman":
            dt = 1.0
            if kalman_x is None or kalman_p is None:
                initial_v = expected_step_s
                kalman_x = np.array([measurement_s, initial_v], dtype=float)
                kalman_p = np.eye(2, dtype=float)
            else:
                transition = np.array([[1.0, dt], [0.0, 1.0]], dtype=float)
                process = kalman_process_var * np.array(
                    [[dt**4 / 4.0, dt**3 / 2.0], [dt**3 / 2.0, dt**2]],
                    dtype=float,
                )
                kalman_x = transition @ kalman_x
                kalman_p = transition @ kalman_p @ transition.T + process

                measurement_matrix = np.array([[1.0, 0.0]], dtype=float)
                measurement_var = kalman_measurement_var / max(confidence, 0.05)
                innovation = np.array([measurement_s], dtype=float) - measurement_matrix @ kalman_x
                innovation_cov = measurement_matrix @ kalman_p @ measurement_matrix.T + measurement_var
                gain = kalman_p @ measurement_matrix.T / innovation_cov[0, 0]
                kalman_x = kalman_x + (gain @ innovation)
                kalman_p = (np.eye(2, dtype=float) - gain @ measurement_matrix) @ kalman_p
            pred_s = float(kalman_x[0])
        elif post_filter == "ekf":
            imu_delta_s = estimate_imu_delta_s(row, expected_step_s)
            ekf_imu_delta_s = imu_delta_s
            if kalman_x is None or kalman_p is None:
                kalman_x = np.array([measurement_s, imu_delta_s], dtype=float)
                kalman_p = np.diag([kalman_measurement_var, max(kalman_process_var, 1e-9)])
            else:
                predicted_s = kalman_x[0] + imu_delta_s
                predicted_v = 0.7 * kalman_x[1] + 0.3 * imu_delta_s
                kalman_x_pred = np.array([predicted_s, predicted_v], dtype=float)
                transition_jacobian = np.array([[1.0, 0.0], [0.0, 0.7]], dtype=float)
                control_jacobian = np.array([[1.0], [0.3]], dtype=float)
                process = kalman_process_var * (control_jacobian @ control_jacobian.T)
                kalman_p_pred = transition_jacobian @ kalman_p @ transition_jacobian.T + process

                measurement_matrix = np.array([[1.0, 0.0]], dtype=float)
                measurement_var = kalman_measurement_var / max(confidence, 0.05)
                innovation = np.array([measurement_s], dtype=float) - measurement_matrix @ kalman_x_pred
                innovation_cov = measurement_matrix @ kalman_p_pred @ measurement_matrix.T + measurement_var
                gain = kalman_p_pred @ measurement_matrix.T / innovation_cov[0, 0]
                kalman_x = kalman_x_pred + (gain @ innovation)
                kalman_p = (np.eye(2, dtype=float) - gain @ measurement_matrix) @ kalman_p_pred
            pred_s = float(kalman_x[0])
            ekf_state_s = float(kalman_x[0])
            ekf_state_v = float(kalman_x[1])

        prev_s = pred_s

        out = {
            "file": row["file"],
            "row": int(row_idx),
            "state_id": int(states[top[np.argmin(top_ranking_dist)], 0]),
            "distance": float(np.min(top_ranking_dist)),
            "rssi_distance": float(top_dist[np.argmin(top_ranking_dist)]),
            "continuity_penalty": float(top_continuity_penalty[np.argmin(top_ranking_dist)]),
            "observed_dims": int(np.sum(obs_mask)),
            "weight_sum": float(top_weight_sum[np.argmin(top_ranking_dist)]),
            "confidence": confidence,
            "confidence_observed": confidence_observed,
            "confidence_distance": confidence_distance,
            "confidence_margin": confidence_margin,
            "confidence_spread": confidence_spread,
            "s_pred_raw": raw_pred_s,
            "s_pred_measurement": measurement_s,
            "s_pred": pred_s,
            "ekf_imu_delta_s": ekf_imu_delta_s,
            "ekf_state_s": ekf_state_s,
            "ekf_state_v": ekf_state_v,
            "imu_acc_dynamic_mean": float(row.get("imu_acc_dynamic_mean", np.nan)),
            "imu_gyro_norm_mean": float(row.get("imu_gyro_norm_mean", np.nan)),
            "imu_motion_score": imu_motion_score,
            "imu_is_static": imu_is_static,
            "effective_max_jump_s": float(effective_max_jump_s) if effective_max_jump_s is not None else np.nan,
        }
        if "timestamp" in test.columns:
            out["timestamp"] = row["timestamp"]
        if "s" in test.columns:
            out["s"] = row["s"]
        rows.append(out)

    predictions = pd.DataFrame(rows)
    if "s" in predictions.columns:
        predictions["s_error"] = np.abs(predictions["s_pred"] - predictions["s"])
    return predictions


def summarize_errors(predictions: pd.DataFrame) -> dict[str, float]:
    if "s_error" not in predictions.columns:
        return {}
    errors = predictions["s_error"].dropna().to_numpy()
    return {
        "count": float(len(errors)),
        "mean": float(np.mean(errors)),
        "median": float(np.median(errors)),
        "p75": float(np.percentile(errors, 75)),
        "p90": float(np.percentile(errors, 90)),
    }


def summarize_by_file(predictions: pd.DataFrame) -> pd.DataFrame:
    if "s_error" not in predictions.columns or "file" not in predictions.columns:
        return pd.DataFrame()
    grouped = predictions.groupby("file")["s_error"]
    return pd.DataFrame(
        {
            "count": grouped.count(),
            "mean": grouped.mean(),
            "median": grouped.median(),
            "p75": grouped.quantile(0.75),
            "p90": grouped.quantile(0.90),
        }
    ).reset_index()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Robot path-progress RSSI baseline.")
    parser.add_argument("--train", nargs="+", required=True, help="Training CSV files, directories, or glob patterns.")
    parser.add_argument("--test", nargs="+", required=True, help="Test CSV files, directories, or glob patterns.")
    parser.add_argument("--out", default="baseline_outputs/robot_s_predictions.csv", help="Prediction CSV output path.")
    parser.add_argument("--map-out", default="baseline_outputs/robot_s_fingerprint_map.csv", help="Fingerprint map CSV output path.")
    parser.add_argument("--k", type=int, default=3, help="K for weighted k-nearest-neighbour matching.")
    parser.add_argument("--min-std", type=float, default=2.0, help="Lower bound for beacon std used in weighted distance.")
    parser.add_argument("--max-jump-s", type=float, default=0.0, help="Max allowed s jump per row; use <=0 to disable.")
    parser.add_argument("--smooth-alpha", type=float, default=0.0, help="EMA smoothing factor for scheme B; use <=0 to disable.")
    parser.add_argument("--missing-marker", type=float, default=-999.0, help="Sentinel value stored by the logger for missing signals.")
    parser.add_argument("--missing-rssi", type=float, default=-105.0, help="RSSI value used internally after replacing the missing marker.")
    parser.add_argument("--strong-rssi", type=float, default=-45.0, help="RSSI level treated as a strong signal for strength weighting.")
    parser.add_argument("--missing-mismatch-weight", type=float, default=1.0, help="Penalty weight when one side observes a beacon and the other side misses it.")
    parser.add_argument("--rssi-representation", choices=["raw", "normalized"], default="raw", help="RSSI representation used inside the distance metric.")
    parser.add_argument("--distance-metric", choices=["euclidean", "manhattan", "sorensen", "cosine"], default="euclidean", help="Distance metric used by KNN matching.")
    parser.add_argument("--continuity-lambda", type=float, default=0.0, help="Soft path-continuity penalty added as lambda * abs(candidate_s - previous_s).")
    parser.add_argument("--expected-step-s", type=float, default=0.0, help="Expected path progress between two consecutive rows for continuity prior.")
    parser.add_argument("--min-observed-dims", type=int, default=2, help="Observed beacon count treated as enough for full confidence.")
    parser.add_argument("--confidence-distance-scale", type=float, default=10.0, help="Distance scale used by the confidence score.")
    parser.add_argument("--confidence-spread-scale", type=float, default=10.0, help="Top-K s-spread scale used by the confidence score.")
    parser.add_argument("--confidence-blend", action="store_true", help="Blend low-confidence RSSI predictions toward the motion prior.")
    parser.add_argument("--post-filter", choices=["none", "sliding_mean", "sliding_median", "kalman", "ekf"], default="none", help="Post-matching trajectory filter.")
    parser.add_argument("--filter-window", type=int, default=5, help="Window size for sliding mean/median filters.")
    parser.add_argument("--kalman-process-var", type=float, default=0.05, help="Process noise variance for the 1D constant-velocity Kalman filter.")
    parser.add_argument("--kalman-measurement-var", type=float, default=4.0, help="Base measurement noise variance for the Kalman filter.")
    parser.add_argument("--imu", default="", help="Optional IMU CSV from test_rssi.cpp, aligned to test rows by elapsed_sec.")
    parser.add_argument("--imu-window-sec", type=float, default=1.0, help="Time window used to aggregate IMU features around each RSSI row.")
    parser.add_argument("--imu-motion-gate", action="store_true", help="Use IMU motion score as an auxiliary constraint for RSSI s predictions.")
    parser.add_argument("--imu-static-acc-threshold", type=float, default=0.04, help="acc_dynamic below this threshold is treated as static evidence.")
    parser.add_argument("--imu-static-gyro-threshold", type=float, default=3.0, help="gyro_norm below this threshold is treated as static evidence.")
    parser.add_argument("--imu-static-alpha", type=float, default=0.15, help="RSSI blend weight when IMU indicates static/low motion.")
    parser.add_argument("--imu-dynamic-alpha", type=float, default=0.75, help="RSSI blend weight when IMU indicates clear motion.")
    parser.add_argument("--imu-static-max-jump-s", type=float, default=0.5, help="Max s jump allowed when IMU indicates static/low motion.")
    parser.add_argument("--imu-dynamic-max-jump-s", type=float, default=3.0, help="Max s jump allowed when IMU indicates clear motion.")
    parser.add_argument("--no-mask", action="store_true", help="Disable explicit observed-mask handling and compare every beacon.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    train_files = expand_inputs(args.train)
    test_files = expand_inputs(args.test)
    train = load_csvs(train_files, args.missing_marker, args.missing_rssi)
    test = load_csvs(test_files, args.missing_marker, args.missing_rssi)
    imu = load_imu_csv(args.imu)
    test = attach_imu_features(
        test,
        imu,
        window_sec=args.imu_window_sec,
        static_acc_threshold=args.imu_static_acc_threshold,
        static_gyro_threshold=args.imu_static_gyro_threshold,
    )

    beacon_ids = infer_beacon_ids(train.columns)
    test_beacon_ids = infer_beacon_ids(test.columns)
    if beacon_ids != test_beacon_ids:
        shared = [beacon for beacon in beacon_ids if beacon in test_beacon_ids]
        if not shared:
            raise ValueError("Train and test have no shared beacon_*_mean columns.")
        beacon_ids = shared

    fingerprint_map = build_fingerprint_map(train, beacon_ids, args.min_std, args.missing_rssi)
    predictions = predict_s(
        test=test,
        fingerprint_map=fingerprint_map,
        beacon_ids=beacon_ids,
        k=args.k,
        min_std=args.min_std,
        missing_rssi=args.missing_rssi,
        strong_rssi=args.strong_rssi,
        use_mask=not args.no_mask,
        missing_mismatch_weight=args.missing_mismatch_weight,
        rssi_representation=args.rssi_representation,
        distance_metric=args.distance_metric,
        continuity_lambda=args.continuity_lambda,
        expected_step_s=args.expected_step_s,
        min_observed_dims=args.min_observed_dims,
        confidence_distance_scale=args.confidence_distance_scale,
        confidence_spread_scale=args.confidence_spread_scale,
        confidence_blend=args.confidence_blend,
        post_filter=args.post_filter,
        filter_window=args.filter_window,
        kalman_process_var=args.kalman_process_var,
        kalman_measurement_var=args.kalman_measurement_var,
        max_jump_s=args.max_jump_s if args.max_jump_s > 0 else None,
        smooth_alpha=args.smooth_alpha if args.smooth_alpha > 0 else None,
        imu_motion_gate=args.imu_motion_gate,
        imu_static_alpha=args.imu_static_alpha,
        imu_dynamic_alpha=args.imu_dynamic_alpha,
        imu_static_max_jump_s=args.imu_static_max_jump_s,
        imu_dynamic_max_jump_s=args.imu_dynamic_max_jump_s,
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
    print(f"beacons: {len(beacon_ids)}")
    if imu is not None:
        print(f"imu rows: {len(imu)}")
        print(f"imu motion gate: {args.imu_motion_gate}")
    print(f"fingerprint states: {len(fingerprint_map)}")
    print(f"predictions: {out_path}")
    print(f"fingerprint map: {map_path}")
    if not file_summary.empty:
        print(f"per-file summary: {summary_path}")
    summary = summarize_errors(predictions)
    if summary:
        print(
            "s_error: "
            f"mean={summary['mean']:.3f}, "
            f"median={summary['median']:.3f}, "
            f"p75={summary['p75']:.3f}, "
            f"p90={summary['p90']:.3f}, "
            f"count={int(summary['count'])}"
        )
    else:
        print("No s labels were found in test data, so s_error was not computed.")


if __name__ == "__main__":
    main()
