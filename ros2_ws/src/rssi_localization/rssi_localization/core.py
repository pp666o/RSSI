from __future__ import annotations

import glob
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
import pandas as pd


@dataclass
class LocalizerConfig:
    k: int = 7
    min_std: float = 2.0
    missing_marker: float = -999.0
    missing_rssi: float = -105.0
    strong_rssi: float = -45.0
    use_mask: bool = True
    missing_mismatch_weight: float = 1.0
    rssi_representation: str = "raw"
    distance_metric: str = "euclidean"
    continuity_lambda: float = 0.3
    expected_step_s: float = 0.5
    min_observed_dims: int = 2
    confidence_distance_scale: float = 10.0
    confidence_spread_scale: float = 10.0
    confidence_blend: bool = True
    post_filter: str = "sliding_mean"
    filter_window: int = 3
    kalman_process_var: float = 0.05
    kalman_measurement_var: float = 4.0
    max_jump_s: float | None = None
    smooth_alpha: float | None = None
    start_s: float = 0.0
    monotonic_increase: bool = True


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
        raise ValueError("No beacon mean columns were found.")
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
    data[mean_cols] = data[mean_cols].apply(pd.to_numeric, errors="coerce")

    for beacon in beacon_ids:
        mean_col = f"{beacon}_mean"
        data[f"{beacon}_observed"] = data[mean_col].notna() & (data[mean_col] != missing_marker)

    data[mean_cols] = data[mean_cols].replace(missing_marker, missing_rssi).fillna(missing_rssi)

    if std_cols:
        data[std_cols] = data[std_cols].apply(pd.to_numeric, errors="coerce")
        data[std_cols] = data[std_cols].replace(missing_marker, np.nan)

    return data


def build_fingerprint_map(
    train: pd.DataFrame,
    beacon_ids: list[str],
    min_std: float,
    missing_rssi: float,
) -> pd.DataFrame:
    working = train.dropna(subset=["s"]).copy()
    grouped = working.groupby("s", sort=True)
    fingerprint = pd.DataFrame({"s": grouped["s"].mean(), "count": grouped.size()}).reset_index(drop=True)

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
            fingerprint[f"std__{beacon}"] = grouped[observed_std_col].mean().fillna(min_std).clip(lower=min_std).to_numpy()
        else:
            fingerprint[f"std__{beacon}"] = min_std

    fingerprint["state_id"] = np.arange(len(fingerprint))
    return fingerprint.sort_values("s").reset_index(drop=True)


def make_strength(values: np.ndarray, missing_rssi: float, strong_rssi: float) -> np.ndarray:
    strength_span = max(strong_rssi - missing_rssi, 1e-9)
    return np.clip((values - missing_rssi) / strength_span, 0.0, 1.0)


def transform_rssi(values: np.ndarray, representation: str, missing_rssi: float, strong_rssi: float) -> np.ndarray:
    if representation == "raw":
        return values
    if representation == "normalized":
        return make_strength(values, missing_rssi, strong_rssi)
    raise ValueError(f"Unsupported RSSI representation: {representation}")


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
    strength_weight = make_strength(means, missing_rssi, strong_rssi)
    return strength_weight * stability_weight


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
    confidence = 0.4 * observed_factor + 0.2 * distance_factor + 0.2 * margin_factor + 0.2 * spread_factor
    return (
        float(np.clip(confidence, 0.0, 1.0)),
        float(observed_factor),
        float(distance_factor),
        float(margin_factor),
        float(spread_factor),
    )


class RssiPathLocalizer:
    def __init__(self, fingerprint_map: pd.DataFrame, beacon_ids: Sequence[str], config: LocalizerConfig | None = None):
        self.config = config or LocalizerConfig()
        self.beacon_ids = list(beacon_ids)
        self.fingerprint_map = fingerprint_map.copy().reset_index(drop=True)

        self.raw_means = self.fingerprint_map[[f"mean__{beacon}" for beacon in self.beacon_ids]].to_numpy(dtype=float)
        self.means = transform_rssi(
            self.raw_means,
            self.config.rssi_representation,
            self.config.missing_rssi,
            self.config.strong_rssi,
        )
        self.weights = make_weights(
            self.fingerprint_map,
            self.beacon_ids,
            self.config.min_std,
            self.config.missing_rssi,
            self.config.strong_rssi,
        )
        self.train_strength = make_strength(self.raw_means, self.config.missing_rssi, self.config.strong_rssi)
        self.observed_rates = self.fingerprint_map[
            [f"observed_rate__{beacon}" for beacon in self.beacon_ids]
        ].to_numpy(dtype=float)
        self.states = self.fingerprint_map[["state_id", "s"]].to_numpy(dtype=float)

        self.reset(self.config.start_s)

    @classmethod
    def from_train_inputs(cls, inputs: Iterable[str], config: LocalizerConfig | None = None) -> "RssiPathLocalizer":
        cfg = config or LocalizerConfig()
        train_files = expand_inputs(inputs)
        train = load_csvs(train_files, cfg.missing_marker, cfg.missing_rssi)
        beacon_ids = infer_beacon_ids(train.columns)
        fingerprint_map = build_fingerprint_map(train, beacon_ids, cfg.min_std, cfg.missing_rssi)
        return cls(fingerprint_map=fingerprint_map, beacon_ids=beacon_ids, config=cfg)

    @classmethod
    def from_radio_map_csv(cls, path: str | Path, config: LocalizerConfig | None = None) -> "RssiPathLocalizer":
        cfg = config or LocalizerConfig()
        fingerprint_map = pd.read_csv(path)
        beacon_ids = sorted(col.replace("mean__", "") for col in fingerprint_map.columns if col.startswith("mean__"))
        return cls(fingerprint_map=fingerprint_map, beacon_ids=beacon_ids, config=cfg)

    def reset(self, start_s: float | None = None) -> None:
        self.prev_s = self.config.start_s if start_s is None else start_s
        self.filter_history: list[float] = []
        self.kalman_x: np.ndarray | None = None
        self.kalman_p: np.ndarray | None = None

    def _prepare_observation(
        self,
        mean_values: Sequence[float],
        observed_mask: Sequence[bool] | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if len(mean_values) != len(self.beacon_ids):
            raise ValueError(f"Expected {len(self.beacon_ids)} beacon values, got {len(mean_values)}")

        raw_obs = np.asarray(mean_values, dtype=float)
        inferred_mask = np.isfinite(raw_obs) & (raw_obs != self.config.missing_marker)
        if observed_mask is None:
            obs_mask = inferred_mask
        else:
            obs_mask = np.asarray(observed_mask, dtype=bool)
            if obs_mask.shape[0] != len(self.beacon_ids):
                raise ValueError("Observed mask length does not match beacon count.")

        raw_obs = np.where(np.isfinite(raw_obs), raw_obs, self.config.missing_marker)
        raw_obs = np.where(obs_mask, raw_obs, self.config.missing_rssi)
        raw_obs = np.where(raw_obs == self.config.missing_marker, self.config.missing_rssi, raw_obs)
        obs = transform_rssi(raw_obs, self.config.rssi_representation, self.config.missing_rssi, self.config.strong_rssi)
        obs_strength = make_strength(raw_obs, self.config.missing_rssi, self.config.strong_rssi)
        return raw_obs, obs, obs_mask, obs_strength

    def update(
        self,
        mean_values: Sequence[float],
        observed_mask: Sequence[bool] | None = None,
        timestamp: float | str | None = None,
    ) -> dict[str, float | int | str]:
        raw_obs, obs, obs_mask, obs_strength = self._prepare_observation(mean_values, observed_mask)

        candidates = np.arange(len(self.fingerprint_map))
        if self.config.max_jump_s is not None and self.prev_s is not None:
            constrained = np.flatnonzero(np.abs(self.states[:, 1] - self.prev_s) <= self.config.max_jump_s)
            if len(constrained) > 0:
                candidates = constrained

        candidate_weights = self.weights[candidates]
        if self.config.use_mask:
            train_mask = self.observed_rates[candidates] > 0.0
            test_mask = obs_mask[np.newaxis, :]
            both_observed = train_mask & test_mask
            mismatch = train_mask ^ test_mask
            mismatch_strength = np.maximum(self.train_strength[candidates], obs_strength[np.newaxis, :])
            mismatch_weights = (
                self.config.missing_mismatch_weight
                * mismatch_strength
                / np.square(self.config.min_std)
            )
            candidate_weights = np.where(both_observed, candidate_weights, 0.0)
            candidate_weights = candidate_weights + np.where(mismatch, mismatch_weights, 0.0)

        dist, weight_sum = compute_distances(self.means[candidates], obs, candidate_weights, self.config.distance_metric)
        valid_candidates = np.isfinite(dist)
        if not np.any(valid_candidates):
            pred_s = self.prev_s if self.prev_s is not None else float(np.median(self.states[:, 1]))
            self.prev_s = pred_s
            return {
                "timestamp": timestamp if timestamp is not None else "",
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
                "s_pred_raw": pred_s,
                "s_pred_measurement": pred_s,
                "s_pred": pred_s,
            }

        valid_candidate_indices = np.flatnonzero(valid_candidates)
        valid_candidates_global = candidates[valid_candidate_indices]
        valid_dist = dist[valid_candidates]
        continuity_penalty = np.zeros_like(valid_dist)
        if self.config.continuity_lambda > 0.0 and self.prev_s is not None:
            expected_s = self.prev_s + self.config.expected_step_s
            continuity_penalty = self.config.continuity_lambda * np.abs(self.states[valid_candidates_global, 1] - expected_s)
        ranking_dist = valid_dist + continuity_penalty

        nn_count = min(self.config.k, len(valid_candidates_global))
        top_local = np.argpartition(ranking_dist, nn_count - 1)[:nn_count]
        top = valid_candidates_global[top_local]
        top_dist = valid_dist[top_local]
        top_ranking_dist = ranking_dist[top_local]
        top_continuity_penalty = continuity_penalty[top_local]
        top_weight_sum = weight_sum[valid_candidate_indices][top_local]
        inv = 1.0 / np.maximum(top_ranking_dist, 1e-9)
        inv /= inv.sum()

        pred_s = float(np.sum(self.states[top, 1] * inv))
        raw_pred_s = pred_s
        sorted_ranking = np.sort(top_ranking_dist)
        second_distance = float(sorted_ranking[1]) if len(sorted_ranking) > 1 else float(sorted_ranking[0])
        confidence, confidence_observed, confidence_distance, confidence_margin, confidence_spread = compute_confidence(
            observed_dims=int(np.sum(obs_mask)),
            best_distance=float(np.min(top_ranking_dist)),
            second_distance=second_distance,
            top_s=self.states[top, 1],
            top_weights=inv,
            pred_s=raw_pred_s,
            min_observed_dims=self.config.min_observed_dims,
            confidence_distance_scale=self.config.confidence_distance_scale,
            confidence_spread_scale=self.config.confidence_spread_scale,
        )

        if self.config.confidence_blend and self.prev_s is not None:
            expected_s = self.prev_s + self.config.expected_step_s
            pred_s = float(confidence * pred_s + (1.0 - confidence) * expected_s)
        if self.config.smooth_alpha is not None and self.prev_s is not None:
            pred_s = float(self.config.smooth_alpha * pred_s + (1.0 - self.config.smooth_alpha) * self.prev_s)
        measurement_s = pred_s

        if self.config.post_filter == "sliding_mean":
            self.filter_history.append(measurement_s)
            self.filter_history = self.filter_history[-max(self.config.filter_window, 1) :]
            pred_s = float(np.mean(self.filter_history))
        elif self.config.post_filter == "sliding_median":
            self.filter_history.append(measurement_s)
            self.filter_history = self.filter_history[-max(self.config.filter_window, 1) :]
            pred_s = float(np.median(self.filter_history))
        elif self.config.post_filter == "kalman":
            dt = 1.0
            if self.kalman_x is None or self.kalman_p is None:
                self.kalman_x = np.array([measurement_s, self.config.expected_step_s], dtype=float)
                self.kalman_p = np.eye(2, dtype=float)
            else:
                transition = np.array([[1.0, dt], [0.0, 1.0]], dtype=float)
                process = self.config.kalman_process_var * np.array(
                    [[dt**4 / 4.0, dt**3 / 2.0], [dt**3 / 2.0, dt**2]],
                    dtype=float,
                )
                self.kalman_x = transition @ self.kalman_x
                self.kalman_p = transition @ self.kalman_p @ transition.T + process

                measurement_matrix = np.array([[1.0, 0.0]], dtype=float)
                measurement_var = self.config.kalman_measurement_var / max(confidence, 0.05)
                innovation = np.array([measurement_s], dtype=float) - measurement_matrix @ self.kalman_x
                innovation_cov = measurement_matrix @ self.kalman_p @ measurement_matrix.T + measurement_var
                gain = self.kalman_p @ measurement_matrix.T / innovation_cov[0, 0]
                self.kalman_x = self.kalman_x + (gain @ innovation)
                self.kalman_p = (np.eye(2, dtype=float) - gain @ measurement_matrix) @ self.kalman_p
            pred_s = float(self.kalman_x[0])

        if self.config.monotonic_increase and self.prev_s is not None:
            pred_s = max(self.prev_s, pred_s)

        self.prev_s = pred_s
        best_idx = int(np.argmin(top_ranking_dist))
        return {
            "timestamp": timestamp if timestamp is not None else "",
            "state_id": int(self.states[top[best_idx], 0]),
            "distance": float(np.min(top_ranking_dist)),
            "rssi_distance": float(top_dist[best_idx]),
            "continuity_penalty": float(top_continuity_penalty[best_idx]),
            "observed_dims": int(np.sum(obs_mask)),
            "weight_sum": float(top_weight_sum[best_idx]),
            "confidence": confidence,
            "confidence_observed": confidence_observed,
            "confidence_distance": confidence_distance,
            "confidence_margin": confidence_margin,
            "confidence_spread": confidence_spread,
            "s_pred_raw": raw_pred_s,
            "s_pred_measurement": measurement_s,
            "s_pred": pred_s,
        }
