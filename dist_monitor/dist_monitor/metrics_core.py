"""Shared distribution metric computation for online and offline modes."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from dist_monitor.kl_divergence import kl_per_joint
from dist_monitor.mmd_test import bandwidth_from_median, permutation_test, subsample_pairs
from dist_monitor.shift_detector import ShiftDetectorConfig, detect_shift
from dist_monitor.time_aligner import TimeAligner
from dist_monitor.wasserstein import wasserstein_per_joint


@dataclass
class DistributionResult:
    kl_per_joint: list[float] = field(default_factory=list)
    kl_mean: float = 0.0
    w1_per_joint: list[float] = field(default_factory=list)
    w1_mean: float = 0.0
    shift_detected_w1: bool = False
    mmd_statistic: float = 0.0
    mmd_p_value: float = 1.0
    shift_detected: bool = False
    detection_method: str = 'none'
    sample_count: int = 0


@dataclass
class MetricsConfig:
    kl_bins: int = 50
    kl_threshold_mean: float = 0.15
    w1_threshold_mean: float = 0.08
    mmd_threshold: float = 0.05
    mmd_p_value_alpha: float = 0.05
    mmd_permutation_count: int = 50
    mmd_max_samples: int = 100
    mmd_gamma: float = 0.0
    use_kl: bool = True
    use_w1: bool = True
    use_mmd: bool = True
    w1_grid_points: int = 256
    align_tolerance_sec: float = 0.02
    min_samples: int = 50
    baseline_frac: float = 0.0


def compute_distribution_metrics(
    sim_ts: np.ndarray,
    sim_states: np.ndarray,
    real_ts: np.ndarray,
    real_states: np.ndarray,
    baseline_errors: np.ndarray | None = None,
    cfg: MetricsConfig | None = None,
) -> DistributionResult:
    """Compute KL/MMD from aligned Sim/Real streams.

    sim_states / real_states: (N, 2*n_dof) as [position, velocity] per row.
    baseline_errors: reference error window for KL (P); current errors are Q.
    """
    cfg = cfg or MetricsConfig()
    result = DistributionResult()

    aligner = TimeAligner(tolerance_sec=cfg.align_tolerance_sec)
    aligned_sim, aligned_real = aligner.align_arrays(sim_ts, sim_states, real_ts, real_states)
    n = len(aligned_sim)
    result.sample_count = n

    if n < cfg.min_samples:
        return result

    n_dof = aligned_sim.shape[1] // 2
    sim_pos = aligned_sim[:, :n_dof]
    real_pos = aligned_real[:, :n_dof]
    errors = sim_pos - real_pos

    if baseline_errors is None and cfg.baseline_frac > 0:
        split = max(int(n * cfg.baseline_frac), cfg.min_samples // 3)
        split = min(split, max(n // 2, 1))
        if split >= 1 and split < n:
            baseline_errors = errors[:split]

    baseline = baseline_errors if baseline_errors is not None and len(baseline_errors) > 0 else errors

    if cfg.use_kl:
        kl_vals = kl_per_joint(baseline, errors, bins=cfg.kl_bins)
        result.kl_per_joint = kl_vals
        result.kl_mean = float(np.mean(kl_vals)) if kl_vals else 0.0

    if cfg.use_w1:
        w1_vals = wasserstein_per_joint(baseline, errors, grid_points=cfg.w1_grid_points)
        result.w1_per_joint = w1_vals
        result.w1_mean = float(np.mean(w1_vals)) if w1_vals else 0.0

    if cfg.use_mmd:
        sim_mmd, real_mmd = subsample_pairs(aligned_sim, aligned_real, max_samples=cfg.mmd_max_samples)
        gamma = cfg.mmd_gamma
        if gamma <= 0:
            gamma = bandwidth_from_median(sim_mmd, real_mmd)
        mmd_stat, p_val = permutation_test(
            sim_mmd,
            real_mmd,
            gamma,
            n_permutations=cfg.mmd_permutation_count,
            max_samples=cfg.mmd_max_samples,
        )
        result.mmd_statistic = mmd_stat
        result.mmd_p_value = p_val

    shift_cfg = ShiftDetectorConfig(
        use_kl=cfg.use_kl,
        use_w1=cfg.use_w1,
        use_mmd=cfg.use_mmd,
        kl_threshold_mean=cfg.kl_threshold_mean,
        w1_threshold_mean=cfg.w1_threshold_mean,
        mmd_threshold=cfg.mmd_threshold,
        mmd_p_value_alpha=cfg.mmd_p_value_alpha,
    )
    result.shift_detected, result.detection_method, result.shift_detected_w1 = detect_shift(
        result.kl_mean,
        result.w1_mean,
        result.mmd_statistic,
        result.mmd_p_value,
        shift_cfg,
    )
    return result
