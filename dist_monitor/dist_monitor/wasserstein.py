"""Wasserstein-1 distance for per-joint error distributions."""

from __future__ import annotations

import numpy as np


def wasserstein_1d(
    samples_p: np.ndarray,
    samples_q: np.ndarray,
    grid_points: int = 256,
) -> float:
    """Empirical W1 distance between two 1D sample sets."""
    if samples_p.size == 0 or samples_q.size == 0:
        return 0.0

    p_sorted = np.sort(samples_p)
    q_sorted = np.sort(samples_q)
    lo = float(min(p_sorted.min(), q_sorted.min()))
    hi = float(max(p_sorted.max(), q_sorted.max()))
    if lo == hi:
        return 0.0

    grid = np.linspace(lo, hi, num=grid_points)
    cdf_p = np.searchsorted(p_sorted, grid, side='right') / len(p_sorted)
    cdf_q = np.searchsorted(q_sorted, grid, side='right') / len(q_sorted)
    return float(np.trapz(np.abs(cdf_p - cdf_q), grid))


def wasserstein_per_joint(
    baseline_samples: np.ndarray,
    current_samples: np.ndarray,
    grid_points: int = 256,
) -> list[float]:
    """Compute per-joint W1 distance between baseline (P) and current (Q) errors."""
    if baseline_samples.size == 0 or current_samples.size == 0:
        return []

    n_joints = baseline_samples.shape[1]
    return [
        wasserstein_1d(baseline_samples[:, j], current_samples[:, j], grid_points=grid_points)
        for j in range(n_joints)
    ]
