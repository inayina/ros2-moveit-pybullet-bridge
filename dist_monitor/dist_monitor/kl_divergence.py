"""KL divergence computation for per-joint error distributions."""

from __future__ import annotations

import numpy as np


def histogram_probs(samples: np.ndarray, bins: int = 30, alpha: float = 1e-6) -> np.ndarray:
    """Build a smoothed normalized histogram."""
    if samples.size == 0:
        return np.full(bins, 1.0 / bins)
    counts, _ = np.histogram(samples, bins=bins)
    probs = (counts.astype(float) + alpha) / (counts.sum() + alpha * bins)
    return probs


def kl_divergence(p: np.ndarray, q: np.ndarray) -> float:
    """Compute KL(P || Q) for discrete distributions."""
    p = np.clip(p, 1e-10, None)
    q = np.clip(q, 1e-10, None)
    return float(np.sum(p * np.log(p / q)))


def kl_per_joint(
    baseline_samples: np.ndarray,
    current_samples: np.ndarray,
    bins: int = 30,
) -> list[float]:
    """Compute per-joint KL divergence between baseline and current error windows."""
    if baseline_samples.size == 0 or current_samples.size == 0:
        return []

    n_joints = baseline_samples.shape[1]
    results: list[float] = []
    for j in range(n_joints):
        p = histogram_probs(baseline_samples[:, j], bins=bins)
        q = histogram_probs(current_samples[:, j], bins=bins)
        results.append(kl_divergence(p, q))
    return results
