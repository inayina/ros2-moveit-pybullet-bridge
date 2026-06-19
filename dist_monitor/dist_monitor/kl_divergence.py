"""KL divergence computation for per-joint error distributions."""

from __future__ import annotations

import numpy as np


def histogram_probs(
    samples: np.ndarray,
    bins: int = 50,
    alpha: float = 1e-6,
    bin_range: tuple[float, float] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Build a smoothed normalized histogram; returns (probs, bin_edges)."""
    if samples.size == 0:
        probs = np.full(bins, 1.0 / bins)
        edges = np.linspace(0.0, 1.0, bins + 1) if bin_range is None else np.linspace(
            bin_range[0], bin_range[1], bins + 1
        )
        return probs, edges

    if bin_range is None:
        counts, edges = np.histogram(samples, bins=bins)
    else:
        counts, edges = np.histogram(samples, bins=bins, range=bin_range)

    probs = (counts.astype(float) + alpha) / (counts.sum() + alpha * bins)
    return probs, edges


def kl_divergence(p: np.ndarray, q: np.ndarray) -> float:
    """Compute KL(P || Q) for discrete distributions."""
    p = np.clip(p, 1e-10, None)
    q = np.clip(q, 1e-10, None)
    return float(np.sum(p * np.log(p / q)))


def kl_per_joint(
    baseline_samples: np.ndarray,
    current_samples: np.ndarray,
    bins: int = 50,
) -> list[float]:
    """Compute per-joint KL(P || Q) with shared bin edges per joint."""
    if baseline_samples.size == 0 or current_samples.size == 0:
        return []

    n_joints = baseline_samples.shape[1]
    results: list[float] = []
    for j in range(n_joints):
        p_samples = baseline_samples[:, j]
        q_samples = current_samples[:, j]
        combined = np.concatenate([p_samples, q_samples])
        bin_range = (float(combined.min()), float(combined.max()))
        if bin_range[0] == bin_range[1]:
            bin_range = (bin_range[0] - 0.5, bin_range[1] + 0.5)

        p, _ = histogram_probs(p_samples, bins=bins, bin_range=bin_range)
        q, _ = histogram_probs(q_samples, bins=bins, bin_range=bin_range)
        results.append(kl_divergence(p, q))
    return results
