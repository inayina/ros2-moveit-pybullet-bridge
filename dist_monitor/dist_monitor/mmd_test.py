"""MMD (Maximum Mean Discrepancy) with RBF kernel and permutation test."""

from __future__ import annotations

import numpy as np


def rbf_kernel(a: np.ndarray, b: np.ndarray, gamma: float) -> np.ndarray:
    """RBF kernel with bandwidth gamma: k(x,y) = exp(-||x-y||^2 / (2*gamma^2))."""
    sq_dists = (
        np.sum(a ** 2, axis=1, keepdims=True)
        + np.sum(b ** 2, axis=1)
        - 2.0 * a @ b.T
    )
    sq_dists = np.maximum(sq_dists, 0.0)
    return np.exp(-sq_dists / (2.0 * gamma ** 2))


def mmd_rbf(x: np.ndarray, y: np.ndarray, gamma: float) -> float:
    """Compute MMD^2 statistic between sample sets X and Y."""
    if len(x) == 0 or len(y) == 0:
        return 0.0
    xx = rbf_kernel(x, x, gamma)
    yy = rbf_kernel(y, y, gamma)
    xy = rbf_kernel(x, y, gamma)
    m, n = len(x), len(y)
    mmd2 = xx.sum() / (m * m) + yy.sum() / (n * n) - 2.0 * xy.sum() / (m * n)
    return float(max(mmd2, 0.0))


def median_pairwise_distance(x: np.ndarray, y: np.ndarray) -> float:
    """Median of non-zero pairwise Euclidean distances in the combined set."""
    combined = np.vstack([x, y])
    if len(combined) < 2:
        return 1.0
    dists = np.linalg.norm(combined[:, None] - combined[None, :], axis=2)
    nonzero = dists[dists > 0]
    if nonzero.size == 0:
        return 1.0
    return float(np.median(nonzero))


def bandwidth_from_median(x: np.ndarray, y: np.ndarray) -> float:
    """sigma = 2 * median distance; used as RBF bandwidth (gamma in kernel)."""
    combined = np.vstack([x, y])
    if len(combined) < 2:
        return 1.0
    dists = np.linalg.norm(combined[:, None] - combined[None, :], axis=2)
    nonzero = dists[dists > 0]
    if nonzero.size == 0:
        return 1.0
    return max(2.0 * float(np.median(nonzero)), 1e-6)


def median_heuristic_gamma(x: np.ndarray, y: np.ndarray) -> float:
    """Backward-compatible alias for bandwidth_from_median."""
    return bandwidth_from_median(x, y)


def subsample_pairs(
    x: np.ndarray,
    y: np.ndarray,
    max_samples: int = 100,
) -> tuple[np.ndarray, np.ndarray]:
    """Uniformly subsample paired rows when N exceeds max_samples."""
    n = min(len(x), len(y))
    if n <= max_samples:
        return x[:n], y[:n]
    idx = np.linspace(0, n - 1, max_samples, dtype=int)
    return x[idx], y[idx]


def _mmd2_from_kernel(k_xx: np.ndarray, k_yy: np.ndarray, k_xy: np.ndarray) -> float:
    m, n = len(k_xx), len(k_yy)
    mmd2 = k_xx.sum() / (m * m) + k_yy.sum() / (n * n) - 2.0 * k_xy.sum() / (m * n)
    return float(max(mmd2, 0.0))


def permutation_test(
    x: np.ndarray,
    y: np.ndarray,
    gamma: float,
    n_permutations: int = 100,
    max_samples: int = 100,
    rng: np.random.Generator | None = None,
) -> tuple[float, float]:
    """Return (mmd_statistic, p_value) via permutation test."""
    x, y = subsample_pairs(x, y, max_samples=max_samples)
    m, n = len(x), len(y)
    combined = np.vstack([x, y])
    kernel = rbf_kernel(combined, combined, gamma)
    observed = _mmd2_from_kernel(
        kernel[:m, :m],
        kernel[m:, m:],
        kernel[:m, m:],
    )

    count = 0
    gen = rng or np.random.default_rng(42)
    total = m + n
    for _ in range(n_permutations):
        perm = gen.permutation(total)
        x_idx = perm[:m]
        y_idx = perm[m:]
        perm_mmd = _mmd2_from_kernel(
            kernel[np.ix_(x_idx, x_idx)],
            kernel[np.ix_(y_idx, y_idx)],
            kernel[np.ix_(x_idx, y_idx)],
        )
        if perm_mmd >= observed:
            count += 1
    p_value = (count + 1) / (n_permutations + 1)
    return observed, p_value
