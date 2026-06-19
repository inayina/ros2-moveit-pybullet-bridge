"""MMD (Maximum Mean Discrepancy) with RBF kernel and permutation test."""

from __future__ import annotations

import numpy as np


def rbf_kernel(a: np.ndarray, b: np.ndarray, gamma: float) -> np.ndarray:
    sq_dists = (
        np.sum(a ** 2, axis=1, keepdims=True)
        + np.sum(b ** 2, axis=1)
        - 2.0 * a @ b.T
    )
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


def median_heuristic_gamma(x: np.ndarray, y: np.ndarray) -> float:
    combined = np.vstack([x, y])
    if len(combined) < 2:
        return 1.0
    dists = np.linalg.norm(combined[:, None] - combined[None, :], axis=2)
    nonzero = dists[dists > 0]
    if nonzero.size == 0:
        return 1.0
    return float(np.median(nonzero) / np.sqrt(2.0))


def permutation_test(
    x: np.ndarray,
    y: np.ndarray,
    gamma: float,
    n_permutations: int = 100,
) -> tuple[float, float]:
    """Return (mmd_statistic, p_value) via permutation test."""
    observed = mmd_rbf(x, y, gamma)
    combined = np.vstack([x, y])
    m, n = len(x), len(y)
    count = 0
    rng = np.random.default_rng(42)
    for _ in range(n_permutations):
        perm = rng.permutation(len(combined))
        x_perm = combined[perm[:m]]
        y_perm = combined[perm[m:m + n]]
        if mmd_rbf(x_perm, y_perm, gamma) >= observed:
            count += 1
    p_value = (count + 1) / (n_permutations + 1)
    return observed, p_value
