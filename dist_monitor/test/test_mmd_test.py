"""Unit tests for MMD statistic and permutation test."""

import numpy as np

from dist_monitor.mmd_test import (
    mmd_rbf,
    median_heuristic_gamma,
    permutation_test,
)


def test_mmd_identical_samples_near_zero():
    rng = np.random.default_rng(0)
    x = rng.normal(size=(50, 3))
    gamma = median_heuristic_gamma(x, x)
    assert mmd_rbf(x, x, gamma) < 1e-6


def test_mmd_different_samples_positive():
    rng = np.random.default_rng(0)
    x = rng.normal(loc=0.0, scale=1.0, size=(50, 3))
    y = rng.normal(loc=3.0, scale=1.0, size=(50, 3))
    gamma = median_heuristic_gamma(x, y)
    assert mmd_rbf(x, y, gamma) > 0.01


def test_mmd_empty_returns_zero():
    x = np.empty((0, 2))
    y = np.array([[1.0, 2.0]])
    assert mmd_rbf(x, y, gamma=1.0) == 0.0
    assert mmd_rbf(y, x, gamma=1.0) == 0.0


def test_median_heuristic_gamma_fallback():
    single = np.array([[0.0, 0.0]])
    assert median_heuristic_gamma(single, single) == 1.0


def test_permutation_test_identical_high_p_value():
    rng = np.random.default_rng(1)
    x = rng.normal(size=(30, 2))
    y = x.copy()
    gamma = median_heuristic_gamma(x, y)
    _, p_value = permutation_test(x, y, gamma, n_permutations=50)
    assert p_value > 0.5


def test_permutation_test_shifted_low_p_value():
    rng = np.random.default_rng(2)
    x = rng.normal(loc=0.0, size=(40, 2))
    y = rng.normal(loc=5.0, size=(40, 2))
    gamma = median_heuristic_gamma(x, y)
    stat, p_value = permutation_test(x, y, gamma, n_permutations=100)
    assert stat > 0.05
    assert p_value < 0.05
