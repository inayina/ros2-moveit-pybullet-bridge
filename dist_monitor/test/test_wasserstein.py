"""Unit tests for Wasserstein-1 helpers."""

import numpy as np

from dist_monitor.wasserstein import wasserstein_1d, wasserstein_per_joint


def test_wasserstein_1d_identical_near_zero():
    rng = np.random.default_rng(0)
    samples = rng.normal(size=200)
    assert wasserstein_1d(samples, samples) < 1e-9


def test_wasserstein_1d_shifted():
    rng = np.random.default_rng(0)
    baseline = rng.normal(size=200)
    shifted = rng.normal(loc=0.5, size=200)
    distance = wasserstein_1d(baseline, shifted)
    assert distance > 0.3


def test_wasserstein_1d_empty_returns_zero():
    samples = np.array([0.1, 0.2, 0.3])
    assert wasserstein_1d(samples, np.array([])) == 0.0
    assert wasserstein_1d(np.array([]), samples) == 0.0


def test_wasserstein_per_joint_shape_and_shift():
    rng = np.random.default_rng(0)
    baseline = rng.normal(size=(200, 3))
    shifted = rng.normal(loc=0.4, size=(200, 3))
    results = wasserstein_per_joint(baseline, shifted)
    assert len(results) == 3
    assert all(w1 >= 0.0 for w1 in results)
    assert max(results) > 0.1


def test_wasserstein_per_joint_empty_returns_empty():
    baseline = np.empty((0, 2))
    current = np.array([[0.1, 0.2]])
    assert wasserstein_per_joint(baseline, current) == []
