"""Unit tests for KL divergence helpers."""

import numpy as np

from dist_monitor.kl_divergence import histogram_probs, kl_divergence, kl_per_joint


def test_histogram_probs_uniform_when_empty():
    probs, _ = histogram_probs(np.array([]), bins=10)
    assert probs.shape == (10,)
    assert np.isclose(probs.sum(), 1.0)
    assert np.allclose(probs, 0.1)


def test_kl_divergence_identical_near_zero():
    p = np.array([0.25, 0.25, 0.25, 0.25])
    assert kl_divergence(p, p) < 1e-9


def test_kl_divergence_asymmetric():
    p = np.array([0.9, 0.1])
    q = np.array([0.5, 0.5])
    forward = kl_divergence(p, q)
    reverse = kl_divergence(q, p)
    assert forward > 0.0
    assert reverse > 0.0
    assert forward != reverse


def test_kl_per_joint_shape_and_shift():
    rng = np.random.default_rng(0)
    baseline = rng.normal(size=(200, 3))
    shifted = rng.normal(loc=1.0, size=(200, 3))
    results = kl_per_joint(baseline, shifted, bins=20)
    assert len(results) == 3
    assert all(kl >= 0.0 for kl in results)
    assert max(results) > 0.01


def test_kl_per_joint_empty_returns_empty():
    baseline = np.empty((0, 2))
    current = np.array([[0.1, 0.2]])
    assert kl_per_joint(baseline, current) == []
