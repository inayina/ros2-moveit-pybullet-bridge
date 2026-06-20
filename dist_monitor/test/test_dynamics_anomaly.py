"""Unit tests for dynamics anomaly scoring."""

from __future__ import annotations

import numpy as np

from dist_monitor.dynamics_anomaly import DynamicsAnomalyConfig, compute_dynamics_anomaly


def test_dynamics_anomaly_detects_velocity_jump():
    n = 20
    n_dof = 2
    sim = np.zeros((n, 2 * n_dof))
    real = np.zeros((n, 2 * n_dof))
    sim[10, n_dof] = 5.0
    sim[11, n_dof] = 0.0
    errors = np.zeros((n, n_dof))
    result = compute_dynamics_anomaly(
        sim, real, errors, None, cfg=DynamicsAnomalyConfig(velocity_jump_threshold=2.0),
    )
    assert result.score > 0.5
    assert len(result.velocity_jump_per_joint) == n_dof


def test_dynamics_anomaly_error_spike():
    n = 10
    n_dof = 2
    sim = np.zeros((n, 2 * n_dof))
    real = np.zeros((n, 2 * n_dof))
    errors = np.random.default_rng(0).normal(0, 0.01, size=(n, n_dof))
    baseline = errors.copy()
    errors[-1, 0] = 0.5
    result = compute_dynamics_anomaly(
        sim,
        real,
        errors,
        baseline,
        cfg=DynamicsAnomalyConfig(error_spike_sigma=3.0),
    )
    assert result.score > 0.3


def test_dynamics_anomaly_empty_returns_zero():
    result = compute_dynamics_anomaly(
        np.empty((0, 4)),
        np.empty((0, 4)),
        np.empty((0, 2)),
        None,
    )
    assert result.score == 0.0
