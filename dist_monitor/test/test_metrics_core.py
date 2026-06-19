"""Unit tests for shared metrics computation."""

import numpy as np

from dist_monitor.metrics_core import MetricsConfig, compute_distribution_metrics


def _make_streams(n: int = 200, offset: float = 0.0, delay: float = 0.0):
    ts = np.linspace(0.0, 2.0, n)
    pos = np.sin(ts)[:, None] * np.ones((n, 2))
    vel = np.cos(ts)[:, None] * np.ones((n, 2))
    sim = np.hstack([pos, vel])
    real = np.hstack([pos + offset, vel])
    real_ts = ts + delay
    return ts, sim, real_ts, real


def test_metrics_identical_near_zero():
    sim_ts, sim, real_ts, real = _make_streams()
    cfg = MetricsConfig(min_samples=50, mmd_permutation_count=30)
    result = compute_distribution_metrics(sim_ts, sim, real_ts, real, cfg=cfg)
    assert result.sample_count >= 50
    assert result.kl_mean < 0.05
    assert result.mmd_statistic < 0.05
    assert result.shift_detected is False


def test_metrics_shift_detected_with_offset():
    sim_ts, sim, real_ts, real = _make_streams(offset=0.8)
    baseline_ts, _, base_real_ts, base_real = _make_streams(offset=0.0)
    baseline_errors = sim[:, :2] - base_real[:, :2]

    cfg = MetricsConfig(min_samples=50, mmd_permutation_count=50)
    result = compute_distribution_metrics(
        sim_ts,
        sim,
        real_ts,
        real,
        baseline_errors=baseline_errors,
        cfg=cfg,
    )
    assert result.kl_mean > 0.0
    assert result.mmd_statistic > 0.0
