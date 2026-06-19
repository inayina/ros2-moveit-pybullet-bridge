"""Unit tests for timestamp alignment."""

import numpy as np

from dist_monitor.time_aligner import TimeAligner


def test_align_exact_timestamps():
    aligner = TimeAligner(tolerance_sec=0.02)
    sim_ts = np.array([0.0, 0.1, 0.2])
    real_ts = np.array([0.0, 0.1, 0.2])
    sim_vals = np.array([[1.0], [2.0], [3.0]])
    real_vals = np.array([[1.1], [2.1], [3.1]])

    sim, real = aligner.align_arrays(sim_ts, sim_vals, real_ts, real_vals)
    assert sim.shape == (3, 1)
    assert np.allclose(sim[:, 0], [1.0, 2.0, 3.0])
    assert np.allclose(real[:, 0], [1.1, 2.1, 3.1])


def test_align_timestamp_jitter_small_error():
    aligner = TimeAligner(tolerance_sec=0.02)
    sim_ts = np.linspace(0.0, 1.0, 101)
    jitter = np.random.default_rng(0).uniform(-0.005, 0.005, size=len(sim_ts))
    real_ts = sim_ts + jitter
    sim_vals = np.full((len(sim_ts), 1), 0.5)
    real_vals = sim_vals.copy()

    sim, real = aligner.align_arrays(sim_ts, sim_vals, real_ts, real_vals)
    assert len(sim) == len(sim_ts)
    assert np.max(np.abs(sim - real)) < 0.001


def test_align_drops_out_of_tolerance():
    aligner = TimeAligner(tolerance_sec=0.02)
    sim_ts = np.array([0.0, 1.0])
    real_ts = np.array([0.5])
    sim_vals = np.array([[1.0], [2.0]])
    real_vals = np.array([[1.5]])

    sim, real = aligner.align_arrays(sim_ts, sim_vals, real_ts, real_vals)
    assert len(sim) == 0
