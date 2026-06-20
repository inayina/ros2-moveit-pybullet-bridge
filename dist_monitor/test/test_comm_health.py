"""Unit tests for communication health monitoring."""

from __future__ import annotations

import time

from dist_monitor.comm_health import CommHealthMonitor


def test_comm_health_score_near_zero_at_expected_rate():
    monitor = CommHealthMonitor(
        expected_sim_hz=100.0,
        expected_real_hz=100.0,
        gap_multiplier=2.0,
        latency_threshold_ms=100.0,
        max_samples=20,
    )
    start = time.monotonic()
    for i in range(25):
        monitor.record('/bridge/sim/joint_states', start + i * 0.01)
    assert monitor.aggregate_score() < 0.15


def test_comm_health_detects_gaps():
    monitor = CommHealthMonitor(
        expected_sim_hz=10.0,
        expected_real_hz=10.0,
        gap_multiplier=2.0,
        latency_threshold_ms=50.0,
        max_samples=10,
    )
    start = time.monotonic()
    for i in range(8):
        monitor.record('/bridge/sim/joint_states', start + i * 0.2)
    score = monitor.aggregate_score()
    assert score > 0.3


def test_diagnostic_array_contains_aggregate():
    monitor = CommHealthMonitor()
    start = time.monotonic()
    for i in range(5):
        monitor.record('/bridge/sim/joint_states', start + i * 0.01)
        monitor.record('/bridge/real/joint_states', start + i * 0.01 + 0.001)

    from builtin_interfaces.msg import Time

    stamp = Time(sec=1, nanosec=0)
    diag = monitor.to_diagnostic_array(stamp)
    assert len(diag.status) >= 3
    names = {s.name for s in diag.status}
    assert 'comm_health_aggregate' in names
