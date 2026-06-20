"""Unit tests for degraded-mode time scaling."""

from pybullet_bridge.degraded_mode import trajectory_time_scale


def test_trajectory_time_scale_normal():
    assert trajectory_time_scale(degraded=False, scale=0.5) == 1.0


def test_trajectory_time_scale_degraded():
    assert trajectory_time_scale(degraded=True, scale=0.5) == 0.5


def test_trajectory_time_scale_clamps():
    assert trajectory_time_scale(degraded=True, scale=2.0) == 1.0
    assert trajectory_time_scale(degraded=True, scale=0.0) == 0.05
