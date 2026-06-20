"""Unit tests for soft joint-limit proximity."""

from __future__ import annotations

import numpy as np

from dist_monitor.soft_limits import JointLimit, compute_soft_limit_proximity


def test_soft_limit_not_triggered_at_center():
    limits = {'j1': JointLimit(lower=-2.0, upper=2.0)}
    result = compute_soft_limit_proximity(['j1'], np.array([0.0]), limits)
    assert not result.triggered
    assert result.score < 0.2


def test_soft_limit_triggered_near_upper():
    limits = {'j1': JointLimit(lower=-2.0, upper=2.0)}
    result = compute_soft_limit_proximity(['j1'], np.array([1.95]), limits, proximity_ratio=0.95)
    assert result.triggered
    assert result.score >= 1.0
    assert 'j1' in result.joints_near_limit


def test_soft_limit_triggered_near_lower():
    limits = {'j1': JointLimit(lower=-3.0, upper=3.0)}
    result = compute_soft_limit_proximity(['j1'], np.array([-2.85]), limits, proximity_ratio=0.95)
    assert result.triggered
