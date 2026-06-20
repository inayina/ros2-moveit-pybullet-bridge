"""Unit tests for planning statistics collector."""

from __future__ import annotations

from risk_engine.planning_stats import PlanningStatsCollector


def test_failure_rate_empty():
    collector = PlanningStatsCollector(window_size=10)
    assert collector.failure_rate() == 0.0
    assert collector.last_error() == ''


def test_failure_rate_mixed():
    collector = PlanningStatsCollector(window_size=5)
    collector.record(success=True, action='pick', message='ok')
    collector.record(success=False, action='pick', message='approach failed')
    collector.record(success=True, action='place', message='ok')
    assert abs(collector.failure_rate() - 1 / 3) < 1e-6
    assert 'approach failed' in collector.last_error()


def test_window_truncates():
    collector = PlanningStatsCollector(window_size=2)
    collector.record(success=False, action='pick', message='a')
    collector.record(success=True, action='pick', message='b')
    collector.record(success=True, action='pick', message='c')
    assert collector.sample_count == 2
    assert collector.failure_rate() == 0.0
