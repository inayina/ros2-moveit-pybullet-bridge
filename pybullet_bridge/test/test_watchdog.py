"""Unit tests for watchdog helper."""

from pybullet_bridge.watchdog import should_trip_watchdog


def test_watchdog_trips_on_active_stall():
    assert should_trip_watchdog(has_active_trajectory=True, idle_ms=600.0, timeout_ms=500.0)


def test_watchdog_idle_without_trajectory():
    assert not should_trip_watchdog(has_active_trajectory=False, idle_ms=600.0, timeout_ms=500.0)


def test_watchdog_within_timeout():
    assert not should_trip_watchdog(has_active_trajectory=True, idle_ms=200.0, timeout_ms=500.0)
