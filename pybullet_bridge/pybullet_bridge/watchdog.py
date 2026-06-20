"""Watchdog helpers for command timeout → HOLD."""


def should_trip_watchdog(
    *,
    has_active_trajectory: bool,
    idle_ms: float,
    timeout_ms: float,
) -> bool:
    """True when an active trajectory stalls longer than the watchdog timeout."""
    return has_active_trajectory and idle_ms > timeout_ms
