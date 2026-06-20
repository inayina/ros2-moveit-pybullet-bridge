"""R2 degraded-mode trajectory time scaling."""


def trajectory_time_scale(*, degraded: bool, scale: float) -> float:
    """Return multiplier applied to trajectory elapsed time (0 < scale <= 1)."""
    if not degraded:
        return 1.0
    return max(min(float(scale), 1.0), 0.05)
