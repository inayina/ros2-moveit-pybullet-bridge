"""Actuator delay buffer: apply position targets with configurable lag."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field


@dataclass
class ActuatorDelayBuffer:
    """Ring buffer of (sim_time, targets) for delayed command execution."""

    delay_sec: float = 0.0
    _history: deque[tuple[float, dict[str, float]]] = field(
        default_factory=lambda: deque(maxlen=2000),
    )
    _last_targets: dict[str, float] = field(default_factory=dict)

    def set_delay(self, delay_sec: float) -> None:
        self.delay_sec = max(0.0, delay_sec)

    def clear(self) -> None:
        self._history.clear()
        self._last_targets.clear()

    def push(self, sim_time_sec: float, targets: dict[str, float]) -> None:
        self._history.append((sim_time_sec, dict(targets)))
        self._last_targets = dict(targets)

    def sample(self, sim_time_sec: float) -> tuple[dict[str, float], float]:
        """Return delayed targets and the sim-time they correspond to."""
        if not self._history:
            return dict(self._last_targets), sim_time_sec

        target_time = sim_time_sec - self.delay_sec
        best_time, best_targets = self._history[0]
        best_delta = abs(best_time - target_time)

        for t, targets in self._history:
            delta = abs(t - target_time)
            if delta < best_delta:
                best_delta = delta
                best_time = t
                best_targets = targets

        return dict(best_targets), best_time
