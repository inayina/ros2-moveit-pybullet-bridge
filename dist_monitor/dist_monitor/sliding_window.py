"""Fixed-duration sliding window buffer."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np


@dataclass
class WindowSample:
    timestamp: float
    value: np.ndarray


class SlidingWindow:
    """Keep samples within a fixed time duration."""

    def __init__(self, duration_sec: float, max_freq_hz: float = 100.0) -> None:
        self.duration = duration_sec
        self.max_samples = int(duration_sec * max_freq_hz)
        self._samples: deque[WindowSample] = deque(maxlen=self.max_samples)

    def push(self, timestamp: float, value: np.ndarray) -> None:
        self._samples.append(WindowSample(timestamp=timestamp, value=value))
        cutoff = timestamp - self.duration
        while self._samples and self._samples[0].timestamp < cutoff:
            self._samples.popleft()

    def get_array(self) -> np.ndarray:
        if not self._samples:
            return np.empty((0, 0))
        return np.vstack([s.value for s in self._samples])

    def count(self) -> int:
        return len(self._samples)

    def clear(self) -> None:
        self._samples.clear()
