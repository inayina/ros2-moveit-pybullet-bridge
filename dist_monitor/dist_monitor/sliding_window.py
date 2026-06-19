"""Fixed-duration sliding window buffer."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class WindowSample:
    timestamp: float
    value: np.ndarray


class SlidingWindow:
    """Keep samples within a fixed time duration (default 5s @ 100Hz → 500 samples)."""

    def __init__(self, duration_sec: float, max_freq_hz: float = 100.0) -> None:
        self.duration = duration_sec
        self.max_samples = int(duration_sec * max_freq_hz)
        self._samples: deque[WindowSample] = deque(maxlen=self.max_samples)

    def push(self, timestamp: float, value: np.ndarray) -> None:
        self._samples.append(WindowSample(timestamp=timestamp, value=np.asarray(value, dtype=float)))
        cutoff = timestamp - self.duration
        while self._samples and self._samples[0].timestamp < cutoff:
            self._samples.popleft()

    def push_batch(self, timestamps: np.ndarray, values: np.ndarray) -> None:
        for t, v in zip(timestamps, values):
            self.push(float(t), v)

    def get_samples(self) -> np.ndarray:
        """Return shape (N, D) array of stored values."""
        if not self._samples:
            return np.empty((0, 0))
        return np.vstack([s.value for s in self._samples])

    def get_array(self) -> np.ndarray:
        """Alias for get_samples() (backward compatible)."""
        return self.get_samples()

    def get_timestamps(self) -> np.ndarray:
        if not self._samples:
            return np.empty(0)
        return np.array([s.timestamp for s in self._samples], dtype=float)

    def count(self) -> int:
        return len(self._samples)

    def clear(self) -> None:
        self._samples.clear()
