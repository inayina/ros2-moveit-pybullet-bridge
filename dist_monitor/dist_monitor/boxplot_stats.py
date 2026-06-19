"""Per-joint boxplot statistics from aligned sample windows."""

from __future__ import annotations

import numpy as np


def boxplot_per_joint(samples: np.ndarray) -> tuple[list[float], ...]:
    """Return min, q1, median, q3, max lists for each joint column."""
    if samples.size == 0:
        return [], [], [], [], []

    mins: list[float] = []
    q1s: list[float] = []
    medians: list[float] = []
    q3s: list[float] = []
    maxs: list[float] = []
    for j in range(samples.shape[1]):
        col = samples[:, j]
        mins.append(float(np.min(col)))
        q1s.append(float(np.percentile(col, 25)))
        medians.append(float(np.median(col)))
        q3s.append(float(np.percentile(col, 75)))
        maxs.append(float(np.max(col)))
    return mins, q1s, medians, q3s, maxs
