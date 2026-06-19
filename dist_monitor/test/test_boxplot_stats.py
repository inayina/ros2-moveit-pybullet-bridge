"""Tests for boxplot statistics helper."""

import numpy as np

from dist_monitor.boxplot_stats import boxplot_per_joint


def test_boxplot_per_joint_uniform_column():
    samples = np.full((10, 2), 0.5)
    mins, q1s, medians, q3s, maxs = boxplot_per_joint(samples)
    assert mins == [0.5, 0.5]
    assert q1s == [0.5, 0.5]
    assert medians == [0.5, 0.5]
    assert q3s == [0.5, 0.5]
    assert maxs == [0.5, 0.5]


def test_boxplot_per_joint_empty():
    samples = np.empty((0, 3))
    assert boxplot_per_joint(samples) == ([], [], [], [], [])


def test_boxplot_per_joint_known_range():
    samples = np.array([[0.0, 10.0], [1.0, 20.0], [2.0, 30.0], [3.0, 40.0]])
    mins, q1s, medians, q3s, maxs = boxplot_per_joint(samples)
    assert mins == [0.0, 10.0]
    assert maxs == [3.0, 40.0]
    assert medians == [1.5, 25.0]
