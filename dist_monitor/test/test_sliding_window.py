"""Unit tests for sliding window buffer."""

import numpy as np

from dist_monitor.sliding_window import SlidingWindow


def test_push_and_get_array():
    window = SlidingWindow(duration_sec=1.0, max_freq_hz=10.0)
    window.push(0.0, np.array([1.0, 2.0]))
    window.push(0.1, np.array([3.0, 4.0]))
    arr = window.get_samples()
    assert arr.shape == (2, 2)
    assert np.allclose(arr[0], [1.0, 2.0])
    assert np.allclose(arr[1], [3.0, 4.0])
    assert window.get_timestamps().shape == (2,)


def test_expires_old_samples():
    window = SlidingWindow(duration_sec=1.0, max_freq_hz=10.0)
    window.push(0.0, np.array([1.0]))
    window.push(0.5, np.array([2.0]))
    window.push(1.5, np.array([3.0]))
    assert window.count() == 2
    arr = window.get_array()
    assert arr.shape == (2, 1)
    assert arr[0, 0] == 2.0
    assert arr[1, 0] == 3.0


def test_empty_window():
    window = SlidingWindow(duration_sec=1.0)
    assert window.count() == 0
    assert window.get_array().size == 0


def test_clear():
    window = SlidingWindow(duration_sec=1.0)
    window.push(0.0, np.array([1.0]))
    window.clear()
    assert window.count() == 0
