"""Unit tests for shift detection logic."""

from dist_monitor.shift_detector import ShiftDetectorConfig, detect_shift


def test_detect_shift_kl_only():
    cfg = ShiftDetectorConfig(use_kl=True, use_mmd=False, kl_threshold_mean=0.15)
    detected, method = detect_shift(0.2, 0.0, 1.0, cfg)
    assert detected is True
    assert method == 'kl'


def test_detect_shift_mmd_only():
    cfg = ShiftDetectorConfig(use_kl=False, use_mmd=True, mmd_threshold=0.05)
    detected, method = detect_shift(0.0, 0.1, 0.01, cfg)
    assert detected is True
    assert method == 'mmd'


def test_detect_shift_both():
    cfg = ShiftDetectorConfig()
    detected, method = detect_shift(0.2, 0.1, 0.01, cfg)
    assert detected is True
    assert method == 'both'


def test_detect_shift_none():
    cfg = ShiftDetectorConfig()
    detected, method = detect_shift(0.01, 0.01, 0.5, cfg)
    assert detected is False
    assert method == 'none'
