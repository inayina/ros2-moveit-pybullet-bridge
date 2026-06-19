"""Unit tests for shift detection logic."""

from dist_monitor.shift_detector import ShiftDetectorConfig, detect_shift


def test_detect_shift_kl_only():
    cfg = ShiftDetectorConfig(use_kl=True, use_w1=False, use_mmd=False, kl_threshold_mean=0.15)
    detected, method, w1_flag = detect_shift(0.2, 0.0, 0.0, 1.0, cfg)
    assert detected is True
    assert method == 'kl'
    assert w1_flag is False


def test_detect_shift_w1_only():
    cfg = ShiftDetectorConfig(use_kl=False, use_w1=True, use_mmd=False, w1_threshold_mean=0.08)
    detected, method, w1_flag = detect_shift(0.0, 0.1, 0.0, 1.0, cfg)
    assert detected is True
    assert method == 'w1'
    assert w1_flag is True


def test_detect_shift_mmd_only():
    cfg = ShiftDetectorConfig(use_kl=False, use_w1=False, use_mmd=True, mmd_threshold=0.05)
    detected, method, w1_flag = detect_shift(0.0, 0.0, 0.1, 0.01, cfg)
    assert detected is True
    assert method == 'mmd'
    assert w1_flag is False


def test_detect_shift_multiple():
    cfg = ShiftDetectorConfig()
    detected, method, w1_flag = detect_shift(0.2, 0.1, 0.1, 0.01, cfg)
    assert detected is True
    assert set(method.split('+')) == {'kl', 'w1', 'mmd'}
    assert w1_flag is True


def test_detect_shift_none():
    cfg = ShiftDetectorConfig()
    detected, method, w1_flag = detect_shift(0.01, 0.01, 0.01, 0.5, cfg)
    assert detected is False
    assert method == 'none'
    assert w1_flag is False
