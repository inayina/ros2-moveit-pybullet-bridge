"""Combined KL/W1/MMD shift detection logic."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ShiftDetectorConfig:
    use_kl: bool = True
    use_w1: bool = True
    use_mmd: bool = True
    kl_threshold_mean: float = 0.15
    w1_threshold_mean: float = 0.08
    mmd_threshold: float = 0.05
    mmd_p_value_alpha: float = 0.05


def detect_shift(
    kl_mean: float,
    w1_mean: float,
    mmd_stat: float,
    mmd_p: float,
    cfg: ShiftDetectorConfig,
) -> tuple[bool, str, bool]:
    """Return (shift_detected, detection_method, shift_detected_w1)."""
    kl_flag = cfg.use_kl and kl_mean > cfg.kl_threshold_mean
    w1_flag = cfg.use_w1 and w1_mean > cfg.w1_threshold_mean
    mmd_flag = (
        cfg.use_mmd
        and mmd_p < cfg.mmd_p_value_alpha
        and mmd_stat > cfg.mmd_threshold
    )

    methods: list[str] = []
    if kl_flag:
        methods.append('kl')
    if w1_flag:
        methods.append('w1')
    if mmd_flag:
        methods.append('mmd')

    if not methods:
        return False, 'none', False
    if len(methods) == 1:
        return True, methods[0], w1_flag
    return True, '+'.join(methods), w1_flag
