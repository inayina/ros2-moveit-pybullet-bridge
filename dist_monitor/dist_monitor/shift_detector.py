"""Combined KL/MMD shift detection logic."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ShiftDetectorConfig:
    use_kl: bool = True
    use_mmd: bool = True
    kl_threshold_mean: float = 0.15
    mmd_threshold: float = 0.05
    mmd_p_value_alpha: float = 0.05


def detect_shift(
    kl_mean: float,
    mmd_stat: float,
    mmd_p: float,
    cfg: ShiftDetectorConfig,
) -> tuple[bool, str]:
    """Return (shift_detected, detection_method)."""
    kl_flag = cfg.use_kl and kl_mean > cfg.kl_threshold_mean
    mmd_flag = (
        cfg.use_mmd
        and mmd_p < cfg.mmd_p_value_alpha
        and mmd_stat > cfg.mmd_threshold
    )

    if kl_flag and mmd_flag:
        return True, 'both'
    if kl_flag:
        return True, 'kl'
    if mmd_flag:
        return True, 'mmd'
    return False, 'none'
