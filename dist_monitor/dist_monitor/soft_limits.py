"""Soft joint-limit proximity checks for risk monitoring."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class JointLimit:
    lower: float
    upper: float


@dataclass
class SoftLimitResult:
    score: float = 0.0
    triggered: bool = False
    joints_near_limit: list[str] = field(default_factory=list)


def load_joint_limits(path: Path, profile: str) -> dict[str, JointLimit]:
    """Load per-profile joint limits from YAML."""
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    profile_data = data.get(profile, {})
    limits: dict[str, JointLimit] = {}
    for name, spec in profile_data.items():
        if not isinstance(spec, dict):
            continue
        limits[str(name)] = JointLimit(
            lower=float(spec['lower']),
            upper=float(spec['upper']),
        )
    return limits


def compute_soft_limit_proximity(
    joint_names: list[str],
    positions: np.ndarray,
    limits: dict[str, JointLimit],
    *,
    proximity_ratio: float = 0.95,
) -> SoftLimitResult:
    """Return score in [0, 1]; triggered when position exceeds ratio of URDF span."""
    result = SoftLimitResult()
    if not joint_names or positions.size == 0:
        return result

    ratio = float(np.clip(proximity_ratio, 0.5, 0.999))
    margin_frac = (1.0 - ratio) / 2.0
    warn_frac = margin_frac * 2.0

    max_score = 0.0
    for idx, name in enumerate(joint_names):
        if idx >= len(positions):
            break
        lim = limits.get(name)
        if lim is None:
            continue
        span = lim.upper - lim.lower
        if span <= 0:
            continue
        frac = (float(positions[idx]) - lim.lower) / span
        if frac <= margin_frac or frac >= (1.0 - margin_frac):
            result.joints_near_limit.append(name)
            max_score = max(max_score, 1.0)
        elif frac <= warn_frac or frac >= (1.0 - warn_frac):
            max_score = max(max_score, 0.5)

    result.score = max_score
    result.triggered = bool(result.joints_near_limit)
    return result
