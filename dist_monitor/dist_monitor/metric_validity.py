"""Validity-first decisions for online distribution metrics."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MetricValidity:
    validity: str
    reason_code: str
    metric_valid: bool
    baseline_ready: bool


def evaluate_metric_validity(
    *,
    aligned_samples: int,
    min_samples: int,
    baseline_ready: bool,
    calibration_id: str,
    reference_source: str,
    sources_fresh: bool,
    sources_seen: bool = True,
    allow_lerobot_reference: bool = False,
) -> MetricValidity:
    """Return a fail-closed distribution metric validity decision."""
    if not sources_seen:
        return MetricValidity('UNAVAILABLE', 'no_data', False, baseline_ready)
    if not sources_fresh:
        return MetricValidity('STALE', 'source_stale', False, baseline_ready)
    if aligned_samples < min_samples:
        return MetricValidity(
            'WARMING_UP', 'insufficient_aligned_samples', False, baseline_ready
        )
    if not baseline_ready:
        return MetricValidity(
            'WARMING_UP', 'baseline_warming_up', False, False
        )
    if not calibration_id.strip():
        return MetricValidity(
            'UNAVAILABLE', 'calibration_missing', False, True
        )
    if reference_source == 'lerobot' and not allow_lerobot_reference:
        return MetricValidity(
            'UNAVAILABLE', 'reference_not_runtime_calibration', False, True
        )
    return MetricValidity('VALID', 'none', True, True)
