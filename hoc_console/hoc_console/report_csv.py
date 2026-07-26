"""CSV experiment report export."""

from __future__ import annotations

import csv
from io import StringIO
from typing import Any


def render_csv_report(
    *,
    risk_timeline: list[dict[str, Any]],
    metrics_timeline: list[dict[str, Any]],
) -> str:
    """Merge risk and metrics timelines into a CSV string."""
    buf = StringIO()
    writer = csv.writer(buf)
    writer.writerow([
        't',
        'risk_level',
        'composite_score',
        'kl_mean',
        'w1_mean',
        'mmd_stat',
        'shift_detected',
        'metric_valid',
        'validity',
        'reason_code',
        'calibration_id',
    ])

    risks = list(risk_timeline)

    def metric_value(payload: dict[str, Any], key: str) -> str:
        if not payload.get('metric_valid', False):
            return ''
        value = payload.get(key)
        return '' if value is None else f'{float(value):.4f}'

    for idx, metrics in enumerate(metrics_timeline):
        risk = risks[idx] if idx < len(risks) else (risks[-1] if risks else {})
        writer.writerow([
            f"{float(metrics.get('t', 0.0)):.3f}",
            int(risk.get('level', 0)),
            f"{float(risk.get('score', 0.0)):.4f}",
            metric_value(metrics, 'kl_mean'),
            metric_value(metrics, 'w1_mean'),
            metric_value(metrics, 'mmd_stat'),
            metrics.get('shift_detected'),
            bool(metrics.get('metric_valid', False)),
            metrics.get('validity', 'UNAVAILABLE'),
            metrics.get('reason_code', 'no_data'),
            metrics.get('calibration_id', ''),
        ])

    return buf.getvalue()
