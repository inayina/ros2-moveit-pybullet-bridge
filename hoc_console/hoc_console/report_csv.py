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
    ])

    risks = list(risk_timeline)
    for idx, metrics in enumerate(metrics_timeline):
        risk = risks[idx] if idx < len(risks) else (risks[-1] if risks else {})
        writer.writerow([
            f"{float(metrics.get('t', 0.0)):.3f}",
            int(risk.get('level', 0)),
            f"{float(risk.get('score', 0.0)):.4f}",
            f"{float(metrics.get('kl_mean', 0.0)):.4f}",
            f"{float(metrics.get('w1_mean', 0.0)):.4f}",
            f"{float(metrics.get('mmd_stat', 0.0)):.4f}",
            bool(metrics.get('shift_detected', False)),
        ])

    return buf.getvalue()
