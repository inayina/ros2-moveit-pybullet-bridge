#!/usr/bin/env python3
"""Generate a sample HOC experiment HTML report for docs/samples/."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'hoc_console'))
OUT = ROOT / 'docs' / 'samples' / 'sample-experiment-report.html'

from hoc_console.report_html import render_html_report


def main() -> None:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    html = render_html_report(
        experiment_id='exp_20260619_sample',
        metadata={
            'scenario_id': 'SC-01',
            'random_seed': 42,
            'randomization_strength': 0.5,
            'duration_sec': 120.5,
        },
        summary={
            'max_risk_level': 2,
            'max_composite_score': 0.68,
            'shift_detected_count': 15,
            'shift_detected_ratio': 0.125,
            'mean_kl': 0.18,
            'max_kl': 0.91,
            'mean_mmd': 0.04,
            'max_mmd': 0.08,
        },
        risk_timeline=[
            {'t': 0.0, 'level': 0, 'score': 0.05},
            {'t': 32.1, 'level': 1, 'score': 0.28},
            {'t': 58.4, 'level': 2, 'score': 0.58},
        ],
        metrics_timeline=[
            {'t': 10.0, 'kl_mean': 0.08, 'mmd_stat': 0.02, 'shift_detected': False},
            {'t': 45.0, 'kl_mean': 0.21, 'mmd_stat': 0.05, 'shift_detected': True},
            {'t': 90.0, 'kl_mean': 0.35, 'mmd_stat': 0.07, 'shift_detected': True},
        ],
        alerts=[
            {
                'event_type': 'level_change',
                'from_level': 0,
                'to_level': 1,
                'primary_driver': 'distribution_shift',
                'message': 'KL mean rising',
            },
            {
                'event_type': 'level_change',
                'from_level': 1,
                'to_level': 2,
                'primary_driver': 'distribution_shift',
                'message': 'MMD shift detected',
            },
        ],
        latest_risk={
            'level': 2,
            'level_name': 'R2',
            'composite_score': 0.58,
            'primary_driver': 'distribution_shift',
            'recommendation': '检查域随机化参数范围',
            'e_stop_active': False,
            'degraded_mode': True,
            'attribution': [],
        },
        latest_metrics={
            'joint_names': [f'iiwa_joint_{i}' for i in range(1, 8)],
            'kl_divergence_mean': 0.21,
            'mmd_statistic': 0.05,
            'shift_detected': True,
        },
        screenshot_b64=None,
        recommendation='检查域随机化参数范围；建议在 Real-Source 上复测 SC-01。',
    )
    OUT.write_text(html, encoding='utf-8')
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
