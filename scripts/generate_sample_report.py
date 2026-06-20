#!/usr/bin/env python3
"""Generate a sample HOC experiment HTML report for docs/samples/."""

from __future__ import annotations

import base64
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'hoc_console'))
OUT = ROOT / 'docs' / 'samples' / 'sample-experiment-report.html'
DASHBOARD_PNG = ROOT / 'docs' / 'assets' / 'm5-hoc-dashboard.png'

from hoc_console.report_html import render_html_report


def _git_commit_hash() -> str:
    try:
        return subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=ROOT,
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return 'unknown'


def _dashboard_screenshot_b64() -> str | None:
    if not DASHBOARD_PNG.is_file():
        return None
    return base64.b64encode(DASHBOARD_PNG.read_bytes()).decode('ascii')


def main() -> None:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    html = render_html_report(
        experiment_id='exp_20260620_sample',
        metadata={
            'scenario_id': 'SC-01',
            'random_seed': 42,
            'randomization_strength': 0.5,
            'duration_sec': 120.5,
            'git_commit_hash': _git_commit_hash(),
            'robot_profile': 'iiwa7',
        },
        summary={
            'max_risk_level': 2,
            'max_composite_score': 0.68,
            'shift_detected_count': 15,
            'shift_detected_ratio': 0.125,
            'mean_kl': 0.18,
            'max_kl': 0.91,
            'mean_w1': 0.07,
            'max_w1': 0.12,
            'mean_mmd': 0.04,
            'max_mmd': 0.08,
        },
        risk_timeline=[
            {'t': 0.0, 'level': 0, 'score': 0.05},
            {'t': 32.1, 'level': 1, 'score': 0.28},
            {'t': 58.4, 'level': 2, 'score': 0.58},
        ],
        metrics_timeline=[
            {
                't': 10.0, 'kl_mean': 0.08, 'w1_mean': 0.04, 'mmd_stat': 0.02,
                'shift_detected': False, 'comm_health_score': 0.05,
            },
            {
                't': 45.0, 'kl_mean': 0.21, 'w1_mean': 0.09, 'mmd_stat': 0.05,
                'shift_detected': True, 'comm_health_score': 0.12,
            },
            {
                't': 90.0, 'kl_mean': 0.35, 'w1_mean': 0.11, 'mmd_stat': 0.07,
                'shift_detected': True, 'comm_health_score': 0.22,
            },
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
            'attribution': [
                {'dimension': 'distribution_shift', 'score': 0.72},
                {'dimension': 'tracking_error', 'score': 0.35},
                {'dimension': 'dynamics_anomaly', 'score': 0.28},
                {'dimension': 'comm_health', 'score': 0.15},
                {'dimension': 'planning_failure', 'score': 0.08},
            ],
        },
        latest_metrics={
            'joint_names': [f'lbr_iiwa_joint_{i}' for i in range(1, 8)],
            'kl_divergence_mean': 0.21,
            'wasserstein_mean': 0.09,
            'mmd_statistic': 0.05,
            'shift_detected': True,
            'comm_health_score': 0.15,
            'dynamics_anomaly_score': 0.28,
            'soft_limit_triggered': False,
        },
        screenshot_b64=_dashboard_screenshot_b64(),
        recommendation='检查域随机化参数范围；建议在 Real-Source 上复测 SC-01。',
    )
    OUT.write_text(html, encoding='utf-8')
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
