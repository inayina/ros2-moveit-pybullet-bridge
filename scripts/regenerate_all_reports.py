#!/usr/bin/env python3
"""Regenerate all experiment charts and HTML reports from docs/samples JSON/NPZ."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SAMPLES = ROOT / 'docs' / 'samples'
ASSETS = ROOT / 'docs' / 'assets'
SCRIPTS = ROOT / 'scripts'

sys.path.insert(0, str(ROOT / 'dist_monitor'))
sys.path.insert(0, str(SCRIPTS))

from report_charts import (
    chart_cross_source_overlay,
    chart_dual_overlay,
    chart_dual_repo_overview,
    chart_lerobot_trajectory,
    chart_metrics,
    metrics_summary_line,
)


def _read_json(path: Path) -> dict:
    if not path.is_file():
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def _lerobot_export() -> Path | None:
    import os

    env = os.environ.get('LEROBOT_EXPORT', '').strip()
    if env:
        p = Path(env)
        if p.is_dir():
            return p
    edl = os.environ.get('EPISODE_DATA_LAB_ROOT', '').strip()
    if edl:
        p = Path(edl) / 'dataset' / 'v1' / 'lerobot_export'
        if p.is_dir():
            return p
    default = Path.home() / 'robot-sim-lab' / 'robot-arm-episode-data-lab' / 'dataset' / 'v1' / 'lerobot_export'
    return default if default.is_dir() else None


def regenerate_charts() -> list[str]:
    """Rebuild PNG assets from JSON/NPZ. Returns list of written paths."""
    written: list[str] = []
    lerobot = _lerobot_export()

    self_m = _read_json(SAMPLES / 'dual-repo-offline-self-metrics.json')
    cross_m = _read_json(SAMPLES / 'dual-repo-cross-source-metrics.json')
    sim_npz = SAMPLES / 'bridge-sim-trajectory.npz'
    iiwa_m = _read_json(SAMPLES / 'same-task-iiwa-metrics.json')
    lerobot_m = _read_json(SAMPLES / 'same-task-lerobot-metrics.json')
    iiwa_npz = SAMPLES / 'same-task-iiwa-dual.npz'
    lerobot_npz = SAMPLES / 'same-task-lerobot-dual.npz'

    ASSETS.mkdir(parents=True, exist_ok=True)

    overview = ASSETS / 'dual-repo-integration-overview.png'
    chart_dual_repo_overview(overview, fig_num='1')
    written.append(str(overview))

    if sim_npz.is_file() and lerobot:
        overlay = ASSETS / 'dual-repo-cross-source-overlay.png'
        chart_cross_source_overlay(
            sim_npz, lerobot, overlay,
            title='Cross-source overlay: Bridge Sim vs LeRobot Real',
            fig_num='2',
            sample_count=cross_m.get('sample_count'),
        )
        written.append(str(overlay))

    if cross_m:
        cross_chart = ASSETS / 'dual-repo-cross-source-metrics.png'
        chart_metrics(
            cross_m, cross_chart,
            title='Cross-source metrics (Sim NPZ vs LeRobot)',
            fig_num='3',
        )
        written.append(str(cross_chart))

    if lerobot:
        traj = ASSETS / 'dual-repo-lerobot-trajectory.png'
        chart_lerobot_trajectory(lerobot, traj, fig_num='4')
        written.append(str(traj))

    if self_m:
        self_chart = ASSETS / 'dual-repo-offline-self-metrics.png'
        chart_metrics(
            self_m, self_chart,
            title='Negative control: LeRobot self-compare',
            fig_num='5',
        )
        written.append(str(self_chart))

    if iiwa_npz.is_file() and iiwa_m:
        p = ASSETS / 'same-task-iiwa-overlay.png'
        chart_dual_overlay(
            iiwa_npz, p,
            title='Same-task A: iiwa sinusoid Sim vs Real',
            fig_num='6',
            sample_count=iiwa_m.get('sample_count'),
        )
        written.append(str(p))
        p = ASSETS / 'same-task-iiwa-metrics.png'
        chart_metrics(iiwa_m, p, title='Same-task A metrics', fig_num='7')
        written.append(str(p))

    if lerobot_npz.is_file() and lerobot_m:
        p = ASSETS / 'same-task-lerobot-overlay.png'
        chart_dual_overlay(
            lerobot_npz, p,
            title='Same-task B: LeRobot ep0 replay Sim vs Real',
            fig_num='8',
            sample_count=lerobot_m.get('sample_count'),
        )
        written.append(str(p))
        p = ASSETS / 'same-task-lerobot-metrics.png'
        chart_metrics(lerobot_m, p, title='Same-task B metrics', fig_num='9')
        written.append(str(p))

    manifest = {
        'charts': written,
        'dual_repo_cross_source': metrics_summary_line(cross_m) if cross_m else None,
        'same_task_iiwa': metrics_summary_line(iiwa_m) if iiwa_m else None,
        'same_task_lerobot': metrics_summary_line(lerobot_m) if lerobot_m else None,
    }
    manifest_path = SAMPLES / 'report-manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding='utf-8')
    written.append(str(manifest_path))

    return written


def regenerate_reports() -> None:
    edl = _lerobot_export()
    edl_root = edl.parent.parent.parent if edl else None
    int_cmd = [sys.executable, str(SCRIPTS / 'generate_integration_report.py')]
    if edl:
        int_cmd.extend(['--lerobot-export', str(edl)])
    if edl_root and edl_root.is_dir():
        int_cmd.extend(['--episode-data-lab-root', str(edl_root)])

    subprocess.run(int_cmd, check=True, cwd=str(ROOT))
    subprocess.run(
        [sys.executable, str(SCRIPTS / 'generate_dual_repo_experiment_report.py')],
        check=True, cwd=str(ROOT),
    )
    subprocess.run(
        [sys.executable, str(SCRIPTS / 'generate_same_task_calibration_report.py')],
        check=True, cwd=str(ROOT),
    )


def main() -> int:
    print('==> Regenerating charts from docs/samples JSON/NPZ')
    written = regenerate_charts()
    for path in written:
        print(f'  {path}')

    print('==> Regenerating HTML reports (embed latest charts)')
    regenerate_reports()
    print('[PASS] Reports and charts aligned')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
