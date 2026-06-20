#!/usr/bin/env python3
"""Generate dual-repo integration HTML report and chart assets."""

from __future__ import annotations

import argparse
import base64
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'dist_monitor'))
sys.path.insert(0, str(ROOT / 'scripts'))

from dist_monitor.lerobot_loader import load_lerobot_dataset
from dist_monitor.metrics_core import MetricsConfig
from dist_monitor.offline_compare import _load_npz_trajectory, compare_offline
from report_charts import (
    chart_cross_source_overlay,
    chart_dual_repo_overview,
    chart_lerobot_trajectory,
    chart_metrics,
    metrics_summary_line,
)

ASSETS = ROOT / 'docs' / 'assets'
SAMPLES = ROOT / 'docs' / 'samples'


def _git_short_hash(cwd: Path) -> str:
    try:
        return subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=cwd,
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return 'unknown'


def _read_json(path: Path | None) -> dict:
    if path is None or not path.is_file():
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def _b64(path: Path) -> str | None:
    if not path.is_file():
        return None
    return base64.b64encode(path.read_bytes()).decode('ascii')


def _metrics_table(metrics: dict) -> str:
    rows = [
        ('KL mean', f"{metrics.get('kl_divergence_mean', 0):.6f}"),
        ('W1 mean', f"{metrics.get('wasserstein_mean', 0):.6f}"),
        ('MMD statistic', f"{metrics.get('mmd_statistic', 0):.6f}"),
        ('MMD p-value', f"{metrics.get('mmd_p_value', 0):.6f}"),
        ('shift_detected', str(metrics.get('shift_detected', False))),
        ('detection_method', str(metrics.get('detection_method', 'none'))),
        ('sample_count', str(metrics.get('sample_count', 0))),
    ]
    body = ''.join(f'<tr><td>{k}</td><td>{v}</td></tr>' for k, v in rows)
    return f'<table><tr><th>指标</th><th>值</th></tr>{body}</table>'


def _render_html(
    *,
    self_metrics: dict,
    cross_metrics: dict,
    dataset_validation: dict,
    online_smoke: dict,
    paths: dict[str, str],
    image_files: dict[str, Path],
) -> str:
    now = datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')
    bridge_hash = _git_short_hash(ROOT)
    edl_root = paths.get('episode_data_lab_root', '')
    edl_hash = _git_short_hash(Path(edl_root)) if edl_root and Path(edl_root).is_dir() else 'unknown'

    imgs_html = ''
    for label, path in image_files.items():
        b64 = _b64(path)
        if b64:
            imgs_html += (
                f'<h3>{label}</h3>'
                f'<img src="data:image/png;base64,{b64}" '
                f'style="max-width:100%;border:1px solid #434343;margin-bottom:16px;" '
                f'alt="{label}"/>'
            )
    val_ok = dataset_validation.get('ok', dataset_validation.get('passed', True))
    online_ok = online_smoke.get('ok', False)
    capture_ok = online_smoke.get('capture_ok', False)
    cross_shift = cross_metrics.get('shift_detected', False)
    cross_class = 'pass' if cross_shift else 'fail'

    online_metrics = online_smoke.get('online_metrics') or {}

    return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8"/>
  <title>Dual-Repo Integration Report</title>
  <style>
    body {{ font-family: system-ui, sans-serif; background: #141414; color: #e8e8e8; margin: 24px; max-width: 960px; }}
    h1, h2, h3 {{ color: #69b1ff; }}
    table {{ border-collapse: collapse; width: 100%; margin: 12px 0; }}
    th, td {{ border: 1px solid #434343; padding: 8px; text-align: left; }}
    th {{ background: #1f1f1f; }}
    .pass {{ color: #95de64; font-weight: bold; }}
    .fail {{ color: #ff7875; font-weight: bold; }}
    .warn {{ color: #faad14; font-weight: bold; }}
    .card {{ background: #1f1f1f; padding: 16px; border-radius: 8px; margin: 12px 0; }}
    code {{ background: #262626; padding: 2px 6px; border-radius: 4px; }}
  </style>
</head>
<body>
  <h1>双仓库联调验收报告</h1>
  <p>Generated: {now}</p>
  <p>Bridge commit: <code>{bridge_hash}</code> · episode-data-lab commit: <code>{edl_hash}</code></p>

  <h2>1. 路径与契约</h2>
  <table>
    <tr><th>项</th><th>值</th></tr>
    <tr><td>EPISODE_DATA_LAB_ROOT</td><td><code>{paths.get('episode_data_lab_root', 'N/A')}</code></td></tr>
    <tr><td>LEROBOT_EXPORT</td><td><code>{paths.get('lerobot_export', 'N/A')}</code></td></tr>
    <tr><td>Bridge Sim NPZ</td><td><code>{paths.get('sim_npz', 'N/A')}</code></td></tr>
    <tr><td>robot_type</td><td><code>{paths.get('robot_type', 'kuka_iiwa')}</code></td></tr>
    <tr><td>total_episodes / frames</td><td>{paths.get('total_episodes', 'N/A')} / {paths.get('total_frames', 'N/A')}</td></tr>
  </table>

  <h2>2. 验收结果</h2>
  <div class="card">
    <p>validate_dataset: <span class="{'pass' if val_ok else 'fail'}">{'PASS' if val_ok else 'FAIL'}</span></p>
    <p>Sim 轨迹采集 (portfolio_demo + capture): <span class="{'pass' if capture_ok else 'fail'}">{'PASS' if capture_ok else 'FAIL'}</span></p>
    <p><strong>跨源 offline_compare</strong> (bridge Sim vs LeRobot Real):
      <span class="{cross_class}">shift_detected={cross_shift}</span></p>
    <p>在线 /monitor/distribution_metrics: <span class="{'pass' if online_ok else 'warn'}">{'PASS' if online_ok else 'WARN'}</span></p>
  </div>

  <h2>3. 跨源分布指标（主验收项）</h2>
  <p>Bridge PyBullet Sim 轨迹 vs episode-data-lab LeRobot Real 轨迹</p>
  {_metrics_table(cross_metrics)}

  <h2>4. 自检对照（同 LeRobot 自对比，预期全 0）</h2>
  {_metrics_table(self_metrics)}

  <h2>5. 在线监控快照</h2>
  <pre style="background:#1f1f1f;padding:12px;border-radius:8px;overflow:auto">{json.dumps(online_metrics, indent=2, ensure_ascii=False)}</pre>

  <h2>6. 图表记录</h2>
  {imgs_html}

  <h2>7. 复现命令</h2>
  <pre style="background:#1f1f1f;padding:12px;border-radius:8px;">
export EPISODE_DATA_LAB_ROOT={paths.get('episode_data_lab_root', '~/robot-sim-lab/robot-arm-episode-data-lab')}
export LEROBOT_EXPORT=$EPISODE_DATA_LAB_ROOT/dataset/v1/lerobot_export
./scripts/run_dual_repo_integration.sh
  </pre>
</body>
</html>
"""


def run_cross_source_compare(sim_npz: Path, lerobot_export: Path) -> dict:
    sim_ts, sim_states = _load_npz_trajectory(sim_npz)
    real_traj = load_lerobot_dataset(lerobot_export)
    cfg = MetricsConfig(min_samples=50, mmd_permutation_count=100)
    return compare_offline(
        sim_ts,
        sim_states,
        real_traj.timestamps,
        real_traj.full_state(),
        cfg=cfg,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description='Generate dual-repo integration report.')
    parser.add_argument('--lerobot-export', type=Path, help='LeRobot export directory')
    parser.add_argument('--self-json', type=Path, default=SAMPLES / 'dual-repo-offline-self-metrics.json')
    parser.add_argument('--cross-source-json', type=Path, default=SAMPLES / 'dual-repo-cross-source-metrics.json')
    parser.add_argument('--sim-npz', type=Path, default=SAMPLES / 'bridge-sim-trajectory.npz')
    parser.add_argument('--validation-json', type=Path, default=SAMPLES / 'dual-repo-validation.json')
    parser.add_argument('--online-json', type=Path, default=SAMPLES / 'dual-repo-online-smoke.json')
    parser.add_argument('--episode-data-lab-root', type=Path, help='episode-data-lab root')
    args = parser.parse_args()

    edl_root = args.episode_data_lab_root
    if edl_root is None:
        env = __import__('os').environ.get('EPISODE_DATA_LAB_ROOT', '').strip()
        if env:
            edl_root = Path(env)
        else:
            candidate = Path.home() / 'robot-sim-lab' / 'robot-arm-episode-data-lab'
            edl_root = candidate if candidate.is_dir() else None

    lerobot = args.lerobot_export
    if lerobot is None and edl_root is not None:
        lerobot = edl_root / 'dataset' / 'v1' / 'lerobot_export'
    if lerobot is None or not lerobot.is_dir():
        print('[ERROR] LeRobot export not found', file=sys.stderr)
        return 1

    self_metrics = _read_json(args.self_json)
    cross_metrics = _read_json(args.cross_source_json)
    if not cross_metrics and args.sim_npz and args.sim_npz.is_file():
        cross_metrics = run_cross_source_compare(args.sim_npz, lerobot)
        out = SAMPLES / 'dual-repo-cross-source-metrics.json'
        out.write_text(json.dumps(cross_metrics, indent=2), encoding='utf-8')
        print(f'wrote {out}')

    validation = _read_json(args.validation_json)
    online = _read_json(args.online_json)

    ASSETS.mkdir(parents=True, exist_ok=True)
    SAMPLES.mkdir(parents=True, exist_ok=True)

    chart_cross = ASSETS / 'dual-repo-cross-source-metrics.png'
    chart_overlay = ASSETS / 'dual-repo-cross-source-overlay.png'
    chart_traj = ASSETS / 'dual-repo-lerobot-trajectory.png'
    chart_overview = ASSETS / 'dual-repo-integration-overview.png'
    chart_self = ASSETS / 'dual-repo-offline-self-metrics.png'

    chart_dual_repo_overview(chart_overview, fig_num='1')
    print(f'wrote {chart_overview}')

    if cross_metrics:
        chart_metrics(
            cross_metrics, chart_cross,
            title='Cross-source metrics (Sim NPZ vs LeRobot)',
            fig_num='3',
        )
        print(f'wrote {chart_cross}')
    if args.sim_npz and args.sim_npz.is_file():
        chart_cross_source_overlay(
            args.sim_npz, lerobot, chart_overlay,
            title='Cross-source overlay: Bridge Sim vs LeRobot Real',
            fig_num='2',
            sample_count=cross_metrics.get('sample_count'),
        )
        print(f'wrote {chart_overlay}')
    if self_metrics:
        chart_metrics(
            self_metrics, chart_self,
            title='Negative control: LeRobot self-compare',
            fig_num='5',
        )
        print(f'wrote {chart_self}')
    chart_lerobot_trajectory(lerobot, chart_traj, fig_num='4')
    print(f'wrote {chart_traj}')

    info_path = lerobot / 'meta' / 'info.json'
    info = _read_json(info_path) if info_path.is_file() else {}
    paths = {
        'episode_data_lab_root': str(edl_root) if edl_root else '',
        'lerobot_export': str(lerobot),
        'sim_npz': str(args.sim_npz) if args.sim_npz else '',
        'robot_type': info.get('robot_type', 'kuka_iiwa'),
        'total_episodes': str(info.get('total_episodes', '')),
        'total_frames': str(info.get('total_frames', '')),
    }

    cross_caption = metrics_summary_line(cross_metrics) if cross_metrics else ''
    self_caption = metrics_summary_line(self_metrics) if self_metrics else ''

    images = {
        f'图 1 · 双仓库集成架构': chart_overview,
        f'图 2 · 跨源轨迹叠加<br/><small>{cross_caption}</small>': chart_overlay,
        f'图 3 · 跨源分布指标<br/><small>{cross_caption}</small>': chart_cross,
        f'图 4 · LeRobot Real 全量轨迹': chart_traj,
    }
    if chart_self.is_file():
        images[f'图 5 · 负对照指标<br/><small>{self_caption}</small>'] = chart_self

    html = _render_html(
        self_metrics=self_metrics,
        cross_metrics=cross_metrics,
        dataset_validation=validation,
        online_smoke=online,
        paths=paths,
        image_files=images,
    )
    report_path = SAMPLES / 'dual-repo-integration-report.html'
    report_path.write_text(html, encoding='utf-8')
    print(f'wrote {report_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
