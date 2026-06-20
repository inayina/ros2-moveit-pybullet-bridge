#!/usr/bin/env python3
"""Generate same-task calibration experiment report (HTML + charts)."""

from __future__ import annotations

import base64
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SAMPLES = ROOT / 'docs' / 'samples'
ASSETS = ROOT / 'docs' / 'assets'

sys.path.insert(0, str(ROOT / 'scripts'))
from report_charts import chart_dual_overlay, chart_metrics, metrics_summary_line


def _git_hash(cwd: Path) -> str:
    try:
        return subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=cwd,
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return 'unknown'


def _read_json(name: str) -> dict:
    path = SAMPLES / name
    if not path.is_file():
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def _b64(path: Path) -> str:
    if not path.is_file():
        return ''
    return base64.b64encode(path.read_bytes()).decode('ascii')


def _metrics_row(label: str, m: dict) -> str:
    if not m:
        return f'<tr><td>{label}</td><td colspan="5"><em>未运行</em></td></tr>'
    return (
        f'<tr><td>{label}</td>'
        f'<td>{m.get("sample_count", 0)}</td>'
        f'<td>{m.get("kl_divergence_mean", 0):.6f}</td>'
        f'<td>{m.get("wasserstein_mean", 0):.6f}</td>'
        f'<td>{m.get("mmd_statistic", 0):.6f} (p={m.get("mmd_p_value", 1):.3f})</td>'
        f'<td class="{"pass" if m.get("shift_detected") else "warn"}">'
        f'{m.get("shift_detected", False)} ({m.get("detection_method", "none")})</td></tr>'
    )


def main() -> int:
    ASSETS.mkdir(parents=True, exist_ok=True)
    iiwa_m = _read_json('same-task-iiwa-metrics.json')
    lerobot_m = _read_json('same-task-lerobot-metrics.json')
    iiwa_npz = SAMPLES / 'same-task-iiwa-dual.npz'
    lerobot_npz = SAMPLES / 'same-task-lerobot-dual.npz'

    if iiwa_npz.is_file() and iiwa_m:
        chart_dual_overlay(
            iiwa_npz, ASSETS / 'same-task-iiwa-overlay.png',
            title='Same-task A: iiwa sinusoid Sim vs Real',
            fig_num='6',
            sample_count=iiwa_m.get('sample_count'),
        )
        chart_metrics(
            iiwa_m, ASSETS / 'same-task-iiwa-metrics.png',
            title='Same-task A metrics', fig_num='7',
        )
    if lerobot_npz.is_file() and lerobot_m:
        chart_dual_overlay(
            lerobot_npz, ASSETS / 'same-task-lerobot-overlay.png',
            title='Same-task B: LeRobot ep0 replay Sim vs Real',
            fig_num='8',
            sample_count=lerobot_m.get('sample_count'),
        )
        chart_metrics(
            lerobot_m, ASSETS / 'same-task-lerobot-metrics.png',
            title='Same-task B metrics', fig_num='9',
        )

    baseline_frac = iiwa_m.get('baseline_frac', lerobot_m.get('baseline_frac', 0.3))
    bridge_hash = _git_hash(ROOT)
    now = datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')
    exp_id = f'exp_same_task_{datetime.now(timezone.utc).strftime("%Y%m%d")}'

    def img_block(fig_num: str, title: str, asset: str, metrics: dict, caption: str) -> str:
        b64 = _b64(ASSETS / asset)
        summary = metrics_summary_line(metrics) if metrics else ''
        if not b64:
            return f'<h3>{fig_num} · {title}</h3><p><em>（图片缺失）</em></p>'
        return f"""
<h3>{fig_num} · {title}</h3>
<p class="caption">{caption}</p>
<p class="caption"><code>{summary}</code></p>
<img src="data:image/png;base64,{b64}" alt="{title}" style="max-width:100%;border:1px solid #434343;margin:12px 0;"/>
"""

    html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8"/>
  <title>同任务校准实验报告 — {exp_id}</title>
  <style>
    body {{ font-family: "Noto Sans SC", system-ui, sans-serif; background: #141414; color: #e8e8e8;
           margin: 32px auto; max-width: 920px; line-height: 1.65; }}
    h1 {{ color: #69b1ff; border-bottom: 1px solid #434343; padding-bottom: 8px; }}
    h2 {{ color: #69b1ff; margin-top: 2em; }}
    table {{ border-collapse: collapse; width: 100%; margin: 16px 0; }}
    th, td {{ border: 1px solid #434343; padding: 10px; text-align: left; }}
    th {{ background: #1f1f1f; }}
    .meta {{ color: #a0a0a0; font-size: 0.95em; }}
    .card {{ background: #1f1f1f; border-radius: 8px; padding: 16px 20px; margin: 16px 0; }}
    .pass {{ color: #95de64; font-weight: 600; }}
    .warn {{ color: #faad14; }}
    .caption {{ color: #bfbfbf; font-size: 0.92em; }}
    .prove {{ border-left: 3px solid #69b1ff; padding-left: 12px; margin: 12px 0; }}
    code {{ background: #262626; padding: 2px 6px; border-radius: 4px; }}
  </style>
</head>
<body>

<h1>同任务 Sim2Real 校准实验报告</h1>
<p class="meta">
  实验编号：<strong>{exp_id}</strong><br/>
  生成时间：{now} · Bridge <code>{bridge_hash}</code><br/>
  baseline_frac = <code>{baseline_frac}</code>（前 {baseline_frac:.0%} 对齐误差作为 KL/W1 基线）<br/>
  图号 6–9 与 <a href="dual-repo-experiment-report.html" style="color:#69b1ff">双仓库报告</a> 图 1–5 连续编号
</p>

<h2>1. 实验目的</h2>
<div class="card">
  <p>解决跨源对比中<strong>任务不一致</strong>（bridge 正弦 demo vs LeRobot pick_and_lift）导致的指标解释困难。
  本实验在<strong>同一 JointTrajectory 命令</strong>下，对比 bridge 双源模式中的 Sim（理想 PyBullet）与
  Real（带 domain randomization 的 PyBullet），使 KL / W1 / MMD 具有可解释的物理含义。</p>
</div>

<h2>2. 实验设计</h2>
<table>
  <tr><th>实验</th><th>motion_source</th><th>real_source</th><th>说明</th></tr>
  <tr>
    <td>A</td><td><code>iiwa</code></td><td><code>topic</code></td>
    <td>内置正弦轨迹；Sim/Real 收到相同 <code>/bridge/command</code></td>
  </tr>
  <tr>
    <td>B</td><td><code>lerobot</code></td><td><code>topic</code></td>
    <td>回放 episode-data-lab episode 0；验证跨仓库同动作</td>
  </tr>
</table>

<h2>3. 指标汇总</h2>
<table>
  <tr>
    <th>实验</th><th>对齐样本</th><th>KL mean</th><th>W1 mean</th>
    <th>MMD</th><th>shift_detected</th>
  </tr>
  {_metrics_row('A — iiwa demo', iiwa_m)}
  {_metrics_row('B — LeRobot replay', lerobot_m)}
</table>

<h2>4. 图表解读</h2>
<div class="prove">
  <p><strong>图 6 / 8 轨迹叠加</strong>：Sim（橙）与 Real（蓝）跟踪同一命令；标题含采样点数与对齐 n。</p>
  <p><strong>图 7 / 9 指标柱状</strong>：页脚与上表数值一致（同一 JSON 源生成）。</p>
</div>

{img_block('图 6', '同任务 A 轨迹', 'same-task-iiwa-overlay.png', iiwa_m,
           'iiwa_motion_demo · 双源同命令')}
{img_block('图 7', '同任务 A 指标', 'same-task-iiwa-metrics.png', iiwa_m,
           'KL/W1 逐关节 + MMD 汇总')}
{img_block('图 8', '同任务 B 轨迹', 'same-task-lerobot-overlay.png', lerobot_m,
           'LeRobot episode 0 回放')}
{img_block('图 9', '同任务 B 指标', 'same-task-lerobot-metrics.png', lerobot_m,
           '跨仓库同动作指标')}

<h2>5. 复现命令</h2>
<pre><code>export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab
export LEROBOT_EXPORT=$EPISODE_DATA_LAB_ROOT/dataset/v1/lerobot_export
./scripts/run_same_task_calibration.sh
python3 scripts/regenerate_all_reports.py   # 对齐图表与报告</code></pre>

<p class="meta">关联报告：
  <a href="dual-repo-experiment-report.html" style="color:#69b1ff">dual-repo-experiment-report.html</a>
  · <a href="dual-repo-integration-report.html" style="color:#69b1ff">dual-repo-integration-report.html</a>
</p>

</body>
</html>
"""

    out = SAMPLES / 'same-task-calibration-report.html'
    out.write_text(html, encoding='utf-8')
    print(f'wrote {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
