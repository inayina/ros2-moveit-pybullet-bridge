"""HTML experiment report rendering (no ROS dependencies)."""

from __future__ import annotations

import json
from datetime import datetime
from typing import Any


def render_html_report(
    *,
    experiment_id: str,
    metadata: dict[str, Any],
    summary: dict[str, Any],
    risk_timeline: list[dict[str, Any]],
    metrics_timeline: list[dict[str, Any]],
    alerts: list[dict[str, Any]],
    latest_risk: dict[str, Any] | None,
    latest_metrics: dict[str, Any] | None,
    screenshot_b64: str | None,
    recommendation: str,
) -> str:
    del alerts  # reserved for future report sections
    screenshot_html = ''
    if screenshot_b64:
        screenshot_html = (
            f'<h2>Dashboard Screenshot</h2>'
            f'<img src="data:image/png;base64,{screenshot_b64}" '
            f'style="max-width:100%;border:1px solid #333;" alt="dashboard"/>'
        )

    def _table_rows(items: list[dict[str, Any]], keys: list[str]) -> str:
        rows = []
        for item in items[-50:]:
            cells = ''.join(f'<td>{item.get(k, "")}</td>' for k in keys)
            rows.append(f'<tr>{cells}</tr>')
        return '\n'.join(rows)

    return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8"/>
  <title>HOC Experiment Report — {experiment_id}</title>
  <style>
    body {{ font-family: system-ui, sans-serif; background: #141414; color: #e8e8e8; margin: 24px; }}
    h1, h2 {{ color: #69b1ff; }}
    table {{ border-collapse: collapse; width: 100%; margin: 12px 0; }}
    th, td {{ border: 1px solid #434343; padding: 8px; text-align: left; }}
    th {{ background: #1f1f1f; }}
    .metric-grid {{ display: grid; grid-template-columns: repeat(5, 1fr); gap: 12px; }}
    .metric-card {{ background: #1f1f1f; padding: 16px; border-radius: 8px; }}
    .metric-value {{ font-size: 24px; font-weight: bold; color: #95de64; }}
  </style>
</head>
<body>
  <h1>Sim2Real HOC Experiment Report</h1>
  <p>Generated: {datetime.now().isoformat(timespec='seconds')}</p>
  <p>Experiment ID: <strong>{experiment_id}</strong></p>

  <h2>Metadata</h2>
  <ul>
    <li>Scenario: {metadata.get('scenario_id', 'N/A')}</li>
    <li>Seed: {metadata.get('random_seed', 'N/A')}</li>
    <li>Randomization strength: {metadata.get('randomization_strength', 'N/A')}</li>
    <li>Duration (sec): {metadata.get('duration_sec', 0):.1f}</li>
  </ul>

  <h2>Summary</h2>
  <div class="metric-grid">
    <div class="metric-card"><div>Max Risk</div><div class="metric-value">R{summary.get('max_risk_level', 0)}</div></div>
    <div class="metric-card"><div>Max Score</div><div class="metric-value">{summary.get('max_composite_score', 0):.3f}</div></div>
    <div class="metric-card"><div>Mean KL</div><div class="metric-value">{summary.get('mean_kl', 0):.4f}</div></div>
    <div class="metric-card"><div>Mean W1</div><div class="metric-value">{summary.get('mean_w1', 0):.4f}</div></div>
    <div class="metric-card"><div>Mean MMD</div><div class="metric-value">{summary.get('mean_mmd', 0):.4f}</div></div>
  </div>
  <p>Shift detected: {summary.get('shift_detected_count', 0)} / ratio {summary.get('shift_detected_ratio', 0):.2%}</p>
  <p>Recommendation: {recommendation}</p>

  {screenshot_html}

  <h2>Risk Timeline (last 50)</h2>
  <table>
    <tr><th>t (s)</th><th>Level</th><th>Score</th></tr>
    {_table_rows(risk_timeline, ['t', 'level', 'score'])}
  </table>

  <h2>Metrics Timeline (last 50)</h2>
  <table>
    <tr><th>t (s)</th><th>KL mean</th><th>W1 mean</th><th>MMD</th><th>Shift</th></tr>
    {_table_rows(metrics_timeline, ['t', 'kl_mean', 'w1_mean', 'mmd_stat', 'shift_detected'])}
  </table>

  <h2>Latest Snapshot</h2>
  <pre>{json.dumps({'risk': latest_risk, 'metrics': latest_metrics}, indent=2, ensure_ascii=False)}</pre>
</body>
</html>"""
