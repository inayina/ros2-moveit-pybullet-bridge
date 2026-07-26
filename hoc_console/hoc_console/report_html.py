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
    runtime_trace_report: dict[str, Any] | None = None,
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

    def _metric(value: Any, precision: int = 4) -> str:
        if value is None:
            return 'unavailable'
        return f'{float(value):.{precision}f}'

    ratio = summary.get('shift_detected_ratio')
    ratio_text = 'unavailable' if ratio is None else f'{float(ratio):.2%}'

    metric_rows = _table_rows(metrics_timeline, [
        't', 'metric_valid', 'validity', 'reason_code', 'calibration_id',
        'kl_mean', 'w1_mean', 'mmd_stat', 'shift_detected',
    ])
    trace = runtime_trace_report or {}
    command_rows = []
    for item in trace.get('commands', [])[-50:]:
        execution = item.get('execution', [])
        safety = item.get('safety', [])
        task_gt = item.get('task_gt', [])
        command_rows.append(
            '<tr>'
            f'<td>{item.get("command_sequence", "")}</td>'
            f'<td>{execution[-1].get("decision", "UNAVAILABLE") if execution else "UNAVAILABLE"}</td>'
            f'<td>{safety[-1].get("proposed_decision", "UNAVAILABLE") if safety else "UNAVAILABLE"}</td>'
            f'<td>{safety[-1].get("actual_decision", "UNAVAILABLE") if safety else "UNAVAILABLE"}</td>'
            f'<td>{task_gt[-1].get("task_status", "UNAVAILABLE") if task_gt else "UNAVAILABLE"}</td>'
            '</tr>'
        )
    command_trace_html = '\n'.join(command_rows)

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
    <div class="metric-card"><div>Mean KL</div><div class="metric-value">{_metric(summary.get('mean_kl'))}</div></div>
    <div class="metric-card"><div>Mean W1</div><div class="metric-value">{_metric(summary.get('mean_w1'))}</div></div>
    <div class="metric-card"><div>Mean MMD</div><div class="metric-value">{_metric(summary.get('mean_mmd'))}</div></div>
  </div>
  <p>Shift detected: {summary.get('shift_detected_count', 0)} / ratio {ratio_text}</p>
  <p>Recommendation: {recommendation}</p>

  {screenshot_html}

  <h2>Risk Timeline (last 50)</h2>
  <table>
    <tr><th>t (s)</th><th>Level</th><th>Score</th></tr>
    {_table_rows(risk_timeline, ['t', 'level', 'score'])}
  </table>

  <h2>Metrics Timeline (last 50)</h2>
  <table>
    <tr><th>t</th><th>Valid</th><th>Validity</th><th>Reason</th><th>Calibration</th><th>KL</th><th>W1</th><th>MMD</th><th>Shift</th></tr>
    {metric_rows}
  </table>

  <h2>Latest Snapshot</h2>
  <pre>{json.dumps({'risk': latest_risk, 'metrics': latest_metrics}, indent=2, ensure_ascii=False)}</pre>

  <h2>Policy Command Correlation (offline replay evidence)</h2>
  <p>Closed loop: false · Claims task success: false · Issues: {', '.join(trace.get('issues', [])) or 'none'}</p>
  <table>
    <tr><th>Command sequence</th><th>Execution</th><th>Safety proposed</th><th>Safety actual</th><th>Task GT</th></tr>
    {command_trace_html}
  </table>
</body>
</html>"""
