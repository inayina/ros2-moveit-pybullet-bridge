#!/usr/bin/env python3
"""Merge PolicyRunner benchmark outputs into a single validation report."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Any


def _load_summary(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f'missing benchmark summary: {path}')
    return json.loads(path.read_text(encoding='utf-8'))


def _render_html(summary: dict[str, Any], output_dir: Path) -> str:
    replay = summary.get('replay', {})
    sine = summary.get('sine_wave', {})
    return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8"/>
  <title>Policy Runner System Validation</title>
  <style>
    body {{ font-family: system-ui, sans-serif; background: #141414; color: #e8e8e8; margin: 24px; }}
    h1, h2 {{ color: #69b1ff; }}
    table {{ border-collapse: collapse; width: 100%; margin: 12px 0; }}
    th, td {{ border: 1px solid #434343; padding: 8px; text-align: left; }}
    th {{ background: #1f1f1f; }}
    .pass {{ color: #95de64; font-weight: bold; }}
    .fail {{ color: #ff7875; font-weight: bold; }}
  </style>
</head>
<body>
  <h1>Policy Runner System Validation</h1>
  <p>Generated: {summary.get('generated_at', '')}</p>
  <p>Output: <code>{output_dir}</code></p>
  <p>Overall: <span class="{'pass' if summary.get('pass') else 'fail'}">{'PASS' if summary.get('pass') else 'FAIL'}</span></p>

  <h2>Benchmark Comparison</h2>
  <table>
    <tr>
      <th>Strategy</th><th>Episodes</th><th>Mean latency (ms)</th><th>Max latency (ms)</th>
      <th>CPU peak (%)</th><th>RSS peak (MB)</th><th>Rows</th>
    </tr>
    <tr>
      <td>replay</td>
      <td>{replay.get('completed_episodes', 0)}</td>
      <td>{replay.get('mean_latency_ms', 0):.3f}</td>
      <td>{replay.get('max_latency_ms', 0):.3f}</td>
      <td>{replay.get('cpu_peak_percent', 0):.3f}</td>
      <td>{replay.get('rss_peak_mb', 0):.3f}</td>
      <td>{replay.get('timeseries_rows', 0)}</td>
    </tr>
    <tr>
      <td>sine_wave</td>
      <td>{sine.get('completed_episodes', 0)}</td>
      <td>{sine.get('mean_latency_ms', 0):.3f}</td>
      <td>{sine.get('max_latency_ms', 0):.3f}</td>
      <td>{sine.get('cpu_peak_percent', 0):.3f}</td>
      <td>{sine.get('rss_peak_mb', 0):.3f}</td>
      <td>{sine.get('timeseries_rows', 0)}</td>
    </tr>
  </table>

  <h2>Dataset</h2>
  <pre>{json.dumps(summary.get('dataset', {}), indent=2, ensure_ascii=False)}</pre>

  <h2>Full Summary JSON</h2>
  <pre>{json.dumps(summary, indent=2, ensure_ascii=False)}</pre>
</body>
</html>"""


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Generate system validation summary report.')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--replay-summary', type=Path, required=True)
    parser.add_argument('--sine-summary', type=Path, required=True)
    parser.add_argument('--dataset-info', type=Path, default=None)
    args = parser.parse_args(argv)

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    replay = _load_summary(args.replay_summary)
    sine = _load_summary(args.sine_summary)
    dataset: dict[str, Any] = {}
    if args.dataset_info and args.dataset_info.is_file():
        dataset = json.loads(args.dataset_info.read_text(encoding='utf-8'))

    passes = (
        replay.get('completed_episodes') == replay.get('episodes')
        and sine.get('completed_episodes') == sine.get('episodes')
        and replay.get('timeseries_rows', 0) > 0
        and sine.get('timeseries_rows', 0) > 0
    )

    summary = {
        'generated_at': datetime.now().isoformat(timespec='seconds'),
        'pass': passes,
        'replay': replay,
        'sine_wave': sine,
        'dataset': dataset,
        'artifacts': {
            'replay_dir': str(output_dir / 'replay'),
            'sine_wave_dir': str(output_dir / 'sine_wave'),
        },
    }

    summary_path = output_dir / 'validation_summary.json'
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'validation_report.html').write_text(
        _render_html(summary, output_dir),
        encoding='utf-8',
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if passes else 1


if __name__ == '__main__':
    raise SystemExit(main())
