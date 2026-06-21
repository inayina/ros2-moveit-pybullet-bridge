#!/usr/bin/env python3
"""Validate benchmark_system.py summary schema."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REQUIRED_KEYS = {
    'strategy',
    'episodes',
    'completed_episodes',
    'max_latency_ms',
    'mean_latency_ms',
    'std_latency_ms',
    'cpu_peak_percent',
    'rss_peak_mb',
    'health_alarm_detected_within_1s',
    'seed',
}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Validate PolicyRunner benchmark summary JSON.')
    parser.add_argument('summary', type=Path, help='Path to benchmark_summary.json')
    parser.add_argument('--require-fault-alarm', action='store_true')
    args = parser.parse_args(argv)

    if not args.summary.is_file():
        print(f'missing summary file: {args.summary}', file=sys.stderr)
        return 1

    data = json.loads(args.summary.read_text(encoding='utf-8'))
    missing = sorted(REQUIRED_KEYS - set(data))
    if missing:
        print(f'missing keys: {missing}', file=sys.stderr)
        return 1

    if data['completed_episodes'] != data['episodes']:
        print('completed_episodes does not match episodes', file=sys.stderr)
        return 1

    if args.require_fault_alarm and not data.get('health_alarm_detected_within_1s'):
        print('health_alarm_detected_within_1s is false', file=sys.stderr)
        return 1

    print(f'[check_policy_runner_benchmark] OK: {args.summary}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
