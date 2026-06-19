#!/usr/bin/env python3
"""Synthetic ROC-style calibration for KL / W1 / MMD monitor thresholds.

Uses error distributions (baseline vs +30% damping proxy shift) aligned with
`/bridge/inject_shift` ground-truth experiments. Writes recommended values to
stdout and optionally updates dist_monitor config YAML files.
"""

from __future__ import annotations

import argparse
import sys
from datetime import date
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'dist_monitor'))

from dist_monitor.metrics_core import MetricsConfig, compute_distribution_metrics  # noqa: E402


def _synthetic_streams(
    n: int,
    *,
    shift: float,
    noise: float,
    seed: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    rng = np.random.default_rng(seed)
    ts = np.linspace(0.0, 5.0, n)
    base = rng.normal(scale=noise, size=(n, 7))
    pos = base + np.sin(ts)[:, None] * 0.05
    vel = np.zeros_like(pos)
    sim = np.hstack([pos, vel])
    real = np.hstack([pos + shift, vel])
    return ts, sim, ts, real


def _percentile_threshold(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    return float(np.percentile(values, percentile))


def calibrate(*, samples: int, seed: int) -> dict:
    baseline_ts, baseline_sim, _, baseline_real = _synthetic_streams(
        samples, shift=0.0, noise=0.02, seed=seed,
    )
    baseline_errors = baseline_sim[:, :7] - baseline_real[:, :7]

    cfg = MetricsConfig(min_samples=50, mmd_permutation_count=40)
    baseline_metrics = compute_distribution_metrics(
        baseline_ts, baseline_sim, baseline_ts, baseline_real,
        baseline_errors=baseline_errors, cfg=cfg,
    )

    kl_baseline: list[float] = []
    w1_baseline: list[float] = []
    mmd_baseline: list[float] = []
    for i in range(20):
        ts, sim, _, real = _synthetic_streams(
            samples, shift=0.0, noise=0.02, seed=seed + i + 1,
        )
        result = compute_distribution_metrics(
            ts, sim, ts, real, baseline_errors=baseline_errors, cfg=cfg,
        )
        kl_baseline.append(result.kl_mean)
        w1_baseline.append(result.w1_mean)
        mmd_baseline.append(result.mmd_statistic)

    shift_ts, shift_sim, _, shift_real = _synthetic_streams(
        samples, shift=0.12, noise=0.02, seed=seed + 100,
    )
    shifted = compute_distribution_metrics(
        shift_ts, shift_sim, shift_ts, shift_real,
        baseline_errors=baseline_errors, cfg=cfg,
    )

    kl_threshold = max(
        _percentile_threshold(kl_baseline, 99),
        baseline_metrics.kl_mean * 2.5,
        0.15,
    )
    w1_threshold = max(
        _percentile_threshold(w1_baseline, 99),
        baseline_metrics.w1_mean * 2.5,
        0.08,
    )
    mmd_threshold = max(
        _percentile_threshold(mmd_baseline, 99),
        baseline_metrics.mmd_statistic * 2.0,
        0.05,
    )

    return {
        'thresholds': {
            'kl_threshold_mean': round(kl_threshold, 4),
            'w1_threshold_mean': round(w1_threshold, 4),
            'mmd_threshold': round(mmd_threshold, 4),
            'mmd_p_value_alpha': 0.05,
        },
        'calibration': {
            'calibration_date': date.today().isoformat(),
            'calibration_scenario': 'synthetic_inject_shift_proxy',
            'baseline_kl_p99': round(_percentile_threshold(kl_baseline, 99), 4),
            'baseline_w1_p99': round(_percentile_threshold(w1_baseline, 99), 4),
            'baseline_mmd_p99': round(_percentile_threshold(mmd_baseline, 99), 4),
            'shift_detection': {
                'kl_mean': round(shifted.kl_mean, 4),
                'w1_mean': round(shifted.w1_mean, 4),
                'mmd_statistic': round(shifted.mmd_statistic, 4),
                'shift_detected': shifted.shift_detected,
                'detection_method': shifted.detection_method,
            },
        },
    }


def _write_yaml(path: Path, data: dict, root_key: str) -> None:
    existing = {}
    if path.exists():
        existing = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    existing[root_key] = existing.get(root_key, {})
    existing[root_key]['ros__parameters'] = {
        **existing[root_key].get('ros__parameters', {}),
        **data,
    }
    path.write_text(yaml.dump(existing, sort_keys=False, allow_unicode=True), encoding='utf-8')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Calibrate dist_monitor thresholds.')
    parser.add_argument('--samples', type=int, default=200)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--write', action='store_true', help='Update thresholds.yaml + calibration.yaml')
    args = parser.parse_args(argv)

    report = calibrate(samples=args.samples, seed=args.seed)
    print(yaml.dump(report, sort_keys=False, allow_unicode=True))

    if args.write:
        dist_cfg = ROOT / 'dist_monitor' / 'config'
        thresholds_path = dist_cfg / 'thresholds.yaml'
        calibration_path = dist_cfg / 'calibration.yaml'

        _write_yaml(thresholds_path, report['thresholds'], 'dist_monitor')
        cal = report['calibration']
        cal_data = {
            'thresholds': report['thresholds'],
            'calibration': {
                'baseline_kl_p99': cal['baseline_kl_p99'],
                'baseline_w1_p99': cal['baseline_w1_p99'],
                'baseline_mmd_p99': cal['baseline_mmd_p99'],
                'calibration_date': cal['calibration_date'],
                'calibration_scenario': cal['calibration_scenario'],
            },
        }
        calibration_path.write_text(
            yaml.dump(cal_data, sort_keys=False, allow_unicode=True),
            encoding='utf-8',
        )
        print(f'Wrote {thresholds_path}', file=sys.stderr)
        print(f'Wrote {calibration_path}', file=sys.stderr)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
