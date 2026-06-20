"""Offline Sim vs Real distribution comparison (LeRobot Real + Sim trajectory)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from dist_monitor.lerobot_loader import LeRobotTrajectory, load_lerobot_dataset
from dist_monitor.metrics_core import MetricsConfig, compute_distribution_metrics


def _positions_to_states(timestamps: np.ndarray, positions: np.ndarray) -> np.ndarray:
    velocities = np.zeros_like(positions)
    if len(positions) > 1:
        dt = np.diff(timestamps)
        dt = np.where(dt <= 0, 1.0, dt)
        velocities[1:] = np.diff(positions, axis=0) / dt[:, None]
        velocities[0] = velocities[1]
    return np.hstack([positions, velocities])


def _load_npz_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = np.load(path)
    timestamps = np.asarray(data['timestamps'], dtype=float).ravel()
    positions = np.asarray(data['positions'], dtype=float)
    velocities = data.get('velocities')
    if velocities is not None:
        velocities = np.asarray(velocities, dtype=float)
        states = np.hstack([positions, velocities])
    else:
        states = _positions_to_states(timestamps, positions)
    return timestamps, states


def _load_dual_npz(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    data = np.load(path)
    sim_ts = np.asarray(data['sim_timestamps'], dtype=float).ravel()
    sim_pos = np.asarray(data['sim_positions'], dtype=float)
    real_ts = np.asarray(data['real_timestamps'], dtype=float).ravel()
    real_pos = np.asarray(data['real_positions'], dtype=float)
    return (
        sim_ts,
        _positions_to_states(sim_ts, sim_pos),
        real_ts,
        _positions_to_states(real_ts, real_pos),
    )


def _trajectory_to_states(traj: LeRobotTrajectory) -> np.ndarray:
    return traj.full_state()


def compare_offline(
    sim_ts: np.ndarray,
    sim_states: np.ndarray,
    real_ts: np.ndarray,
    real_states: np.ndarray,
    cfg: MetricsConfig | None = None,
    baseline_errors: np.ndarray | None = None,
) -> dict:
    """Run offline distribution comparison and return a JSON-serializable dict."""
    cfg = cfg or MetricsConfig()
    result = compute_distribution_metrics(
        sim_ts,
        sim_states,
        real_ts,
        real_states,
        baseline_errors=baseline_errors,
        cfg=cfg,
    )
    n_dof = sim_states.shape[1] // 2 if sim_states.ndim == 2 and sim_states.shape[1] > 0 else 0
    return {
        'sample_count': result.sample_count,
        'kl_divergence_per_joint': result.kl_per_joint,
        'kl_divergence_mean': result.kl_mean,
        'wasserstein_per_joint': result.w1_per_joint,
        'wasserstein_mean': result.w1_mean,
        'shift_detected_w1': result.shift_detected_w1,
        'mmd_statistic': result.mmd_statistic,
        'mmd_p_value': result.mmd_p_value,
        'shift_detected': result.shift_detected,
        'detection_method': result.detection_method,
        'n_dof': n_dof,
        'baseline_frac': cfg.baseline_frac,
    }


def compare_dual_npz(path: Path, cfg: MetricsConfig | None = None) -> dict:
    """Compare Sim/Real streams from a dual-source capture NPZ."""
    sim_ts, sim_states, real_ts, real_states = _load_dual_npz(path)
    report = compare_offline(sim_ts, sim_states, real_ts, real_states, cfg=cfg)
    report['dual_npz'] = str(path)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Offline Sim/Real distribution comparison (LeRobot Real source).',
    )
    parser.add_argument('--real-dataset', type=Path, help='LeRobot dataset root')
    parser.add_argument('--sim-npz', type=Path, help='Sim trajectory as .npz (timestamps, positions)')
    parser.add_argument('--sim-dataset', type=Path, help='Optional second LeRobot dataset as Sim source')
    parser.add_argument('--output', type=Path, help='Write JSON results to this path')
    parser.add_argument('--dual-npz', type=Path, help='Dual-source NPZ (sim_* + real_* arrays)')
    parser.add_argument('--min-samples', type=int, default=50)
    parser.add_argument('--mmd-permutations', type=int, default=100)
    parser.add_argument(
        '--baseline-frac',
        type=float,
        default=0.0,
        help='Use first fraction of aligned errors as KL/W1 baseline (same-task calibration)',
    )
    args = parser.parse_args(argv)

    cfg = MetricsConfig(
        min_samples=args.min_samples,
        mmd_permutation_count=args.mmd_permutations,
        baseline_frac=args.baseline_frac,
    )

    if args.dual_npz:
        report = compare_dual_npz(args.dual_npz, cfg=cfg)
    else:
        if not args.real_dataset:
            print('Provide --real-dataset, or --dual-npz for dual-source capture.', file=sys.stderr)
            return 2
        real_traj = load_lerobot_dataset(args.real_dataset)
        real_ts = real_traj.timestamps
        real_states = _trajectory_to_states(real_traj)

        if args.sim_npz:
            sim_ts, sim_states = _load_npz_trajectory(args.sim_npz)
        elif args.sim_dataset:
            sim_traj = load_lerobot_dataset(args.sim_dataset)
            sim_ts = sim_traj.timestamps
            sim_states = _trajectory_to_states(sim_traj)
        else:
            print('Provide --sim-npz or --sim-dataset for the Sim trajectory.', file=sys.stderr)
            return 2

        report = compare_offline(sim_ts, sim_states, real_ts, real_states, cfg=cfg)
    text = json.dumps(report, indent=2)
    print(text)

    if args.output:
        args.output.write_text(text, encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
