#!/usr/bin/env python3
"""Record /bridge/sim/joint_states to NPZ for cross-source offline_compare."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def _stamp_sec(msg: JointState) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


class SimTrajectoryRecorder(Node):
    """Collect Sim-Source joint states from bridge during portfolio_demo."""

    def __init__(self) -> None:
        super().__init__('sim_trajectory_recorder')
        self._timestamps: list[float] = []
        self._positions: list[list[float]] = []
        self._velocities: list[list[float]] = []
        self._joint_names: list[str] = []
        self._t0: float | None = None
        self.create_subscription(
            JointState,
            '/bridge/sim/joint_states',
            self._on_joint_state,
            qos_profile_sensor_data,
        )

    @property
    def sample_count(self) -> int:
        return len(self._timestamps)

    def _on_joint_state(self, msg: JointState) -> None:
        t = _stamp_sec(msg)
        if self._t0 is None:
            self._t0 = t
        if msg.name and not self._joint_names:
            self._joint_names = list(msg.name)
        self._timestamps.append(t - self._t0)
        self._positions.append(list(msg.position))
        vel = list(msg.velocity) if msg.velocity else [0.0] * len(msg.position)
        self._velocities.append(vel)

    def save(self, path: Path) -> None:
        if not self._timestamps:
            raise RuntimeError('No /bridge/sim/joint_states samples recorded')
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            path,
            timestamps=np.asarray(self._timestamps, dtype=float),
            positions=np.asarray(self._positions, dtype=float),
            velocities=np.asarray(self._velocities, dtype=float),
            joint_names=np.asarray(self._joint_names, dtype=object),
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Capture bridge Sim trajectory to NPZ.')
    parser.add_argument('--duration', type=float, default=15.0, help='Recording duration (sec)')
    parser.add_argument('--output', type=Path, required=True, help='Output .npz path')
    parser.add_argument('--min-samples', type=int, default=50, help='Minimum samples required')
    args = parser.parse_args(argv)

    rclpy.init(args=argv)
    node = SimTrajectoryRecorder()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        count = node.sample_count
        if count < args.min_samples:
            node.destroy_node()
            rclpy.shutdown()
            print(
                f'[ERROR] Only {count} samples (need {args.min_samples}). '
                'Is portfolio_demo running?',
                file=sys.stderr,
            )
            return 1
        node.save(args.output)
        print(f'[PASS] Recorded {count} samples → {args.output}')
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
