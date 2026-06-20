#!/usr/bin/env python3
"""Record /bridge/sim and /bridge/real joint_states to NPZ for same-task calibration."""

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


class DualSourceRecorder(Node):
    def __init__(self) -> None:
        super().__init__('dual_source_recorder')
        self._sim_t: list[float] = []
        self._sim_pos: list[list[float]] = []
        self._real_t: list[float] = []
        self._real_pos: list[list[float]] = []
        self._joint_names: list[str] = []
        self._t0: float | None = None

        self.create_subscription(
            JointState, '/bridge/sim/joint_states', self._on_sim, qos_profile_sensor_data)
        self.create_subscription(
            JointState, '/bridge/real/joint_states', self._on_real, qos_profile_sensor_data)

    @property
    def sim_count(self) -> int:
        return len(self._sim_t)

    def _on_sim(self, msg: JointState) -> None:
        t = _stamp_sec(msg)
        if self._t0 is None:
            self._t0 = t
        if msg.name and not self._joint_names:
            self._joint_names = list(msg.name)
        self._sim_t.append(t - self._t0)
        self._sim_pos.append(list(msg.position))

    def _on_real(self, msg: JointState) -> None:
        t = _stamp_sec(msg)
        if self._t0 is None:
            self._t0 = t
        self._real_t.append(t - self._t0)
        self._real_pos.append(list(msg.position))

    def save(self, path: Path) -> None:
        if not self._sim_t or not self._real_t:
            raise RuntimeError(
                f'Incomplete capture: sim={len(self._sim_t)} real={len(self._real_t)}',
            )
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            path,
            sim_timestamps=np.asarray(self._sim_t, dtype=float),
            sim_positions=np.asarray(self._sim_pos, dtype=float),
            real_timestamps=np.asarray(self._real_t, dtype=float),
            real_positions=np.asarray(self._real_pos, dtype=float),
            joint_names=np.asarray(self._joint_names, dtype=object),
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Capture dual-source Sim/Real NPZ.')
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--min-samples', type=int, default=50)
    args = parser.parse_args(argv)

    rclpy.init(args=argv)
    node = DualSourceRecorder()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node.sim_count < args.min_samples:
            node.destroy_node()
            rclpy.shutdown()
            print(f'[ERROR] sim samples={node.sim_count} (need {args.min_samples})', file=sys.stderr)
            return 1
        node.save(args.output)
        print(
            f'[PASS] Dual-source recorded sim={node.sim_count} real={len(node._real_t)} '
            f'→ {args.output}',
        )
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
