#!/usr/bin/env python3
"""Measure bridge command latency and joint-state publish stability."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass
class TopicSamples:
    receive_times: list[float] = field(default_factory=list)
    header_times: list[float] = field(default_factory=list)
    last_msg: JointState | None = None

    def append(self, msg: JointState) -> None:
        self.receive_times.append(time.monotonic())
        self.header_times.append(_stamp_sec(msg))
        self.last_msg = msg


def _stamp_sec(msg: JointState) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * percentile / 100.0
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return ordered[lo]
    frac = rank - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def _interval_metrics(times: list[float], expected_hz: float) -> dict[str, float | int]:
    if len(times) < 2:
        return {
            'samples': len(times),
            'mean_hz': 0.0,
            'mean_error_pct': 100.0,
            'jitter_pct': 100.0,
            'max_gap_ms': math.nan,
        }

    intervals = [b - a for a, b in zip(times, times[1:]) if b > a]
    if not intervals:
        return {
            'samples': len(times),
            'mean_hz': 0.0,
            'mean_error_pct': 100.0,
            'jitter_pct': 100.0,
            'max_gap_ms': math.nan,
        }

    mean_interval = statistics.fmean(intervals)
    mean_hz = 1.0 / mean_interval if mean_interval > 0.0 else 0.0
    expected_period = 1.0 / expected_hz
    stdev = statistics.pstdev(intervals) if len(intervals) > 1 else 0.0
    return {
        'samples': len(times),
        'mean_hz': round(mean_hz, 3),
        'mean_error_pct': round(abs(mean_hz - expected_hz) / expected_hz * 100.0, 3),
        'jitter_pct': round(stdev / expected_period * 100.0, 3),
        'max_gap_ms': round(max(intervals) * 1000.0, 3),
    }


def _topic_metrics(samples: TopicSamples, expected_hz: float) -> dict[str, float | int | bool | dict]:
    publish = _interval_metrics(samples.header_times, expected_hz)
    receive = _interval_metrics(samples.receive_times, expected_hz)
    return {
        'samples': publish['samples'],
        'mean_hz': publish['mean_hz'],
        'mean_error_pct': publish['mean_error_pct'],
        'jitter_pct': publish['jitter_pct'],
        'max_gap_ms': publish['max_gap_ms'],
        'basis': 'header_stamp',
        'receive_diagnostics': receive,
    }


class BridgeCommChecker(Node):
    def __init__(self) -> None:
        super().__init__('bridge_comm_checker')
        self.topics: dict[str, TopicSamples] = {
            '/joint_states': TopicSamples(),
            '/bridge/sim/joint_states': TopicSamples(),
            '/bridge/real/joint_states': TopicSamples(),
        }
        self._cmd_pub = self.create_publisher(JointTrajectory, '/bridge/command', 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint, 10)
        self.create_subscription(
            JointState,
            '/bridge/sim/joint_states',
            self._on_sim,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            '/bridge/real/joint_states',
            self._on_real,
            qos_profile_sensor_data,
        )

    @property
    def latest_joint_state(self) -> JointState | None:
        return self.topics['/joint_states'].last_msg

    def _on_joint(self, msg: JointState) -> None:
        self.topics['/joint_states'].append(msg)

    def _on_sim(self, msg: JointState) -> None:
        self.topics['/bridge/sim/joint_states'].append(msg)

    def _on_real(self, msg: JointState) -> None:
        self.topics['/bridge/real/joint_states'].append(msg)

    def wait_for_topics(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if all(topic.last_msg is not None for topic in self.topics.values()):
                return True
        return False

    def reset_samples(self) -> None:
        for samples in self.topics.values():
            samples.receive_times.clear()
            samples.header_times.clear()

    def sample_frequency(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.001)

    def measure_command_latency(
        self,
        *,
        trials: int,
        delta_rad: float,
        movement_epsilon: float,
        timeout_sec: float,
    ) -> list[float]:
        latencies_ms: list[float] = []
        sign = 1.0
        for _ in range(trials):
            baseline = self.latest_joint_state
            if baseline is None or not baseline.name or not baseline.position:
                break

            joint_names = list(baseline.name)
            target = list(baseline.position)
            target[0] += sign * delta_rad
            sign *= -1.0

            msg = JointTrajectory()
            msg.joint_names = joint_names
            point = JointTrajectoryPoint()
            point.positions = target
            point.time_from_start = Duration(sec=0, nanosec=0)
            msg.points.append(point)

            before = list(baseline.position)
            t_publish = time.monotonic()
            self._cmd_pub.publish(msg)

            deadline = t_publish + timeout_sec
            observed = False
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.01)
                current = self.latest_joint_state
                if current is None or len(current.position) <= 0:
                    continue
                if abs(float(current.position[0]) - float(before[0])) >= movement_epsilon:
                    latencies_ms.append((time.monotonic() - t_publish) * 1000.0)
                    observed = True
                    break
            if not observed:
                latencies_ms.append(timeout_sec * 1000.0)

            # Let PyBullet settle on the previous target before the next step.
            settle_deadline = time.monotonic() + 0.15
            while rclpy.ok() and time.monotonic() < settle_deadline:
                rclpy.spin_once(self, timeout_sec=0.02)

        return latencies_ms


def _build_result(
    *,
    topic_results: dict[str, dict[str, float | int | bool]],
    expected_hz: float,
    hz_tolerance_pct: float,
    jitter_tolerance_pct: float,
    latency_threshold_ms: float,
    latency_ms: list[float],
    min_samples: int,
) -> dict:
    for metrics in topic_results.values():
        metrics['passes'] = (
            metrics['samples'] >= min_samples
            and metrics['mean_error_pct'] <= hz_tolerance_pct
            and metrics['jitter_pct'] <= jitter_tolerance_pct
        )

    latency_summary = {
        'trials': len(latency_ms),
        'mean_ms': round(statistics.fmean(latency_ms), 3) if latency_ms else math.nan,
        'p95_ms': round(_percentile(latency_ms, 95.0), 3) if latency_ms else math.nan,
        'p99_ms': round(_percentile(latency_ms, 99.0), 3) if latency_ms else math.nan,
        'max_ms': round(max(latency_ms), 3) if latency_ms else math.nan,
        'threshold_ms': latency_threshold_ms,
    }
    latency_summary['passes'] = (
        len(latency_ms) > 0 and latency_summary['p99_ms'] <= latency_threshold_ms
    )

    return {
        'timestamp_unix': round(time.time(), 3),
        'criteria': {
            'expected_publish_hz': expected_hz,
            'mean_frequency_error_pct_max': hz_tolerance_pct,
            'jitter_pct_max': jitter_tolerance_pct,
            'command_feedback_p99_ms_max': latency_threshold_ms,
            'min_samples_per_topic': min_samples,
        },
        'topics': topic_results,
        'command_feedback_latency': latency_summary,
        'overall_passes': all(m['passes'] for m in topic_results.values())
        and latency_summary['passes'],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Verify FR-BRG bridge communication timing evidence.',
    )
    parser.add_argument('--output', type=Path, default=Path('docs/samples/bridge-comm-metrics.json'))
    parser.add_argument('--expected-hz', type=float, default=100.0)
    parser.add_argument('--sample-duration-sec', type=float, default=3.0)
    parser.add_argument('--startup-timeout-sec', type=float, default=15.0)
    parser.add_argument('--hz-tolerance-pct', type=float, default=5.0)
    parser.add_argument('--jitter-tolerance-pct', type=float, default=5.0)
    parser.add_argument('--min-samples', type=int, default=200)
    parser.add_argument('--latency-threshold-ms', type=float, default=20.0)
    parser.add_argument('--latency-trials', type=int, default=12)
    parser.add_argument('--delta-rad', type=float, default=0.035)
    parser.add_argument('--movement-epsilon', type=float, default=0.0005)
    parser.add_argument('--latency-timeout-sec', type=float, default=0.25)
    args = parser.parse_args(argv)

    rclpy.init(args=[])
    checker = BridgeCommChecker()
    try:
        if not checker.wait_for_topics(args.startup_timeout_sec):
            missing = [
                name for name, samples in checker.topics.items()
                if samples.last_msg is None
            ]
            print(f'[FAIL] Missing bridge topics: {missing}', file=sys.stderr)
            return 1

        checker.reset_samples()
        checker.sample_frequency(args.sample_duration_sec)
        topic_results = {
            name: _topic_metrics(samples, args.expected_hz)
            for name, samples in checker.topics.items()
        }
        latency_ms = checker.measure_command_latency(
            trials=args.latency_trials,
            delta_rad=args.delta_rad,
            movement_epsilon=args.movement_epsilon,
            timeout_sec=args.latency_timeout_sec,
        )
        result = _build_result(
            topic_results=topic_results,
            expected_hz=args.expected_hz,
            hz_tolerance_pct=args.hz_tolerance_pct,
            jitter_tolerance_pct=args.jitter_tolerance_pct,
            latency_threshold_ms=args.latency_threshold_ms,
            latency_ms=latency_ms,
            min_samples=args.min_samples,
        )

        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if not result['overall_passes']:
            print(f'[FAIL] Bridge communication criteria failed: {args.output}', file=sys.stderr)
            return 1
        print(f'[PASS] Bridge communication criteria met: {args.output}')
        return 0
    finally:
        checker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
