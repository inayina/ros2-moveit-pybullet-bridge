#!/usr/bin/env python3
"""Verify NFR-P performance criteria with live ROS measurements."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from bridge_monitor_msgs.msg import DistributionMetrics
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass
class TimedSamples:
    header_times: list[float] = field(default_factory=list)
    receive_times: list[float] = field(default_factory=list)

    def add(self, stamp_sec: float) -> None:
        self.header_times.append(stamp_sec)
        self.receive_times.append(time.monotonic())


def _stamp_sec(msg) -> float:
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


def _frequency_metrics(times: list[float], expected_hz: float) -> dict[str, Any]:
    intervals = [b - a for a, b in zip(times, times[1:]) if b > a]
    if not intervals:
        return {
            'samples': len(times),
            'mean_hz': 0.0,
            'mean_error_pct': 100.0,
            'jitter_pct': 100.0,
            'max_gap_ms': None,
        }
    mean_interval = statistics.fmean(intervals)
    mean_hz = 1.0 / mean_interval if mean_interval > 0.0 else 0.0
    stdev = statistics.pstdev(intervals) if len(intervals) > 1 else 0.0
    return {
        'samples': len(times),
        'mean_hz': round(mean_hz, 3),
        'mean_error_pct': round(abs(mean_hz - expected_hz) / expected_hz * 100.0, 3),
        'jitter_pct': round(stdev / (1.0 / expected_hz) * 100.0, 3),
        'max_gap_ms': round(max(intervals) * 1000.0, 3),
    }


class PerformanceChecker(Node):
    def __init__(self) -> None:
        super().__init__('performance_nfr_checker')
        self.joint = TimedSamples()
        self.monitor = TimedSamples()
        self.latest_joint: JointState | None = None
        self.performance_samples: list[dict[str, float | int | bool]] = []
        self._cmd_pub = self.create_publisher(JointTrajectory, '/bridge/command', 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint, 10)
        self.create_subscription(
            DistributionMetrics,
            '/monitor/distribution_metrics',
            self._on_monitor,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            '/bridge/performance',
            self._on_performance,
            qos_profile_sensor_data,
        )

    def _on_joint(self, msg: JointState) -> None:
        self.latest_joint = msg
        self.joint.add(_stamp_sec(msg))

    def _on_monitor(self, msg: DistributionMetrics) -> None:
        self.monitor.add(_stamp_sec(msg))

    def _on_performance(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != 'pybullet_bridge.performance':
                continue
            values = {item.key: item.value for item in status.values}
            try:
                self.performance_samples.append({
                    'source_count': int(values.get('source_count', '0')),
                    'dual_source_enabled': values.get('dual_source_enabled', 'false') == 'true',
                    'physics_total_hz': float(values.get('physics_total_hz', '0')),
                    'physics_per_source_hz': float(values.get('physics_per_source_hz', '0')),
                    'realtime_factor': float(values.get('realtime_factor', '0')),
                })
            except ValueError:
                continue

    def wait_ready(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.latest_joint is not None and self.monitor.header_times and self.performance_samples:
                return True
        return False

    def reset_samples(self) -> None:
        self.joint = TimedSamples()
        self.monitor = TimedSamples()
        self.performance_samples.clear()

    def collect(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.005)

    def measure_control_latency(
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
            baseline = self.latest_joint
            if baseline is None or not baseline.name or not baseline.position:
                break

            target = list(baseline.position)
            before = float(target[0])
            target[0] = before + sign * delta_rad
            sign *= -1.0

            cmd = JointTrajectory()
            cmd.joint_names = list(baseline.name)
            point = JointTrajectoryPoint()
            point.positions = target
            point.time_from_start = Duration(sec=0, nanosec=0)
            cmd.points.append(point)

            start = time.monotonic()
            self._cmd_pub.publish(cmd)
            deadline = start + timeout_sec
            observed = False
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.005)
                current = self.latest_joint
                if current is None or not current.position:
                    continue
                if abs(float(current.position[0]) - before) >= movement_epsilon:
                    latencies_ms.append((time.monotonic() - start) * 1000.0)
                    observed = True
                    break
            if not observed:
                latencies_ms.append(timeout_sec * 1000.0)

            settle_until = time.monotonic() + 0.08
            while rclpy.ok() and time.monotonic() < settle_until:
                rclpy.spin_once(self, timeout_sec=0.005)
        return latencies_ms


def _summarize_performance(samples: list[dict[str, Any]]) -> dict[str, Any]:
    if not samples:
        return {
            'samples': 0,
            'dual_source_enabled': False,
            'source_count': 0,
            'mean_physics_per_source_hz': 0.0,
            'min_physics_per_source_hz': 0.0,
            'mean_realtime_factor': 0.0,
            'min_realtime_factor': 0.0,
        }
    per_source = [float(s['physics_per_source_hz']) for s in samples]
    rtf = [float(s['realtime_factor']) for s in samples]
    return {
        'samples': len(samples),
        'dual_source_enabled': bool(samples[-1]['dual_source_enabled']),
        'source_count': int(samples[-1]['source_count']),
        'mean_physics_per_source_hz': round(statistics.fmean(per_source), 3),
        'min_physics_per_source_hz': round(min(per_source), 3),
        'mean_realtime_factor': round(statistics.fmean(rtf), 3),
        'min_realtime_factor': round(min(rtf), 3),
    }


def _load_hoc_metrics(path: Path, min_hz: float) -> dict[str, Any]:
    if not path.is_file():
        return {'source': str(path), 'available': False, 'passes': False}
    data = json.loads(path.read_text(encoding='utf-8'))
    streams = data.get('streams', {})
    hz = {
        topic: stream.get('mean_hz', 0.0)
        for topic, stream in streams.items()
        if isinstance(stream, dict)
    }
    return {
        'source': str(path),
        'available': True,
        'streams_hz': hz,
        'min_hz_observed': round(min(hz.values()), 3) if hz else 0.0,
        'passes': bool(hz) and all(float(value) >= min_hz for value in hz.values()),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Verify NFR-P performance metrics.')
    parser.add_argument('--output', type=Path, default=Path('docs/samples/performance-nfr-metrics.json'))
    parser.add_argument('--hoc-metrics', type=Path, default=Path('docs/samples/hoc-console-metrics.json'))
    parser.add_argument('--startup-timeout-sec', type=float, default=20.0)
    parser.add_argument('--sample-duration-sec', type=float, default=6.0)
    parser.add_argument('--control-trials', type=int, default=20)
    parser.add_argument('--control-p95-threshold-ms', type=float, default=50.0)
    parser.add_argument('--joint-hz', type=float, default=100.0)
    parser.add_argument('--joint-hz-tolerance-pct', type=float, default=5.0)
    parser.add_argument('--monitor-hz', type=float, default=10.0)
    parser.add_argument('--monitor-hz-tolerance-pct', type=float, default=10.0)
    parser.add_argument('--hoc-hz-min', type=float, default=5.0)
    parser.add_argument('--physics-hz', type=float, default=240.0)
    parser.add_argument('--realtime-factor-min', type=float, default=0.8)
    args = parser.parse_args(argv)

    rclpy.init(args=[])
    node = PerformanceChecker()
    try:
        if not node.wait_ready(args.startup_timeout_sec):
            raise RuntimeError('Timed out waiting for /joint_states, /monitor/distribution_metrics, /bridge/performance')
        node.reset_samples()
        node.collect(args.sample_duration_sec)
        control_latencies = node.measure_control_latency(
            trials=args.control_trials,
            delta_rad=0.025,
            movement_epsilon=0.001,
            timeout_sec=0.5,
        )

        joint_freq = _frequency_metrics(node.joint.header_times, args.joint_hz)
        monitor_freq = _frequency_metrics(node.monitor.header_times, args.monitor_hz)
        perf = _summarize_performance(node.performance_samples)
        hoc = _load_hoc_metrics(args.hoc_metrics, args.hoc_hz_min)
        control = {
            'trials': len(control_latencies),
            'mean_ms': round(statistics.fmean(control_latencies), 3) if control_latencies else None,
            'p95_ms': round(_percentile(control_latencies, 95), 3) if control_latencies else None,
            'max_ms': round(max(control_latencies), 3) if control_latencies else None,
            'threshold_ms': args.control_p95_threshold_ms,
        }

        passes = {
            'NFR-P01_control_loop_p95': bool(control_latencies)
            and control['p95_ms'] <= args.control_p95_threshold_ms,
            'NFR-P02_joint_state_100hz': joint_freq['samples'] >= 200
            and joint_freq['mean_error_pct'] <= args.joint_hz_tolerance_pct,
            'NFR-P03_monitor_10hz': monitor_freq['samples'] >= 30
            and monitor_freq['mean_error_pct'] <= args.monitor_hz_tolerance_pct,
            'NFR-P04_hoc_5hz': hoc['passes'],
            'NFR-P05_dual_240hz_rtf': perf['dual_source_enabled']
            and perf['source_count'] == 2
            and perf['min_physics_per_source_hz'] >= args.physics_hz * args.realtime_factor_min
            and perf['min_realtime_factor'] >= args.realtime_factor_min,
        }
        result = {
            'timestamp_unix': round(time.time(), 3),
            'criteria': {
                'control_loop_p95_ms_max': args.control_p95_threshold_ms,
                'joint_state_hz': args.joint_hz,
                'joint_state_tolerance_pct': args.joint_hz_tolerance_pct,
                'monitor_hz': args.monitor_hz,
                'monitor_tolerance_pct': args.monitor_hz_tolerance_pct,
                'hoc_hz_min': args.hoc_hz_min,
                'physics_per_source_hz_target': args.physics_hz,
                'realtime_factor_min': args.realtime_factor_min,
            },
            'control_loop_latency': control,
            'joint_state_frequency': joint_freq,
            'monitor_frequency': monitor_freq,
            'bridge_physics': perf,
            'hoc_refresh': hoc,
            'passes': passes,
            'overall_passes': all(passes.values()),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result['overall_passes'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
