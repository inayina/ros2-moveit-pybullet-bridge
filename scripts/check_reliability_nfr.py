#!/usr/bin/env python3
"""Verify NFR-R reliability evidence for watchdog, recovery, smoke, and rosbag."""

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
from bridge_monitor_msgs.msg import DistributionMetrics, RiskStatus
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass
class TopicRecorder:
    times: list[float] = field(default_factory=list)

    def add(self) -> None:
        self.times.append(time.monotonic())

    def summary(self) -> dict[str, Any]:
        intervals = [b - a for a, b in zip(self.times, self.times[1:]) if b > a]
        return {
            'samples': len(self.times),
            'mean_hz': round((len(intervals) / sum(intervals)), 3) if intervals and sum(intervals) > 0 else 0.0,
            'max_gap_sec': round(max(intervals), 3) if intervals else None,
        }


class ReliabilityChecker(Node):
    def __init__(self) -> None:
        super().__init__('reliability_nfr_checker')
        self.topics = {
            '/joint_states': TopicRecorder(),
            '/monitor/distribution_metrics': TopicRecorder(),
            '/risk/status': TopicRecorder(),
            '/bridge/performance': TopicRecorder(),
        }
        self.system_states: list[tuple[float, str]] = []
        self.latest_joint: JointState | None = None
        self._cmd_pub = self.create_publisher(JointTrajectory, '/bridge/command', 10)
        self._reset_client = self.create_client(Trigger, '/bridge/reset_simulation')
        self._start_recording_client = self.create_client(Trigger, '/hoc/start_recording')
        self._stop_recording_client = self.create_client(Trigger, '/hoc/stop_recording')
        self.create_subscription(JointState, '/joint_states', self._on_joint, qos_profile_sensor_data)
        self.create_subscription(DistributionMetrics, '/monitor/distribution_metrics', self._on_metrics, 10)
        self.create_subscription(RiskStatus, '/risk/status', self._on_risk, 10)
        self.create_subscription(DiagnosticArray, '/bridge/performance', self._on_performance, qos_profile_sensor_data)
        self.create_subscription(String, '/bridge/system_state', self._on_system_state, 10)

    def _on_joint(self, msg: JointState) -> None:
        self.latest_joint = msg
        self.topics['/joint_states'].add()

    def _on_metrics(self, _msg: DistributionMetrics) -> None:
        self.topics['/monitor/distribution_metrics'].add()

    def _on_risk(self, _msg: RiskStatus) -> None:
        self.topics['/risk/status'].add()

    def _on_performance(self, _msg: DiagnosticArray) -> None:
        self.topics['/bridge/performance'].add()

    def _on_system_state(self, msg: String) -> None:
        self.system_states.append((time.monotonic(), msg.data))

    def wait_ready(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
            if (
                self.latest_joint is not None
                and self.system_states
                and all(rec.times for rec in self.topics.values())
            ):
                return True
        return False

    def _call_trigger(self, client, timeout_sec: float) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, 'service unavailable'
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return False, 'service timeout'
        response = future.result()
        return bool(response.success), response.message

    def reset_simulation(self, timeout_sec: float) -> tuple[bool, str]:
        return self._call_trigger(self._reset_client, timeout_sec)

    def start_recording(self, timeout_sec: float) -> tuple[bool, str]:
        return self._call_trigger(self._start_recording_client, timeout_sec)

    def stop_recording(self, timeout_sec: float) -> tuple[bool, str]:
        return self._call_trigger(self._stop_recording_client, timeout_sec)

    def publish_long_trajectory(self, duration_sec: float) -> bool:
        baseline = self.latest_joint
        if baseline is None or not baseline.name or not baseline.position:
            return False
        cmd = JointTrajectory()
        cmd.joint_names = list(baseline.name)
        point = JointTrajectoryPoint()
        point.positions = list(baseline.position)
        point.positions[0] += 0.15
        sec = int(duration_sec)
        point.time_from_start = Duration(sec=sec, nanosec=int((duration_sec - sec) * 1e9))
        cmd.points.append(point)
        self._cmd_pub.publish(cmd)
        return True

    def wait_for_state(self, state: str, timeout_sec: float) -> tuple[bool, float | None]:
        start = time.monotonic()
        start_idx = len(self.system_states)
        deadline = start + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            for stamp, value in self.system_states[start_idx:]:
                if value == state:
                    return True, round((stamp - start) * 1000.0, 3)
        return False, None

    def collect(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def reset_topic_samples(self) -> None:
        for recorder in self.topics.values():
            recorder.times.clear()


def _parse_recording_path(message: str) -> str:
    for marker in ('Recording to ', 'Recording stopped: '):
        if marker in message:
            return message.split(marker, 1)[1].strip()
    return ''


def _inspect_bag(path: str) -> dict[str, Any]:
    if not path:
        return {'path': path, 'exists': False}
    bag = Path(path)
    metadata = bag / 'metadata.yaml'
    metadata_text = metadata.read_text(encoding='utf-8') if metadata.is_file() else ''
    db_files = sorted(p.name for p in bag.glob('*.db3')) + sorted(p.name for p in bag.glob('*.mcap'))
    required_topics = ['/monitor/distribution_metrics', '/risk/status', '/risk/alerts']
    return {
        'path': str(bag),
        'exists': bag.is_dir(),
        'metadata_exists': metadata.is_file(),
        'storage_files': db_files,
        'required_topics_present': all(topic in metadata_text for topic in required_topics),
        'metadata_size_bytes': len(metadata_text.encode('utf-8')),
    }


def _process_memory_summary() -> dict[str, Any]:
    rss_kb: list[int] = []
    for status_path in Path('/proc').glob('[0-9]*/status'):
        try:
            text = status_path.read_text(encoding='utf-8', errors='ignore')
        except OSError:
            continue
        if not any(name in text for name in ('bridge_node', 'monitor_node', 'risk_node', 'hoc_server')):
            continue
        for line in text.splitlines():
            if line.startswith('VmRSS:'):
                parts = line.split()
                if len(parts) >= 2:
                    rss_kb.append(int(parts[1]))
                break
    return {
        'processes_observed': len(rss_kb),
        'total_rss_mb': round(sum(rss_kb) / 1024.0, 3),
        'max_process_rss_mb': round(max(rss_kb) / 1024.0, 3) if rss_kb else 0.0,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Verify NFR-R reliability evidence.')
    parser.add_argument('--output', type=Path, default=Path('docs/samples/reliability-nfr-metrics.json'))
    parser.add_argument('--startup-timeout-sec', type=float, default=25.0)
    parser.add_argument('--watchdog-timeout-sec', type=float, default=2.0)
    parser.add_argument('--smoke-duration-sec', type=float, default=10.0)
    parser.add_argument('--recording-duration-sec', type=float, default=4.0)
    parser.add_argument('--max-joint-gap-sec', type=float, default=0.2)
    parser.add_argument('--max-monitor-gap-sec', type=float, default=0.3)
    args = parser.parse_args(argv)

    rclpy.init(args=[])
    node = ReliabilityChecker()
    try:
        if not node.wait_ready(args.startup_timeout_sec):
            raise RuntimeError('Timed out waiting for reliability topics/services')

        command_published = node.publish_long_trajectory(duration_sec=5.0)
        hold_ok, hold_latency_ms = node.wait_for_state('HOLD', args.watchdog_timeout_sec)
        reset_ok, reset_msg = node.reset_simulation(timeout_sec=3.0)
        running_ok, running_latency_ms = node.wait_for_state('RUNNING', timeout_sec=3.0)

        start_ok, start_msg = node.start_recording(timeout_sec=5.0)
        bag_path = _parse_recording_path(start_msg)
        node.reset_topic_samples()
        mem_before = _process_memory_summary()
        node.collect(args.recording_duration_sec)
        stop_ok, stop_msg = node.stop_recording(timeout_sec=5.0)
        mem_after = _process_memory_summary()
        if not bag_path:
            bag_path = _parse_recording_path(stop_msg)
        bag = _inspect_bag(bag_path)

        node.reset_topic_samples()
        node.collect(args.smoke_duration_sec)
        topic_summaries = {topic: recorder.summary() for topic, recorder in node.topics.items()}
        joint_gap = topic_summaries['/joint_states']['max_gap_sec'] or math.inf
        monitor_gap = topic_summaries['/monitor/distribution_metrics']['max_gap_sec'] or math.inf

        passes = {
            'watchdog_hold': command_published and hold_ok,
            'reset_recovery': reset_ok and running_ok,
            'short_smoke_continuity': (
                topic_summaries['/joint_states']['samples'] > 0
                and topic_summaries['/monitor/distribution_metrics']['samples'] > 0
                and topic_summaries['/risk/status']['samples'] > 0
                and topic_summaries['/bridge/performance']['samples'] > 0
                and joint_gap <= args.max_joint_gap_sec
                and monitor_gap <= args.max_monitor_gap_sec
            ),
            'rosbag_independent_recording': start_ok
            and stop_ok
            and bag.get('exists')
            and bag.get('metadata_exists')
            and bag.get('required_topics_present'),
        }
        result = {
            'timestamp_unix': round(time.time(), 3),
            'criteria': {
                'watchdog_hold_timeout_sec': args.watchdog_timeout_sec,
                'smoke_duration_sec': args.smoke_duration_sec,
                'recording_duration_sec': args.recording_duration_sec,
                'max_joint_gap_sec': args.max_joint_gap_sec,
                'max_monitor_gap_sec': args.max_monitor_gap_sec,
                'full_soak_target_sec': 7200,
            },
            'phase2_scope': {
                'process_supervisor_crash_stop': 'not implemented; current evidence covers watchdog HOLD and risk e-stop only',
                'persistent_state_snapshot': 'not implemented; current evidence covers reset-to-safe-home recovery only',
                'full_2h_soak': 'not run in this verification; short smoke records continuity and RSS snapshot',
            },
            'watchdog_hold': {
                'command_published': command_published,
                'passes': hold_ok,
                'latency_ms': hold_latency_ms,
            },
            'reset_recovery': {
                'reset_success': reset_ok,
                'reset_message': reset_msg,
                'running_after_reset': running_ok,
                'running_latency_ms': running_latency_ms,
            },
            'rosbag_recording': {
                'start_success': start_ok,
                'start_message': start_msg,
                'stop_success': stop_ok,
                'stop_message': stop_msg,
                'bag': bag,
            },
            'short_smoke': {
                'topics': topic_summaries,
                'memory_before': mem_before,
                'memory_after': mem_after,
                'rss_delta_mb': round(mem_after['total_rss_mb'] - mem_before['total_rss_mb'], 3),
            },
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
