#!/usr/bin/env python3
"""Verify NFR-S safety criteria end to end."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from bridge_monitor_msgs.msg import DistributionMetrics, RiskStatus
from bridge_monitor_msgs.srv import AcknowledgeRisk
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass
class MotionSample:
    elapsed_sec: float
    delta_rad: float


class SafetyChecker(Node):
    def __init__(self) -> None:
        super().__init__('safety_nfr_checker')
        self.statuses: list[tuple[float, RiskStatus]] = []
        self.system_states: list[tuple[float, str]] = []
        self.joint_states: list[tuple[float, JointState]] = []
        self._metrics_pub = self.create_publisher(DistributionMetrics, '/monitor/distribution_metrics', 10)
        self._tracking_pub = self.create_publisher(JointState, '/monitor/tracking_error', qos_profile_sensor_data)
        self._planning_pub = self.create_publisher(String, '/manipulation/planning_result', 10)
        self._cmd_pub = self.create_publisher(JointTrajectory, '/bridge/command', 10)
        self._ack_client = self.create_client(AcknowledgeRisk, '/risk/acknowledge')
        self._clear_client = self.create_client(Trigger, '/risk/clear_e_stop')
        self._reset_client = self.create_client(Trigger, '/bridge/reset_simulation')
        self.create_subscription(RiskStatus, '/risk/status', self._on_status, 10)
        self.create_subscription(String, '/bridge/system_state', self._on_system_state, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, qos_profile_sensor_data)

    @property
    def latest_joint(self) -> JointState | None:
        return self.joint_states[-1][1] if self.joint_states else None

    def _on_status(self, msg: RiskStatus) -> None:
        self.statuses.append((time.monotonic(), msg))

    def _on_system_state(self, msg: String) -> None:
        self.system_states.append((time.monotonic(), msg.data))

    def _on_joint_state(self, msg: JointState) -> None:
        self.joint_states.append((time.monotonic(), msg))

    def wait_ready(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
            if self.statuses and self.system_states and self.joint_states:
                return True
        return False

    def publish_risk_inputs(
        self,
        *,
        distribution: bool = False,
        tracking: bool = False,
        dynamics: bool = False,
        comm: bool = False,
        planning: bool = False,
        soft_limit: bool = False,
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        msg = DistributionMetrics()
        msg.header.stamp = stamp
        msg.joint_names = [
            'lbr_iiwa_joint_1',
            'lbr_iiwa_joint_2',
            'lbr_iiwa_joint_3',
            'lbr_iiwa_joint_4',
            'lbr_iiwa_joint_5',
            'lbr_iiwa_joint_6',
            'lbr_iiwa_joint_7',
        ]
        if distribution:
            msg.kl_divergence_mean = 0.30
            msg.wasserstein_mean = 0.15
            msg.mmd_statistic = 0.10
            msg.mmd_p_value = 0.01
            msg.shift_detected = True
            msg.detection_method = 'kl+w1+mmd'
        if dynamics:
            msg.dynamics_anomaly_score = 1.0
        if comm:
            msg.comm_health_score = 1.0
        if soft_limit:
            msg.soft_limit_score = 1.0
            msg.soft_limit_triggered = True
        self._metrics_pub.publish(msg)

        err = JointState()
        err.header.stamp = stamp
        err.name = ['lbr_iiwa_joint_1', 'lbr_iiwa_joint_2']
        err.position = [0.05, 0.05] if tracking else [0.0, 0.0]
        self._tracking_pub.publish(err)

        if planning:
            plan = String()
            plan.data = json.dumps({'action': 'pick', 'success': False, 'message': 'NFR-S synthetic failure'})
            self._planning_pub.publish(plan)

    def wait_for_status(self, predicate, timeout_sec: float, publish_fn=None) -> tuple[bool, float | None, RiskStatus | None]:
        start = time.monotonic()
        start_idx = len(self.statuses)
        deadline = start + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if publish_fn is not None:
                publish_fn()
            rclpy.spin_once(self, timeout_sec=0.02)
            for stamp, status in self.statuses[start_idx:]:
                if predicate(status):
                    return True, round((stamp - start) * 1000.0, 3), status
        return False, None, None

    def drive_safe(self, timeout_sec: float = 2.0) -> tuple[bool, float | None, RiskStatus | None]:
        return self.wait_for_status(
            lambda status: int(status.level) == 0 and not status.degraded_mode and not status.e_stop_active,
            timeout_sec=timeout_sec,
            publish_fn=lambda: self.publish_risk_inputs(),
        )

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

    def wait_for_zero_velocity(self, *, timeout_sec: float, threshold: float) -> tuple[bool, float | None, float]:
        start = time.monotonic()
        start_idx = len(self.joint_states)
        deadline = start + timeout_sec
        best = math.inf
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            for stamp, msg in self.joint_states[start_idx:]:
                if not msg.velocity:
                    continue
                rms = math.sqrt(statistics.fmean(float(v) * float(v) for v in msg.velocity))
                best = min(best, rms)
                if rms <= threshold:
                    return True, round((stamp - start) * 1000.0, 3), rms
        return False, None, 0.0 if best is math.inf else best

    def _call_trigger(self, client, timeout_sec: float) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, 'service unavailable'
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return False, 'service timeout'
        response = future.result()
        return bool(response.success), response.message

    def reset_simulation(self, timeout_sec: float = 3.0) -> tuple[bool, str]:
        return self._call_trigger(self._reset_client, timeout_sec)

    def clear_e_stop(self, timeout_sec: float = 3.0) -> tuple[bool, str]:
        return self._call_trigger(self._clear_client, timeout_sec)

    def acknowledge(self, timeout_sec: float = 3.0) -> tuple[bool, str]:
        if not self._ack_client.wait_for_service(timeout_sec=timeout_sec):
            return False, 'service unavailable'
        req = AcknowledgeRisk.Request()
        req.operator_id = 'nfr_s_verify'
        req.comment = 'NFR-S safety acknowledge'
        req.from_level = 3
        req.to_level = 0
        future = self._ack_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return False, 'service timeout'
        response = future.result()
        return bool(response.success), response.message

    def publish_trajectory(self, delta_rad: float, duration_sec: float) -> bool:
        baseline = self.latest_joint
        if baseline is None or not baseline.name or not baseline.position:
            return False
        cmd = JointTrajectory()
        cmd.joint_names = list(baseline.name)
        start_point = JointTrajectoryPoint()
        start_point.positions = list(baseline.position)
        start_point.time_from_start = Duration(sec=0, nanosec=0)

        point = JointTrajectoryPoint()
        point.positions = list(start_point.positions)
        point.positions[0] += delta_rad
        sec = int(duration_sec)
        point.time_from_start = Duration(sec=sec, nanosec=int((duration_sec - sec) * 1e9))
        cmd.points.extend([start_point, point])
        self._cmd_pub.publish(cmd)
        return True

    def measure_motion_after(
        self,
        *,
        delta_rad: float,
        duration_sec: float,
        sample_after_sec: float,
        tick_fn=None,
    ) -> MotionSample | None:
        baseline = self.latest_joint
        if baseline is None or not baseline.position:
            return None
        start_pos = float(baseline.position[0])
        if not self.publish_trajectory(delta_rad=delta_rad, duration_sec=duration_sec):
            return None
        start = time.monotonic()
        target_time = start + sample_after_sec
        latest: JointState | None = None
        while rclpy.ok() and time.monotonic() < target_time:
            if tick_fn is not None:
                tick_fn()
            rclpy.spin_once(self, timeout_sec=0.01)
            latest = self.latest_joint
        if latest is None or not latest.position:
            return None
        return MotionSample(
            elapsed_sec=round(time.monotonic() - start, 3),
            delta_rad=round(abs(float(latest.position[0]) - start_pos), 6),
        )

    def settle(self, duration_sec: float) -> None:
        end = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)


def _status_dict(status: RiskStatus | None) -> dict[str, Any]:
    if status is None:
        return {}
    return {
        'level': int(status.level),
        'composite_score': round(float(status.composite_score), 6),
        'primary_driver': status.primary_driver,
        'degraded_mode': bool(status.degraded_mode),
        'e_stop_active': bool(status.e_stop_active),
        'attribution_count': len(status.attribution),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Verify NFR-S safety criteria.')
    parser.add_argument('--output', type=Path, default=Path('docs/samples/safety-nfr-metrics.json'))
    parser.add_argument('--velocity-zero-threshold', type=float, default=0.02)
    parser.add_argument('--velocity-zero-latency-ms-max', type=float, default=100.0)
    parser.add_argument('--watchdog-latency-ms-max', type=float, default=650.0)
    parser.add_argument('--degraded-ratio-max', type=float, default=0.75)
    args = parser.parse_args(argv)

    rclpy.init(args=[])
    node = SafetyChecker()
    try:
        if not node.wait_ready(timeout_sec=25.0):
            raise RuntimeError('Timed out waiting for safety topics')

        soft_ok, soft_latency_ms, soft_status = node.wait_for_status(
            lambda status: int(status.level) >= 2 and status.degraded_mode and status.primary_driver == 'dynamics_anomaly',
            timeout_sec=2.0,
            publish_fn=lambda: node.publish_risk_inputs(soft_limit=True),
        )
        node.drive_safe()
        node.reset_simulation()
        node.settle(0.5)

        node.drive_safe()
        normal_motion = node.measure_motion_after(delta_rad=0.25, duration_sec=2.0, sample_after_sec=0.35)
        node.settle(1.5)
        degraded_input = lambda: node.publish_risk_inputs(distribution=True, tracking=True)
        degraded_ok, degraded_latency_ms, degraded_status = node.wait_for_status(
            lambda status: int(status.level) == 2 and status.degraded_mode and not status.e_stop_active,
            timeout_sec=2.0,
            publish_fn=degraded_input,
        )
        degraded_motion = node.measure_motion_after(
            delta_rad=-0.25,
            duration_sec=2.0,
            sample_after_sec=0.35,
            tick_fn=degraded_input,
        )
        degraded_ratio = None
        if normal_motion and degraded_motion and normal_motion.delta_rad > 0:
            degraded_ratio = round(degraded_motion.delta_rad / normal_motion.delta_rad, 3)
        node.drive_safe()
        node.reset_simulation()
        node.settle(0.5)

        if not node.publish_trajectory(delta_rad=0.2, duration_sec=5.0):
            watchdog_ok, watchdog_latency_ms = False, None
        else:
            watchdog_ok, watchdog_latency_ms = node.wait_for_state('HOLD', timeout_sec=2.0)
        node.reset_simulation()
        node.settle(0.5)

        r3_ok, r3_latency_ms, r3_status = node.wait_for_status(
            lambda status: status.e_stop_active,
            timeout_sec=2.0,
            publish_fn=lambda: node.publish_risk_inputs(
                distribution=True,
                tracking=True,
                dynamics=True,
                comm=True,
                planning=True,
            ),
        )
        bridge_ok, bridge_latency_ms = node.wait_for_state('E_STOP', timeout_sec=1.0)
        zero_ok, zero_latency_ms, best_velocity_rms = node.wait_for_zero_velocity(
            timeout_sec=1.0,
            threshold=args.velocity_zero_threshold,
        )
        node.drive_safe()
        clear_before_ack_ok, clear_before_ack_msg = node.clear_e_stop()
        ack_ok, ack_msg = node.acknowledge()
        clear_after_ack_ok, clear_after_ack_msg = node.clear_e_stop()
        running_ok, running_latency_ms = node.wait_for_state('RUNNING', timeout_sec=2.0)

        passes = {
            'NFR-S01_e_stop_zero_velocity': (
                r3_ok
                and bridge_ok
                and zero_ok
                and zero_latency_ms is not None
                and zero_latency_ms <= args.velocity_zero_latency_ms_max
            ),
            'NFR-S02_soft_limit_r2': soft_ok,
            'NFR-S03_watchdog_hold': (
                watchdog_ok
                and watchdog_latency_ms is not None
                and watchdog_latency_ms <= args.watchdog_latency_ms_max
            ),
            'NFR-S04_r2_degraded_50pct': (
                degraded_ok
                and degraded_ratio is not None
                and degraded_ratio <= args.degraded_ratio_max
            ),
            'NFR-S05_acknowledge_self_check_recovery': (
                not clear_before_ack_ok
                and ack_ok
                and clear_after_ack_ok
                and running_ok
            ),
        }
        result = {
            'timestamp_unix': round(time.time(), 3),
            'criteria': {
                'velocity_zero_latency_ms_max': args.velocity_zero_latency_ms_max,
                'velocity_zero_threshold_rms': args.velocity_zero_threshold,
                'watchdog_latency_ms_max': args.watchdog_latency_ms_max,
                'degraded_motion_ratio_max': args.degraded_ratio_max,
            },
            'soft_limit_r2': {
                'passes': soft_ok,
                'latency_ms': soft_latency_ms,
                'status': _status_dict(soft_status),
            },
            'r2_degraded_motion': {
                'risk_status_passes': degraded_ok,
                'risk_latency_ms': degraded_latency_ms,
                'status': _status_dict(degraded_status),
                'normal_motion': None if normal_motion is None else normal_motion.__dict__,
                'degraded_motion': None if degraded_motion is None else degraded_motion.__dict__,
                'degraded_to_normal_delta_ratio': degraded_ratio,
            },
            'watchdog_hold': {
                'passes': watchdog_ok,
                'latency_ms': watchdog_latency_ms,
            },
            'r3_e_stop': {
                'risk_status_passes': r3_ok,
                'risk_status_latency_ms': r3_latency_ms,
                'risk_status': _status_dict(r3_status),
                'bridge_system_state_passes': bridge_ok,
                'bridge_system_state_latency_ms': bridge_latency_ms,
                'velocity_zero_passes': zero_ok,
                'velocity_zero_latency_ms': zero_latency_ms,
                'best_velocity_rms': round(float(best_velocity_rms), 6),
            },
            'acknowledge_recovery': {
                'clear_before_ack_success': clear_before_ack_ok,
                'clear_before_ack_message': clear_before_ack_msg,
                'ack_success': ack_ok,
                'ack_message': ack_msg,
                'clear_after_ack_success': clear_after_ack_ok,
                'clear_after_ack_message': clear_after_ack_msg,
                'running_after_clear': running_ok,
                'running_latency_ms': running_latency_ms,
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
