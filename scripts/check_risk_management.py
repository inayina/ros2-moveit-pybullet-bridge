#!/usr/bin/env python3
"""Verify FR-RSK risk aggregation, e-stop, attribution, and acknowledge flow."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from bridge_monitor_msgs.msg import DistributionMetrics, RiskStatus
from bridge_monitor_msgs.srv import AcknowledgeRisk
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


@dataclass
class TransitionResult:
    target_level: int
    reached: bool
    latency_ms: float | None
    composite_score: float | None
    primary_driver: str
    attribution_count: int
    message: str


class RiskManagementChecker(Node):
    def __init__(self) -> None:
        super().__init__('risk_management_checker')
        self.statuses: list[tuple[float, RiskStatus]] = []
        self.system_states: list[tuple[float, str]] = []
        self.joint_states: list[tuple[float, JointState]] = []
        self._metrics_pub = self.create_publisher(
            DistributionMetrics, '/monitor/distribution_metrics', 10)
        self._tracking_pub = self.create_publisher(
            JointState, '/monitor/tracking_error', qos_profile_sensor_data)
        self._planning_pub = self.create_publisher(
            String, '/manipulation/planning_result', 10)
        self._ack_client = self.create_client(AcknowledgeRisk, '/risk/acknowledge')
        self._clear_client = self.create_client(Trigger, '/risk/clear_e_stop')
        self.create_subscription(RiskStatus, '/risk/status', self._on_status, 10)
        self.create_subscription(String, '/bridge/system_state', self._on_system_state, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, qos_profile_sensor_data)

    def _on_status(self, msg: RiskStatus) -> None:
        self.statuses.append((time.monotonic(), msg))

    def _on_system_state(self, msg: String) -> None:
        self.system_states.append((time.monotonic(), msg.data))

    def _on_joint_state(self, msg: JointState) -> None:
        self.joint_states.append((time.monotonic(), msg))

    def wait_ready(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.statuses and self.system_states and self.joint_states:
                return True
        return False

    def _publish_risk_inputs(
        self,
        *,
        distribution: bool,
        tracking: bool,
        dynamics: bool,
        comm: bool,
        planning: bool,
    ) -> None:
        msg = DistributionMetrics()
        msg.header.stamp = self.get_clock().now().to_msg()
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
        self._metrics_pub.publish(msg)

        err = JointState()
        err.header = msg.header
        err.name = ['lbr_iiwa_joint_1', 'lbr_iiwa_joint_2']
        err.position = [0.05, 0.05] if tracking else [0.0, 0.0]
        self._tracking_pub.publish(err)

        if planning:
            plan = String()
            plan.data = json.dumps({'action': 'pick', 'success': False, 'message': 'FR-RSK synthetic failure'})
            self._planning_pub.publish(plan)

    def drive_to_level(self, target_level: int, timeout_sec: float) -> TransitionResult:
        cases = {
            0: dict(distribution=False, tracking=False, dynamics=False, comm=False, planning=False),
            1: dict(distribution=True, tracking=False, dynamics=False, comm=False, planning=False),
            2: dict(distribution=True, tracking=True, dynamics=False, comm=False, planning=False),
            3: dict(distribution=True, tracking=True, dynamics=True, comm=True, planning=True),
        }
        start_idx = len(self.statuses)
        start = time.monotonic()
        deadline = start + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._publish_risk_inputs(**cases[target_level])
            rclpy.spin_once(self, timeout_sec=0.03)
            for stamp, status in self.statuses[start_idx:]:
                if int(status.level) >= target_level:
                    return TransitionResult(
                        target_level=target_level,
                        reached=True,
                        latency_ms=round((stamp - start) * 1000.0, 3),
                        composite_score=round(float(status.composite_score), 6),
                        primary_driver=status.primary_driver,
                        attribution_count=len(status.attribution),
                        message='target level reached',
                    )
        latest = self.statuses[-1][1] if self.statuses else None
        return TransitionResult(
            target_level=target_level,
            reached=False,
            latency_ms=None,
            composite_score=None if latest is None else round(float(latest.composite_score), 6),
            primary_driver='' if latest is None else latest.primary_driver,
            attribution_count=0 if latest is None else len(latest.attribution),
            message='timed out waiting for target level',
        )

    def wait_for_e_stop(self, timeout_sec: float) -> tuple[bool, float | None, RiskStatus | None]:
        start = time.monotonic()
        start_idx = len(self.statuses)
        deadline = start + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._publish_risk_inputs(
                distribution=True, tracking=True, dynamics=True, comm=True, planning=True)
            rclpy.spin_once(self, timeout_sec=0.02)
            for stamp, status in self.statuses[start_idx:]:
                if status.e_stop_active:
                    return True, round((stamp - start) * 1000.0, 3), status
        return False, None, None

    def wait_for_bridge_e_stop(self, timeout_sec: float) -> tuple[bool, float | None]:
        start = time.monotonic()
        start_idx = len(self.system_states)
        deadline = start + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            for stamp, state in self.system_states[start_idx:]:
                if state == 'E_STOP':
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

    def clear_e_stop(self, timeout_sec: float) -> tuple[bool, str]:
        if not self._clear_client.wait_for_service(timeout_sec=timeout_sec):
            return False, '/risk/clear_e_stop unavailable'
        future = self._clear_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return False, 'clear_e_stop timed out'
        response = future.result()
        return bool(response.success), response.message

    def acknowledge(self, timeout_sec: float) -> tuple[bool, str]:
        if not self._ack_client.wait_for_service(timeout_sec=timeout_sec):
            return False, '/risk/acknowledge unavailable'
        request = AcknowledgeRisk.Request()
        request.from_level = 3
        request.to_level = 0
        request.operator_id = 'fr_rsk_verify'
        request.comment = 'FR-RSK verification acknowledge'
        future = self._ack_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return False, 'acknowledge timed out'
        response = future.result()
        return bool(response.success), response.message


def _transition_to_dict(item: TransitionResult) -> dict:
    return {
        'target_level': item.target_level,
        'reached': item.reached,
        'latency_ms': item.latency_ms,
        'composite_score': item.composite_score,
        'primary_driver': item.primary_driver,
        'attribution_count': item.attribution_count,
        'message': item.message,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Verify FR-RSK risk management behavior.')
    parser.add_argument('--output', type=Path, default=Path('docs/samples/risk-management-metrics.json'))
    parser.add_argument('--transition-timeout-sec', type=float, default=2.0)
    parser.add_argument('--latency-threshold-ms', type=float, default=500.0)
    parser.add_argument('--velocity-zero-threshold', type=float, default=0.02)
    args = parser.parse_args(argv)

    rclpy.init(args=[])
    node = RiskManagementChecker()
    try:
        if not node.wait_ready(timeout_sec=25.0):
            print('[FAIL] risk status, bridge state, or joint states not ready', file=sys.stderr)
            return 1

        transitions = [
            node.drive_to_level(level, timeout_sec=args.transition_timeout_sec)
            for level in (0, 1, 2, 3)
        ]
        e_stop_ok, e_stop_latency_ms, e_stop_status = node.wait_for_e_stop(timeout_sec=2.0)
        bridge_ok, bridge_latency_ms = node.wait_for_bridge_e_stop(timeout_sec=1.0)
        velocity_ok, velocity_latency_ms, best_velocity_rms = node.wait_for_zero_velocity(
            timeout_sec=0.5,
            threshold=args.velocity_zero_threshold,
        )
        clear_before_ack_ok, clear_before_ack_msg = node.clear_e_stop(timeout_sec=3.0)
        ack_ok, ack_msg = node.acknowledge(timeout_sec=3.0)
        clear_after_ack_ok, clear_after_ack_msg = node.clear_e_stop(timeout_sec=3.0)

        r3 = transitions[-1]
        payload = {
            'timestamp_unix': round(time.time(), 3),
            'criteria': {
                'level_transition_latency_ms_max': args.latency_threshold_ms,
                'velocity_zero_latency_ms_max': 100.0,
                'velocity_zero_threshold_rms': args.velocity_zero_threshold,
                'attribution_dimensions_expected': 5,
            },
            'transitions': [_transition_to_dict(item) for item in transitions],
            'level_latency_passes': all(
                item.reached
                and item.latency_ms is not None
                and item.latency_ms <= args.latency_threshold_ms
                for item in transitions
            ),
            'r3_status': {
                'e_stop_active': bool(e_stop_status.e_stop_active) if e_stop_status else False,
                'degraded_mode': bool(e_stop_status.degraded_mode) if e_stop_status else False,
                'primary_driver': '' if e_stop_status is None else e_stop_status.primary_driver,
                'attribution_count': 0 if e_stop_status is None else len(e_stop_status.attribution),
                'composite_score': None if e_stop_status is None else round(float(e_stop_status.composite_score), 6),
            },
            'e_stop': {
                'risk_status_passes': e_stop_ok,
                'risk_status_latency_ms': e_stop_latency_ms,
                'bridge_system_state_passes': bridge_ok,
                'bridge_system_state_latency_ms': bridge_latency_ms,
                'velocity_zero_passes': velocity_ok and velocity_latency_ms is not None and velocity_latency_ms <= 100.0,
                'velocity_zero_latency_ms': velocity_latency_ms,
                'best_velocity_rms': round(best_velocity_rms, 6),
            },
            'acknowledge_flow': {
                'clear_before_ack_success': clear_before_ack_ok,
                'clear_before_ack_message': clear_before_ack_msg,
                'ack_success': ack_ok,
                'ack_message': ack_msg,
                'clear_after_ack_success': clear_after_ack_ok,
                'clear_after_ack_message': clear_after_ack_msg,
                'passes': (not clear_before_ack_ok) and ack_ok and clear_after_ack_ok,
            },
            'overall_passes': (
                all(item.reached for item in transitions)
                and all(
                    item.latency_ms is not None and item.latency_ms <= args.latency_threshold_ms
                    for item in transitions
                )
                and r3.attribution_count == 5
                and bool(r3.primary_driver)
                and e_stop_ok
                and bridge_ok
                and velocity_ok
                and velocity_latency_ms is not None
                and velocity_latency_ms <= 100.0
                and (not clear_before_ack_ok)
                and ack_ok
                and clear_after_ack_ok
            ),
        }

        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        if not payload['overall_passes']:
            print(f'[FAIL] Risk management criteria failed: {args.output}', file=sys.stderr)
            return 1
        print(f'[PASS] Risk management criteria met: {args.output}')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
