"""Node tests for PolicyRunner command path, health, and fault injection."""

import os
import time

import pytest
import rclpy
from bridge_monitor_msgs.msg import DistributionMetrics
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from pybullet_bridge.learning.policy_runner import PolicyRunner


class _JointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__('test_policy_runner_joint_state_pub')
        self.pub = self.create_publisher(
            JointState,
            '/bridge/sim/joint_states',
            qos_profile_sensor_data,
        )

    def publish_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2']
        msg.position = [0.2, -0.1]
        msg.velocity = [0.0, 0.0]
        self.pub.publish(msg)


class _CommandObserver(Node):
    def __init__(self) -> None:
        super().__init__('test_policy_runner_command_observer')
        self.commands: list[JointTrajectory] = []
        self.create_subscription(
            JointTrajectory,
            '/bridge/command',
            self._on_command,
            10,
        )

    def _on_command(self, msg: JointTrajectory) -> None:
        self.commands.append(msg)


class _HealthObserver(Node):
    def __init__(self) -> None:
        super().__init__('test_policy_runner_health_observer')
        self.health_msgs: list[DiagnosticArray] = []
        self.create_subscription(
            DiagnosticArray,
            '/system_health',
            self._on_health,
            10,
        )

    def _on_health(self, msg: DiagnosticArray) -> None:
        self.health_msgs.append(msg)


class _MetricsPublisher(Node):
    def __init__(self) -> None:
        super().__init__('test_policy_runner_metrics_pub')
        self.pub = self.create_publisher(
            DistributionMetrics,
            '/monitor/distribution_metrics',
            10,
        )

    def publish_metrics(self) -> None:
        msg = DistributionMetrics()
        msg.kl_divergence_mean = 0.012
        msg.wasserstein_mean = 0.004
        msg.mmd_statistic = 0.031
        msg.shift_detected = False
        self.pub.publish(msg)


def _init_rclpy(log_dir, extra_args: list[str] | None = None) -> None:
    if rclpy.ok():
        rclpy.shutdown()
    os.environ['ROS_LOG_DIR'] = str(log_dir)
    args = [
        '--ros-args',
        '-p', 'strategy_type:=sine_wave',
        '-p', 'policy_inference_freq:=20',
        '-p', 'sine_amplitude:=0.0',
    ]
    if extra_args:
        args.extend(extra_args)
    rclpy.init(args=args)


def test_policy_runner_publishes_joint_trajectory_from_joint_state(tmp_path):
    _init_rclpy(tmp_path / 'ros-log')
    runner = PolicyRunner()
    publisher = _JointStatePublisher()
    observer = _CommandObserver()
    executor = SingleThreadedExecutor()
    executor.add_node(runner)
    executor.add_node(publisher)
    executor.add_node(observer)

    try:
        deadline = time.time() + 3.0
        while time.time() < deadline and not observer.commands:
            publisher.publish_state()
            executor.spin_once(timeout_sec=0.05)

        assert observer.commands, 'expected PolicyRunner to publish /bridge/command'
        command = observer.commands[-1]
        assert command.joint_names == ['joint1', 'joint2']
        assert len(command.points) == 1
        assert command.points[0].positions == pytest.approx([0.2, -0.1])
        assert command.points[0].time_from_start.nanosec == 50_000_000
    finally:
        executor.shutdown()
        observer.destroy_node()
        publisher.destroy_node()
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_policy_runner_subscribes_metrics_and_publishes_health(tmp_path):
    _init_rclpy(tmp_path / 'ros-log')
    runner = PolicyRunner()
    publisher = _JointStatePublisher()
    metrics_pub = _MetricsPublisher()
    health_observer = _HealthObserver()
    executor = SingleThreadedExecutor()
    executor.add_node(runner)
    executor.add_node(publisher)
    executor.add_node(metrics_pub)
    executor.add_node(health_observer)

    try:
        deadline = time.time() + 3.0
        while time.time() < deadline and not health_observer.health_msgs:
            publisher.publish_state()
            metrics_pub.publish_metrics()
            executor.spin_once(timeout_sec=0.05)

        assert health_observer.health_msgs
        status = health_observer.health_msgs[-1].status[0]
        assert status.name == 'policy_runner.health'
        assert status.values

        deadline = time.time() + 2.0
        while time.time() < deadline and runner._latest_metrics is None:  # noqa: SLF001
            metrics_pub.publish_metrics()
            executor.spin_once(timeout_sec=0.05)
        assert runner._latest_metrics is not None  # noqa: SLF001
        assert runner._latest_metrics.kl_divergence_mean == pytest.approx(0.012)  # noqa: SLF001
    finally:
        executor.shutdown()
        health_observer.destroy_node()
        metrics_pub.destroy_node()
        publisher.destroy_node()
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_policy_runner_fault_injection_reports_stalled_health(tmp_path):
    _init_rclpy(
        tmp_path / 'ros-log',
        [
            '-p', 'fault_injection_enabled:=true',
            '-p', 'fault_sleep_probability:=1.0',
            '-p', 'fault_sleep_sec:=0.1',
            '-p', 'watchdog_timeout_sec:=0.05',
        ],
    )
    runner = PolicyRunner()
    publisher = _JointStatePublisher()
    health_observer = _HealthObserver()
    executor = SingleThreadedExecutor()
    executor.add_node(runner)
    executor.add_node(publisher)
    executor.add_node(health_observer)

    try:
        deadline = time.time() + 3.0
        while time.time() < deadline:
            publisher.publish_state()
            executor.spin_once(timeout_sec=0.05)
            if health_observer.health_msgs:
                levels = [msg.status[0].level for msg in health_observer.health_msgs]
                reasons = [
                    next(
                        (kv.value for kv in msg.status[0].values if kv.key == 'reason'),
                        '',
                    )
                    for msg in health_observer.health_msgs
                ]
                if any(level >= DiagnosticStatus.WARN for level in levels):
                    assert 'stalled' in reasons
                    break
        else:
            pytest.fail('expected WARN/ERROR health after fault injection')
    finally:
        executor.shutdown()
        health_observer.destroy_node()
        publisher.destroy_node()
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_policy_runner_deactivate_publishes_inactive_health(tmp_path):
    _init_rclpy(tmp_path / 'ros-log')
    runner = PolicyRunner()
    health_observer = _HealthObserver()
    executor = SingleThreadedExecutor()
    executor.add_node(runner)
    executor.add_node(health_observer)

    try:
        runner.on_deactivate()
        deadline = time.time() + 2.0
        while time.time() < deadline and not health_observer.health_msgs:
            executor.spin_once(timeout_sec=0.05)

        assert health_observer.health_msgs
        status = health_observer.health_msgs[-1].status[0]
        reason = next(kv.value for kv in status.values if kv.key == 'reason')
        assert reason == 'inactive'
        assert runner.health_reason == 'inactive'
    finally:
        executor.shutdown()
        health_observer.destroy_node()
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
