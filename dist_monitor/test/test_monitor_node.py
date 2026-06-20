"""Node-level tests for dist_monitor."""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest
import rclpy
from bridge_monitor_msgs.msg import DistributionMetrics
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from dist_monitor.lerobot_loader import LeRobotTrajectory
from dist_monitor.monitor_node import DistMonitorNode


class _JointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__('test_joint_state_publisher')
        self.pub_sim = self.create_publisher(JointState, '/bridge/sim/joint_states', 10)
        self.pub_real = self.create_publisher(JointState, '/bridge/real/joint_states', 10)

    def publish_sequence(
        self,
        count: int,
        sim_offset: float = 0.0,
        real_offset: float = 0.0,
    ) -> None:
        stamp_base = self.get_clock().now()
        for i in range(count):
            t = stamp_base + rclpy.duration.Duration(seconds=i * 0.05)
            phase = i * 0.1
            sim = JointState()
            sim.header.stamp = t.to_msg()
            sim.name = ['joint1', 'joint2']
            sim.position = [
                float(np.sin(phase) + sim_offset),
                float(np.cos(phase) + sim_offset),
            ]
            real = JointState()
            real.header.stamp = t.to_msg()
            real.name = ['joint1', 'joint2']
            real.position = [
                float(np.sin(phase) + real_offset),
                float(np.cos(phase) + real_offset),
            ]
            self.pub_sim.publish(sim)
            self.pub_real.publish(real)


@pytest.fixture(scope='module')
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def monitor_node(ros_context):
    monitor = DistMonitorNode()
    monitor.set_parameters([
        rclpy.parameter.Parameter('min_samples', rclpy.Parameter.Type.INTEGER, 10),
        rclpy.parameter.Parameter('update_frequency_hz', rclpy.Parameter.Type.DOUBLE, 20.0),
        rclpy.parameter.Parameter('mmd_permutation_count', rclpy.Parameter.Type.INTEGER, 20),
        rclpy.parameter.Parameter('baseline_duration_sec', rclpy.Parameter.Type.DOUBLE, 0.0),
        rclpy.parameter.Parameter('mmd_gamma', rclpy.Parameter.Type.DOUBLE, 0.0),
    ])
    publisher = _JointStatePublisher()
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(publisher)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield monitor, publisher
    executor.shutdown()
    thread.join(timeout=2.0)
    publisher.destroy_node()
    monitor.destroy_node()


def test_monitor_node_publishes_metrics(monitor_node):
    monitor, publisher = monitor_node
    metrics: list[DistributionMetrics] = []

    def _on_metrics(msg: DistributionMetrics) -> None:
        metrics.append(msg)

    monitor.create_subscription(
        DistributionMetrics, '/monitor/distribution_metrics', _on_metrics, 10)

    deadline = time.time() + 2.0
    while time.time() < deadline and publisher.pub_sim.get_subscription_count() == 0:
        time.sleep(0.05)

    publisher.publish_sequence(count=12)

    deadline = time.time() + 5.0
    while time.time() < deadline and len(metrics) < 2:
        time.sleep(0.1)

    assert len(metrics) >= 1
    latest = metrics[-1]
    assert latest.joint_names == ['joint1', 'joint2']
    assert latest.sample_count_sim >= 10
    assert latest.sample_count_real >= 10
    assert len(latest.sim_position_median_per_joint) == 2
    assert len(latest.real_position_median_per_joint) == 2
    assert latest.sim_position_max_per_joint[0] >= latest.sim_position_min_per_joint[0]


def test_monitor_node_detects_shift(monitor_node):
    monitor, publisher = monitor_node
    metrics: list[DistributionMetrics] = []

    def _on_metrics(msg: DistributionMetrics) -> None:
        metrics.append(msg)

    monitor.create_subscription(
        DistributionMetrics, '/monitor/distribution_metrics', _on_metrics, 10)

    deadline = time.time() + 2.0
    while time.time() < deadline and publisher.pub_sim.get_subscription_count() == 0:
        time.sleep(0.05)

    publisher.publish_sequence(count=12, sim_offset=0.0, real_offset=0.8)

    deadline = time.time() + 6.0
    while time.time() < deadline:
        time.sleep(0.1)
        if metrics and metrics[-1].sample_count_sim >= 10:
            if metrics[-1].mmd_statistic > 0.0 or metrics[-1].kl_divergence_mean > 0.0:
                break

    assert metrics, 'expected distribution metrics'
    latest = metrics[-1]
    assert latest.sample_count_sim >= 10
    assert latest.mmd_statistic > 0.0 or latest.kl_divergence_mean > 0.0


def test_monitor_node_threshold_hot_reload(monitor_node):
    monitor, _publisher = monitor_node
    result = monitor.set_parameters([
        rclpy.parameter.Parameter('kl_threshold_mean', rclpy.Parameter.Type.DOUBLE, 0.99),
        rclpy.parameter.Parameter('w1_threshold_mean', rclpy.Parameter.Type.DOUBLE, 0.50),
        rclpy.parameter.Parameter('mmd_threshold', rclpy.Parameter.Type.DOUBLE, 0.20),
    ])
    assert all(r.successful for r in result)
    assert monitor.get_parameter('kl_threshold_mean').value == pytest.approx(0.99)
    assert monitor.get_parameter('w1_threshold_mean').value == pytest.approx(0.50)
    assert monitor.get_parameter('mmd_threshold').value == pytest.approx(0.20)
    cfg = monitor._metrics_config()
    assert cfg.kl_threshold_mean == pytest.approx(0.99)
    assert cfg.w1_threshold_mean == pytest.approx(0.50)
    assert cfg.mmd_threshold == pytest.approx(0.20)


def test_lerobot_source_uses_first_sim_stamp_as_time_origin(monitor_node):
    monitor, _publisher = monitor_node
    monitor.set_parameters([
        rclpy.parameter.Parameter('real_source', rclpy.Parameter.Type.STRING, 'lerobot'),
    ])
    monitor._joint_names = ['joint1', 'joint2']
    monitor._lerobot_traj = LeRobotTrajectory(
        timestamps=np.array([0.0, 0.05, 0.10], dtype=float),
        positions=np.array([
            [1.0, 2.0],
            [1.1, 2.1],
            [1.2, 2.2],
        ], dtype=float),
        velocities=np.zeros((3, 2), dtype=float),
        joint_names=['joint1', 'joint2'],
        fps=20.0,
    )
    monitor._lerobot_origin = None

    for idx in range(3):
        msg = JointState()
        stamp = monitor.get_clock().now() + rclpy.duration.Duration(seconds=100.0 + idx * 0.05)
        msg.header.stamp = stamp.to_msg()
        msg.name = ['joint1', 'joint2']
        msg.position = [0.0, 0.0]
        monitor._on_sim(msg)

    assert monitor._real_window.count() == 3
