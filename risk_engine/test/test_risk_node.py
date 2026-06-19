"""Node-level tests for risk_engine."""

from __future__ import annotations

import threading
import time

import pytest
import rclpy
from bridge_monitor_msgs.msg import DistributionMetrics, RiskStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from risk_engine.risk_node import RiskEngineNode


class _MonitorFeed(Node):
    def __init__(self) -> None:
        super().__init__('test_monitor_feed')
        self.metrics_pub = self.create_publisher(
            DistributionMetrics, '/monitor/distribution_metrics', 10)
        self.error_pub = self.create_publisher(
            JointState, '/monitor/tracking_error', qos_profile_sensor_data)

    def publish_shifted_metrics(self) -> None:
        metrics = DistributionMetrics()
        metrics.header.stamp = self.get_clock().now().to_msg()
        metrics.kl_divergence_mean = 0.25
        metrics.mmd_statistic = 0.08
        metrics.mmd_p_value = 0.01
        metrics.shift_detected = True
        self.metrics_pub.publish(metrics)

        err = JointState()
        err.header = metrics.header
        err.position = [0.12, 0.08]
        self.error_pub.publish(err)


@pytest.fixture(scope='module')
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def risk_node(ros_context):
    risk = RiskEngineNode()
    feed = _MonitorFeed()
    executor = SingleThreadedExecutor()
    executor.add_node(risk)
    executor.add_node(feed)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield risk, feed
    executor.shutdown()
    thread.join(timeout=2.0)
    feed.destroy_node()
    risk.destroy_node()


def test_risk_node_publishes_status(risk_node):
    risk, feed = risk_node
    statuses: list[RiskStatus] = []

    def _on_status(msg: RiskStatus) -> None:
        statuses.append(msg)

    risk.create_subscription(RiskStatus, '/risk/status', _on_status, 10)

    deadline = time.time() + 2.0
    while time.time() < deadline and feed.metrics_pub.get_subscription_count() == 0:
        time.sleep(0.05)

    feed.publish_shifted_metrics()

    deadline = time.time() + 3.0
    while time.time() < deadline and len(statuses) < 1:
        time.sleep(0.1)

    assert statuses, 'expected /risk/status'
    latest = statuses[-1]
    assert latest.level >= 0
    assert latest.composite_score > 0.0
    assert latest.primary_driver
    assert len(latest.attribution) == 5
