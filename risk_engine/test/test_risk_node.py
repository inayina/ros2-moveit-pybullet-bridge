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
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from risk_engine.risk_node import RiskEngineNode


class _MonitorFeed(Node):
    def __init__(self) -> None:
        super().__init__('test_monitor_feed')
        self.metrics_pub = self.create_publisher(
            DistributionMetrics, '/monitor/distribution_metrics', 10)
        self.error_pub = self.create_publisher(
            JointState, '/monitor/tracking_error', qos_profile_sensor_data)
        from std_msgs.msg import String
        self.planning_pub = self.create_publisher(
            String, '/manipulation/planning_result', 10)
        self.telemetry_pub = self.create_publisher(
            DiagnosticArray, '/system/telemetry', 10)

    def publish_shifted_metrics(self) -> None:
        metrics = DistributionMetrics()
        metrics.header.stamp = self.get_clock().now().to_msg()
        metrics.kl_divergence_mean = 0.25
        metrics.mmd_statistic = 0.08
        metrics.mmd_p_value = 0.01
        metrics.shift_detected = True
        metrics.comm_health_score = 0.85
        metrics.dynamics_anomaly_score = 0.55
        metrics.soft_limit_triggered = False
        self.metrics_pub.publish(metrics)

        err = JointState()
        err.header = metrics.header
        err.position = [0.12, 0.08]
        self.error_pub.publish(err)

    def publish_planning_failure(self) -> None:
        from std_msgs.msg import String
        msg = String()
        msg.data = '{"action":"pick","success":false,"message":"canceled"}'
        self.planning_pub.publish(msg)

    def publish_critical_cpu(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'system_telemetry/host'
        status.values = [
            KeyValue(key='cpu_total_percent', value='100.0'),
            KeyValue(key='memory_percent', value='50.0'),
        ]
        msg.status = [status]
        self.telemetry_pub.publish(msg)


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
    feed.publish_planning_failure()
    feed.publish_planning_failure()

    deadline = time.time() + 3.0
    while time.time() < deadline and len(statuses) < 1:
        time.sleep(0.1)

    assert statuses, 'expected /risk/status'
    latest = statuses[-1]
    assert latest.level >= 0
    assert latest.composite_score > 0.0
    assert latest.primary_driver
    assert len(latest.attribution) == 6
    comm_attr = next(a for a in latest.attribution if a.dimension == 'comm_health')
    assert comm_attr.raw_score > 0.5
    dyn_attr = next(a for a in latest.attribution if a.dimension == 'dynamics_anomaly')
    assert dyn_attr.raw_score > 0.5
    plan_attr = next(a for a in latest.attribution if a.dimension == 'planning_failure')
    assert plan_attr.raw_score >= 1.0


def test_resource_pressure_caps_at_r2_without_auto_estop(risk_node):
    risk, feed = risk_node
    statuses: list[RiskStatus] = []
    risk.create_subscription(RiskStatus, '/risk/status', statuses.append, 10)
    feed.publish_critical_cpu()

    deadline = time.time() + 2.0
    while time.time() < deadline and not any(
        any(attr.dimension == 'resource_pressure' and attr.raw_score >= 1.0
            for attr in status.attribution)
        for status in statuses
    ):
        time.sleep(0.05)

    resource_status = next(
        status for status in reversed(statuses)
        if any(attr.dimension == 'resource_pressure' and attr.raw_score >= 1.0
               for attr in status.attribution)
    )
    assert resource_status.level == 2
    assert resource_status.primary_driver == 'resource_pressure'
    assert resource_status.e_stop_active is False
