"""Node-level tests for hoc_console server."""

from __future__ import annotations

import threading
import time

import pytest
import rclpy
from bridge_monitor_msgs.msg import RiskStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from hoc_console.hoc_server import HocServerNode


class _RiskPublisher(Node):
    def __init__(self) -> None:
        super().__init__('test_risk_publisher')
        self.pub = self.create_publisher(RiskStatus, '/risk/status', 10)

    def publish_sample(self) -> None:
        msg = RiskStatus()
        msg.level = 1
        msg.composite_score = 0.3
        msg.primary_driver = 'tracking_error'
        msg.recommendation = 'check gains'
        self.pub.publish(msg)


@pytest.fixture(scope='module')
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def hoc_node(ros_context):
    server = HocServerNode()
    publisher = _RiskPublisher()
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(publisher)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield server, publisher
    executor.shutdown()
    thread.join(timeout=2.0)
    publisher.destroy_node()
    server.destroy_node()


def test_hoc_node_accepts_risk_status(hoc_node):
    server, publisher = hoc_node

    deadline = time.time() + 2.0
    while time.time() < deadline and publisher.pub.get_subscription_count() == 0:
        time.sleep(0.05)

    publisher.publish_sample()

    deadline = time.time() + 2.0
    while time.time() < deadline and server._latest_risk is None:  # noqa: SLF001
        time.sleep(0.1)

    assert server._latest_risk is not None  # noqa: SLF001
    assert server._latest_risk.level == 1
