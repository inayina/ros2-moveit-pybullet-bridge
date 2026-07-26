"""ROS-level M4A dry-run and one-shot E-stop integration tests."""

from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bridge_monitor_msgs.msg import RiskStatus  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from risk_engine.risk_to_safety_bridge import (  # noqa: E402
    RiskToSafetyBridgeNode,
)
from std_msgs.msg import Bool  # noqa: E402
from teleop_interfaces.srv import TriggerEstop  # noqa: E402


def _risk(level: int) -> RiskStatus:
    message = RiskStatus()
    message.level = level
    message.validity = 'VALID'
    message.has_valid_sources = True
    return message


def _spin_until(executor, predicate, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


def test_dry_run_publishes_proposal_without_hold(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'log'))
    rclpy.init()
    bridge = RiskToSafetyBridgeNode(parameter_overrides=[
        Parameter('risk_topic', value='/m4_test/dry/risk'),
        Parameter('hold_topic', value='/m4_test/dry/hold'),
        Parameter('decision_topic', value='/m4_test/dry/decision'),
        Parameter('estop_service', value='/m4_test/dry/estop'),
    ])
    peer = rclpy.create_node('m4_dry_run_peer')
    proposals = []
    holds = []
    peer.create_subscription(
        DiagnosticArray, '/m4_test/dry/decision', proposals.append, 10
    )
    peer.create_subscription(Bool, '/m4_test/dry/hold', holds.append, 10)
    publisher = peer.create_publisher(RiskStatus, '/m4_test/dry/risk', 10)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(bridge)
    executor.add_node(peer)
    try:
        publisher.publish(_risk(2))
        assert _spin_until(executor, lambda: bool(proposals))
        fields = {
            item.key: item.value for item in proposals[-1].status[0].values
        }
        assert fields['proposed_decision'] == 'HOLD'
        assert fields['dry_run'] == 'true'
        assert holds == []
    finally:
        executor.shutdown()
        bridge.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()


def test_live_r3_requests_estop_once_per_latch(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'log'))
    rclpy.init()
    bridge = RiskToSafetyBridgeNode(parameter_overrides=[
        Parameter('dry_run', value=False),
        Parameter('risk_topic', value='/m4_test/live/risk'),
        Parameter('hold_topic', value='/m4_test/live/hold'),
        Parameter('decision_topic', value='/m4_test/live/decision'),
        Parameter('estop_service', value='/m4_test/live/estop'),
    ])
    peer = rclpy.create_node('m4_live_peer')
    requests = []

    def on_estop(request, response):
        requests.append(request.reason)
        response.success = True
        response.message = 'accepted'
        return response

    peer.create_service(TriggerEstop, '/m4_test/live/estop', on_estop)
    publisher = peer.create_publisher(RiskStatus, '/m4_test/live/risk', 10)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(bridge)
    executor.add_node(peer)
    try:
        assert _spin_until(executor, bridge._estop_client.service_is_ready)
        publisher.publish(_risk(3))
        assert _spin_until(executor, lambda: len(requests) == 1)
        publisher.publish(_risk(3))
        _spin_until(executor, lambda: False, timeout=0.2)
        assert len(requests) == 1
    finally:
        executor.shutdown()
        bridge.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()
