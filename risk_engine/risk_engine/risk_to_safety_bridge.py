"""M4A ROS bridge from RiskStatus to proposed/applied runtime safety actions."""

from __future__ import annotations

from bridge_monitor_msgs.msg import RiskStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from risk_engine.safety_bridge import RiskToSafetyStateMachine
from std_msgs.msg import Bool
from teleop_interfaces.msg import PolicyExecutionReport
from teleop_interfaces.srv import TriggerEstop


class RiskToSafetyBridgeNode(Node):
    """Publish proposed decisions; apply them only when dry_run is false."""

    def __init__(self, **kwargs) -> None:
        super().__init__('risk_to_safety_bridge', **kwargs)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('healthy_recovery_count', 5)
        self.declare_parameter('risk_topic', '/risk/status')
        self.declare_parameter('hold_topic', '/policy/runtime_hold')
        self.declare_parameter('decision_topic', '/policy/safety_decision')
        self.declare_parameter('estop_service', '/safety/trigger_estop')
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._machine = RiskToSafetyStateMachine(
            int(self.get_parameter('healthy_recovery_count').value)
        )
        self._actual_estop = False
        self._actual_hold = False
        self._last_safety_estop = False
        self._estop_request_count = 0
        self._estop_request_sent_for_latch = False
        self._hold_pub = self.create_publisher(
            Bool, str(self.get_parameter('hold_topic').value), 10
        )
        self._decision_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('decision_topic').value),
            10,
        )
        self._estop_client = self.create_client(
            TriggerEstop, str(self.get_parameter('estop_service').value)
        )
        self.create_subscription(
            RiskStatus,
            str(self.get_parameter('risk_topic').value),
            self._on_risk,
            10,
        )
        self.create_subscription(
            Bool, '/safety/estop', self._on_safety_estop, 10
        )
        self.create_subscription(
            PolicyExecutionReport,
            '/policy/execution_report',
            self._on_execution_report,
            10,
        )
        self.get_logger().info(
            f'RiskToSafetyBridge started dry_run={self._dry_run}'
        )

    def _on_safety_estop(self, message: Bool) -> None:
        active = bool(message.data)
        if self._last_safety_estop and not active:
            self._machine.observe_manual_estop_reset()
            self._estop_request_sent_for_latch = False
        self._last_safety_estop = active
        self._actual_estop = active

    def _on_execution_report(self, message: PolicyExecutionReport) -> None:
        self._actual_hold = bool(message.hold_active)
        self._actual_estop = self._actual_estop or bool(message.estop_active)

    def _actual_decision(self) -> str:
        if self._actual_estop:
            return 'E_STOP'
        if self._actual_hold:
            return 'HOLD'
        return 'RUN' if not self._dry_run else 'UNAVAILABLE'

    def _on_risk(self, message: RiskStatus) -> None:
        proposed = self._machine.update(
            level=int(message.level),
            degraded_mode=bool(message.degraded_mode),
            validity=str(message.validity),
            has_valid_sources=bool(message.has_valid_sources),
            e_stop_active=bool(message.e_stop_active),
        )
        applied = False
        if not self._dry_run:
            self._hold_pub.publish(Bool(data=proposed.hold_active))
            applied = True
            if (
                proposed.decision == 'E_STOP'
                and not self._actual_estop
                and not self._estop_request_sent_for_latch
            ):
                if self._estop_client.service_is_ready():
                    request = TriggerEstop.Request()
                    request.reason = (
                        f'risk_to_safety_bridge:{proposed.reason_code}'
                    )
                    self._estop_client.call_async(request)
                    self._estop_request_count += 1
                    self._estop_request_sent_for_latch = True
                else:
                    applied = False
        self._publish_decision(
            proposed=proposed,
            risk_level=int(message.level),
            applied=applied,
        )

    def _publish_decision(self, *, proposed, risk_level: int, applied: bool) -> None:
        actual = self._actual_decision()
        mismatch = actual != 'UNAVAILABLE' and actual != proposed.decision

        def value(key: str, raw) -> KeyValue:
            if isinstance(raw, bool):
                rendered = str(raw).lower()
            else:
                rendered = str(raw)
            return KeyValue(key=key, value=rendered)

        status = DiagnosticStatus(
            level=(DiagnosticStatus.WARN if mismatch else DiagnosticStatus.OK),
            name='policy_runtime/safety_decision',
            message=proposed.reason_code,
            hardware_id='panda_policy_runtime_v1',
            values=[
                value('proposed_decision', proposed.decision),
                value('proposed_reason', proposed.reason_code),
                value('actual_decision', actual),
                value('decision_mismatch', mismatch),
                value('dry_run', self._dry_run),
                value('applied', applied),
                value('risk_level', risk_level),
                value('hold_active', proposed.hold_active),
                value('estop_latched', self._machine.estop_latched),
                value('estop_request_count', self._estop_request_count),
                value('recovery_count', proposed.recovery_count),
            ],
        )
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._decision_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RiskToSafetyBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
