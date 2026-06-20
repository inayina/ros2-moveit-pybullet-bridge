"""ROS 2 node aggregating multi-dimensional risk posture."""

from __future__ import annotations

import json

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from bridge_monitor_msgs.msg import DistributionMetrics, RiskAttribution, RiskStatus
from bridge_monitor_msgs.srv import AcknowledgeRisk

from risk_engine.aggregator import RiskAggregator, RiskWeights
from risk_engine.move_group_cancel import MoveGroupCancelClient
from risk_engine.planning_stats import PlanningStatsCollector


class RiskEngineNode(Node):
    """Subscribe to monitor metrics and publish R0-R3 risk status."""

    def __init__(self) -> None:
        super().__init__('risk_engine')

        self.declare_parameter('weights.distribution_shift', 0.35)
        self.declare_parameter('weights.tracking_error', 0.25)
        self.declare_parameter('weights.dynamics_anomaly', 0.20)
        self.declare_parameter('weights.comm_health', 0.10)
        self.declare_parameter('weights.planning_failure', 0.10)
        self.declare_parameter('level_thresholds', [0.25, 0.50, 0.75])
        self.declare_parameter('tracking_rmse_threshold', 0.05)
        self.declare_parameter('planning_failure_rate_threshold', 0.1)
        self.declare_parameter('planning_stats_window_size', 20)
        self.declare_parameter('auto_e_stop_on_r3', True)
        self.declare_parameter('move_group_action', '/move_action')
        self.declare_parameter('cancel_move_group_on_e_stop', True)

        weights = RiskWeights(
            distribution_shift=self.get_parameter('weights.distribution_shift').value,
            tracking_error=self.get_parameter('weights.tracking_error').value,
            dynamics_anomaly=self.get_parameter('weights.dynamics_anomaly').value,
            comm_health=self.get_parameter('weights.comm_health').value,
            planning_failure=self.get_parameter('weights.planning_failure').value,
        )
        thresholds = tuple(self.get_parameter('level_thresholds').value)
        self._aggregator = RiskAggregator(weights=weights, level_thresholds=thresholds)

        self._latest_metrics: DistributionMetrics | None = None
        self._latest_tracking_rmse = 0.0
        self._e_stop_active = False
        self._acknowledged = True
        self._prev_level = 0
        self._move_cancel = MoveGroupCancelClient(
            self,
            action_name=self.get_parameter('move_group_action').value,
        )
        self._planning_stats = PlanningStatsCollector(
            window_size=int(self.get_parameter('planning_stats_window_size').value),
        )
        self._bridge_system_state = 'RUNNING'

        self.create_subscription(
            DistributionMetrics, '/monitor/distribution_metrics', self._on_metrics, 10)
        self.create_subscription(
            JointState, '/monitor/tracking_error', self._on_tracking_error, qos_profile_sensor_data)
        self.create_subscription(
            String, '/manipulation/planning_result', self._on_planning_result, 10)
        self.create_subscription(
            String, '/bridge/system_state', self._on_bridge_system_state, 10)

        self._status_pub = self.create_publisher(RiskStatus, '/risk/status', 10)
        self._alerts_pub = self.create_publisher(String, '/risk/alerts', 10)
        self._planning_stats_pub = self.create_publisher(
            DiagnosticArray, '/risk/planning_stats', 10)

        self.create_service(AcknowledgeRisk, '/risk/acknowledge', self._handle_acknowledge)
        self.create_service(Trigger, '/risk/force_e_stop', self._handle_force_e_stop)
        self.create_service(Trigger, '/risk/clear_e_stop', self._handle_clear_e_stop)

        self._timer = self.create_timer(0.1, self._publish_risk)
        self._planning_timer = self.create_timer(1.0, self._publish_planning_stats)
        self.get_logger().info('risk_engine node started.')

    def _on_planning_result(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid planning_result JSON: {msg.data[:120]}')
            return
        self._planning_stats.record(
            success=bool(payload.get('success', False)),
            message=str(payload.get('message', '')),
            action=str(payload.get('action', '')),
        )

    def _publish_planning_stats(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._planning_stats_pub.publish(self._planning_stats.to_diagnostic_array(stamp))

    def _trigger_e_stop(self, result, *, reason: str) -> None:
        if self._e_stop_active:
            return
        self._e_stop_active = True
        self._acknowledged = False
        if self.get_parameter('cancel_move_group_on_e_stop').value:
            self._move_cancel.cancel_all()
        self._publish_alert(reason, result)

    def _on_metrics(self, msg: DistributionMetrics) -> None:
        self._latest_metrics = msg

    def _on_tracking_error(self, msg: JointState) -> None:
        if msg.position:
            self._latest_tracking_rmse = float(np.sqrt(np.mean(np.square(msg.position))))

    def _on_bridge_system_state(self, msg: String) -> None:
        self._bridge_system_state = msg.data

    def _compute_raw_scores(self) -> dict[str, float]:
        scores = {
            'distribution_shift': 0.0,
            'tracking_error': 0.0,
            'dynamics_anomaly': 0.0,
            'comm_health': 0.0,
            'planning_failure': 0.0,
        }
        if self._latest_metrics:
            kl_norm = self._latest_metrics.kl_divergence_mean / 0.30
            w1_norm = self._latest_metrics.wasserstein_mean / 0.15
            mmd_norm = self._latest_metrics.mmd_statistic / 0.10
            scores['distribution_shift'] = max(kl_norm, w1_norm, mmd_norm)
            if self._latest_metrics.shift_detected:
                scores['distribution_shift'] = max(scores['distribution_shift'], 0.5)
            scores['comm_health'] = float(self._latest_metrics.comm_health_score)
            if self._bridge_system_state == 'HOLD':
                scores['comm_health'] = max(scores['comm_health'], 0.4)
            dyn_score = float(self._latest_metrics.dynamics_anomaly_score)
            if self._latest_metrics.soft_limit_triggered:
                dyn_score = max(dyn_score, 0.6)
            else:
                dyn_score = max(dyn_score, float(self._latest_metrics.soft_limit_score))
            scores['dynamics_anomaly'] = dyn_score

        rmse_thresh = self.get_parameter('tracking_rmse_threshold').value
        if rmse_thresh > 0:
            scores['tracking_error'] = self._latest_tracking_rmse / rmse_thresh

        pf_thresh = self.get_parameter('planning_failure_rate_threshold').value
        if pf_thresh > 0 and self._planning_stats.sample_count > 0:
            scores['planning_failure'] = min(
                self._planning_stats.failure_rate() / pf_thresh,
                1.0,
            )

        return scores

    def _publish_risk(self) -> None:
        result = self._aggregator.aggregate(self._compute_raw_scores())

        if (
            self.get_parameter('auto_e_stop_on_r3').value
            and result.level >= 3
            and not self._e_stop_active
        ):
            self._trigger_e_stop(result, reason='e_stop_triggered')

        if result.level != self._prev_level:
            self._publish_alert('level_change', result)
            self._prev_level = result.level

        status = RiskStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.level = result.level
        status.composite_score = result.composite_score
        status.primary_driver = result.primary_driver
        status.recommendation = result.recommendation
        status.e_stop_active = self._e_stop_active
        status.degraded_mode = result.level >= 2

        for dim in result.dimensions:
            attr = RiskAttribution()
            attr.dimension = dim.dimension
            attr.raw_score = dim.raw_score
            attr.weight = dim.weight
            attr.weighted_score = dim.weighted_score
            attr.is_primary_driver = dim.dimension == result.primary_driver
            status.attribution.append(attr)

        self._status_pub.publish(status)

    def _publish_alert(self, event_type: str, result) -> None:
        alert = {
            'event_type': event_type,
            'from_level': self._prev_level,
            'to_level': result.level,
            'primary_driver': result.primary_driver,
            'message': result.recommendation,
        }
        msg = String()
        msg.data = json.dumps(alert)
        self._alerts_pub.publish(msg)

    def _handle_acknowledge(self, request, response):
        if not self._e_stop_active:
            response.success = False
            response.message = 'No active e-stop to acknowledge.'
            return response
        self._acknowledged = True
        response.success = True
        response.message = f'Acknowledged by {request.operator_id}.'
        return response

    def _handle_force_e_stop(self, request, response):
        result = self._aggregator.aggregate(self._compute_raw_scores())
        self._trigger_e_stop(result, reason='e_stop_forced')
        response.success = True
        response.message = 'E-stop activated.'
        return response

    def _handle_clear_e_stop(self, request, response):
        if not self._acknowledged:
            response.success = False
            response.message = 'Must acknowledge before clearing e-stop.'
            return response
        self._e_stop_active = False
        response.success = True
        response.message = 'E-stop cleared.'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RiskEngineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
