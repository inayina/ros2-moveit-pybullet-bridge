"""ROS 2 node aggregating multi-dimensional risk posture."""

from __future__ import annotations

import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from bridge_monitor_msgs.msg import DistributionMetrics, RiskAttribution, RiskStatus
from bridge_monitor_msgs.srv import AcknowledgeRisk

from risk_engine.aggregator import RiskAggregator, RiskWeights


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
        self.declare_parameter('auto_e_stop_on_r3', True)

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

        self.create_subscription(
            DistributionMetrics, '/monitor/distribution_metrics', self._on_metrics, 10)
        self.create_subscription(
            JointState, '/monitor/tracking_error', self._on_tracking_error, 10)

        self._status_pub = self.create_publisher(RiskStatus, '/risk/status', 10)
        self._alerts_pub = self.create_publisher(String, '/risk/alerts', 10)

        self.create_service(AcknowledgeRisk, '/risk/acknowledge', self._handle_acknowledge)
        self.create_service(Trigger, '/risk/force_e_stop', self._handle_force_e_stop)
        self.create_service(Trigger, '/risk/clear_e_stop', self._handle_clear_e_stop)

        self._timer = self.create_timer(0.1, self._publish_risk)
        self.get_logger().info('risk_engine node started (scaffold).')

    def _on_metrics(self, msg: DistributionMetrics) -> None:
        self._latest_metrics = msg

    def _on_tracking_error(self, msg: JointState) -> None:
        if msg.position:
            self._latest_tracking_rmse = float(np.sqrt(np.mean(np.square(msg.position))))

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
            mmd_norm = self._latest_metrics.mmd_statistic / 0.10
            scores['distribution_shift'] = max(kl_norm, mmd_norm)
            if self._latest_metrics.shift_detected:
                scores['distribution_shift'] = max(scores['distribution_shift'], 0.5)

        rmse_thresh = self.get_parameter('tracking_rmse_threshold').value
        if rmse_thresh > 0:
            scores['tracking_error'] = self._latest_tracking_rmse / rmse_thresh

        return scores

    def _publish_risk(self) -> None:
        result = self._aggregator.aggregate(self._compute_raw_scores())

        if (
            self.get_parameter('auto_e_stop_on_r3').value
            and result.level >= 3
            and not self._e_stop_active
        ):
            self._e_stop_active = True
            self._acknowledged = False
            self._publish_alert('e_stop_triggered', result)

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
        self._e_stop_active = True
        self._acknowledged = False
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
