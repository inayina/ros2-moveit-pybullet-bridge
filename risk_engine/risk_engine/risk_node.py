"""ROS 2 node aggregating multi-dimensional risk posture."""

from __future__ import annotations

import json
import time

from bridge_monitor_msgs.msg import (
    DistributionMetrics,
    RiskAttribution,
    RiskStatus,
)
from bridge_monitor_msgs.srv import AcknowledgeRisk
from diagnostic_msgs.msg import DiagnosticArray
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from risk_engine.aggregator import RECOMMENDATIONS, RiskAggregator, RiskWeights
from risk_engine.move_group_cancel import MoveGroupCancelClient
from risk_engine.planning_stats import PlanningStatsCollector
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


class RiskEngineNode(Node):
    """Subscribe to monitor metrics and publish R0-R3 risk status."""

    def __init__(self) -> None:
        super().__init__('risk_engine')

        self.declare_parameter('weights.distribution_shift', 0.30)
        self.declare_parameter('weights.tracking_error', 0.25)
        self.declare_parameter('weights.dynamics_anomaly', 0.20)
        self.declare_parameter('weights.comm_health', 0.10)
        self.declare_parameter('weights.planning_failure', 0.05)
        self.declare_parameter('weights.resource_pressure', 0.10)
        self.declare_parameter('resource_cpu_threshold_percent', 85.0)
        self.declare_parameter('resource_effective_hz_min', 8.0)
        self.declare_parameter('resource_scene_age_threshold_s', 0.5)
        self.declare_parameter('level_thresholds', [0.25, 0.50, 0.75])
        self.declare_parameter('tracking_rmse_threshold', 0.05)
        self.declare_parameter('planning_failure_rate_threshold', 0.1)
        self.declare_parameter('planning_stats_window_size', 20)
        self.declare_parameter('auto_e_stop_on_r3', True)
        self.declare_parameter('move_group_action', '/move_action')
        self.declare_parameter('cancel_move_group_on_e_stop', True)
        self.declare_parameter('source_stale_after_sec', 1.0)

        weights = RiskWeights(
            distribution_shift=self.get_parameter('weights.distribution_shift').value,
            tracking_error=self.get_parameter('weights.tracking_error').value,
            dynamics_anomaly=self.get_parameter('weights.dynamics_anomaly').value,
            comm_health=self.get_parameter('weights.comm_health').value,
            planning_failure=self.get_parameter('weights.planning_failure').value,
            resource_pressure=self.get_parameter('weights.resource_pressure').value,
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
        self._host_cpu_percent = 0.0
        self._host_memory_percent = 0.0
        self._recorder_effective_hz = 0.0
        self._scene_age_s = 0.0
        self._recorder_recording = False
        self._received_at: dict[str, float] = {}
        self._policy_health_validity = 'UNAVAILABLE'
        self._policy_health_reason = 'no_data'
        self._policy_health_score = 0.0

        self.create_subscription(
            DistributionMetrics,
            '/monitor/distribution_metrics',
            self._on_metrics,
            10,
        )
        self.create_subscription(
            JointState,
            '/monitor/tracking_error',
            self._on_tracking_error,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, '/manipulation/planning_result', self._on_planning_result, 10)
        self.create_subscription(
            String, '/bridge/system_state', self._on_bridge_system_state, 10)
        self.create_subscription(
            DiagnosticArray, '/system/telemetry', self._on_system_telemetry, 10)
        self.create_subscription(
            DiagnosticArray, '/recorder/diagnostics', self._on_recorder_diagnostics, 10)
        self.create_subscription(
            DiagnosticArray,
            '/policy/runtime_health',
            self._on_policy_runtime_health,
            10,
        )

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
        self._received_at['metrics'] = time.monotonic()

    def _on_tracking_error(self, msg: JointState) -> None:
        if msg.position:
            self._latest_tracking_rmse = float(np.sqrt(np.mean(np.square(msg.position))))
            self._received_at['tracking'] = time.monotonic()

    def _on_bridge_system_state(self, msg: String) -> None:
        self._bridge_system_state = msg.data

    @staticmethod
    def _diagnostic_values(status) -> dict[str, str]:
        return {value.key: value.value for value in status.values}

    def _on_system_telemetry(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != 'system_telemetry/host':
                continue
            values = self._diagnostic_values(status)
            try:
                self._host_cpu_percent = float(values.get('cpu_total_percent', 0.0))
                self._host_memory_percent = float(values.get('memory_percent', 0.0))
            except ValueError:
                return
            self._received_at['resource'] = time.monotonic()

    def _on_recorder_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != 'lerobot_recorder/health':
                continue
            values = self._diagnostic_values(status)
            try:
                self._recorder_effective_hz = float(values.get('effective_hz', 0.0))
                self._scene_age_s = float(values.get('scene_age_s', 0.0))
                self._recorder_recording = (
                    values.get('recording', 'False').lower() == 'true')
            except ValueError:
                return
            self._received_at['resource'] = time.monotonic()

    def _on_policy_runtime_health(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != 'policy_runtime/brain':
                continue
            values = self._diagnostic_values(status)
            self._policy_health_validity = values.get(
                'validity', 'UNAVAILABLE'
            )
            self._policy_health_reason = values.get('reason_code', 'no_data')
            self._policy_health_score = {
                'VALID': 0.0,
                'WARMING_UP': 0.2,
                'STALE': 0.8,
                'UNAVAILABLE': 0.6,
                'ERROR': 1.0,
            }.get(self._policy_health_validity, 1.0)
            self._received_at['policy_health'] = time.monotonic()

    def _source_fresh(self, key: str) -> bool:
        received = self._received_at.get(key)
        if received is None:
            return False
        max_age = float(self.get_parameter('source_stale_after_sec').value)
        return time.monotonic() - received <= max_age

    def _source_status(self) -> dict[str, dict[str, object]]:
        metrics = self._latest_metrics
        metrics_fresh = self._source_fresh('metrics')

        def state(
            valid: bool,
            provenance: str,
            reason: str = 'none',
            validity: str = 'VALID',
        ) -> dict[str, object]:
            rendered_validity = validity
            if not valid and reason == 'source_stale':
                rendered_validity = 'STALE'
            elif not valid and validity == 'VALID':
                rendered_validity = 'UNAVAILABLE'
            return {
                'valid': valid,
                'validity': rendered_validity,
                'reason_code': reason if not valid else 'none',
                'provenance': provenance,
            }

        distribution_valid = bool(
            metrics_fresh and metrics and metrics.metric_valid
        )
        distribution_reason = 'no_data'
        distribution_validity = 'UNAVAILABLE'
        if metrics:
            distribution_reason = (
                metrics.reason_code if metrics_fresh else 'source_stale'
            )
            distribution_validity = metrics.validity

        comm_from_metrics = bool(
            metrics_fresh and metrics and metrics.comm_health_valid
        )
        policy_fresh = self._source_fresh('policy_health')
        comm_valid = comm_from_metrics or policy_fresh
        comm_reason = 'none' if comm_valid else 'no_comm_health_source'
        if not comm_valid and (
            'metrics' in self._received_at
            or 'policy_health' in self._received_at
        ):
            comm_reason = 'source_stale'

        return {
            'distribution_shift': state(
                distribution_valid,
                '/monitor/distribution_metrics',
                distribution_reason,
                distribution_validity,
            ),
            'tracking_error': state(
                self._source_fresh('tracking'),
                '/monitor/tracking_error',
                'source_stale' if 'tracking' in self._received_at else 'no_data',
            ),
            'dynamics_anomaly': state(
                bool(metrics_fresh and metrics and metrics.dynamics_valid),
                '/monitor/distribution_metrics:dynamics',
                (
                    'source_stale'
                    if metrics and not metrics_fresh
                    else 'insufficient_aligned_samples'
                ),
                'WARMING_UP',
            ),
            'comm_health': state(
                comm_valid,
                '/monitor/comm_health+/policy/runtime_health',
                comm_reason,
            ),
            'planning_failure': state(
                self._planning_stats.sample_count > 0,
                '/manipulation/planning_result',
                'no_samples',
            ),
            'resource_pressure': state(
                self._source_fresh('resource'),
                '/system/telemetry+/recorder/diagnostics',
                'source_stale' if 'resource' in self._received_at else 'no_data',
            ),
        }

    def _compute_raw_scores(self) -> dict[str, float]:
        scores = {
            'distribution_shift': 0.0,
            'tracking_error': 0.0,
            'dynamics_anomaly': 0.0,
            'comm_health': 0.0,
            'planning_failure': 0.0,
            'resource_pressure': 0.0,
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

        if self._source_fresh('policy_health'):
            scores['comm_health'] = max(
                scores['comm_health'], self._policy_health_score
            )

        rmse_thresh = self.get_parameter('tracking_rmse_threshold').value
        if rmse_thresh > 0:
            scores['tracking_error'] = self._latest_tracking_rmse / rmse_thresh

        pf_thresh = self.get_parameter('planning_failure_rate_threshold').value
        if pf_thresh > 0 and self._planning_stats.sample_count > 0:
            scores['planning_failure'] = min(
                self._planning_stats.failure_rate() / pf_thresh,
                1.0,
            )

        cpu_threshold = float(
            self.get_parameter('resource_cpu_threshold_percent').value)
        hz_min = float(self.get_parameter('resource_effective_hz_min').value)
        age_threshold = float(
            self.get_parameter('resource_scene_age_threshold_s').value)
        cpu_score = (
            max(0.0, (self._host_cpu_percent - cpu_threshold) / max(100.0 - cpu_threshold, 1.0))
            if cpu_threshold > 0.0 else 0.0
        )
        memory_score = max(0.0, (self._host_memory_percent - 85.0) / 15.0)
        capture_score = 0.0
        if self._recorder_recording:
            if hz_min > 0.0:
                capture_score = max(
                    capture_score,
                    max(0.0, (hz_min - self._recorder_effective_hz) / hz_min),
                )
            if age_threshold > 0.0:
                capture_score = max(
                    capture_score, self._scene_age_s / age_threshold)
        scores['resource_pressure'] = min(
            1.0, max(cpu_score, memory_score, capture_score))

        return scores

    def _publish_risk(self) -> None:
        raw_scores = self._compute_raw_scores()
        source_status = self._source_status()
        result = self._aggregator.aggregate(raw_scores, source_status)
        safety_scores = dict(raw_scores)
        safety_scores['resource_pressure'] = 0.0
        safety_status = dict(source_status)
        safety_status['resource_pressure'] = {
            'valid': False,
            'reason_code': 'excluded_from_safety_estop',
            'provenance': '/system/telemetry+/recorder/diagnostics',
        }
        safety_result = self._aggregator.aggregate(
            safety_scores, safety_status
        )
        if (
            source_status['resource_pressure']['valid']
            and raw_scores['resource_pressure'] >= 0.85
        ):
            # Resource pressure is operational Hold/degrade evidence, never an
            # automatic R3 safety E-stop source.
            if result.primary_driver == 'resource_pressure':
                result.level = 2
                result.recommendation = RECOMMENDATIONS['resource_pressure']
            elif result.level < 2:
                result.level = 2
                result.primary_driver = 'resource_pressure'
                result.recommendation = RECOMMENDATIONS['resource_pressure']
        if (
            source_status['dynamics_anomaly']['valid']
            and self._latest_metrics
            and self._latest_metrics.soft_limit_triggered
            and result.level < 2
        ):
            # A soft-limit breach is a safety override: require degraded operation even
            # when the weighted aggregate score would otherwise stay below R2.
            result.level = 2
            result.primary_driver = 'dynamics_anomaly'
            result.recommendation = RECOMMENDATIONS['dynamics_anomaly']

        if (
            self.get_parameter('auto_e_stop_on_r3').value
            and safety_result.level >= 3
            and not self._e_stop_active
        ):
            self._trigger_e_stop(result, reason='e_stop_triggered')

        if result.level != self._prev_level:
            self._publish_alert('level_change', result)
            self._prev_level = result.level

        status = RiskStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.validity = result.validity
        status.reason_code = result.reason_code
        status.has_valid_sources = bool(result.active_dimensions)
        status.active_dimensions = result.active_dimensions
        status.invalid_dimensions = result.invalid_dimensions
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
            attr.source_valid = dim.source_valid
            attr.validity = dim.validity
            attr.reason_code = dim.reason_code
            attr.provenance = dim.provenance
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
