"""Convert ROS 2 messages to JSON-serializable dicts for WebSocket push."""

from __future__ import annotations

import json

from bridge_monitor_msgs.msg import (
    DistributionMetrics,
    GraspStatus,
    RiskStatus,
)
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState


def _stamp_to_dict(stamp) -> dict:
    return {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)}


def tracking_error_to_dict(msg: JointState) -> dict:
    return {
        'joint_names': list(msg.name),
        'errors': list(msg.position),
    }


def _parse_diagnostic_value(value: str):
    try:
        return json.loads(value)
    except (json.JSONDecodeError, TypeError):
        return value


def diagnostic_array_to_dict(msg: DiagnosticArray) -> dict:
    return {
        'statuses': [
            {
                'name': status.name,
                'level': status.level,
                'message': status.message,
                'hardware_id': status.hardware_id,
                'values': {
                    item.key: _parse_diagnostic_value(item.value)
                    for item in status.values
                },
            }
            for status in msg.status
        ],
    }


def grasp_status_to_dict(msg: GraspStatus) -> dict:
    force = msg.net_wrench.force
    torque = msg.net_wrench.torque
    return {
        'grasp_established': msg.grasp_established,
        'object_slipped': msg.object_slipped,
        'force_xyz': [force.x, force.y, force.z],
        'torque_xyz': [torque.x, torque.y, torque.z],
        'force_norm': (force.x ** 2 + force.y ** 2 + force.z ** 2) ** 0.5,
        'confidence': msg.confidence,
    }


def risk_status_to_dict(msg: RiskStatus) -> dict:
    return {
        'validity': getattr(msg, 'validity', 'UNAVAILABLE'),
        'reason_code': getattr(msg, 'reason_code', 'legacy_no_validity'),
        'has_valid_sources': getattr(msg, 'has_valid_sources', False),
        'active_dimensions': list(getattr(msg, 'active_dimensions', [])),
        'invalid_dimensions': list(getattr(msg, 'invalid_dimensions', [])),
        'level': msg.level,
        'level_name': f'R{msg.level}',
        'composite_score': msg.composite_score,
        'primary_driver': msg.primary_driver,
        'recommendation': msg.recommendation,
        'e_stop_active': msg.e_stop_active,
        'degraded_mode': msg.degraded_mode,
        'attribution': [
            {
                'dimension': a.dimension,
                'raw_score': a.raw_score,
                'weighted_score': a.weighted_score,
                'weight': a.weight,
                'is_primary_driver': a.is_primary_driver,
                'source_valid': getattr(a, 'source_valid', False),
                'validity': getattr(a, 'validity', 'UNAVAILABLE'),
                'reason_code': getattr(
                    a, 'reason_code', 'legacy_no_validity'
                ),
                'provenance': getattr(a, 'provenance', 'legacy'),
            }
            for a in msg.attribution
        ],
    }


def distribution_metrics_to_dict(msg: DistributionMetrics) -> dict:
    return {
        'validity': getattr(msg, 'validity', 'UNAVAILABLE'),
        'reason_code': getattr(msg, 'reason_code', 'legacy_no_validity'),
        'metric_valid': getattr(msg, 'metric_valid', False),
        'baseline_ready': getattr(msg, 'baseline_ready', False),
        'calibration_id': getattr(msg, 'calibration_id', ''),
        'reference_source': getattr(msg, 'reference_source', 'legacy'),
        'aligned_sample_count': getattr(msg, 'aligned_sample_count', 0),
        'comm_health_valid': getattr(msg, 'comm_health_valid', False),
        'dynamics_valid': getattr(msg, 'dynamics_valid', False),
        'joint_names': list(msg.joint_names),
        'kl_divergence_per_joint': list(msg.kl_divergence_per_joint),
        'kl_divergence_mean': msg.kl_divergence_mean,
        'wasserstein_per_joint': list(msg.wasserstein_per_joint),
        'wasserstein_mean': msg.wasserstein_mean,
        'w1_threshold': msg.w1_threshold,
        'shift_detected_w1': msg.shift_detected_w1,
        'mmd_statistic': msg.mmd_statistic,
        'mmd_p_value': msg.mmd_p_value,
        'mmd_threshold': msg.mmd_threshold,
        'window_duration_sec': msg.window_duration_sec,
        'sample_count_sim': msg.sample_count_sim,
        'sample_count_real': msg.sample_count_real,
        'sim_position_min_per_joint': list(msg.sim_position_min_per_joint),
        'sim_position_q1_per_joint': list(msg.sim_position_q1_per_joint),
        'sim_position_median_per_joint': list(msg.sim_position_median_per_joint),
        'sim_position_q3_per_joint': list(msg.sim_position_q3_per_joint),
        'sim_position_max_per_joint': list(msg.sim_position_max_per_joint),
        'real_position_min_per_joint': list(msg.real_position_min_per_joint),
        'real_position_q1_per_joint': list(msg.real_position_q1_per_joint),
        'real_position_median_per_joint': list(msg.real_position_median_per_joint),
        'real_position_q3_per_joint': list(msg.real_position_q3_per_joint),
        'real_position_max_per_joint': list(msg.real_position_max_per_joint),
        'shift_detected': msg.shift_detected,
        'detection_method': msg.detection_method,
        'comm_health_score': msg.comm_health_score,
        'dynamics_anomaly_score': msg.dynamics_anomaly_score,
        'velocity_jump_per_joint': list(msg.velocity_jump_per_joint),
        'soft_limit_score': msg.soft_limit_score,
        'soft_limit_triggered': msg.soft_limit_triggered,
    }


def policy_health_to_dict(msg: DiagnosticArray) -> dict:
    """Convert policy runtime DiagnosticArray into the Brain lane payload."""
    status = next(
        (item for item in msg.status if item.name == 'policy_runtime/brain'),
        None,
    )
    if status is None:
        return {
            'lane': 'brain',
            'validity': 'UNAVAILABLE',
            'reason_code': 'policy_health_status_missing',
        }
    values = {
        item.key: _parse_diagnostic_value(item.value)
        for item in status.values
    }
    return {
        'lane': 'brain',
        'validity': str(values.get('validity', 'UNAVAILABLE')),
        'reason_code': str(values.get('reason_code', status.message)),
        'lifecycle_state': values.get('lifecycle_state', 'unknown'),
        'observation_age_ms': values.get('observation_age_ms'),
        'inference_latency_ms': values.get('inference_latency_ms_last'),
        'queue_depth': values.get('queue_depth'),
        'queue_underrun_count': values.get('queue_underrun_count'),
        'command_sequence': values.get('last_command_sequence'),
        'policy_loaded': values.get('policy_loaded', False),
        'contract_version': values.get('contract_version', ''),
        'contract_sha256': values.get('contract_sha256', ''),
        'trace_run_id': values.get('trace_run_id', ''),
        'episode_id': values.get('episode_id', ''),
        'claims_task_success': False,
    }


def safety_decision_to_dict(msg: DiagnosticArray) -> dict:
    """Convert the M4 proposed-vs-actual safety decision diagnostic."""
    status = next(
        (item for item in msg.status
         if item.name == 'policy_runtime/safety_decision'),
        None,
    )
    if status is None:
        return {
            'proposed_decision': 'UNAVAILABLE',
            'actual_decision': 'UNAVAILABLE',
            'decision_mismatch': False,
            'safety_decision_reason': 'safety_decision_status_missing',
        }
    values = {
        item.key: _parse_diagnostic_value(item.value)
        for item in status.values
    }
    return {
        **values,
        'safety_decision_reason': str(
            values.get('proposed_reason', status.message)
        ),
    }


def execution_report_to_dict(msg) -> dict:
    return {
        'artifact_type': 'policy_execution_report',
        'lane': 'execution',
        'contract_version': msg.contract_version,
        'event_id': msg.event_id,
        'parent_event_id': msg.parent_event_id,
        'trace_run_id': msg.trace_run_id,
        'episode_id': msg.episode_id,
        'source_stamp': _stamp_to_dict(msg.header.stamp),
        'received_stamp': _stamp_to_dict(msg.received_stamp),
        'validity': msg.validity,
        'reason_code': msg.reason_code,
        'command_sequence': msg.command_sequence,
        'accepted': msg.accepted,
        'decision': msg.decision,
        'source_action_schema_version': msg.source_action_schema_version,
        'execution_action_schema_version': msg.execution_action_schema_version,
        'source_action': list(msg.source_action),
        'bounded_action': (
            list(msg.bounded_action) if msg.has_bounded_action else None
        ),
        'clipped': msg.clipped,
        'clip_axes': list(msg.clip_axes),
        'hold_active': msg.hold_active,
        'estop_active': msg.estop_active,
        'adapter_name': msg.adapter_name,
        'adapter_version': msg.adapter_version,
        'claims_task_success': False,
    }


def policy_command_to_dict(msg) -> dict:
    """Convert PolicyCommand into the M5 replay JSONL representation."""
    return {
        'contract_version': msg.contract_version,
        'artifact_type': 'policy_command',
        'event_id': msg.event_id,
        'parent_event_id': msg.parent_event_id or None,
        'trace_run_id': msg.trace_run_id,
        'episode_id': msg.episode_id,
        'source_stamp': _stamp_to_dict(msg.header.stamp),
        'received_stamp': _stamp_to_dict(msg.received_stamp),
        'validity': msg.validity,
        'reason_code': msg.reason_code,
        'observation_sequence': msg.observation_sequence,
        'command_sequence': msg.command_sequence,
        'action_schema_version': msg.action_schema_version,
        'action': list(msg.action),
        'chunk_index': msg.chunk_index,
        'chunk_size': msg.chunk_size,
        'from_prefetched_chunk': msg.from_prefetched_chunk,
        'inference_latency_ms': msg.inference_latency_ms,
        'valid_until': _stamp_to_dict(msg.valid_until),
        'claims_task_success': False,
    }


def task_evaluation_to_dict(msg) -> dict:
    def outcome(value: int):
        return None if value < 0 else bool(value)

    return {
        'lane': 'task_gt',
        'artifact_type': 'task_gt_timeline_event',
        'contract_version': msg.contract_version,
        'event_id': msg.event_id,
        'parent_event_id': msg.parent_event_id,
        'trace_run_id': msg.trace_run_id,
        'episode_id': msg.episode_id,
        'source_stamp': _stamp_to_dict(msg.header.stamp),
        'received_stamp': _stamp_to_dict(msg.received_stamp),
        'validity': msg.validity,
        'reason_code': msg.reason_code,
        'phase': msg.phase,
        'task_status': msg.task_status,
        'reach': outcome(msg.reach),
        'grasp': outcome(msg.grasp),
        'lift': outcome(msg.lift),
        'place': outcome(msg.place),
        'object_delta_m': (
            [msg.object_delta_x_m, msg.object_delta_y_m, msg.object_delta_z_m]
            if msg.has_object_delta else None
        ),
        'gt_source': msg.gt_source,
        'risk_may_override': False,
        'claims_task_success': False,
    }
