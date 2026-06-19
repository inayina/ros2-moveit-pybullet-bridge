"""Convert ROS 2 messages to JSON-serializable dicts for WebSocket push."""

from __future__ import annotations

from bridge_monitor_msgs.msg import DistributionMetrics, DomainRandomizationConfig, RiskStatus
from sensor_msgs.msg import JointState


def tracking_error_to_dict(msg: JointState) -> dict:
    return {
        'joint_names': list(msg.name),
        'errors': list(msg.position),
    }


def risk_status_to_dict(msg: RiskStatus) -> dict:
    return {
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
            }
            for a in msg.attribution
        ],
    }


def distribution_metrics_to_dict(msg: DistributionMetrics) -> dict:
    return {
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
    }
