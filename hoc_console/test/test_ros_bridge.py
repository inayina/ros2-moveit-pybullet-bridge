"""Unit tests for ROS message to JSON conversion."""

from bridge_monitor_msgs.msg import DistributionMetrics, RiskAttribution, RiskStatus
from sensor_msgs.msg import JointState

from hoc_console.ros_bridge import (
    distribution_metrics_to_dict,
    risk_status_to_dict,
    tracking_error_to_dict,
)


def test_risk_status_to_dict():
    msg = RiskStatus()
    msg.level = 2
    msg.composite_score = 0.62
    msg.primary_driver = 'tracking_error'
    msg.recommendation = 'check gains'
    msg.e_stop_active = True
    msg.degraded_mode = False

    attr = RiskAttribution()
    attr.dimension = 'tracking_error'
    attr.raw_score = 0.8
    attr.weighted_score = 0.2
    attr.weight = 0.25
    attr.is_primary_driver = True
    msg.attribution = [attr]

    data = risk_status_to_dict(msg)
    assert data['level'] == 2
    assert data['level_name'] == 'R2'
    assert data['composite_score'] == 0.62
    assert data['primary_driver'] == 'tracking_error'
    assert data['e_stop_active'] is True
    assert len(data['attribution']) == 1
    assert data['attribution'][0]['dimension'] == 'tracking_error'
    assert data['attribution'][0]['is_primary_driver'] is True


def test_distribution_metrics_to_dict():
    msg = DistributionMetrics()
    msg.joint_names = ['j1', 'j2']
    msg.kl_divergence_per_joint = [0.1, 0.2]
    msg.kl_divergence_mean = 0.15
    msg.mmd_statistic = 0.05
    msg.mmd_p_value = 0.01
    msg.mmd_threshold = 0.03
    msg.window_duration_sec = 5.0
    msg.sample_count_sim = 100
    msg.sample_count_real = 95
    msg.shift_detected = True
    msg.detection_method = 'mmd'

    data = distribution_metrics_to_dict(msg)
    assert data['joint_names'] == ['j1', 'j2']
    assert data['kl_divergence_mean'] == 0.15
    assert data['shift_detected'] is True
    assert data['detection_method'] == 'mmd'
    assert data['sample_count_sim'] == 100


def test_tracking_error_to_dict():
    msg = JointState()
    msg.name = ['joint_1', 'joint_2']
    msg.position = [0.01, 0.02]

    data = tracking_error_to_dict(msg)
    assert data['joint_names'] == ['joint_1', 'joint_2']
    assert data['errors'] == [0.01, 0.02]
