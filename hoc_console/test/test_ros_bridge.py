"""Unit tests for ROS message to JSON conversion."""

from types import SimpleNamespace

from bridge_monitor_msgs.msg import (
    DistributionMetrics,
    GraspStatus,
    RiskAttribution,
    RiskStatus,
)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from hoc_console.ros_bridge import (
    diagnostic_array_to_dict,
    distribution_metrics_to_dict,
    execution_report_to_dict,
    grasp_status_to_dict,
    policy_health_to_dict,
    policy_command_to_dict,
    risk_status_to_dict,
    safety_decision_to_dict,
    task_evaluation_to_dict,
    tracking_error_to_dict,
)
from sensor_msgs.msg import JointState


def test_risk_status_to_dict():
    msg = RiskStatus()
    msg.level = 2
    msg.composite_score = 0.62
    msg.primary_driver = 'tracking_error'
    msg.recommendation = 'check gains'
    msg.e_stop_active = True
    msg.degraded_mode = False
    msg.validity = 'DEGRADED'
    msg.reason_code = 'partial_sources_unavailable'
    msg.has_valid_sources = True
    msg.active_dimensions = ['tracking_error']
    msg.invalid_dimensions = ['distribution_shift']

    attr = RiskAttribution()
    attr.dimension = 'tracking_error'
    attr.raw_score = 0.8
    attr.weighted_score = 0.2
    attr.weight = 0.25
    attr.is_primary_driver = True
    attr.source_valid = True
    attr.validity = 'VALID'
    attr.reason_code = 'none'
    attr.provenance = '/monitor/tracking_error'
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
    assert data['invalid_dimensions'] == ['distribution_shift']
    assert data['attribution'][0]['source_valid'] is True


def test_distribution_metrics_to_dict():
    msg = DistributionMetrics()
    msg.joint_names = ['j1', 'j2']
    msg.kl_divergence_per_joint = [0.1, 0.2]
    msg.kl_divergence_mean = 0.15
    msg.wasserstein_per_joint = [0.05, 0.06]
    msg.wasserstein_mean = 0.055
    msg.w1_threshold = 0.08
    msg.shift_detected_w1 = False
    msg.mmd_statistic = 0.05
    msg.mmd_p_value = 0.01
    msg.mmd_threshold = 0.03
    msg.window_duration_sec = 5.0
    msg.sample_count_sim = 100
    msg.sample_count_real = 95
    msg.sim_position_min_per_joint = [0.0, 0.1]
    msg.sim_position_q1_per_joint = [0.1, 0.2]
    msg.sim_position_median_per_joint = [0.2, 0.3]
    msg.sim_position_q3_per_joint = [0.3, 0.4]
    msg.sim_position_max_per_joint = [0.4, 0.5]
    msg.real_position_min_per_joint = [0.05, 0.15]
    msg.real_position_q1_per_joint = [0.15, 0.25]
    msg.real_position_median_per_joint = [0.25, 0.35]
    msg.real_position_q3_per_joint = [0.35, 0.45]
    msg.real_position_max_per_joint = [0.45, 0.55]
    msg.shift_detected = True
    msg.detection_method = 'mmd'
    msg.validity = 'VALID'
    msg.reason_code = 'none'
    msg.metric_valid = True
    msg.baseline_ready = True
    msg.calibration_id = 'same_scene'
    msg.reference_source = 'topic'
    msg.aligned_sample_count = 95
    msg.comm_health_valid = True
    msg.dynamics_valid = True

    data = distribution_metrics_to_dict(msg)
    assert data['joint_names'] == ['j1', 'j2']
    assert data['kl_divergence_mean'] == 0.15
    assert data['wasserstein_mean'] == 0.055
    assert data['w1_threshold'] == 0.08
    assert data['shift_detected'] is True
    assert data['detection_method'] == 'mmd'
    assert data['sample_count_sim'] == 100
    assert data['sim_position_median_per_joint'] == [0.2, 0.3]
    assert data['real_position_median_per_joint'] == [0.25, 0.35]
    assert data['metric_valid'] is True
    assert data['calibration_id'] == 'same_scene'


def test_tracking_error_to_dict():
    msg = JointState()
    msg.name = ['joint_1', 'joint_2']
    msg.position = [0.01, 0.02]

    data = tracking_error_to_dict(msg)
    assert data['joint_names'] == ['joint_1', 'joint_2']
    assert data['errors'] == [0.01, 0.02]


def test_grasp_status_to_dict():
    msg = GraspStatus()
    msg.grasp_established = True
    msg.object_slipped = False
    msg.net_wrench.force.x = 3.0
    msg.net_wrench.force.y = 4.0
    msg.confidence = 0.95

    data = grasp_status_to_dict(msg)

    assert data['grasp_established'] is True
    assert data['object_slipped'] is False
    assert data['force_norm'] == 5.0
    assert data['confidence'] == 0.95


def test_diagnostic_array_to_dict_parses_numeric_and_array_values():
    msg = DiagnosticArray()
    status = DiagnosticStatus()
    status.name = 'system_telemetry/host'
    status.message = 'ok'
    status.values = [
        KeyValue(key='cpu_total_percent', value='87.5'),
        KeyValue(key='cpu_per_core_percent', value='[80.0, 95.0]'),
        KeyValue(key='recording', value='true'),
    ]
    msg.status = [status]

    data = diagnostic_array_to_dict(msg)

    values = data['statuses'][0]['values']
    assert values['cpu_total_percent'] == 87.5
    assert values['cpu_per_core_percent'] == [80.0, 95.0]
    assert values['recording'] is True


def test_policy_health_converter_preserves_unavailable_values() -> None:
    msg = DiagnosticArray()
    status = DiagnosticStatus()
    status.name = 'policy_runtime/brain'
    status.message = 'queue_underrun'
    status.values = [
        KeyValue(key='validity', value='ERROR'),
        KeyValue(key='reason_code', value='queue_underrun'),
        KeyValue(key='queue_depth', value='0'),
        KeyValue(key='observation_age_ms', value='unavailable'),
        KeyValue(key='last_command_sequence', value='42'),
        KeyValue(key='trace_run_id', value='trace-health'),
        KeyValue(key='episode_id', value='episode-health'),
    ]
    msg.status = [status]
    data = policy_health_to_dict(msg)
    assert data['validity'] == 'ERROR'
    assert data['reason_code'] == 'queue_underrun'
    assert data['observation_age_ms'] == 'unavailable'
    assert data['command_sequence'] == 42
    assert data['trace_run_id'] == 'trace-health'
    assert data['episode_id'] == 'episode-health'


def test_safety_decision_converter_exposes_proposed_actual_mismatch() -> None:
    msg = DiagnosticArray()
    status = DiagnosticStatus()
    status.name = 'policy_runtime/safety_decision'
    status.message = 'risk_r2_hold'
    status.values = [
        KeyValue(key='proposed_decision', value='HOLD'),
        KeyValue(key='actual_decision', value='RUN'),
        KeyValue(key='decision_mismatch', value='true'),
        KeyValue(key='dry_run', value='true'),
    ]
    msg.status = [status]
    data = safety_decision_to_dict(msg)
    assert data['proposed_decision'] == 'HOLD'
    assert data['actual_decision'] == 'RUN'
    assert data['decision_mismatch'] is True
    assert data['dry_run'] is True


def test_execution_and_task_gt_converters_keep_trace_separate() -> None:
    stamp = SimpleNamespace(sec=100, nanosec=1)
    header = SimpleNamespace(stamp=stamp)
    execution = SimpleNamespace(
        header=header, received_stamp=stamp,
        contract_version='panda_policy_runtime_v1', event_id='exec:1',
        parent_event_id='command:1', trace_run_id='trace', episode_id='ep',
        validity='VALID', reason_code='none', command_sequence=1,
        accepted=True, decision='EXECUTED',
        source_action_schema_version='smolvla_panda_abs_eef8_v1',
        execution_action_schema_version='panda_bounded_pose_gripper_v0',
        source_action=[0.0] * 8, has_bounded_action=True,
        bounded_action=[0.0] * 8, clipped=False, clip_axes=[],
        hold_active=False, estop_active=False, adapter_name='shadow',
        adapter_version='m2_v1',
    )
    task = SimpleNamespace(
        header=header, received_stamp=stamp,
        contract_version='panda_policy_runtime_v1', event_id='gt:1',
        parent_event_id='exec:1', trace_run_id='trace', episode_id='ep',
        validity='VALID', reason_code='none', phase='lift',
        task_status='RUNNING', reach=1, grasp=1, lift=-1, place=-1,
        has_object_delta=False, object_delta_x_m=0.0,
        object_delta_y_m=0.0, object_delta_z_m=0.0, gt_source='isaac_gt',
    )
    execution_data = execution_report_to_dict(execution)
    task_data = task_evaluation_to_dict(task)
    assert execution_data['parent_event_id'] == 'command:1'
    assert task_data['parent_event_id'] == 'exec:1'
    assert task_data['lift'] is None
    assert task_data['risk_may_override'] is False


def test_policy_command_converter_preserves_absolute_action_and_sequence() -> None:
    stamp = SimpleNamespace(sec=100, nanosec=1)
    msg = SimpleNamespace(
        header=SimpleNamespace(stamp=stamp), received_stamp=stamp,
        valid_until=stamp, contract_version='panda_policy_runtime_v1',
        event_id='command:7', parent_event_id='observation:3',
        trace_run_id='trace', episode_id='episode', validity='VALID',
        reason_code='none', observation_sequence=3, command_sequence=7,
        action_schema_version='panda_absolute_eef_gripper_v0',
        action=[0.42, 0.0, 0.31, 0.0, 1.0, 0.0, 0.0, 0.7],
        chunk_index=0, chunk_size=1, from_prefetched_chunk=False,
        inference_latency_ms=24.5,
    )
    data = policy_command_to_dict(msg)
    assert data['command_sequence'] == 7
    assert data['action_schema_version'] == 'panda_absolute_eef_gripper_v0'
    assert data['source_stamp'] == {'sec': 100, 'nanosec': 1}
    assert data['claims_task_success'] is False
