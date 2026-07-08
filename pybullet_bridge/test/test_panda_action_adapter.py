"""Tests for Panda task-space action adaptation."""

import numpy as np
import pytest

from pybullet_bridge.learning import PandaActionAdapter, PandaActionAdapterConfig


def test_panda_action_adapter_hold_keeps_joint_positions():
    adapter = PandaActionAdapter(PandaActionAdapterConfig(command_mode='hold'))
    action = np.array([0.001, 0.0, -0.002, 0.0, 0.0, 0.01, 0.5])
    obs = {'joint_positions': np.array([0.2, -0.1, 0.3], dtype=np.float64)}

    command = adapter.to_joint_command(action, obs, ['j1', 'j2', 'j3'])

    np.testing.assert_allclose(command.joint_targets, obs['joint_positions'])
    assert command.gripper_command == pytest.approx(0.5)
    assert command.command_mode == 'hold'


def test_panda_action_adapter_mock_ik_applies_small_delta_to_first_joints():
    adapter = PandaActionAdapter(PandaActionAdapterConfig(command_mode='mock_ik'))
    action = np.array([0.001, -0.002, 0.003, 0.01, -0.02, 0.03, 0.0])
    obs = {'joint_positions': np.zeros(7, dtype=np.float64)}

    joint_targets = adapter.to_joint_target(action, obs, [f'j{idx}' for idx in range(7)])

    np.testing.assert_allclose(
        joint_targets,
        np.array([0.001, -0.002, 0.003, 0.01, -0.02, 0.03, 0.0]),
    )


@pytest.mark.parametrize(
    ('action', 'message'),
    [
        (np.zeros(6), 'shape \\[7\\]'),
        (np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), 'delta_xyz exceeds'),
        (np.array([0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0]), 'delta_rpy exceeds'),
        (np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0]), 'gripper command'),
        (np.array([0.0, 0.0, np.inf, 0.0, 0.0, 0.0, 0.0]), 'finite'),
    ],
)
def test_panda_action_adapter_rejects_invalid_actions(action, message):
    adapter = PandaActionAdapter()
    obs = {'joint_positions': np.zeros(7, dtype=np.float64)}

    with pytest.raises(ValueError, match=message):
        adapter.to_joint_target(action, obs, [f'j{idx}' for idx in range(7)])


def test_panda_action_adapter_validates_joint_name_dimension():
    adapter = PandaActionAdapter()
    action = np.zeros(7, dtype=np.float64)
    obs = {'joint_positions': np.zeros(2, dtype=np.float64)}

    with pytest.raises(ValueError, match='joint_names length'):
        adapter.to_joint_target(action, obs, ['j1'])


def test_panda_action_adapter_rejects_unsupported_mode():
    with pytest.raises(ValueError, match='Unsupported Panda command mode'):
        PandaActionAdapter(PandaActionAdapterConfig(command_mode='moveit_ik'))


def test_panda_action_adapter_pybullet_ik_computes_valid_ik():
    adapter = PandaActionAdapter(PandaActionAdapterConfig(command_mode='pybullet_ik'))
    action = np.array([0.01, 0.0, -0.01, 0.0, 0.0, 0.0, 0.5])
    obs = {'joint_positions': np.zeros(7, dtype=np.float64)}

    joint_targets = adapter.to_joint_target(action, obs, [f'j{idx}' for idx in range(7)])

    assert len(joint_targets) == 7
    assert np.isfinite(joint_targets).all()
    assert not np.allclose(joint_targets, np.zeros(7))


def test_panda_action_adapter_deadband_mock_ik():
    config = PandaActionAdapterConfig(
        command_mode='mock_ik',
        enable_deadband=True,
        deadband_val=0.001,
        deadband_feedforward=0.0002,
    )
    adapter = PandaActionAdapter(config)
    obs = {'joint_positions': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)}

    # Tiny delta (0.0005 < 0.001) -> returns joint_positions
    action_tiny = np.array([0.0005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    targets_tiny = adapter.to_joint_target(action_tiny, obs, [f'j{idx}' for idx in range(7)])
    np.testing.assert_allclose(targets_tiny, obs['joint_positions'])

    # Large delta (0.002 > 0.001) -> returns target + feedforward (0.002 + 0.0002 = 0.0022)
    action_large = np.array([0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    targets_large = adapter.to_joint_target(action_large, obs, [f'j{idx}' for idx in range(7)])
    assert targets_large[0] == pytest.approx(0.0022)


def test_panda_action_adapter_backlash_mock_ik():
    config = PandaActionAdapterConfig(
        command_mode='mock_ik',
        enable_backlash=True,
        backlash_val=0.001,
    )
    adapter = PandaActionAdapter(config)
    obs = {'joint_positions': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)}

    # First step sets history -> target = [0.01, 0.0, ...]
    action1 = np.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    targets1 = adapter.to_joint_target(action1, obs, [f'j{idx}' for idx in range(7)])
    np.testing.assert_allclose(targets1[:2], [0.01, 0.0])

    # Second step within backlash gap (0.0105 - 0.01 = 0.0005 < 0.001) -> returns y_prev (0.01)
    obs2 = {'joint_positions': targets1}
    action2a = np.array([0.0005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    targets2a = adapter.to_joint_target(action2a, obs2, [f'j{idx}' for idx in range(7)])
    assert targets2a[0] == pytest.approx(0.01)

    # Second step outside backlash gap (0.012 - 0.01 = 0.002 > 0.001) -> returns target - backlash (0.012 - 0.001 = 0.011)
    action2b = np.array([0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    targets2b = adapter.to_joint_target(action2b, obs2, [f'j{idx}' for idx in range(7)])
    assert targets2b[0] == pytest.approx(0.011)


def test_panda_action_adapter_velocity_limit_mock_ik():
    config = PandaActionAdapterConfig(
        command_mode='mock_ik',
        enable_limits=True,
        max_joint_velocity=2.0,       # rad/s
        max_joint_acceleration=1000.0,  # high acceleration to avoid clamping
        control_loop_dt=0.01,         # 100Hz
    )
    adapter = PandaActionAdapter(config)
    obs1 = {'joint_positions': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)}

    # First command to establish history -> target = [0.01, 0.0, ...]
    action1 = np.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    targets1 = adapter.to_joint_target(action1, obs1, [f'j{idx}' for idx in range(7)])

    # Try huge velocity step: from 0.01 to 0.05 in 0.01s (vel = 4.0 rad/s > 2.0 rad/s)
    # Clamped to step = vel_max * dt = 2.0 * 0.01 = 0.02 -> target = 0.01 + 0.02 = 0.03
    action2 = np.array([0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    obs2 = {'joint_positions': targets1}
    targets2 = adapter.to_joint_target(action2, obs2, [f'j{idx}' for idx in range(7)])
    assert targets2[0] == pytest.approx(0.03)


def test_panda_action_adapter_acceleration_limit_mock_ik():
    config = PandaActionAdapterConfig(
        command_mode='mock_ik',
        enable_limits=True,
        max_joint_velocity=100.0,      # high velocity to avoid clamping
        max_joint_acceleration=10.0,   # rad/s^2
        control_loop_dt=0.01,          # 100Hz
    )
    adapter = PandaActionAdapter(config)
    obs1 = {'joint_positions': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)}

    # First command to establish history -> target = [0.01, 0.0, ...], vel = 0.0
    action1 = np.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    targets1 = adapter.to_joint_target(action1, obs1, [f'j{idx}' for idx in range(7)])

    # Try huge acceleration step: from +0.0 rad/s to +2.0 rad/s in 0.01s (accel = 200.0 rad/s^2 > 10.0 rad/s^2)
    # Clamped to accel_max * dt = 10.0 * 0.01 = 0.1 rad/s -> new vel = 0.0 + 0.1 = 0.1 rad/s
    # target = 0.01 + 0.1 * 0.01 = 0.011
    action2 = np.array([0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    obs2 = {'joint_positions': targets1}
    targets2 = adapter.to_joint_target(action2, obs2, [f'j{idx}' for idx in range(7)])
    assert targets2[0] == pytest.approx(0.011)
