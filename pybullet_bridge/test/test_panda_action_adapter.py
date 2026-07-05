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
