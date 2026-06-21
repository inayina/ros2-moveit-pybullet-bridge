"""Unit tests for baseline learning policies."""

import pickle

import numpy as np
import pytest

from pybullet_bridge.learning import BasePolicy, ReplayPolicy, SineWavePolicy


def _write_replay(path, payload):
    with path.open('wb') as handle:
        pickle.dump(payload, handle)


def test_replay_policy_replays_and_holds_last_frame(tmp_path):
    replay_path = tmp_path / 'episode.pkl'
    _write_replay(
        replay_path,
        {
            'joint_names': ['j1', 'j2'],
            'positions': np.array([[0.0, 0.1], [0.2, 0.3]], dtype=np.float64),
            'timestamps': np.array([0.0, 0.02], dtype=np.float64),
        },
    )

    policy = ReplayPolicy(str(replay_path), inference_freq=25)
    obs = {'joint_positions': np.zeros(2, dtype=np.float64)}

    assert isinstance(policy, BasePolicy)
    assert policy.inference_freq == 25
    assert policy.joint_names == ('j1', 'j2')
    np.testing.assert_allclose(policy.timestamps, np.array([0.0, 0.02]))
    np.testing.assert_allclose(policy.get_action(obs), np.array([0.0, 0.1]))
    np.testing.assert_allclose(policy.get_action(obs), np.array([0.2, 0.3]))
    np.testing.assert_allclose(policy.get_action(obs), np.array([0.2, 0.3]))


def test_replay_policy_reset_and_loop(tmp_path):
    replay_path = tmp_path / 'loop.pkl'
    _write_replay(
        replay_path,
        {
            'positions': np.array([[1.0], [2.0]], dtype=np.float64),
        },
    )

    policy = ReplayPolicy(str(replay_path), loop=True)
    obs = {'joint_positions': np.zeros(1, dtype=np.float64)}

    np.testing.assert_allclose(policy.get_action(obs), np.array([1.0]))
    np.testing.assert_allclose(policy.get_action(obs), np.array([2.0]))
    np.testing.assert_allclose(policy.get_action(obs), np.array([1.0]))
    policy.reset()
    np.testing.assert_allclose(policy.get_action(obs), np.array([1.0]))


def test_replay_policy_validates_payload(tmp_path):
    replay_path = tmp_path / 'bad.pkl'
    _write_replay(replay_path, {'positions': np.array([1.0, 2.0])})

    with pytest.raises(ValueError, match='2D array'):
        ReplayPolicy(str(replay_path))


def test_replay_policy_validates_observation_dimension(tmp_path):
    replay_path = tmp_path / 'episode.pkl'
    _write_replay(replay_path, {'positions': np.zeros((2, 3), dtype=np.float64)})
    policy = ReplayPolicy(str(replay_path))

    with pytest.raises(ValueError, match='observation joint dimension'):
        policy.get_action({'joint_positions': np.zeros(2, dtype=np.float64)})


def test_sine_wave_policy_is_deterministic_after_reset():
    policy = SineWavePolicy(amplitude=0.1, frequency_hz=0.5, inference_freq=10, seed=42)
    obs = {'joint_positions': np.array([0.5, -0.5], dtype=np.float64)}

    first = policy.get_action(obs)
    second = policy.get_action(obs)
    policy.reset()
    first_after_reset = policy.get_action(obs)

    assert isinstance(policy, BasePolicy)
    assert policy.inference_freq == 10
    np.testing.assert_allclose(first, first_after_reset)
    assert not np.allclose(first, second)
    np.testing.assert_allclose(obs['joint_positions'], np.array([0.5, -0.5]))


def test_sine_wave_policy_validates_observation():
    policy = SineWavePolicy()

    with pytest.raises(ValueError, match='joint_positions'):
        policy.get_action({})
    with pytest.raises(ValueError, match='1D array'):
        policy.get_action({'joint_positions': np.zeros((1, 1), dtype=np.float64)})


def test_sine_wave_policy_rejects_joint_dimension_changes():
    policy = SineWavePolicy(seed=7)
    policy.get_action({'joint_positions': np.zeros(2, dtype=np.float64)})

    with pytest.raises(ValueError, match='joint dimension changed'):
        policy.get_action({'joint_positions': np.zeros(3, dtype=np.float64)})
