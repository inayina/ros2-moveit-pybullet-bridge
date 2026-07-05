"""Tests for Panda JSONL action replay policy."""

import json

import numpy as np
import pytest

from pybullet_bridge.learning import BasePolicy, JsonlActionReplayPolicy


def _row(timestamp, action):
    return {
        'timestamp': timestamp,
        'episode_index': 0,
        'frame_index': int(round(timestamp * 1000)),
        'task': 'pick_lift',
        'robot': 'panda',
        'schema_id': 'panda_ee_delta_gripper_v0',
        'release_id': 'panda_demo_delta_v0',
        'action_type': 'ee_delta_gripper',
        'action': action,
    }


def _write_jsonl(path, rows):
    path.write_text(
        ''.join(json.dumps(row) + '\n' for row in rows),
        encoding='utf-8',
    )


def _write_bundle(path, rows):
    path.mkdir()
    (path / 'handoff_manifest.json').write_text(
        json.dumps({'handoff_format': 'panda_bridge_handoff_v0'}),
        encoding='utf-8',
    )
    (path / 'replay_check.json').write_text(
        json.dumps({'status': 'PASS'}),
        encoding='utf-8',
    )
    _write_jsonl(path / 'predicted_actions.jsonl', rows)


def test_jsonl_action_replay_policy_replays_and_holds_last_frame(tmp_path):
    bundle = tmp_path / 'bridge_handoff'
    first = [0.001, 0.0, -0.002, 0.0, 0.0, 0.01, 0.0]
    second = [0.002, 0.0, -0.003, 0.0, 0.0, 0.02, 0.1]
    _write_bundle(bundle, [_row(0.033, first), _row(0.066, second)])

    policy = JsonlActionReplayPolicy(str(bundle), inference_freq=25)
    obs = {'joint_positions': np.zeros(2, dtype=np.float64)}

    assert isinstance(policy, BasePolicy)
    assert policy.inference_freq == 25
    assert policy.schema_id == 'panda_ee_delta_gripper_v0'
    assert policy.action_type == 'ee_delta_gripper'
    assert policy.action_dim == 7
    np.testing.assert_allclose(policy.timestamps, np.array([0.033, 0.066]))
    np.testing.assert_allclose(policy.get_action(obs), np.asarray(first))
    np.testing.assert_allclose(policy.get_action(obs), np.asarray(second))
    np.testing.assert_allclose(policy.get_action(obs), np.asarray(second))


def test_jsonl_action_replay_policy_reset_and_loop_from_jsonl(tmp_path):
    replay_path = tmp_path / 'predicted_actions.jsonl'
    _write_jsonl(
        replay_path,
        [
            _row(0.0, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            _row(0.1, [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ],
    )

    policy = JsonlActionReplayPolicy(str(replay_path), loop=True)
    obs = {'joint_positions': np.zeros(1, dtype=np.float64)}

    np.testing.assert_allclose(policy.get_action(obs)[0], 1.0)
    np.testing.assert_allclose(policy.get_action(obs)[0], 2.0)
    np.testing.assert_allclose(policy.get_action(obs)[0], 1.0)
    policy.reset()
    np.testing.assert_allclose(policy.get_action(obs)[0], 1.0)


def test_jsonl_action_replay_policy_rejects_bad_schema_guard(tmp_path):
    replay_path = tmp_path / 'predicted_actions.jsonl'
    _write_jsonl(replay_path, [_row(0.0, [0.0] * 7)])

    with pytest.raises(ValueError, match='schema_id'):
        JsonlActionReplayPolicy(str(replay_path), expected_schema_id='other_schema')
