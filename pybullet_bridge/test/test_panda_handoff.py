"""Tests for Panda handoff bundle validation."""

import json

import numpy as np
import pytest

from pybullet_bridge.learning.panda_handoff import load_handoff_bundle


def _row(action=None, **overrides):
    payload = {
        'timestamp': 0.033,
        'episode_index': 0,
        'frame_index': 1,
        'task': 'pick_lift',
        'robot': 'panda',
        'schema_id': 'panda_ee_delta_gripper_v0',
        'release_id': 'panda_demo_delta_v0',
        'action_type': 'ee_delta_gripper',
        'action': [0.001, 0.0, -0.002, 0.0, 0.0, 0.01, 0.0],
    }
    if action is not None:
        payload['action'] = action
    payload.update(overrides)
    return payload


def _write_bundle(path, rows=None, manifest=None, replay_check=None):
    path.mkdir()
    (path / 'handoff_manifest.json').write_text(
        json.dumps(manifest or {'handoff_format': 'panda_bridge_handoff_v0'}),
        encoding='utf-8',
    )
    (path / 'replay_check.json').write_text(
        json.dumps(replay_check or {'status': 'PASS'}),
        encoding='utf-8',
    )
    rows = rows or [_row()]
    (path / 'predicted_actions.jsonl').write_text(
        ''.join(json.dumps(row) + '\n' for row in rows),
        encoding='utf-8',
    )


def test_load_handoff_bundle_accepts_valid_panda_jsonl(tmp_path):
    bundle = tmp_path / 'bridge_handoff'
    _write_bundle(bundle, rows=[_row(), _row(timestamp=0.066)])

    handoff = load_handoff_bundle(bundle)

    assert handoff.schema_id == 'panda_ee_delta_gripper_v0'
    assert handoff.action_type == 'ee_delta_gripper'
    assert handoff.action_dim == 7
    np.testing.assert_allclose(handoff.timestamps, np.array([0.033, 0.066]))
    assert handoff.actions.shape == (2, 7)
    assert handoff.manifest['handoff_format'] == 'panda_bridge_handoff_v0'
    assert handoff.replay_check['status'] == 'PASS'


def test_load_handoff_bundle_requires_manifest(tmp_path):
    bundle = tmp_path / 'bridge_handoff'
    _write_bundle(bundle)
    (bundle / 'handoff_manifest.json').unlink()

    with pytest.raises(FileNotFoundError, match='handoff manifest'):
        load_handoff_bundle(bundle)


def test_load_handoff_bundle_rejects_failed_replay_check(tmp_path):
    bundle = tmp_path / 'bridge_handoff'
    _write_bundle(bundle, replay_check={'status': 'FAIL'})

    with pytest.raises(ValueError, match='status must be PASS'):
        load_handoff_bundle(bundle)


@pytest.mark.parametrize(
    ('rows', 'message'),
    [
        ([_row(action=[0.0, 0.1])], 'shape \\[7\\]'),
        ([_row(robot='iiwa7')], 'robot'),
        ([_row(schema_id='other_schema')], 'schema_id'),
        ([_row(action_type='joint_positions')], 'action_type'),
        ([_row(action=[0.0, 0.0, 0.0, 0.0, float('nan'), 0.0, 0.0])], 'finite'),
    ],
)
def test_load_handoff_bundle_rejects_contract_violations(tmp_path, rows, message):
    bundle = tmp_path / 'bridge_handoff'
    _write_bundle(bundle, rows=rows)

    with pytest.raises(ValueError, match=message):
        load_handoff_bundle(bundle)
