"""M5 trace bundle replay and absolute EEF adapter tests."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from pybullet_bridge.learning import (
    PandaAbsoluteEefReplayAdapter,
    PandaAbsoluteEefReplayAdapterConfig,
    PolicyCommandReplayPolicy,
    load_policy_trace_bundle,
)


def _write_jsonl(path: Path, rows: list[dict]) -> dict:
    text = ''.join(json.dumps(row, separators=(',', ':')) + '\n' for row in rows)
    path.write_text(text, encoding='utf-8')
    return {
        'path': path.name,
        'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'record_count': len(rows),
    }


def _row(kind: str, sequence: int = 42) -> dict:
    base = {
        'contract_version': 'panda_policy_runtime_v1',
        'artifact_type': kind,
        'event_id': f'{kind}:{sequence}',
        'parent_event_id': f'command:{sequence}',
        'trace_run_id': 'm5_test_trace',
        'episode_id': 'episode_test',
        'command_sequence': sequence,
        'claims_task_success': False,
    }
    if kind == 'policy_command':
        base.update({
            'event_id': f'command:{sequence}',
            'action_schema_version': 'panda_absolute_eef_gripper_v0',
            'action': [0.42, 0.0, 0.31, 0.0, 1.0, 0.0, 0.0, 0.7],
        })
    return base


def _bundle(tmp_path: Path) -> Path:
    root = tmp_path / 'trace_bundle'
    root.mkdir()
    command = _row('policy_command')
    report = _row('policy_execution_report')
    files = {
        'policy_commands': _write_jsonl(
            root / 'policy_commands.jsonl', [command]
        ),
        'policy_health_timeline': _write_jsonl(
            root / 'policy_health_timeline.jsonl',
            [_row('policy_health_timeline_event')],
        ),
        'execution_reports': _write_jsonl(
            root / 'execution_reports.jsonl', [report]
        ),
        'risk_timeline': _write_jsonl(
            root / 'risk_timeline.jsonl', [_row('risk_timeline_event')]
        ),
        'task_gt_timeline': _write_jsonl(
            root / 'task_gt_timeline.jsonl',
            [_row('task_gt_timeline_event')],
        ),
    }
    manifest = {
        'bundle_format': 'panda_policy_trace_bundle_v1',
        'contract_version': 'panda_policy_runtime_v1',
        'trace_run_id': 'm5_test_trace',
        'episode_id': 'episode_test',
        'created_at_utc': '2026-07-26T09:30:00Z',
        'is_closed_loop': False,
        'claims_task_success': False,
        'files': files,
        'sequence_bounds': {'first': 42, 'last': 42},
        'correlation': {
            'command_count': 1,
            'execution_report_count': 1,
            'orphan_execution_report_count': 0,
            'missing_execution_report_count': 0,
            'sequence_regression_count': 0,
            'trace_consistent': True,
        },
    }
    (root / 'manifest.json').write_text(
        json.dumps(manifest), encoding='utf-8'
    )
    return root


def _rewrite_file(root: Path, key: str, rows: list[dict]) -> None:
    manifest_path = root / 'manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    path = root / manifest['files'][key]['path']
    manifest['files'][key] = _write_jsonl(path, rows)
    manifest_path.write_text(json.dumps(manifest), encoding='utf-8')


def test_policy_command_replay_loads_trace_and_holds_last(tmp_path: Path) -> None:
    root = _bundle(tmp_path)
    policy = PolicyCommandReplayPolicy(root)
    action = policy.get_action({})
    assert policy.trace_run_id == 'm5_test_trace'
    assert policy.is_closed_loop is False
    assert policy.bundle.correlation_index[42]['execution_report'][
        'artifact_type'
    ] == 'policy_execution_report'
    np.testing.assert_allclose(action, policy.get_action({}))


def test_trace_bundle_rejects_hash_tamper(tmp_path: Path) -> None:
    root = _bundle(tmp_path)
    with (root / 'policy_commands.jsonl').open('a', encoding='utf-8') as handle:
        handle.write('{}\n')
    with pytest.raises(ValueError, match='sha256 mismatch'):
        load_policy_trace_bundle(root)


def test_trace_bundle_rejects_task_claim_even_with_valid_hash(
    tmp_path: Path,
) -> None:
    root = _bundle(tmp_path)
    row = _row('task_gt_timeline_event')
    row['claims_task_success'] = True
    _rewrite_file(root, 'task_gt_timeline', [row])
    with pytest.raises(ValueError, match='claims_task_success'):
        load_policy_trace_bundle(root)


def test_trace_bundle_rejects_missing_execution_report(tmp_path: Path) -> None:
    root = _bundle(tmp_path)
    _rewrite_file(root, 'execution_reports', [])
    with pytest.raises(ValueError, match='missing execution report'):
        load_policy_trace_bundle(root)


def test_trace_bundle_rejects_absolute_schema_mismatch(tmp_path: Path) -> None:
    root = _bundle(tmp_path)
    command = _row('policy_command')
    command['action_schema_version'] = 'panda_ee_delta_gripper_v0'
    _rewrite_file(root, 'policy_commands', [command])
    with pytest.raises(ValueError, match='action schema mismatch'):
        load_policy_trace_bundle(root)


def test_absolute_adapter_hold_preserves_joints_and_gripper() -> None:
    adapter = PandaAbsoluteEefReplayAdapter()
    joints = np.linspace(-0.3, 0.3, 7)
    action = np.array([0.42, 0.0, 0.31, 0.0, 1.0, 0.0, 0.0, 0.7])
    command = adapter.to_joint_command(
        action, {'joint_positions': joints}, [f'j{i}' for i in range(7)]
    )
    np.testing.assert_allclose(command.joint_targets, joints)
    assert command.gripper_command == pytest.approx(0.7)
    assert command.command_mode == 'hold'


@pytest.mark.parametrize('action,message', [
    (np.zeros(7), 'shape \\[8\\]'),
    (np.array([0.9, 0.0, 0.31, 0.0, 1.0, 0.0, 0.0, 0.7]), 'workspace'),
    (np.array([0.42, 0.0, 0.31, 0.0, 0.0, 0.0, 0.0, 0.7]), 'unit norm'),
    (np.array([0.42, 0.0, 0.31, 0.0, 1.0, 0.0, 0.0, 1.1]), 'gripper'),
])
def test_absolute_adapter_rejects_invalid_actions(action, message) -> None:
    adapter = PandaAbsoluteEefReplayAdapter(
        PandaAbsoluteEefReplayAdapterConfig(command_mode='hold')
    )
    with pytest.raises(ValueError, match=message):
        adapter.to_joint_target(
            action,
            {'joint_positions': np.zeros(7)},
            [f'j{i}' for i in range(7)],
        )
