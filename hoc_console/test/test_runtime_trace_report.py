"""M5 HOC command-correlation and bundle export tests."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from hoc_console.runtime_trace_report import (
    build_runtime_trace_report,
    export_policy_trace_bundle,
)


def _row(artifact_type: str, sequence: int = 42) -> dict:
    row = {
        'contract_version': 'panda_policy_runtime_v1',
        'artifact_type': artifact_type,
        'event_id': f'{artifact_type}:{sequence}',
        'parent_event_id': f'command:{sequence}',
        'trace_run_id': 'trace_m5',
        'episode_id': 'episode_m5',
        'command_sequence': sequence,
        'claims_task_success': False,
    }
    if artifact_type == 'policy_command':
        row.update({
            'event_id': f'command:{sequence}',
            'action_schema_version': 'panda_absolute_eef_gripper_v0',
            'action': [0.42, 0.0, 0.31, 0.0, 1.0, 0.0, 0.0, 0.7],
        })
    return row


def _timelines() -> dict[str, list[dict]]:
    return {
        'brain': [_row('policy_health_timeline_event')],
        'execution': [_row('policy_execution_report')],
        'safety': [_row('risk_timeline_event')],
        'task_gt': [_row('task_gt_timeline_event')],
    }


def test_runtime_trace_report_correlates_all_four_lanes() -> None:
    report = build_runtime_trace_report([_row('policy_command')], _timelines())
    assert report['issues'] == []
    assert report['command_count'] == 1
    assert report['correlated_command_count'] == 1
    assert report['is_closed_loop'] is False
    assert report['claims_task_success'] is False
    assert report['commands'][0]['task_gt'][0]['command_sequence'] == 42


def test_runtime_trace_report_refuses_missing_execution(tmp_path: Path) -> None:
    timelines = _timelines()
    timelines['execution'] = []
    report = build_runtime_trace_report([_row('policy_command')], timelines)
    assert 'missing_execution_report' in report['issues']
    with pytest.raises(ValueError, match='missing_execution_report'):
        export_policy_trace_bundle(
            tmp_path / 'bundle', [_row('policy_command')], timelines
        )


def test_runtime_trace_report_checks_claims_inside_deque() -> None:
    from collections import deque

    timelines = _timelines()
    timelines['task_gt'][0]['nested'] = deque([
        {'claims_task_success': True}
    ])
    report = build_runtime_trace_report([_row('policy_command')], timelines)
    assert 'invalid_task_success_claim' in report['issues']


def test_warmup_events_without_command_are_reported_but_not_exported(
    tmp_path: Path,
) -> None:
    timelines = _timelines()
    timelines['brain'].insert(0, {
        'artifact_type': 'policy_health_timeline_event',
        'validity': 'WARMING_UP', 'claims_task_success': False,
    })
    report = build_runtime_trace_report([_row('policy_command')], timelines)
    assert report['issues'] == []
    assert report['unscoped_event_counts']['brain'] == 1
    root = tmp_path / 'bundle'
    manifest = export_policy_trace_bundle(
        root, [_row('policy_command')], timelines
    )
    assert manifest['files']['policy_health_timeline']['record_count'] == 1


def test_export_bundle_writes_hashed_five_track_evidence(
    tmp_path: Path,
) -> None:
    root = tmp_path / 'bundle'
    manifest = export_policy_trace_bundle(
        root, [_row('policy_command')], _timelines()
    )
    assert manifest['bundle_format'] == 'panda_policy_trace_bundle_v1'
    assert set(manifest['files']) == {
        'policy_commands', 'policy_health_timeline', 'execution_reports',
        'risk_timeline', 'task_gt_timeline',
    }
    for file_record in manifest['files'].values():
        path = root / file_record['path']
        assert hashlib.sha256(path.read_bytes()).hexdigest() == file_record['sha256']
        assert len(path.read_text(encoding='utf-8').splitlines()) == 1
    persisted = json.loads((root / 'manifest.json').read_text(encoding='utf-8'))
    assert persisted['is_closed_loop'] is False
    assert persisted['claims_task_success'] is False
