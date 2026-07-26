"""M5 command-correlated four-lane runtime evidence and bundle export."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from collections.abc import Mapping, Sequence
from typing import Any


BUNDLE_FORMAT = 'panda_policy_trace_bundle_v1'
CONTRACT_VERSION = 'panda_policy_runtime_v1'


def _claims_are_false(value) -> bool:
    if isinstance(value, Mapping):
        if value.get('claims_task_success', False) is not False:
            return False
        return all(_claims_are_false(child) for child in value.values())
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return all(_claims_are_false(child) for child in value)
    return True


def build_runtime_trace_report(
    policy_commands: Sequence[Mapping[str, Any]],
    runtime_timelines: Mapping[str, Sequence[Mapping[str, Any]]],
) -> dict[str, Any]:
    """Correlate commands with Brain/Execution/Safety/Task GT evidence."""
    commands = [dict(row) for row in policy_commands]
    sequences = [row.get('command_sequence') for row in commands]
    issues = []
    if sequences != sorted(set(sequences)):
        issues.append('command_sequence_regression_or_duplicate')
    command_index = {
        int(row['command_sequence']): {
            'policy_command': row,
            'brain': [],
            'execution': [],
            'safety': [],
            'task_gt': [],
        }
        for row in commands
        if isinstance(row.get('command_sequence'), int)
    }
    unscoped_event_counts = {
        lane: 0 for lane in ('brain', 'execution', 'safety', 'task_gt')
    }
    traces = {row.get('trace_run_id') for row in commands}
    episodes = {row.get('episode_id') for row in commands}
    if len(traces) > 1 or len(episodes) > 1:
        issues.append('multiple_trace_or_episode_ids')

    for lane in ('brain', 'execution', 'safety', 'task_gt'):
        for raw in runtime_timelines.get(lane, ()):
            row = dict(raw)
            sequence = row.get('command_sequence')
            if not isinstance(sequence, int):
                # Startup/warm-up telemetry can legitimately precede the first
                # execution report. Keep it visible in the session report, but
                # do not place it in a replay trace or make export impossible.
                unscoped_event_counts[lane] += 1
                continue
            if sequence not in command_index:
                issues.append(f'orphan_{lane}_event')
                continue
            command = command_index[sequence]['policy_command']
            if (
                row.get('trace_run_id') != command.get('trace_run_id')
                or row.get('episode_id') != command.get('episode_id')
            ):
                issues.append(f'{lane}_trace_mismatch')
                continue
            command_index[sequence][lane].append(row)

    missing_execution = [
        sequence for sequence, row in command_index.items()
        if not row['execution']
    ]
    if missing_execution:
        issues.append('missing_execution_report')
    if not _claims_are_false(commands) or not _claims_are_false(runtime_timelines):
        issues.append('invalid_task_success_claim')
    return {
        'report_format': 'panda_policy_runtime_trace_report_v1',
        'contract_version': CONTRACT_VERSION,
        'is_closed_loop': False,
        'claims_task_success': False,
        'trace_run_ids': sorted(value for value in traces if value),
        'episode_ids': sorted(value for value in episodes if value),
        'command_count': len(commands),
        'correlated_command_count': sum(
            1 for row in command_index.values() if row['execution']
        ),
        'unscoped_event_counts': unscoped_event_counts,
        'issues': sorted(set(issues)),
        'commands': [
            {'command_sequence': sequence, **command_index[sequence]}
            for sequence in sorted(command_index)
        ],
    }


def _jsonl_text(rows: Sequence[Mapping[str, Any]]) -> str:
    return ''.join(
        json.dumps(dict(row), sort_keys=True, separators=(',', ':')) + '\n'
        for row in rows
    )


def _write_jsonl(path: Path, rows: Sequence[Mapping[str, Any]]) -> dict:
    text = _jsonl_text(rows)
    path.write_text(text, encoding='utf-8')
    return {
        'path': path.name,
        'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'record_count': len(rows),
    }


def export_policy_trace_bundle(
    output_dir: str | Path,
    policy_commands: Sequence[Mapping[str, Any]],
    runtime_timelines: Mapping[str, Sequence[Mapping[str, Any]]],
) -> dict[str, Any]:
    """Write a replayable bundle only when correlation is complete."""
    report = build_runtime_trace_report(policy_commands, runtime_timelines)
    if report['issues']:
        raise ValueError(
            'cannot export incomplete policy trace: '
            + ', '.join(report['issues'])
        )
    if report['command_count'] == 0:
        raise ValueError('cannot export policy trace without commands')
    trace_id = report['trace_run_ids'][0]
    episode_id = report['episode_ids'][0]
    root = Path(output_dir).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)

    commands = [dict(row) for row in policy_commands]
    correlated = report['commands']
    brain = [row for item in correlated for row in item['brain']]
    execution = [row for item in correlated for row in item['execution']]
    safety = [row for item in correlated for row in item['safety']]
    task_gt = [row for item in correlated for row in item['task_gt']]
    files = {
        'policy_commands': _write_jsonl(
            root / 'policy_commands.jsonl', commands
        ),
        'policy_health_timeline': _write_jsonl(
            root / 'policy_health_timeline.jsonl',
            brain,
        ),
        'execution_reports': _write_jsonl(
            root / 'execution_reports.jsonl', execution
        ),
        'risk_timeline': _write_jsonl(
            root / 'risk_timeline.jsonl',
            safety,
        ),
        'task_gt_timeline': _write_jsonl(
            root / 'task_gt_timeline.jsonl',
            task_gt,
        ),
    }
    sequences = [int(row['command_sequence']) for row in commands]
    manifest = {
        'bundle_format': BUNDLE_FORMAT,
        'contract_version': CONTRACT_VERSION,
        'trace_run_id': trace_id,
        'episode_id': episode_id,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'is_closed_loop': False,
        'claims_task_success': False,
        'files': files,
        'sequence_bounds': {
            'first': min(sequences), 'last': max(sequences)
        },
        'correlation': {
            'command_count': len(commands),
            'execution_report_count': len(execution),
            'orphan_execution_report_count': 0,
            'missing_execution_report_count': 0,
            'sequence_regression_count': 0,
            'trace_consistent': True,
        },
    }
    (root / 'manifest.json').write_text(
        json.dumps(manifest, indent=2, sort_keys=True), encoding='utf-8'
    )
    return manifest
