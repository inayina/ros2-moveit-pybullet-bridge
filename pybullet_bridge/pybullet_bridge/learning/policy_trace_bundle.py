"""Strict loader for M5 policy trace replay bundles."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


BUNDLE_FORMAT = 'panda_policy_trace_bundle_v1'
CONTRACT_VERSION = 'panda_policy_runtime_v1'
ABSOLUTE_ACTION_SCHEMA = 'panda_absolute_eef_gripper_v0'
FILE_KEYS = (
    'policy_commands',
    'policy_health_timeline',
    'execution_reports',
    'risk_timeline',
    'task_gt_timeline',
)
ARTIFACT_TYPES = {
    'policy_commands': 'policy_command',
    'policy_health_timeline': 'policy_health_timeline_event',
    'execution_reports': 'policy_execution_report',
    'risk_timeline': 'risk_timeline_event',
    'task_gt_timeline': 'task_gt_timeline_event',
}


@dataclass(frozen=True)
class PolicyTraceBundle:
    """Validated immutable view of one policy trace bundle."""

    root: Path
    manifest: Mapping[str, object]
    records: Mapping[str, tuple[Mapping[str, object], ...]]
    correlation_index: Mapping[int, Mapping[str, object]]


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read_jsonl(path: Path) -> tuple[Mapping[str, object], ...]:
    rows = []
    for line_number, line in enumerate(
        path.read_text(encoding='utf-8').splitlines(), start=1
    ):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as error:
            raise ValueError(
                f'invalid JSONL at {path.name}:{line_number}: {error}'
            ) from error
        if not isinstance(row, dict):
            raise ValueError(f'{path.name}:{line_number} must be an object')
        rows.append(row)
    return tuple(rows)


def _assert_claims_false(value, location: str) -> None:
    if isinstance(value, dict):
        if value.get('claims_task_success', False) is not False:
            raise ValueError(f'claims_task_success must be false at {location}')
        for key, child in value.items():
            _assert_claims_false(child, f'{location}.{key}')
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _assert_claims_false(child, f'{location}[{index}]')


def load_policy_trace_bundle(path: str | Path) -> PolicyTraceBundle:
    """Load and fully correlate a fail-closed M5 replay bundle."""
    root = Path(path).expanduser().resolve()
    manifest_path = root / 'manifest.json'
    if not manifest_path.is_file():
        raise FileNotFoundError(f'M5 manifest does not exist: {manifest_path}')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if manifest.get('bundle_format') != BUNDLE_FORMAT:
        raise ValueError('bundle_format mismatch')
    if manifest.get('contract_version') != CONTRACT_VERSION:
        raise ValueError('contract_version mismatch')
    if manifest.get('is_closed_loop') is not False:
        raise ValueError('M5 replay must declare is_closed_loop=false')
    _assert_claims_false(manifest, 'manifest')

    trace = manifest.get('trace_run_id')
    episode = manifest.get('episode_id')
    files = manifest.get('files')
    if not isinstance(trace, str) or not trace:
        raise ValueError('trace_run_id is required')
    if not isinstance(episode, str) or not episode:
        raise ValueError('episode_id is required')
    if not isinstance(files, dict) or set(files) != set(FILE_KEYS):
        raise ValueError(f'files must contain exactly {list(FILE_KEYS)}')

    all_records: dict[str, tuple[Mapping[str, object], ...]] = {}
    for key in FILE_KEYS:
        record = files[key]
        if not isinstance(record, dict):
            raise ValueError(f'files.{key} must be an object')
        relative = Path(str(record.get('path', '')))
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError(f'files.{key}.path must stay inside bundle')
        file_path = (root / relative).resolve()
        if root not in file_path.parents or not file_path.is_file():
            raise ValueError(f'files.{key}.path is missing or outside bundle')
        if _sha256(file_path) != record.get('sha256'):
            raise ValueError(f'files.{key} sha256 mismatch')
        rows = _read_jsonl(file_path)
        if len(rows) != record.get('record_count'):
            raise ValueError(f'files.{key} record_count mismatch')
        for index, row in enumerate(rows):
            if row.get('contract_version') != CONTRACT_VERSION:
                raise ValueError(f'{key}[{index}] contract_version mismatch')
            if row.get('artifact_type') != ARTIFACT_TYPES[key]:
                raise ValueError(f'{key}[{index}] artifact_type mismatch')
            if row.get('trace_run_id') != trace:
                raise ValueError(f'{key}[{index}] trace_run_id mismatch')
            if row.get('episode_id') != episode:
                raise ValueError(f'{key}[{index}] episode_id mismatch')
            _assert_claims_false(row, f'{key}[{index}]')
        all_records[key] = rows

    commands = all_records['policy_commands']
    if not commands:
        raise ValueError('policy_commands must not be empty')
    command_sequences = [row.get('command_sequence') for row in commands]
    if any(not isinstance(sequence, int) for sequence in command_sequences):
        raise ValueError('policy command_sequence must be integer')
    if command_sequences != sorted(set(command_sequences)):
        raise ValueError('policy command_sequence regression or duplicate')
    for index, row in enumerate(commands):
        if row.get('action_schema_version') != ABSOLUTE_ACTION_SCHEMA:
            raise ValueError(f'policy_commands[{index}] action schema mismatch')
        action = row.get('action')
        if (
            not isinstance(action, list)
            or len(action) != 8
            or not all(isinstance(value, (int, float)) and math.isfinite(value)
                       for value in action)
        ):
            raise ValueError(f'policy_commands[{index}] requires finite action[8]')

    command_by_sequence = {
        int(row['command_sequence']): row for row in commands
    }
    reports = all_records['execution_reports']
    report_by_sequence = {}
    for index, report in enumerate(reports):
        sequence = report.get('command_sequence')
        if not isinstance(sequence, int) or sequence not in command_by_sequence:
            raise ValueError(f'execution_reports[{index}] is orphaned')
        if sequence in report_by_sequence:
            raise ValueError('duplicate execution report sequence')
        command = command_by_sequence[sequence]
        if report.get('parent_event_id') != command.get('event_id'):
            raise ValueError('execution report parent_event_id mismatch')
        report_by_sequence[sequence] = report
    missing = set(command_by_sequence) - set(report_by_sequence)
    if missing:
        raise ValueError(f'missing execution report for sequences {sorted(missing)}')

    index: dict[int, dict[str, object]] = {
        sequence: {'policy_command': command, 'execution_report': report_by_sequence[sequence]}
        for sequence, command in command_by_sequence.items()
    }
    for key in ('policy_health_timeline', 'risk_timeline', 'task_gt_timeline'):
        for row_index, row in enumerate(all_records[key]):
            sequence = row.get('command_sequence')
            if not isinstance(sequence, int) or sequence not in index:
                raise ValueError(f'{key}[{row_index}] references unknown command')
            index[sequence].setdefault(key, []).append(row)

    bounds = manifest.get('sequence_bounds', {})
    if bounds != {'first': command_sequences[0], 'last': command_sequences[-1]}:
        raise ValueError('sequence_bounds mismatch')
    correlation = manifest.get('correlation', {})
    expected = {
        'command_count': len(commands),
        'execution_report_count': len(reports),
        'orphan_execution_report_count': 0,
        'missing_execution_report_count': 0,
        'sequence_regression_count': 0,
        'trace_consistent': True,
    }
    if correlation != expected:
        raise ValueError('manifest correlation summary mismatch')
    return PolicyTraceBundle(root, manifest, all_records, index)
