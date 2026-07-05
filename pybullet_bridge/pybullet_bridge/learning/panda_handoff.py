"""Validation utilities for Panda bridge handoff bundles."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


EXPECTED_HANDOFF_FORMAT = 'panda_bridge_handoff_v0'
EXPECTED_ROBOT = 'panda'
EXPECTED_SCHEMA_ID = 'panda_ee_delta_gripper_v0'
EXPECTED_ACTION_TYPE = 'ee_delta_gripper'
EXPECTED_ACTION_DIM = 7


@dataclass(frozen=True)
class PandaHandoff:
    """Parsed and validated Panda action handoff bundle."""

    manifest: Mapping[str, Any]
    replay_check: Mapping[str, Any]
    rows: Sequence[Mapping[str, Any]]
    schema_id: str
    action_type: str
    action_dim: int
    timestamps: np.ndarray
    actions: np.ndarray


def load_handoff_bundle(
    path: Path | str,
    *,
    expected_schema_id: str = EXPECTED_SCHEMA_ID,
    expected_action_type: str = EXPECTED_ACTION_TYPE,
) -> PandaHandoff:
    """Load and validate a Panda handoff bundle directory."""

    bundle_path = Path(path)
    if not bundle_path.exists():
        raise FileNotFoundError(f'Panda handoff bundle does not exist: {bundle_path}')
    if not bundle_path.is_dir():
        raise ValueError(f'Panda handoff path must be a directory: {bundle_path}')

    manifest_path = bundle_path / 'handoff_manifest.json'
    replay_check_path = bundle_path / 'replay_check.json'
    actions_path = bundle_path / 'predicted_actions.jsonl'

    manifest = _load_json_object(manifest_path, 'handoff manifest')
    replay_check = _load_json_object(replay_check_path, 'replay check')

    handoff_format = manifest.get('handoff_format')
    if handoff_format != EXPECTED_HANDOFF_FORMAT:
        raise ValueError(
            'handoff_manifest.json handoff_format must be '
            f'{EXPECTED_HANDOFF_FORMAT!r}, got {handoff_format!r}'
        )

    status = replay_check.get('status')
    if status != 'PASS':
        raise ValueError(f'replay_check.json status must be PASS, got {status!r}')

    return load_action_jsonl(
        actions_path,
        manifest=manifest,
        replay_check=replay_check,
        expected_schema_id=expected_schema_id,
        expected_action_type=expected_action_type,
    )


def load_action_jsonl(
    path: Path | str,
    *,
    manifest: Mapping[str, Any] | None = None,
    replay_check: Mapping[str, Any] | None = None,
    expected_schema_id: str = EXPECTED_SCHEMA_ID,
    expected_action_type: str = EXPECTED_ACTION_TYPE,
) -> PandaHandoff:
    """Load and validate a Panda predicted_actions.jsonl file."""

    jsonl_path = Path(path)
    if not jsonl_path.exists():
        raise FileNotFoundError(f'Panda action JSONL does not exist: {jsonl_path}')
    if not jsonl_path.is_file():
        raise ValueError(f'Panda action JSONL path must be a file: {jsonl_path}')

    rows: list[Mapping[str, Any]] = []
    timestamps: list[float] = []
    actions: list[np.ndarray] = []
    with jsonl_path.open('r', encoding='utf-8') as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                row = json.loads(stripped)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f'Invalid JSON in Panda action JSONL at line {line_number}: {exc}'
                ) from exc
            _validate_row(
                row,
                line_number=line_number,
                expected_schema_id=expected_schema_id,
                expected_action_type=expected_action_type,
            )
            rows.append(row)
            timestamps.append(float(row['timestamp']))
            actions.append(np.asarray(row['action'], dtype=np.float64))

    if not rows:
        raise ValueError('Panda action JSONL must contain at least one action row')

    return PandaHandoff(
        manifest=dict(manifest or {}),
        replay_check=dict(replay_check or {}),
        rows=tuple(rows),
        schema_id=expected_schema_id,
        action_type=expected_action_type,
        action_dim=EXPECTED_ACTION_DIM,
        timestamps=np.asarray(timestamps, dtype=np.float64),
        actions=np.vstack(actions),
    )


def _load_json_object(path: Path, label: str) -> Mapping[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f'Missing {label}: {path}')
    with path.open('r', encoding='utf-8') as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError(f'{label} must be a JSON object')
    return payload


def _validate_row(
    row: object,
    *,
    line_number: int,
    expected_schema_id: str,
    expected_action_type: str,
) -> None:
    if not isinstance(row, dict):
        raise ValueError(f'Panda action row {line_number} must be a JSON object')

    _expect_field(row, 'robot', EXPECTED_ROBOT, line_number)
    _expect_field(row, 'schema_id', expected_schema_id, line_number)
    _expect_field(row, 'action_type', expected_action_type, line_number)

    try:
        timestamp = np.asarray([row.get('timestamp')], dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f'Panda action row {line_number} timestamp must be numeric'
        ) from exc
    if not np.isfinite(timestamp).all():
        raise ValueError(f'Panda action row {line_number} timestamp must be finite')

    try:
        action = np.asarray(row.get('action'), dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'Panda action row {line_number} action must be numeric') from exc
    if action.shape != (EXPECTED_ACTION_DIM,):
        raise ValueError(
            f'Panda action row {line_number} action must have shape '
            f'[{EXPECTED_ACTION_DIM}], got {list(action.shape)}'
        )
    if not np.isfinite(action).all():
        raise ValueError(f'Panda action row {line_number} action must contain finite values')


def _expect_field(
    row: Mapping[str, Any],
    field_name: str,
    expected: str,
    line_number: int,
) -> None:
    actual = row.get(field_name)
    if actual != expected:
        raise ValueError(
            f'Panda action row {line_number} {field_name} must be '
            f'{expected!r}, got {actual!r}'
        )
