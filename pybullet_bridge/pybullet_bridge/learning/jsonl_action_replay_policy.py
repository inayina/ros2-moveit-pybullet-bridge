"""Open-loop replay policy for Panda JSONL task-space actions."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Mapping, Optional, Sequence

import numpy as np

from pybullet_bridge.learning.base_policy import BasePolicy
from pybullet_bridge.learning.panda_handoff import (
    EXPECTED_ACTION_TYPE,
    EXPECTED_SCHEMA_ID,
    PandaHandoff,
    load_action_jsonl,
    load_handoff_bundle,
)


class JsonlActionReplayPolicy(BasePolicy):
    """Replay Panda ee_delta_gripper actions from a handoff bundle or JSONL file."""

    def __init__(
        self,
        path: str,
        inference_freq: int = 50,
        loop: bool = False,
        expected_schema_id: str = EXPECTED_SCHEMA_ID,
        expected_action_type: str = EXPECTED_ACTION_TYPE,
    ) -> None:
        if inference_freq <= 0:
            raise ValueError('inference_freq must be positive')

        self._path = Path(path)
        self._inference_freq = int(inference_freq)
        self._loop = bool(loop)
        self._step_index = 0

        self._handoff = self._load(
            self._path,
            expected_schema_id=expected_schema_id,
            expected_action_type=expected_action_type,
        )
        self._actions = self._handoff.actions
        self._timestamps = self._handoff.timestamps

    @property
    def inference_freq(self) -> int:
        return self._inference_freq

    @property
    def schema_id(self) -> str:
        return self._handoff.schema_id

    @property
    def action_type(self) -> str:
        return self._handoff.action_type

    @property
    def action_dim(self) -> int:
        return self._handoff.action_dim

    @property
    def manifest(self) -> Mapping[str, object]:
        return dict(self._handoff.manifest)

    @property
    def replay_check(self) -> Mapping[str, object]:
        return dict(self._handoff.replay_check)

    @property
    def rows(self) -> Sequence[Mapping[str, object]]:
        return tuple(self._handoff.rows)

    @property
    def timestamps(self) -> Optional[np.ndarray]:
        return self._timestamps.copy()

    def reset(self) -> None:
        self._step_index = 0

    def get_action(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        del obs
        index = min(self._step_index, len(self._actions) - 1)
        action = self._actions[index].copy()

        if self._loop:
            self._step_index = (self._step_index + 1) % len(self._actions)
        elif self._step_index < len(self._actions) - 1:
            self._step_index += 1

        return action

    @staticmethod
    def _load(
        path: Path,
        *,
        expected_schema_id: str,
        expected_action_type: str,
    ) -> PandaHandoff:
        if path.is_dir():
            return load_handoff_bundle(
                path,
                expected_schema_id=expected_schema_id,
                expected_action_type=expected_action_type,
            )
        return load_action_jsonl(
            path,
            expected_schema_id=expected_schema_id,
            expected_action_type=expected_action_type,
        )
