"""Open-loop replay of versioned Panda PolicyCommand trace bundles."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Mapping, Sequence

import numpy as np

from pybullet_bridge.learning.base_policy import BasePolicy
from pybullet_bridge.learning.policy_trace_bundle import (
    PolicyTraceBundle,
    load_policy_trace_bundle,
)


class PolicyCommandReplayPolicy(BasePolicy):
    """Replay absolute EEF8 actions while preserving trace evidence."""

    def __init__(
        self,
        path: str | Path,
        inference_freq: int = 10,
        loop: bool = False,
    ) -> None:
        if inference_freq <= 0:
            raise ValueError('inference_freq must be positive')
        self._bundle = load_policy_trace_bundle(path)
        self._inference_freq = int(inference_freq)
        self._loop = bool(loop)
        self._step_index = 0
        self._commands = self._bundle.records['policy_commands']

    @property
    def inference_freq(self) -> int:
        return self._inference_freq

    @property
    def bundle(self) -> PolicyTraceBundle:
        return self._bundle

    @property
    def rows(self) -> Sequence[Mapping[str, object]]:
        return self._commands

    @property
    def trace_run_id(self) -> str:
        return str(self._bundle.manifest['trace_run_id'])

    @property
    def is_closed_loop(self) -> bool:
        return False

    @property
    def current_command_sequence(self) -> int:
        index = min(self._step_index, len(self._commands) - 1)
        return int(self._commands[index]['command_sequence'])

    def reset(self) -> None:
        self._step_index = 0

    def get_action(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        del obs
        index = min(self._step_index, len(self._commands) - 1)
        action = np.asarray(self._commands[index]['action'], dtype=np.float64)
        if self._loop:
            self._step_index = (self._step_index + 1) % len(self._commands)
        elif self._step_index < len(self._commands) - 1:
            self._step_index += 1
        return action.copy()
