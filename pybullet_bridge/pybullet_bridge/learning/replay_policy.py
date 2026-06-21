"""Open-loop replay policy backed by a pickled joint trajectory."""

from pathlib import Path
import pickle
from typing import Dict, Optional, Sequence

import numpy as np

from pybullet_bridge.learning.base_policy import BasePolicy


class ReplayPolicy(BasePolicy):
    """Replay target joint positions from a deterministic trajectory file."""

    def __init__(
        self,
        path: str,
        inference_freq: int = 50,
        loop: bool = False,
    ) -> None:
        if inference_freq <= 0:
            raise ValueError('inference_freq must be positive')

        self._path = Path(path)
        self._inference_freq = int(inference_freq)
        self._loop = bool(loop)
        self._step_index = 0

        payload = self._load_payload(self._path)
        self._positions = self._parse_positions(payload)
        self._joint_names = self._parse_joint_names(payload, self._positions.shape[1])
        self._timestamps = self._parse_timestamps(payload, self._positions.shape[0])

    @property
    def inference_freq(self) -> int:
        return self._inference_freq

    @property
    def joint_names(self) -> Sequence[str]:
        return tuple(self._joint_names)

    @property
    def timestamps(self) -> Optional[np.ndarray]:
        if self._timestamps is None:
            return None
        return self._timestamps.copy()

    def reset(self) -> None:
        self._step_index = 0

    def get_action(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        self._validate_observation(obs)

        index = min(self._step_index, len(self._positions) - 1)
        action = self._positions[index].copy()

        if self._loop:
            self._step_index = (self._step_index + 1) % len(self._positions)
        elif self._step_index < len(self._positions) - 1:
            self._step_index += 1

        return action

    @staticmethod
    def _load_payload(path: Path) -> object:
        if not path.exists():
            raise FileNotFoundError(f'Replay file does not exist: {path}')
        with path.open('rb') as handle:
            return pickle.load(handle)

    @staticmethod
    def _parse_positions(payload: object) -> np.ndarray:
        if not isinstance(payload, dict) or 'positions' not in payload:
            raise ValueError("Replay pkl must contain a 'positions' field")

        positions = np.asarray(payload['positions'], dtype=np.float64)
        if positions.ndim != 2:
            raise ValueError('positions must be a 2D array with shape [T, J]')
        if positions.shape[0] == 0 or positions.shape[1] == 0:
            raise ValueError('positions must contain at least one timestep and joint')
        return positions

    @staticmethod
    def _parse_joint_names(payload: object, joint_count: int) -> Sequence[str]:
        if not isinstance(payload, dict):
            return tuple(f'joint_{idx}' for idx in range(joint_count))

        joint_names = payload.get('joint_names')
        if joint_names is None:
            return tuple(f'joint_{idx}' for idx in range(joint_count))
        if len(joint_names) != joint_count:
            raise ValueError('joint_names length must match positions joint dimension')
        return tuple(str(name) for name in joint_names)

    @staticmethod
    def _parse_timestamps(payload: object, timestep_count: int) -> Optional[np.ndarray]:
        if not isinstance(payload, dict) or payload.get('timestamps') is None:
            return None

        timestamps = np.asarray(payload['timestamps'], dtype=np.float64)
        if timestamps.ndim != 1 or timestamps.shape[0] != timestep_count:
            raise ValueError('timestamps must be a 1D array with one entry per timestep')
        return timestamps

    def _validate_observation(self, obs: Dict[str, np.ndarray]) -> None:
        if 'joint_positions' not in obs:
            return

        joint_positions = np.asarray(obs['joint_positions'])
        if joint_positions.ndim != 1:
            raise ValueError('obs["joint_positions"] must be a 1D array')
        if joint_positions.shape[0] != self._positions.shape[1]:
            raise ValueError('observation joint dimension must match replay action dimension')
