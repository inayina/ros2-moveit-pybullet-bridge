"""Adapters from Panda task-space actions to bridge joint commands."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Sequence

import numpy as np


@dataclass(frozen=True)
class PandaJointCommand:
    """Bridge command produced from one Panda task-space action."""

    joint_targets: np.ndarray
    gripper_command: float
    command_mode: str


@dataclass(frozen=True)
class PandaActionAdapterConfig:
    """Safety limits for first-pass Panda action adaptation."""

    command_mode: str = 'hold'
    max_delta_xyz: float = 0.05
    max_delta_rpy: float = 0.25
    gripper_min: float = 0.0
    gripper_max: float = 1.0


class PandaActionAdapter:
    """Convert Panda ee_delta_gripper[7] actions into bridge joint targets.

    ``hold`` validates the action and keeps the current joint state. ``mock_ik`` is
    an offline-safe shim that maps small task-space deltas onto the first joints,
    making the PolicyRunner path testable before a real Panda IK backend exists.
    """

    _SUPPORTED_MODES = {'hold', 'mock_ik'}

    def __init__(self, config: PandaActionAdapterConfig | None = None) -> None:
        self._config = config or PandaActionAdapterConfig()
        if self._config.command_mode not in self._SUPPORTED_MODES:
            raise ValueError(
                'Unsupported Panda command mode '
                f'{self._config.command_mode!r}; supported modes: '
                f'{sorted(self._SUPPORTED_MODES)}'
            )
        if self._config.max_delta_xyz <= 0.0:
            raise ValueError('max_delta_xyz must be positive')
        if self._config.max_delta_rpy <= 0.0:
            raise ValueError('max_delta_rpy must be positive')
        if self._config.gripper_min > self._config.gripper_max:
            raise ValueError('gripper_min must be <= gripper_max')

    @property
    def command_mode(self) -> str:
        return self._config.command_mode

    def to_joint_command(
        self,
        action: np.ndarray,
        obs: Dict[str, np.ndarray],
        joint_names: Sequence[str],
    ) -> PandaJointCommand:
        action = self._validate_action(action)
        joint_positions = self._validate_joint_positions(obs)
        if len(joint_names) != joint_positions.shape[0]:
            raise ValueError(
                'joint_names length must match observation joint dimension: '
                f'{len(joint_names)} vs {joint_positions.shape[0]}'
            )

        self._validate_delta_limits(action)
        gripper_command = self._validate_gripper(action[6])

        if self._config.command_mode == 'hold':
            joint_targets = joint_positions.copy()
        else:
            joint_targets = self._mock_ik(action, joint_positions)

        return PandaJointCommand(
            joint_targets=joint_targets,
            gripper_command=gripper_command,
            command_mode=self._config.command_mode,
        )

    def to_joint_target(
        self,
        action: np.ndarray,
        obs: Dict[str, np.ndarray],
        joint_names: Sequence[str],
    ) -> np.ndarray:
        """Return only joint targets for the existing JointTrajectory command path."""

        return self.to_joint_command(action, obs, joint_names).joint_targets

    @staticmethod
    def _validate_action(action: np.ndarray) -> np.ndarray:
        values = np.asarray(action, dtype=np.float64)
        if values.shape != (7,):
            raise ValueError(f'Panda action must have shape [7], got {list(values.shape)}')
        if not np.isfinite(values).all():
            raise ValueError('Panda action must contain finite values')
        return values

    @staticmethod
    def _validate_joint_positions(obs: Dict[str, np.ndarray]) -> np.ndarray:
        if 'joint_positions' not in obs:
            raise ValueError('obs["joint_positions"] is required for Panda action adaptation')
        joint_positions = np.asarray(obs['joint_positions'], dtype=np.float64)
        if joint_positions.ndim != 1:
            raise ValueError('obs["joint_positions"] must be a 1D array')
        if not np.isfinite(joint_positions).all():
            raise ValueError('obs["joint_positions"] must contain finite values')
        return joint_positions

    def _validate_delta_limits(self, action: np.ndarray) -> None:
        max_xyz = float(np.max(np.abs(action[:3])))
        if max_xyz > self._config.max_delta_xyz:
            raise ValueError(
                f'Panda delta_xyz exceeds limit: {max_xyz:.6f} > '
                f'{self._config.max_delta_xyz:.6f}'
            )

        max_rpy = float(np.max(np.abs(action[3:6])))
        if max_rpy > self._config.max_delta_rpy:
            raise ValueError(
                f'Panda delta_rpy exceeds limit: {max_rpy:.6f} > '
                f'{self._config.max_delta_rpy:.6f}'
            )

    def _validate_gripper(self, value: float) -> float:
        gripper = float(value)
        if gripper < self._config.gripper_min or gripper > self._config.gripper_max:
            raise ValueError(
                f'Panda gripper command out of range: {gripper:.6f} not in '
                f'[{self._config.gripper_min:.6f}, {self._config.gripper_max:.6f}]'
            )
        return gripper

    @staticmethod
    def _mock_ik(action: np.ndarray, joint_positions: np.ndarray) -> np.ndarray:
        joint_targets = joint_positions.copy()
        active_dims = min(6, joint_targets.shape[0])
        joint_targets[:active_dims] += action[:active_dims]
        return joint_targets
