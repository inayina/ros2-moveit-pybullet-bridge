"""Independent Panda absolute EEF8 replay adapter for M5."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Sequence

import numpy as np

from pybullet_bridge.learning.panda_action_adapter import PandaJointCommand


@dataclass(frozen=True)
class PandaAbsoluteEefReplayAdapterConfig:
    """Workspace and execution settings for absolute EEF replay."""

    command_mode: str = 'hold'
    workspace_min: tuple[float, float, float] = (0.20, -0.40, 0.02)
    workspace_max: tuple[float, float, float] = (0.65, 0.40, 0.75)
    quaternion_norm_tolerance: float = 1e-3


class PandaAbsoluteEefReplayAdapter:
    """Convert absolute pose+gripper actions without touching delta semantics."""

    _SUPPORTED_MODES = {'hold', 'pybullet_ik'}

    def __init__(
        self, config: PandaAbsoluteEefReplayAdapterConfig | None = None
    ) -> None:
        self._config = config or PandaAbsoluteEefReplayAdapterConfig()
        if self._config.command_mode not in self._SUPPORTED_MODES:
            raise ValueError('Unsupported Panda absolute command mode')
        self._client_id = None
        if self._config.command_mode == 'pybullet_ik':
            import pybullet as p
            from pybullet_bridge.robot_profiles import resolve_urdf_path
            self._client_id = p.connect(p.DIRECT)
            self._robot_id = p.loadURDF(
                resolve_urdf_path('panda'),
                useFixedBase=True,
                physicsClientId=self._client_id,
            )
            self._ee_link_idx = 6

    def __del__(self) -> None:
        if getattr(self, '_client_id', None) is not None:
            try:
                import pybullet as p
                p.disconnect(self._client_id)
            except Exception:
                pass

    @property
    def command_mode(self) -> str:
        return self._config.command_mode

    def reset(self) -> None:
        """Absolute replay has no temporal compensation state."""

    def to_joint_command(
        self,
        action: np.ndarray,
        obs: Dict[str, np.ndarray],
        joint_names: Sequence[str],
    ) -> PandaJointCommand:
        values = self._validate_action(action)
        joints = self._validate_joints(obs, joint_names)
        if self._config.command_mode == 'hold':
            targets = joints.copy()
        else:
            targets = self._pybullet_ik(values, joints, len(joint_names))
        return PandaJointCommand(
            joint_targets=targets,
            gripper_command=float(values[7]),
            command_mode=self._config.command_mode,
        )

    def to_joint_target(
        self,
        action: np.ndarray,
        obs: Dict[str, np.ndarray],
        joint_names: Sequence[str],
    ) -> np.ndarray:
        return self.to_joint_command(action, obs, joint_names).joint_targets

    def _validate_action(self, action: np.ndarray) -> np.ndarray:
        values = np.asarray(action, dtype=np.float64)
        if values.shape != (8,) or not np.isfinite(values).all():
            raise ValueError('Panda absolute action must be finite shape [8]')
        lower = np.asarray(self._config.workspace_min)
        upper = np.asarray(self._config.workspace_max)
        if np.any(values[:3] < lower) or np.any(values[:3] > upper):
            raise ValueError('Panda absolute position is outside workspace')
        norm = float(np.linalg.norm(values[3:7]))
        if abs(norm - 1.0) > self._config.quaternion_norm_tolerance:
            raise ValueError('Panda absolute quaternion must have unit norm')
        if not 0.0 <= values[7] <= 1.0:
            raise ValueError('Panda absolute gripper must be in [0, 1]')
        return values

    @staticmethod
    def _validate_joints(
        obs: Dict[str, np.ndarray], joint_names: Sequence[str]
    ) -> np.ndarray:
        if 'joint_positions' not in obs:
            raise ValueError('obs["joint_positions"] is required')
        joints = np.asarray(obs['joint_positions'], dtype=np.float64)
        if joints.ndim != 1 or not np.isfinite(joints).all():
            raise ValueError('joint positions must be a finite 1D array')
        if len(joint_names) != len(joints):
            raise ValueError('joint_names length must match joint positions')
        return joints

    def _pybullet_ik(
        self, action: np.ndarray, joints: np.ndarray, joint_count: int
    ) -> np.ndarray:
        import pybullet as p
        for index, value in enumerate(joints[:7]):
            p.resetJointState(
                self._robot_id, index, float(value),
                physicsClientId=self._client_id,
            )
        solution = p.calculateInverseKinematics(
            self._robot_id,
            self._ee_link_idx,
            targetPosition=action[:3].tolist(),
            targetOrientation=action[3:7].tolist(),
            physicsClientId=self._client_id,
        )
        arm = np.asarray(solution[:7], dtype=np.float64)
        if joint_count <= 7:
            return arm[:joint_count]
        gripper = np.full(joint_count - 7, float(action[7]), dtype=np.float64)
        return np.concatenate([arm, gripper])
