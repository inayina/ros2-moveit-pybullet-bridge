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
    gripper_tolerance: float = 1e-6

    # Deadband, backlash, and joint limit constraints
    enable_deadband: bool = False
    deadband_val: float = 0.0001
    deadband_feedforward: float = 0.0002
    enable_backlash: bool = False
    backlash_val: float = 0.001
    enable_limits: bool = False
    max_joint_velocity: float = 2.0
    max_joint_acceleration: float = 10.0
    control_loop_dt: float = 0.02


class PandaActionAdapter:
    """Convert Panda ee_delta_gripper[7] actions into bridge joint targets.

    ``hold`` validates the action and keeps the current joint state. ``mock_ik`` is
    an offline-safe shim that maps small task-space deltas onto the first joints,
    making the PolicyRunner path testable before a real Panda IK backend exists.
    """

    _SUPPORTED_MODES = {'hold', 'mock_ik', 'pybullet_ik'}

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

        self._client_id: int | None = None
        if self._config.command_mode == 'pybullet_ik':
            import pybullet as p
            from pybullet_bridge.robot_profiles import resolve_urdf_path
            self._client_id = p.connect(p.DIRECT)
            urdf_path = resolve_urdf_path('panda')
            self._robot_id = p.loadURDF(urdf_path, useFixedBase=True, physicsClientId=self._client_id)
            self._ee_link_idx = 6  # panda_link7 is index 6

        self._prev_targets: np.ndarray | None = None
        self._prev_velocities: np.ndarray | None = None

    def __del__(self) -> None:
        if hasattr(self, '_client_id') and self._client_id is not None:
            try:
                import pybullet as p
                p.disconnect(self._client_id)
            except Exception:
                pass

    @property
    def command_mode(self) -> str:
        return self._config.command_mode

    def reset(self) -> None:
        """Reset historical joint commands to clear deadband, backlash, and limit states."""
        self._prev_targets = None
        self._prev_velocities = None

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

        arm_positions = joint_positions[:7]

        if self._config.command_mode == 'hold':
            arm_targets = arm_positions.copy()
        elif self._config.command_mode == 'mock_ik':
            arm_targets = self._mock_ik(action, arm_positions)
        elif self._config.command_mode == 'pybullet_ik':
            arm_targets = self._pybullet_ik(action, arm_positions)
        else:
            raise ValueError(f'Unsupported command mode: {self._config.command_mode}')

        if len(joint_names) > 7:
            num_gripper_joints = len(joint_names) - 7
            gripper_targets = np.full(num_gripper_joints, gripper_command, dtype=np.float64)
            joint_targets = np.concatenate([arm_targets, gripper_targets])
        else:
            joint_targets = arm_targets[:len(joint_names)]

        # Apply deadband, backlash, and velocity/acceleration limits
        targets_post_db = self._apply_deadband(joint_targets, joint_positions, self._prev_targets)
        targets_post_bl = self._apply_backlash(targets_post_db, self._prev_targets)
        final_targets, final_velocities = self._apply_limits(
            targets_post_bl,
            self._prev_targets,
            self._prev_velocities,
        )

        self._prev_targets = final_targets.copy()
        self._prev_velocities = final_velocities.copy()

        return PandaJointCommand(
            joint_targets=final_targets,
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

    def _apply_deadband(
        self,
        joint_targets: np.ndarray,
        joint_positions: np.ndarray,
        prev_targets: np.ndarray | None,
    ) -> np.ndarray:
        if not self._config.enable_deadband:
            return joint_targets

        output_targets = joint_targets.copy()
        for i in range(joint_targets.shape[0]):
            error = joint_targets[i] - joint_positions[i]
            if abs(error) < self._config.deadband_val:
                output_targets[i] = prev_targets[i] if prev_targets is not None else joint_positions[i]
            else:
                comp = self._config.deadband_feedforward
                output_targets[i] = joint_targets[i] + (comp if error > 0 else -comp)
        return output_targets

    def _apply_backlash(
        self,
        joint_targets: np.ndarray,
        prev_targets: np.ndarray | None,
    ) -> np.ndarray:
        if not self._config.enable_backlash or prev_targets is None:
            return joint_targets

        output_targets = joint_targets.copy()
        b = self._config.backlash_val
        for i in range(joint_targets.shape[0]):
            u = joint_targets[i]
            y_prev = prev_targets[i]
            if u > y_prev + b:
                output_targets[i] = u - b
            elif u < y_prev - b:
                output_targets[i] = u + b
            else:
                output_targets[i] = y_prev
        return output_targets

    def _apply_limits(
        self,
        joint_targets: np.ndarray,
        prev_targets: np.ndarray | None,
        prev_velocities: np.ndarray | None,
    ) -> tuple[np.ndarray, np.ndarray]:
        dt = self._config.control_loop_dt
        if not self._config.enable_limits or prev_targets is None:
            curr_targets = joint_targets.copy()
            curr_velocities = np.zeros_like(curr_targets)
            if prev_targets is not None:
                curr_velocities = (curr_targets - prev_targets) / dt
            return curr_targets, curr_velocities

        v_max = self._config.max_joint_velocity
        a_max = self._config.max_joint_acceleration

        curr_targets = joint_targets.copy()
        curr_velocities = np.zeros_like(curr_targets)

        for i in range(joint_targets.shape[0]):
            q = joint_targets[i]
            q_prev = prev_targets[i]
            v = (q - q_prev) / dt

            # Velocity limit
            if abs(v) > v_max:
                v = np.sign(v) * v_max
                q = q_prev + v * dt

            # Acceleration limit
            if prev_velocities is not None:
                v_prev = prev_velocities[i]
                a = (v - v_prev) / dt
                if abs(a) > a_max:
                    a = np.sign(a) * a_max
                    v = v_prev + a * dt
                    q = q_prev + v * dt

            curr_targets[i] = q
            curr_velocities[i] = v

        return curr_targets, curr_velocities

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
        lower = self._config.gripper_min
        upper = self._config.gripper_max
        tolerance = max(0.0, float(self._config.gripper_tolerance))
        if gripper < lower - tolerance or gripper > upper + tolerance:
            raise ValueError(
                f'Panda gripper command out of range: {gripper:.6f} not in '
                f'[{lower:.6f}, {upper:.6f}]'
            )
        return float(np.clip(gripper, lower, upper))

    @staticmethod
    def _mock_ik(action: np.ndarray, joint_positions: np.ndarray) -> np.ndarray:
        joint_targets = joint_positions.copy()
        active_dims = min(6, joint_targets.shape[0])
        joint_targets[:active_dims] += action[:active_dims]
        return joint_targets

    def _pybullet_ik(self, action: np.ndarray, joint_positions: np.ndarray) -> np.ndarray:
        import pybullet as p

        for i in range(min(7, len(joint_positions))):
            p.resetJointState(self._robot_id, i, joint_positions[i], physicsClientId=self._client_id)

        state = p.getLinkState(self._robot_id, self._ee_link_idx, physicsClientId=self._client_id)
        current_pos = np.array(state[0], dtype=np.float64)
        current_orn = np.array(state[1], dtype=np.float64)

        target_pos = current_pos + action[:3]

        delta_rpy = action[3:6]
        delta_orn = p.getQuaternionFromEuler(delta_rpy.tolist())
        target_orn = p.multiplyTransforms(
            [0.0, 0.0, 0.0], current_orn.tolist(),
            [0.0, 0.0, 0.0], delta_orn,
            physicsClientId=self._client_id
        )[1]

        joint_targets = p.calculateInverseKinematics(
            self._robot_id,
            self._ee_link_idx,
            targetPosition=target_pos.tolist(),
            targetOrientation=target_orn,
            physicsClientId=self._client_id,
        )

        return np.array(joint_targets[:7], dtype=np.float64)
