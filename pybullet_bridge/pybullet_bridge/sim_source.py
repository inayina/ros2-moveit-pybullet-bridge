"""Sim-Source: ideal PyBullet instance with nominal physics parameters."""

from __future__ import annotations

import os
from dataclasses import dataclass, field


@dataclass
class SimSourceConfig:
    urdf_path: str = ''
    use_gui: bool = False
    physics_frequency: float = 240.0
    home_positions: list[float] = field(default_factory=lambda: [0.0, 0.0])


@dataclass
class JointStateSnapshot:
    names: list[str] = field(default_factory=list)
    positions: list[float] = field(default_factory=list)
    velocities: list[float] = field(default_factory=list)
    efforts: list[float] = field(default_factory=list)


class SimSource:
    """PyBullet simulation instance: load URDF, apply position control, read joint state."""

    def __init__(self, config: SimSourceConfig) -> None:
        self._config = config
        self._initialized = False
        self._client_id: int | None = None
        self._robot_id: int | None = None
        self._joint_indices: list[int] = []
        self._joint_names: list[str] = []
        self._effort_limits: list[float] = []
        self._target_positions: list[float] = list(config.home_positions)
        self._p = None

    def initialize(self) -> bool:
        try:
            import pybullet as p
            import pybullet_data
        except ImportError:
            return False

        if not self._config.urdf_path or not os.path.isfile(self._config.urdf_path):
            return False

        self._p = p
        mode = p.GUI if self._config.use_gui else p.DIRECT
        self._client_id = p.connect(mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation(physicsClientId=self._client_id)
        p.setGravity(0, 0, -9.81, physicsClientId=self._client_id)
        p.setTimeStep(1.0 / self._config.physics_frequency, physicsClientId=self._client_id)
        p.setRealTimeSimulation(0, physicsClientId=self._client_id)

        self._robot_id = p.loadURDF(
            self._config.urdf_path,
            basePosition=[0.0, 0.0, 0.0],
            baseOrientation=[0.0, 0.0, 0.0, 1.0],
            useFixedBase=True,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self._client_id,
        )

        self._discover_joints()
        if not self._joint_indices:
            return False

        self.reset()
        self._initialized = True
        return True

    def _discover_joints(self) -> None:
        assert self._p is not None and self._robot_id is not None
        p = self._p

        self._joint_indices.clear()
        self._joint_names.clear()
        self._effort_limits.clear()

        for idx in range(p.getNumJoints(self._robot_id, physicsClientId=self._client_id)):
            info = p.getJointInfo(self._robot_id, idx, physicsClientId=self._client_id)
            joint_type = info[2]
            if joint_type not in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                continue
            self._joint_indices.append(idx)
            self._joint_names.append(info[1].decode('utf-8'))
            self._effort_limits.append(float(info[10]) if info[10] > 0 else 100.0)

        if len(self._target_positions) != len(self._joint_indices):
            self._target_positions = [0.0] * len(self._joint_indices)

    @property
    def ready(self) -> bool:
        return self._initialized

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    def set_position_targets(self, positions: list[float]) -> None:
        if len(positions) != len(self._joint_indices):
            raise ValueError(
                f'Expected {len(self._joint_indices)} positions, got {len(positions)}')
        self._target_positions = list(positions)

    def set_position_targets_by_name(self, targets: dict[str, float]) -> None:
        updated = list(self._target_positions)
        for i, name in enumerate(self._joint_names):
            if name in targets:
                updated[i] = targets[name]
        self._target_positions = updated

    def reset(self, positions: list[float] | None = None) -> None:
        if not self._initialized or self._p is None or self._robot_id is None:
            return

        p = self._p
        home = positions if positions is not None else self._config.home_positions
        if len(home) != len(self._joint_indices):
            home = [0.0] * len(self._joint_indices)

        for idx, pos in zip(self._joint_indices, home):
            p.resetJointState(
                self._robot_id, idx, targetValue=pos, targetVelocity=0.0,
                physicsClientId=self._client_id)

        self._target_positions = list(home)
        self._apply_position_control()

    def step(self) -> JointStateSnapshot:
        if not self._initialized or self._p is None or self._robot_id is None:
            return JointStateSnapshot()

        p = self._p
        self._apply_position_control()
        p.stepSimulation(physicsClientId=self._client_id)
        return self.read_state()

    def read_state(self) -> JointStateSnapshot:
        if not self._initialized or self._p is None or self._robot_id is None:
            return JointStateSnapshot()

        p = self._p
        positions: list[float] = []
        velocities: list[float] = []
        efforts: list[float] = []

        for idx in self._joint_indices:
            state = p.getJointState(
                self._robot_id, idx, physicsClientId=self._client_id)
            positions.append(float(state[0]))
            velocities.append(float(state[1]))
            efforts.append(float(state[3]))

        return JointStateSnapshot(
            names=list(self._joint_names),
            positions=positions,
            velocities=velocities,
            efforts=efforts,
        )

    def _apply_position_control(self) -> None:
        assert self._p is not None and self._robot_id is not None
        p = self._p

        p.setJointMotorControlArray(
            bodyUniqueId=self._robot_id,
            jointIndices=self._joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=self._target_positions,
            forces=self._effort_limits,
            physicsClientId=self._client_id,
        )

    def shutdown(self) -> None:
        if self._p is not None and self._client_id is not None:
            self._p.disconnect(self._client_id)
        self._client_id = None
        self._robot_id = None
        self._initialized = False
        self._p = None
