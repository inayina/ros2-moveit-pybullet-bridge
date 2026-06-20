"""Sim-Source: ideal PyBullet instance with nominal physics parameters."""

from __future__ import annotations

import os
from dataclasses import dataclass, field


@dataclass
class SimSourceConfig:
    urdf_path: str = ''
    use_gui: bool = False
    physics_frequency: float = 240.0
    home_positions: list[float] = field(default_factory=lambda: [0.8, -0.6])


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
        self._add_ur_mesh_search_paths(p)
        urdf_dir = os.path.dirname(os.path.abspath(self._config.urdf_path))
        if os.path.isdir(urdf_dir):
            p.setAdditionalSearchPath(urdf_dir, physicsClientId=self._client_id)
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

        self._initialized = True
        self.reset()
        return True

    @staticmethod
    def _add_ur_mesh_search_paths(p) -> None:
        try:
            from ament_index_python.packages import get_package_share_directory

            ur_share = get_package_share_directory('ur_description')
            for variant in ('ur5', 'ur5e', 'ur10', 'ur3'):
                for sub in ('visual', 'collision'):
                    mesh_dir = os.path.join(ur_share, 'meshes', variant, sub)
                    if os.path.isdir(mesh_dir):
                        p.setAdditionalSearchPath(mesh_dir)
        except Exception:
            pass

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

    @property
    def joint_indices(self) -> list[int]:
        return list(self._joint_indices)

    @property
    def client_id(self) -> int | None:
        return self._client_id

    @property
    def robot_id(self) -> int | None:
        return self._robot_id

    def get_link_index_by_name(self, link_name: str) -> int | None:
        if not self._initialized or self._p is None or self._robot_id is None:
            return None
        p = self._p
        for idx in range(p.getNumJoints(self._robot_id, physicsClientId=self._client_id)):
            info = p.getJointInfo(self._robot_id, idx, physicsClientId=self._client_id)
            if info[12].decode('utf-8') == link_name:
                return idx
        if link_name == p.getBodyInfo(self._robot_id, physicsClientId=self._client_id)[0].decode('utf-8'):
            return -1
        return None

    def get_link_mass(self, link_index: int) -> float:
        if not self._initialized or self._p is None or self._robot_id is None:
            return 0.0
        dynamics = self._p.getDynamicsInfo(
            self._robot_id, link_index, physicsClientId=self._client_id)
        return float(dynamics[0])

    def set_joint_dynamics(
        self,
        joint_index: int,
        damping: float,
        friction: float,
    ) -> None:
        if not self._initialized or self._p is None or self._robot_id is None:
            return
        self._p.changeDynamics(
            self._robot_id,
            joint_index,
            jointDamping=float(damping),
            physicsClientId=self._client_id,
        )

    def set_link_mass(self, link_index: int, mass: float) -> None:
        if not self._initialized or self._p is None or self._robot_id is None:
            return
        self._p.changeDynamics(
            self._robot_id,
            link_index,
            mass=float(mass),
            physicsClientId=self._client_id,
        )

    def set_motor_strength_scale(self, scales: list[float]) -> None:
        if len(scales) != len(self._effort_limits):
            raise ValueError(
                f'Expected {len(self._effort_limits)} motor scales, got {len(scales)}')
        base_limits = []
        for idx in range(len(self._joint_indices)):
            info = self._p.getJointInfo(
                self._robot_id, self._joint_indices[idx], physicsClientId=self._client_id)
            base_limits.append(float(info[10]) if info[10] > 0 else 100.0)
        self._effort_limits = [
            base * scale for base, scale in zip(base_limits, scales)
        ]

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

    def capture_camera_jpeg(self, width: int = 640, height: int = 480) -> bytes | None:
        if not self._initialized or self._p is None or self._client_id is None:
            return None
        from pybullet_bridge.sim_camera import capture_jpeg
        return capture_jpeg(self._p, self._client_id, width=width, height=height)

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
