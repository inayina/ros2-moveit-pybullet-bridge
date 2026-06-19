"""Real-Source: domain-randomized PyBullet instance simulating virtual real world."""

from __future__ import annotations

from dataclasses import dataclass

from pybullet_bridge.noise_injector import NoiseConfig, NoiseInjector
from pybullet_bridge.sim_source import JointStateSnapshot, SimSource, SimSourceConfig


@dataclass
class RealSourceConfig(SimSourceConfig):
    randomization_strength: float = 0.5
    joint_damping_range: tuple[float, float] = (0.0, 0.5)
    joint_friction_range: tuple[float, float] = (0.0, 0.3)
    motor_strength_range: tuple[float, float] = (0.85, 1.15)
    random_seed: int = 42
    position_noise_std: float = 0.01
    velocity_noise_std: float = 0.05


class RealSource:
    """Separate PyBullet instance with sensor noise (M3: domain randomization)."""

    def __init__(self, config: RealSourceConfig) -> None:
        self._config = config
        self._sim = SimSource(SimSourceConfig(
            urdf_path=config.urdf_path,
            use_gui=False,
            physics_frequency=config.physics_frequency,
            home_positions=config.home_positions,
        ))
        self._noise = NoiseInjector(NoiseConfig(
            random_seed=config.random_seed,
            position_noise_std=config.position_noise_std,
            velocity_noise_std=config.velocity_noise_std,
        ))

    def initialize(self) -> bool:
        return self._sim.initialize()

    @property
    def ready(self) -> bool:
        return self._sim.ready

    @property
    def joint_names(self) -> list[str]:
        return self._sim.joint_names

    def set_position_targets_by_name(self, targets: dict[str, float]) -> None:
        self._sim.set_position_targets_by_name(targets)

    def reset(self) -> None:
        self._sim.reset()

    def step(self) -> JointStateSnapshot:
        snapshot = self._sim.step()
        return self._apply_noise(snapshot)

    def read_state(self) -> JointStateSnapshot:
        return self._apply_noise(self._sim.read_state())

    def _apply_noise(self, snapshot: JointStateSnapshot) -> JointStateSnapshot:
        if not snapshot.names:
            return snapshot

        noisy_pos, noisy_vel = self._noise.apply(snapshot.positions, snapshot.velocities)
        return JointStateSnapshot(
            names=snapshot.names,
            positions=noisy_pos,
            velocities=noisy_vel,
            efforts=snapshot.efforts,
        )

    def shutdown(self) -> None:
        self._sim.shutdown()
