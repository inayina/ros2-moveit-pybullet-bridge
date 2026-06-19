"""Real-Source: domain-randomized PyBullet instance simulating virtual real world."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from pybullet_bridge.actuator_delay import ActuatorDelayBuffer
from pybullet_bridge.domain_randomizer import (
    DomainRandomizationSample,
    DomainRandomizer,
    DomainRandomizerConfig,
)
from pybullet_bridge.sim_source import JointStateSnapshot, SimSource, SimSourceConfig


@dataclass
class RealSourceConfig(SimSourceConfig):
    randomization_strength: float = 1.0
    joint_damping_base: float = 0.04
    joint_friction_base: float = 0.1
    damping_perturb_fraction: float = 0.30
    friction_perturb_fraction: float = 0.50
    motor_strength_range: tuple[float, float] = (0.85, 1.15)
    payload_mass_range: tuple[float, float] = (0.0, 0.5)
    time_delay_range_ms: tuple[float, float] = (0.0, 50.0)
    random_seed: int = 42
    end_effector_link: str = 'tool0'


class RealSource:
    """Separate PyBullet DIRECT instance with per-episode domain randomization."""

    def __init__(
        self,
        config: RealSourceConfig,
        log_fn: Callable[[str], None] | None = None,
    ) -> None:
        self._config = config
        self._log = log_fn or (lambda _msg: None)
        self._sim = SimSource(SimSourceConfig(
            urdf_path=config.urdf_path,
            use_gui=False,
            physics_frequency=config.physics_frequency,
            home_positions=config.home_positions,
        ))
        self._randomizer = DomainRandomizer(DomainRandomizerConfig(
            random_seed=config.random_seed,
            randomization_strength=config.randomization_strength,
            joint_damping_base=config.joint_damping_base,
            joint_friction_base=config.joint_friction_base,
            damping_perturb_fraction=config.damping_perturb_fraction,
            friction_perturb_fraction=config.friction_perturb_fraction,
            motor_strength_range=config.motor_strength_range,
            payload_mass_range=config.payload_mass_range,
            time_delay_range_ms=config.time_delay_range_ms,
        ))
        self._delay_buffer = ActuatorDelayBuffer()
        self._latest_sample: DomainRandomizationSample | None = None
        self._base_ee_mass: float = 0.0
        self._ee_link_index: int | None = None
        self._last_execution_sim_time: float = 0.0
        self._pending_targets: dict[str, float] = {}

    def initialize(self) -> bool:
        if not self._sim.initialize():
            return False

        self._ee_link_index = self._sim.get_link_index_by_name(self._config.end_effector_link)
        if self._ee_link_index is not None:
            self._base_ee_mass = self._sim.get_link_mass(self._ee_link_index)

        self._resample_and_apply(log=True)
        return True

    @property
    def ready(self) -> bool:
        return self._sim.ready

    @property
    def joint_names(self) -> list[str]:
        return self._sim.joint_names

    @property
    def latest_sample(self) -> DomainRandomizationSample | None:
        return self._latest_sample

    @property
    def last_execution_sim_time(self) -> float:
        return self._last_execution_sim_time

    @property
    def randomizer(self) -> DomainRandomizer:
        return self._randomizer

    def set_position_targets_by_name(self, targets: dict[str, float]) -> None:
        self._pending_targets = dict(targets)

    def reset(self) -> None:
        self._sim.reset()
        self._delay_buffer.clear()
        self._resample_and_apply(log=True)

    def step(self, sim_time_sec: float) -> JointStateSnapshot:
        self._delay_buffer.push(sim_time_sec, self._pending_targets)
        delayed_targets, exec_time = self._delay_buffer.sample(sim_time_sec)
        self._last_execution_sim_time = exec_time
        self._sim.set_position_targets_by_name(delayed_targets)
        return self._sim.step()

    def read_state(self) -> JointStateSnapshot:
        return self._sim.read_state()

    def update_randomization_config(self, config: DomainRandomizerConfig) -> None:
        self._randomizer.update_config(config)

    def inject_shift(
        self,
        parameter_name: str,
        delta_percent: float,
        duration_sec: float,
        now_mono: float,
    ) -> None:
        self._randomizer.set_inject_shift(parameter_name, delta_percent, duration_sec, now_mono)

    def tick_inject_shift(self, now_mono: float) -> None:
        if self._randomizer.inject_shift_expired(now_mono):
            self._resample_and_apply(log=True)

    def resample_episode(self) -> None:
        """Public entry to resample physics parameters (e.g. after inject_shift)."""
        self._resample_and_apply(log=True)

    def _resample_and_apply(self, log: bool = False) -> None:
        num_joints = len(self._sim.joint_indices)
        sample = self._randomizer.sample_episode(num_joints)
        self._latest_sample = sample
        self._apply_sample(sample)
        if log:
            self._log(
                f'Real-Source episode randomization (seed={sample.episode_seed}): '
                f'damping={[round(v, 4) for v in sample.joint_damping]} '
                f'friction={[round(v, 4) for v in sample.joint_friction]} '
                f'motor={[round(v, 3) for v in sample.motor_strength]} '
                f'payload={sample.payload_mass_kg:.3f}kg '
                f'delay={sample.actuator_delay_ms:.1f}ms',
            )

    def _apply_sample(self, sample: DomainRandomizationSample) -> None:
        joint_indices = self._sim.joint_indices
        for idx, joint_idx in enumerate(joint_indices):
            self._sim.set_joint_dynamics(
                joint_idx,
                sample.joint_damping[idx],
                sample.joint_friction[idx],
            )

        if len(sample.motor_strength) == len(joint_indices):
            self._sim.set_motor_strength_scale(sample.motor_strength)

        if self._ee_link_index is not None:
            self._sim.set_link_mass(
                self._ee_link_index,
                self._base_ee_mass + sample.payload_mass_kg,
            )

        self._delay_buffer.set_delay(sample.actuator_delay_ms / 1000.0)

    def shutdown(self) -> None:
        self._sim.shutdown()
