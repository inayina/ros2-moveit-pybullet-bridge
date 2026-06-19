"""Domain randomization parameter sampling for Real-Source episodes."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class DomainRandomizationSample:
    """One episode's sampled physics parameters."""

    joint_damping: list[float] = field(default_factory=list)
    joint_friction: list[float] = field(default_factory=list)
    motor_strength: list[float] = field(default_factory=list)
    payload_mass_kg: float = 0.0
    actuator_delay_ms: float = 0.0
    episode_seed: int = 0


@dataclass
class DomainRandomizerConfig:
    random_seed: int = 42
    randomization_strength: float = 1.0
    joint_damping_base: float = 0.04
    joint_friction_base: float = 0.1
    damping_perturb_fraction: float = 0.30
    friction_perturb_fraction: float = 0.50
    motor_strength_range: tuple[float, float] = (0.85, 1.15)
    payload_mass_range: tuple[float, float] = (0.0, 0.5)
    time_delay_range_ms: tuple[float, float] = (0.0, 50.0)


class DomainRandomizer:
    """Sample per-episode physics perturbations for Real-Source."""

    def __init__(self, config: DomainRandomizerConfig) -> None:
        self._config = config
        self._episode_counter = 0
        self._rng = np.random.default_rng(config.random_seed)
        self._latest: DomainRandomizationSample | None = None
        self._inject_shift: _InjectShift | None = None

    @property
    def latest_sample(self) -> DomainRandomizationSample | None:
        return self._latest

    def update_config(self, config: DomainRandomizerConfig) -> None:
        self._config = config

    def reseed(self, seed: int) -> None:
        self._config.random_seed = seed
        self._rng = np.random.default_rng(seed)
        self._episode_counter = 0

    def sample_episode(self, num_joints: int) -> DomainRandomizationSample:
        """Draw a new parameter set for an episode reset."""
        strength = float(np.clip(self._config.randomization_strength, 0.0, 1.0))
        episode_seed = self._config.random_seed + self._episode_counter
        episode_rng = np.random.default_rng(episode_seed)
        self._episode_counter += 1

        damping = self._sample_perturbed(
            episode_rng, num_joints,
            self._config.joint_damping_base,
            self._config.damping_perturb_fraction,
            strength,
        )
        friction = self._sample_perturbed(
            episode_rng, num_joints,
            self._config.joint_friction_base,
            self._config.friction_perturb_fraction,
            strength,
        )

        motor_lo, motor_hi = self._config.motor_strength_range
        motor_center = 0.5 * (motor_lo + motor_hi)
        motor_span = 0.5 * (motor_hi - motor_lo) * strength
        motor = episode_rng.uniform(
            motor_center - motor_span, motor_center + motor_span, num_joints,
        ).tolist()

        payload_lo, payload_hi = self._config.payload_mass_range
        payload = float(episode_rng.uniform(payload_lo, payload_hi) * strength)

        delay_lo, delay_hi = self._config.time_delay_range_ms
        delay_ms = float(episode_rng.uniform(delay_lo, delay_hi) * strength)

        sample = DomainRandomizationSample(
            joint_damping=damping,
            joint_friction=friction,
            motor_strength=motor,
            payload_mass_kg=payload,
            actuator_delay_ms=delay_ms,
            episode_seed=episode_seed,
        )
        self._apply_inject_shift(sample)
        self._latest = sample
        return sample

    def set_inject_shift(
        self,
        parameter_name: str,
        delta_percent: float,
        duration_sec: float,
        now_mono: float,
    ) -> None:
        self._inject_shift = _InjectShift(
            parameter_name=parameter_name,
            delta_percent=delta_percent,
            until_mono=now_mono + duration_sec,
        )

    def inject_shift_expired(self, now_mono: float) -> bool:
        if self._inject_shift is None:
            return False
        if now_mono < self._inject_shift.until_mono:
            return False
        self.clear_inject_shift()
        return True

    def clear_inject_shift(self) -> None:
        self._inject_shift = None

    def _sample_perturbed(
        self,
        rng: np.random.Generator,
        count: int,
        base: float,
        fraction: float,
        strength: float,
    ) -> list[float]:
        if strength <= 0.0 or base <= 0.0:
            return [base] * count
        span = base * fraction * strength
        return rng.uniform(base - span, base + span, count).clip(min=0.0).tolist()

    def _apply_inject_shift(self, sample: DomainRandomizationSample) -> None:
        if self._inject_shift is None:
            return
        shift = self._inject_shift
        scale = 1.0 + shift.delta_percent / 100.0
        name = shift.parameter_name.lower()

        if name in ('joint_damping', 'damping'):
            sample.joint_damping = [v * scale for v in sample.joint_damping]
        elif name in ('joint_friction', 'friction'):
            sample.joint_friction = [v * scale for v in sample.joint_friction]
        elif name in ('motor_strength', 'motor'):
            sample.motor_strength = [v * scale for v in sample.motor_strength]
        elif name in ('payload_mass', 'payload'):
            sample.payload_mass_kg *= scale
        elif name in ('time_delay', 'actuator_delay', 'delay'):
            sample.actuator_delay_ms *= scale


@dataclass
class _InjectShift:
    parameter_name: str
    delta_percent: float
    until_mono: float
