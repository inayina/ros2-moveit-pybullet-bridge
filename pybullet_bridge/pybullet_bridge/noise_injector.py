"""Domain randomization noise injection for Real-Source simulation."""

from __future__ import annotations

import random
from dataclasses import dataclass

import numpy as np


@dataclass
class NoiseConfig:
    position_noise_std: float = 0.01
    velocity_noise_std: float = 0.05
    random_seed: int = 42


class NoiseInjector:
    """Apply sensor noise to joint state readings."""

    def __init__(self, config: NoiseConfig) -> None:
        self._config = config
        self._rng = np.random.default_rng(config.random_seed)

    def apply(
        self,
        positions: list[float],
        velocities: list[float],
    ) -> tuple[list[float], list[float]]:
        pos_noise = self._rng.normal(0.0, self._config.position_noise_std, len(positions))
        vel_noise = self._rng.normal(0.0, self._config.velocity_noise_std, len(velocities))
        noisy_pos = (np.asarray(positions) + pos_noise).tolist()
        noisy_vel = (np.asarray(velocities) + vel_noise).tolist()
        return noisy_pos, noisy_vel

    def reseed(self, seed: int) -> None:
        self._rng = np.random.default_rng(seed)
