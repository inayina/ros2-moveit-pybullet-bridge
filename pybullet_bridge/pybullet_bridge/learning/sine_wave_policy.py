"""Closed-loop sine-wave perturbation policy."""

from typing import Dict, Optional

import numpy as np

from pybullet_bridge.learning.base_policy import BasePolicy


class SineWavePolicy(BasePolicy):
    """Add deterministic sine perturbations to the current joint positions."""

    def __init__(
        self,
        amplitude: float = 0.05,
        frequency_hz: float = 0.2,
        inference_freq: int = 50,
        seed: int = 0,
    ) -> None:
        if amplitude < 0.0:
            raise ValueError('amplitude must be non-negative')
        if frequency_hz < 0.0:
            raise ValueError('frequency_hz must be non-negative')
        if inference_freq <= 0:
            raise ValueError('inference_freq must be positive')

        self._amplitude = float(amplitude)
        self._frequency_hz = float(frequency_hz)
        self._inference_freq = int(inference_freq)
        self._seed = int(seed)
        self._time_sec = 0.0
        self._phase: Optional[np.ndarray] = None
        self._rng = np.random.default_rng(self._seed)

    @property
    def inference_freq(self) -> int:
        return self._inference_freq

    def reset(self) -> None:
        self._time_sec = 0.0
        self._phase = None
        self._rng = np.random.default_rng(self._seed)

    def get_action(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        if 'joint_positions' not in obs:
            raise ValueError('obs must contain joint_positions')

        joint_positions = np.asarray(obs['joint_positions'], dtype=np.float64)
        if joint_positions.ndim != 1:
            raise ValueError('obs["joint_positions"] must be a 1D array')

        phase = self._phase_for_dimension(joint_positions.shape[0])
        perturbation = self._amplitude * np.sin(
            2.0 * np.pi * self._frequency_hz * self._time_sec + phase
        )
        self._time_sec += 1.0 / self._inference_freq

        return joint_positions.copy() + perturbation

    def _phase_for_dimension(self, joint_count: int) -> np.ndarray:
        if joint_count == 0:
            raise ValueError('joint_positions must contain at least one joint')
        if self._phase is None:
            self._phase = self._rng.uniform(0.0, 2.0 * np.pi, size=joint_count)
        elif self._phase.shape[0] != joint_count:
            raise ValueError('joint dimension changed after policy initialization')
        return self._phase
