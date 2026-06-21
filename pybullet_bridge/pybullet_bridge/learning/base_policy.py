"""Policy abstraction for robot joint-command generation."""

from abc import ABC, abstractmethod
from typing import Dict

import numpy as np


class BasePolicy(ABC):
    """Base interface for policies consumed by the PolicyRunner."""

    @property
    @abstractmethod
    def inference_freq(self) -> int:
        """Recommended inference frequency in Hz."""

    @abstractmethod
    def reset(self) -> None:
        """Reset internal policy state at an episode or lifecycle boundary."""

    @abstractmethod
    def get_action(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        """Return target joint positions for the current observation."""
