"""Dynamics anomaly scoring from velocity jumps and tracking-error spikes."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class DynamicsAnomalyConfig:
    velocity_jump_threshold: float = 2.0
    error_spike_sigma: float = 3.0


@dataclass
class DynamicsAnomalyResult:
    score: float = 0.0
    velocity_jump_per_joint: list[float] = field(default_factory=list)


def compute_dynamics_anomaly(
    sim_states: np.ndarray,
    real_states: np.ndarray,
    errors: np.ndarray,
    baseline_errors: np.ndarray | None,
    cfg: DynamicsAnomalyConfig | None = None,
) -> DynamicsAnomalyResult:
    """Score dynamics anomalies in [0, 1] from aligned sim/real windows.

    sim_states / real_states: (N, 2*n_dof) with [pos, vel] per row.
    errors: (N, n_dof) position tracking error.
    """
    cfg = cfg or DynamicsAnomalyConfig()
    result = DynamicsAnomalyResult()

    if sim_states.size == 0 or errors.size == 0:
        return result

    n_dof = errors.shape[1]
    sim_vel = sim_states[:, n_dof: 2 * n_dof]
    real_vel = real_states[:, n_dof: 2 * n_dof]

    jump_scores: list[float] = []
    if len(sim_vel) >= 2:
        sim_dvel = np.abs(np.diff(sim_vel, axis=0))
        real_dvel = np.abs(np.diff(real_vel, axis=0))
        per_joint = np.maximum(sim_dvel.max(axis=0), real_dvel.max(axis=0))
        thresh = max(cfg.velocity_jump_threshold, 1e-6)
        jump_scores = [float(v) for v in per_joint]
        jump_norm = [min(v / thresh, 1.0) for v in per_joint]
        jump_score = float(max(jump_norm)) if jump_norm else 0.0
    else:
        jump_score = 0.0
        jump_scores = [0.0] * n_dof

    spike_score = 0.0
    if baseline_errors is not None and len(baseline_errors) > 0 and len(errors) > 0:
        baseline_std = np.std(baseline_errors, axis=0)
        baseline_std = np.maximum(baseline_std, 1e-4)
        latest = np.abs(errors[-1])
        spike_ratios = latest / (cfg.error_spike_sigma * baseline_std)
        spike_score = float(np.clip(np.max(spike_ratios), 0.0, 1.0))

    result.velocity_jump_per_joint = jump_scores
    result.score = float(max(jump_score, spike_score))
    return result
