"""Timestamp alignment between Sim and Real joint-state streams."""

from __future__ import annotations

import numpy as np

from dist_monitor.sliding_window import SlidingWindow


class TimeAligner:
    """Match Real samples to Sim timestamps via nearest-neighbor search."""

    def __init__(self, tolerance_sec: float = 0.02) -> None:
        self.tolerance = tolerance_sec

    def align_windows(
        self,
        sim_window: SlidingWindow,
        real_window: SlidingWindow,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return aligned (sim, real) arrays using Sim timestamps as reference."""
        sim_ts = sim_window.get_timestamps()
        sim_vals = sim_window.get_samples()
        real_ts = real_window.get_timestamps()
        real_vals = real_window.get_samples()

        if sim_ts.size == 0 or real_ts.size == 0:
            return np.empty((0, 0)), np.empty((0, 0))

        return self.align_arrays(sim_ts, sim_vals, real_ts, real_vals)

    def align_arrays(
        self,
        sim_ts: np.ndarray,
        sim_vals: np.ndarray,
        real_ts: np.ndarray,
        real_vals: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Align sample arrays; unmatched Sim samples are dropped."""
        if sim_ts.size == 0 or real_ts.size == 0:
            return np.empty((0, sim_vals.shape[1] if sim_vals.ndim == 2 else 0)), np.empty(
                (0, real_vals.shape[1] if real_vals.ndim == 2 else 0)
            )

        order = np.argsort(real_ts)
        real_ts_sorted = real_ts[order]
        real_vals_sorted = real_vals[order]

        aligned_sim: list[np.ndarray] = []
        aligned_real: list[np.ndarray] = []

        for t_sim, v_sim in zip(sim_ts, sim_vals):
            idx = int(np.searchsorted(real_ts_sorted, t_sim))
            candidates: list[int] = []
            if idx < len(real_ts_sorted):
                candidates.append(idx)
            if idx > 0:
                candidates.append(idx - 1)

            best_idx = -1
            best_dt = float('inf')
            for c in candidates:
                dt = abs(real_ts_sorted[c] - t_sim)
                if dt < best_dt:
                    best_dt = dt
                    best_idx = c

            if best_idx >= 0 and best_dt < self.tolerance:
                aligned_sim.append(v_sim)
                aligned_real.append(real_vals_sorted[best_idx])

        if not aligned_sim:
            dim = sim_vals.shape[1] if sim_vals.ndim == 2 else 0
            return np.empty((0, dim)), np.empty((0, dim))

        return np.vstack(aligned_sim), np.vstack(aligned_real)
