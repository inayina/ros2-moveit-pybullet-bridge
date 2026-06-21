"""Minimal replay trajectory for planar 2-DOF policy benchmarks."""

from __future__ import annotations

import pickle
from pathlib import Path

import numpy as np


def write_planar_replay_fixture(path: Path, steps: int = 40) -> Path:
    positions = np.zeros((steps, 2), dtype=np.float64)
    for idx in range(steps):
        phase = 2.0 * np.pi * idx / max(steps - 1, 1)
        positions[idx, 0] = 0.3 * np.sin(phase)
        positions[idx, 1] = 0.2 * np.cos(phase)

    payload = {
        'joint_names': ['joint1', 'joint2'],
        'positions': positions,
        'timestamps': np.arange(steps, dtype=np.float64) * 0.02,
        'metadata': {'source': 'benchmark_fixture'},
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('wb') as handle:
        pickle.dump(payload, handle)
    return path


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    target = root / 'test' / 'fixtures' / 'planar_2dof_replay.pkl'
    write_planar_replay_fixture(target)
    print(f'Wrote {target}')
