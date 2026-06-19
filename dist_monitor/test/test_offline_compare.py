"""Unit tests for offline comparison CLI."""

import json
from pathlib import Path

import numpy as np
import pytest

pyarrow = pytest.importorskip('pyarrow')
import pyarrow as pa
import pyarrow.parquet as pq

from dist_monitor.lerobot_loader import load_lerobot_dataset
from dist_monitor.offline_compare import compare_offline


def _write_dataset(root: Path, states: np.ndarray, fps: float = 100.0) -> None:
    meta = root / 'meta'
    data = root / 'data' / 'chunk-000'
    meta.mkdir(parents=True)
    data.mkdir(parents=True)
    (meta / 'info.json').write_text(
        json.dumps({
            'fps': fps,
            'features': {'observation.state': {'dtype': 'float32', 'shape': [states.shape[1]]}},
        }),
        encoding='utf-8',
    )
    ts = np.arange(len(states), dtype=float) / fps
    table = pa.table({
        'timestamp': ts,
        'observation.state': [row.tolist() for row in states],
    })
    pq.write_table(table, data / 'episode_000000.parquet')


def test_offline_compare_identical(tmp_path):
    n = 120
    ts = np.linspace(0.0, 1.2, n)
    states = np.hstack([
        np.sin(ts)[:, None],
        np.cos(ts)[:, None],
        np.zeros((n, 2)),
        np.zeros((n, 2)),
    ])

    real_root = tmp_path / 'real'
    sim_root = tmp_path / 'sim'
    pos_vel = states[:, :2]
    full = np.hstack([pos_vel, np.gradient(pos_vel, ts, axis=0)])
    _write_dataset(real_root, pos_vel)
    _write_dataset(sim_root, pos_vel)

    real_traj = load_lerobot_dataset(real_root)
    sim_traj = load_lerobot_dataset(sim_root)
    report = compare_offline(
        sim_traj.timestamps,
        sim_traj.full_state(),
        real_traj.timestamps,
        real_traj.full_state(),
    )
    assert report['sample_count'] >= 50
    assert report['kl_divergence_mean'] < 0.05
    assert report['shift_detected'] is False
