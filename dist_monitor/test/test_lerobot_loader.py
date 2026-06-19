"""Unit tests for LeRobot dataset loader."""

import json
from pathlib import Path

import numpy as np
import pytest

pyarrow = pytest.importorskip('pyarrow')
import pyarrow as pa
import pyarrow.parquet as pq

from dist_monitor.lerobot_loader import load_lerobot_dataset


def _write_minimal_lerobot_dataset(root: Path, n: int = 50) -> None:
    meta = root / 'meta'
    data = root / 'data' / 'chunk-000'
    meta.mkdir(parents=True)
    data.mkdir(parents=True)

    info = {
        'fps': 100.0,
        'features': {
            'observation.state': {'dtype': 'float32', 'shape': [2]},
        },
    }
    (meta / 'info.json').write_text(json.dumps(info), encoding='utf-8')

    ts = np.arange(n, dtype=float) / 100.0
    states = np.stack([np.sin(ts), np.cos(ts)], axis=1)
    table = pa.table({
        'timestamp': ts,
        'observation.state': [row.tolist() for row in states],
    })
    pq.write_table(table, data / 'episode_000000.parquet')


def test_load_lerobot_dataset(tmp_path):
    _write_minimal_lerobot_dataset(tmp_path)
    traj = load_lerobot_dataset(tmp_path)
    assert len(traj.timestamps) == 50
    assert traj.positions.shape == (50, 2)
    assert traj.velocities.shape == (50, 2)
    assert traj.joint_names == ['joint_1', 'joint_2']


def test_lerobot_lookup_nearest(tmp_path):
    _write_minimal_lerobot_dataset(tmp_path, n=10)
    traj = load_lerobot_dataset(tmp_path)
    pos = traj.lookup_nearest_position(0.05, tolerance_sec=0.02)
    assert pos is not None
    assert pos.shape == (2,)
