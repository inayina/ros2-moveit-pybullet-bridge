"""Load joint trajectories from LeRobot-format datasets."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from dist_monitor.joint_names import normalize_joint_names


@dataclass
class LeRobotTrajectory:
    """Time-series joint data extracted from a LeRobot dataset."""

    timestamps: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    joint_names: list[str]
    fps: float

    def full_state(self) -> np.ndarray:
        """Return [position, velocity] concatenated per row."""
        if self.velocities.size == 0:
            return self.positions
        return np.hstack([self.positions, self.velocities])

    def lookup_nearest(self, timestamp: float, tolerance_sec: float = 0.02) -> np.ndarray | None:
        if self.timestamps.size == 0:
            return None
        idx = int(np.argmin(np.abs(self.timestamps - timestamp)))
        if abs(self.timestamps[idx] - timestamp) > tolerance_sec:
            return None
        return self.full_state()[idx]

    def lookup_nearest_position(self, timestamp: float, tolerance_sec: float = 0.02) -> np.ndarray | None:
        if self.timestamps.size == 0:
            return None
        idx = int(np.argmin(np.abs(self.timestamps - timestamp)))
        if abs(self.timestamps[idx] - timestamp) > tolerance_sec:
            return None
        return self.positions[idx]


def _read_parquet_tables(paths: list[Path]) -> list:
    try:
        import pyarrow.parquet as pq
    except ImportError as exc:
        raise ImportError(
            'pyarrow is required to load LeRobot parquet datasets. '
            'Install with: pip install pyarrow'
        ) from exc

    tables = []
    for path in paths:
        tables.append(pq.read_table(path))
    return tables


def _column_to_numpy(table, name: str) -> np.ndarray | None:
    if name not in table.column_names:
        return None
    col = table.column(name)
    if hasattr(col, 'combine_chunks'):
        col = col.combine_chunks()
    return np.asarray(col.to_numpy(zero_copy_only=False), dtype=object)


def _stack_vector_column(values: np.ndarray) -> np.ndarray:
    rows = [np.asarray(v, dtype=float).ravel() for v in values]
    if not rows:
        return np.empty((0, 0))
    return np.vstack(rows)


def _discover_parquet_files(dataset_root: Path) -> list[Path]:
    data_dir = dataset_root / 'data'
    if data_dir.is_dir():
        files = sorted(data_dir.rglob('*.parquet'))
        if files:
            return files
    return sorted(dataset_root.rglob('*.parquet'))


def _infer_state_keys(features: dict) -> tuple[str | None, str | None]:
    position_key = None
    velocity_key = None
    for key in features:
        lower = key.lower()
        if position_key is None and 'observation' in lower and 'state' in lower:
            position_key = key
        if velocity_key is None and 'velocity' in lower:
            velocity_key = key
    if position_key is None:
        for key in ('observation.state', 'state', 'joint_positions'):
            if key in features:
                position_key = key
                break
    return position_key, velocity_key


def _joint_names_from_features(features: dict, position_key: str | None, dim: int) -> list[str]:
    if position_key and position_key in features:
        feat = features[position_key]
        raw_names = feat.get('names')
        if raw_names:
            return normalize_joint_names([str(name) for name in raw_names])
    return normalize_joint_names([f'joint_{index}' for index in range(dim)])


def load_lerobot_dataset(
    dataset_path: str | Path,
    episode_indices: list[int] | None = None,
) -> LeRobotTrajectory:
    """Load positions/velocities from a LeRobot v2-style dataset directory."""
    root = Path(dataset_path)
    if not root.is_dir():
        raise FileNotFoundError(f'LeRobot dataset directory not found: {root}')

    info_path = root / 'meta' / 'info.json'
    fps = 30.0
    joint_names: list[str] = []
    position_key = 'observation.state'
    velocity_key = 'observation.velocity'

    if info_path.is_file():
        with info_path.open(encoding='utf-8') as f:
            info = json.load(f)
        fps = float(info.get('fps', fps))
        features = info.get('features', {})
        position_key, velocity_key = _infer_state_keys(features)
        if position_key and position_key in features:
            shape = features[position_key].get('shape', [])
            if shape:
                joint_names = _joint_names_from_features(
                    features, position_key, int(shape[0]),
                )

    parquet_files = _discover_parquet_files(root)
    if not parquet_files:
        raise FileNotFoundError(f'No parquet files found under {root}')

    if episode_indices is not None:
        selected: list[Path] = []
        for ep in episode_indices:
            matches = [p for p in parquet_files if f'episode_{ep:06d}' in p.name or f'_{ep}.' in p.name]
            if matches:
                selected.extend(matches)
        if selected:
            parquet_files = selected

    tables = _read_parquet_tables(parquet_files)

    timestamps_list: list[np.ndarray] = []
    positions_list: list[np.ndarray] = []
    velocities_list: list[np.ndarray] = []
    time_offset = 0.0

    for table in tables:
        ts_col = None
        for key in ('timestamp', 'time', 't'):
            ts_col = _column_to_numpy(table, key)
            if ts_col is not None:
                break

        pos_col = _column_to_numpy(table, position_key) if position_key else None
        if pos_col is None:
            for key in ('observation.state', 'state', 'joint_positions'):
                pos_col = _column_to_numpy(table, key)
                if pos_col is not None:
                    break
        if pos_col is None:
            continue

        positions = _stack_vector_column(pos_col)
        if positions.size == 0:
            continue

        if ts_col is not None:
            ts = np.asarray(ts_col, dtype=float).ravel()
            if ts.max() > 1e4:
                ts = ts * 1e-9
        else:
            frame_idx = _column_to_numpy(table, 'frame_index')
            if frame_idx is not None:
                ts = np.asarray(frame_idx, dtype=float).ravel() / fps
            else:
                ts = np.arange(len(positions), dtype=float) / fps

        if ts.size != len(positions):
            n = min(ts.size, len(positions))
            ts = ts[:n]
            positions = positions[:n]

        ts = ts - ts[0] + time_offset
        time_offset = float(ts[-1]) + 1.0 / fps

        vel = np.zeros_like(positions)
        vel_key = velocity_key
        vel_col = _column_to_numpy(table, vel_key) if vel_key else None
        if vel_col is None:
            for key in ('observation.velocity', 'velocity', 'joint_velocities'):
                vel_col = _column_to_numpy(table, key)
                if vel_col is not None:
                    break
        if vel_col is not None:
            vel = _stack_vector_column(vel_col)
            n = min(len(vel), len(positions))
            vel = vel[:n]
            positions = positions[:n]
            ts = ts[:n]
        elif len(positions) > 1:
            vel[1:] = np.diff(positions, axis=0) * fps
            vel[0] = vel[1]

        timestamps_list.append(ts)
        positions_list.append(positions)
        velocities_list.append(vel)

    if not timestamps_list:
        raise ValueError(f'No usable joint state columns found in {root}')

    timestamps = np.concatenate(timestamps_list)
    positions = np.vstack(positions_list)
    velocities = np.vstack(velocities_list)

    if not joint_names:
        joint_names = normalize_joint_names(
            [f'joint_{index}' for index in range(positions.shape[1])],
        )

    return LeRobotTrajectory(
        timestamps=timestamps,
        positions=positions,
        velocities=velocities,
        joint_names=joint_names,
        fps=fps,
    )
