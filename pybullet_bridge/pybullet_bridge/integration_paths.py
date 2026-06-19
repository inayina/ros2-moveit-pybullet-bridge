"""Resolve cross-repo paths for robot-arm-episode-data-lab integration."""

from __future__ import annotations

import os
from pathlib import Path

_REPO_DIRNAME = 'robot-arm-episode-data-lab'
_LEROBOT_REL = Path('dataset/v1/lerobot_export')
_DOCKER_MOUNT = Path('/data/episode-data-lab')


def _unique_existing_dirs(candidates: list[Path]) -> list[Path]:
    seen: set[Path] = set()
    existing: list[Path] = []
    for candidate in candidates:
        if not candidate:
            continue
        resolved = candidate.expanduser().resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        if resolved.is_dir():
            existing.append(resolved)
    return existing


def candidate_episode_data_lab_roots() -> list[Path]:
    """Return existing directories that may host episode-data-lab."""
    candidates: list[Path] = []

    env_root = os.environ.get('EPISODE_DATA_LAB_ROOT', '').strip()
    if env_root:
        candidates.append(Path(env_root))

    candidates.append(_DOCKER_MOUNT)
    candidates.append(Path.home() / 'robot-sim-lab' / _REPO_DIRNAME)

    try:
        bridge_repo = Path(__file__).resolve().parents[2]
        if (bridge_repo / 'pybullet_bridge').is_dir():
            candidates.append(bridge_repo.parent / _REPO_DIRNAME)
    except IndexError:
        pass

    return _unique_existing_dirs(candidates)


def resolve_episode_data_lab_root(*, required: bool = False) -> Path | None:
    roots = candidate_episode_data_lab_roots()
    if roots:
        return roots[0]
    if required:
        raise FileNotFoundError(
            'episode-data-lab root not found. Set EPISODE_DATA_LAB_ROOT to the '
            'robot-arm-episode-data-lab checkout.',
        )
    return None


def resolve_lerobot_export(*, required: bool = False) -> Path | None:
    env_export = os.environ.get('LEROBOT_EXPORT', '').strip()
    if env_export:
        path = Path(env_export).expanduser().resolve()
        if path.is_dir():
            return path
        if required:
            raise FileNotFoundError(f'LEROBOT_EXPORT directory not found: {path}')

    root = resolve_episode_data_lab_root()
    if root is not None:
        path = (root / _LEROBOT_REL).resolve()
        if path.is_dir():
            return path

    if required:
        raise FileNotFoundError(
            'LeRobot export not found. Run export_lerobot_style.py in '
            'episode-data-lab or set LEROBOT_EXPORT.',
        )
    return None


def default_lerobot_export_path() -> str:
    """Default launch argument when export may not exist yet."""
    export = resolve_lerobot_export()
    if export is not None:
        return str(export)

    root = resolve_episode_data_lab_root()
    if root is not None:
        return str((root / _LEROBOT_REL).resolve())

    return str(Path.home() / 'robot-sim-lab' / _REPO_DIRNAME / _LEROBOT_REL)
