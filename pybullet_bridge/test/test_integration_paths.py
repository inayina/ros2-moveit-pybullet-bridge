"""Tests for cross-repo path resolution."""

from __future__ import annotations

from pybullet_bridge import integration_paths


def test_resolve_lerobot_export_from_env(monkeypatch, tmp_path):
    export = tmp_path / 'lerobot_export'
    export.mkdir()
    monkeypatch.setenv('LEROBOT_EXPORT', str(export))
    monkeypatch.delenv('EPISODE_DATA_LAB_ROOT', raising=False)

    assert integration_paths.resolve_lerobot_export() == export.resolve()


def test_resolve_episode_root_from_env(monkeypatch, tmp_path):
    monkeypatch.setenv('EPISODE_DATA_LAB_ROOT', str(tmp_path))
    monkeypatch.delenv('LEROBOT_EXPORT', raising=False)

    assert integration_paths.resolve_episode_data_lab_root() == tmp_path.resolve()


def test_default_lerobot_export_path_uses_episode_root(monkeypatch, tmp_path):
    export = tmp_path / 'dataset/v1/lerobot_export'
    export.mkdir(parents=True)
    monkeypatch.setenv('EPISODE_DATA_LAB_ROOT', str(tmp_path))
    monkeypatch.delenv('LEROBOT_EXPORT', raising=False)

    assert integration_paths.default_lerobot_export_path() == str(export.resolve())
