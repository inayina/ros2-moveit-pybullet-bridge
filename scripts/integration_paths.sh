#!/usr/bin/env bash
# Resolve EPISODE_DATA_LAB_ROOT and LEROBOT_EXPORT for shell scripts.
# Usage: source "$(dirname "$0")/integration_paths.sh" && resolve_integration_paths

_integration_paths_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_integration_bridge_root="$(cd "${_integration_paths_script_dir}/.." && pwd)"

resolve_integration_paths() {
  if [ -z "${EPISODE_DATA_LAB_ROOT:-}" ]; then
    if [ -d /data/episode-data-lab ]; then
      EPISODE_DATA_LAB_ROOT=/data/episode-data-lab
    elif [ -d "${_integration_bridge_root}/../robot-arm-episode-data-lab" ]; then
      EPISODE_DATA_LAB_ROOT="$(cd "${_integration_bridge_root}/../robot-arm-episode-data-lab" && pwd)"
    elif [ -d "${HOME}/robot-sim-lab/robot-arm-episode-data-lab" ]; then
      EPISODE_DATA_LAB_ROOT="${HOME}/robot-sim-lab/robot-arm-episode-data-lab"
    fi
  fi
  export EPISODE_DATA_LAB_ROOT

  if [ -z "${LEROBOT_EXPORT:-}" ] && [ -n "${EPISODE_DATA_LAB_ROOT:-}" ]; then
    LEROBOT_EXPORT="${EPISODE_DATA_LAB_ROOT}/dataset/v1/lerobot_export"
  fi
  export LEROBOT_EXPORT
}
