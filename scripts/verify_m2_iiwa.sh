#!/usr/bin/env bash
# M2 iiwa smoke test: joint consistency + MoveIt move_group startup.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

export PATH="/usr/bin:/bin:/opt/ros/jazzy/bin:${PATH}"
unset CONDA_PREFIX CONDA_DEFAULT_ENV VIRTUAL_ENV
export ROS_LOG_DIR="${ROOT}/.ros_log"
mkdir -p "${ROS_LOG_DIR}"

# shellcheck source=/dev/null
source "$ROOT/setup.sh"

echo "==> 1/4 前置检查"
if ! ros2 pkg prefix moveit_config &>/dev/null; then
  echo "[FAIL] moveit_config 未找到，请先 colcon build moveit_config"
  exit 1
fi

echo "==> 2/4 关节一致性"
python3 scripts/check_iiwa_joint_consistency.py

echo "==> 3/4 清理残留进程"
pkill -f "ros2 launch moveit_config" 2>/dev/null || true
pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
sleep 1

echo "==> 4/4 启动 m2_iiwa_demo（12s，无 RViz）"
ros2 launch moveit_config m2_iiwa_demo.launch.py sim_mode:=DIRECT use_rviz:=false &
LAUNCH_PID=$!
cleanup() { kill "$LAUNCH_PID" 2>/dev/null || true; wait "$LAUNCH_PID" 2>/dev/null || true; }
trap cleanup EXIT

sleep 8

python3 - <<'PY'
import sys
import subprocess

out = subprocess.check_output(
    ['ros2', 'node', 'list'], text=True, stderr=subprocess.STDOUT)
nodes = [line.strip() for line in out.splitlines() if line.strip()]
required = {'/move_group', '/pybullet_bridge', '/arm_trajectory_controller'}
missing = sorted(required - set(nodes))
if missing:
    print('[FAIL] 缺少节点:', ', '.join(missing))
    print('当前节点:', nodes)
    sys.exit(1)

actions = subprocess.check_output(
    ['ros2', 'action', 'list'], text=True, stderr=subprocess.STDOUT)
if '/arm_controller/follow_joint_trajectory' not in actions:
    print('[FAIL] FollowJointTrajectory action 未就绪')
    print(actions)
    sys.exit(1)

print('[PASS] M2 iiwa 验证通过：move_group + 轨迹控制器已就绪')
PY
