#!/bin/bash
# 加载 ROS 2 Jazzy 环境（供 source 使用，勿启用 set -u）
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "[错误] 未找到 ROS 2 Jazzy: /opt/ros/jazzy/setup.bash"
  exit 1
fi
source /opt/ros/jazzy/setup.bash

if [ ! -d "$ROOT/.venv" ]; then
  echo "[错误] 未找到 .venv，请先执行: python3 -m venv .venv && pip install -r requirements.txt"
  exit 1
fi
source "$ROOT/.venv/bin/activate"
export PYTHONPATH="${VIRTUAL_ENV}/lib/python$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')/site-packages:${PYTHONPATH}"

if [ -f ~/ros2_ws/install/setup.bash ]; then
  source ~/ros2_ws/install/setup.bash
else
  echo "[警告] ~/ros2_ws/install/setup.bash 不存在，请先 colcon build"
fi

if ! python3 -c "import pybullet" 2>/dev/null; then
  echo "[警告] pybullet 未安装，请执行: pip install -r requirements.txt"
fi

if ! ros2 pkg prefix pybullet_bridge &>/dev/null; then
  echo "[警告] pybullet_bridge 未编译，请执行:"
  echo "  cd ~/ros2_ws && colcon build --packages-select bridge_monitor_msgs pybullet_bridge --symlink-install"
fi

echo "环境就绪：ROS 2 Jazzy + .venv 已激活"
echo "M1 一键验证: ./scripts/verify_m1.sh"
