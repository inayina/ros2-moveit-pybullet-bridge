#!/bin/bash
# 加载 ROS 2 Jazzy 环境
source /opt/ros/jazzy/setup.bash
# 激活本地虚拟环境（提供 pybullet / numpy 等依赖）
source "$(dirname "${BASH_SOURCE[0]}")/.venv/bin/activate"
export PYTHONPATH="${VIRTUAL_ENV}/lib/python$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')/site-packages:${PYTHONPATH}"
# 加载工作区环境（如果已经 colcon build 过）
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi
echo "环境就绪：ROS 2 Jazzy + .venv 已激活"
echo "编译: cd ~/ros2_ws && colcon build --packages-select bridge_monitor_msgs pybullet_bridge dist_monitor risk_engine hoc_console moveit_config --symlink-install"
