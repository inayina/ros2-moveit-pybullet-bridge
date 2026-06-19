# ROS2 + MoveIt2 + PyBullet Sim2Real Bridge

虚实映射与分布监控系统 — ROS 2 Jazzy 工作区。

## 包结构

| 包 | 说明 |
|----|------|
| `bridge_monitor_msgs` | 自定义消息/服务/Action |
| `pybullet_bridge` | PyBullet 双源仿真桥接 |
| `dist_monitor` | KL/MMD 分布偏移监控 |
| `risk_engine` | 多维风险态势聚合 |
| `hoc_console` | 人机运维控制台后端 |
| `moveit_config` | MoveIt2 配置（占位 2-DOF 臂） |

设计文档见 [`docs/design/`](docs/design/README.md)。

## 快速开始

```bash
# 1. 环境
source setup.sh

# 2. 安装 Python 依赖
pip install -r requirements.txt

# 3. 编译（conda 用户请先执行下一行，确保消息包绑定 Python 3.12）
# export PATH="/usr/bin:/bin:/opt/ros/jazzy/bin:$PATH" && unset CONDA_PREFIX
cd ~/ros2_ws
colcon build --packages-select bridge_monitor_msgs pybullet_bridge dist_monitor risk_engine hoc_console moveit_config --symlink-install
source install/setup.bash

# 4. 启动完整系统（脚手架）
ros2 launch pybullet_bridge full_system.launch.py
```

## 里程碑

- **M1**：PyBullet 单实例 + 关节轨迹控制 + `/joint_states` 反馈
- **M2**（当前）：MoveIt2 规划闭环 — `FollowJointTrajectory` → PyBullet
- **M3**：双源域随机化
- **M4**：分布监控标定
- **M5**：HOC 前端 + 实验报告

### M1 验证

```bash
# 终端 1
ros2 launch pybullet_bridge bridge.launch.py

# 终端 2（2s 后发送正弦关节轨迹）
ros2 run pybullet_bridge joint_sweep_demo

# 终端 3（观察关节状态变化）
ros2 topic echo /joint_states --spin-time 1
# 若显示异常，请确认已 source setup.sh（含 .venv 的 pybullet）
```

### M2 验证（MoveIt2 + RViz 拖拽执行）

```bash
# 编译后启动 M2 全栈（PyBullet + 轨迹控制器 + move_group + RViz）
source setup.sh
cd ~/ros2_ws && colcon build --packages-select bridge_monitor_msgs pybullet_bridge moveit_config --symlink-install
source install/setup.bash

ros2 launch moveit_config m2_demo.launch.py sim_mode:=DIRECT

# 另开终端确认 Action 已就绪
ros2 action list | grep follow_joint_trajectory
# 期望: /arm_controller/follow_joint_trajectory

# RViz MotionPlanning 面板：
# 1. Planning Group 选 manipulator
# 2. 拖动末端交互 marker 设目标
# 3. Plan → Execute
# 4. 观察 PyBullet 中机械臂运动，/joint_states 同步更新
```

无 GUI 时可 `sim_mode:=DIRECT use_rviz:=true`；需要 PyBullet 窗口时 `sim_mode:=GUI`。
