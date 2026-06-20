# 环境搭建与配置

本文档汇总 **ros2-moveit-pybullet-bridge** 的依赖安装、编译与常用配置项。跨仓库联调见 [INTEGRATION.md](./INTEGRATION.md)。

---

## 1. 系统要求

| 组件 | 版本 |
|------|------|
| Ubuntu | 24.04（推荐） |
| ROS 2 | Jazzy |
| Python | 3.12（系统 Python，与 ROS 绑定一致） |
| Node.js | 18+（仅 HOC 前端开发/构建） |

---

## 2. 安装方式

### 方式 A · Docker（推荐）

```bash
export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab
docker compose build
docker compose run --rm verify          # 冒烟验证
docker compose run --rm portfolio-demo  # headless 演示
```

详见 [docker/README.md](../docker/README.md)。

### 方式 B · 源码编译

```bash
# 1. 克隆到 colcon 工作区
cd ~/ros2_ws/src
git clone https://github.com/inayina/ros2-moveit-pybullet-bridge.git

# 2. 创建并激活 Python 环境
cd ros2-moveit-pybullet-bridge
python3 -m venv .venv
source .venv/bin/activate

# 3. Python 依赖
pip install -r requirements.txt

# 4. 编译（conda 用户请先 unset CONDA_PREFIX 并使用系统 Python）
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build \
  --packages-select bridge_monitor_msgs pybullet_bridge dist_monitor \
                    risk_engine hoc_console manipulation_actions moveit_config \
  --symlink-install
source install/setup.bash

# 5. 加载项目环境检查脚本
cd ~/ros2_ws/src/ros2-moveit-pybullet-bridge
source setup.sh          # ROS 2 Jazzy + .venv + workspace overlay
```

---

## 3. 核心配置项

### 3.1 环境变量

| 变量 | 默认值 / 解析顺序 | 说明 |
|------|-------------------|------|
| `EPISODE_DATA_LAB_ROOT` | `/data/episode-data-lab`（Docker）→ `~/robot-sim-lab/robot-arm-episode-data-lab` | episode-data-lab 仓库根目录 |
| `LEROBOT_EXPORT` | `$EPISODE_DATA_LAB_ROOT/dataset/v1/lerobot_export` | LeRobot 导出路径（`real_source:=lerobot` 时使用） |
| `HOC_FRONTEND_DIR` | 自动解析 `hoc_console/frontend/dist` | HOC 生产模式静态资源目录 |
| `BRIDGE_ROOT` | 本仓库 checkout 路径 | 集成脚本路径解析 |
| `ROS2_WS_ROOT` | `~/ros2_ws` | colcon 工作区根 |

### 3.2 HOC 控制台（`hoc_console/config/hoc_config.yaml`）

| 参数 | 默认 | 说明 |
|------|------|------|
| `websocket_port` | `8765` | WebSocket 实时推送端口 |
| `http_port` | `8080` | 生产模式 HTTP 静态服务 |
| `push_frequency_hz` | `5.0` | 仪表盘推送频率 |
| `rosbag_output_dir` | `~/ros2_ws/bags` | 录制输出目录 |
| `report_output_dir` | `~/ros2_ws/reports` | HTML/CSV 报告输出 |

### 3.3 分布监控（`dist_monitor` launch 参数）

| 参数 | 默认 | 说明 |
|------|------|------|
| `real_source` | `topic` | `topic`：双 PyBullet 源；`lerobot`：LeRobot 回放作为 Real 源 |
| `lerobot_dataset_path` | 自动解析 | `real_source:=lerobot` 时必填 |

### 3.4 关键 ROS 2 话题（Sim / Real 源）

| 逻辑名 | 实际话题 | 说明 |
|--------|----------|------|
| Sim 源 | `/bridge/sim/joint_states` | 理想物理参数关节状态 |
| Real 源 | `/bridge/real/joint_states` | 域随机化后的关节状态 |

阈值与标定：`dist_monitor/config/thresholds.yaml`、`dist_monitor/config/calibration.yaml`。

---

## 4. 常用 Launch 参数

```bash
# 作品集一键演示
ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=GUI \
  real_source:=topic

# MoveIt 2 + RViz 闭环
ros2 launch moveit_config m2_iiwa_demo.launch.py sim_mode:=GUI

# 完整系统（bridge + monitor + risk + HOC）
ros2 launch pybullet_bridge full_system.launch.py enable_hoc:=true
```

---

## 5. 验证脚本

```bash
./scripts/verify_m1.sh          # M1 桥接冒烟
./scripts/verify_portfolio.sh   # 作品集链路
./scripts/run_tests.sh          # 全量测试
```

---

## 6. 更多文档

- [设计规格 docs/design/](./design/README.md)
- [接口与数据流](./design/05-ros2-node-interface-and-dataflow-spec.md)
- [HOC 控制台设计](./design/04-hoc-console-design.md)
