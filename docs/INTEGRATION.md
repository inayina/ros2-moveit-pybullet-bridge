# 与 robot-arm-episode-data-lab 集成指南

两个**独立 Git 仓库**组成**同一作品集**，设计时必须通盘考虑。完整跨仓库架构、数据契约与 Sprint 见 **[docs/design/08-dual-repo-portfolio-integration-spec.md](design/08-dual-repo-portfolio-integration-spec.md)**。

| 仓库 | 典型路径 | 职责 |
|------|----------|------|
| **robot-arm-episode-data-lab** | `~/robot-sim-lab/robot-arm-episode-data-lab` | 离线 PyBullet 采集、LeRobot 导出、任务 FSM、HAL |
| **ros2-moveit-pybullet-bridge**（本仓库） | `~/ros2_ws/src/ros2-moveit-pybullet-bridge` | ROS2 桥接、KL/MMD 监控、MoveIt 闭环、HOC |

集成契约：**KUKA iiwa 7-DOF** + **LeRobot v2.1**（`dataset/v1/lerobot_export`）。

> episode-data-lab 侧简要说明：[integration_with_bridge.md](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/reference/integration_with_bridge.md)

---

## 环境变量

| 变量 | 含义 | 解析顺序 |
|------|------|----------|
| `EPISODE_DATA_LAB_ROOT` | episode-data-lab 仓库根 | 显式设置 → `/data/episode-data-lab`（Docker）→ `~/robot-sim-lab/...` → bridge 同级目录 |
| `LEROBOT_EXPORT` | LeRobot 导出目录 | 显式设置 → `$EPISODE_DATA_LAB_ROOT/dataset/v1/lerobot_export` |
| `BRIDGE_ROOT` | 本仓库根 | 默认：本仓库 checkout 路径 |
| `ROS2_WS_ROOT` | colcon 工作区 | 默认 `~/ros2_ws` |

实现：`pybullet_bridge/integration_paths.py`、`scripts/integration_paths.sh`。

---

## 一键联调（本仓库）

```bash
cd ~/ros2_ws/src/ros2-moveit-pybullet-bridge

export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab   # 按实际 checkout 修改

./scripts/run_integration_demo.sh              # 本地冒烟
./scripts/run_integration_demo.sh --collect    # 先在 episode-data-lab 采 2 条 + 导出
./scripts/run_integration_demo.sh --docker     # Docker 跑 bridge 验证
```

---

## 手动分步

### Step A · 数据侧（episode-data-lab 仓库）

```bash
cd "$EPISODE_DATA_LAB_ROOT"
python -m pip install -r requirements.txt

python scripts/batch_collect.py --output dataset/v1 --num-episodes 20 --seed 42
python scripts/export_lerobot_style.py dataset/v1 --output dataset/v1/lerobot_export
```

浏览器演示（无需 ROS）：episode-data-lab 的 `notebooks/portfolio_demo.ipynb`（Colab 一键复现）。

### Step B · 监控侧（本仓库，iiwa7）

```bash
export EPISODE_DATA_LAB_ROOT=...   # episode-data-lab checkout
export LEROBOT_EXPORT=$EPISODE_DATA_LAB_ROOT/dataset/v1/lerobot_export

source setup.sh
cd ~/ros2_ws && colcon build --packages-select bridge_monitor_msgs pybullet_bridge dist_monitor risk_engine hoc_console moveit_config --symlink-install
source install/setup.bash

ros2 run dist_monitor offline_compare \
  --real-dataset "$LEROBOT_EXPORT" \
  --sim-dataset "$LEROBOT_EXPORT"

ros2 launch pybullet_bridge portfolio_demo.launch.py sim_mode:=GUI real_source:=lerobot

./scripts/verify_portfolio.sh
```

### Step C · MoveIt（可选）

```bash
ros2 launch moveit_config m2_iiwa_demo.launch.py sim_mode:=GUI
python3 scripts/check_iiwa_joint_consistency.py
```

---

## Docker

episode-data-lab 保持 **Colab / pip**；本仓库用 Docker 固定 ROS Jazzy：

```bash
export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab
docker compose build
docker compose run --rm verify
```

详见 [`docker/README.md`](../docker/README.md)。

---

## 机型策略（方案 C）

| 场景 | `robot` profile |
|------|-----------------|
| CI / M1 冒烟 | `planar_2dof` |
| 与 episode-data-lab 联调 / M4 标定 | **`iiwa7`** |

不要混用 2-DOF 与 7-DOF LeRobot 导出做 KL/MMD。

---

## 相关文档

- [08-dual-repo-portfolio-integration-spec.md](design/08-dual-repo-portfolio-integration-spec.md) — **双仓库通盘设计（优先）**
- [06-robot-platform-selection.md](design/06-robot-platform-selection.md)
- episode-data-lab：[migration_ros2_moveit.md](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/reference/migration_ros2_moveit.md)
