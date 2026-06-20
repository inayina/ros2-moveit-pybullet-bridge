# 项目学习指南

本文档面向项目作者和后续维护者，用来快速建立对 **ros2-moveit-pybullet-bridge** 的整体掌握：它解决什么问题、系统如何分层、代码从哪里读起、实验如何复现，以及面试或作品集讲解时应该抓住哪些主线。

---

## 1. 一句话理解项目

本项目把 **MoveIt 2 规划**、**PyBullet 物理仿真**、**Sim/Real 分布偏移监控** 和 **HOC 运维控制台** 串成一条 ROS 2 闭环链路：

```text
MoveIt / Demo 轨迹
  -> /bridge/command
  -> PyBullet Sim + domain-randomized Real
  -> /bridge/sim/joint_states + /bridge/real/joint_states
  -> dist_monitor 计算 KL / W1 / MMD / tracking error
  -> risk_engine 聚合风险
  -> hoc_console 展示、录制、导出报告、触发干预
```

核心价值不是“只跑一个机械臂仿真”，而是证明一个面向机器人部署的工程链路：

- 规划结果能进入物理仿真闭环，而不是停留在 RViz 里。
- Sim 与 Real 的差异可以量化，而不是靠肉眼或实机事故发现。
- 风险状态、实验控制和报告导出有统一操作入口。
- 代码、测试、Docker、报告和截图能组成可复现作品集。

---

## 2. 推荐学习路线

### 第 0 步：先跑通最小闭环

目标：确认环境、构建、最小 demo 可用。

```bash
cd ~/ros2_ws/src/ros2-moveit-pybullet-bridge

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

cd ~/ros2_ws/src/ros2-moveit-pybullet-bridge
source setup.sh
ros2 launch pybullet_bridge portfolio_demo.launch.py sim_mode:=GUI
```

如果只想快速验证，不看 GUI：

```bash
export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab
docker compose build
docker compose run --rm verify
docker compose run --rm portfolio-demo
```

### 第 1 步：读 README 建立系统图

先读根目录 [README.md](../README.md)，重点看：

- “解决的核心痛点”：理解项目为什么存在。
- “系统架构”：记住 Planning、Bridge、Monitor、Risk、Ops 五层。
- “关键接口速查”：熟悉 topic / service / action 名称。
- “快速开始”：知道如何启动 portfolio demo、HOC 和 RViz 闭环。

### 第 2 步：读设计索引，不要一次读完所有长文档

从 [docs/design/README.md](./design/README.md) 开始，按用途选读：

| 你想掌握 | 推荐文档 |
|----------|----------|
| 总体架构与需求 | [01-system-architecture-and-requirements.md](./design/01-system-architecture-and-requirements.md) |
| ROS 2 接口 | [02-interface-design.md](./design/02-interface-design.md) |
| 数据流和节点契约 | [05-ros2-node-interface-and-dataflow-spec.md](./design/05-ros2-node-interface-and-dataflow-spec.md) |
| KL / W1 / MMD 算法 | [03-distribution-monitoring-algorithm.md](./design/03-distribution-monitoring-algorithm.md) |
| HOC 控制台 | [04-hoc-console-design.md](./design/04-hoc-console-design.md) |
| 双仓库联动 | [08-dual-repo-portfolio-integration-spec.md](./design/08-dual-repo-portfolio-integration-spec.md) |
| 风险监控 | [09-risk-monitoring-completion-spec.md](./design/09-risk-monitoring-completion-spec.md) |

建议先读 01、05、03、04、09。08 适合在理解本仓库后再读。

### 第 3 步：按运行链路读代码

不要按目录字母顺序读。按数据从输入到输出的方向读：

1. `bridge_monitor_msgs/`：消息、服务、Action 契约。
2. `pybullet_bridge/pybullet_bridge/bridge_node.py`：轨迹如何进入 PyBullet，Sim/Real 如何发布。
3. `pybullet_bridge/pybullet_bridge/real_source.py` 和 `domain_randomizer.py`：Real 源如何通过域随机化模拟偏移。
4. `dist_monitor/dist_monitor/monitor_node.py`：Sim/Real 如何对齐、滑窗、发布指标。
5. `dist_monitor/dist_monitor/metrics_core.py`：KL / W1 / MMD 的核心计算。
6. `risk_engine/risk_engine/risk_node.py` 和 `aggregator.py`：指标如何变成风险等级和告警。
7. `hoc_console/hoc_console/hoc_server.py`：ROS 2 数据如何进入 WebSocket / HTTP / 报告导出。
8. `hoc_console/frontend/src/`：前端如何展示风险、曲线和实验控制。
9. `moveit_config/launch/m2_iiwa_demo.launch.py`：MoveIt + RViz + bridge 闭环如何启动。
10. `manipulation_actions/`：Pick / Place 高层 Action 如何复用 MoveIt 或 bridge fallback。

### 第 4 步：用测试理解边界条件

测试文件是理解模块边界的捷径：

| 模块 | 推荐先读测试 |
|------|--------------|
| bridge | `pybullet_bridge/test/test_bridge_node.py`、`test_full_system_launch.py` |
| monitor | `dist_monitor/test/test_metrics_core.py`、`test_monitor_node.py` |
| risk | `risk_engine/test/test_aggregator.py`、`test_risk_node.py` |
| HOC | `hoc_console/test/test_hoc_commands.py`、`test_ws_hub.py`、`test_report_html.py` |
| manipulation | `manipulation_actions/test/test_manipulation_node.py` |

读测试时关注三个问题：

- 输入是什么 topic / service / action？
- 输出如何断言？
- 异常或降级路径如何处理？

---

## 3. 模块地图

### `bridge_monitor_msgs`

职责：定义跨包共享的 ROS 2 消息、服务和 Action。

重点概念：

- `DistributionMetrics`：监控输出的核心指标。
- `RiskStatus`：风险引擎对外发布的状态。
- `DomainRandomizationConfig`：域随机化配置。
- `SetRandomization`、`InjectShift`：HOC 和脚本控制 Real 源偏移。
- `Pick`、`Place`、`ExecuteScenario`：高层操作和实验动作。

学习要点：先看接口定义，再看哪些节点发布或消费这些接口。这个包是整个项目的“合同层”。

### `pybullet_bridge`

职责：把 ROS 2 轨迹命令转成 PyBullet 物理仿真，并发布关节状态。

核心入口：

- `bridge_node.py`：主节点，订阅 `/bridge/command`，发布 `/joint_states`、`/bridge/sim/joint_states`、`/bridge/real/joint_states`。
- `trajectory_executor.py`：轨迹插值和执行。
- `real_source.py`：Real 源仿真封装。
- `domain_randomizer.py`：阻尼、摩擦、负载等域随机化。
- `trajectory_controller_node.py`：提供 `FollowJointTrajectory` Action 到 `/bridge/command` 的适配。
- `portfolio_demo.launch.py`：作品集一键演示入口。

关键问题：

- 为什么同时发布 `/joint_states` 和 `/bridge/sim/joint_states`？
- `real_source:=topic` 与 `real_source:=lerobot` 的差异是什么？
- `/bridge/inject_shift` 如何影响后续指标？
- 风险进入 R2/R3 时 bridge 如何降级或暂停？

### `dist_monitor`

职责：对齐 Sim / Real 关节流，计算分布偏移指标。

核心入口：

- `monitor_node.py`：订阅双源关节状态，发布 `/monitor/distribution_metrics` 和 `/monitor/tracking_error`。
- `time_aligner.py`：按时间戳对齐双源。
- `sliding_window.py`：维护滑动窗口。
- `metrics_core.py`：KL、W1、MMD 计算。
- `offline_compare.py`：离线 NPZ / LeRobot 数据对比。
- `lerobot_loader.py`：加载跨仓库 LeRobot 导出。

关键问题：

- 为什么同任务校准比跨源对比更容易解释 KL / W1？
- MMD 为什么适合跨源联合分布对比？
- `baseline_frac` 或 reset baseline 对结果有什么影响？
- `tracking_error` 与分布指标分别代表什么风险？

### `risk_engine`

职责：把监控指标、跟踪误差、规划结果、系统状态聚合成风险等级。

核心入口：

- `aggregator.py`：多维分数聚合。
- `risk_node.py`：订阅指标并发布 `/risk/status`、`/risk/alerts`。
- `move_group_cancel.py`：高风险时取消 MoveIt action。

关键问题：

- R0/R1/R2/R3 的含义是什么？
- 哪些指标会成为 `primary_driver`？
- 为什么风险确认 `/risk/acknowledge` 和清除急停要分开？
- R2 降级和 R3 急停分别保护什么？

### `hoc_console`

职责：把 ROS 2 运行状态转成 WebSocket / HTTP / 前端视图，并提供实验控制。

核心入口：

- `hoc_server.py`：ROS 2 后端，聚合 topic、service、action。
- `ws_hub.py`：WebSocket 广播。
- `report.py`、`report_csv.py`：报告和 CSV 产物。
- `experiment_runner.py`：实验场景 Action。
- `frontend/src/`：React + ECharts Dashboard。
- `launch/hoc.launch.py`：开发模式，Vite 热更新。
- `launch/hoc_prod.launch.py`：生产模式，静态服务。

关键问题：

- HOC 是直接控制 PyBullet，还是通过 ROS service/action 控制？
- 前端命令如何映射到 `/risk/force_e_stop`、`/bridge/set_randomization` 等服务？
- 报告导出依赖哪些最新状态缓存？
- WebSocket payload 如何保持前后端契约稳定？

### `moveit_config`

职责：提供 MoveIt 2 配置和 RViz 规划闭环。

重点入口：

- `launch/m2_iiwa_demo.launch.py`：iiwa7 MoveIt + bridge demo。
- `config/iiwa_moveit_controllers.yaml`：MoveIt controller 到 `FollowJointTrajectory` 的配置。

关键问题：

- RViz 的 Plan / Execute 最终如何变成 `/bridge/command`？
- `PlanningSceneMonitor` 为什么需要 `/joint_states`？
- `manipulator` planning group 对应哪些 iiwa 关节？

### `manipulation_actions`

职责：封装 Pick / Place 高层 Action，既可以走 MoveIt，也可以走 bridge fallback。

核心入口：

- `manipulation_node.py`：ActionServer。
- `move_group_client.py`：MoveIt MoveGroup 客户端。
- `bridge_motion_client.py`：通过 `/bridge/command` 执行动作。
- `pick_executor.py`、`place_executor.py`：动作阶段逻辑。

关键问题：

- 高层任务如何拆成 approach、grasp、lift、place？
- 无 MoveIt 时 fallback 如何保持 demo 可运行？
- 风险状态如何影响高层动作接受或取消？

---

## 4. 关键数据流

### 4.1 Demo 轨迹到仿真

```text
iiwa_motion_demo / lerobot_replay_demo / trajectory_controller_node
  -> publishes trajectory_msgs/JointTrajectory
  -> /bridge/command
  -> bridge_node
  -> trajectory_executor
  -> PyBullet step
  -> /joint_states
  -> /bridge/sim/joint_states
  -> /bridge/real/joint_states
```

理解点：`/joint_states` 服务 MoveIt / TF 闭环，`/bridge/sim/joint_states` 和 `/bridge/real/joint_states` 服务监控。

### 4.2 分布监控到风险

```text
/bridge/sim/joint_states + /bridge/real/joint_states
  -> dist_monitor time aligner
  -> sliding window
  -> metrics_core
  -> /monitor/distribution_metrics
  -> /monitor/tracking_error
  -> risk_engine
  -> /risk/status + /risk/alerts
```

理解点：监控不是只看瞬时误差，而是看一段窗口内的误差和分布变化。

### 4.3 HOC 控制闭环

```text
React Dashboard
  -> WebSocket command
  -> hoc_server
  -> ROS service/action
  -> bridge_node / risk_engine / experiment_runner
  -> ROS topic/status
  -> WebSocket broadcast
  -> Dashboard update
```

理解点：前端不绕过 ROS 2，所有关键控制仍走 ROS service/action，便于测试和复现。

### 4.4 双仓库实验

```text
robot-arm-episode-data-lab
  -> dataset/v1/lerobot_export
  -> LEROBOT_EXPORT
  -> dist_monitor offline_compare
  -> bridge portfolio_demo real_source:=lerobot or motion_source:=lerobot
  -> docs/samples/*.json / *.npz / *.html
  -> docs/assets/*.png
```

理解点：episode-data-lab 是数据侧，本仓库是 ROS 2 集成、监控和运维侧。

---

## 5. 常用命令速查

### 环境和构建

```bash
cd ~/ros2_ws/src/ros2-moveit-pybullet-bridge
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

cd ~/ros2_ws/src/ros2-moveit-pybullet-bridge
source setup.sh
```

### 运行主 demo

```bash
ros2 launch pybullet_bridge portfolio_demo.launch.py sim_mode:=GUI

ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=topic motion_source:=iiwa
```

### MoveIt + RViz 闭环

```bash
ros2 launch moveit_config m2_iiwa_demo.launch.py sim_mode:=GUI
```

RViz 中选择 **Planning Group -> manipulator**，拖动末端，点击 **Plan** 和 **Execute**。

### HOC 控制台

```bash
ros2 launch hoc_console hoc.launch.py
# http://localhost:5173

cd hoc_console/frontend
npm run build
ros2 launch hoc_console hoc_prod.launch.py
# http://localhost:8080
```

### 观察核心 topic

```bash
ros2 topic echo /bridge/sim/joint_states --once
ros2 topic echo /bridge/real/joint_states --once
ros2 topic echo /monitor/distribution_metrics --once
ros2 topic echo /risk/status --once
```

### 调用关键 service

```bash
ros2 service call /bridge/inject_shift bridge_monitor_msgs/srv/InjectShift \
  "{parameter_name: joint_damping, delta_percent: 30.0, duration_sec: 5.0}"

ros2 service call /risk/force_e_stop std_srvs/srv/Trigger "{}"
```

### 测试

```bash
./scripts/run_tests.sh

cd dist_monitor
python3 -m pytest test/ -v -m "not launch_test"

cd ../pybullet_bridge
python3 -m pytest test/ -v -m launch_test
```

### 实验报告

```bash
export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab
export LEROBOT_EXPORT=$EPISODE_DATA_LAB_ROOT/dataset/v1/lerobot_export

./scripts/run_dual_repo_integration.sh
./scripts/run_same_task_calibration.sh
```

---

## 6. 学习检查清单

### 能独立讲清楚

- 项目为什么需要 Sim / Real 双源，而不是只跑一个 PyBullet。
- MoveIt 规划结果如何进入 PyBullet。
- `/joint_states`、`/bridge/sim/joint_states`、`/bridge/real/joint_states` 分别服务谁。
- KL、W1、MMD 的直觉含义和适用场景。
- 风险等级如何从监控指标推导出来。
- HOC 前端命令如何最终调用 ROS service/action。
- Docker 验证、源码 GUI、双仓库实验三种路径的区别。

### 能独立定位问题

- demo 没有运动：检查 `/bridge/command` 是否有轨迹、bridge 是否在跑。
- 监控没有指标：检查 `/bridge/sim/joint_states` 和 `/bridge/real/joint_states` 是否都有数据。
- HOC 没有刷新：检查 `hoc_server`、WebSocket `8765`、前端 `5173`。
- MoveIt Execute 后 PyBullet 不动：检查 controller action 和 `/bridge/command` 适配。
- 报告图片或 HTML 断链：检查 `docs/assets/` 和 `docs/samples/` 是否一起提交。

### 能独立扩展一个功能

建议练习顺序：

1. 给 `risk_engine/aggregator.py` 增加一个新的风险维度，并补单元测试。
2. 给 HOC WebSocket payload 增加一个只读字段，并更新前端展示。
3. 给 `dist_monitor/offline_compare.py` 增加一个 CLI 参数，并补测试。
4. 给 `pybullet_bridge` 增加一种新的 domain randomization 参数，并用 `/bridge/inject_shift` 验证。
5. 给 README 或报告脚本增加一张新图，并确保 `docs/assets/README.md` 有说明。

---

## 7. 面试 / 作品集讲解主线

可以按 4 分钟结构讲：

1. **问题**：MoveIt 规划、物理仿真、Sim2Real 偏移和运维观测通常是割裂的。
2. **方案**：用 ROS 2 把 MoveIt、PyBullet、监控、风险和 HOC 串成闭环。
3. **关键实现**：`/bridge/command` 驱动双源 PyBullet；`dist_monitor` 做 KL/W1/MMD；`risk_engine` 聚合风险；HOC 提供实时看板和实验报告。
4. **验证**：三层测试、Docker、launch_testing、双仓库实验、HTML 报告和真实截图。
5. **工程取舍**：无硬件条件下使用 domain randomization 和 LeRobot 数据作为 Real 替代源；同任务校准解决跨源任务不一致导致的指标解释问题。

如果被追问算法：

- KL / W1 更适合同任务、同误差空间下比较。
- MMD 更适合检测联合分布差异，跨源对比时更稳。
- baseline 选择会影响 KL / W1 的解释，报告中需要说明实验前提。

如果被追问工程：

- 所有关键控制走 ROS 2 service/action，不让前端直接改内部状态。
- 自定义消息包让 bridge、monitor、risk、HOC 的契约稳定。
- Docker 固定 ROS Jazzy 环境，测试分单元、节点、集成三层。

---

## 8. 后续维护建议

- 修改接口前，先改 `bridge_monitor_msgs`，再同步消费者测试。
- 修改 README 截图或报告时，同时提交 `docs/assets/`、`docs/samples/`。
- 修改 launch 参数时，同步 [README.md](../README.md)、[SETUP.md](./SETUP.md) 和相关脚本。
- 修改监控算法时，至少跑 `dist_monitor/test/test_metrics_core.py`、`test_monitor_node.py` 和相关实验脚本。
- 修改 HOC payload 时，同时检查后端 `hoc_server.py`、前端 store/components 和 `hoc_console/test/`。
- 新增作品集材料时，优先更新 [docs/README.md](./README.md) 的索引，避免文档散落。

---

## 9. 你应该优先记住的文件

| 文件 | 为什么重要 |
|------|------------|
| [README.md](../README.md) | 对外介绍和快速启动入口 |
| [docs/SETUP.md](./SETUP.md) | 安装、构建、Launch 参数 |
| [docs/EXPERIMENTS.md](./EXPERIMENTS.md) | 实验流水线和指标解读 |
| [docs/design/05-ros2-node-interface-and-dataflow-spec.md](./design/05-ros2-node-interface-and-dataflow-spec.md) | 节点和数据流规格 |
| `pybullet_bridge/pybullet_bridge/bridge_node.py` | 仿真桥接主节点 |
| `dist_monitor/dist_monitor/monitor_node.py` | 在线监控主节点 |
| `dist_monitor/dist_monitor/metrics_core.py` | 核心指标算法 |
| `risk_engine/risk_engine/risk_node.py` | 风险发布和急停入口 |
| `hoc_console/hoc_console/hoc_server.py` | HOC 后端中枢 |
| `pybullet_bridge/launch/portfolio_demo.launch.py` | 作品集一键演示入口 |

