---
marp: true
theme: default
paginate: true
size: 16:9
header: 'ros2-moveit-pybullet-bridge'
footer: '系统设计说明书 v1.0 · 2026-06-20 · 集成交付版'
style: |
  section { font-size: 28px; }
  h1 { color: #1a365d; }
  h2 { color: #2c5282; }
  table { font-size: 22px; }
  footer { font-size: 14px; color: #718096; }
---

<!-- _class: lead -->
<!-- _paginate: false -->

# ros2-moveit-pybullet-bridge

## 机械臂 Sim2Real 闭环仿真与分布监控
### 系统设计说明书 · 集成交付版

**ROS 2 Jazzy · MoveIt 2 · PyBullet · HOC 运维控制台**

inayina · v1.0 · 2026-06-20

---

## 版本记录

| 版本 | 日期 | 变更 |
|------|------|------|
| **v1.0** | 2026-06-20 | 交付视角说明书初版 |
| v0.7 | 2026-06-20 | 五维风险 / R2 降级补全 |
| v0.1 | 2026-06-19 | 架构 / 接口 / 算法初稿 |

**交付定位**

> 可脚本化验收的 Sim2Real 预集成环境：核心 launch、CI 配置、verify 脚本、HOC 可审计报告

**仓库** github.com/inayina/ros2-moveit-pybullet-bridge · **License** Apache-2.0

---

<!-- _class: lead -->

# 1. 项目背景与目标

---

## 解决什么问题（Sim2Real Gap）

| 痛点 | 交付后果 | 本系统应对 |
|------|----------|------------|
| MoveIt 与物理仿真脱节 | 实机才暴露问题 | PyBullet 闭环桥接 |
| 偏移不可量化 | 验收标准模糊 | KL / W1 / MMD 监控 |
| 数据分散在 CLI | 无法留痕 | HOC + HTML/CSV 报告 |

**适用场景**

- 机械臂**预集成验证**（无真机）
- Sim2Real **迁移前评估**与阈值标定
- **演示 / 验收 / CI** 冒烟
- 可选：与 episode-data-lab LeRobot 联动

**So What**：降低无真机条件下的**交付不确定性**

---

## 双仓库作品集架构

```
robot-arm-episode-data-lab
  PyBullet 采集 → 任务 FSM / HAL → LeRobot v2.1 export
                                      │
                                      ▼
ros2-moveit-pybullet-bridge
  MoveIt 2 → pybullet_bridge → Sim / Real Proxy
                                      │
                                      ▼
  dist_monitor (KL/W1/MMD) → risk_engine (R0-R3)
                                      │
                                      ▼
  HOC 控制台 → HTML / JSON / CSV / rosbag 验收证据
```

| 仓库 | 职责 |
|------|------|
| `robot-arm-episode-data-lab` | episode 采集、任务数据、LeRobot 导出 |
| `ros2-moveit-pybullet-bridge` | ROS 2 联调、MoveIt 闭环、监控、风险、HOC |

**边界**：LeRobot 回放是外部数据接入证据，不等同于真机接入。

---

<!-- _class: lead -->

# 2. 系统整体架构（1/2）
### 分层模块

---

## 六层架构

```
┌─────────────────────────────────────────┐
│ L5  交互层   RViz2  │  HOC 控制台        │
├─────────────────────────────────────────┤
│ L4  规划层   move_group │ manipulation    │
├─────────────────────────────────────────┤
│ L3  监控层   dist_monitor │ risk_engine   │
├─────────────────────────────────────────┤
│ L2  桥接层   pybullet_bridge │ RSP       │
├─────────────────────────────────────────┤
│ L1  仿真层   Sim-Source │ Real-Source     │
└─────────────────────────────────────────┘
```

| 包 | 交付职责 |
|----|----------|
| `pybullet_bridge` | 物理仿真、双源调度 |
| `dist_monitor` | KL/W1/MMD 量化 |
| `risk_engine` | R0–R3 风险、急停 |
| `hoc_console` | Web 运维、报告 |

**图示**：`docs/assets/portfolio-overview.png`

---

<!-- _class: lead -->

# 2. 系统整体架构（2/2）
### 数据流

---

## 控制闭环 + 监控侧链

```
RViz 拖拽 → move_group → FollowJointTrajectory
    → /bridge/command → PyBullet stepSimulation
    → /joint_states → MoveIt 闭环

侧链：/bridge/sim|real/joint_states
    → dist_monitor → /monitor/distribution_metrics
    → risk_engine → /risk/status → HOC (ws:8765)
```

**关键接口 Top 6**

| 类型 | 名称 |
|------|------|
| Topic | `/bridge/command` · `/joint_states` |
| Topic | `/monitor/distribution_metrics` |
| Service | `/bridge/inject_shift` |
| WS | `ws://localhost:8765` |

**So What**：控制链与监控链**解耦**，监控故障不阻塞运动（除 R3）

---

<!-- _class: lead -->

# 3. 核心模块（1/3）
### MoveIt 2 桥接器

---

## pybullet_bridge

| 模块 | 交付价值 |
|------|----------|
| 双 PyBullet 实例 | 无真机构造 Ground Truth |
| TrajectoryExecutor | 对齐 MoveIt 轨迹格式 |
| 240Hz / 100Hz 双定时器 | 物理不阻塞 DDS |
| changeDynamics 域随机化 | 可控偏移注入 |

**验收检查项**

- `./scripts/verify_m1.sh` PASS
- `check_iiwa_joint_consistency.py` PASS
- RViz Execute ↔ PyBullet GUI 同步

**限制**：trajectory relay 替代完整 ros2_control（闭环已通）

**图示**：`docs/assets/m2-iiwa-pipeline.svg`

---

<!-- _class: lead -->

# 3. 核心模块（2/3）
### Sim/Real 分布监控器

---

## dist_monitor

**算法栈**

| 指标 | 用途 |
|------|------|
| KL 散度 | 逐关节、可解释 |
| Wasserstein-1 | 均值漂移 |
| MMD | 多维联合分布 |

**流水线**

```
双源 JointState → 时间对齐 → ε = q_sim - q_real
    → 5s 滑窗 → shift_detected
```

**验收场景**

1. 基线 30s → R0
2. `inject_shift` → 5s 内检出
3. 报告归档

**So What**：验收看 **shift_detected + 时序**，非单点 RMSE

**图示**：`docs/assets/m4-monitor-metrics.png`

---

<!-- _class: lead -->

# 3. 核心模块（3/3）
### 风险引擎 + HOC

---

## risk_engine + hoc_console

**五维风险 R0–R3**

D1 分布偏移 · D2 跟踪误差 · D3 动力学 · D4 通信 · D5 规划失败

**Fail-Safe**

`R3 → e_stop → bridge 停步 → Acknowledge → 恢复`

**HOC 控制台**

| 层 | 技术 | 端口 |
|----|------|------|
| 前端 | React + ECharts | :5173 / :8080 |
| 后端 | hoc_server + WS | :8765 |

- 风险雷达 · 分布对比 · 急停 · rosbag · HTML/CSV 报告

**验收**：`./scripts/verify_risk_complete.sh`

**图示**：`docs/assets/m5-hoc-dashboard.png`

---

<!-- _class: lead -->

# 4. 部署与运维（1/2）
### 环境与依赖

---

## 部署拓扑

| 路径 | 适用 | 风险 |
|------|------|------|
| **Docker** | CI、新环境验收 | GUI 需 X11 |
| **源码** | RViz 录屏 | conda 冲突 |

```bash
docker compose build
docker compose run --rm verify
```

**依赖**：Ubuntu 24.04 · ROS 2 Jazzy · Python **3.12 系统版** · PyBullet ≥3.2.5

**交付风险**

| 现象 | 缓解 |
|------|------|
| UnsupportedTypeSupport | `unset CONDA_PREFIX` |
| package not found | colcon build + source |
| HOC 404 | `npm run build` |

---

<!-- _class: lead -->

# 4. 部署与运维（2/2）
### 配置与启动

---

## 环境变量 + 一键启动

| 变量 / 参数 | 说明 |
|-------------|------|
| `EPISODE_DATA_LAB_ROOT` | LeRobot 联动 |
| `real_source` | `topic` / `lerobot` |
| `websocket_port` | 8765 |

```bash
./scripts/verify_portfolio.sh          # 预检
ros2 launch pybullet_bridge portfolio_demo.launch.py sim_mode:=GUI
ros2 launch hoc_console hoc.launch.py  # → :5173
```

**5 分钟验收**：R0 稳定 → 注入偏移 → R2 → 急停 → 导出报告

详配：[docs/SETUP.md](../SETUP.md)

---

<!-- _class: lead -->

# 5. 测试策略与 CI/CD

---

## 三层测试体系

```
L1 单元（KL/MMD/插值）
  → L2 节点（Topic/WS）
    → L3 集成（launch_testing 全链路）
      → GitHub Actions (ros:jazzy-ros-base)
```

```bash
./scripts/run_tests.sh   # 全量
```

| 层级 | 验证 |
|------|------|
| 单元 | 纯算法 |
| 节点 | rclpy 单节点 |
| 集成 | bridge→monitor→risk |

**So What**：本机复验已完成；公开视频投递前仍以最新 Actions 绿勾作为最低公开证据

![bg right:40% 80%](https://github.com/inayina/ros2-moveit-pybullet-bridge/actions/workflows/ci.yml/badge.svg)

---

<!-- _class: lead -->

# 6. 已知问题与后续规划

---

## 交付边界与 Roadmap

| 范围 | 状态 |
|------|------|
| M1–M5 核心 | ✅ ~98% 可 Demo |
| S5 五维风险 | 本机复验通过 |
| M6 展示材料 | ~95%，待 5–8 分钟公开视频链接 |
| 双仓库联动 | ~95%，本机样例通过，HAL / 真机属 Phase-2+ |

**已知限制**（写入验收说明）

- 真机 `real_source:=ros2` 未实现
- 完整 ros2_control 用 relay 替代
- Bridge jitter 严格 5% 阈值略超，当前标为部分满足
- HOC 阈值 UI 未做（改 YAML）
- `portfolio_demo.launch.py` 不自动启动 HOC，演示时另开 `hoc.launch.py`

**Phase-2**：真机 HAL · NavigateToPose · use_sim_time 全栈

**本版本交付范围** = 仿真预集成 + 监控 + HOC

---

<!-- _class: lead -->

# 附录 A
### 接口清单（摘要）

---

## Topics · Services · Actions

**Topics**：`/bridge/command` · `/joint_states` · `/bridge/sim|real/joint_states` · `/monitor/distribution_metrics` · `/risk/status`

**Services**：`/bridge/inject_shift` · `/bridge/set_randomization` · `/monitor/reset_baseline` · `/risk/acknowledge` · `/hoc/export_experiment`

**Actions**：`/move_action` · `/manipulation/pick` · `/manipulation/place`

完整定义 → `docs/design/02-interface-design.md`

---

<!-- _class: lead -->

# 附录 B
### 配置示例

---

## 关键 YAML 片段

**thresholds.yaml**
```yaml
kl.mean_threshold: 0.10
wasserstein.mean_threshold: 0.05
baseline.warmup_sec: 30.0
```

**hoc_config.yaml**
```yaml
websocket_port: 8765
http_port: 8080
push_frequency_hz: 5.0
```

**bridge_config.yaml**
```yaml
physics_frequency: 240.0
publish_frequency: 100.0
enable_dual_source: true
```

---

<!-- _class: lead -->
<!-- _paginate: false -->

# 谢谢

**ros2-moveit-pybullet-bridge**

github.com/inayina/ros2-moveit-pybullet-bridge

设计详设 · `docs/design/` · 导出说明 · `docs/portfolio/README.md`

**Apache-2.0** · inayina · 2026
