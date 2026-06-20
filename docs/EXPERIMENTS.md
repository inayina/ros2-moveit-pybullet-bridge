# 实验与报告指南

本文档整理 bridge 侧 **Sim2Real 验证实验** 的两条流水线、指标含义与产物对照，便于作品集交付与复现。

## 实验体系概览

```
                    ┌─────────────────────────────────────┐
                    │  episode-data-lab                   │
                    │  batch_collect → LeRobot export     │
                    └──────────────┬──────────────────────┘
                                   │ LEROBOT_EXPORT
         ┌─────────────────────────┼─────────────────────────┐
         ▼                         ▼                         ▼
  负对照（自对比）           跨源对比（联调）            同任务校准
  LeRobot vs LeRobot    bridge Sim NPZ vs LeRobot   双源同 /bridge/command
  预期 KL/W1/MMD ≈ 0    任务不同，MMD 主结论         KL/W1/MMD 均可解释
         │                         │                         │
         └─────────────────────────┴─────────────────────────┘
                                   ▼
                         dist_monitor (KL / W1 / MMD)
                                   ▼
                    docs/samples/*.html + docs/assets/*.png
```

| 实验 | 脚本 | 主报告 | 科学问题 |
|------|------|--------|----------|
| **A. 双仓库联调** | `run_dual_repo_integration.sh` | `dual-repo-experiment-report.html` | 两仓库数据能否连通？监控能否区分不同来源？ |
| **B. 同任务校准** | `run_same_task_calibration.sh` | `same-task-calibration-report.html` | 同一命令下 Sim/Real 偏移是否可量化？ |

建议阅读顺序：**A（连通性）→ B（标定）**。

---

## 实验 A：双仓库联调

### 目的

- 验证 LeRobot 导出可被 bridge 加载（`real_source:=lerobot`）
- 负对照：相同数据集自对比，监控不误报
- 跨源：bridge 录制的 Sim 轨迹 vs LeRobot Real，检验 MMD 灵敏度

### 运行

```bash
export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab
export LEROBOT_EXPORT=$EPISODE_DATA_LAB_ROOT/dataset/v1/lerobot_export

cd ~/ros2_ws/src/ros2-moveit-pybullet-bridge
./scripts/run_dual_repo_integration.sh
```

### 步骤

| 步骤 | 内容 | 产物 |
|------|------|------|
| A | `validate_dataset` | `dual-repo-validation.json` |
| B | LeRobot 自对比（负对照） | `dual-repo-offline-self-metrics.json` |
| C | 在线采集 Sim NPZ（`real_source:=lerobot`） | `bridge-sim-trajectory.npz` |
| B2 | Sim NPZ vs LeRobot 跨源对比 | `dual-repo-cross-source-metrics.json` |
| D/E | 生成报告与图表 | `dual-repo-*.html`, `docs/assets/dual-repo-*.png` |

### 典型结果（参考）

| 指标 | 负对照 | 跨源 |
|------|--------|------|
| KL mean | ≈ 0 | ≈ 0（基线设定导致，见局限） |
| W1 mean | ≈ 0 | ≈ 0 |
| MMD | ≈ 0 | **非零，p < 0.05** |
| shift_detected | false | **true** |

### 局限（报告已写明）

- **任务不一致**：Sim = iiwa 正弦 demo，Real = pick_and_lift 采集
- **KL/W1 为 0**：离线对比中误差序列同时作为 P/Q；联合维 MMD 仍有效
- **在线 12s 窗口**：未跑满 30s 基线，在线 shift 可能为 false

→ 以上局限由 **实验 B** 解决。

---

## 实验 B：同任务校准

### 目的

在 **同一 JointTrajectory** 下对比 bridge 双源：

- **Sim**：理想 PyBullet
- **Real**：带 domain randomization 的 PyBullet（`real_source:=topic`）

使 KL / W1 / MMD 具有可解释的物理含义；并支持 LeRobot episode 回放（跨仓库同动作）。

### 运行

```bash
export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab
export LEROBOT_EXPORT=$EPISODE_DATA_LAB_ROOT/dataset/v1/lerobot_export

./scripts/run_same_task_calibration.sh
# 可选：BASELINE_FRAC=0.3（默认，前 30% 对齐误差作 KL/W1 基线）
```

### 子实验

| 子实验 | Launch | 说明 |
|--------|--------|------|
| **B-A** | `motion_source:=iiwa` `real_source:=topic` | 内置正弦轨迹，双源同命令 |
| **B-B** | `motion_source:=lerobot` `real_source:=topic` | 回放 LeRobot episode 0 |

### 产物

| 文件 | 说明 |
|------|------|
| `same-task-iiwa-dual.npz` | B-A 双源关节轨迹 |
| `same-task-iiwa-metrics.json` | B-A 指标 |
| `same-task-lerobot-dual.npz` | B-B 双源轨迹 |
| `same-task-lerobot-metrics.json` | B-B 指标 |
| `same-task-calibration-report.html` | 正式报告 |
| `docs/assets/same-task-*.png` | 轨迹叠加 + 指标柱状图 |

### 典型结果（参考）

| 子实验 | KL mean | W1 mean | MMD (p) | shift |
|--------|---------|---------|---------|-------|
| B-A iiwa | ~0.47 | ~0.008 | ~0.05 (p≈0.01) | true (KL) |
| B-B LeRobot | ~0.22 | ~0.001 | ~0.16 (p≈0.01) | true (KL+MMD) |

### Launch 参数（手动复现）

```bash
# B-A：正弦 demo
ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=topic motion_source:=iiwa

# B-B：LeRobot 回放
ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=topic motion_source:=lerobot \
  lerobot_dataset_path:=$LEROBOT_EXPORT episode_index:=0

# 另开终端采集
python3 scripts/capture_dual_source_trajectory.py \
  --duration 12 --output docs/samples/my-dual.npz

# 离线对比
ros2 run dist_monitor offline_compare -- \
  --dual-npz docs/samples/my-dual.npz \
  --baseline-frac 0.3 --min-samples 50
```

---

## 指标解读速查

| 指标 | 含义 | 何时可信 |
|------|------|----------|
| **KL mean** | 误差分布相对基线的散度 | 同任务 + `baseline_frac > 0` |
| **W1 mean** | 误差分布的一阶 Wasserstein 距离 | 同上 |
| **MMD** | Sim/Real 关节位置联合分布检验 | 跨源、同任务均可用 |
| **shift_detected** | 超阈值或 MMD 显著 | 参考 `detection_method` 字段 |

---

## 图表对照

### 双仓库联调（`docs/assets/dual-repo-*`）

| 图 | 证明什么 |
|----|----------|
| `integration-overview.png` | 两仓库数据流与脚本化路径 |
| `cross-source-overlay.png` | Sim 与 LeRobot 轨迹肉眼可辨不同 |
| `cross-source-metrics.png` | 跨源量化结果（MMD 驱动 shift） |
| `lerobot-trajectory.png` | Real 源为完整 7-DOF 时序 |
| `offline-self-metrics.png` | 负对照不误报 |

### 同任务校准（`docs/assets/same-task-*`）

| 图 | 证明什么 |
|----|----------|
| `iiwa-overlay.png` | 同命令下 Sim（橙）vs 随机化 Real（蓝）跟踪偏差 |
| `iiwa-metrics.png` | 非零 KL/W1 + MMD 显著 |
| `lerobot-overlay.png` | LeRobot 动作在 bridge 双源上复现 |
| `lerobot-metrics.png` | 跨仓库同动作的监控指标 |

---

## 相关设计文档

- [08-dual-repo-portfolio-integration-spec.md](./design/08-dual-repo-portfolio-integration-spec.md) — 双仓库架构契约
- [03-distribution-monitoring-algorithm.md](./design/03-distribution-monitoring-algorithm.md) — KL/MMD/W1 算法
- [INTEGRATION.md](./INTEGRATION.md) — 环境变量与分步联调
