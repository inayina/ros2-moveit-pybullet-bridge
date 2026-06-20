# Sample artifacts（报告 · 指标 · 轨迹）

预生成的 HTML 报告、JSON 指标与 NPZ 轨迹，供 README / 作品集 / 面试演示引用。

> 总索引：[docs/README.md](../README.md) · 实验说明：[docs/EXPERIMENTS.md](../EXPERIMENTS.md)
> 最新复验：`run_dual_repo_integration.sh` 与 `run_same_task_calibration.sh` 已通过；online LeRobot smoke 样本 `sim=421` / `real=421`。

---

## HTML 报告

| 文件 | 类型 | 说明 |
|------|------|------|
| [dual-repo-experiment-report.html](./dual-repo-experiment-report.html) | **正式** | 双仓库联调：目的 / 设计 / 图表解读 / 局限 |
| [same-task-calibration-report.html](./same-task-calibration-report.html) | **正式** | 同任务校准：双源同命令，KL/W1/MMD 可解释 |
| [dual-repo-integration-report.html](./dual-repo-integration-report.html) | 验收 | 指标表 + 嵌入图表（联调用） |
| [sample-experiment-report.html](./sample-experiment-report.html) | HOC 样例 | 五维风险 + dashboard 截图 |

**推荐阅读顺序**：`dual-repo-experiment-report` → `same-task-calibration-report`

---

## 实验 A · 双仓库联调产物

| 文件 | 说明 |
|------|------|
| [dual-repo-validation.json](./dual-repo-validation.json) | episode-data-lab 数据集校验摘要 |
| [dual-repo-offline-self-metrics.json](./dual-repo-offline-self-metrics.json) | **负对照**：LeRobot 自对比（预期 ≈ 0） |
| [dual-repo-cross-source-metrics.json](./dual-repo-cross-source-metrics.json) | **跨源**：bridge Sim NPZ vs LeRobot Real |
| [bridge-sim-trajectory.npz](./bridge-sim-trajectory.npz) | 联调时录制的 `/bridge/sim/joint_states` |
| [dual-repo-online-smoke.json](./dual-repo-online-smoke.json) | `real_source:=lerobot` 在线监控快照（sim/real 均有样本） |

```bash
export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab
./scripts/run_dual_repo_integration.sh
```

图表：`docs/assets/` 图 1–9（与报告统一编号）

对齐报告与图片（JSON/NPZ → PNG → HTML）：

```bash
python3 scripts/regenerate_all_reports.py
```

---

## 实验 B · 同任务校准产物

| 文件 | 说明 |
|------|------|
| [same-task-iiwa-dual.npz](./same-task-iiwa-dual.npz) | B-A：iiwa 正弦 · Sim + Real 双源 |
| [same-task-iiwa-metrics.json](./same-task-iiwa-metrics.json) | B-A 指标（baseline_frac=0.3） |
| [same-task-lerobot-dual.npz](./same-task-lerobot-dual.npz) | B-B：LeRobot ep0 回放 · 双源（sim=1543 / real=1542） |
| [same-task-lerobot-metrics.json](./same-task-lerobot-metrics.json) | B-B 指标 |

```bash
export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab
export LEROBOT_EXPORT=$EPISODE_DATA_LAB_ROOT/dataset/v1/lerobot_export
./scripts/run_same_task_calibration.sh
```

图表：`docs/assets/same-task-*.png`

---

## HOC 样例（独立）

```bash
python3 scripts/generate_milestone_assets.py   # 需先有 docs/assets/m5-hoc-dashboard.png
python3 scripts/generate_sample_report.py
```

---

## NPZ 格式说明

**单源**（`bridge-sim-trajectory.npz`）：

- `timestamps`, `positions`, `velocities`, `joint_names`

**双源**（`same-task-*-dual.npz`）：

- `sim_timestamps`, `sim_positions`
- `real_timestamps`, `real_positions`
- `joint_names`
