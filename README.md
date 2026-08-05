# ros2-moveit-pybullet-bridge

**命令重放、运行时监控、风险聚合与故障观测层**

校验并消费中游 Handoff，在 PyBullet 中重放 JSONL 或 PolicyCommand 动作，记录 latency / trace / distribution，把风险聚合到受控的 Hold 或 E-stop 路径，并在 HOC 中关联 Brain、Execution、Safety 与 Task GT。`PolicyRunner` 是 **replay harness**，不是在线策略大脑。

面向：**机器人系统软件工程师｜ROS 2、C++、Linux、设备通信、执行监督与系统验证**。

---

## 解决什么系统问题

导出的动作能加载，不代表执行端语义正确；命令能下发，不代表任务完成；风险灯变绿，也不代表抓取成功。下游需要把这些问题拆开观察：

- 交付物是否通过静态合同；
- 动作是否按约定维度与语义进入仿真；
- 时序、分布与风险如何变化；
- Hold / E-stop 路径是否可被观测与复现。

本仓回答的是“交付物能否被安全地重放和观测”，而不是“策略是否学会了抓取”。

---

## 在三仓架构中的位置

```text
ros2-arm-teleoperation-suite（上游）
  在线执行 · 控制 · 设备接口 · 采集 · Task GT
                     │ raw episode
                     ▼
robot-arm-episode-data-lab（中游）
  合同 · Release · 训练 · 离线评测 · Handoff
                     │ actions / reports
                     ▼
ros2-moveit-pybullet-bridge（本仓 · 下游）
  Replay · Monitor · Risk · Safety · HOC
```

<p align="center">
  <img src="docs/assets/readme_three_repo_overview.svg" alt="Franka Panda 三仓执行、交付与验证架构" width="100%">
</p>

上游拥有在线执行与物理 Task GT；中游拥有合同与 Handoff；本仓拥有 replay、监控与风险观测。边界见中游 [BOUNDARY_FREEZE.md](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/portfolio/BOUNDARY_FREEZE.md)。

---

## 输入 · 处理 · 输出

| 方向 | 内容 |
| --- | --- |
| **输入** | 中游 `bridge_handoff/`：`handoff_manifest.json` + `predicted_actions.jsonl` + `replay_check.json`；或五轨 PolicyCommand trace（M5） |
| **处理** | 静态合同校验 → PolicyRunner → Panda Action Adapter → PyBullet；并行 dist_monitor / risk_engine / HOC |
| **输出** | `benchmark_summary.json`、timeseries、distribution metrics、risk status、HOC 四泳道关联、离线 risk readiness 附录 |

```text
版本化 Release 与 Handoff
        ↓
命令重放、运行监控、风险和故障验证
```

---

## 核心模块

| 模块 | 路径 | 职责 |
| --- | --- | --- |
| Handoff Loader | `pybullet_bridge/.../learning/panda_handoff.py` | 校验 robot、schema、维度、NaN、manifest、`replay_check=PASS` |
| PolicyRunner | `pybullet_bridge/.../learning/policy_runner.py` | JSONL / PolicyCommand **重放**；发布 `/bridge/command` |
| Panda Action Adapter | `panda_action_adapter.py` | `ee_delta_gripper[7]` → 关节目标（`hold` / `mock_ik` / `pybullet_ik`） |
| Benchmark | `scripts/benchmark_system.py` | 有界 replay 入口；可 `--launch-stack` |
| Distribution Monitor | `dist_monitor/` | KL / W1 / MMD 等分布漂移（validity-first） |
| Risk Engine | `risk_engine/` | 风险聚合；Risk→Safety bridge **默认 `dry_run=true`** |
| HOC | `hoc_console/` | Brain / Execution / Safety / Task GT 四泳道；缺数据为 `UNAVAILABLE/STALE` |
| Offline Risk Readiness | `scripts/run_offline_risk_readiness.py` | timeseries 附录；**不**作任务 go/no-go |
| Sensor Fusion | `sensor_fusion_node.py` | **Experimental**：图像仅时间同步，像素不进估计 |

---

## 正常执行链

```text
Handoff
  → 静态合同校验（panda_handoff）
  → PolicyRunner（replay harness）
  → Panda Action Adapter
  → PyBullet
  → Trace / Distribution / Risk / HOC / benchmark report
```

Replay 完成只证明接口与重放路径工作，**不等于**真实任务成功。Task GT 结论仍以上游或 Isaac 连续真值为准；本仓不得用 risk 或 HOC 覆盖物理成败。

## 异常 / 故障链

```text
非法命令、状态异常或风险升高
  → risk_engine 聚合有效输入
  →（dry_run=false 时）发布 runtime Hold 或 TriggerEstop
  → 状态进入 HELD 或 ESTOPPED
  → Execution 与 Safety 分别记录
  → HOC 按泳道关联展示（不发明绿色零）
```

默认监控路径下 Safety bridge 为 **dry-run**：只观测提议，不自动落到生产式停机。M6 有界 mock wiring smoke 在受控脚本中显式关闭 dry-run，用于验证 `EXECUTED → HELD → ESTOPPED` 的真实 ROS/DDS 布线；使用 mock PolicyBackend，**不**加载 SmolVLA，**不**声称任务成功。

软件 Hold/E-stop ≠ 认证的硬件功能安全。

<p align="center">
  <img src="docs/assets/hoc-runtime-four-lane-dashboard.png" alt="HOC Brain、Execution、Safety 与 Task GT 四泳道运行时界面" width="100%">
</p>

<p align="center"><sub>真实运行界面：Brain、Execution、Safety、Task GT 保持独立；缺失真值明确显示 UNAVAILABLE，不用“绿色零”补齐。</sub></p>

---

## 当前已验证状态

- Handoff 校验、JSONL replay、Panda adapter、benchmark CLI 已实现。
- `--launch-stack` 可拉起 bridge、distribution monitor 与 risk 路径。
- M5：五轨 PolicyCommand trace 离线严格重放（`is_closed_loop=false`，拒绝任务成功声明）。
- M6：mock PolicyBackend 在真实 ROS/DDS 下跑通 `EXECUTED → HELD → ESTOPPED` 并正常清理。
- SmolVLA v3 的 1-episode PolicyRunner smoke 可完成接口重放；`is_closed_loop=false`。
- Offline risk readiness 仅作系统层附录（`use_as_task_go_no_go=false`）。
- Sensor Fusion 为 **experimental**。
- **没有**真实 Panda 驱动；**没有**完成 Sim2Real。

状态细则：[CURRENT_STATUS.md](docs/CURRENT_STATUS.md)。跨仓策略结论（open-loop Pass / Isaac lift 0/5 Hold）以中游 [FINAL_PROJECT_SUMMARY.md](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/portfolio/FINAL_PROJECT_SUMMARY.md) 为准。

---

## 快速开始

CPU 主线健康检查：

```bash
./scripts/run_cpu_tests.sh
```

已有中游 `bridge_handoff/` 时的有界 replay：

```bash
python3 scripts/benchmark_system.py \
  --strategy panda_jsonl_replay \
  --panda-handoff-path /path/to/bridge_handoff \
  --episodes 1 \
  --duration-sec 5.0 \
  --launch-stack \
  --panda-command-mode pybullet_ik
```

离线 risk 附录：

```bash
PYTHONPATH=risk_engine python3 scripts/run_offline_risk_readiness.py \
  --timeseries /path/to/benchmark_timeseries.csv \
  --summary /path/to/benchmark_summary.json \
  --out /tmp/risk_offline_readiness.json
```

```bash
bin/ask-project "PolicyRunner 能证明什么，不能证明什么？"
```

中游不在默认路径时：`export EPISODE_DATA_LAB_ROOT=/path/to/robot-arm-episode-data-lab`。

---

## 目录导航

| 路径 | 用途 |
| --- | --- |
| `pybullet_bridge/pybullet_bridge/learning/` | handoff、PolicyRunner、动作适配 |
| `pybullet_bridge/launch/` | bridge / monitor / risk 组合启动 |
| `scripts/benchmark_system.py` | replay benchmark |
| `scripts/run_offline_risk_readiness.py` | 离线 risk 附录 |
| `dist_monitor/` | 分布漂移监控 |
| `risk_engine/` | 在线 risk、离线 readiness、Safety bridge |
| `hoc_console/` | 四泳道 HOC、M5/M6 wiring |
| `evidence/downstream/` | PolicyRunner / risk 运行产物 |
| `docs/` | 状态、接口、设计 |
| `docs/assets/` | 运行截图与示意（可选） |
| `moveit_config/`、`manipulation_actions/` | **Legacy** iiwa 相关，非 Panda 主线 |

---

## 跨仓接口

| 交接 | 内容 |
| --- | --- |
| ← 中游 | `bridge_handoff/`（`panda_bridge_handoff_v0`，`ee_delta_gripper` dim 7） |
| ← 上游类型 | PolicyCommand 等消息定义来自上游 `teleop_interfaces`（跨仓消费） |
| → 中游 | replay summary / timeseries / offline risk 附录（不覆盖任务 Gate） |

接口合同：[INTER_REPO_CONTRACTS.md](docs/INTER_REPO_CONTRACTS.md)。

---

## 边界与未完成事项

**本仓不负责：** MuJoCo 批采、raw episode 录制、数据清洗、Release、ACT/SmolVLA 训练、离线 open-loop Gate、在线 chunk 调度与上游 Task GT 裁定。

**真实性边界：**

- 无真实 Panda；无完成 Sim2Real；PyBullet replay ≠ 实机。
- Replay 完成 ≠ 任务成功；Risk readiness / HOC 状态 ≠ 功能安全认证。
- Open-loop Pass、Interface Pass、M5/M6 wiring Pass 均 ≠ Reach/Grasp/Lift 成功。
- Scripted oracle 成功不能替代 learned policy 结果。
- Sensor Fusion、部分 demo 与 iiwa MoveIt 路径须标注 Experimental / Legacy。
- 香橙派、`robot-control-runtime`、真实 Modbus **尚未接入**，不在本仓。

KUKA iiwa7 与旧双仓材料为 Historical / Legacy，见 `docs/archive/`；Panda 主线与 legacy profile 隔离由 CPU 测试断言。

进一步阅读：[下游 Agent 映射](docs/AGENTS.md) · [当前状态](docs/CURRENT_STATUS.md) · [文档索引](docs/README.md)

---

## 面向招聘者的 30 秒摘要

这是三仓系统的**下游验证层**：静态校验 handoff，用 PolicyRunner 在 PyBullet 重放动作，监控分布与时序，把风险接到可观测的 Hold/E-stop 路径，并在 HOC 里分开展示执行与安全事实。它证明接口与故障路径可被监督，不证明策略已学会抓取，也不等于实机或功能安全认证。

---

## English Brief

**Downstream command replay, runtime monitoring, risk aggregation, and fault-observability layer** for a three-repo Franka Panda stack. It validates midstream handoffs, replays JSONL or PolicyCommand actions in PyBullet via a `PolicyRunner` **replay harness** (not an online policy brain), monitors distribution and latency, and correlates Brain, Execution, Safety, and Task GT in HOC.

Risk→Safety defaults to dry-run. Replay completion and risk readiness are not task success. No real Panda deployment or completed Sim2Real. Sensor fusion remains experimental; iiwa paths are legacy.
