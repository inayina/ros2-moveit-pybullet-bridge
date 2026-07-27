# ros2-moveit-pybullet-bridge

模型已经输出了一串动作，接下来怎样确认它能被系统正确读取、执行和观察？这个仓库负责三仓流程的最后一段。

它加载中游生成的 handoff bundle，在 PyBullet 中重放 Panda 动作，同时记录控制时序、分布漂移和风险状态，并把 Brain、Execution、Safety、Task GT 四类事实放到 HOC 中关联展示。它不是另一个训练仓，也不是在线策略“大脑”；这里的 `PolicyRunner` 是 replay harness。

> 可以把它理解为三仓系统的“验收台”：检查交付物能否被消费、运行时哪里出现异常，以及安全链会做出什么反应。

## 它解决什么问题

一个 checkpoint 在训练环境里能加载，不代表导出的动作一定符合执行端语义；动作能下发，也不代表机器人完成了抓取。下游需要把这些问题拆开观察：

```text
中游 handoff bundle
  manifest + predicted_actions.jsonl
                  │
                  ▼
静态合同校验 → PolicyRunner replay → Panda action adapter → PyBullet
                        │                      │
                        ├─ latency / trace     ├─ execution status
                        ├─ distribution        └─ safety events
                        ▼
                 Risk / HOC / benchmark report
```

因此，本仓的 replay 完成、risk 等级或 HOC 绿色状态都不能替代上游/Isaac 的连续任务真值。

## 在三仓系统中的位置

```text
ros2-arm-teleoperation-suite（上游）
  控制 · 采集 · 在线执行 · task GT
                  │ raw episode
                  ▼
robot-arm-episode-data-lab（中游）
  合同 · 数据 · 训练 · 离线评测 · handoff
                  │ handoff / trace
                  ▼
本仓（下游）
  replay · monitor · risk · HOC
```

完整边界见中游 [BOUNDARY_FREEZE.md](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/portfolio/BOUNDARY_FREEZE.md)。

## 你会在这里找到什么

- **Handoff Loader**：在动作进入仿真前检查 robot、schema、维度、NaN 和 manifest。
- **PolicyRunner replay harness**：消费 `ee_delta_gripper[7]` JSONL 或带关联信息的 PolicyCommand trace。
- **Panda Action Adapter**：把 handoff 动作转换为 PyBullet 可以执行的关节目标。
- **Distribution Monitor**：观察参考分布与运行分布之间的 KL、W1、MMD 等变化。
- **Risk → Safety**：根据有效输入聚合风险，在受控路径中产生 Hold 或 E-stop；默认以 dry-run 方式接入。
- **HOC**：把 Brain、Execution、Safety、Task GT 四条泳道保持分离，缺少数据时显示 `UNAVAILABLE/STALE`，不会用绿色零补齐。
- **Offline Readiness**：把 replay timeseries 汇总为可挂到中游统一报告的 appendix，但不参与任务 go/no-go。

Sensor Fusion 也在本仓，但仍是 **experimental**：三个输入已显式使用 SensorDataQoS，并通过合成 joint+image+FT 的 ROS graph wiring；图像仍只参与时间同步，像素没有进入估计，因此它不属于 Panda 主线或真实传感器验收。

## 当前状态

- Handoff 校验、JSONL replay、Panda adapter 和 benchmark CLI 已实现。
- `--launch-stack` 可以一起启动 bridge、distribution monitor 和 risk engine。
- M5 支持五轨 PolicyCommand trace 的离线严格重放。
- M6 已用 mock PolicyBackend 跑通真实 ROS/DDS 下的 `EXECUTED → HELD → ESTOPPED` wiring，并正常清理退出。
- SmolVLA v3 的 1-episode replay smoke 可以完成接口重放，但 `is_closed_loop=false`。
- Risk readiness 只表示系统层对照，不覆盖任务 `failure_lane`。
- 当前没有真实 Panda 驱动，也没有完成 Sim2Real。

这些状态是接口与系统验证结果：**Not task success / Not Sim2Real / Not real robot**。

## 快速开始

只想确认本仓的 Panda 主线在普通 Python 环境里是否健康：

```bash
./scripts/run_cpu_tests.sh
```

已有中游生成的 `bridge_handoff/` 时，可以运行一次有界 replay benchmark：

```bash
python3 scripts/benchmark_system.py \
  --strategy panda_jsonl_replay \
  --panda-handoff-path /path/to/bridge_handoff \
  --episodes 1 \
  --duration-sec 5.0 \
  --launch-stack \
  --panda-command-mode pybullet_ik
```

输出目录中会包含 benchmark summary 和 timeseries。它们回答的是“接口是否工作、时序和风险怎样”，不是“机器人是否自主抓取成功”。

从已有 timeseries 生成离线 risk 对照：

```bash
PYTHONPATH=risk_engine python3 scripts/run_offline_risk_readiness.py \
  --timeseries /path/to/benchmark_timeseries.csv \
  --summary /path/to/benchmark_summary.json \
  --out /tmp/risk_offline_readiness.json
```

项目事实检索：

```bash
bin/ask-project "PolicyRunner 能证明什么，不能证明什么？"
bin/project-evidence impact --base HEAD~1 --head HEAD
```

如果中游仓不在默认位置，设置 `EPISODE_DATA_LAB_ROOT=/path/to/robot-arm-episode-data-lab`。

## 目录怎么读

| 路径 | 用途 |
| --- | --- |
| `pybullet_bridge/pybullet_bridge/learning/` | handoff loader、PolicyRunner 与动作适配 |
| `pybullet_bridge/launch/` | bridge、monitor、risk 的组合启动 |
| `scripts/benchmark_system.py` | replay benchmark 主入口 |
| `scripts/run_offline_risk_readiness.py` | 离线 risk appendix 生成入口 |
| `dist_monitor/` | 分布漂移监控 |
| `risk_engine/` | 在线 risk、离线 readiness 与 Safety bridge |
| `hoc_console/` | 四泳道 HOC、trace export 和运行时关联 |
| `docs/` | 接口、状态、设计和验收说明 |

## 边界

本仓不负责：

- MuJoCo 批采和 raw episode 录制；这些属于上游。
- 数据清洗、release、ACT/SmolVLA 训练和离线 Gate；这些属于中游。
- 用 risk R-level、replay completion 或 HOC 状态判断抓取成功。
- 真实 Panda 驱动、生产安全认证或已经完成的 Sim2Real。

KUKA iiwa7、旧双仓材料和旧版 HOC 截图保留为 Historical / Legacy，不属于当前 Panda 主线。

## 进一步阅读

- [下游 Agent 与模块映射](docs/AGENTS.md)
- [三仓接口合同](docs/INTER_REPO_CONTRACTS.md)
- [当前状态](docs/CURRENT_STATUS.md)
- [设计与运行文档索引](docs/README.md)
- [中游最终项目总结](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/portfolio/FINAL_PROJECT_SUMMARY.md)

## English brief

This repository is the downstream replay, monitoring, and safety-observability surface of a three-repo Franka Panda system. It validates midstream handoffs, replays actions in PyBullet, monitors distribution and runtime health, and correlates Brain, Execution, Safety, and Task GT in HOC.

`PolicyRunner` is a replay harness, not an online policy brain. Replay completion and risk readiness do not establish task success. The repository does not collect training data, train policies, drive a real Panda, or prove completed Sim2Real.
