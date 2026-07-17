# ros2-moveit-pybullet-bridge

`ros2-moveit-pybullet-bridge` 是三仓 Panda 闭环的下游：消费中游 `bridge_handoff`，在 PyBullet 中执行 Panda JSONL replay，并输出 tracking、distribution monitoring、fault/risk benchmark 结果。

本仓输入是中游 handoff bundle；输出是 downstream benchmark/report。它不采集 raw episode，不清洗数据，不训练模型，不执行 real-robot control，也不证明 completed Sim2Real。

## Position In The Three-Repo Loop

![Canonical three-repo dataflow](docs/assets/three_repo_canonical_dataflow.svg)

| 仓库 | 职责 |
| --- | --- |
| 上游：`ros2-arm-teleoperation-suite` | ROS 2/MuJoCo collection and upstream gate |
| 中游：`robot-arm-episode-data-lab` | release, MLP BC, predicted JSONL, bridge handoff |
| 下游：本仓 | handoff loader, JSONL replay, PandaActionAdapter, PyBullet monitoring/risk |

统一事实源见中游 `docs/portfolio/THREE_REPO_CANONICAL_FACTS.md`。本仓证据资产索引见 [docs/portfolio/EVIDENCE_INDEX.md](docs/portfolio/EVIDENCE_INDEX.md)。

## Verified Capabilities

| 能力 | 当前状态 | 证据 |
| --- | --- | --- |
| Handoff bundle static validation | `implemented_and_verified` | `pybullet_bridge/pybullet_bridge/learning/panda_handoff.py`, `pybullet_bridge/test/test_panda_handoff.py` |
| JSONL open-loop replay strategy | `implemented_and_verified` | `pybullet_bridge/pybullet_bridge/learning/jsonl_action_replay_policy.py` |
| Panda `ee_delta_gripper[7]` adapter | `implemented_and_verified` | `pybullet_bridge/pybullet_bridge/learning/panda_action_adapter.py`, adapter tests |
| PyBullet replay benchmark CLI | `implemented_and_verified` | `scripts/benchmark_system.py` |
| Distribution/risk components | `implemented_not_fully_verified` for full canonical scenario | `dist_monitor/`, `risk_engine/` |
| Real Panda execution and full Sim2Real | `not_supported` | `docs/CURRENT_STATUS.md` |

## Current Verified Evidence

The midstream handoff `panda_30_mlp_bridge_v0` and the latest archived downstream
smoke are evidence from different runs. Current artifacts do not prove that the
smoke consumed that 30-episode handoff.

| Fact | Value |
| --- | --- |
| Input action type | `ee_delta_gripper` |
| Action dim | 7 |
| Available midstream handoff frames | 71,737 |
| Replay strategy | `panda_jsonl_replay` |
| Command mode used in latest archived smoke | `pybullet_ik` |
| Latest archived downstream smoke | 1/1 completed, mean/max latency `9.79 / 34.218 ms`, no fault injection |

Untraceable legacy latency/fault numbers have been retired from current canonical results.

## Core Evidence

### 实验证据图解读

下表区分中游可用 handoff 与本仓独立 downstream smoke，避免把不同 run 拼成一次端到端实验。

| 图中区域 | 与本仓关系 | 原始来源 | 边界 |
| --- | --- | --- | --- |
| G0 Upstream Dataset | 上游提供 Panda 仿真 raw episode | 中游归档的 `evidence/upstream/validate_dataset.json` | 本仓不采集 raw episode |
| G1 Midstream Release | 中游提供 release、MLP predicted actions 和 handoff | `handoff_manifest.json`, `replay_check.json` | 本仓不训练 MLP/ACT；当前未证明该 handoff 是下述 smoke 输入 |
| Independent downstream smoke | 本仓使用 `panda_jsonl_replay` 与 `pybullet_ik` 完成独立 replay smoke | `evidence/downstream/benchmark_summary.json`, `scripts/benchmark_system.py` | 证明 1-episode smoke，不证明 completed Sim2Real、real-robot control 或物理抓取成功 |

`9.79 / 34.218 ms` 是独立 1-episode smoke 的 mean/max latency；`3,275 gripper cmds out of range` 来自另一份中游 handoff 的 `replay_check.json`，表示 replay 前必须 clamp 或 reject，不能把两组数字描述成同一 run。

| Evidence | What it shows | What it does not show |
| --- | --- | --- |
| `pybullet_bridge/test/test_panda_handoff.py` | bundle validation behavior | physical task success |
| `pybullet_bridge/test/test_panda_action_adapter.py` | action adapter validation and IK behavior | real robot execution |
| `evidence/downstream/benchmark_summary.json` in midstream bundle | latest archived smoke benchmark | full fault campaign or Sim2Real |
| `docs/assets/panda_replay_control_latency.png` | replay latency visualization if regenerated from JSON | production latency guarantee |

### 可用实验图片

这些图片可以作为下游 replay/monitor/risk 的辅助说明，但要与原始 benchmark JSON 或 timeseries 对齐。未重新绑定数据源前，README 中只把它们作为可视化证据，不作为新的 headline 数字来源。

| 图片 | 解释 | 边界 |
| --- | --- | --- |
| ![Replay control latency](docs/assets/panda_replay_control_latency.png) | Panda replay 控制延迟可视化 | 不证明 real-robot latency 或 production guarantee |
| ![Replay resource usage](docs/assets/panda_replay_resource_usage.png) | benchmark resource usage 可视化 | 不证明长期容量或可靠性 |
| ![Target object randomization](docs/assets/panda_domain_randomization_distribution.png) | 30-episode target object starting pose distribution | 只能说明输入分布/随机化覆盖，不证明泛化或 completed Sim2Real |
| ![Distribution monitoring](docs/assets/panda_replay_distribution_monitoring.png) | KL/W1/MMD 等分布监控展示 | 不证明真实 Sim2Real 完成 |
| ![Fault injection response](docs/assets/panda_fault_injection_safety_response.png) | fault/watchdog response 可视化 | 原始 fault benchmark JSON 未绑定前不作 headline 数字 |
| ![Sim2Sim trajectory alignment](docs/assets/panda_sim2sim_trajectory_alignment.png) | PyBullet replay trajectory alignment 展示 | 不证明真实机器人轨迹对齐 |
| ![HOC dashboard](docs/assets/m5-hoc-dashboard.png) | HOC/dashboard 运维界面截图 | 是辅助 UI，不是 Panda replay 主线证据 |

## Quick Verification

```bash
# Run unit-level validation in this repo.
pytest pybullet_bridge/test/test_panda_handoff.py pybullet_bridge/test/test_panda_action_adapter.py

# Run a Panda handoff replay benchmark when the handoff bundle is available.
python3 scripts/benchmark_system.py \
  --strategy panda_jsonl_replay \
  --panda-handoff-path /path/to/bridge_handoff \
  --episodes 1 \
  --duration-sec 5.0 \
  --launch-stack \
  --panda-command-mode pybullet_ik
```

Project evidence query and downstream change impact are also available from this checkout:

```bash
bin/ask-project "下游当前负责什么？"
bin/project-evidence impact --base HEAD~1 --head HEAD
```

The registry and retrieval implementation remain owned by the midstream repository. Set
`EPISODE_DATA_LAB_ROOT` when that checkout is not in a configured fallback location.

## Code Map

| Path | Purpose |
| --- | --- |
| `pybullet_bridge/pybullet_bridge/learning/panda_handoff.py` | handoff manifest and JSONL validation |
| `pybullet_bridge/pybullet_bridge/learning/jsonl_action_replay_policy.py` | open-loop handoff action replay |
| `pybullet_bridge/pybullet_bridge/learning/panda_action_adapter.py` | Panda action-to-command conversion |
| `scripts/benchmark_system.py` | replay benchmark and summary output |
| `dist_monitor/` | KL/W1/MMD distribution monitoring |
| `risk_engine/` | risk aggregation |
| `docs/portfolio/EVIDENCE_INDEX.md` | image and evidence asset audit |

## Boundaries

Do not claim from this repo:

- raw episode collection;
- data cleaning or model training;
- ACT online runtime;
- real Panda driver execution;
- downstream physical grasp success validation;
- complete Sim2Real.

## Legacy And Extended Material

KUKA iiwa7, older dual-repo figures, HOC dashboard screenshots, and portfolio-wide diagrams are retained as legacy or extended reading. They are not the current Panda handoff replay mainline.

## Key Documents

- [docs/AGENTS.md](docs/AGENTS.md)
- [docs/INTER_REPO_CONTRACTS.md](docs/INTER_REPO_CONTRACTS.md)
- [docs/CURRENT_STATUS.md](docs/CURRENT_STATUS.md)
- [docs/portfolio/EVIDENCE_INDEX.md](docs/portfolio/EVIDENCE_INDEX.md)
- [docs/AGENTS.md#7-project-evidence-agent-集成](docs/AGENTS.md#7-project-evidence-agent-集成)

## English Brief

This repository is the downstream Panda handoff replay and risk-validation platform. It loads midstream bridge handoffs, replays `ee_delta_gripper[7]` actions in PyBullet, and reports replay/monitoring/risk evidence. It does not collect data, train policies, execute a real Panda robot, or prove completed Sim2Real.
