# 下游 Agent 实现映射 (AGENTS.md)

Canonical 总览：中游仓 `robot-arm-episode-data-lab/AGENTS.md` V2.1。

本文件描述 **ros2-moveit-pybullet-bridge** 内的 Sim2Sim 执行与监控 Agent。

---

## 1. Handoff Loader Agent

| 项 | 值 |
|----|-----|
| **实现** | `pybullet_bridge/learning/panda_handoff.py` |
| **输入** | 中游 `bridge_handoff/`（manifest + JSONL） |
| **职责** | 静态字段校验、action 维度、NaN/robot 标识 fail-fast |
| **不做** | 数据集清洗、训练 |

---

## 2. Replay / Policy Agent

| 项 | 值 |
|----|-----|
| **实现** | `learning/policy_runner.py`、delta/absolute 独立 adapter、`jsonl_action_replay_policy.py`、`policy_command_replay_policy.py` |
| **策略** | `panda_jsonl_replay`；M5 `panda_policy_command_replay`（离线证据） |
| **输入** | `ee_delta_gripper[7]` JSONL，或带五轨关联的 native absolute EEF8 trace bundle |
| **输出** | `/bridge/command` → PyBullet 关节控制 |

M5 bundle replay 必须保留 `is_closed_loop=false`、`claims_task_success=false`。M6 已完成 mock-policy 真实 ROS/DDS wiring，但未启动 PyBullet/Isaac、未切 SmolVLA authoritative，也不证明 task success。

闭环验收：

```bash
python3 scripts/benchmark_system.py \
  --strategy panda_jsonl_replay \
  --panda-handoff-path <bridge_handoff> \
  --launch-stack \
  --output-dir /tmp/benchmark_out
```

---

## 3. Risk / Monitor Agent

| 项 | 值 |
|----|-----|
| **实现** | `dist_monitor/monitor_node.py`、`risk_engine`（在线 `risk_node` + 离线 `offline_readiness.py`）、`hoc_console/runtime_lanes.py` |
| **职责** | validity-first 分布漂移（KL/W1/MMD）、valid-source Risk 聚合、Risk→Safety 决策桥、Brain/Execution/Safety/Task GT 四通道展示；portfolio 用 offline readiness 对照 |
| **话题** | `/monitor/distribution_metrics`、`/risk/status`、`/policy/safety_decision`、`/policy/runtime_hold` |
| **离线入口** | `scripts/run_offline_risk_readiness.py` → JSON；经中游 `--risk-readiness` 挂 `appendix.risk_readiness` |
| **硬边界** | `use_as_task_go_no_go=false`；M4 monitoring launch 默认 `safety_dry_run:=true`；M6 仅在专用 bounded mock wiring 中使用非 dry-run；不得覆盖上游/Isaac 任务真值或改写 `failure_lane`；authoritative cutover 仍须另行显式批准 |

---

## 4. Sensor Fusion Agent（Sim2Sim）

| 项 | 值 |
|----|-----|
| **实现** | `pybullet_bridge/sensor_fusion_node.py` |
| **职责** | 异频传感器近似对齐、接触/滑落估计 |
| **话题** | `/bridge/sim/grasp_status`（规划） |

---

## 5. 反馈 Agent

闭环完成后填写模板，回流中游/上游：

- `docs/templates/downstream_replay_summary.yaml`

---

## 6. 本仓不负责

- MuJoCo 批采、raw episode 录制（上游）
- Schema 适配、ACT 训练、release（中游）
- 真机驱动（文档级 Real-Machine Readiness only）

契约：[docs/INTER_REPO_CONTRACTS.md](INTER_REPO_CONTRACTS.md)

Handoff 计划：[docs/design/13-three-repo-integration-development-plan.md](design/13-three-repo-integration-development-plan.md)

---

## 7. Project Evidence Agent 集成

Project Evidence Agent 的 registry、检索、audit 和 impact 核心由中游
`robot-arm-episode-data-lab/project_knowledge/` 维护；本仓只提供薄入口，不重复实现知识检索逻辑。

```bash
# 项目事实查询
bin/ask-project "下游 Panda handoff loader 是否已实现？"

# 三仓 audit
bin/project-evidence audit --json-out /tmp/project-audit.json --markdown-out /tmp/project-audit.md

# 本仓 Git 影响分析；wrapper 自动注入 downstream repository 名
bin/project-evidence impact --base HEAD~1 --head HEAD
```

入口沿用本仓已有环境变量：

```bash
export EPISODE_DATA_LAB_ROOT=/path/to/robot-arm-episode-data-lab
```
