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
| **实现** | `learning/policy_runner.py`、`learning/panda_action_adapter.py`、`learning/jsonl_action_replay_policy.py` |
| **策略** | `panda_jsonl_replay`（Panda 主线） |
| **输入** | `ee_delta_gripper[7]` JSONL |
| **输出** | `/bridge/command` → PyBullet 关节控制 |

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
| **实现** | `dist_monitor/monitor_node.py`、`risk_engine` |
| **职责** | 分布漂移（KL/W1/MMD）、推理超时 Hold、E-stop 联动 |
| **话题** | `/monitor/distribution_metrics`、`/risk/status` |

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
