# Panda JSONL Replay Roadmap

本文档说明 `ros2-moveit-pybullet-bridge` 如何消费
`robot-arm-episode-data-lab` 导出的 Panda handoff bundle。目标是把训练侧产物接入
bridge 的 PolicyRunner、PyBullet backend、分布偏移监控和风险闭环，同时继续保留
当前 KUKA iiwa7 legacy backend。

## 1. 输入契约

上游产物来自：

```text
robot-arm-episode-data-lab/training/reports/panda_act_smoke/bridge_handoff/
├── predicted_actions.jsonl
├── dataset_manifest.json
├── dataset_inspection_report.json
├── replay_check.json
└── handoff_manifest.json
```

`predicted_actions.jsonl` 每行是一条 Panda action：

```json
{
  "timestamp": 0.033,
  "episode_index": 0,
  "frame_index": 1,
  "task": "pick_lift",
  "robot": "panda",
  "schema_id": "panda_ee_delta_gripper_v0",
  "release_id": "panda_demo_delta_v0",
  "action_type": "ee_delta_gripper",
  "action": [0.001, 0.0, -0.002, 0.0, 0.0, 0.01, 0.0]
}
```

bridge 必须校验：

- `handoff_manifest.json` 的 `handoff_format == panda_bridge_handoff_v0`。
- `robot == panda`。
- `schema_id == panda_ee_delta_gripper_v0`。
- `action_type == ee_delta_gripper`。
- `action` 是有限值，shape 为 `[7]`。
- `replay_check.json` 的 `status == PASS`。
- 执行前仍需做 runtime limit / workspace / collision / watchdog 检查。

## 2. 当前状态

当前 bridge 已有：

- `ReplayPolicy`：读取 pkl joint-position replay，输出关节目标。
- `SineWavePolicy`：闭环稳定性和 watchdog 压测策略。
- `PolicyRunner`：订阅 `/bridge/sim/joint_states`，发布 `/bridge/command`。
- distribution monitor：KL / W1 / MMD。
- risk engine：系统健康、急停、风险归因。
- KUKA iiwa7 backend：作为 legacy validation backend 保留。

当前 bridge 尚未有：

- Panda JSONL action consumer。
- Panda `ee_delta_gripper` 到 bridge command 的转换层。
- Panda PyBullet / MoveIt backend 的完整配置。

## 3. 设计原则

- 不替换现有 iiwa7 demo；Panda 接入走新增路径。
- 不在 bridge 中重新训练 policy；bridge 只消费声明过 schema 的 replay / policy action。
- 不静默把 `ee_delta_gripper[7]` 当作 joint target。
- `JsonlActionReplayPolicy` 只负责读取和按帧输出动作；坐标解释、IK、限幅和碰撞检查属于 adapter / controller / risk 层。
- 所有运行时失败都转成明确异常或 `/system_health`，不能悄悄跳帧。

## 4. 开发阶段

### Phase B0：文档和契约冻结

目标：让 bridge 开发入口清楚知道上游产物格式和边界。

任务：

- 保留 `docs/PANDA_ALIGNMENT_ROADMAP.md` 作为 iiwa7/Panda 总路线。
- 使用本文档作为 JSONL replay 接入路线。
- 在 `docs/README.md` 导航中加入本文档。
- 在后续 PR 中引用 `handoff_manifest.json` 和 `replay_check.json`。

验收：

```bash
test -f docs/PANDA_JSONL_REPLAY_ROADMAP.md
```

### Phase B1：Handoff bundle loader

目标：在不启动 ROS 2 的情况下解析和校验上游 bundle。

新增建议：

```text
pybullet_bridge/pybullet_bridge/learning/panda_handoff.py
pybullet_bridge/test/test_panda_handoff.py
```

接口建议：

```python
load_handoff_bundle(path: Path) -> PandaHandoff
```

`PandaHandoff` 至少包含：

- `manifest`
- `replay_check`
- `rows`
- `schema_id`
- `action_type`
- `action_dim`
- `timestamps`
- `actions`

验收：

- 缺 `handoff_manifest.json` fail。
- `replay_check.status != PASS` fail。
- action dim 非 `[7]` fail。
- robot/schema/action type 不匹配 fail。
- NaN/Inf fail。

### Phase B2：`JsonlActionReplayPolicy`

目标：让 PolicyRunner 能像使用 `ReplayPolicy` 一样使用 Panda JSONL。

新增建议：

```text
pybullet_bridge/pybullet_bridge/learning/jsonl_action_replay_policy.py
pybullet_bridge/test/test_jsonl_action_replay_policy.py
```

行为：

- 构造时读取 handoff bundle 或 replay JSONL。
- `reset()` 回到第一帧。
- `get_action(obs)` 输出当前 `ee_delta_gripper[7]`。
- 到末尾后保持最后一帧，和现有 `ReplayPolicy` 一致。
- 不在 policy 内做 ROS publish。

注意：这一步输出的是 task-space delta action，不是 joint target。PolicyRunner 当前假设 action length 等于 joint count，因此 B2 可以先只做离线单元测试；真正接 `/bridge/command` 需要 B3。

### Phase B3：Panda action adapter

目标：把 `ee_delta_gripper[7]` 转为 bridge 可执行命令。

新增建议：

```text
pybullet_bridge/pybullet_bridge/learning/panda_action_adapter.py
pybullet_bridge/test/test_panda_action_adapter.py
```

职责：

- 读取当前观测中的 joint positions / EE pose。
- 应用 `delta_xyz[3]` 和 `delta_rpy[3]` 得到目标 EE pose。
- 调用 Panda IK 或 MoveIt planning adapter。
- 输出 Panda joint target 和 gripper command。
- 对 delta、速度、workspace、gripper range 做硬校验。

第一版可以只提供纯函数和 mock IK，确保 action 语义不再混进 `ReplayPolicy`。

### Phase B4：PolicyRunner strategy 接入

目标：新增 runtime strategy，不破坏现有 replay/sine_wave。

参数建议：

| 参数 | 默认 | 说明 |
|------|------|------|
| `strategy_type` | `replay` | 新增 `panda_jsonl_replay` |
| `panda_handoff_path` | `""` | handoff bundle 目录 |
| `panda_schema_id` | `panda_ee_delta_gripper_v0` | schema guard |
| `panda_action_type` | `ee_delta_gripper` | action type guard |
| `panda_command_mode` | `hold` | `hold` / `mock_ik` / `moveit_ik` |

验收：

- `strategy_type:=panda_jsonl_replay` 能加载 bundle。
- 缺 handoff path 时 fail-fast。
- action dim 与 runtime joint target dim 不一致时发布 `/system_health` error。
- legacy `replay` 和 `sine_wave` 测试不回归。

### Phase B5：Panda PyBullet / MoveIt backend

目标：让 Panda command 可以进入真实 bridge 仿真链。

任务：

- 新增 Panda URDF/SRDF/MoveIt config。
- 新增 Panda robot profile 和 joint limits。
- 新增 Panda joint name mapping。
- 新增 Panda PyBullet loader。
- 在 `/bridge/command` 上验证 Panda joint trajectory。

验收：

- headless smoke 能启动 Panda backend。
- 单帧 hold command 稳定。
- 小幅 delta action 不触发软限位。
- 超限 delta action 触发 risk / health，而不是静默执行。

### Phase B6：Sim2Real 监控接入

目标：把 Panda replay 纳入 bridge 的双源验证和风险闭环。

任务：

- 记录 Panda sim trajectory。
- 接入 distribution monitor 的 KL / W1 / MMD。
- 用 `handoff_manifest.release_id` 标记实验。
- 将 `replay_check` 的 action range 写入实验报告。
- 更新 HOC dashboard / reports 的 Panda 字段。

验收：

- 生成 Panda replay benchmark summary。
- 生成 Panda sim-vs-real proxy comparison。
- shift 检出、watchdog、R2/R3 风险链路可复验。

## 5. 推荐开发顺序

1. 先做 B1 loader，完全离线，风险最低。
2. 再做 B2 policy，只跑单元测试，不接 ROS command。
3. 做 B3 adapter，明确 `ee_delta_gripper` 不是 joint target。
4. 做 B4 PolicyRunner strategy，把健康状态和异常路径打通。
5. 最后做 B5/B6 backend 与监控联调。

## 6. 解决的痛点

### 痛点 1：训练动作和运行时命令语义容易混

上游训练输出是 `ee_delta_gripper[7]`，当前 bridge `ReplayPolicy` 吃的是 joint positions。直接塞进去会把 task-space delta 错当关节目标，轻则维度错，重则执行危险。

这条路线用 `JsonlActionReplayPolicy + PandaActionAdapter` 把“读取动作”和“解释动作”拆开，语义边界更硬。

### 痛点 2：跨仓库产物缺少可审计交接点

以前只有某个 replay 文件路径，很难知道它来自哪个 dataset、哪个 schema、是否通过 inspection。

handoff bundle 带 `handoff_manifest.json`、`dataset_manifest.json`、`dataset_inspection_report.json` 和 `replay_check.json`，bridge 可以在启动前 fail-fast，也能在报告里追溯 release。

### 痛点 3：保留 iiwa7 demo 和迁移 Panda 容易互相破坏

当前 iiwa7 backend 是可运行证据，不应该被 Panda 改造一次性替换。

路线图把 Panda 作为新增 strategy/backend 接入，legacy `replay` / `sine_wave` / iiwa7 继续保留，降低迁移风险。

### 痛点 4：安全检查责任不清

训练仓库只能检查 schema 和数值质量，不能保证机器人执行安全。

handoff 明确将 runtime limits、IK、collision、watchdog、distribution shift 和 risk closure 放在 bridge，避免训练侧“看起来通过”就被误认为可直接上机。

### 痛点 5：后续面试或作品集讲解链路断

现在可以讲成一条闭环：

```text
teleop/MuJoCo episode
-> dataset inspection/release
-> smoke train/eval
-> replay JSONL
-> bridge handoff
-> PolicyRunner / Panda adapter
-> PyBullet/MoveIt validation
-> distribution monitor / risk engine
```

这让数据、训练、执行、监控和风险闭环各司其职，不再像一堆临时脚本拼在一起。

## 7. 非目标

- 不在 bridge 内训练 ACT/ Diffusion policy。
- 不把 Panda JSONL 静默转成 iiwa7 replay。
- 不在第一版直接接真实 Panda。
- 不删除现有 iiwa7 validation backend。
- 不绕过 risk engine 或 watchdog 执行动作。
