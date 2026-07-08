# 12 · Real-Machine Readiness 实施验证 Spec

**文档版本**：v0.1  
**状态**：Draft / 待实现  
**日期**：2026-07-06  
**范围**：`ros2-moveit-pybullet-bridge` 作为机械臂三仓数据闭环的下游实施验证仓库  
**依赖**：[ARCHITECTURE](../ARCHITECTURE.md)、[ICD](../ICD.md)、[FMEA](../FMEA.md)、[EXPERIMENTS](../EXPERIMENTS.md)、[PANDA_ALIGNMENT_ROADMAP](../PANDA_ALIGNMENT_ROADMAP.md)

---

## 1. 背景与新定位

当前仓库已经具备 MoveIt -> PyBullet 执行闭环、双源分布监控、PolicyRunner、risk_engine、HOC 控制台和 LeRobot / Panda handoff 入口。下一阶段不应继续把重点放在新增复杂算法或扩展前端界面，而应把已有能力整理成真实机械臂接入前的实施验证体系。

新的仓库定位：

> `ros2-moveit-pybullet-bridge` 是机械臂三仓数据闭环的下游实施验证仓库，负责在接入真实机械臂之前，对 MoveIt 规划、轨迹执行、坐标系、策略 replay、抓取稳定性、通信健康、安全联锁和迁移风险进行系统化评估。

当前阶段属于 **Sim-to-Sim / Sim2Real-readiness**。`Real-Source` 仍指 randomized PyBullet 或 LeRobot replay，不表示已经接入真实 Panda、iiwa7 或其他真实机械臂。

---

## 2. 目标与非目标

### 2.1 目标

| ID | 目标 | 交付形式 |
|----|------|----------|
| RM-SPEC-01 | 将仓库定位从 demo 展示重构为真机接入前实施验证 | README 与 docs 口径更新 |
| RM-SPEC-02 | 定义真实机械臂接入前 readiness checklist | `docs/REAL_MACHINE_READINESS.md` |
| RM-SPEC-03 | 定义系统集成测试矩阵 | `docs/INTEGRATION_TEST_PLAN.md` |
| RM-SPEC-04 | 定义抓取稳定性评估协议 | `docs/GRASP_EVALUATION_PROTOCOL.md` |
| RM-SPEC-05 | 定义 frame / calibration 检查流程 | `docs/FRAME_AND_CALIBRATION_CHECK.md` |
| RM-SPEC-06 | 定义安全验收计划 | `docs/SAFETY_ACCEPTANCE_PLAN.md` |
| RM-SPEC-07 | 定义实施报告模板 | `docs/IMPLEMENTATION_REPORT_TEMPLATE.md` |
| RM-SPEC-08 | 设计最小抓取评测脚本接口 | `scripts/run_grasp_evaluation.py` 方案，不要求一次性完整实现 |

### 2.2 非目标

- 不新增复杂控制、规划、抓取或学习算法。
- 不继续扩展 HOC 前端页面或 dashboard 交互。
- 不宣称已经完成真实机械臂 Sim2Real。
- 不在本阶段实现 `real_source:=ros2` 的完整硬件驱动。
- 不把真实安全责任交给软件 E-stop 单独承担；真实部署必须包含硬件安全链路。

---

## 3. 当前能力基线

| 能力 | 当前状态 | 证据入口 |
|------|----------|----------|
| MoveIt -> PyBullet 闭环 | 已具备 | `m2_iiwa_demo.launch.py`、`scripts/verify_moveit_closure.sh` |
| `/bridge/command` -> `/joint_states` 回读 | 已具备 | `pybullet_bridge.bridge_node`、`trajectory_controller_node` |
| dual-source monitoring | 已具备 | `/bridge/sim/joint_states`、`/bridge/real/joint_states` |
| KL / W1 / MMD 分布偏移 | 已具备 | `dist_monitor`、`docs/EXPERIMENTS.md` |
| risk_engine R0-R3 | 已具备 | `risk_engine`、`docs/ACCEPTANCE_GAP.md` |
| PolicyRunner replay | 已具备 | `pybullet_bridge.learning.policy_runner` |
| HOC dashboard | 已具备 | `hoc_console` |
| LeRobot / Panda handoff | 已具备入口 | `panda_handoff.py`、`panda_action_adapter.py` |
| 抓取 action smoke | 部分具备 | `manipulation_actions`、`scripts/verify_pick.sh` |
| 真机 readiness 文档 | 缺失 | 本 Spec 要补齐 |
| 轨迹准入验收 | 缺失 | 本 Spec 要补齐 |
| TF / 标定验收 | 部分具备 | MoveIt closure 有 TF lookup，但缺系统化检查 |
| 抓取稳定性评估 | 缺失 | 本 Spec 要补齐 |
| 现场实施报告模板 | 缺失 | 本 Spec 要补齐 |

---

## 4. 开发原则

1. **文档先行**：先补清单、矩阵、协议、模板，再决定是否补脚本。
2. **验收优先于演示**：每个条目必须回答“上线前如何判定可接真机”。
3. **保留当前仿真边界**：Real-Source 不改口成真机；所有真机描述必须写为未来接入条件。
4. **小步脚本化**：脚本只负责采集、计算、汇总，不引入新控制算法。
5. **证据可追溯**：每次评测产出 JSON / Markdown / rosbag / screenshot 路径，能放入实施报告。
6. **安全不绕过**：任何 R3、E-stop、stale state、trajectory over limit 都应进入 hold / stop / operator acknowledgement 流程。

---

## 5. 文档交付物规格

### D1 · `docs/REAL_MACHINE_READINESS.md`

目的：真实机械臂接入前总检查清单。

必须包含：

| 小节 | 内容 |
|------|------|
| Scope | 说明适用于接入真实 Panda / iiwa / UR / 定制臂前的预验收 |
| Hardware Interface | 控制柜、机械臂、夹爪、相机、末端工具、供电、接地 |
| Control Interface | `ros2_control`、厂商 SDK、`FollowJointTrajectory`、Servo、速度/位置/阻抗模式 |
| Gripper Interface | open/close、width、force、state feedback、timeout、object detected |
| Sensors | camera、depth、joint state、force/torque、IO、时间戳 |
| TF / Calibration | base、tool、camera、object frame 的必备标定 |
| Safety Layer | 硬件 E-stop、安全 PLC、软件 quick stop、限速、限位 |
| Time Sync | system clock、ROS time、NTP/PTP、header stamp 单调性 |
| Network / DDS | NIC、VLAN、QoS、domain id、Cyclone/FastDDS 配置 |
| Fallback Plan | 断网、驱动异常、策略异常、标定失败时的回退 |
| Go / No-Go | 所有 P0 项通过才允许进入单关节 bring-up |

验收标准：

- 每个条目有 `Pass / Fail / N/A / Evidence / Owner / Next action` 字段。
- 明确写出当前仓库只能完成 readiness 的仿真预验收，不能替代现场安全验收。

### D2 · `docs/INTEGRATION_TEST_PLAN.md`

目的：系统集成测试矩阵。

测试项必须覆盖：

| ID | 测试项 | 当前可用证据 | 未来脚本 |
|----|--------|--------------|----------|
| IT-01 | static interface check | `docs/ICD.md` | topic/service/action introspection |
| IT-02 | joint order / limits check | `check_iiwa_joint_consistency.py` | profile-generic check |
| IT-03 | single joint motion | bridge command | future acceptance script |
| IT-04 | multi-joint trajectory | bridge command / MoveIt closure | future acceptance script |
| IT-05 | MoveIt planned trajectory | `verify_moveit_closure.sh` | extend to profile matrix |
| IT-06 | policy replay trajectory | `run_system_validation.sh` | replay acceptance report |
| IT-07 | gripper open/close | `GripperStub` only | real gripper check |
| IT-08 | pick-lift-place | `verify_pick.sh` smoke | grasp evaluation |
| IT-09 | fault injection | PolicyRunner / risk scripts | fault matrix |
| IT-10 | recovery test | safety NFR | recovery SOP |

每个测试项字段：

- Objective
- Preconditions
- Command
- Expected result
- Metrics
- Pass criteria
- Evidence path
- Risk if failed
- Next action

### D3 · `docs/GRASP_EVALUATION_PROTOCOL.md`

目的：定义抓取稳定性评估协议。

必须包含指标：

| 指标 | 单位 | 说明 |
|------|------|------|
| `approach_pose_error_m` | m | TCP 到目标抓取位姿的位置误差 |
| `approach_orientation_error_rad` | rad | TCP 姿态误差 |
| `gripper_closing_time_ms` | ms | 下发 close 到状态到达闭合/接触的时间 |
| `contact_detected` | bool | 是否检测到接触 |
| `lift_height_m` | m | 物体相对初始高度抬升 |
| `hold_duration_sec` | s | 抓取后保持时长 |
| `slip_distance_m` | m | 保持期间物体相对夹爪滑移 |
| `ee_object_distance_m` | m | 末端与物体中心距离 |
| `success` | bool | 是否达成抓取 |
| `failure_reason` | enum | 失败分类 |

参数 sweep：

- object mass
- object friction
- gripper force / command
- approach speed
- lift speed
- grasp pose offset
- hold duration

推荐 acceptance：

- 每组参数至少 20 次 trial。
- 单一标准物体 `success_rate >= 0.8` 才允许进入真实低速抓取 bring-up。
- 若 `drop_count > 0`，必须记录 rosbag / video / JSON trial evidence。

### D4 · `docs/FRAME_AND_CALIBRATION_CHECK.md`

目的：定义 frame 与标定验收。

必须覆盖 frame：

- `world`
- `base_link` 或机型等价 base frame
- `ee_link`
- `tool0`
- `gripper_tcp`
- `object_frame`
- `camera_frame`

必须包含约定：

- 长度单位：m
- 角度单位：rad
- 关节位置：rad 或 m，按 joint type 区分
- 四元数顺序：ROS `x, y, z, w`
- 坐标系手性：ROS REP-103
- 时间戳：ROS header stamp，单调性检查

TF sanity tests：

- base -> ee transform 可查。
- base -> tool transform 可查且不跳变。
- camera -> object transform 在静态场景下漂移低于阈值。
- quaternion norm 接近 1。
- grasp pose frame 与 MoveIt planning frame 一致或有显式 transform。

### D5 · `docs/SAFETY_ACCEPTANCE_PLAN.md`

目的：定义安全验收与恢复流程。

必须覆盖：

| 场景 | 预期行为 |
|------|----------|
| command timeout | bridge 进入 HOLD |
| stale joint state | risk 至少 R1/R2，禁止继续执行新轨迹 |
| trajectory over limit | reject 或进入 hold，不下发到硬件 |
| E-stop | 停止运动，清空 active trajectory |
| pause / hold / resume | pause 后保持当前姿态，resume 需确认 |
| R0-R3 action | R0 normal，R1 warn，R2 degraded，R3 E-stop |
| operator acknowledgement | R3 clear 前必须 acknowledge |
| recovery procedure | reset、低速回 home、重新 readiness check |

必须区分：

- software hold
- software quick stop
- hardware E-stop
- controller fault
- operator stop

### D6 · `docs/IMPLEMENTATION_REPORT_TEMPLATE.md`

目的：实施报告模板。

必须包含表格字段：

- test environment
- commit / version
- simulator or hardware backend
- robot profile
- controller mode
- test item
- expected result
- actual result
- pass / fail
- evidence path
- risk level
- issue id
- next action
- operator / reviewer

报告结论必须包含：

- Go / No-Go
- residual risks
- required fixes before hardware
- required fixes before unattended operation

---

## 6. 最小抓取评测脚本设计

目标脚本：`scripts/run_grasp_evaluation.py`

本阶段只要求设计，不要求一次性实现完整复杂功能。后续实现时应优先支持仿真输入和离线报告，避免引入真实硬件依赖。

### 6.1 CLI 参数

```bash
python3 scripts/run_grasp_evaluation.py \
  --robot-profile panda \
  --backend sim \
  --trials 20 \
  --object-mass 0.2 \
  --object-friction 0.8 \
  --approach-speed 0.05 \
  --lift-height 0.08 \
  --hold-sec 3.0 \
  --grasp-pose-set docs/samples/grasp_pose_set.json \
  --output-dir docs/samples/grasp-evaluation/panda_sim_001 \
  --seed 42
```

参数说明：

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `--robot-profile` | str | `panda` | `panda` / `iiwa7` / future profile |
| `--backend` | enum | `sim` | `sim` / `hardware`，当前只实现 sim |
| `--trials` | int | 20 | 每组参数重复次数 |
| `--object-mass` | float | 0.2 | kg |
| `--object-friction` | float | 0.8 | PyBullet friction 或真实物体记录值 |
| `--approach-speed` | float | 0.05 | m/s 或等价任务速度 |
| `--lift-height` | float | 0.08 | m |
| `--hold-sec` | float | 3.0 | 抓取保持时间 |
| `--grasp-pose-set` | path | optional | grasp pose JSON |
| `--output-dir` | path | required | 产物目录 |
| `--seed` | int | 0 | 可复现实验 |
| `--dry-run` | bool | false | 只生成计划，不执行 |

### 6.2 输出文件

| 文件 | 说明 |
|------|------|
| `grasp_trials.jsonl` | 每行一个 trial 记录 |
| `grasp_summary.json` | 聚合指标 |
| `grasp_report.md` | 人读报告 |
| `evidence/` | 可选 rosbag、图片、视频、日志 |

### 6.3 Trial JSON Schema

```json
{
  "trial_id": "trial_0001",
  "seed": 42,
  "robot_profile": "panda",
  "backend": "sim",
  "params": {
    "object_mass_kg": 0.2,
    "object_friction": 0.8,
    "approach_speed_mps": 0.05,
    "lift_height_target_m": 0.08,
    "hold_duration_target_sec": 3.0
  },
  "metrics": {
    "approach_pose_error_m": 0.006,
    "approach_orientation_error_rad": 0.04,
    "gripper_closing_time_ms": 180.0,
    "contact_detected": true,
    "lift_height_m": 0.081,
    "hold_duration_sec": 3.0,
    "slip_distance_m": 0.003,
    "ee_object_distance_m": 0.018
  },
  "success": true,
  "failure_reason": "none",
  "evidence_paths": []
}
```

### 6.4 Summary JSON Schema

```json
{
  "experiment_id": "grasp_eval_20260706_001",
  "robot_profile": "panda",
  "backend": "sim",
  "trials": 20,
  "success_rate": 0.85,
  "mean_lift_height_m": 0.079,
  "slip_count": 2,
  "miss_count": 1,
  "drop_count": 0,
  "mean_ee_object_distance_m": 0.021,
  "failure_reason_distribution": {
    "none": 17,
    "missed_object": 1,
    "object_slipped": 2
  },
  "passes_acceptance": true
}
```

### 6.5 Failure Reason 枚举

| Reason | 含义 |
|--------|------|
| `none` | 成功 |
| `missed_object` | approach 到位但未覆盖物体 |
| `no_contact` | 夹爪闭合后无接触 |
| `insufficient_closure` | 夹爪状态未到达闭合或力不足 |
| `object_slipped` | hold 阶段滑移超过阈值 |
| `object_dropped` | lift/hold 后物体掉落 |
| `planning_failed` | MoveIt / fallback motion 失败 |
| `trajectory_timeout` | 轨迹执行超时 |
| `safety_stop` | risk / E-stop / watchdog 中断 |
| `sensor_missing` | 缺少 object/contact/gripper state |
| `unknown` | 未分类失败 |

---

## 7. README 修改要求

README 顶部定位应从：

> MoveIt 2 与 PyBullet 闭环仿真桥接，内置 Sim/Real 分布偏移监控与运维控制台。

调整为：

> 本仓库是机械臂三仓数据闭环的下游实施验证环节，负责在接入真实机械臂之前，对 MoveIt 规划、轨迹执行、坐标系、策略 replay、抓取稳定性、通信健康、安全联锁和迁移风险进行系统化评估。当前阶段属于 Sim-to-Sim / Sim2Real-readiness，不宣称已经完成真实机械臂 Sim2Real。

README 应新增 `Sim2Real-readiness` 小节：

- 说明当前已完成：MoveIt closure、dual-source monitoring、PolicyRunner replay、risk R0-R3、HOC、LeRobot handoff。
- 说明当前未完成：真实机械臂硬件接口、真实夹爪反馈、现场相机标定、硬件安全回路、2h soak。
- 链接本 Spec 与 D1-D6 文档。

---

## 8. 开发阶段拆分

### Phase A · 文档重定位

目标：不改业务代码，补齐 docs。

任务：

1. 新增 D1-D6 文档。
2. 更新 README 顶部定位。
3. 更新 `docs/README.md` 或 `docs/design/README.md` 索引。
4. 确认所有文档都显式写明当前不是实机 Sim2Real。

完成标准：

- `rg "Sim2Real-readiness|真实机械臂|Real-Source" README.md docs` 能看到口径一致。
- D1-D6 文件存在，且每篇都有 pass/fail/evidence 或 acceptance criteria。

### Phase B · 最小脚本骨架

目标：只实现可运行骨架，不引入复杂仿真算法。

任务：

1. 新增 `scripts/run_grasp_evaluation.py --dry-run`。
2. 生成 `grasp_summary.json` 和 `grasp_report.md` 的空模板。
3. 支持从预生成 JSONL 汇总报告。

完成标准：

- `python3 scripts/run_grasp_evaluation.py --dry-run --output-dir /tmp/grasp_eval` 生成文件。
- 不依赖真实硬件。

### Phase C · 仿真 trial 接入

目标：接入现有 `manipulation_actions` 和 bridge topic。

任务：

1. 复用 `/manipulation/pick` 或直接 replay grasp pose。
2. 记录 `/joint_states`、`/bridge/system_state`、`/risk/status`。
3. 在没有 object/contact topic 时允许 `sensor_missing` 分类。

完成标准：

- 可完成至少 5 个 sim trial。
- 输出 success/failure reason。
- 失败 trial 有 evidence path。

### Phase D · 真机接入前预留

目标：只定义接口，不接真实硬件。

任务：

1. 为 `--backend hardware` 输出明确 NotImplemented / No-Go。
2. 在 readiness 文档列出需要的真实 topic/service/action。
3. 定义 gripper state 与 object tracking topic 的最小契约。

完成标准：

- 不会误把 sim 结果标为 hardware pass。
- 实施报告模板能记录 hardware backend 的缺失项。

---

## 9. 验收标准

| ID | 验收项 | 通过条件 |
|----|--------|----------|
| ACC-01 | 文档完整 | D1-D6 全部存在 |
| ACC-02 | 口径一致 | README 与 docs 均声明 Sim2Real-readiness，不宣称真机完成 |
| ACC-03 | 接口可追溯 | 每个测试矩阵项都有 evidence path 字段 |
| ACC-04 | 安全边界明确 | D5 区分 software hold、quick stop、hardware E-stop |
| ACC-05 | 抓取指标可复现 | D3 与脚本方案使用相同指标名 |
| ACC-06 | 报告可交付 | D6 可直接复制用于一次现场实施记录 |
| ACC-07 | 不破坏现有链路 | 不修改 bridge / monitor / risk / HOC 行为 |

---

## 10. 风险与约束

| 风险 | 影响 | 缓解 |
|------|------|------|
| 把 Real-Source 误解为真机 | 面试或交付口径不严谨 | README 与 readiness 文档反复声明边界 |
| 文档过多但不可执行 | 后续开发失焦 | 每篇文档都包含 pass/fail/evidence |
| 抓取评估缺 object tracking | 指标无法完整计算 | 允许 `sensor_missing`，先记录缺口 |
| 安全计划只停留在软件层 | 真实部署风险高 | 明确硬件 E-stop / PLC 为真机必要条件 |
| 一次性实现脚本过重 | 破坏当前稳定链路 | 分 Phase，只先做 dry-run 和报告骨架 |

---

## 11. 面试表达要点

这部分用于后续 README 或面试材料复用：

> 下游不是单纯 demo。它模拟真实机械臂接入前的验收流程：先确认接口、joint order、limits、TF、通信健康和安全联锁，再用 MoveIt closure、dual-source PyBullet、LeRobot replay 和 PolicyRunner 做策略与轨迹的前置验证。当前没有真机时，我不会夸大 Sim2Real，而是把 PyBullet randomization、分布偏移指标、risk R0-R3 和抓取评估协议作为 Sim2Real-readiness 证据。未来接真机时，会按 readiness checklist 从单关节低速 bring-up、空载轨迹、夹爪检查、pick-lift-place、fault injection 到 recovery report 逐步推进。这体现的是系统集成实施工程师的接口契约、验收矩阵、风险控制、可观测性和现场交付能力。

---

## 12. 后续文档创建顺序

推荐顺序：

1. `REAL_MACHINE_READINESS.md`
2. `INTEGRATION_TEST_PLAN.md`
3. `SAFETY_ACCEPTANCE_PLAN.md`
4. `FRAME_AND_CALIBRATION_CHECK.md`
5. `GRASP_EVALUATION_PROTOCOL.md`
6. `IMPLEMENTATION_REPORT_TEMPLATE.md`
7. README 顶部定位更新
8. `scripts/run_grasp_evaluation.py` dry-run 骨架

原因：先定义能不能接真机，再定义怎么测，再定义失败怎么停和怎么恢复，最后做抓取和报告。

