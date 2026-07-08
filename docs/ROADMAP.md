# Downstream Sim-to-Sim & Real-Machine Readiness Roadmap (开发路线图)

本文档定义了 `ros2-moveit-pybullet-bridge` 作为机械臂三仓数据闭环下游的**新开发周期（Panda 对齐、策略闭环、多源传感器融合、多模型 HOC 看板及真机就绪度验收）**的详细里程碑、开发时间线与验收条件。

---

## 1. 路线图总览与设计原则

### 1.1 核心开发目标
本周期的核心开发目标是：将下游仓库从“KUKA iiwa7 仿真 Demo”重构升级为“统一 Panda 机器人平台、支持中游策略闭环执行、具备多源异步传感器融合、支持多模型 HOC 看板对比的**真机就绪度前置验证平台**”。

```
【Phase 1: 契约对齐】 ───► 【Phase 2: 闭环执行】 ───► 【Phase 3: 就绪度文档】 ───► 【Phase 4: 自动化评测】
  • 统一 Panda 关节语义     • PandaActionAdapter     • 6 份物理就绪度规范      • HOC 多模型大屏重构
  • Handoff Loader 实现     • 逆运动学 (IK) 适配器     • 重力补偿与上电自检 SOP  • 自动网格扫描脚本 (Batch)
  • 静态字段与元数据校验    • 关节与电气死区补偿      • 手眼标定与抓取稳定性规程 • 孤立进程强杀与 CI 加固
```

### 1.2 核心开发六原则 (基于 12-spec)
1. **文档先行**：先补清单、矩阵、协议、模板，再决定是否补脚本。
2. **验收优先于演示**：每个条目必须回答“上线前如何判定可接真机”。
3. **保留当前仿真边界**：Real-Source 不改口成真机；所有真机描述必须写为未来接入条件。
4. **小步脚本化**：脚本只负责采集、计算、汇总，不引入新控制算法。
5. **证据可追溯**：每次评测产出 JSON / Markdown / rosbag / screenshot 路径，能放入实施报告。
6. **安全不绕过**：任何 R3、E-stop、stale state、trajectory over limit 都应进入 hold / stop / operator acknowledgement 流程。

---

## 2. 里程碑细化与验收标准 (Milestones & DoD)

### 里程碑 1 (RM-M1)：Panda 关节语义对齐与离线 Handoff 载入器
- **周期**：Day 1 – Day 3
- **目标**：解决中下游数据契约不一致的问题，确保 downstream 可以离线读取并校验来自 midstream 的 Panda 数据。
- **关联需求**：RM-SPEC-01（定位重构）, RM-SPEC-02（Readiness清单）
- **当前代码基线状态**：
  - *已完成*：`learning/panda_handoff.py` 已实现 `load_handoff_bundle()`，支持对 `handoff_manifest.json`、`replay_check.json` 以及 `predicted_actions.jsonl` 的静态字段和元数据格式进行校验。
  - *已完成*：单元测试 `test_panda_action_adapter.py` 中已包含对 Panda 关节配置文件加载与 PyBullet 载入测试。
  - *待完成*：结合实机就绪度规范，完成 `docs/REAL_MACHINE_READINESS.md` 中对应的字段准入校验。
- **验收标准 (DoD)**：
  - 单元测试 `test_panda_handoff.py` 全绿通过。
  - 尝试加载故意损坏的 handoff 文件（如包含 NaN 或非 Panda 机器人标识），系统必须主动抛出 `ValueError` 并 fail-fast。
  - **非破坏性约束**：本次更改不改动任何现有的 `dist_monitor` / `risk_engine` 历史运行代码。

### 里程碑 2 (RM-M2)：PandaActionAdapter 与逆运动学闭环执行
- **周期**：Day 4 – Day 7
- **目标**：实现从神经网络动作空间指令（任务空间 delta 位姿 + 夹爪宽度）到 PyBullet 关节控制指令的实时解算。
- **关联需求**：RM-SPEC-08（评测脚本接口设计）
- **当前代码基线状态**：
  - *已完成*：`learning/panda_action_adapter.py` 已支持 `hold`、`mock_ik` 以及基于 PyBullet 的 `pybullet_ik` 逆运动学求解，且单元测试均已覆盖。
  - *已完成*：`learning/jsonl_action_replay_policy.py` 已支持按步进读取 Replay。
  - *待完成*：将 `PolicyRunner` 的控制循环和适配器深度挂接，支持在 ROS 2 节点运行时动态实例化与执行。
- **验收标准 (DoD)**：
  - 运行 `PolicyRunner` 热加载动作文件，Panda 机械臂在 PyBullet 中能够按照增量方向连续运动。
  - 在靠近工作空间奇异点或限位时，适配器应能够安全截断输出并向 `/system_health` 发布警告。
  - **非回归约束**：原有 KUKA iiwa7 的 Replay 模式以及 `sine_wave` 压测路径不受影响，单元测试不回归。

### 里程碑 3 (RM-M3)：控制刚度、传动及电气死区补偿控制
- **周期**：Day 8 – Day 10
- **目标**：消除仿真纯刚性控制与实机弹性/死区特性之间的差距，预防指令高频蜂鸣与大死区滞后。
- **具体开发内容**：
  - 在适配器输出端，加入减速器回程误差物理建模（模拟谐波减速器空程）。
  - 设计 **控制死区（Deadband）** 过滤器：对于位置偏差小于 $10^{-4}$ rad 的微调指令不作输出，挂起 PID 积分项，防止电机高频颤震。
  - 设计前馈死区补偿（Feedforward Compensation）：当指令跃出死区时，提供一个阶跃前馈力矩以快速克服静摩擦力。
  - 接入控制器一阶导数（速度）与二阶导数（加速度）的安全限幅（Profile limit）。
- **验收标准 (DoD)**：
  - 示波器或数据曲线显示：关节位置在极微小微扰下保持静止（无颤震），且大范围阶跃响应时起动延迟控制在 $10\text{ms}$ 以内。
  - **非破坏性约束**：死区过滤器只对新 Panda 适配器生效，iiwa7 传统指令直通模式不受干扰。

### 里程碑 4 (RM-M4)：异步多源传感器融合引擎
- **周期**：Day 11 – Day 14
- **目标**：在下游处理实机多源异频数据流的对齐、动态力矩滤波与接触力估计。
- **具体开发内容**：
  - 建立 `sensor_fusion_node`，利用 `message_filters::ApproximateTime` 接收异频观测（相机 30Hz，FT 100Hz，关节角 100Hz）。
  - 基于 Panda 关节位置、加速度前馈及动力学模型，估算夹爪自身运动带来的惯性及重力分量，并在 FT 传感器读数中扣除，计算出真实的**末端净接触力力矩**。
  - 融合关节电流变化率、力矩反馈和触觉形变，输出高可信度的 `/bridge/sim/grasp_status` 话题。
- **验收标准 (DoD)**：
  - 运行单次抓取，在夹爪高速移动和加减速过程中，估计出的净外力矩漂移量不超过实测噪底的 $5\%$。
  - 滑落与抓取判定灵敏度比纯阈值法提升一倍，无误报。

### 里程碑 5 (RM-M5)：真机就绪度规范文档库（D1-D6）落地
- **周期**：Day 15 – Day 22
- **目标**：完成 12-spec 中规划的全部就绪度验证规范及现场实施模板。
- **关联需求**：RM-SPEC-02 至 RM-SPEC-07
- **文档落地顺序说明**（依 12-spec §12 规定的依赖顺序）：
  1. `docs/REAL_MACHINE_READINESS.md` (D1: 定义能不能接真机)
  2. `docs/INTEGRATION_TEST_PLAN.md` (D2: 定义系统集成怎么测)
  3. `docs/SAFETY_ACCEPTANCE_PLAN.md` (D5: 定义失败怎么停和怎么恢复)
  4. `docs/FRAME_AND_CALIBRATION_CHECK.md` (D4: 定义坐标系与手眼标定验收)
  5. `docs/GRASP_EVALUATION_PROTOCOL.md` (D3: 定义抓取与力对称性指标)
  6. `docs/IMPLEMENTATION_REPORT_TEMPLATE.md` (D6: 定义实施报告模板)
- **验收标准 (DoD)**：
  - 所有 6 份文档完全落盘入库，格式规范统一，包含明确的判定标准及证据留存路径字段。

### 里程碑 6 (RM-M6)：HOC 看板重构（多模型评测与 A/B 对比）及多目标分类交互
- **周期**：Day 23 – Day 27
- **目标**：重构人机控制台，支持中游多算法（ACT / Diffusion）、多 Checkpoint 版本的离线验证对比，并集成多目标语言条件分类抓取（Multi-Object Sorting）下发链路。
- **具体开发内容**：
  - **多模型对比看板**：
    - 重构 Settings 面板：增加 Model / Checkpoint 选择框，可动态下发 ROS 服务通知下游 `PolicyRunner` 重新加载不同的策略文件。
    - 重构主界面：添加 `ModelBenchmarkPanel` 指标对比大屏。利用 ECharts 支持横向叠加多组历史运行曲线（对比各模型的 RMSE 轨迹误差、时延分布、累计成功率）。
    - 重构 HTML 报告导出模块，支持将所测模型 Checkpoint 元数据自动嵌入导出产物中。
  - **自然语言任务下发链路**：
    - 升级 `PolicyRunner` 节点：增加并实现 `/risk/set_language_task` ROS 2 服务（使用 `bridge_monitor_msgs/srv/SetLanguageTask`），支持文本指令（如 `"pick up the red box"`）的实时接收与分词 Embedding 向量提取。
    - 升级 `hoc_server` WebSocket 服务端：监听网页端发送的 `trigger_language_task` 消息事件，并调用 ROS 2 服务将任务指令安全转发给 `PolicyRunner`。
    - 重构 HOC React 前端：增加 `TaskControlPanel` 组件，为操作员提供多目标任务菜单和启动策略按钮。
- **验收标准 (DoD)**：
  - 在 HOC 界面中能成功热切换两个不同的 JSONL 动作文件，且图形界面能够清晰重叠显示两组轨迹的实时偏差比对。
  - 在 HOC 前端下发特定抓取指令（如 `"pick up the blue cylinder and place it in the right bin"`），`hoc_server` 能够通过 WebSocket 正确解析并触发 ROS 2 服务请求。
  - `PolicyRunner` 的 `/risk/set_language_task` 服务能够正常响应，输出成功消息并正确记录文本 Embedding 调试日志。

### 里程碑 7 (RM-M7)：自动化评估脚本与进程防错 CI 管线
- **周期**：Day 28 – Day 34
- **目标**：实现无人值守的自动化基准测试，解决进程遗留与 CI 冲突问题。
- **关联需求**：RM-SPEC-08（评估脚本）
- **开发三步分拆**（依 12-spec §8 拆分）：
  - **Phase B (最小脚本骨架)**：实现 `scripts/run_grasp_evaluation.py --dry-run` 模式，不依赖任何物理仿真或真实硬件，能够根据传入的测试计划生成包含 `none / missed_object / no_contact / object_slipped / trajectory_timeout` 等分类的空报告模板。
  - **Phase C (仿真 Trial 接入)**：接入 PyBullet 的抓取与抬升闭环，判定接触与滑动，记录 `/joint_states`、`/bridge/system_state` 等，能够完整跑通 5 个 sim trial。
  - **Phase D (真机接口预留)**：在脚本中校验 `--backend hardware`，输出明确的 `NotImplemented / No-Go` 信号，锁定仿真边界。
- **进程管理加固**：
  - 在 `run_system_validation.sh` 脚本中引入 `trap cleanup EXIT` 及 `lsof -t -i` 强杀机制，确保任何异常中断或测试结束后，不残留任何 ROS、PyBullet GUI 及 node/Vite 进程。
- **验收标准 (DoD)**：
  - 运行自动化测试并故意中途强制中断（发送 `SIGINT` 或 `kill`），用 `ps aux | grep -E "ros2|pybullet|node"` 检查确认无任何后台泄露进程。
  - CI 脚本能在 Headless 容器下完整通过 Panda 烟雾验证管线。

---

## 3. 开发优先级与敏捷分配

### P0 (必须完成 - 决定真机接入的安全下限)
- 里程碑 1 (Panda 语义对齐)
- 里程碑 2 (动作适配器与 IK)
- 里程碑 5 (就绪度文档库：安全 SOP、标定与 Go/No-Go)
- 里程碑 7 (进程清理与 CI 强健性，包含 Phase B/C/D 三步分拆)

### P1 (建议完成 - 决定评测的专业度与演示完整性)
- 里程碑 6 (HOC 看板多模型重构与对比大屏)
- 里程碑 3 (控制死区与 PID 对齐)
- 里程碑 4 (异步传感器融合引擎)

---

## 4. 风险与约束控制 (基于 12-spec)

| 风险项 | 潜在影响 | 缓解措施 |
|:---|:---|:---|
| **Real-Source 概念混淆** | 面试或代码交付时被误解为已完成物理真机 Sim2Real。 | 在 README 及就绪度文档中反复显式声明当前为 Sim-to-Sim 验证，Real 仅指代理源。 |
| **文档过多导致开发失焦** | 产生大量不可执行的死文档，偏离闭环大方向。 | 坚持“每个清单条目必须有 evidence_path”原则，做到文档可审计、可检查。 |
| **抓取评测缺少目标姿态估计** | 无法准确计算 `approach_pose_error` 导致指标缺失。 | 允许将该状态归类为 `sensor_missing`，在报告中记录硬件就绪度缺口。 |
| **进程防错泄露** | 后续 CI 任务因为端口占用、zombie 节点残留导致频繁构建失败。 | 强制在入口 Shell 脚本中使用 `trap` 捕获信号，确保环境绝对干净。 |

---

## 5. 相关文档索引

- 设计总 Spec：[12-real-machine-readiness-spec.md](design/12-real-machine-readiness-spec.md)
- 开发总方案：[13-three-repo-integration-development-plan.md](design/13-three-repo-integration-development-plan.md)
- 机械臂对齐决策：[11-franka-panda-alignment-adr.md](design/11-franka-panda-alignment-adr.md)
- 验收账目：[ACCEPTANCE_GAP.md](ACCEPTANCE_GAP.md)
