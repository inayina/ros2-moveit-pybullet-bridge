# 11 · Franka Panda 机械臂对齐架构决策记录 (ADR)

**文档版本**：v1.0  
**状态**：已接受 (Accepted)  
**依赖**：[06 · 机器人平台选型评估](./06-robot-platform-selection.md)、[08 · 双仓库通盘集成设计](./08-dual-repo-portfolio-integration-spec.md)  
**关联仓库**：
- `ros2-moveit-pybullet-bridge`
- `robot-arm-episode-data-lab`
- `ros2-arm-teleoperation-suite`

---

## 1. 背景与上下文 (Context)

在作品集项目（V2 版本）中，我们的核心叙事是一条完整的数据与控制闭环：
`遥操作数据采集 (MuJoCo) -> 数据校验与 LeRobot 导出 (Data-Lab) -> 离线/在线策略训练 (ACT) -> 下游轨迹回放与 Sim2Real 双源监控 (MoveIt + PyBullet Bridge)`。

然而，在实现该链条时，系统存在以下不一致问题：
1. **主线机型分裂**：上游采集与中游训练已标准化使用 **Franka Panda（7-DOF）** 机械臂，但下游 bridge 的默认配置、 sinusoidal 运动演示（`iiwa_motion_demo`）以及 launch 文件中大量硬编码了 **KUKA iiwa7** 的关节定义和 Home 位姿。
2. **调试开销大**：缺少一键式 Headless 运行 Franka Panda 仿真的 Docker 入口，限制了面试官或外部评审在无硬件/无图形界面环境下的快速验证。

为了确保三仓库叙事的一致性与技术闭环的严密性，下游 bridge 必须对齐 Franka Panda 机械臂。

---

## 2. 决策决策 (Decision)

我们做出了以下架构与代码设计决策：

### 决策 2.1：统一 Franka Panda 为主线 profile，保留 KUKA iiwa7 作为 Legacy Fallback
- 在 `pybullet_bridge` 的 `ROBOT_PROFILES` 注册表及 `dist_monitor` 的监控配置中引入 `panda` 机器人型态，支持自动通过 `pybullet_data` 寻找并载入内置的 `franka_panda/panda.urdf`。
- 保留 `iiwa7` 与 `planar_2dof` 配置文件以实现平滑降级，确保现有的 CI 自动化测试和历史演示在零回归的状态下继续运行。

### 决策 2.2：重构仿真驱动与启动链路为“参数自适应”架构
- 消除 `iiwa_motion_demo.py` 中对 KUKA 关节数量、名称及 Home 状态的硬编码，引入通用的 `home_positions` 与 `joint_names` 参数。
- 重构 `portfolio_demo.launch.py`，使 `bridge_node`、`robot_state_publisher`、`manipulation_node` 等关键节点的物理/控制参数，全部从当前激活的 `robot_profile` 动态解析获取。
- 在 `hoc_experiment.launch.py` 一键启动器中添加 `robot_profile` 参数传递，实现“一个命令拉起全栈（包含前端 HOC 和指定机械臂物理后端）”。

### 决策 2.3：将 Robot Profile 信息下沉至 `dist_monitor` 归一化层
- 扩展 `normalize_joint_names` 函数以接收 `robot` 参数。
- 当 `robot='panda'` 时，规范化规则自动识别 `panda_joint1` ~ `panda_joint7` 并映射 parallel finger 关节；而对于 legacy KUKA 流程，归一化默认逻辑维持原有 iiwa7 的映射规则，避免跨仓库 LeRobot 数据回放时因关节名称冲突导致 MMD 监控崩溃。

### 决策 2.4：补充容器化 DEMO 入口与复现耗时标注
- 在 `docker-compose.yml` 中新增 `portfolio-panda-demo` 服务，支持 headless 环境下的一键 Panda 仿真闭环。
- 在三仓库的 README.md 顶部增加统一的 `Estimated Replication Time`（估计复现时间）徽章，并将 Midstream 的 Google Colab 一键运行徽章置顶，极大降低面试官的体验门槛。

---

## 3. 决策后果 (Consequences)

### 正面后果 (Pros)
1. **完整闭环一致性**：实现了上游 teleop、中游 training 和下游 evaluation 在 Franka Panda 机型上的全链路对齐，消除了 7-DOF 数据与 bridge 维度不一致的工程痛点。
2. **高内聚低耦合**：演示节点（`iiwa_motion_demo`）和启动脚本升级为泛型参数架构，后续添加新型号机械臂（如 UR5 或定制臂）仅需在 `robot_profiles.py` 注册，无需修改节点逻辑。
3. **零回归风险**：旧有的 KUKA iiwa7 CI 链和测试点通过 fallback 路径被 100% 保留且全部测试通过（73/73 passed）。
4. **面试演示友好**：通过 `robot_profile:=panda` 命令行参数和新增的 Docker Compose 镜像，能够在 3 分钟内向面试官展示闭环。

### 负面后果 (Cons)
1. **配置碎片**：需要同时维护 `panda` 与 `iiwa7` 两套关节限位（`joint_limits.yaml`）和映射规则，但通过统一的 `joint_names.py` 路由已将该影响局限在配置层面。
