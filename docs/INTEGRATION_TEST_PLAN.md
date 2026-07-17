# 系统集成测试矩阵 (docs/INTEGRATION_TEST_PLAN.md)

**文档版本**：v1.0
**状态**：Released
**关联里程碑**：RM-M3 / RM-SPEC-03

本文档定义了真实机械臂接入硬件前系统集成的验证用例矩阵，确保控制链路、状态发布、轨迹限制与安全保护动作行为符合预期。

---

## 测试矩阵总览

| ID | 测试项 (Test Item) | 验证手段 | 通过标准 |
|---|---|---|---|
| IT-01 | **接口契约校验** (Static Interface Check) | ROS 2 Topic/Service/Action 内省 | 消息类型和参数定义完全匹配 ICD |
| IT-02 | **关节顺序与硬限位校验** (Joint Limits Check) | 配置数据读取与解析 | URDF 关节数量/名称及速度限幅加载正确 |
| IT-03 | **单轴运动测试** (Single Joint Motion) | `/bridge/command` 单关节指令注入 | 单关节响应正确，无其他关节随动 |
| IT-04 | **多轴联动测试** (Multi-Joint Trajectory) | `/bridge/command` 多关节插值指令 | 轨迹平滑，无高频蜂鸣或卡顿现象 |
| IT-05 | **MoveIt规划轨迹测试** (MoveIt Trajectory) | 运行 `verify_moveit_closure.sh` | 规划成功率 100%，末端跟踪偏差符合精度 |
| IT-06 | **策略Replay闭环测试** (Policy Replay) | 运行 `run_system_validation.sh` | head-less 模式下 policy Replay 100% 成功 |
| IT-07 | **夹爪开合测试** (Gripper Open/Close) | `/bridge/command` 夹爪指令注入 | 夹爪开合宽度位置输出及状态反馈一致 |
| IT-08 | **完整抓取动作测试** (Pick-Lift-Place) | 运行 `verify_pick.sh` | 物理引擎成功拾取物体，无掉落且无异常接触 |
| IT-09 | **故障注入测试** (Fault Injection) | PolicyRunner 随机注入超载/时延 | `risk_engine` 正确评估并触发降级或 Hold 状态 |
| IT-10 | **自愈与手动恢复测试** (Recovery Test) | 熔断后触发 `/bridge/reset_simulation` | 系统清空异常标记，可重新开始正常接收指令 |

---

## 详细测试用例规格

### IT-01: Static Interface Check
* **Objective**: 验证系统内主要 Topic, Service, Action 与 ICD 规范 the 契约定义是否完全匹配。
* **Preconditions**: ROS 2 主节点与监控节点正常带电启动。
* **Command**:
  ```bash
  ros2 topic list
  ros2 service list
  ros2 action list
  ```
* **Expected result**: 控制接口 `/bridge/command`，监控接口 `/bridge/sim/joint_states`、`/monitor/distribution_metrics`，安全接口 `/risk/status` 及服务存在。
* **Metrics**: 接口存在率。
* **Pass criteria**: 所有接口定义无缺失。
* **Evidence path**: `<ros2_ws>/reports/it_01_interface.log`
* **Risk if failed**: 系统节点启动后无法建立通信连接，导致系统异常瘫痪。
* **Next action**: 调整 package.xml 中的依赖关系与 launch 启动节点声明。

### IT-02: Joint Limits Check
* **Objective**: 验证机械臂关节名称、排列顺序与速度、加速度硬限位等安全参数被正确加载。
* **Preconditions**: `pybullet_bridge` 节点已初始化。
* **Command**:
  ```bash
  ros2 param get /pybullet_bridge robot_profile
  ```
* **Expected result**: 显示加载对应机器人（如 `panda`）的关节参数与限幅设置。
* **Metrics**: 参数读取正确率。
* **Pass criteria**: 关节名称与数目匹配，限位值符合 profile。
* **Evidence path**: `<ros2_ws>/reports/it_02_limits.json`
* **Risk if failed**: 关节名称不匹配或限位缺失会导致轨迹计算失常、奇异或超出实机机械极限。
* **Next action**: 修改 [robot_profiles.py](../pybullet_bridge/pybullet_bridge/robot_profiles.py) 对齐参数定义。

### IT-03: Single Joint Motion
* **Objective**: 验证向特定单关节发布指令时，机械臂各关节运动及反馈的独立性。
* **Preconditions**: 桥接节点处于 RUNNING 状态，机械臂处于 Home 位姿。
* **Command**:
  注入单关节小范围目标点（例如控制 joint1 步进 0.1 rad）。
* **Expected result**: 对应关节响应运动，其余关节保持锁死。
* **Metrics**: 非目标关节的最大偏移量 < $10^{-4}$ rad。
* **Pass criteria**: 只有目标关节随动，且反馈状态正常。
* **Evidence path**: `<ros2_ws>/reports/it_03_single_joint.csv`
* **Risk if failed**: 控制通道映射错乱会导致机械臂失控碰撞。
* **Next action**: 重新标定关节控制的 index 对应关系。

### IT-04: Multi-Joint Trajectory
* **Objective**: 验证多轴联动下，插补计算的顺畅度以及过滤器的处理时效性。
* **Preconditions**: 机械臂多轴正常上电，轨迹无奇异。
* **Command**:
  发布平滑的多轴联动 JointTrajectory。
* **Expected result**: 机械臂顺滑过度，无奇异点卡顿。
* **Metrics**: 关节响应时滞 < 50ms。
* **Pass criteria**: 机械臂以预定轨迹完成运动，无抖动与鸣颤。
* **Evidence path**: `<ros2_ws>/reports/it_04_multi_joint.csv`
* **Risk if failed**: 联动轨迹突变产生力矩过载，烧毁实机驱动器或触发硬件紧急制动。
* **Next action**: 调整动作适配器的导数限幅与死区值。

### IT-05: MoveIt Planned Trajectory
* **Objective**: 验证 MoveIt 2 规划的轨迹能够无损下发并在仿真中成功收敛。
* **Preconditions**: MoveIt 节点与仿真桥接就绪。
* **Command**:
  ```bash
  bash scripts/verify_moveit_closure.sh
  ```
* **Expected result**: 规划生成轨迹，仿真桥读取并跟踪，到达目标点。
* **Metrics**: 跟踪 RMSE < 0.01 rad。
* **Pass criteria**: `verify_moveit_closure.sh` 脚本输出为 SUCCESS。
* **Evidence path**: `<ros2_ws>/reports/it_05_moveit.log`
* **Risk if failed**: 规划与控制解耦不良，实机无法闭环运行复杂长轨迹。
* **Next action**: 调整 MoveIt controller manager 与仿真器之间的轨迹下发时间步长。

### IT-06: Policy Replay Trajectory
* **Objective**: 验证 PolicyRunner 能成功加载手风琴 Replay 包，并进行实时动作适配。
* **Preconditions**: Handoff 动作文件存在。
* **Command**:
  ```bash
  bash scripts/run_system_validation.sh
  ```
* **Expected result**: PolicyRunner 输出控制指令，轨迹无越界。
* **Metrics**: 系统验证成功率 100%。
* **Pass criteria**: 自动化 Replay 测试无 failure。
* **Evidence path**: `<ros2_ws>/reports/it_06_policy.log`
* **Risk if failed**: 模型预测指令下发导致控制中断或超界，损坏设备。
* **Next action**: 检查 Handoff 动作的格式完整性与 adapter 配置。

### IT-07: Gripper Open/Close
* **Objective**: 验证夹爪在开合时的动作控制与状态反馈通路。
* **Preconditions**: 桥接节点与仿真就绪。
* **Command**:
  发布包含夹爪开度指令（如 0.0m 与 0.04m）的动作包。
* **Expected result**: 夹爪在仿真中产生位移，且反馈观测值一致。
* **Metrics**: 夹爪开度跟踪误差 < 1mm。
* **Pass criteria**: 成功合拢和张开，无命令丢失。
* **Evidence path**: `<ros2_ws>/reports/it_07_gripper.csv`
* **Risk if failed**: 抓取动作失败或由于闭合命令丢失导致物体掉落。
* **Next action**: 修复 [panda_action_adapter.py](../pybullet_bridge/pybullet_bridge/learning/panda_action_adapter.py) 中夹爪位置映射。

### IT-08: Pick-Lift-Place
* **Objective**: 验证由动作适配器和物理仿真组成的端到端抓取-抬升-放置完整管线。
* **Preconditions**: 仿真场景中有抓取物体，初始位姿正常。
* **Command**:
  ```bash
  bash scripts/verify_pick.sh
  ```
* **Expected result**: 顺利抓取、抬升、运送并放置，状态监控输出 success。
* **Metrics**: 抓取成功率。
* **Pass criteria**: 拾取率符合预期，并且 `verify_pick.sh` 返回成功。
* **Evidence path**: `<ros2_ws>/reports/it_08_pick.log`
* **Risk if failed**: 无法正常获取数据样本或导致仿真假成功样本混入。
* **Next action**: 调整抓取补偿偏置与力矩限幅。

### IT-09: Fault Injection
* **Objective**: 验证网络时延或状态数据污染时，系统的安全熔断响应。
* **Preconditions**: 仿真桥接与 PolicyRunner 在线，开启故障注入。
* **Command**:
  在 `PolicyRunner` 参数中配置 `fault_injection_enabled:=true` 注入大延迟。
* **Expected result**: `risk_engine` 检测到状态不一致或延迟严重，发布 E-Stop。
* **Metrics**: 安全响应延时 < 100ms。
* **Pass criteria**: 触发 R3 警报，轨迹被截断且机械臂 HOLD。
* **Evidence path**: `<ros2_ws>/reports/it_09_fault.log`
* **Risk if failed**: 遇到硬件死机或 DDS 异常时，实机无法紧急制动导致失控。
* **Next action**: 优化 `risk_engine` 的看门狗评估周期。

### IT-10: Recovery Test
* **Objective**: 验证系统发生软件熔断或 E-Stop 后的手动恢复与重置。
* **Preconditions**: 系统处于 HOLD / E-STOP 状态。
* **Command**:
  调用 `/bridge/reset_simulation` 触发器服务。
* **Expected result**: 警报解除，系统重置为 Home，恢复 RUNNING。
* **Metrics**: 重置时间。
* **Pass criteria**: 清空警报标志且可重新被控制。
* **Evidence path**: `<ros2_ws>/reports/it_10_recovery.log`
* **Risk if failed**: 报警无法复位导致系统死锁，或复位不当导致二次冲击。
* **Next action**: 重新编写系统状态重置状态机逻辑。
