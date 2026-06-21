# 接口控制文件（ICD）

**范围**：PyBullet bridge、Policy Runner、分布监控、风险引擎与 HOC 控制台之间的 ROS 2 topic 契约。

---

## Topic 接口表

| 话题名称 | 类型 | 发布者 | 订阅者 | 频率 | QoS 策略 | 数据范围约束 |
|----------|------|--------|--------|------|----------|--------------|
| `/bridge/command` | `trajectory_msgs/msg/JointTrajectory` | `joint_trajectory_controller`, `policy_runner` | `pybullet_bridge` | 事件驱动；policy 模式 10-100 Hz | Reliable, depth 10 | `joint_names` 必须匹配 robot profile；`positions` 单位 rad，位于 joint limits 内；`time_from_start >= 0`。 |
| `/joint_states` | `sensor_msgs/msg/JointState` | `pybullet_bridge` | `robot_state_publisher`, MoveIt PlanningSceneMonitor | 100 Hz | BestEffort / SensorDataQoS | name/order 与 URDF 一致；position rad，velocity rad/s，effort Nm。 |
| `/bridge/sim/joint_states` | `sensor_msgs/msg/JointState` | `pybullet_bridge` | `dist_monitor`, `policy_runner`, HOC bridge | 100 Hz | BestEffort / SensorDataQoS | Sim 源关节状态；header stamp 单调递增；数组长度等于 active joints。 |
| `/bridge/real/joint_states` | `sensor_msgs/msg/JointState` | `real_source` / domain-randomized PyBullet / LeRobot replay | `dist_monitor`, HOC bridge | 100 Hz 目标频率 | BestEffort / SensorDataQoS | joint order 必须与 sim 源可对齐；缺失关节必须在 source adapter 层处理。 |
| `/bridge/sim_real_error` | `sensor_msgs/msg/JointState` 或现有误差消息 | `pybullet_bridge` / `dist_monitor` | `risk_engine`, HOC bridge | 10-100 Hz | Reliable, depth 10 | position 表示 per-joint error rad；用于 tracking RMSE。 |
| `/bridge/performance` | `diagnostic_msgs/msg/DiagnosticArray` | `pybullet_bridge` | HOC bridge, diagnostics tools | 1-10 Hz | Reliable, depth 10 | 包含 physics step、publish loop、real-time factor 等诊断 key。 |
| `/bridge/system_state` | `std_msgs/msg/String` | `pybullet_bridge` | `risk_engine`, HOC bridge | 事件驱动 | Reliable, depth 10 | 枚举：`RUNNING`, `HOLD`, `PAUSED`, `E_STOP`, `IDLE`；不得改为 JSON。 |
| `/monitor/distribution_metrics` | `bridge_monitor_msgs/msg/DistributionMetrics` | `dist_monitor` | `risk_engine`, HOC bridge, `policy_runner` | 5-10 Hz | Reliable, depth 10 | KL/W1/MMD 非负；`shift_detected` 表示阈值判定；window metadata 与配置一致。 |
| `/monitor/tracking_error` | `diagnostic_msgs/msg/DiagnosticArray` 或现有 tracking 消息 | `dist_monitor` | `risk_engine`, HOC bridge | 5-10 Hz | Reliable, depth 10 | RMSE、max error 非负；单位 rad 或按 key 标注。 |
| `/monitor/comm_health` | `diagnostic_msgs/msg/DiagnosticArray` | `dist_monitor` | `risk_engine`, HOC bridge | 1 Hz | Reliable, depth 10 | measured_hz >= 0；latency_ms >= 0；gap_count >= 0；score in [0, 1]。 |
| `/system_health` | `diagnostic_msgs/msg/DiagnosticArray` | `policy_runner`, benchmark hooks | `risk_engine`, HOC bridge, diagnostics tools | 1 Hz 或事件驱动 | Reliable, optional transient local | level 使用 OK/WARN/ERROR；必须包含 `strategy_type`, `inference_latency_ms`, `reason`。 |
| `/risk/status` | `bridge_monitor_msgs/msg/RiskStatus` | `risk_engine` | HOC bridge, `pybullet_bridge` | 1-10 Hz | Reliable, depth 10 | risk level R0-R3；原因列表应可解释且稳定。 |
| `/risk/alerts` | `diagnostic_msgs/msg/DiagnosticArray` 或现有 alert 类型 | `risk_engine` | HOC bridge, operator tools | 事件驱动 | Reliable, depth 10 | 严重报警必须包含来源维度、阈值和建议动作。 |

## 服务与 Action 边界

| 名称 | 类型 | 提供者 | 使用者 | 约束 |
|------|------|--------|--------|------|
| `/bridge/reset_simulation` | service | `pybullet_bridge` | HOC, validation script | reset 后应重新发布初始 joint state。 |
| `/bridge/set_mode` | service | `pybullet_bridge` | HOC, risk workflow | mode 改变不得绕过 E-stop 互锁。 |
| `/monitor/reset_baseline` | service | `dist_monitor` | HOC, benchmark | benchmark episode 边界可调用，保证指标可复现。 |
| `/risk/acknowledge` | service | `risk_engine` | HOC | 只确认已处理报警，不直接解除底层故障。 |
| `/arm_controller/follow_joint_trajectory` | action | `joint_trajectory_controller` | MoveIt | 规划链路入口；PolicyRunner 不使用该 action，直接发布 `/bridge/command`。 |

## 版本与兼容性规则

- 新增字段优先放在新 topic 或 `DiagnosticArray` key-value 中，避免破坏自定义 msg ABI。
- `/bridge/system_state` 保持简单字符串枚举，已有 HOC 与 risk_engine 依赖该语义。
- 高频状态 topic 使用相同 joint ordering；若数据源缺关节，必须在 adapter 层补齐或显式失败。
- 控制命令与健康报警使用 Reliable；状态采样使用 BestEffort。
