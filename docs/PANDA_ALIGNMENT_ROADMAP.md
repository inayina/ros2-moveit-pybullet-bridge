# Panda Alignment Roadmap

本文档说明 `ros2-moveit-pybullet-bridge` 当前 KUKA iiwa7 后端与作品集长期 Franka Panda schema 的关系。这里的目标是明确边界和迁移路线，不在当前阶段重写 PyBullet 后端，也不把现有 iiwa7 demo 直接替换成 Panda。

## 1. 当前后端

当前仓库使用 KUKA iiwa7 作为早期 MoveIt-PyBullet 验证后端。它的职责是证明 ROS 2 / MoveIt / PyBullet / 监控 / 运维链路可以端到端跑通，包括：

- `FollowJointTrajectory` relay；
- MoveIt -> PyBullet 执行链；
- Sim / Real 双源 PyBullet；
- KL / W1 / MMD 分布偏移监控；
- risk engine；
- HOC 控制台。

因此，当前 iiwa7 后端会继续保留为 legacy validation backend。它用于维护可运行 demo、复验脚本和面试讲解中的 MoveIt-PyBullet 闭环证据，不作为作品集未来统一操作臂 schema 的终态。

## 2. 统一目标

作品集主线操作臂 schema 将逐步统一到 Franka Panda，覆盖：

- MuJoCo teleoperation；
- `robot-arm-episode-data-lab` training；
- 未来 PyBullet bridge backend。

统一后的目标是让 teleoperation、episode 数据、policy action、LeRobot replay、PyBullet bridge 和 Sim/Real 监控共享同一套 observation/action schema，减少跨仓库 joint names、action dimension 和数据解释上的转换成本。

## 3. 为什么不立刻切换

当前阶段不直接把 iiwa7 backend 改成 Panda，原因是：

- 避免破坏当前可运行的 MoveIt / PyBullet demo；
- 当前优先级是 schema 统一和面试表达清晰；
- 完整 Panda backend 迁移放到 Phase-2。

换句话说，当前仓库先把“已有链路能跑、边界讲清楚、未来方向一致”做好。Panda backend 是后续增强，不阻塞当前 portfolio demo、分布监控和 HOC 展示。

## 4. Phase-2 迁移步骤

未来 Panda backend 迁移计划包括：

1. 新增 Panda URDF / SRDF / MoveIt config；
2. 新增 Panda joint name mapping；
3. 新增 Panda PyBullet loader；
4. 将 `/bridge/command` action dimension 对齐 Panda schema；
5. 支持读取 `robot-arm-episode-data-lab` 导出的 `predicted_actions.jsonl`；
6. 用 Panda sim-source vs real-source 验证 KL / W1 / MMD 和 tracking error。

这些步骤应作为独立 Phase-2 任务推进，并在迁移期间继续保留 iiwa7 legacy backend，避免把 schema 调整、机器人模型迁移和监控验证混在一次高风险改动中。

## 5. 边界说明

当前 `Real-Source` 不是真实机械臂。

当前 `Real-Source` 指以下两类真实世界代理：

- randomized PyBullet source；
- LeRobot replay source。

物理真机支持需要额外工程工作，包括：

- `ros2_control` hardware interface 或厂商 SDK；
- 机器人、夹爪、相机和工作空间标定；
- 关节限位、速度限制、急停和碰撞保护等安全验证；
- 低速、分阶段 bring-up，从单关节、空载轨迹、受限工作空间逐步过渡到完整任务。

因此，在当前版本中可以说 bridge 已经完成 MoveIt-PyBullet 闭环、双源分布监控和 LeRobot replay 联动；不应把 `Real-Source` 表述成已经接入真实 Panda 或真实 iiwa7 机械臂。
