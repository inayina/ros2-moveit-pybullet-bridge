# 简历描述整理

本页把 `robot-arm-episode-data-lab` 与 `ros2-moveit-pybullet-bridge` 合并成一个作品集项目描述。建议在简历中按“一个端到端机器人数据与仿真验证平台”呈现，而不是拆成两个孤立项目。

## 推荐项目名称

**机器人操作数据采集与 ROS 2 / MoveIt / PyBullet 联调验证平台**

可选英文名：

**Robot Episode Data Collection and ROS 2 Sim2Real Validation Platform**

## 一句话版本

构建双仓库机器人作品集项目：采集仓库负责 PyBullet 任务数据生成与 LeRobot 格式导出，ROS 2 联调仓库负责 MoveIt 规划闭环、PyBullet 双源仿真、可插拔 Policy Runner、Sim2Real 分布监控、风险急停和 HOC 可视化运维，形成从数据采集到验证报告的可复现实验链路。

## 简历项目描述

**机器人操作数据采集与 ROS 2 / MoveIt / PyBullet 联调验证平台**  
个人项目｜ROS 2 Jazzy、MoveIt 2、PyBullet、LeRobot、React、ECharts、Docker、pytest

- 设计并实现双仓库机器人验证平台：`robot-arm-episode-data-lab` 负责离线 PyBullet episode 采集、任务 FSM 与 LeRobot v2.1 数据导出；`ros2-moveit-pybullet-bridge` 负责 ROS 2 桥接、MoveIt 闭环执行、分布监控、风险管理与 HOC 控制台。
- 打通 MoveIt 2 到 PyBullet 的轨迹执行闭环，将 `FollowJointTrajectory` / JointTrajectory 转换为 PyBullet 控制，并发布 `/joint_states`、双源 joint states、TF 与监控输入；复验中 4/4 MoveGroup joint goals 成功，最大执行 RMSE 0.006004 rad。
- 构建 Sim/Real 双源分布偏移监控链路，支持双 PyBullet 实例、LeRobot 回放与离线对比，使用 KL、Wasserstein-1、MMD 量化关节分布差异；10 Hz 监控输出、5 s 滑窗、3/3 偏移注入检出。
- 实现五维风险管理与安全闭环，将分布偏移、tracking error、安全、通信和系统健康聚合为 R0-R3 风险等级；R3 自动触发 E-stop，速度归零延迟 0.796 ms，HOC acknowledge 后才允许恢复。
- 开发 React + ECharts HOC 运维控制台，展示风险横幅、五维雷达、分布曲线、tracking error、实验控制和报告导出；WebSocket 三路 stream 达到 5 Hz，最大延迟 70.712 ms，支持 JSON/CSV 审计记录。
- 实现可插拔 Policy Runner（`ReplayPolicy` / `SineWavePolicy`），订阅 `/bridge/sim/joint_states`、发布 `/bridge/command`，含 lifecycle、`/system_health` 与故障注入；`run_system_validation.sh` 一键 benchmark 输出 HTML/JSON/CSV，SineWave 策略 mean latency 4.785 ms。
- 建立可复现实验与验收体系，编写 `verify_*.sh`、pytest、HTML 报告与指标导出脚本，覆盖 bridge 通信、MoveIt 闭环、监控、风险、HOC、Policy Runner、性能、安全、可靠性和可维护性；核心包测试 142 passed，coverage 73.2%。

## 精简版项目描述

**机器人操作数据采集与 ROS 2 仿真验证平台**  
个人项目｜ROS 2、MoveIt 2、PyBullet、LeRobot、React

- 搭建双仓库端到端链路：采集仓库生成 PyBullet episode 并导出 LeRobot 数据，ROS 2 仓库完成 MoveIt 规划闭环、PyBullet 双源仿真、Sim2Real 监控和 HOC 运维展示。
- 实现 JointTrajectory 到 PyBullet 的执行桥接，发布 `/joint_states` 与双源状态供 MoveIt、TF、监控和风险引擎消费；复验中 4/4 MoveGroup goals 成功，最大 RMSE 0.006004 rad。
- 基于 KL / W1 / MMD 构建 10 Hz 分布偏移监控，支持 LeRobot 回放、偏移注入和离线报告；3/3 注入检出。
- 实现 R0-R3 风险闭环与 HOC 控制台，R3 自动 E-stop，支持 acknowledge / resume、参数调节和 JSON/CSV 报告导出。
- 实现 Policy Runner 策略抽象与系统 benchmark，`run_system_validation.sh` 生成可审计验证报告。

## 如果简历空间很短

- 搭建机器人数据采集与 ROS 2 仿真验证双仓库项目，串联 PyBullet episode 采集、LeRobot 导出、MoveIt 规划闭环、Sim2Real 分布监控、R0-R3 风险急停和 React HOC 控制台。
- 使用 KL / W1 / MMD 实现 10 Hz 双源偏移监控，支持偏移注入、LeRobot 回放和 HTML/JSON/CSV 报告；MoveIt 闭环复验 4/4 成功，最大 RMSE 0.006004 rad。

## 中文面试口述版

这个项目我按两个仓库来组织：第一个仓库做机器人操作数据采集，基于 PyBullet 生成 episode，并导出 LeRobot 格式数据；第二个仓库是 ROS 2 联调验证环境，把 MoveIt 2 规划结果接到 PyBullet 执行，同时发布双源关节状态给分布监控和风险引擎。在 MoveIt 之外我还做了可插拔 Policy Runner，用 Replay / SineWave 策略驱动同一套 bridge 和监控链路，并有一键 benchmark 验证。监控层用 KL、W1、MMD 判断 Sim/Real 偏移，风险层聚合成 R0-R3，R3 会触发急停，HOC 控制台负责可视化、控制和报告导出。它不是单个动画 demo，而是一条从数据采集、仿真执行、策略运行、偏移检测、风险处置到验收报告的完整工程链路。

## 英文简历版本

**Robot Episode Data Collection and ROS 2 Sim2Real Validation Platform**  
Personal Project｜ROS 2 Jazzy, MoveIt 2, PyBullet, LeRobot, React, ECharts, Docker, pytest

- Built a two-repository robotics validation platform: `robot-arm-episode-data-lab` generates PyBullet manipulation episodes and exports LeRobot datasets, while `ros2-moveit-pybullet-bridge` provides ROS 2 integration, MoveIt execution, Sim2Real monitoring, risk control, and an HOC dashboard.
- Implemented a MoveIt 2 to PyBullet execution bridge that converts JointTrajectory commands into physics simulation control and publishes `/joint_states`, dual-source joint states, TF, and monitoring inputs; validation achieved 4/4 successful MoveGroup goals with max RMSE of 0.006004 rad.
- Developed a dual-source distribution monitoring pipeline using KL divergence, Wasserstein-1, and MMD for Sim/Real drift detection, supporting PyBullet domain randomization, LeRobot replay, offline comparison, and 10 Hz online metrics.
- Implemented an R0-R3 risk management loop with five-dimensional attribution, automatic E-stop on R3, acknowledge-gated recovery, and JSON/CSV audit export through a React + ECharts HOC dashboard.
- Implemented a pluggable PolicyRunner with ReplayPolicy and SineWavePolicy, lifecycle management, `/system_health` diagnostics, and fault injection; `run_system_validation.sh` produces HTML/JSON/CSV benchmark reports (SineWave mean latency 4.785 ms).
- Added reproducible verification scripts and reports covering bridge latency, MoveIt closure, monitoring, risk, HOC, Policy Runner, performance, safety, reliability, and maintainability; core package tests reached 142 passed with 73.2% coverage.

## 表述边界

简历和面试中建议保持以下边界：

- 可以说“LeRobot 数据回放 / 外部采集仓库联调”，不要说“已完成真机接入”。
- 可以说“MoveIt 到 PyBullet 的 `FollowJointTrajectory` relay 闭环”，不要说“完整 `ros2_control` 硬件接口已完成”。
- 可以说“短时 smoke、脚本化验收和样例报告已完成”，不要把 2 小时长稳说成已完成。
- 可以说“可插拔 Policy Runner 与系统 benchmark”，不要说“已完成 RL 训练或模型部署”。
- 可以强调“可复现工程链路”，少强调单次视觉效果。
