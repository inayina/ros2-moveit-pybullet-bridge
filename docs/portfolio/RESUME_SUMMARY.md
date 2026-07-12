# 简历描述整理

本页把上游 `ros2-arm-teleoperation-suite`、中游 `robot-arm-episode-data-lab` 与下游 `ros2-moveit-pybullet-bridge` 合并成一个完整的三仓闭环作品集项目描述。建议在简历中按“工业级机器人遥操作与 Sim2Real 联调验证平台”呈现。

## 推荐项目名称

**工业级机器人遥操作数据采集与 ROS 2 / Sim2Real 联调验证平台**

可选英文名：

**Industrial Robotic Teleoperation, Data Collection & ROS 2 Sim2Real Validation Platform**

## 一句话版本

构建三仓联动机器人闭环验证平台：上游负责 L0-L7 遥操作与 MuJoCo 视触觉数据录制，中游负责数据适配、质检与 ACT 策略训练，下游负责策略重放、Sim2Real 分布偏移监控、五维风险急停联动以及 React + ECharts HOC 前端可视化运维。

## 简历项目描述

**工业级机器人遥操作数据采集与 ROS 2 / Sim2Real 联调验证平台**  
个人项目｜ROS 2 Jazzy、MoveIt 2、ros2_control、CANopen DS402、MuJoCo、React、ECharts、Docker、pytest、ACT

- **三仓全链路闭环系统架构**：设计并打通上游遥操作采集（MuJoCo v3）、中游数据适配训练（LeRobot 格式/ACT 算法）与下游回放验证（PyBullet 物理引擎）的三仓数据流与运行观测环。
- **上游工业级控制总线与采集**：上游实现 L0 遥操作驱动（键盘/手柄）、L1 C++ 独立安全监测、L2 MoveIt Servo 笛卡尔伺服、L3 1kHz 阻抗控制、L4 CANopen DS402 虚拟总线驱动，最终通过多模态录制生成 LeRobot 兼容数据集。
- **中游数据适配与离线训练**：中游实现数据状态与动作的语义转换（Adapter），完成 schema 规约校验与数据集质检（Inspector），离线进行行为克隆（ACT）模型训练并打包为 handoff 交付件。
- **下游双源监控与偏移量化**：下游构建 Sim/Real 双源分布偏移监控链路，利用 KL 散度、Wasserstein-1 距离和 MMD 量化关节与控制分布差异，实现 10 Hz 在线输出、5 s 滑窗和 3/3 偏移注入报警检出。
- **五维风险决策与安全闭环**：聚合分布偏移、跟踪误差、物理碰撞、通信抖动和系统健康至 R0-R3 风险等级；R3 自动触发 E-stop 保护，速度归零延迟仅 0.796 ms。
- **前端 HOC 可视化运维控制台（作品集主亮点）**：基于 **React + ECharts** 搭建 HOC 运维浏览器控制台，通过 WebSocket 5 Hz 实时渲染五维风险雷达、分布偏移曲线、三路数据流与相机预览，实现人机协同的异常确认（Acknowledge）与复位（Resume）闭环控制。
- **测试与验证体系**：编写 `verify_*.sh` 与 pytest 覆盖全栈功能与非功能（NFR）验收，核心包通过 142 项测试，单元/节点测试覆盖率达 73.2%，PolicyRunner 推理平均时延 4.785 ms。

## 精简版项目描述

**机器人遥操作采集与 ROS 2 仿真验证平台**  
个人项目｜ROS 2、MoveIt 2、PyBullet、LeRobot、React、ECharts

- 搭建三仓端到端闭环：上游负责键盘/手柄遥操作与多模态数据录制，中游完成适配质检与策略训练，下游实现回放执行、分布偏移监控与 HOC 控制台展示。
- 基于 KL / W1 / MMD 实现 10 Hz 双源分布偏移监控，支持偏置注入与离线报告生成；3/3 注入检出。
- 研发 React + ECharts HOC 运维控制台，通过 WebSocket 实现风险态势可视化、异常 acknowledge 复位互锁及 JSON/CSV 报告导出。
- 建立 R0-R3 风险决策机制与 PolicyRunner 策略执行抽象，在 R3 触发自动急停与 degraded 降速保护。

## 如果简历空间很短

- 研发工业级机器人遥操作、多模态数据采集与 ROS 2 验证平台。打通上游遥操作采集（MuJoCo）、中游训练（ACT）与下游回放监控（PyBullet）三仓闭环。
- 设计并开发基于 React + ECharts 的 HOC 可视化控制台，实现 Sim2Real 分布偏移（KL/W1/MMD）、五维风险状态的实时观测与异常 acknowledge 复位互锁，沉淀可审计验证报告。

## 中文面试口述版

这个项目我按三仓闭环来组织：上游是遥操作采集仓，实现了 Teleop $\rightarrow$ 安全监视 $\rightarrow$ MoveIt Servo $\rightarrow$ ros2_control 阻抗控制 $\rightarrow$ CANopen DS402 虚拟总线 $\rightarrow$ MuJoCo 物理与多模态数据录制的七层软硬件架构；中游是数据加工和训练仓，做格式转换和质检后训练 ACT 行为克隆模型；下游是回放与监控验证仓，重放动作包的同时，基于双源数据在在线运行 KL、W1、MMD 检测 Sim/Real 分布偏移，聚合成五维风险 R0-R3。我们在下游做了一个 React + ECharts 的前端 HOC 运维控制台，通过 WebSocket 把这些漂移指标、雷达图和相机画面实时渲染出来，并提供了 Acknowledge 复位控制逻辑。通过这个控制台，能够直观地演示“注入偏置 $\rightarrow$ 警报触发 $\rightarrow$ 急停锁死 $\rightarrow$ 人工确认复位”的安全闭环。

## 英文简历版本

**Industrial Robotic Teleoperation, Data Collection & ROS 2 Sim2Real Validation Platform**  
Personal Project | ROS 2 Jazzy, MoveIt 2, ros2_control, CANopen DS402, MuJoCo, React, ECharts, Docker, pytest, ACT

- **Three-Repo Closed-Loop Architecture**: Designed and integrated a closed-loop robot pipeline connecting upstream teleoperation (MuJoCo v3), midstream adaptation/training (LeRobot/ACT), and downstream replay validation (PyBullet).
- **Upstream Teleoperation & Recording**: Implemented pluggable teleop input (L0), C++ safety monitor (L1), MoveIt Servo (L2), 1kHz Cartesian impedance control (L3), and virtual CANopen DS402 servo bus (L4) in MuJoCo, exporting multi-modal datasets.
- **Downstream Sim/Real Drift Monitoring**: Built a dual-source distribution monitoring pipeline using KL divergence, Wasserstein-1, and MMD for online Sim2Real drift detection, supporting 10 Hz metric updates and 5 s sliding window.
- **Multi-Dimensional Risk Management**: Aggregated distribution shift, tracking error, safety, communication, and health metrics into R0-R3 risk levels, triggering automatic E-stop on R3 with a velocity zeroing latency of 0.796 ms.
- **HOC Web Cockpit (Portfolio Highlight)**: Developed a **React + ECharts** HOC dashboard, rendering 5 Hz real-time radar charts, distribution curves, and camera feeds via WebSockets, allowing human-in-the-loop Acknowledge & Resume controls.
- **System Verification & Benchmarking**: Created reproducible verification suites and benchmark scripts; achieved 142 passed tests with 73.2% coverage for the core bridge package, with policy inference latency averaging 4.785 ms.

## 表述边界

在简历和面试中建议保持以下真实性边界，防止 Overclaim：

- 可以说“LeRobot 数据回放 / 外部采集与适配仓联调”，不要说“已部署真机”。
- 可以说“MuJoCo 中 CANopen DS402 总线仿真与 ros2_control 硬件接口重构”，不要说“实体总线硬件联调已全部拉起”。
- 可以说“通过 PolicyRunner 实现了可插拔 Replay 策略重放与在线监控”，不要宣称“已实现稳定在线自主抓取”。
- 强调系统的**“可观测性”**、**“工程验证与测试”**以及**“安全硬化闭环”**作为核心卖点。
