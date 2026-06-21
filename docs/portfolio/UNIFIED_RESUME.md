# 统一简历描述 · 五仓系统工程作品集

**主投**：机器人系统工程 / 平台集成 / 仿真验证  
**更新**：2026-06-21  
**关联**：[MASTER_PORTFOLIO_PLAN.md](./MASTER_PORTFOLIO_PLAN.md)

---

## 推荐项目名称（简历只列 1 个项目）

**机器人系统集成与验证平台**  
（AMR 运维 Dashboard + 操作臂 Sim2Real 联调 + 数据采集）

**英文**：Robotics Systems Integration & Validation Platform  

**技术栈一行**：ROS 2 Jazzy · MoveIt 2 · Nav2 · FastAPI · MQTT · WebSocket · PyBullet · micro-ROS · LeRobot · Docker · pytest  

**关联仓库（脚注或第二行小字）**：`robot-ops-dashboard` · `amr_warehouse_navigation` · `ros2-robot-digital-twin` · `ros2-moveit-pybullet-bridge` · `robot-arm-episode-data-lab`

---

## 一句话版本

构建五仓库机器人系统工程作品集：Dashboard 层聚合 AMR/WMS 任务、MQTT 遥测与评测展示；Bridge 层完成 MoveIt/PyBullet 闭环、双源 Sim2Real 监控与风险急停；数据层导出 LeRobot episode，全链路脚本化验收与可审计报告。

---

## 完整版 bullet（系统工程向 · 6–8 条）

- 设计分层机器人集成架构：上层 `robot-ops-dashboard`（FastAPI + WebSocket + MQTT cache）聚合 AMR Mock WMS、micro-ROS 遥测与 Motor bench；下层 `ros2-moveit-pybullet-bridge` 负责 MoveIt 2 规划执行、PyBullet 双源仿真、分布监控与 R0–R3 风险闭环；离线 `robot-arm-episode-data-lab` 输出 LeRobot v2.1 数据供在线回放与校准。
- 打通三条可演示数据链：（1）Dashboard → HTTP → Mock WMS → Nav2/Gazebo 任务回写；（2）STM32/ESP32 → micro-ROS → MQTT → Dashboard 只读状态镜像；（3）受限 `POST /api/robot/motor/cmd` → MQTT → 电机 bench 编码器反馈；接口契约见 `api_contract.md` 与 `ICD.md`。
- 实现 MoveIt 2 到 PyBullet 轨迹执行桥接，`FollowJointTrajectory` relay 驱动 7-DOF iiwa7，发布 `/joint_states` 与双源 joint states；复验 4/4 MoveGroup goals 成功，最大 RMSE 0.006004 rad。
- 构建 Sim/Real 双源分布偏移监控（KL / W1 / MMD，10 Hz，5 s 滑窗），支持域随机化、LeRobot 回放与 3/3 偏移注入检出；跨仓同任务校准报告可量化 sim/real 轨迹对齐。
- 实现五维风险聚合与 Fail-Safe：R3 自动 E-stop，速度归零 ~0.8 ms，HOC/WebSocket acknowledge 互锁后恢复；Bridge HOC（React + ECharts）与 Dashboard `/ws/status` 分别服务深度运维与多链路聚合。
- 实现可插拔 Policy Runner（`BasePolicy` / Replay / SineWave）、lifecycle、`/system_health` 与 `run_system_validation.sh` 系统 benchmark（HTML/JSON/CSV）；SineWave mean latency ~4.8 ms。
- 建立可复现验收体系：Dashboard `verify_demo_readiness` + Bridge `verify_*.sh` + pytest + Docker compose verify；Bridge 核心包 142 passed，coverage 73.2%；Evaluation 层明确 mock/baseline/reserved 口径。

---

## 精简版（4–5 条）

- 五仓库机器人系统集成平台：Dashboard 聚合 AMR 任务、MQTT 遥测与评测层；Bridge 完成 MoveIt/PyBullet 闭环、Sim2Real 监控与风险急停；episode-data-lab 提供 LeRobot 数据链。
- AMR / micro-ROS / Motor bench 三链：HTTP task proxy、MQTT 状态镜像、受限 motor 命令与 WebSocket 实时推送（2026-06-14 全链路验收 PASS）。
- 双源 PyBullet + LeRobot 回放，KL/W1/MMD 10 Hz 监控，R0–R3 风险与 E-stop 闭环；MoveIt 执行 RMSE 0.006 rad 级。
- Policy Runner 插件化 benchmark + Docker/CI 可复现验证脚本与 HTML 报告。

---

## 超短版（2 条 · 空间有限）

- 机器人系统工程作品集（5 仓）：FastAPI/MQTT/WS 运维 Dashboard + ROS 2 MoveIt/PyBullet Sim2Real 验证 + LeRobot 数据采集，含 AMR 任务链、micro-ROS 遥测、双源监控、R0–R3 急停与脚本化验收。
- 量化证据：MoveIt 4/4 闭环、3/3 偏移检出、142 tests / 73.2% coverage、Dashboard 全链路 demo readiness PASS。

---

## 中文面试 3 分钟口述（系统工程版）

我这套作品集按**系统工程**组织，不是单点 demo。

**第一层是运维聚合**：`robot-ops-dashboard` 用 FastAPI 和 WebSocket 把 AMR 任务、MQTT 上来的 IMU/电机状态、以及评测摘要放在一个驾驶舱里。AMR 侧通过 HTTP 接 Mock WMS，不替代 Nav2；遥测侧是只读镜像；电机命令是低频、受限、显式的 bench 入口。

**第二层是深度验证**：`ros2-moveit-pybullet-bridge` 做操作臂——MoveIt 规划接到 PyBullet，开双源仿真量化 Sim2Real 偏移，KL/MMD 监控，R0–R3 风险急停，还有 Policy Runner 做策略插件和系统 benchmark。

**第三层是数据**：`robot-arm-episode-data-lab` 离线采 episode 导出 LeRobot，给 Bridge 做 real 侧回放和跨仓校准。

三条链的共同点是：**接口契约、边界清晰、脚本验收、报告可审计**。真机、完整 ros2_control、VLA 训练我明确标成 Phase-2，但架构和 ICD 已经预留。

---

## 英文简历（精简）

**Robotics Systems Integration & Validation Platform**  
Personal Project｜ROS 2 Jazzy, MoveIt 2, Nav2, FastAPI, MQTT, WebSocket, PyBullet, micro-ROS, LeRobot, Docker, pytest

- Built a five-repository robotics platform: a FastAPI/WebSocket/MQTT dashboard aggregating AMR Mock WMS tasks, micro-ROS telemetry, and baseline evaluation views; a ROS 2 bridge for MoveIt-to-PyBullet execution, dual-source Sim2Real monitoring, and R0-R3 fail-safe control; and an offline episode-data lab exporting LeRobot datasets.
- Integrated three demonstrable data links: HTTP AMR task proxy to Nav2/Gazebo, read-only MQTT telemetry mirroring, and safety-limited motor bench commands with encoder feedback; full demo readiness validated on 2026-06-14.
- Delivered 10 Hz KL/W1/MMD drift monitoring with inject-shift detection (3/3), MoveIt closure validation (4/4 goals, max RMSE 0.006 rad), and pluggable PolicyRunner benchmarks with `/system_health` diagnostics.
- Added reproducible verification via Docker, CI, pytest (142 passed, 73.2% coverage), and HTML/JSON/CSV audit reports; evaluation layers are explicitly mock/baseline/reserved, not claimed as trained VLA/RL results.

---

## 表述边界（五仓统一口径）

| 可以说 | 不要说 |
|--------|--------|
| 五仓分层集成、HTTP/MQTT/ROS/WS 多协议 | 完整 Fleet 管理平台已量产 |
| Mock WMS + Nav2 任务链已联调 | 商业 WMS 或多机调度已上线 |
| micro-ROS + MQTT 实机 bench 联调 | 完整嵌入式驱动栈 / 量产固件 |
| LeRobot 导出 + 在线回放 + 跨仓校准 | 已完成真机 iiwa 部署 |
| FollowJointTrajectory relay 闭环 | 完整 ros2_control 硬件接口已交付 |
| Policy 插件 + benchmark | RL/VLA 模型已训练部署 |
| Evaluation mock/baseline/reserved | 真实 GPU 训练成绩 |
| CI + Docker + verify 脚本 | 2h 长稳 / 固件 OTA 已完成 |

---

## 按岗位微调（只改排序，不改事实）

**系统工程 / 平台（默认）**：先 Dashboard 三链 → 再 Bridge 双源/风险 → 最后验收/ Docker  
**ROS 2 集成**：先 ROS topic/launch → Nav2 + MoveIt 各一条 → micro-ROS  
**仿真 / Test**：先 KL/MMD/inject → benchmark 报告 → Docker/CI  
**AMR 入门**：先 task chain → 弱化 Manipulation 为「另有验证环境」  
**Manipulation 入门**：先 MoveIt/HOC → 弱化 Dashboard 为「另有运维平台经验」
