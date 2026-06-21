# 面试巩固 · 必掌握清单与标准答法

**主投方向**：机器人系统工程 / 平台集成 / 仿真验证  
**更新**：2026-06-21  
**关联**：[MASTER_PORTFOLIO_PLAN.md](./MASTER_PORTFOLIO_PLAN.md) · [UNIFIED_RESUME.md](./UNIFIED_RESUME.md)

---

## 1. 必须掌握 vs 必须了解 vs 可后置

### 1.1 必须掌握（说不清就别投）

| 主题 | 你要能做什么 |
|------|--------------|
| 五仓分工 | 3 分钟画架构图 + 说清各仓职责与边界 |
| 四协议 | HTTP / WebSocket / MQTT / ROS 2 在本项目中的分工 |
| Dashboard 三链 | AMR 任务、IMU 遥测（只读）、Motor bench（受限命令） |
| Bridge 闭环 | MoveIt → PyBullet → 监控 → R0–R3 → HOC |
| 验收数字 | 每个简历数字能指向 `docs/samples/` 或 dashboard validation 报告 |
| 交付口径 | mock/baseline/reserved、Phase-2+、不说真机/VLA/OTA 已完成 |

### 1.2 必须了解（概念 + 本项目关系，不必推公式）

ROS 2 基础 · MoveIt Plan & Execute · Nav2 NavigateToPose · micro-ROS · Docker/CI · Sim2Real 域随机化 · LeRobot episode · CD/SDK/OTA/Socket 在本项目的真实完成度

### 1.3 可后置（被问到 30 秒内转向 Phase-2）

SLAM/点云 · RL/VLA 训练 · TensorRT · K8s Fleet · ISO 功能安全认证 · 2h 长稳 · 完整 ros2_control 硬件接口

---

## 2. 必背数字速查

### Dashboard（2026-06-14 demo readiness）

| 项 | 结果 |
|----|------|
| `GET /health` | PASS |
| `GET /api/tasks` | PASS |
| `WebSocket /ws/status` | PASS |
| `POST /api/wms/tasks` | PASS（Mock WMS proxy） |
| `POST /api/robot/motor/cmd` | PASS（STOP + 短时低速探针） |
| Evaluation | mock/baseline/reserved，非真实训练 |

### Bridge（本机复验 · 见 `docs/samples/`）

| 项 | 结果 |
|----|------|
| MoveIt 闭环 | 4/4 goals，max RMSE **0.006004 rad** |
| 偏移注入检出 | **3/3**，10 Hz，5 s 滑窗 |
| R3 速度归零 | ~**0.8 ms**；acknowledge 互锁 |
| HOC WebSocket | **5 Hz**；e_stop ~**12 ms** |
| Policy Runner | SineWave mean latency ~**4.8 ms** |
| pytest | **142 passed**，coverage **73.2%** |
| LeRobot smoke | sim=421 / real=421（样本） |

---

## 3. 四协议对照（30 秒版）

| 协议 | Dashboard | Bridge | 典型用途 |
|------|-----------|--------|----------|
| HTTP REST | FastAPI `/api/*` | HOC 8080 静态 | 请求-响应、任务创建 |
| WebSocket | `/ws/status` | HOC `:8765` | 实时推送风险/状态 |
| MQTT | `robot/imu` 等 | 未用 | 边缘到 PC 的状态镜像 |
| ROS 2 | 不直连 | topic/service/action | 机器人执行与监控 |

**为什么 Dashboard 不直连 ROS 2**：集成边界在 HTTP/MQTT 适配层，降低耦合，AMR 与嵌入式子系统可独立演进。

---

## 4. 模拟面试 10 题 · 标准答法

### Q1. 为什么做五仓而不是一个 monorepo？

**答**：机器人系统工程本来就是多子系统拼装。我按**职责边界**拆仓：

- `robot-ops-dashboard` 做上层聚合，不依赖 ROS 2 运行时；
- `amr_warehouse_navigation` 和 `ros2-robot-digital-twin` 是可独立启动的 AMR 与边缘子系统；
- `ros2-moveit-pybullet-bridge` 专注操作臂深度验证；
- `robot-arm-episode-data-lab` 专注离线数据。

这样每个仓有独立验收脚本和接口契约，联调通过 HTTP/MQTT/LeRobot 文件衔接，更接近真实团队分工，也方便 CI 分仓跑。

---

### Q2. Dashboard 为什么不直连 ROS 2？

**答**：Dashboard 的定位是**运维与测试驾驶舱**，不是机器人控制器。AMR 侧我通过 **HTTP Mock WMS API** 集成；遥测侧通过 **ROS 2 → MQTT 镜像** 进 backend 内存缓存；前端只连 FastAPI 和 WebSocket。

好处是：Dashboard 可以在没有 Gazebo/Nav2 的机器上展示任务与缓存状态；子系统升级时不强迫改 Dashboard 的 ROS 依赖。Motor 命令也是走 `POST /api/robot/motor/cmd` → MQTT，而不是 rclpy 直接 publish。

---

### Q3. 双源 PyBullet 如何替代真机做 Sim2Real？

**答**：没有真机时，我开两个 PyBullet 实例：

- **Sim-Source**：理想物理参数，代表规划/训练基准环境；
- **Real-Source**：`changeDynamics` 域随机化 + 噪声，代表可控的「真实世界代理」。

同一 `JointTrajectory` 同时驱动两路，监控层对关节误差分布做 **KL / W1 / MMD** 对比；还可 `/bridge/inject_shift` 做 Ground Truth 注入验证检出率。另外可以用 **LeRobot 回放** 作为 Real 侧基线，做跨仓同任务校准。

这是**迁移前可度量**，不是声称真机已验证。

---

### Q4. KL 和 MMD 分别解决什么问题？

**答**：

- **KL（散度）**：看单关节或统计量分布是否偏离基线，可解释性好，适合告警阈值；
- **W1（Wasserstein-1）**：对均值漂移敏感，看整体位移；
- **MMD**：核方法，适合看多维联合分布是否变了，配合置换检验给 p-value。

我线上用 5 秒滑窗、10 Hz 发布，和 risk_engine 的 D1 分布偏移维度联动。验收看 **shift_detected + 时序**，不是单个 RMSE 点。

---

### Q5. R3 为什么必须人工 acknowledge？

**答**：这是 Fail-Safe 设计：R3 表示分布或安全状态**未知或严重异常**，系统自动 E-stop 并 cancel 轨迹，但**不能自动恢复**，因为模型/环境可能仍不可信。

操作员在 HOC 确认已排查（`/risk/acknowledge`）后，才允许 `/risk/clear_e_stop` 和恢复 RUNNING。复验里未 acknowledge 时 clear 会被拒绝。口径是**宁可误停，不可在未知状态下继续执行**。

---

### Q6. Policy Runner 和 MoveIt 是什么关系？

**答**：它们是**两条独立的控制入口**，都最终发布 `/bridge/command`：

- **MoveIt 路径**：规划 → `FollowJointTrajectory` → relay → PyBullet；
- **Policy Runner 路径**：订阅 `/bridge/sim/joint_states` → `BasePolicy.get_action()` → 发布 trajectory。

Policy Runner 用于策略插件和系统 benchmark（Replay/SineWave），证明接口可替换、延迟可度量；**不替代** MoveIt 的碰撞检测与规划。安全仍走 risk_engine，策略节点不直接触发 E-stop。

---

### Q7. REST / MQTT / WebSocket 在你项目里各负责什么？

**答**：

- **REST**：Dashboard 的任务查询/创建、robot status 查询、evaluation 只读展示；请求-响应、易缓存、易测；
- **MQTT**：边缘 ESP32 到 PC 的 **IMU/motor 状态镜像** 和受限 motor 命令；适合边缘低带宽、多订阅者；
- **WebSocket**：Dashboard `/ws/status` 和 HOC `:8765` 向浏览器**推送**任务/风险/指标快照；低延迟、服务端主动推。

Bridge 侧主要是 **ROS 2 DDS**，HOC 用 WebSocket 桥接浏览器，因为前端不适合直接进 ROS graph。

---

### Q8. Evaluation 层的 mock 和 live 怎么区分？

**答**：Dashboard 的 Evaluation 全部来自本仓库 **只读 JSON**（`GET /api/evaluation/*`），口径标注为 `mock_evaluation`、`baseline_system_evaluation` 或 `interface_reserved`。

- **live 部分**：AMR 任务状态、MQTT 上来的 IMU/motor 缓存、WebSocket 推送——这些来自真实联调链路；
- **mock 部分**：dataset/model registry、GPU 利用率（未接入显示 `not_connected`）、部分 failure case 样例。

面试和简历里我会明确说：**评测层是系统验收展示，不是 VLA/RL 训练成绩**。

---

### Q9. CI 通过了证明什么、没证明什么？

**答**：

**证明了**：bridge 在 `ros:jazzy-ros-base` 容器里能 colcon build、跑通 `run_tests.sh`（单元+节点+集成）、资产生成 smoke；Docker compose verify 在挂载 episode-data-lab 后可复现 headless 冒烟。

**没证明**：真机部署、2 小时长稳、跨进程 supervisor、固件 OTA、Dashboard 全链路在 GitHub Actions 里每日跑（Dashboard 目前是本地 demo readiness + pytest）。

CI 是**工程可复现的下限证据**，不是量产就绪。

---

### Q10. 若上真机，你会改哪一层、不动哪一层？

**答**：

**改/替换**：

- Real-Source：从「域随机化 PyBullet」或 LeRobot 回放 → `real_source:=ros2` 订阅真机关节状态；
- episode-data-lab HAL：`PyBulletRobot` → `Ros2Robot`（`/joint_states` + FollowJointTrajectory）；
- 执行层：trajectory relay → 逐步换完整 `ros2_control` 硬件接口（若 JD 要求）。

**尽量不动**：

- dist_monitor / risk_engine 的监控与 R0–R3 逻辑；
- HOC 与 Dashboard 的运维命令语义（急停、acknowledge、报告导出）；
- ICD 里已稳定的 topic 名与 `/bridge/system_state` 枚举。

真机迁移是 **adapter 层替换**，不是重写监控与运维体系。

---

## 5. 边界口径 · 快速拒答表

| 面试官可能问 | 正确说法 |
|--------------|----------|
| 上真机了吗？ | 仿真+LeRobot 回放验证；真机接口 ICD/HAL 预留，Phase-2 |
| 有 OTA 吗？ | 固件 OTA 未做；配置热更新/策略 lifecycle 可扩展为软件 OTA |
| 训练 VLA 了吗？ | 数据链+Replay 基线；Evaluation 为 mock/baseline |
| ros2_control 完成了吗？ | FollowJointTrajectory relay 闭环已验收；完整硬件接口 Phase-2 |
| Dashboard 控制 Nav2 吗？ | 不；HTTP 到 Mock WMS，Nav2 在 AMR 子系统内 |
| 多机 Fleet？ | 单机 Demo；架构可扩展为多 backend 实例 |

---

## 6. 面试前 48 小时 Checklist

- [ ] 五仓架构白板 1 遍（≤3 min）
- [ ] Dashboard storyboard 口述 1 遍（60–90s）
- [ ] Bridge DEMO_SCRIPT 排练 1 遍（5–8 min）
- [ ] 本节 §2 数字能脱口而出
- [ ] §4 十题每题 ≤90s 回答
- [ ] GitHub：两仓 README 互链、CI badge、视频/Release 链接（若已上传）
- [ ] 演示环境：`unset CONDA_PREFIX`、启动命令预跑

---

## 7. 3 天冲刺日程（时间紧时用）

| 天 | 上午 | 下午 | 晚上 |
|----|------|------|------|
| D1 | §1 + §3 四协议 | Dashboard 三链 + storyboard | Q1–Q3 自问自答 |
| D2 | Bridge DEMO 排练 | §2 数字 + samples 对照 | Q4–Q7 |
| D3 | 模拟面试 10 题 | 录屏/Release 或 PPT | Q8–Q10 + §6 checklist |

---

## 8. 相关文档

| 文档 | 用途 |
|------|------|
| [MASTER_PORTFOLIO_PLAN.md](./MASTER_PORTFOLIO_PLAN.md) | 下周开发 + 资料索引 |
| [UNIFIED_RESUME.md](./UNIFIED_RESUME.md) | 简历 bullet 与边界 |
| [DEMO_SCRIPT.md](./DEMO_SCRIPT.md) | Bridge 录屏顺序 |
| [CODE_WALKTHROUGH.md](./CODE_WALKTHROUGH.md) | 源码深挖 |
| dashboard `docs/dashboard_demo_storyboard.md` | Dashboard 录屏 |
| dashboard `docs/api_contract.md` | REST/WS 契约 |
