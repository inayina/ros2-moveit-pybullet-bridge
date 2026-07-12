# 统一作品集主计划 · 资料整理 / 下周开发 / 面试巩固

**更新**：2026-06-21  
**主投方向**：机器人系统工程 / 平台集成 / 仿真验证  
**副投方向**：ROS 2 集成、运维 Dashboard、AMR 入门、Manipulation 入门、具身数据入门  

> 本文是**新一版作品集**的总控文档：汇总五个仓库的可提取材料、缺口清单、下周（2026-06-22 ~ 06-28）开发计划，以及面试前知识巩固安排。  
> Dashboard 三仓主入口：`~/workspace/robot-ops-dashboard`  
> 操作臂 + Sim2Real 双仓：`~/ros2_ws/src/ros2-moveit-pybullet-bridge` + `~/robot-sim-lab/robot-arm-episode-data-lab`

---

## 1. 新一版作品集结构（建议对外呈现）

### 1.1 对外标题（简历 / README / 幻灯片封面）

**中文**：机器人系统集成与验证平台（AMR 运维 Dashboard + 操作臂 Sim2Real 联调）  

**英文**：Robotics Systems Integration & Validation Platform  

### 1.2 对内五仓分工

| 仓库 | 路径 | 作品集角色 | 一句话 |
|------|------|------------|--------|
| **robot-ops-dashboard** | `~/workspace/robot-ops-dashboard` | **主展示入口** | FastAPI + WebSocket + MQTT 聚合 AMR / 遥测 / bench |
| **amr_warehouse_navigation** | GitHub `inayina/amr_warehouse_navigation` | AMR 子系统 | Nav2 + Gazebo + Mock WMS |
| **ros2-robot-digital-twin** | GitHub `inayina/ros2-robot-digital-twin` | 边缘 / 孪生子系统 | micro-ROS + MQTT + 电机 bench |
| **ros2-moveit-pybullet-bridge** | `~/ros2_ws/src/ros2-moveit-pybullet-bridge` | 深度验证子系统 | MoveIt + PyBullet + 监控 + 风险 + Policy Runner |
| **robot-arm-episode-data-lab** | `~/robot-sim-lab/robot-arm-episode-data-lab` | 数据子系统 | Episode 采集 + LeRobot 导出 |

### 1.3 投递物清单（新一版目标）

| 产物 | 状态 | 负责仓库 | 截止 |
|------|------|----------|------|
| 统一架构总图（1 页 Mermaid） | ✅ 已完成 | `UNIFIED_ARCHITECTURE.md` + 两仓 README | — |
| 主 README 互链（dashboard ↔ bridge） | ✅ 已完成 | 两仓 README | — |
| 简历项目描述（中/英） | 已有草稿 | bridge `RESUME_SUMMARY.md` + dashboard `portfolio_demo_summary.md` | 周二合并 |
| 8–13 页 PDF/PPTX | 已有 Marp 源 | bridge `system-design-slides.md` | 周三导出 + 补 dashboard 页 |
| Demo 视频 A：Dashboard 60–90s | 待录/待公开 | dashboard | 周四 |
| Demo 视频 B：Bridge 5–8min | 本地有素材 | bridge `DEMO_SCRIPT.md` | 周四 |
| GitHub Release v0.1.0（两主仓） | 待做 | 两仓 | 周五 |
| CI 绿勾截图 / badge | 部分有 | bridge | 周一 push 后确认 |
| 协议对照表（REST/MQTT/ROS/WS） | 待做 | 本文 §6 | 周三 |

---

## 2. 资料整理索引（从项目里直接提取）

### 2.1 Dashboard 三仓 · 已有材料

| 类别 | 路径 | 用途 |
|------|------|------|
| 作品集主入口 | `robot-ops-dashboard/README.md` | GitHub 首页 |
| 提取指南 | `docs/portfolio_demo_summary.md` | 简历 bullet、关键词 |
| 简短摘要 | `docs/portfolio_summary.md` | 一句话 / 精简版 |
| API 契约 | `docs/api_contract.md` | SDK/集成向面试 |
| 当前边界 | `docs/current_scope.md` | 防 overclaim |
| 录屏分镜 | `docs/dashboard_demo_storyboard.md` | 60–90s 视频 |
| 录屏检查 | `docs/demo_recording_checklist.md` | 录前预检 |
| 全链路验证 | `docs/full_pipeline_validation_report.md` | 证据 |
| 最终验收 | `artifacts/reports/final_demo_validation_2026-06-14.zh.md` | 面试引用数字 |
| 数据链说明 | `docs/robot_data_pipeline.md` | 架构讲解 |
| MQTT 集成 | `docs/integration_mqtt_device.md` | 遥测链 |
| AMR HTTP | `docs/integration_amr_http.md` | 任务链 |
| WebSocket | `docs/websocket_status_stream.md` | Socket 向面试 |
| 截图 | `artifacts/screenshots/`（README 有清单；若缺失需重跑 `scripts/capture_dashboard_artifacts.js`） | README / PPT |
| pytest | `backend/tests/` | CI/质量证据 |

**Dashboard 面试必背数字**（来自 2026-06-14 验收）：

- `/health`、`/api/tasks`、`/ws/status`：PASS  
- WMS task 创建 proxy：PASS  
- Motor cmd STOP 探针：PASS  
- Evaluation：mock/baseline/reserved，不宣称 VLA/RL 已训练  

### 2.2 Bridge + Episode-data-lab · 已有材料

| 类别 | 路径 | 用途 |
|------|------|------|
| 作品集包索引 | `docs/portfolio/README.md` | 阅读顺序 |
| 验收摘要 | `docs/portfolio/ACCEPTANCE_SUMMARY.md` | FR/NFR 状态 |
| Demo 脚本 | `docs/portfolio/DEMO_SCRIPT.md` | 5–8min 录屏 |
| 代码导览 | `docs/portfolio/CODE_WALKTHROUGH.md` | 源码面试 |
| 简历描述 | `docs/portfolio/RESUME_SUMMARY.md` | 中/英 bullet |
| 系统设计 | `docs/portfolio/SYSTEM_DESIGN_SPEC.md` | PDF 长文 |
| 幻灯片源 | `docs/portfolio/system-design-slides.md` | Marp → PDF/PPTX |
| 接口 ICD | `docs/ICD.md` | 系统集成 |
| 架构 | `docs/ARCHITECTURE.md` | Policy Runner 层 |
| 差距台账 | `docs/ACCEPTANCE_GAP.md` | 诚实边界 |
| 收尾清单 | `docs/PORTFOLIO_REMAINING.md` | 待办 |
| 双仓集成 | `docs/design/08-dual-repo-portfolio-integration-spec.md` | §11 三分钟讲稿 |
| 联调报告 | `docs/samples/dual-repo-integration-report.html` | 跨仓证据 |
| 同任务校准 | `docs/samples/same-task-calibration-report.html` | Sim2Real 证据 |
| HOC 报告 | `docs/samples/sample-experiment-report.html` | 运维证据 |
| Policy 验证 | `docs/samples/system-validation/validation_report.html` | benchmark |
| 指标索引 | `docs/samples/README.md` | JSON/CSV/rosbag |
| 配图 | `docs/assets/`（m1–m6、双仓 overlay 等） | README/PPT |
| 本地视频 | `docs/samples/portfolio-demo-zh-*.mp4` | 待上传 Release |
| CI | `.github/workflows/ci.yml` | 工程交付 |

**Bridge 面试必背数字**：

- MoveIt 闭环：4/4 goals，max RMSE 0.006004 rad  
- 监控：10 Hz，KL+MMD，3/3 偏移注入检出  
- 风险：R3 速度归零 ~0.8 ms，acknowledge 互锁  
- HOC WS：5 Hz，e_stop ~12 ms  
- Policy Runner：SineWave mean latency ~4.8 ms  
- 测试：142 passed，coverage 73.2%  

### 2.3 统一架构总图

见 **[UNIFIED_ARCHITECTURE.md](./UNIFIED_ARCHITECTURE.md)**（已入库；README 与 dashboard README 已嵌入总览 Mermaid）。

源文件：`unified-architecture-overview.mmd` · 可选导出 `docs/assets/unified-architecture-overview.png`

---

## 3. 下周开发计划（2026-06-22 ~ 06-28）

**原则**：作品集周 — **70% 整理/展示/巩固，30% 轻量工程**。主投系统工程，不新开 Phase-2+ 真机/HAL。

| 周一 · 资料归仓 + CI 证据

| 时段 | 任务 | 产出 |
|------|------|------|
| 上午 | 通读本文 + 两仓 portfolio 索引；列出 GitHub 要公开的 repo 链接 | 链接清单 |
| 上午 | push bridge 当前改动；确认 Actions 绿勾 | badge 可截图 |
| 下午 | ~~写统一架构总图~~；dashboard README 加 Related → bridge | ✅ `UNIFIED_ARCHITECTURE.md` |
| 下午 | bridge README 加 Related → dashboard 三仓 | ✅ 互链 + 嵌入总图 |
| 晚上 | 跑 `./scripts/verify_portfolio.sh` + dashboard `verify_demo_readiness.sh` | 预检 PASS 记录 |

### 周二 · 简历 + 合并项目描述

| 时段 | 任务 | 产出 |
|------|------|------|
| 上午 | 合并 `RESUME_SUMMARY.md` + `portfolio_demo_summary.md` → 一份 **`UNIFIED_RESUME.md`**（可放 bridge `docs/portfolio/`） | 中/英完整版 + 精简版 |
| 下午 | 按 JD「系统工程」重排 bullet：Dashboard 三链 → Bridge 双源 → 共同工程化 | 简历定稿 |
| 晚上 | 背 §11 双仓讲稿 + dashboard storyboard 旁白各 1 遍 | 口述录音自评 |

### 周三 · 幻灯片 + 协议对照

| 时段 | 任务 | 产出 |
|------|------|------|
| 上午 | Marp 导出 bridge 幻灯片；**新增 3–4 页** dashboard 链（AMR/MQTT/Evaluation） | `system-design-slides.pdf` |
| 下午 | 写 **协议对照表**（见 §6）并入 portfolio | `PROTOCOL_MATRIX.md` |
| 晚上 | 可选：bridge 草拟 `docs/SDK.md` 大纲（ROS + BasePolicy + WS） | 文档骨架 |

### 周四 · 录屏日

| 时段 | 任务 | 产出 |
|------|------|------|
| 上午 | Dashboard 60–90s：按 `dashboard_demo_storyboard.md` | `dashboard-demo.mp4` |
| 下午 | Bridge 5–8min：按 `DEMO_SCRIPT.md`（偏移 + R3 + 报告） | `bridge-demo.mp4` |
| 晚上 | 各录一版**备用静态 PPT + 旁白**（环境崩了用） | 兜底素材 |

### 周五 · Release + 公开链接

| 时段 | 任务 | 产出 |
|------|------|------|
| 上午 | GitHub Release v0.1.0（dashboard + bridge）；视频放 Release asset 或 B 站/网盘 | 公开 URL |
| 下午 | README 加：视频链接、PDF 链接、统一架构图 | 投递就绪 |
| 晚上 | 全文校对 overclaim（mock/VLA/真机/ros2_control） | 边界一致 |

### 周末 · 排练 + 模拟面试

| 时段 | 任务 | 产出 |
|------|------|------|
| 周六 | 完整 Demo 排练 2 遍（Dashboard 1 + Bridge 1） | 卡顿清单 |
| 周六 | 模拟面试：3min 自我介绍 + 10min 架构深挖 | 自评笔记 |
| 周日 | 按 §7 知识巩固做**错题/弱项**回补 | 巩固清单勾选 |
| 周日 | 休息；只保留轻量复习卡片 | — |

### 下周可选工程（仅当 Mon–Thu 提前完成）

| 优先级 | 项 | 估时 | 价值 |
|--------|-----|------|------|
| P1 | bridge CI 加 `run_system_validation.sh` job | 2h | Test/Validation JD |
| P1 | dashboard 重跑截图脚本（若 artifacts 缺失） | 1h | README 观感 |
| P2 | bridge `docs/SDK.md` + `examples/ws_min_client.py` | 4h | SDK JD |
| P2 | 轻量「配置 OTA」设计 1 页（不实现也可） | 2h | OTA 口径 |
| 不做 | Ros2Robot HAL、真机、RL 训练 | — | 拖慢作品集周 |

---

## 4. 面试前知识巩固（按系统工程主线）

### 4.1 每日安排（与开发并行）

| 日 | 巩固主题 | 复习材料 | 自测题 |
|----|----------|----------|--------|
| 一 | 五仓架构与边界 | 本文 §1–2、dashboard `current_scope.md` | 画一张分层图不看文档 |
| 二 | REST / MQTT / ROS / WS 区别 | §6、`api_contract.md`、`ICD.md` | 各举一条本项目真实 topic/API |
| 三 | AMR 任务链 | dashboard `integration_amr_http.md`、storyboard | Mock WMS 为何不直连 Nav2 |
| 四 | micro-ROS + MQTT 遥测 | `integration_mqtt_device.md`、validation 报告 | 只读 vs motor cmd 边界 |
| 五 | MoveIt → PyBullet 闭环 | `CODE_WALKTHROUGH.md`、`DEMO_SCRIPT.md` | relay 与完整 ros2_control 区别 |
| 六 | Sim2Real 监控 | 08 文档、`ACCEPTANCE_SUMMARY.md` | KL/MMD 各解决什么问题 |
| 日 | 风险与安全 | 09 spec 摘要、R0–R3 数字 | R3 为何必须 acknowledge |

### 4.2 系统工程高频题 · 标准答法要点

**Q：你的系统和 Nav2 / MoveIt 什么关系？**  
A：Dashboard 不替代 Nav2；bridge 不替代 MoveIt。我是**集成与验证层**——HTTP/MQTT/WS 聚合状态，ROS 2 做执行，脚本化验收沉淀证据。

**Q：Sim2Real 没真机怎么验证？**  
A：双源 PyBullet + LeRobot 回放构造 Ground Truth 偏移；KL/W1/MMD 在线；注入实验可复现；真机接口在 ICD/HAL 预留。

**Q：SDK 在哪里？**  
A：三层——REST（dashboard `api_contract`）、ROS 2 ICD（bridge）、Python `BasePolicy` 插件（PolicyRunner）；不是 PyPI 商业 SDK，是**契约 + 示例**。

**Q：CI/CD 做了什么？**  
A：CI：colcon + pytest + Docker verify；CD：Release + 镜像/视频可追溯；OTA 未做固件，配置/策略热更新有边界演示空间。

**Q：为什么两个作品集？**  
A：同一能力的两条腿——Dashboard 证明**多子系统聚合与运维**；Bridge 证明**深度运动链与量化验证**。岗位偏哪条就深讲哪条。

### 4.3 弱项回补（根据你历史对话）

| 弱项 | 巩固动作 |
|------|----------|
| CD / OTA 词义 | 记「CI=自动测，CD=可发布产物，OTA=版本升级+互锁」；本项目各到哪一步 |
| Socket | 记 dashboard `/ws/status` vs bridge HOC `:8765`；TCP 之上 WebSocket 帧 |
| 算法细节 | 只需讲 KL=分布差、MMD=核方法、窗口 5s/10Hz；不必推公式 |
| conda / ROS 冲突 | 演示前 `unset CONDA_PREFIX`，系统 Python 3.12 |
| 模拟面试十题 | 见 [INTERVIEW_PREP.md](./INTERVIEW_PREP.md) §4 标准答法 |

### 4.5 面试前 48 小时 checklist

- [ ] 两个 Demo 各排练 1 遍  
- [ ] 简历每个数字能在 `docs/samples/` 或 validation 报告里找到  
- [ ] 能 3 分钟画五仓架构  
- [ ] 能 1 分钟说清 mock/baseline/真机/训练的边界  
- [ ] GitHub 首页：视频链接 + CI badge + 架构图  
- [ ] 备用：录屏视频 U 盘/离线版  

---

## 5. 岗位投递策略（简表）

| 岗位 | 主 Demo | 主文档 | 弱化 |
|------|---------|--------|------|
| **系统工程 / 平台集成** | Dashboard → Bridge 各 3min | 本文 + 双 portfolio 索引 | 算法推导 |
| ROS 2 集成 | Bridge MoveIt + Dashboard Nav2/micro-ROS | ICD + api_contract | VLA |
| 仿真 / Test | Bridge inject + reports | ACCEPTANCE_GAP + validation HTML | Fleet |
| AMR 入门 | Dashboard 任务链 | storyboard + AMR 仓 | Manipulation 深度 |
| Manipulation 入门 | Bridge HOC + 双源 | DEMO_SCRIPT + 08 文档 | MQTT 细节 |
| 具身数据入门 | episode-data-lab + LeRobot | dual-repo 报告 | 训练 |

---

## 6. 协议对照表（PROTOCOL_MATRIX · 面试速查）

| 通道 | Dashboard 三仓 | Bridge 双仓 | 典型延迟/频率 | 谁连接 |
|------|----------------|-------------|---------------|--------|
| **HTTP REST** | FastAPI `/api/*` | —（HOC 8080 静态） | 请求级 | 前端、WMS proxy |
| **WebSocket** | `/ws/status` | HOC `:8765` | 5 Hz 级 | 浏览器 |
| **MQTT** | `robot/imu` 等 | 未用 | 低频缓存 | backend ↔ broker |
| **ROS 2 Topic** | 经 bridge 镜像 | `/bridge/command`、`/joint_states`、监控/风险 | 10–100 Hz | rclpy 节点 |
| **ROS 2 Service** | — | `/bridge/*`、`/risk/*` | 事件 | HOC / 脚本 |
| **ROS 2 Action** | — | MoveIt、Pick/Place | 轨迹级 | MoveIt / manipulation |
| **文件契约** | mock eval JSON | LeRobot export、pkl replay | 离线 | 脚本 / PolicyRunner |

---

## 7. 相关文档快捷链接

### Dashboard（`~/workspace/robot-ops-dashboard`）

- `README.md`
- `docs/portfolio_demo_summary.md`
- `docs/api_contract.md`
- `docs/dashboard_demo_storyboard.md`
- `docs/current_scope.md`
- `artifacts/reports/final_demo_validation_2026-06-14.zh.md`

### Bridge（本仓库）

- [portfolio/README.md](./README.md)
- [RESUME_SUMMARY.md](./RESUME_SUMMARY.md)
- [DEMO_SCRIPT.md](./DEMO_SCRIPT.md)
- [ACCEPTANCE_SUMMARY.md](./ACCEPTANCE_SUMMARY.md)

---

## 8. 版本记录

| 日期 | 变更 |
|------|------|
| 2026-06-21 | 初版：五仓统一作品集计划、下周日程、面试巩固、协议对照表 |
