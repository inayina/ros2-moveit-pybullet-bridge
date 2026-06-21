# 作品集剩余工作清单

**项目**：ros2-moveit-pybullet-bridge（+ robot-arm-episode-data-lab 联动）  
**更新**：2026-06-21  
**用途**：面试 Demo 与作品集交付的待办跟踪；按优先级推进，避免与「可选扩展」混淆。

---

## 1. 总体完成度（面试 Demo 视角）

| 范围 | 完成度 | 说明 |
|------|--------|------|
| **M1–M5 核心功能** | ~98% | 桥接、MoveIt、双源、监控、HOC、Pick/Place 已落地 |
| **S5 风险补全（09 Spec）** | ~98% | 五维风险、R2 降速、看门狗、CSV、verify 脚本已落地；2026-06-20 本机复验通过 |
| **M6 作品集打磨** | ~95% | pick-and-lift 抓取 GIF、真实 HOC 浏览器截图、双仓报告与样例报告已入库；RViz 仅作可选本地演示证据，待 Demo 视频与公开 CI 证据 |
| **Phase-1.5 Policy Runner** | ✅ 已完成 | D0–D5：策略抽象、PolicyRunner、benchmark、一键验证；见 `docs/design/10-policy-runner-system-engineering-spec.md` |
| **双仓库一体叙事** | ~95% | episode-data-lab 数据集、LeRobot export、bridge online/offline 联调与同任务校准已通过；HAL 属 Phase-2 |
| **Phase-2+ 扩展** | ~15% | 真机、独立虚拟相机、NavigateToPose 等 — **刻意预留，非面试必做** |

> **结论**：系统已能 Live Demo；核心与双仓联调已本机验收。若目标是面试交付，优先补展示材料、排练和公开 CI/Docker 证据；若目标是继续开发，Phase-2+ 真机与 HAL 迁移可排在展示材料之后。

---

## 2. 优先级定义

| 级别 | 含义 | 目标时间 |
|------|------|----------|
| **P0** | 不做则面试 Demo 有风险 | 推送后 1–2 天 |
| **P1** | 提升作品集专业度（README / GitHub 第一印象） | 面试前 1 周内 |
| **P1.5** | 下一轮系统工程增强开发 | 1–2 周 |
| **P2** | 加分项，有时间再做 | 可选 |
| **P3** | 设计文档中的未来扩展，不必为求职阻塞 | 长期 |

---

## 3. P0 · 面试 Demo 必做

### 3.1 确认 CI 通过

- [x] GitHub Actions 上次运行成功，**build-and-test** 全绿  
- [ ] 若后续提交 CI 失败：查看失败步骤日志；`ci.yml` 已改为 `"${GITHUB_WORKSPACE}"`，常见剩余问题是依赖或 launch 测试超时
- [x] 本机已复现：`./scripts/run_tests.sh` 通过（单元 + 节点 + launch 集成）

**本地复现 CI：**

```bash
export PATH="/usr/bin:/bin:/opt/ros/jazzy/bin:$PATH" && unset CONDA_PREFIX
cd ~/ros2_ws && colcon build --packages-select bridge_monitor_msgs pybullet_bridge dist_monitor risk_engine hoc_console manipulation_actions moveit_config --symlink-install
source install/setup.bash
cd ~/ros2_ws/src/ros2-moveit-pybullet-bridge && ./scripts/run_tests.sh
```

### 3.2 完整 Demo 排练（至少 2 遍）

**推荐两终端流程**（比 `hoc_demo_3min.sh` 更完整，含 Pick/manipulation）：

```bash
# 终端 1
source ~/ros2_ws/install/setup.bash
ros2 launch pybullet_bridge portfolio_demo.launch.py sim_mode:=GUI

# 终端 2
source ~/ros2_ws/install/setup.bash
ros2 launch hoc_console hoc.launch.py
# 浏览器 → http://localhost:5173
```

如果需要单命令本地实验入口，可用代码中已有的组合 launch：

```bash
ros2 launch hoc_console hoc_experiment.launch.py sim_mode:=DIRECT
# 浏览器 → http://localhost:5173
```

**5 分钟操作脚本：**

| 顺序 | 动作 | 讲解要点（15–30 秒） |
|------|------|----------------------|
| 1 | 指 PyBullet 双实例 + HOC 雷达 | 双源仿真 = 无真机时的 Sim2Real 代理 |
| 2 | 观察 R0、KL/MMD 时序 | 监控的是**分布漂移**，不是单点误差 |
| 3 | HOC「注入偏移」或 SC-01 | Ground Truth 可复现，对应风控指标漂移 |
| 4 | 看 R0→R2、`shift_detected`、主因 | 五维归因 + 分级处置 |
| 5 | 急停 → Acknowledge → 恢复 | FRM 式熔断，人工复核后恢复 |
| 6 | 导出 HTML/CSV 报告 | 实验可审计、可复现 |

- [ ] 排练 2 遍，记录卡顿点（启动慢、话题无数据、浏览器未连 WS 等）
- [ ] 准备 **备用录屏**（5–8 分钟），现场环境异常时可播放

**讲解稿**：[`docs/design/07-portfolio-system-spec-supplement.md`](design/07-portfolio-system-spec-supplement.md) §8（约 3 分钟架构口述）

### 3.3 环境预检（面试前一天）

```bash
cd ~/ros2_ws/src/ros2-moveit-pybullet-bridge
./scripts/verify_portfolio.sh
./scripts/verify_risk_complete.sh   # 可选，验证五维风险链路
python3 scripts/check_iiwa_joint_consistency.py
```

- [x] 上述脚本 PASS（`verify_portfolio.sh`、`verify_risk_complete.sh`、`check_iiwa_joint_consistency.py`）  
- [ ] conda 用户：演示前 `unset CONDA_PREFIX`，用系统 Python 3.12  
- [ ] HOC：`cd hoc_console/frontend && npm install`（`hoc.launch.py` 会自动 `npm run dev`）

### 3.4 已知脚本不一致（可选修复）

| 问题 | 影响 | 建议 |
|------|------|------|
| `hoc_demo_3min.sh` 用 `full_system.launch.py` | Pick 演示被 SKIP | 面试改用 `portfolio_demo` + `hoc.launch.py`；或改脚本默认 launch |
| `portfolio_demo.launch.py` 不自动启动 HOC | “一键演示”容易被误解为含浏览器控制台 | README 已标注；需要单命令时改用 `hoc_experiment.launch.py` |

---

## 4. 当前收尾优先级

| 优先级 | 任务 | 当前判断 |
|--------|------|----------|
| **S0** | GitHub Actions 最新提交绿勾 | 必做，作为公开交付证据 |
| **S0** | 5–8 分钟备用 Demo 视频 | 必做，现场环境异常时兜底 |
| **S1** | HOC 浏览器真实截图 / 抓取 GIF | 已补；RViz 仅作可选本地演示证据 |
| **S1** | Docker `compose run verify` | 已通过，证明干净容器环境可复现 |
| **S2** | README 增加双仓报告入口与一句话讲解 | 低成本，建议做 |

---

## 5. P1 · 作品集展示材料

### 5.1 README / GitHub 配图

当前 `docs/assets/` 已入库基础配图（见 [`docs/assets/README.md`](assets/README.md)）。

| 文件 | 状态 | 操作 |
|------|------|------|
| `m1-joint-sweep.gif` | ✅ 已生成 | 可选替换为真实 M1 录屏 |
| `m1-pybullet.png` | ✅ 已生成 | 可选 |
| `m3-dual-source.gif` | ✅ 已生成（合成） | **建议** `portfolio_demo` 录屏替换 |
| `m4-monitor-metrics.png` | ✅ 已生成（合成） | 可选 |
| `m2-iiwa-pybullet.gif/png` | ✅ 已生成（PyBullet） | 可选补更清晰 RViz 录屏；README 主图改用抓取 GIF |
| `m6-pick-and-lift.gif` | ✅ 已生成（episode-data-lab 成功抓取 episode） | README 首图展示任务过程 |
| `m5-hoc-dashboard.png` | ✅ 已替换为 HOC 浏览器真实截图 | Playwright + `hoc_prod` 截图 |
| `portfolio-overview.png` | ✅ 已生成 | README 顶部栈图 |
| `m5-hoc-*.svg` | ✅ 已有 | 架构/线框保留 |

- [x] 运行 `python3 scripts/generate_milestone_assets.py`  
- [x] 运行 `python3 scripts/generate_sample_report.py`（含 dashboard 嵌入）  
- [x] **P1-1** `./scripts/capture_readme_assets.sh` — 从真实 NPZ/指标生成 m3、m2-iiwa-*、m5  
- [x] **P1-2** `python3 scripts/capture_pick_lift_asset.py` — 从 episode-data-lab 成功抓取 episode 生成 `m6-pick-and-lift.gif`
- [ ] 可选：若需要 RViz 证据，再录一版清晰可见的 MoveIt **Plan → Execute** 过程
- [x] HOC 浏览器真实截图（`hoc_prod` + Playwright）
- [ ] 将 Demo 视频上传并在 README 加链接

### 5.2 演示视频与样例报告

- [ ] 录制 **5–8 分钟** 完整 Demo（iiwa7 + HOC + 注入 + 急停）  
- [ ] 上传：GitHub Release / 个人网盘 / B站（README 加链接）  
- [x] [`docs/samples/sample-experiment-report.html`](samples/sample-experiment-report.html) 已更新（五维 + git hash + 截图）
- [x] [`docs/samples/dual-repo-integration-report.html`](samples/dual-repo-integration-report.html) 已更新（online smoke: sim=421 / real=421）
- [x] [`docs/samples/same-task-calibration-report.html`](samples/same-task-calibration-report.html) 已更新（LeRobot replay: sim=1543 / real=1542）

### 5.3 HOC 生产构建（可选，录屏用 dev 模式可跳过）

```bash
cd hoc_console/frontend && npm install && npm run build
ros2 launch hoc_console hoc_prod.launch.py   # http://localhost:8080
```

- [ ] 前端五维/HOLD/CSV 改动已 `npm run build`（仅 `hoc_prod` 需要）

### 5.4 文档索引

- [ ] 在 [`docs/design/README.md`](design/README.md) 增加本文链接（便于自己回顾）  
- [ ] README 里程碑表可增加 **M6** 一行（打磨中 / 待录屏）

---

## 6. P1.5 · Policy Runner 系统工程增强

目标：在不训练 AI 模型、不接真机的前提下，把现有仿真平台升级为可插拔策略运行与可复现系统 benchmark 平台。

完整规格见 [`docs/design/10-policy-runner-system-engineering-spec.md`](design/10-policy-runner-system-engineering-spec.md)。

### 6.1 开发阶段

| 阶段 | 任务 | 交付物 | 状态 |
|------|------|--------|------|
| D0 | 规划与文档 | 10 Spec、ARCHITECTURE、ICD、FMEA、README 入口 | ✅ 已完成 |
| D1 | 策略抽象层 | `BasePolicy`、`ReplayPolicy`、`SineWavePolicy`、单元测试 | ✅ 已完成 |
| D2 | 最小 PolicyRunner | 订阅 `/bridge/sim/joint_states`，发布 `/bridge/command` | ✅ 已完成 |
| D3 | 健康与故障注入 | lifecycle/state machine、`/system_health`、metrics 日志、sleep 注入 | ✅ 已完成 |
| D4 | Benchmark | `scripts/benchmark_system.py`、CSV/JSON/HTML 输出 | ✅ 已完成 |
| D5 | 一键验证 | `scripts/run_system_validation.sh`、汇总报告、CI 友好退出码 | ✅ 已完成 |

### 6.2 推荐开工顺序

1. 先做 D1，保证策略接口和基线策略完全脱离 ROS，可快速测试。
2. 再做 D2，让 `SineWavePolicy` 能通过 `PolicyRunner` 驱动现有 PyBullet bridge。
3. D3 加 `/system_health` 和故障注入，用它证明可靠性工程能力。
4. D4/D5 最后做，因为 benchmark 依赖前面节点稳定。

### 6.3 当前不做

| 项 | 原因 |
|----|------|
| PyTorch 训练 / RL pipeline | 本阶段证明系统工程能力，不证明模型训练能力 |
| 真机 `real_source:=ros2` | 属于 Phase-2+，需要硬件和 HAL 迁移 |
| 完整 `ros2_control` 硬件接口 | 当前 MoveIt relay 已满足演示闭环，完整控制栈后续做 |
| 2 小时 soak / supervisor | 可由 benchmark 铺垫，但完整长稳仍属于 Phase-2+ 硬化 |

---

## 7. P2 · 双仓库联动（episode-data-lab）

与 [`docs/design/08-dual-repo-portfolio-integration-spec.md`](design/08-dual-repo-portfolio-integration-spec.md) §10 对齐。

### 7.1 episode-data-lab 侧

- [x] `dataset/v1/lerobot_export` 已导出且 `validate_dataset.py` 通过（20 episodes，20/20 success）  
- [x] Episode 使用 `robot=kuka_iiwa`、7-DOF、LeRobot `observation.state` / `action` 关节名对齐 `lbr_iiwa_joint_1..7`  
- [x] LeRobot meta 含任务、episode、video、state/action schema；原始 episode metadata 仍由 episode-data-lab 侧维护

### 7.2 bridge 侧

- [x] `export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab`  
- [x] `./scripts/run_dual_repo_integration.sh` PASS，生成 `dual-repo-integration-report.html` / `dual-repo-experiment-report.html`  
- [x] `./scripts/run_same_task_calibration.sh` PASS，生成 `same-task-calibration-report.html`  
- [x] `real_source:=topic` 与 `real_source:=lerobot` 均跑通；online smoke 最新样本 `sim=421`、`real=421`

```bash
export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab
ros2 launch pybullet_bridge portfolio_demo.launch.py real_source:=lerobot
```

### 7.3 一体讲解

- [ ] 熟练 [08 §11 面试一体讲解稿](design/08-dual-repo-portfolio-integration-spec.md#11-面试一体讲解稿3-分钟)（offline 采集 + online 监控两条腿）
- [ ] 可选：把最新双仓报告截图/链接放入 README 或演示视频脚本

---

## 8. Phase-2+ 可选扩展（收尾取舍）

来自设计文档、**已实现占位或未实现**的项：

| 项 | 当前状态 | 对收尾帮助 | 建议 |
|----|----------|------------|------|
| Docker `compose run verify` | 已通过 | **高**：证明干净环境可复现 | 已收口 |
| HOC 浏览器真实截图 / pick-and-lift 抓取 GIF | 已补 | **高**：直接提升 GitHub/面试观感 | 已收口；RViz 可选再录 |
| episode-data-lab `Ros2Robot` HAL | 未实现 | 中：叙事加分，但工作量大 | 不建议现在做；保留 Phase-2 |
| 独立 `virtual_camera_node` | 部分，bridge 已有 camera | 低：已有相机/HOC 证据可讲 | 不建议为收尾做 |
| HOC Settings 面板调阈值 | 服务已有，UI 未做 | 低：不影响 Demo 主线 | 不建议为收尾做 |
| `/clock` + `use_sim_time` | 参数预留 | 低：偏工程严谨性 | 不建议为收尾做 |
| 完整 `ros2_control` 接入 | relay 替代 | 低：当前 MoveIt 闭环已通 | 不建议为收尾做 |
| 真实夹爪物理 / `/gripper/command` | `GripperStub` | 低：Pick/Place 演示够用 | 不建议为收尾做 |
| `real_source:=ros2` 真机模式 | 未实现 | 低/高风险：无真机条件下难验收 | 明确作为未来迁移 |
| `NavigateToPose` Action | 未实现 | 低：偏人形/底盘扩展 | 不建议为收尾做 |
| NFR 状态快照持久化 | 未做 | 低：面试演示不明显 | 不建议为收尾做 |
| 力矩饱和 D3 增强 | PyBullet POSITION_CONTROL 限制 | 低：现有 D3 已可验收 | 不建议为收尾做 |

> 面试口径：Phase-2+ 不是“欠缺没做完”，而是未来迁移边界。当前收尾应优先做交付证据：CI、Docker smoke、真实截图/视频、报告入口。
> 验证提醒：`verify_portfolio.sh` / `verify_risk_*.sh` 会清理并启动 ROS 进程，适合干净终端复验；如果本机已有录屏、HOC 或 launch 流程在跑，先确认再执行。

---

## 8. 建议推进顺序（时间盒）

```
立即     GitHub Actions 绿勾 + Docker verify（若本机 Docker 可用）
当天     录备用视频 + 截 HOC/RViz 真实图
当天     README 加双仓报告入口 / Demo 视频链接
面试前   排练 2 遍；背 07 §8 + 08 §11；打印/备忘 Demo 操作顺序
```

---

## 9. 快速命令备忘

```bash
# 环境
source ~/ros2_ws/install/setup.bash
export PATH="/usr/bin:/bin:/opt/ros/jazzy/bin:$PATH" && unset CONDA_PREFIX

# 作品集主线 Demo
ros2 launch pybullet_bridge portfolio_demo.launch.py sim_mode:=GUI
ros2 launch hoc_console hoc.launch.py

# MoveIt 加分 Demo
ros2 launch moveit_config m2_iiwa_demo.launch.py sim_mode:=GUI

# 验证
./scripts/verify_portfolio.sh
./scripts/verify_risk_complete.sh
./scripts/run_tests.sh
./scripts/run_dual_repo_integration.sh
./scripts/run_same_task_calibration.sh

# 资产生成
python3 scripts/generate_milestone_assets.py
python3 scripts/generate_sample_report.py
```

---

## 10. 相关文档

| 文档 | 用途 |
|------|------|
| [07 · 作品集总 Spec](design/07-portfolio-system-spec-supplement.md) | Sprint、讲解稿、交付清单 |
| [08 · 双仓库集成](design/08-dual-repo-portfolio-integration-spec.md) | LeRobot 契约、联调命令 |
| [09 · 风险补全 Spec](design/09-risk-monitoring-completion-spec.md) | S5 已实现定义（DoD） |
| [INTEGRATION.md](INTEGRATION.md) | episode-data-lab 联调步骤 |
| [MILESTONE_VERIFICATION_FIXES.md](MILESTONE_VERIFICATION_FIXES.md) | 历史验证问题记录 |

---

## 11. 版本记录

| 日期 | 变更 |
|------|------|
| 2026-06-20 | 初版：基于 M1–M5 + S5 完成态整理 P0–P3 剩余项 |
