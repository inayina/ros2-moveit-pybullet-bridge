# 作品集剩余工作清单

**项目**：ros2-moveit-pybullet-bridge（+ robot-arm-episode-data-lab 联动）  
**更新**：2026-06-20  
**用途**：面试 Demo 与作品集交付的待办跟踪；按优先级推进，避免与「可选扩展」混淆。

---

## 1. 总体完成度（面试 Demo 视角）

| 范围 | 完成度 | 说明 |
|------|--------|------|
| **M1–M5 核心功能** | ~98% | 桥接、MoveIt、双源、监控、HOC、Pick/Place 已落地 |
| **S5 风险补全（09 Spec）** | ~98% | 五维风险、R2 降速、看门狗、CSV、verify 脚本已 commit `a6cb248` |
| **M6 作品集打磨** | ~70% | 基础配图/样例报告已入库；待真实录屏与 Demo 视频 |
| **双仓库一体叙事** | ~60% | LeRobot 路径与脚本已有；联调验收与 HAL 收敛未做 |
| **Phase-2+ 扩展** | ~15% | 真机、独立虚拟相机、NavigateToPose 等 — **刻意预留，非面试必做** |

> **结论**：系统已能 Live Demo；剩余工作主要是 **展示材料 + 排练 + 双仓库验收**，不是补核心代码。

---

## 2. 优先级定义

| 级别 | 含义 | 目标时间 |
|------|------|----------|
| **P0** | 不做则面试 Demo 有风险 | 推送后 1–2 天 |
| **P1** | 提升作品集专业度（README / GitHub 第一印象） | 面试前 1 周内 |
| **P2** | 加分项，有时间再做 | 可选 |
| **P3** | 设计文档中的未来扩展，不必为求职阻塞 | 长期 |

---

## 3. P0 · 面试 Demo 必做

### 3.1 确认 CI 通过

- [ ] Push 后打开 [Actions](https://github.com/inayina/ros2-moveit-pybullet-bridge/actions)，确认 `a6cb248` 的 **build-and-test** 全绿  
- [ ] 若仍失败：查看失败步骤日志；`ci.yml` 已改为 `"${GITHUB_WORKSPACE}"`，常见剩余问题是依赖或 launch 测试超时

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

- [ ] 上述脚本 PASS  
- [ ] conda 用户：演示前 `unset CONDA_PREFIX`，用系统 Python 3.12  
- [ ] HOC：`cd hoc_console/frontend && npm install`（`hoc.launch.py` 会自动 `npm run dev`）

### 3.4 已知脚本不一致（可选修复）

| 问题 | 影响 | 建议 |
|------|------|------|
| `hoc_demo_3min.sh` 用 `full_system.launch.py` | Pick 演示被 SKIP | 面试改用 `portfolio_demo` + `hoc.launch.py`；或改脚本默认 launch |

---

## 4. P1 · 作品集展示材料

### 4.1 README / GitHub 配图

当前 `docs/assets/` 已入库基础配图（见 [`docs/assets/README.md`](assets/README.md)）。

| 文件 | 状态 | 操作 |
|------|------|------|
| `m1-joint-sweep.gif` | ✅ 已生成 | 可选替换为真实 M1 录屏 |
| `m1-pybullet.png` | ✅ 已生成 | 可选 |
| `m3-dual-source.gif` | ✅ 已生成（合成） | **建议** `portfolio_demo` 录屏替换 |
| `m4-monitor-metrics.png` | ✅ 已生成（合成） | 可选 |
| `m2-iiwa-pybullet.gif/png` | ✅ 已生成（PyBullet） | **建议** RViz 录屏 → `m2-iiwa-rviz.gif` |
| `m5-hoc-dashboard.png` | ✅ 已生成（matplotlib 预览） | **建议** 浏览器真实截图替换 |
| `portfolio-overview.png` | ✅ 已生成 | README 顶部栈图 |
| `m5-hoc-*.svg` | ✅ 已有 | 架构/线框保留 |

- [x] 运行 `python3 scripts/generate_milestone_assets.py`  
- [x] 运行 `python3 scripts/generate_sample_report.py`（含 dashboard 嵌入）  
- [x] **P1-1** `./scripts/capture_readme_assets.sh` — 从真实 NPZ/指标生成 m3、m2-iiwa-*、m5  
- [ ] 可选：RViz Plan & Execute 手动录屏替换 `m2-iiwa-rviz.gif`  
- [ ] 可选：HOC 浏览器真实截图（脚本已尝试 `hoc_prod` + Playwright）  
- [ ] 将 Demo 视频上传并在 README 加链接

### 4.2 演示视频与样例报告

- [ ] 录制 **5–8 分钟** 完整 Demo（iiwa7 + HOC + 注入 + 急停）  
- [ ] 上传：GitHub Release / 个人网盘 / B站（README 加链接）  
- [x] [`docs/samples/sample-experiment-report.html`](samples/sample-experiment-report.html) 已更新（五维 + git hash + 截图）

### 4.3 HOC 生产构建（可选，录屏用 dev 模式可跳过）

```bash
cd hoc_console/frontend && npm install && npm run build
ros2 launch hoc_console hoc_prod.launch.py   # http://localhost:8080
```

- [ ] 前端五维/HOLD/CSV 改动已 `npm run build`（仅 `hoc_prod` 需要）

### 4.4 文档索引

- [ ] 在 [`docs/design/README.md`](design/README.md) 增加本文链接（便于自己回顾）  
- [ ] README 里程碑表可增加 **M6** 一行（打磨中 / 待录屏）

---

## 5. P2 · 双仓库联动（episode-data-lab）

与 [`docs/design/08-dual-repo-portfolio-integration-spec.md`](design/08-dual-repo-portfolio-integration-spec.md) §10 对齐。

### 5.1 episode-data-lab 侧

- [ ] `dataset/v1/lerobot_export` 已导出且 `validate_dataset.py` 通过  
- [ ] Episode 使用 `robot=kuka_iiwa`、7-DOF、`grasp_mode=constraint`  
- [ ] `metadata.json` 含 `task_name`、`success`、`random_seed`

### 5.2 bridge 侧

- [ ] `export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab`  
- [ ] `./scripts/run_integration_demo.sh` 或至少 `offline_compare` PASS  
- [ ] `real_source:=topic` 与 `real_source:=lerobot` **各跑通一次**

```bash
export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab
ros2 launch pybullet_bridge portfolio_demo.launch.py real_source:=lerobot
```

### 5.3 一体讲解

- [ ] 熟练 [08 §11 面试一体讲解稿](design/08-dual-repo-portfolio-integration-spec.md#11-面试一体讲解稿3-分钟)（offline 采集 + online 监控两条腿）

---

## 6. P3 · 可选扩展（不必为求职阻塞）

来自设计文档、**已实现占位或未实现**的项：

| 项 | 设计来源 | 当前状态 | 备注 |
|----|----------|----------|------|
| `NavigateToPose` Action | 07 §3 | 未实现 | 人形/底盘预留 |
| `real_source:=ros2` 真机模式 | 08 §4 | 未实现 | 接真机时再开 |
| episode-data-lab `Ros2Robot` HAL | 08 S5 | 未实现 | `batch_collect --backend ros2` |
| 独立 `virtual_camera_node` | 07 §5 | 部分 | bridge 已有 `enable_camera` + `verify_camera.sh` |
| 完整 `ros2_control` 接入 | 05 §3.5 | relay 替代 | MoveIt 闭环已通 |
| `/clock` + `use_sim_time` | 05 §7 | 参数预留 | 仿真时间同步 |
| 真实夹爪物理 / `/gripper/command` | 07 Pick | `GripperStub` | Pick/Place 演示够用 |
| 力矩饱和 D3 增强 | 09 §6 | 未做 | PyBullet POSITION_CONTROL 限制 |
| HOC Settings 面板调阈值 | 09 G7 | 服务已有 | UI 未做 |
| NFR 状态快照持久化 | 01 NFR-R02 | 未做 | 节点重启恢复 |
| Docker `compose run verify` | README | 待验证 | 可选冒烟 |

---

## 7. 建议推进顺序（时间盒）

```
第 1 天   P0.1 CI 绿勾 + P0.3 verify 脚本
第 2 天   P0.2 Demo 排练 2 遍 + 录备用视频
第 3–4 天 P1 配图/录屏入库 + README 链接 Demo 视频
第 5 天   P2 双仓库联调（若 episode-data-lab 已就绪）
面试前   背诵 07 §8 + 08 §11；打印/备忘 Demo 操作顺序
```

---

## 8. 快速命令备忘

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

# 资产生成
python3 scripts/generate_milestone_assets.py
python3 scripts/generate_sample_report.py
```

---

## 9. 相关文档

| 文档 | 用途 |
|------|------|
| [07 · 作品集总 Spec](design/07-portfolio-system-spec-supplement.md) | Sprint、讲解稿、交付清单 |
| [08 · 双仓库集成](design/08-dual-repo-portfolio-integration-spec.md) | LeRobot 契约、联调命令 |
| [09 · 风险补全 Spec](design/09-risk-monitoring-completion-spec.md) | S5 已实现定义（DoD） |
| [INTEGRATION.md](INTEGRATION.md) | episode-data-lab 联调步骤 |
| [MILESTONE_VERIFICATION_FIXES.md](MILESTONE_VERIFICATION_FIXES.md) | 历史验证问题记录 |

---

## 10. 版本记录

| 日期 | 变更 |
|------|------|
| 2026-06-20 | 初版：基于 M1–M5 + S5 完成态整理 P0–P3 剩余项 |
