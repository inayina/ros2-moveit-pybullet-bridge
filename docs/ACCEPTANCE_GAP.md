# 验收标准差距台账

**项目**：ros2-moveit-pybullet-bridge  
**更新**：2026-06-20  
**用途**：按 `docs/design/01-system-architecture-and-requirements.md` 的验收标准逐项记录当前实现、证据缺口与收尾取舍，避免把「未复验证据」「范围调整」和「真正未实现」混在一起。

---

## 1. 结论摘要

当前项目已经具备面试 Demo 主线：MoveIt/PyBullet 闭环、双源监控、五维风险、HOC 控制台、报告导出与本机验证脚本。主要差距不是核心代码不可用，而是**公开交付证据仍需补齐**，集中在 GitHub Actions 最新绿勾、5–8 分钟公开 Demo 视频、Bridge jitter 严格阈值与长稳 / supervisor 可靠性证据。

| 类别 | 当前判断 | 处理方式 |
|------|----------|----------|
| 核心功能链路 | 基本满足 | 保持 `run_tests.sh`、`verify_portfolio.sh`、`verify_risk_complete.sh` 复验记录 |
| 公开交付证据 | 有本机证据，公开证据不足 | Push 后补 GitHub Actions 绿勾与 Demo 视频链接 |
| 性能/可靠性量化 | 性能、安全、HOC、可维护性已有样例报告；长稳仍不足 | 保留 Bridge jitter 与 2h soak / supervisor 为后续硬化项 |
| 早期设计口径偏差 | 存在 `UR5/FR3`、完整 `ros2_control`、真机 Real 等偏差 | 写入范围调整或 Phase-2+，不再按当前里程碑阻塞 |

---

## 2. 状态定义

| 状态 | 含义 |
|------|------|
| 已满足 | 代码与脚本已覆盖，且已有本机或样例证据 |
| 部分满足 | 核心指标已满足，但仍有子指标或严格阈值未完全达标 |
| 待复验 | 功能存在，但缺少最新、可引用的验收记录 |
| 证据缺口 | 需要补量测、截图、视频、CI 或报告，不一定需要改代码 |
| 范围调整 | 早期验收标准与当前作品集边界不一致，需在说明中显式改口径 |
| Phase-2+ | 有价值但不阻塞当前面试 Demo / 作品集验收 |

---

## 3. 功能需求差距

### 3.1 桥接通信（FR-BRG）

| ID | 验收标准 | 当前状态 | 差距 / 处理 |
|----|----------|----------|-------------|
| FR-BRG-01 | 指令延迟 P99 < 20ms | 已满足 | `./scripts/verify_bridge_comm.sh` 已生成 `docs/samples/bridge-comm-metrics.json`：command→feedback P99 = 10.524ms |
| FR-BRG-02 | 100Hz 发布，频率抖动 < 5% | 部分满足 | 三路 joint state 平均频率均为 100.0Hz；header stamp jitter = 5.501%，距 5% 目标仍差约 0.5 pct，需要后续优化或明确非实时 Python/rclpy 口径 |
| FR-BRG-03 | 至少验证 UR5/FR3 之一 | 范围调整 | 当前主线已改为 KUKA iiwa7；在说明中以 iiwa7 作为统一平台，UR5/FR3 不再作为当前验收项 |
| FR-BRG-04 | `DIRECT` / `GUI` launch 参数切换 | 已满足 | README 与 launch 流程已有 `sim_mode:=DIRECT/GUI` |
| FR-BRG-05 | 双 PyBullet 实例同步驱动，时间戳对齐 | 已满足 | `portfolio_demo`、双源样例报告与同任务校准报告覆盖 |

### 3.2 MoveIt2 规划闭环（FR-MOV）

| ID | 验收标准 | 当前状态 | 差距 / 处理 |
|----|----------|----------|-------------|
| FR-MOV-01 | 标准场景规划成功率 > 95% | 已满足 | `./scripts/verify_moveit_closure.sh` 已生成 `docs/samples/moveit-closure-metrics.json`：4/4 标准 joint goals 成功，success_rate = 1.0 |
| FR-MOV-02 | 通过 `ros2_control` 执行到 PyBullet，RMSE < 0.05 rad | 部分满足 | 当前以 `FollowJointTrajectory` relay 替代完整 `ros2_control`；同脚本测得最大 RMSE = 0.006004 rad，完整硬件接口仍归 Phase-2+ |
| FR-MOV-03 | 完整 TF 树供 RViz2 可视化 | 已满足 | `verify_moveit_closure.sh` 通过 TF lookup：`lbr_iiwa_link_0->lbr_iiwa_link_7` |
| FR-MOV-04 | 碰撞场景规划被拒绝 | 已满足 | `verify_moveit_closure.sh` 应用 `fr_mov_blocking_box` 到 planning scene，plan-only 请求被 MoveIt 拒绝（error_code=99999） |

### 3.3 分布监控（FR-MON）

| ID | 验收标准 | 当前状态 | 差距 / 处理 |
|----|----------|----------|-------------|
| FR-MON-01 | KL 滑窗 5s，10Hz 更新 | 已满足 | `./scripts/verify_monitor_metrics.sh` 已生成 `docs/samples/monitor-metrics.json`：window=5.0s，mean_hz=10.0，jitter=0.221% |
| FR-MON-02 | MMD 统计量 + 置换检验 p-value | 已满足 | 同一复验输出 MMD=0.297172，p=0.019608，`detection_method=kl+mmd` |
| FR-MON-03 | YAML 阈值热加载 / 可配置阈值 | 已满足 | 复验通过 `/dist_monitor/set_parameters` 热更新 `kl_threshold_mean=0.02`、`w1_threshold_mean=0.01`、`mmd_threshold=0.005`；HOC Settings UI 不作为当前阻塞 |
| FR-MON-04 | 分布指标时序支持 rosbag2 回放分析 | 已满足 | 复验生成 `docs/samples/monitor-metrics-timeline.csv` 与 `docs/samples/monitor-metrics-rosbag/metadata.yaml` |
| FR-MON-05 | 注入 ±20% 阻尼，检出率 > 90% | 已满足 | 复验执行 3 次 `joint_damping +20%` 注入，3/3 检出，detection_rate=1.0 |

### 3.4 风险管理（FR-RSK）

| ID | 验收标准 | 当前状态 | 差距 / 处理 |
|----|----------|----------|-------------|
| FR-RSK-01 | 五维风险 R0-R3，等级变化延迟 < 500ms | 已满足 | `./scripts/verify_risk_management.sh` 已生成 `docs/samples/risk-management-metrics.json`：R0/R1/R2/R3 延迟分别约 100.113/99.49/100.686/351.452ms，五维 attribution_count=5 |
| FR-RSK-02 | R3 自动急停，关节速度 < 100ms 归零 | 已满足 | 同一复验输出：R3 后 `e_stop_active=true`，bridge `E_STOP` 延迟 1.887ms，速度归零延迟 0.796ms |
| FR-RSK-03 | 等级变化附带 `primary_driver` | 已满足 | R3 状态 `primary_driver=distribution_shift`，每次状态含 5 维 attribution |
| FR-RSK-04 | HOC 确认后恢复运行 | 已满足 | 复验确认未 acknowledge 不能 clear；`/risk/acknowledge` 后 `/risk/clear_e_stop` 成功 |

### 3.5 人机控制台（FR-HOC）

| ID | 验收标准 | 当前状态 | 差距 / 处理 |
|----|----------|----------|-------------|
| FR-HOC-01 | 风险等级与五维雷达，WS 延迟 < 200ms | 已满足 | `./scripts/verify_hoc_console.sh` 已生成 `docs/samples/hoc-console-metrics.json`：risk WS 5.0Hz，max latency=16.131ms，payload 含 5 维 attribution |
| FR-HOC-02 | 分布对比可视化，支持暂停/缩放 | 已满足 | 复验确认 distribution payload 含 KL/W1/MMD 与 sim/real boxplot 数据；前端 `DistributionPanel`/`TrendChart` 已补 ECharts `dataZoom`，HOC pause 命令成功 |
| FR-HOC-03 | 急停、暂停、恢复，响应 < 100ms | 已满足 | WebSocket command_result 延迟：pause=66.236ms，e_stop=12.557ms，acknowledge=5.34ms，resume=30.152ms |
| FR-HOC-04 | 实验参数在线调节，下一周期生效 | 已满足 | WebSocket `set_randomization` 成功（71.852ms）并触发 episode resample；`inject_shift +20%` 成功（10.794ms） |
| FR-HOC-05 | JSON/CSV 报告，含风险时序 + 分布指标 | 已满足 | 复验导出 `fr_hoc_verify.json`（risk timeline=91，metrics timeline=92）与 `fr_hoc_verify.csv`（92 rows，含 risk/metric columns） |

---

## 4. 非功能需求差距

### 4.1 性能（NFR-P）

| ID | 目标 | 当前状态 | 差距 / 处理 |
|----|------|----------|-------------|
| NFR-P01 | 控制回路 P95 < 50ms | 已满足 | `./scripts/verify_performance_nfr.sh` 已生成 `docs/samples/performance-nfr-metrics.json`：command→joint feedback P95 = 14.571ms |
| NFR-P02 | 关节状态 100Hz ± 5% | 已满足 | 同一复验输出 `/joint_states` mean_hz = 99.984，mean_error = 0.016%；严格 jitter 仍由 FR-BRG-02 单独跟踪 |
| NFR-P03 | 分布指标 10Hz | 已满足 | 同一复验输出 `/monitor/distribution_metrics` mean_hz = 10.0，mean_error = 0.001% |
| NFR-P04 | HOC 刷新率 ≥ 5Hz | 已满足 | `verify_hoc_console.sh` / `verify_performance_nfr.sh` 复验：risk / distribution / tracking WebSocket streams 均为 5.0Hz |
| NFR-P05 | 双实例 240Hz，实时因子 ≥ 0.8 | 已满足 | `bridge_node` 发布 `/bridge/performance` 诊断；复验双源启用，source_count=2，min physics per source = 239.507Hz，min realtime_factor = 0.998 |

### 4.2 可靠性（NFR-R）

| ID | 目标 | 当前状态 | 差距 / 处理 |
|----|------|----------|-------------|
| NFR-R01 | 桥接节点崩溃触发安全停机 | Phase-2+ | `./scripts/verify_reliability_nfr.sh` 已生成 `docs/samples/reliability-nfr-metrics.json`：当前覆盖 command watchdog→HOLD（513.392ms）与 FR-RSK 急停链路；跨进程 supervisor 崩溃检测仍列 Phase-2+ |
| NFR-R02 | 节点重启后恢复上次安全状态 | Phase-2+ | 同一复验覆盖 `/bridge/reset_simulation` 安全回 home 后 RUNNING（6.953ms）；状态快照持久化与重启恢复仍列 Phase-2+ |
| NFR-R03 | 连续 2 小时无内存泄漏 | 部分满足 | 同一复验完成 10s smoke：joint/monitor/risk/performance 持续输出，RSS delta=+4.531MB；完整 2h soak 未跑，保留为长稳证据缺口 |
| NFR-R04 | rosbag 录制不因单节点异常中断 | 已满足 | HOC 独立 rosbag2 进程复验通过：生成 `docs/samples/reliability-nfr-rosbags/hoc_20260620_215136/metadata.yaml` 与 `.mcap`，包含 `/monitor/distribution_metrics`、`/risk/status`、`/risk/alerts` |

### 4.3 安全（NFR-S）

| ID | 目标 | 当前状态 | 差距 / 处理 |
|----|------|----------|-------------|
| NFR-S01 | 任意时刻急停，速度归零 | 已满足 | `./scripts/verify_safety_nfr.sh` 已生成 `docs/samples/safety-nfr-metrics.json`：R3 后 bridge `E_STOP` 延迟 9.498ms，速度归零 0.367ms |
| NFR-S02 | 95% URDF limits 触发 R2 | 已满足 | 同一复验通过 soft-limit safety override：R2 延迟 99.8ms，`primary_driver=dynamics_anomaly`，`degraded_mode=true` |
| NFR-S03 | 指令中断 > 500ms 自动 HOLD | 已满足 | 同一复验长轨迹停止续命后进入 `HOLD`，延迟 509.678ms |
| NFR-S04 | R2 自动 50% 降速 | 已满足 | 同一复验对比 0.35s 运动量：normal=0.038422rad，degraded=0.019242rad，ratio=0.501 |
| NFR-S05 | R3 后 HOC 确认 + 自检恢复 | 已满足 | 未 acknowledge 时 clear 被拒绝；ack 后 clear 成功，bridge 恢复 `RUNNING` 延迟 35.902ms |

### 4.4 可维护性与可复现性（NFR-M / NFR-REP）

| ID | 目标 | 当前状态 | 差距 / 处理 |
|----|------|----------|-------------|
| NFR-M01 | YAML 配置，支持动态调参 | 已满足 | `./scripts/verify_maintainability_nfr.sh` 已生成 `docs/samples/maintainability-nfr-metrics.json`：18 个 YAML 配置解析通过；FR-MON 阈值热更新与 HOC `set_randomization` 均有复验证据 |
| NFR-M02 | 一键 launch 完整系统 | 已满足 | 同一复验确认 `hoc_console/launch/hoc_experiment.launch.py` 同时 include `portfolio_demo.launch.py` 与 `hoc.launch.py`；14 个 launch 文件均可 `py_compile` |
| NFR-M03 | ROS2 标准包结构，`colcon build` 可编译 | 已满足 | 同一复验确认 7 个 ROS 包均有 `package.xml` + build file，且 `ros2 pkg prefix` 均可解析到 install 前缀 |
| NFR-M04 | 单元测试覆盖率 > 70% | 已满足 | 同一复验运行核心包 coverage：142 passed，coverage=73.2%，产出 `docs/samples/maintainability-coverage/coverage.json` 与 `coverage.xml` |
| NFR-REP-01 | 随机过程可配置 seed | 已满足 | 同一复验确认 seed/randomization 配置链路存在；HOC `set_randomization` 成功，样例产物含固定 seed 输出 |
| NFR-REP-02 | 实验配置与结果关联存储 | 已满足 | 同一复验确认可靠性 rosbag `metadata.yaml` 存在，HOC/report/rosbag metadata 与配置结果产物可关联 |
| NFR-REP-03 | 标准 Demo 脚本输出确定性基准 | 已满足 | 同一复验确认 9 个 verify 脚本存在、可执行且 `bash -n` 通过；8 个 JSON 样例指标产物已落盘 |

---

## 5. 当前 P0/P1 差距清单

| 优先级 | 差距 | 验收完成标志 |
|--------|------|--------------|
| P0 | GitHub Actions 最新提交公开绿勾 | README badge / Actions 页面显示 latest run 通过 |
| P0 | 5–8 分钟完整 Demo 视频 | README 或 Release 中有视频链接，覆盖启动、偏移、R2/R3、导出 |
| P0 | Bridge jitter 口径统一 | `FR-BRG-02` 保持“部分满足”：平均 100Hz 与 P99 延迟达标，header stamp jitter = 5.501% 略超 5% |
| P1 | 公开视频与 README 入口 | 本地视频素材已生成；公开 Release / 网盘 / B 站链接仍待补 |
| P1 | 长稳 / supervisor 可靠性证据 | 当前已有短时 smoke、watchdog HOLD、HOC 独立 rosbag；2h soak 与跨进程 supervisor 仍属 Phase-2+ |

---

## 6. Phase-2+ 不阻塞项

以下项属于未来迁移或增强，不作为当前面试 Demo / 作品集验收阻塞：

| 项 | 当前口径 |
|----|----------|
| 真机 `real_source:=ros2` | 未来接真机驱动时实现；当前双源 PyBullet 与 LeRobot replay 替代 |
| 完整 `ros2_control` 硬件接口 | 当前 relay 已满足演示闭环；完整控制栈归后续工程化 |
| episode-data-lab `Ros2Robot` HAL | 双仓联动加分项，不阻塞 bridge 当前交付 |
| `/clock` + `use_sim_time` 全链路 | 严格 rosbag 同步增强，当前不阻塞 Demo |
| 2 小时长稳 / valgrind | 交付硬化项，面试前以短时 smoke 和 CI 证明为主 |
| HOC Settings 完整阈值 UI | 服务/配置能力优先，UI 打磨后续补 |

---

## 7. 更新规则

1. 验收脚本或报告新增后，先更新本文件对应行的状态与证据。
2. 若某项不再属于当前里程碑，必须改为「范围调整」或「Phase-2+」，并在 `README.md` / `PORTFOLIO_REMAINING.md` 保持同一口径。
3. 不把“代码已实现”直接等同于“验收已完成”；缺少可引用记录时仍标为「待复验」或「证据缺口」。

---

## 8. 相关文档

| 文档 | 用途 |
|------|------|
| [design/01-system-architecture-and-requirements.md](design/01-system-architecture-and-requirements.md) | 原始功能 / 非功能验收标准 |
| [PORTFOLIO_REMAINING.md](PORTFOLIO_REMAINING.md) | 面试 Demo 收尾任务与复验命令 |
| `docs/samples/bridge-comm-metrics.json` | 运行 `./scripts/verify_bridge_comm.sh` 后生成的桥接通信频率、抖动与 command→feedback 延迟复验输出 |
| [samples/moveit-closure-metrics.json](samples/moveit-closure-metrics.json) | MoveIt 规划成功率、PyBullet 执行 RMSE 与 TF lookup 复验输出 |
| [samples/monitor-metrics.json](samples/monitor-metrics.json) | 分布监控频率、MMD p-value、阈值热更新、注入检出率复验输出 |
| [samples/risk-management-metrics.json](samples/risk-management-metrics.json) | R0-R3 延迟、R3 急停、速度归零、primary_driver 与 acknowledge 互锁复验输出 |
| [design/09-risk-monitoring-completion-spec.md](design/09-risk-monitoring-completion-spec.md) | S5 风险补全历史缺口与完成定义 |
| [portfolio/SYSTEM_DESIGN_SPEC.md](portfolio/SYSTEM_DESIGN_SPEC.md) | 交付视角说明书与已知限制 |
| [samples/README.md](samples/README.md) | 样例报告与指标产物索引 |
