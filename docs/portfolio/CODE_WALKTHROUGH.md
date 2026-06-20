# 代码导览

这份导览用于面试或作品集讲解，不替代完整设计文档。建议先讲系统边界，再按 Bridge、Monitor、Risk、HOC 四个模块展开。

## 1. 桥接层：MoveIt 到 PyBullet

核心文件：

- `pybullet_bridge/pybullet_bridge/bridge_node.py`
- `pybullet_bridge/pybullet_bridge/sim_source.py`
- `pybullet_bridge/pybullet_bridge/real_source.py`
- `pybullet_bridge/launch/portfolio_demo.launch.py`
- `moveit_config/launch/m2_iiwa_demo.launch.py`

讲解主线：

> Bridge 层负责把 MoveIt 输出的 JointTrajectory 变成 PyBullet 控制命令，并把仿真状态以 ROS 2 topic 的形式反馈给 MoveIt、TF、监控层和 HOC。

可以重点讲：

- `/bridge/command` 接收轨迹指令，桥接节点按仿真时间采样执行。
- `/joint_states` 提供 MoveIt / TF 闭环反馈。
- `/bridge/sim/joint_states` 与 `/bridge/real/joint_states` 同时发布，为分布监控提供双源输入。
- `set_randomization` / `inject_shift` 让偏移可控、可复现。
- `pause`、`resume`、`force_e_stop`、`reset_simulation` 支持运维控制和验收脚本。

作品集证据：

- [moveit-closure-metrics.json](../samples/moveit-closure-metrics.json)
- [bridge-comm-metrics.json](../samples/bridge-comm-metrics.json)
- [m2-iiwa-pipeline.svg](../assets/m2-iiwa-pipeline.svg)

## 2. 监控层：Sim/Real 分布偏移

核心文件：

- `dist_monitor/dist_monitor/monitor_node.py`
- `dist_monitor/config/thresholds.yaml`
- `dist_monitor/config/calibration.yaml`
- `scripts/check_monitor_metrics.py`
- `scripts/verify_monitor_metrics.sh`

讲解主线：

> Monitor 层订阅双源 joint states，对齐时间戳后在滑窗内计算 KL、W1 和 MMD，并把是否发生偏移、主要关节和统计值发布给风险引擎与 HOC。

可以重点讲：

- 滑窗长度和发布频率是可配置验收项。
- KL 用于可解释的逐关节分布差异。
- W1 对均值漂移敏感。
- MMD 适合捕捉多维联合分布差异。
- 阈值支持参数热更新，便于在 Demo 中调整敏感度。
- 监控结果可落盘为 CSV、rosbag 和 JSON，支撑离线分析。

作品集证据：

- [monitor-metrics.json](../samples/monitor-metrics.json)
- [monitor-metrics-timeline.csv](../samples/monitor-metrics-timeline.csv)
- [same-task-calibration-report.html](../samples/same-task-calibration-report.html)
- [m4-monitor-metrics.png](../assets/m4-monitor-metrics.png)

## 3. 风险层：R0-R3 与安全闭环

核心文件：

- `risk_engine/risk_engine/risk_node.py`
- `risk_engine/config/risk_thresholds.yaml`
- `scripts/check_risk_management.py`
- `scripts/check_safety_nfr.py`
- `scripts/verify_risk_management.sh`
- `scripts/verify_safety_nfr.sh`

讲解主线：

> Risk 层把分布偏移、tracking error、安全、通信健康和系统健康聚合成五维风险，并映射到 R0-R3。R3 会触发急停，恢复前必须由 HOC 或服务 acknowledge。

可以重点讲：

- 每次风险状态包含 composite score、risk level、primary driver 和五维 attribution。
- R2 进入 degraded mode，可用于展示降级策略。
- R3 触发 E_STOP，联动 bridge 停止运动。
- acknowledge / clear 分离，避免风险未确认就恢复运行。
- 风险状态既服务 UI，也服务自动化验收脚本。

作品集证据：

- [risk-management-metrics.json](../samples/risk-management-metrics.json)
- [safety-nfr-metrics.json](../samples/safety-nfr-metrics.json)

## 4. 运维层：HOC 控制台

核心文件：

- `hoc_console/hoc_console/hoc_server.py`
- `hoc_console/frontend/src/App.tsx`
- `hoc_console/frontend/src/hooks/useWebSocket.ts`
- `hoc_console/frontend/src/components/DistributionPanel.tsx`
- `hoc_console/frontend/src/components/TrendChart.tsx`
- `hoc_console/frontend/src/components/RiskRadar.tsx`
- `hoc_console/frontend/src/components/RiskBanner.tsx`
- `hoc_console/launch/hoc_experiment.launch.py`

讲解主线：

> HOC 把 ROS 2 topic/service 抽象成 WebSocket 数据流和控制命令，让评审能从一个界面看到风险、分布、tracking、相机和实验报告。

可以重点讲：

- 后端 `hoc_server` 聚合 ROS 2 数据并推送 WebSocket。
- 前端用 ECharts 展示分布箱线图、趋势图和五维风险雷达。
- 控制命令包括 pause、resume、e-stop、acknowledge、set_randomization、inject_shift。
- 实验报告可导出 JSON/CSV，形成可审计记录。
- `hoc_experiment.launch.py` 把 portfolio demo 和 HOC 组合成一键演示入口。

作品集证据：

- [hoc-console-metrics.json](../samples/hoc-console-metrics.json)
- [hoc-verification-reports/fr_hoc_verify.json](../samples/hoc-verification-reports/fr_hoc_verify.json)
- [hoc-verification-reports/fr_hoc_verify.csv](../samples/hoc-verification-reports/fr_hoc_verify.csv)
- [m5-hoc-dashboard.png](../assets/m5-hoc-dashboard.png)

## 5. 联调与报告链路

核心文件：

- `scripts/run_dual_repo_integration.sh`
- `scripts/run_same_task_calibration.sh`
- `scripts/regenerate_all_reports.py`
- `docs/samples/dual-repo-experiment-report.html`
- `docs/samples/same-task-calibration-report.html`

讲解主线：

> 联调部分把当前 bridge 仓库和外部 episode-data-lab / LeRobot 数据接起来，证明监控链路不只消费本仓库里的 synthetic demo，也能处理外部数据集导出的轨迹。

可以重点讲：

- `EPISODE_DATA_LAB_ROOT` 指向外部数据仓。
- LeRobot export 提供 Real 侧回放数据。
- bridge 录制 Sim 侧轨迹，offline compare 输出跨源指标。
- 报告脚本把 JSON/NPZ 转成图表和 HTML，减少手工整理。

作品集证据：

- [dual-repo-experiment-report.html](../samples/dual-repo-experiment-report.html)
- [dual-repo-online-smoke.json](../samples/dual-repo-online-smoke.json)
- [dual-repo-cross-source-metrics.json](../samples/dual-repo-cross-source-metrics.json)

## 6. 建议讲解顺序

1. `portfolio_demo.launch.py`：从 launch 入口讲系统怎么组合。
2. `bridge_node.py`：讲 MoveIt / PyBullet 闭环和双源状态。
3. `monitor_node.py`：讲指标如何产生。
4. `risk_node.py`：讲指标如何变成 R0-R3 决策。
5. `hoc_server.py` + `App.tsx`：讲数据如何到 UI，控制命令如何回到 ROS 2。
6. `scripts/verify_*.sh`：讲验收如何复现。

## 7. 面试时避免误解

- 不把双源 PyBullet 说成真机接入；它是当前版本的 Real proxy。
- 不把 `FollowJointTrajectory` relay 说成完整 `ros2_control` 硬件接口。
- 不把样例 JSON 当成唯一证据；它们应和 verify 脚本、报告、README 边界一起出现。
- 不展示 `.ros-log/`、`.coverage`、`.capture_tmp/` 这类本地中间文件。
