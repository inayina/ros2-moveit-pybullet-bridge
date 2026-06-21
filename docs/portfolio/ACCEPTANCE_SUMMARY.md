# 作品集验收摘要

本页把 `docs/ACCEPTANCE_GAP.md` 和 `docs/samples/*-metrics.json` 中的验收记录压缩成作品集可读版本。详细逐项台账仍以 [ACCEPTANCE_GAP.md](../ACCEPTANCE_GAP.md) 为准。

## 总体结论

当前仓库已经具备面试 Demo 主线：MoveIt 2 / PyBullet 闭环、双源分布监控、五维风险闭环、HOC 控制台、Policy Runner 系统验证、报告导出与脚本化复验。主要剩余工作不是核心链路不可用，而是公开展示材料的补齐：CI 最新绿勾、完整 Demo 视频外链、2 小时长稳等长期证据。

## 功能需求证据

| 范围 | 当前结论 | 关键证据 |
|------|----------|----------|
| Bridge 通信 | 部分满足 | `bridge-comm-metrics.json`：command→feedback P99 = 10.524 ms；三路 joint state 均为 100.0 Hz；header stamp jitter = 5.501%，略高于 5% 严格阈值 |
| MoveIt 闭环 | 已满足 | `moveit-closure-metrics.json`：4/4 MoveGroup joint goals 成功，最大 RMSE = 0.006004 rad，TF lookup 通过，blocking collision scene 被拒绝 |
| 分布监控 | 已满足 | `monitor-metrics.json`：10 Hz 输出，5 s 滑窗，MMD p-value = 0.019608，3/3 偏移注入检出 |
| 风险管理 | 已满足 | `risk-management-metrics.json`：R0-R3 转换均 < 500 ms，R3 后 bridge 进入 E_STOP，速度归零延迟 0.796 ms |
| HOC 控制台 | 已满足 | `hoc-console-metrics.json`：三路 WebSocket stream 5 Hz，最大延迟 70.712 ms，控制命令、参数调节、JSON/CSV 导出均通过 |
| Policy Runner | 已满足 | `system-validation/validation_summary.json`：Replay / SineWave benchmark PASS；SineWave mean latency 4.785 ms；`/system_health` 报警在 1 s 内检出 |

## 非功能需求证据

| 范围 | 当前结论 | 关键证据 |
|------|----------|----------|
| 性能 | 已满足 | `performance-nfr-metrics.json`：控制回路 P95 = 14.571 ms，monitor 10 Hz，HOC 5 Hz，双源物理步进 min = 239.507 Hz/source，RTF min = 0.998 |
| 可靠性 | 部分满足 | `reliability-nfr-metrics.json`：覆盖 watchdog HOLD、reset 恢复、短时 smoke/RSS、HOC 独立 rosbag；2 小时 soak 与跨进程 supervisor 属于 Phase-2+ |
| 安全 | 已满足 | `safety-nfr-metrics.json`：R3 急停速度归零、软限位 R2、watchdog HOLD、R2 降速、ack 恢复均通过 |
| 可维护性 | 已满足 | `maintainability-nfr-metrics.json`：YAML/launch/package 结构检查通过，核心包测试 142 passed，coverage = 73.2% |
| 可复现性 | 已满足 | 9 个 `scripts/verify_*.sh` 可执行且有样例产物；HOC 导出 JSON/CSV，rosbag metadata 与配置结果可关联 |

## 联调仓库证据

跨仓库联动材料集中在 `docs/samples/`：

- [dual-repo-experiment-report.html](../samples/dual-repo-experiment-report.html)：正式联调报告，展示 bridge Sim NPZ 与 LeRobot Real 数据的跨源对比。
- [dual-repo-online-smoke.json](../samples/dual-repo-online-smoke.json)：`real_source:=lerobot` 在线监控 smoke，sim/real 均有样本。
- [dual-repo-cross-source-metrics.json](../samples/dual-repo-cross-source-metrics.json)：跨源分布指标，用于说明真实数据回放与仿真分布差异。
- [same-task-calibration-report.html](../samples/same-task-calibration-report.html)：同任务校准报告，适合解释为什么需要 KL / W1 / MMD 组合指标。

作品集讲解时建议这样表达：联调仓库不是“真机已接入”的证明，而是“外部数据仓可接入当前 ROS 2 监控链路，并能形成可审计实验报告”的证明。

## 交付边界

当前版本边界是仿真预集成、分布监控、风险闭环和 HOC 运维控制台。以下内容明确归入 Phase-2+，不作为当前作品集主线阻塞：

- 真机 `real_source:=ros2`。
- 完整 `ros2_control` 硬件接口。
- episode-data-lab `Ros2Robot` HAL。
- `/clock` + `use_sim_time` 全链路严格同步。
- 2 小时长稳、valgrind 或跨进程 supervisor 硬化。

## 投递前检查

- 确认 GitHub Actions 最新提交为绿勾。
- 给 Demo 视频准备公开链接，README 中只保留短入口。
- 若公开仓库，清理 `.ros-log/`、`.ros_log/`、`docs/samples/system-validation/ros_logs/`、`.coverage`、`.capture_tmp/` 和大体积 `.mcap` 原始包。
- 保留 `docs/samples/README.md` 作为原始证据索引，避免在作品集主页面堆满 JSON 文件。
