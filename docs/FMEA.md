# FMEA · 失效模式与影响分析

**范围**：MoveIt 2 → Policy Runner → PyBullet Bridge → dist_monitor → risk_engine → HOC 的闭环系统。

---

| ID | 失效模式 | 影响 | 严重度 | 发生概率 | 可检测性 | 现有缓解措施 | 建议检测手段 | 验收证据 |
|----|----------|------|--------|----------|----------|--------------|--------------|----------|
| FMEA-01 | PyBullet 物理引擎崩溃或 step 卡死 | `/joint_states` 停止更新，MoveIt 当前状态过期，策略观测 stale | 高 | 中 | 高 | bridge watchdog、`/bridge/system_state=HOLD`、HOC 日志 | 监控 `/bridge/sim/joint_states` gap、`/monitor/comm_health` score、进程存活 | headless smoke 中 kill/阻塞 bridge 后 1 s 内出现 health/risk 报警 |
| FMEA-02 | MoveIt 规划超时或失败率升高 | 轨迹无法生成，任务执行中断；可能导致 operator 重复触发命令 | 中 | 中 | 高 | `risk_engine` planning stats、HOC 风险归因 | 订阅规划结果统计，记录 failure rate 和 timeout reason | 故意设置不可达目标时 `/risk/status` 至少 R1/R2 |
| FMEA-03 | 分布偏移异常激增 | Sim/Real 轨迹差异扩大，仿真结论不可信 | 高 | 中 | 高 | `dist_monitor` KL/W1/MMD 阈值、`risk_engine` R2/R3 | 注入 domain randomization shift，观察 `/monitor/distribution_metrics.shift_detected` | `verify_monitor_metrics.sh` 或 benchmark 报告包含 shift 检出 |
| FMEA-04 | 策略推理卡顿或异常 | `/bridge/command` 更新变慢，机器人保持旧目标或停止响应 | 高 | 中 | 高 | `PolicyRunner` fault watchdog、`/system_health`、bridge command watchdog | 在 `policy_runner` 注入 `sleep(0.1s)` 或抛异常，测报警延迟 | `/system_health` 在 1 s 内发布 WARN/ERROR，benchmark summary 标记通过 |
| FMEA-05 | JointState 高频流丢帧或延迟 | dist_monitor 时间对齐质量下降，策略观测延迟，风险误判 | 中 | 中 | 中 | `dist_monitor.comm_health`、BestEffort 防反压 | 统计 measured_hz、gap_count、latency EWMA | `/monitor/comm_health` score 上升并进入风险聚合 |
| FMEA-06 | `/bridge/command` joint order 或长度不匹配 | PyBullet 执行错误关节目标，可能导致仿真不稳定 | 高 | 低 | 高 | robot profile joint names、TrajectoryExecutor 校验 | PolicyRunner 发布前校验 action 长度和 joint_names；bridge 拒绝非法 trajectory | 单元测试覆盖缺关节、乱序、超限命令 |
| FMEA-07 | Benchmark 脚本未清理后台进程 | 后续 CI job 端口冲突或结果污染 | 中 | 中 | 高 | `trap cleanup EXIT`、输出目录隔离 | 检查 ros2 launch pid、端口、output-dir 时间戳 | `run_system_validation.sh` 失败路径返回非 0 且无残留进程 |

## 风险优先级建议

| 优先级 | 处理项 |
|--------|--------|
| P0 | FMEA-04 策略卡顿报警、FMEA-06 command 契约校验 |
| P1 | FMEA-01 bridge 卡死检测、FMEA-03 分布偏移检出 |
| P2 | FMEA-05 通信健康细化、FMEA-07 CI 清理健壮性 |

## 验证原则

- 每个高严重度失效都必须有可脚本化的注入方式。
- 故障注入只在 benchmark 或验证脚本中启用，默认运行路径必须关闭。
- 检测结果应落入 `docs/samples/system-validation/`，供 README 和作品集报告引用。
