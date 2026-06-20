# 作品集 Demo 脚本

目标时长：5-8 分钟。目标观众：机器人集成、ROS 2、仿真验证或平台工程方向的面试官。

## 演示主线

一句话开场：

> 这个项目把 MoveIt 2 规划、PyBullet 物理执行、Sim/Real 分布偏移监控、风险闭环和 HOC 运维控制台串成一个可脚本化验收的 ROS 2 联调环境。

推荐顺序：

1. 先展示 README 架构图和接口表，让观众知道系统边界。
2. 启动 portfolio demo，说明 MoveIt 输出如何进入 PyBullet。
3. 打开 HOC 控制台，展示风险、分布、tracking 和控制命令。
4. 注入偏移，观察 KL / W1 / MMD 与风险等级变化。
5. 触发 R3 或急停，展示 acknowledge / resume 的安全闭环。
6. 打开联调报告和验收摘要，说明证据如何沉淀。

## 准备工作

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ~/ros2_ws/src/ros2-moveit-pybullet-bridge
source setup.sh
```

如果需要 HOC 前端开发模式，确认 Node.js / npm 可用；`hoc.launch.py` 会在 `hoc_console/frontend` 下启动前端。

## 方案 A：完整组合入口

适合录屏或现场演示，一条命令拉起 portfolio demo + HOC：

```bash
ros2 launch hoc_console hoc_experiment.launch.py sim_mode:=DIRECT
```

讲解重点：

- `portfolio_demo.launch.py` 启动 iiwa7 双源 PyBullet、分布监控、风险引擎与运动 demo。
- `hoc_console` 通过 WebSocket 聚合 `/risk/status`、`/monitor/distribution_metrics`、`/monitor/tracking_error`。
- HOC 控制命令会落到 `/bridge/*` 和 `/risk/*` 服务，形成可操作闭环。

## 方案 B：分步展示

适合排障或需要展示 PyBullet GUI：

终端 1：

```bash
ros2 launch pybullet_bridge portfolio_demo.launch.py sim_mode:=GUI
```

终端 2：

```bash
ros2 launch hoc_console hoc.launch.py
```

可选 MoveIt 闭环：

```bash
ros2 launch moveit_config m2_iiwa_demo.launch.py
```

讲解重点：

- GUI 中展示 PyBullet 机械臂物理执行。
- MoveIt / RViz 中展示 Plan & Execute。
- HOC 中展示同一套运行状态和风险闭环。

## 关键演示动作

### 1. MoveIt 到 PyBullet 闭环

讲法：

> MoveIt 规划出来的轨迹通过 `FollowJointTrajectory` relay 转成 `/bridge/command`，PyBullet 执行后发布 `/joint_states`，再反馈给 MoveIt 和 TF。

可引用证据：

- [moveit-closure-metrics.json](../samples/moveit-closure-metrics.json)：4/4 planning goals 成功，最大 RMSE = 0.006004 rad。
- [m2-iiwa-pipeline.svg](../assets/m2-iiwa-pipeline.svg)：闭环结构图。

### 2. 双源 Sim/Real 监控

讲法：

> 这里的 Real 在当前版本中由第二个 PyBullet 实例或 LeRobot 回放代理。它不是宣称已经接真机，而是为了在无硬件条件下构造可控、可复现的偏移样本。

可引用证据：

- [monitor-metrics.json](../samples/monitor-metrics.json)：10 Hz 指标、5 s 滑窗、3/3 注入检出。
- [same-task-calibration-report.html](../samples/same-task-calibration-report.html)：同任务双源校准。

### 3. 偏移注入和风险升级

HOC 中执行 `inject_shift` 或调用服务注入 `joint_damping +20%`。

讲法：

> 注入已知偏移后，监控层会输出 KL / W1 / MMD，风险引擎把分布、tracking、安全、通信和系统健康聚合成 R0-R3。

可引用证据：

- [risk-management-metrics.json](../samples/risk-management-metrics.json)：R0-R3 转换均小于 500 ms。
- [m4-monitor-metrics.png](../assets/m4-monitor-metrics.png)：分布指标图。

### 4. R3 急停与恢复

HOC 中触发 e-stop，随后 acknowledge，再 resume。

讲法：

> R3 不只是 UI 告警，它会联动 bridge 进入 E_STOP；恢复前必须先 acknowledge，这避免了未确认风险直接清除。

可引用证据：

- [risk-management-metrics.json](../samples/risk-management-metrics.json)：速度归零延迟 0.796 ms。
- [hoc-console-metrics.json](../samples/hoc-console-metrics.json)：pause/e_stop/acknowledge/resume 命令均通过。

### 5. 联调仓库报告

打开 [dual-repo-experiment-report.html](../samples/dual-repo-experiment-report.html)。

讲法：

> 这一部分证明当前 bridge 仓库可以消费外部 episode-data-lab / LeRobot 数据，并把跨源对比结果沉淀成 HTML、JSON 和图表。它展示的是跨仓数据链路与报告闭环，不把当前版本包装成已经完成真机接入。

可引用证据：

- [dual-repo-online-smoke.json](../samples/dual-repo-online-smoke.json)：在线 LeRobot smoke 样本。
- [dual-repo-cross-source-metrics.json](../samples/dual-repo-cross-source-metrics.json)：跨源指标。

## 录屏分镜

| 时间 | 画面 | 讲解重点 |
|------|------|----------|
| 0:00-0:45 | README 架构图 | 项目定位、五层架构、关键接口 |
| 0:45-1:45 | launch / PyBullet | MoveIt 规划如何进入仿真闭环 |
| 1:45-3:00 | HOC dashboard | 风险、分布、tracking、控制按钮 |
| 3:00-4:30 | inject shift | KL/W1/MMD 与风险等级变化 |
| 4:30-5:30 | e-stop / acknowledge / resume | R3 安全闭环 |
| 5:30-6:30 | HTML 报告与验收摘要 | 工程证据、联调仓库、交付边界 |
| 6:30-7:30 | 代码导览可选 | bridge / monitor / risk / HOC 四个核心模块 |

## 常见追问回答

**问：Real 侧是真机吗？**  
当前版本不是。Real 侧由双源 PyBullet 或 LeRobot 回放代理，目标是在无硬件条件下验证监控、风险和报告链路。真机 `real_source:=ros2` 属于 Phase-2+。

**问：为什么不用完整 `ros2_control`？**  
当前作品集主线用 `FollowJointTrajectory` relay 打通 MoveIt 到 PyBullet 的闭环，已满足演示和验收证据。完整硬件接口是后续工程化增强。

**问：指标为什么用 KL / W1 / MMD 三个？**  
KL 便于解释逐关节分布差异，W1 对均值漂移敏感，MMD 能捕捉多维非高斯差异。三者组合更适合 Sim2Real 偏移监控。

**问：项目最像真实交付的部分是什么？**  
不是单个动画，而是脚本化验收和证据闭环：launch、verify 脚本、JSON/CSV/HTML、rosbag metadata、HOC 报告和明确的交付边界。
