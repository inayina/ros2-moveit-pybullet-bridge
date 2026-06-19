# 里程碑验证修复说明

本文档记录 `run_tests.sh` / `verify_m*.sh` 失败根因、HOC 交互界面功能缺陷及对应修复。

## 1. Launch 集成测试：`robot_launch_utils` 不可导入

**现象**

```
ModuleNotFoundError: No module named 'robot_launch_utils'
```

**根因**  
`robot_launch_utils.py` 放在 `launch/` 目录，colcon 将其安装到 `share/.../launch/`，Python 无法作为包模块 `import`。

**修复**

- 迁移至 `pybullet_bridge/pybullet_bridge/robot_launch_utils.py`
- 所有 launch 文件改为 `from pybullet_bridge.robot_launch_utils import declare_robot_launch_arg`
- 新增 `test/test_launch_imports.py`
- 若 build 缓存仍引用旧路径，需清理后重建：

```bash
rm -rf ~/ros2_ws/build/pybullet_bridge ~/ros2_ws/install/pybullet_bridge
cd ~/ros2_ws && colcon build --packages-select pybullet_bridge --symlink-install
```

## 2. 验证脚本与 Python 环境

**现象**  
`setup.sh` 激活 venv/conda 后，`rclpy` / `launch_testing` 与系统 ROS Jazzy (Python 3.12) 不兼容。

**修复**

- 新增 `scripts/verify_env.sh`：固定 `/usr/bin/python3`，unset venv/conda，`ROS_LOG_DIR` 指向仓库 `.ros_log`
- `verify_m1.sh`、`verify_m2_iiwa.sh`、`verify_portfolio.sh` 改为 source `verify_env.sh`

## 3. Bridge 未响应 risk E-stop

**现象**  
`risk_engine` 发布 `/risk/status` 且 `e_stop_active=true` 时，PyBullet 仍执行轨迹。

**修复**（`bridge_node.py`）

- 订阅 `/risk/status`
- `_on_risk_status()` 设置 `_e_stop`、清空轨迹；清除后恢复（非 pause 状态）
- 新增节点测试 `test_bridge_honors_risk_e_stop`

## 4. HOC 后端功能缺陷

| 问题 | 修复 |
|------|------|
| `start_experiment` 在同节点 ActionClient 自调用，单线程 spin 死锁 | `ExperimentRunner.start_scenario()` 后台线程执行；`hoc_server` 改用 `MultiThreadedExecutor` |
| `_build_randomization_config()` 默认值与 `bridge_config.yaml` 不一致 | 对齐 damping/friction/motor/payload 范围 |
| `set_randomization` 服务参数未完整应用 | `bridge_node._handle_set_randomization()` 重写，按 strength 插值各维参数 |
| 开始录制未广播 `recording_status` | `_handle_start_recording` 成功后 WebSocket broadcast |
| 缺少 `verify_hoc.sh` | 新增 `scripts/verify_hoc.sh` |

## 5. HOC 前端功能缺陷

| 问题 | 修复 |
|------|------|
| 未处理 `experiment_progress` | `useWebSocket` + `dashboardStore.ingestExperimentProgress` |
| 无暂停按钮 | 工具栏与实验区增加「暂停」→ `pause` 命令 |
| R3 确认后未清除 E-stop | `R3Modal` 在 `acknowledge` 成功后自动 `resume` |
| 录制状态仅本地更新 | 依赖后端 broadcast + command 返回 `bag_path` |

## 6. 关节名契约（bridge ↔ episode-data-lab）

已在双仓库对齐：`lbr_iiwa_joint_1…7` 为 canonical；bridge `joint_names.py` 映射 legacy `joint_0…6`。

## 验证命令

```bash
export EPISODE_DATA_LAB_ROOT=~/robot-sim-lab/robot-arm-episode-data-lab

cd ~/ros2_ws/src/ros2-moveit-pybullet-bridge
./scripts/run_tests.sh
./scripts/verify_m1.sh
./scripts/verify_m2_iiwa.sh
./scripts/verify_portfolio.sh
./scripts/verify_hoc.sh
./scripts/run_integration_demo.sh   # 双仓集成
```

HOC 完整 UI：

```bash
ros2 launch hoc_console hoc.launch.py
# 浏览器 http://localhost:5173 （dev）或 hoc_prod.launch.py :8080
```

## 7. 已知限制

- `portfolio_demo.launch.py` 仍不自动启动 `hoc_server`；需单独 `hoc.launch.py` 或 `hoc_prod.launch.py`
- Launch 集成测试依赖 PyBullet + 完整 colcon install，耗时较长
- 前端 `npm run build` 需在修改 TS 后执行，`hoc_prod` 才包含最新 UI

## 8. `/monitor/tracking_error` QoS 不匹配

**现象**  
`portfolio_demo` 启动时 `risk_engine` / `hoc_server` 报 RELIABILITY QoS 不兼容，跟踪误差无法进入风险聚合。

**根因**  
`dist_monitor` 以 `BEST_EFFORT`（sensor QoS）发布；订阅方使用默认 `RELIABLE`。

**修复**

- `risk_engine/risk_node.py` 与 `hoc_console/hoc_server.py` 订阅改为 `qos_profile_sensor_data`
- `verify_portfolio.sh` 增加 `dist_monitor` / `offline_compare` 可执行文件检查

## 9. `offline_compare` 未安装

**现象**  
`ros2 run dist_monitor offline_compare` → `No executable found`

**修复**  
重建 `dist_monitor` 包：

```bash
cd ~/ros2_ws && colcon build --packages-select dist_monitor --symlink-install
```
