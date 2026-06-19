#!/usr/bin/env bash
# 3-minute HOC demo script for interviews / portfolio review.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${ROS_WS:-$HOME/ros2_ws}"

echo "=== HOC 3-min Demo Script ==="
echo "Workspace: $WS"

# shellcheck disable=SC1091
source "$WS/install/setup.bash"

echo
echo "[1/5] 启动完整 ROS 系统 (后台)..."
ros2 launch pybullet_bridge full_system.launch.py &
FULL_PID=$!
sleep 8

echo
echo "[2/5] 启动 HOC 控制台 (后台)..."
ros2 launch hoc_console hoc.launch.py &
HOC_PID=$!
sleep 6

echo
echo "[3/5] 等待监控话题..."
timeout 30 bash -c 'until ros2 topic echo /monitor/distribution_metrics --once >/dev/null 2>&1; do sleep 1; done'
echo "  /monitor/distribution_metrics OK"

echo
echo "[4/5] 演示动作序列..."
echo "  → 设置随机化强度 50%"
ros2 service call /bridge/set_randomization bridge_monitor_msgs/srv/SetRandomization \
  "{config: {random_seed: 42, randomization_strength: 0.5, joint_damping_min: 0.5, joint_damping_max: 2.0, joint_friction_min: 0.0, joint_friction_max: 0.2, motor_strength_min: 0.8, motor_strength_max: 1.2, position_noise_std: 0.01, velocity_noise_std: 0.05, time_delay_min_ms: 0.0, time_delay_max_ms: 50.0, payload_mass_min: 0.0, payload_mass_max: 5.0}}" \
  >/dev/null 2>&1 || true

sleep 3
echo "  → 注入 Ground Truth 偏移 (+30% damping, 10s)"
ros2 service call /bridge/inject_shift bridge_monitor_msgs/srv/InjectShift \
  "{parameter_name: 'joint_damping', delta_percent: 30.0, duration_sec: 10.0}" \
  >/dev/null 2>&1 || true

sleep 5
echo "  → 运行 SC-01 场景"
ros2 action send_goal /hoc/execute_scenario bridge_monitor_msgs/action/ExecuteScenario \
  "{scenario_id: 'SC-01', random_seed: 42, randomization_strength: 0.5, record_bag: false}" \
  --feedback >/dev/null 2>&1 || true

echo
echo "[5/5] 打开浏览器控制台"
echo "  URL: http://localhost:5173"
echo "  WebSocket: ws://localhost:8765"
echo
echo "演示要点 (约 3 分钟):"
echo "  1. 顶部 RiskBanner 显示 R0→R2 变化与趋势箭头"
echo "  2. 雷达图 D1(分布偏移) 升高，分布对比面板检出 shift"
echo "  3. KL/MMD 时序曲线实时更新"
echo "  4. 点击「导出报告」生成 HTML (~/ros2_ws/reports/)"
echo
echo "按 Ctrl+C 停止所有进程"

cleanup() {
  kill "$HOC_PID" "$FULL_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait
