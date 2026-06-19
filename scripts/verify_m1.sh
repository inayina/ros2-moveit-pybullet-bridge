#!/usr/bin/env bash
# M1 一键验证：环境检查 → 清理残留进程 → 启动 demo → 自动判定 PASS/FAIL
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# shellcheck source=verify_env.sh
source "${ROOT}/scripts/verify_env.sh"
verify_env_init "${ROOT}"

echo "==> 1/5 环境就绪（ROS Jazzy + system Python 3.12）"

echo "==> 2/5 前置检查"
verify_env_require_pkg pybullet_bridge

echo "==> 3/5 清理残留 ROS 节点"
pkill -f "ros2 launch pybullet_bridge" 2>/dev/null || true
pkill -f "ros2 launch moveit_config" 2>/dev/null || true
pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
pkill -f "joint_sweep_demo" 2>/dev/null || true
sleep 1

echo "==> 4/5 启动 M1 demo（15s 后自动检测）"
ros2 launch pybullet_bridge m1_demo.launch.py &
LAUNCH_PID=$!
cleanup() { kill "$LAUNCH_PID" 2>/dev/null || true; wait "$LAUNCH_PID" 2>/dev/null || true; }
trap cleanup EXIT

sleep 5

echo "==> 5/5 检测关节运动"
python3 - <<'PY'
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

class Checker(Node):
    def __init__(self):
        super().__init__('m1_checker')
        self.samples = []
        self.create_subscription(
            JointState, '/bridge/sim/joint_states', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        if len(msg.position) >= 2:
            self.samples.append((list(msg.name), list(msg.position)))

rclpy.init()
node = Checker()
deadline = time.time() + 8
while time.time() < deadline and len(node.samples) < 10:
    rclpy.spin_once(node, timeout_sec=0.2)

if len(node.samples) < 2:
    print('[FAIL] 未收到 /bridge/sim/joint_states，请确认 bridge 已启动且 colcon build 完成')
    rclpy.shutdown()
    sys.exit(1)

names = node.samples[-1][0]
peak = max(max(abs(v) for v in pos) for _, pos in node.samples)
first = node.samples[0][1]
last = node.samples[-1][1]

print(f'  关节: {names}')
print(f'  首样本: {[round(x, 3) for x in first]}')
print(f'  末样本: {[round(x, 3) for x in last]}')
print(f'  峰值幅度: {peak:.3f} rad')

if names != ['joint1', 'joint2']:
    print('[FAIL] 关节名不匹配，可能有 M2/UR5 残留进程。请重新运行本脚本')
    rclpy.shutdown()
    sys.exit(1)

if peak < 0.3:
    print('[FAIL] 关节未达到预期幅度（<0.3 rad）。检查轨迹是否下发')
    rclpy.shutdown()
    sys.exit(1)

print('[PASS] M1 验证通过：轨迹下发 + 关节状态反馈正常')
rclpy.shutdown()
PY
