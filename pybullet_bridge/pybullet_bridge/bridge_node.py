"""ROS 2 node bridging control trajectories to PyBullet simulation."""

from __future__ import annotations

import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

from bridge_monitor_msgs.msg import DomainRandomizationConfig, ExperimentMetadata
from bridge_monitor_msgs.srv import InjectShift, SetRandomization

from pybullet_bridge.real_source import RealSource, RealSourceConfig
from pybullet_bridge.sim_source import SimSource, SimSourceConfig
from pybullet_bridge.trajectory_executor import TrajectoryExecutor


class PyBulletBridgeNode(Node):
    """Bridge ros2_control trajectories to PyBullet and publish joint states."""

    def __init__(self) -> None:
        super().__init__('pybullet_bridge')

        self.declare_parameter('urdf_path', '')
        self.declare_parameter('sim_mode', 'DIRECT')
        self.declare_parameter('physics_frequency', 240.0)
        self.declare_parameter('publish_frequency', 100.0)
        self.declare_parameter('enable_dual_source', False)
        self.declare_parameter('random_seed', 42)
        self.declare_parameter('randomization_strength', 0.5)
        self.declare_parameter('watchdog_timeout_ms', 5000)
        self.declare_parameter('home_positions', [0.0, 0.0])

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._joint_pub = self.create_publisher(
            JointState, '/joint_states', reliable_qos)
        self._sim_joint_pub = self.create_publisher(
            JointState, '/bridge/sim/joint_states', qos_profile_sensor_data)
        self._real_joint_pub = self.create_publisher(
            JointState, '/bridge/real/joint_states', qos_profile_sensor_data)
        self._torque_pub = self.create_publisher(
            JointState, '/bridge/sim/joint_torques', qos_profile_sensor_data)

        self._system_state_pub = self.create_publisher(
            String, '/bridge/system_state', reliable_qos)
        self._randomization_pub = self.create_publisher(
            DomainRandomizationConfig, '/bridge/randomization_config', reliable_qos)
        self._metadata_pub = self.create_publisher(
            ExperimentMetadata, '/bridge/experiment_metadata', reliable_qos)

        self.create_subscription(
            JointTrajectory, '/bridge/command', self._on_command, 10)

        self.create_service(SetRandomization, '/bridge/set_randomization',
                            self._handle_set_randomization)
        self.create_service(InjectShift, '/bridge/inject_shift', self._handle_inject_shift)
        self.create_service(Trigger, '/bridge/reset_simulation', self._handle_reset)
        self.create_service(SetBool, '/bridge/set_mode', self._handle_set_mode)

        urdf_path = self._resolve_urdf_path()
        use_gui = self.get_parameter('sim_mode').value == 'GUI'
        physics_freq = self.get_parameter('physics_frequency').value
        home_positions = list(self.get_parameter('home_positions').value)

        sim_config = SimSourceConfig(
            urdf_path=urdf_path,
            use_gui=use_gui,
            physics_frequency=physics_freq,
            home_positions=home_positions,
        )
        self._sim = SimSource(sim_config)
        self._real: RealSource | None = None
        self._dual_source = self.get_parameter('enable_dual_source').value

        if self._dual_source:
            self._real = RealSource(RealSourceConfig(
                urdf_path=urdf_path,
                physics_frequency=physics_freq,
                home_positions=home_positions,
                random_seed=self.get_parameter('random_seed').value,
                randomization_strength=self.get_parameter('randomization_strength').value,
            ))

        self._pybullet_ok = self._sim.initialize()
        if self._dual_source and self._real is not None:
            self._pybullet_ok = self._pybullet_ok and self._real.initialize()

        if not self._pybullet_ok:
            self.get_logger().error(
                f'PyBullet init failed. urdf_path={urdf_path}')
        else:
            self.get_logger().info(
                f'PyBullet ready: joints={self._sim.joint_names}, urdf={urdf_path}')

        self._trajectory = TrajectoryExecutor()
        if self._pybullet_ok:
            snap = self._sim.read_state()
            self._trajectory.set_hold_positions(snap.names, snap.positions)

        self._paused = False
        self._e_stop = False
        self._last_command_time = time.monotonic()
        self._sim_start_mono = time.monotonic()

        physics_hz = self.get_parameter('physics_frequency').value
        pub_hz = self.get_parameter('publish_frequency').value
        self._physics_timer = self.create_timer(1.0 / physics_hz, self._on_physics_step)
        self._publish_timer = self.create_timer(1.0 / pub_hz, self._on_publish)
        self._meta_timer = self.create_timer(1.0, self._publish_metadata)

        self._latest_sim_snapshot = self._sim.read_state() if self._pybullet_ok else None
        self._latest_real_snapshot = None

    def _resolve_urdf_path(self) -> str:
        urdf_path = self.get_parameter('urdf_path').value
        if urdf_path and os.path.isfile(urdf_path):
            return urdf_path
        share = get_package_share_directory('pybullet_bridge')
        default_path = os.path.join(share, 'urdf', 'planar_2dof.urdf')
        return default_path

    def _on_command(self, msg: JointTrajectory) -> None:
        if not self._pybullet_ok or self._e_stop:
            return
        self._last_command_time = time.monotonic()
        self._trajectory.set_trajectory(msg, self._sim_time_sec())
        self.get_logger().info(
            f'Trajectory received: joints={list(msg.joint_names)}, '
            f'points={len(msg.points)}')

    def _sim_time_sec(self) -> float:
        return time.monotonic() - self._sim_start_mono

    def _on_physics_step(self) -> None:
        if not self._pybullet_ok or self._paused or self._e_stop:
            return

        try:
            watchdog_ms = self.get_parameter('watchdog_timeout_ms').value
            if not self._trajectory.has_active_trajectory:
                if (time.monotonic() - self._last_command_time) * 1000.0 > watchdog_ms:
                    pass  # idle hold — no active trajectory to clear

            targets = self._trajectory.sample(self._sim_time_sec())
            self._sim.set_position_targets_by_name(targets)
            self._latest_sim_snapshot = self._sim.step()

            if self._dual_source and self._real is not None:
                self._real.set_position_targets_by_name(targets)
                self._latest_real_snapshot = self._real.step()

            self.get_logger().debug(
                f'physics: targets={targets} pos={self._latest_sim_snapshot.positions}',
                throttle_duration_sec=2.0,
            )
        except Exception as exc:
            self.get_logger().error(f'physics step failed: {exc}')

    def _on_publish(self) -> None:
        stamp = self.get_clock().now().to_msg()

        state = String()
        if self._e_stop:
            state.data = 'E_STOP'
        elif self._paused:
            state.data = 'PAUSED'
        elif self._pybullet_ok:
            state.data = 'RUNNING'
        else:
            state.data = 'IDLE'
        self._system_state_pub.publish(state)

        if not self._pybullet_ok:
            return

        snapshot = self._sim.read_state()
        if not snapshot.names:
            return

        sim_js = self._snapshot_to_joint_state(snapshot, stamp)
        self._joint_pub.publish(sim_js)
        self._sim_joint_pub.publish(sim_js)

        torque_js = JointState()
        torque_js.header = sim_js.header
        torque_js.name = sim_js.name
        torque_js.effort = sim_js.effort
        self._torque_pub.publish(torque_js)

        if self._dual_source and self._latest_real_snapshot is not None:
            self._real_joint_pub.publish(
                self._snapshot_to_joint_state(self._latest_real_snapshot, stamp))
        else:
            self._real_joint_pub.publish(sim_js)

    @staticmethod
    def _snapshot_to_joint_state(snapshot, stamp) -> JointState:
        js = JointState()
        js.header.stamp = stamp
        js.name = list(snapshot.names)
        js.position = [float(v) for v in snapshot.positions]
        js.velocity = [float(v) for v in snapshot.velocities]
        js.effort = [float(v) for v in snapshot.efforts]
        return js

    def _publish_metadata(self) -> None:
        meta = ExperimentMetadata()
        meta.header.stamp = self.get_clock().now().to_msg()
        meta.experiment_id = 'm1_bridge'
        meta.scenario_id = 'SC-00'
        meta.random_seed = self.get_parameter('random_seed').value
        meta.randomization_strength = self.get_parameter('randomization_strength').value
        self._metadata_pub.publish(meta)

        cfg = DomainRandomizationConfig()
        cfg.header.stamp = meta.header.stamp
        cfg.random_seed = meta.random_seed
        cfg.randomization_strength = meta.randomization_strength
        self._randomization_pub.publish(cfg)

    def _handle_set_randomization(self, request, response):
        response.success = True
        response.message = 'Randomization update accepted (M3: full implementation pending).'
        return response

    def _handle_inject_shift(self, request, response):
        response.success = True
        response.message = (
            f'Inject {request.parameter_name} +{request.delta_percent}% '
            f'for {request.duration_sec}s (M3 pending).')
        return response

    def _handle_reset(self, request, response):
        if self._pybullet_ok:
            self._sim.reset()
            if self._real is not None:
                self._real.reset()
            snap = self._sim.read_state()
            self._trajectory.clear()
            self._trajectory.set_hold_positions(snap.names, snap.positions)
            self._sim_start_mono = time.monotonic()
            self._e_stop = False
            self._paused = False
        response.success = True
        response.message = 'Simulation reset to home.'
        return response

    def _handle_set_mode(self, request, response):
        self._paused = not request.data
        response.success = True
        response.message = 'RUNNING' if request.data else 'PAUSED'
        return response

    def destroy_node(self) -> bool:
        self._sim.shutdown()
        if self._real is not None:
            self._real.shutdown()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PyBulletBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
