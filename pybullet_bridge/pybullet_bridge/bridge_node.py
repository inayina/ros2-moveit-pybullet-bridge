"""ROS 2 node bridging control trajectories to PyBullet simulation."""

from __future__ import annotations

import os
import tempfile
import time

import rclpy
from builtin_interfaces.msg import Time as BuiltinTime
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

from bridge_monitor_msgs.msg import (
    DomainRandomizationConfig,
    ExperimentMetadata,
    SimRealError,
)
from bridge_monitor_msgs.srv import InjectShift, SetRandomization

from pybullet_bridge.domain_randomizer import DomainRandomizerConfig
from pybullet_bridge.real_source import RealSource, RealSourceConfig
from pybullet_bridge.robot_profiles import get_profile, resolve_urdf_path
from pybullet_bridge.sim_source import SimSource, SimSourceConfig
from pybullet_bridge.trajectory_executor import TrajectoryExecutor


class PyBulletBridgeNode(Node):
    """Bridge ros2_control trajectories to PyBullet and publish joint states."""

    def __init__(self) -> None:
        super().__init__('pybullet_bridge')

        self.declare_parameter('urdf_path', '')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('sim_mode', 'DIRECT')
        self.declare_parameter('physics_frequency', 240.0)
        self.declare_parameter('publish_frequency', 100.0)
        self.declare_parameter('enable_dual_source', True)
        self.declare_parameter('random_seed', 42)
        self.declare_parameter('randomization_strength', 1.0)
        self.declare_parameter('watchdog_timeout_ms', 5000)
        self.declare_parameter('robot_profile', 'iiwa7')
        self.declare_parameter('home_positions', [0.8, -0.6])
        self.declare_parameter('end_effector_link', 'tool0')
        self.declare_parameter('joint_damping_base', 0.04)
        self.declare_parameter('joint_friction_base', 0.1)
        self.declare_parameter('joint_damping_range', [0.0, 0.5])
        self.declare_parameter('joint_friction_range', [0.0, 0.3])
        self.declare_parameter('motor_strength_range', [0.85, 1.15])
        self.declare_parameter('time_delay_range_ms', [0.0, 50.0])
        self.declare_parameter('payload_mass_range', [0.0, 0.5])

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
        self._error_pub = self.create_publisher(
            SimRealError, '/bridge/sim_real_error', qos_profile_sensor_data)
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

        urdf_path, home_positions, end_effector_link = self._resolve_robot_setup()
        use_gui = self.get_parameter('sim_mode').value == 'GUI'
        physics_freq = self.get_parameter('physics_frequency').value

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
            self._real = RealSource(
            self._build_real_config(
                urdf_path, physics_freq, home_positions, end_effector_link),
                log_fn=lambda msg: self.get_logger().info(msg),
            )

        self._pybullet_ok = self._sim.initialize()
        if self._dual_source and self._real is not None:
            self._pybullet_ok = self._pybullet_ok and self._real.initialize()

        if not self._pybullet_ok:
            self.get_logger().error(
                f'PyBullet init failed. urdf_path={urdf_path}')
        else:
            mode = 'dual-source' if self._dual_source else 'sim-only'
            self.get_logger().info(
                f'PyBullet ready ({mode}): profile='
                f'{self.get_parameter("robot_profile").value}, '
                f'joints={self._sim.joint_names}, urdf={urdf_path}')
            if self._dual_source and self._real is not None and self._real.latest_sample:
                sample = self._real.latest_sample
                self.get_logger().info(
                    f'Real-Source initial randomization: '
                    f'damping={sample.joint_damping} friction={sample.joint_friction} '
                    f'payload={sample.payload_mass_kg:.3f}kg delay={sample.actuator_delay_ms:.1f}ms')

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

    def _build_real_config(
        self,
        urdf_path: str,
        physics_freq: float,
        home_positions: list[float],
        end_effector_link: str,
    ) -> RealSourceConfig:
        motor_range = list(self.get_parameter('motor_strength_range').value)
        delay_range = list(self.get_parameter('time_delay_range_ms').value)
        payload_range = list(self.get_parameter('payload_mass_range').value)
        return RealSourceConfig(
            urdf_path=urdf_path,
            physics_frequency=physics_freq,
            home_positions=home_positions,
            end_effector_link=end_effector_link,
            random_seed=self.get_parameter('random_seed').value,
            randomization_strength=self.get_parameter('randomization_strength').value,
            joint_damping_base=self.get_parameter('joint_damping_base').value,
            joint_friction_base=self.get_parameter('joint_friction_base').value,
            motor_strength_range=(motor_range[0], motor_range[1]),
            time_delay_range_ms=(delay_range[0], delay_range[1]),
            payload_mass_range=(payload_range[0], payload_range[1]),
        )

    def _resolve_robot_setup(self) -> tuple[str, list[float], str]:
        profile_name = str(self.get_parameter('robot_profile').value)
        profile = get_profile(profile_name)

        urdf_path = str(self.get_parameter('urdf_path').value)
        if not urdf_path or not os.path.isfile(urdf_path):
            robot_description = self.get_parameter('robot_description').value
            if robot_description:
                cache_dir = os.path.join(tempfile.gettempdir(), 'pybullet_bridge')
                os.makedirs(cache_dir, exist_ok=True)
                urdf_path = os.path.join(cache_dir, 'robot_description.urdf')
                with open(urdf_path, 'w', encoding='utf-8') as handle:
                    handle.write(robot_description)
            else:
                urdf_path = resolve_urdf_path(profile_name)

        home_positions = list(self.get_parameter('home_positions').value)
        if len(home_positions) != len(profile.home_positions):
            home_positions = list(profile.home_positions)

        end_effector_link = str(self.get_parameter('end_effector_link').value)
        if profile.name == 'iiwa7' and end_effector_link == 'tool0':
            end_effector_link = profile.end_effector_link

        return urdf_path, home_positions, end_effector_link

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

    @staticmethod
    def _sec_to_time_msg(t_sec: float) -> BuiltinTime:
        sec = int(t_sec)
        nanosec = int(round((t_sec - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        msg = BuiltinTime()
        msg.sec = sec
        msg.nanosec = nanosec
        return msg

    def _on_physics_step(self) -> None:
        if not self._pybullet_ok or self._paused or self._e_stop:
            return

        try:
            if self._dual_source and self._real is not None:
                self._real.tick_inject_shift(time.monotonic())

            watchdog_ms = self.get_parameter('watchdog_timeout_ms').value
            if not self._trajectory.has_active_trajectory:
                if (time.monotonic() - self._last_command_time) * 1000.0 > watchdog_ms:
                    pass  # idle hold — no active trajectory to clear

            t_sim = self._sim_time_sec()
            targets = self._trajectory.sample(t_sim)
            self._sim.set_position_targets_by_name(targets)
            self._latest_sim_snapshot = self._sim.step()

            if self._dual_source and self._real is not None:
                self._real.set_position_targets_by_name(targets)
                self._latest_real_snapshot = self._real.step(t_sim)

            self.get_logger().debug(
                f'physics: targets={targets} pos={self._latest_sim_snapshot.positions}',
                throttle_duration_sec=2.0,
            )
        except Exception as exc:
            self.get_logger().error(f'physics step failed: {exc}')

    def _on_publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        t_sim = self._sim_time_sec()

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

        sim_snapshot = self._sim.read_state()
        if not sim_snapshot.names:
            return

        sim_js = self._snapshot_to_joint_state(sim_snapshot, stamp)
        self._joint_pub.publish(sim_js)
        self._sim_joint_pub.publish(sim_js)

        torque_js = JointState()
        torque_js.header = sim_js.header
        torque_js.name = sim_js.name
        torque_js.effort = sim_js.effort
        self._torque_pub.publish(torque_js)

        if self._dual_source and self._latest_real_snapshot is not None:
            real_snapshot = self._latest_real_snapshot
            self._real_joint_pub.publish(
                self._snapshot_to_joint_state(real_snapshot, stamp))
            self._publish_sim_real_error(sim_snapshot, real_snapshot, stamp, t_sim)
        else:
            self._real_joint_pub.publish(sim_js)

    def _publish_sim_real_error(self, sim_snapshot, real_snapshot, stamp, t_sim: float) -> None:
        if self._real is None:
            return

        err = SimRealError()
        err.header.stamp = stamp
        err.joint_names = list(sim_snapshot.names)
        err.q_error = [
            float(s) - float(r)
            for s, r in zip(sim_snapshot.positions, real_snapshot.positions)
        ]
        err.dq_error = [
            float(s) - float(r)
            for s, r in zip(sim_snapshot.velocities, real_snapshot.velocities)
        ]
        err.timestamp = self._sec_to_time_msg(t_sim)
        err.real_execution_time_sec = float(self._real.last_execution_sim_time)
        self._error_pub.publish(err)

    @staticmethod
    def _snapshot_to_joint_state(snapshot, stamp) -> JointState:
        js = JointState()
        js.header.stamp = stamp
        js.name = list(snapshot.names)
        js.position = [float(v) for v in snapshot.positions]
        js.velocity = [float(v) for v in snapshot.velocities]
        js.effort = [float(v) for v in snapshot.efforts]
        return js

    def _sample_to_domain_config(self, stamp) -> DomainRandomizationConfig:
        cfg = DomainRandomizationConfig()
        cfg.header.stamp = stamp
        cfg.random_seed = self.get_parameter('random_seed').value
        cfg.randomization_strength = self.get_parameter('randomization_strength').value

        damping_range = list(self.get_parameter('joint_damping_range').value)
        friction_range = list(self.get_parameter('joint_friction_range').value)
        motor_range = list(self.get_parameter('motor_strength_range').value)
        delay_range = list(self.get_parameter('time_delay_range_ms').value)
        payload_range = list(self.get_parameter('payload_mass_range').value)

        cfg.joint_damping_min = damping_range[0]
        cfg.joint_damping_max = damping_range[1]
        cfg.joint_friction_min = friction_range[0]
        cfg.joint_friction_max = friction_range[1]
        cfg.motor_strength_min = motor_range[0]
        cfg.motor_strength_max = motor_range[1]
        cfg.time_delay_min_ms = delay_range[0]
        cfg.time_delay_max_ms = delay_range[1]
        cfg.payload_mass_min = payload_range[0]
        cfg.payload_mass_max = payload_range[1]
        cfg.position_noise_std = 0.0
        cfg.velocity_noise_std = 0.0

        if self._real is not None and self._real.latest_sample is not None:
            sample = self._real.latest_sample
            if sample.joint_damping:
                cfg.joint_damping_min = min(sample.joint_damping)
                cfg.joint_damping_max = max(sample.joint_damping)
            if sample.joint_friction:
                cfg.joint_friction_min = min(sample.joint_friction)
                cfg.joint_friction_max = max(sample.joint_friction)
            if sample.motor_strength:
                cfg.motor_strength_min = min(sample.motor_strength)
                cfg.motor_strength_max = max(sample.motor_strength)
            cfg.time_delay_min_ms = sample.actuator_delay_ms
            cfg.time_delay_max_ms = sample.actuator_delay_ms
            cfg.payload_mass_min = sample.payload_mass_kg
            cfg.payload_mass_max = sample.payload_mass_kg
            cfg.random_seed = sample.episode_seed

        return cfg

    def _publish_metadata(self) -> None:
        stamp = self.get_clock().now().to_msg()
        meta = ExperimentMetadata()
        meta.header.stamp = stamp
        meta.experiment_id = 'm3_dual_source'
        meta.scenario_id = 'SC-01'
        meta.random_seed = self.get_parameter('random_seed').value
        meta.randomization_strength = self.get_parameter('randomization_strength').value
        self._metadata_pub.publish(meta)
        self._randomization_pub.publish(self._sample_to_domain_config(stamp))

    def _handle_set_randomization(self, request, response):
        cfg = request.config
        self.set_parameters([
            rclpy.parameter.Parameter('random_seed', rclpy.Parameter.Type.INTEGER, cfg.random_seed),
            rclpy.parameter.Parameter(
                'randomization_strength', rclpy.Parameter.Type.DOUBLE, cfg.randomization_strength),
        ])

        if self._real is not None:
            dr_cfg = DomainRandomizerConfig(
                random_seed=cfg.random_seed,
                randomization_strength=cfg.randomization_strength,
                joint_damping_base=self.get_parameter('joint_damping_base').value,
                joint_friction_base=self.get_parameter('joint_friction_base').value,
                motor_strength_range=(cfg.motor_strength_min, cfg.motor_strength_max),
                payload_mass_range=(cfg.payload_mass_min, cfg.payload_mass_max),
                time_delay_range_ms=(cfg.time_delay_min_ms, cfg.time_delay_max_ms),
            )
            self._real.update_randomization_config(dr_cfg)
            self._real.reset()

        response.success = True
        response.message = 'Domain randomization updated and episode resampled.'
        return response

    def _handle_inject_shift(self, request, response):
        if self._real is None:
            response.success = False
            response.message = 'Dual-source mode disabled; inject_shift requires Real-Source.'
            return response

        self._real.inject_shift(
            request.parameter_name,
            request.delta_percent,
            request.duration_sec,
            time.monotonic(),
        )
        self._real.resample_episode()
        response.success = True
        response.message = (
            f'Injected {request.parameter_name} +{request.delta_percent}% '
            f'for {request.duration_sec}s')
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
            if self._real is not None and self._real.latest_sample:
                sample = self._real.latest_sample
                self.get_logger().info(
                    f'Episode reset — new Real randomization: '
                    f'damping={sample.joint_damping} friction={sample.joint_friction} '
                    f'payload={sample.payload_mass_kg:.3f}kg delay={sample.actuator_delay_ms:.1f}ms')
        response.success = True
        response.message = 'Simulation reset to home with new domain randomization.'
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
