"""ROS 2 Node for multi-source asynchronous sensor fusion and grasp/slip estimation.

**Status: EXPERIMENTAL (not Panda mainline verification).**

What is actually used today
---------------------------
- Synchronized inputs: ``/bridge/sim/joint_states``, ``/camera/color/image_raw``,
  ``/ft_sensor`` via ``message_filters.ApproximateTimeSynchronizer``.
- **JointState**: used for FK / gravity-inertia compensation of the gripper wrench.
- **WrenchStamped**: used as the raw FT measurement for net contact force/torque.
- **Image**: subscribed for time-sync only; **pixel contents are discarded**
  (``del image_msg``). Camera is not used for vision-based grasp/slip.

Verification limits
-------------------
- Unit tests cover gravity compensation math with synthetic messages only.
- Not part of handoff replay harness go/no-go; not used by SmolVLA S4 GT.
- Must not be cited as evidence of grasp success, Sim2Real, or task Pass.
- Risk / ContinuousTaskEvaluator remain authoritative for readiness / task GT.
"""

from __future__ import annotations

from typing import Optional

from bridge_monitor_msgs.msg import GraspStatus
from geometry_msgs.msg import WrenchStamped
import message_filters
import numpy as np
import pybullet as p
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

from pybullet_bridge.robot_profiles import resolve_urdf_path

# Portfolio / AGENTS marker — keep in sync with docs/AGENTS.md §4.
SENSOR_FUSION_STATUS = "experimental"
SENSOR_FUSION_CAMERA_UTILIZATION = "timestamp_sync_only"


class SensorFusionNode(Node):
    """EXPERIMENTAL: asynchronously sync JointState, Camera Image, and FT Wrench.

    Compensates gravity and inertia of the gripper mass to estimate net contact wrench
    and detect contact establishment and slip. Camera frames are sync-only.
    """

    def __init__(self) -> None:
        super().__init__('sensor_fusion_node')
        self._declare_parameters()
        self.get_logger().warn(
            '[SensorFusionNode] EXPERIMENTAL: camera pixels unused; '
            'not a task-success or Sim2Real evidence source.'
        )

        self._gripper_mass = float(self.get_parameter('gripper_mass').value)
        self._gripper_com_z = float(self.get_parameter('gripper_com_z').value)
        self._contact_force_threshold = float(self.get_parameter('contact_force_threshold').value)
        self._slip_variance_threshold = float(self.get_parameter('slip_variance_threshold').value)
        self._window_size = int(self.get_parameter('sliding_window_size').value)
        self._sync_slop = float(self.get_parameter('sync_slop').value)

        # Pybullet client for forward kinematics
        self._client_id = p.connect(p.DIRECT)
        urdf_path = resolve_urdf_path('panda')
        self._robot_id = p.loadURDF(urdf_path, useFixedBase=True, physicsClientId=self._client_id)
        # panda_link7 is index 6, hand/sensor link index is 7 or 8 (usually panda_hand is 7)
        self._sensor_link_idx = 7

        # State tracking for derivatives
        self._prev_time: Optional[float] = None
        self._prev_world_pos: Optional[np.ndarray] = None
        self._prev_world_vel: Optional[np.ndarray] = None
        self._force_history: list[float] = []

        # Publisher
        self._status_pub = self.create_publisher(
            GraspStatus,
            '/bridge/sim/grasp_status',
            10,
        )

        # Subscribers with message filters
        # All three producers are sensor streams. BestEffort/volatile KeepLast
        # avoids a Reliable subscriber silently becoming incompatible with the
        # upstream SensorDataQoS publishers under load.
        self._joint_sub = message_filters.Subscriber(
            self, JointState, '/bridge/sim/joint_states', qos_profile_sensor_data)
        self._camera_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw', qos_profile_sensor_data)
        self._ft_sub = message_filters.Subscriber(
            self, WrenchStamped, '/ft_sensor', qos_profile_sensor_data)

        self._ts = message_filters.ApproximateTimeSynchronizer(
            [self._joint_sub, self._camera_sub, self._ft_sub],
            queue_size=10,
            slop=self._sync_slop,
        )
        self._ts.registerCallback(self._on_synced_data)
        self.get_logger().info('[SensorFusionNode] initialized and listening.')

    def _declare_parameters(self) -> None:
        self.declare_parameter('gripper_mass', 0.73)  # kg
        self.declare_parameter('gripper_com_z', 0.05)  # m (CoM z offset from FT sensor)
        self.declare_parameter('contact_force_threshold', 2.0)  # N
        self.declare_parameter('slip_variance_threshold', 0.1)  # N^2
        self.declare_parameter('sliding_window_size', 5)
        self.declare_parameter('sync_slop', 0.1)  # seconds

    def __del__(self) -> None:
        if hasattr(self, '_client_id') and self._client_id is not None:
            try:
                p.disconnect(self._client_id)
            except Exception:
                pass

    def _on_synced_data(
        self,
        joint_msg: JointState,
        image_msg: Image,
        ft_msg: WrenchStamped,
    ) -> None:
        del image_msg  # Sync confirmation only

        # 1. Kinematics via PyBullet
        joint_positions = joint_msg.position
        joint_names = joint_msg.name

        for i, name in enumerate(joint_names):
            # Check standard joint names or indices
            if 'panda_joint' in name:
                idx = int(name[-1]) - 1
                if 0 <= idx < 7:
                    p.resetJointState(
                        self._robot_id,
                        idx,
                        joint_positions[i],
                        physicsClientId=self._client_id,
                    )

        # Forward kinematics
        state = p.getLinkState(
            self._robot_id,
            self._sensor_link_idx,
            computeLinkVelocity=1,
            physicsClientId=self._client_id,
        )
        world_pos = np.array(state[0], dtype=np.float64)
        world_orn = np.array(state[1], dtype=np.float64)
        world_vel = np.array(state[6], dtype=np.float64)  # linear velocity

        # Compute acceleration feedforward via finite difference
        now = float(joint_msg.header.stamp.sec) + float(joint_msg.header.stamp.nanosec) * 1e-9
        world_accel = np.zeros(3)
        if self._prev_time is not None and now > self._prev_time:
            dt = now - self._prev_time
            world_accel = (world_vel - self._prev_world_vel) / dt

        self._prev_time = now
        self._prev_world_pos = world_pos
        self._prev_world_vel = world_vel

        # Rotation matrix from Link/Sensor frame to World frame
        r_mat = np.array(p.getMatrixFromQuaternion(world_orn), dtype=np.float64).reshape(3, 3)

        # 2. Gravity and Inertia Compensation
        # Gravity force in world: [0, 0, -m*g]
        g_world = np.array([0.0, 0.0, -9.81], dtype=np.float64)
        f_grav_world = self._gripper_mass * g_world
        f_grav_sensor = r_mat.T @ f_grav_world

        # Inertia force in world: m * a
        f_inertia_world = self._gripper_mass * world_accel
        f_inertia_sensor = r_mat.T @ f_inertia_world

        # Estimated torque from gravity
        # CoM offset in sensor frame: [0, 0, gripper_com_z]
        com_sensor = np.array([0.0, 0.0, self._gripper_com_z], dtype=np.float64)
        t_grav_sensor = np.cross(com_sensor, f_grav_sensor)

        # Raw wrench from FT sensor
        raw_force = np.array([ft_msg.wrench.force.x, ft_msg.wrench.force.y, ft_msg.wrench.force.z])
        raw_torque = np.array([
            ft_msg.wrench.torque.x,
            ft_msg.wrench.torque.y,
            ft_msg.wrench.torque.z,
        ])

        # Compensated contact wrench
        net_force = raw_force - f_grav_sensor - f_inertia_sensor
        net_torque = raw_torque - t_grav_sensor

        # 3. Grasp Contact and Slip Estimation
        force_norm = float(np.linalg.norm(net_force))
        self._force_history.append(force_norm)
        if len(self._force_history) > self._window_size:
            self._force_history.pop(0)

        # Contact is established if net force exceeds threshold
        grasp_established = force_norm > self._contact_force_threshold

        # Slip detection based on contact force variance during hold
        object_slipped = False
        if grasp_established and len(self._force_history) >= self._window_size:
            force_var = float(np.var(self._force_history))
            if force_var > self._slip_variance_threshold:
                object_slipped = True

        # 4. Publish
        status_msg = GraspStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.header.frame_id = 'panda_hand'
        status_msg.grasp_established = grasp_established
        status_msg.object_slipped = object_slipped
        status_msg.net_wrench.force.x = net_force[0]
        status_msg.net_wrench.force.y = net_force[1]
        status_msg.net_wrench.force.z = net_force[2]
        status_msg.net_wrench.torque.x = net_torque[0]
        status_msg.net_wrench.torque.y = net_torque[1]
        status_msg.net_wrench.torque.z = net_torque[2]
        status_msg.confidence = 0.95 if grasp_established else 1.0

        self._status_pub.publish(status_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
