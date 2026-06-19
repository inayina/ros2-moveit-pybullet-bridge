# ROS 2 Jazzy image for pybullet_bridge + dist_monitor integration demos.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    ROS2_WS_ROOT=/ws \
    EPISODE_DATA_LAB_ROOT=/data/episode-data-lab \
    LEROBOT_EXPORT=/data/episode-data-lab/dataset/v1/lerobot_export

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-moveit-ros-move-group \
    ros-jazzy-moveit-planners-ompl \
    ros-jazzy-moveit-kinematics \
    ros-jazzy-moveit-configs-utils \
    ros-jazzy-rviz2 \
    ros-jazzy-ur-description \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --break-system-packages --no-cache-dir -r /tmp/requirements.txt

COPY . /ws/src/ros2-moveit-pybullet-bridge

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build \
      --packages-select bridge_monitor_msgs pybullet_bridge dist_monitor risk_engine hoc_console manipulation_actions moveit_config \
      --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
