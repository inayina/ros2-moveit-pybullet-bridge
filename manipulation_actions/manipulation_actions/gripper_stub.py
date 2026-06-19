"""Simple gripper command stub for pick/place phases."""

from __future__ import annotations

from typing import TYPE_CHECKING

from std_msgs.msg import Float64

if TYPE_CHECKING:
    from rclpy.node import Node


class GripperStub:
    """Publish open/close commands on /gripper/command (Float64: 0=open, 1=close)."""

    def __init__(self, node: Node, *, topic: str = '/gripper/command') -> None:
        self._node = node
        self._pub = node.create_publisher(Float64, topic, 10)
        self._closed = False

    @property
    def is_closed(self) -> bool:
        return self._closed

    def open(self) -> None:
        self._pub.publish(Float64(data=0.0))
        self._closed = False
        self._node.get_logger().info('Gripper: open')

    def close(self) -> None:
        self._pub.publish(Float64(data=1.0))
        self._closed = True
        self._node.get_logger().info('Gripper: close')
