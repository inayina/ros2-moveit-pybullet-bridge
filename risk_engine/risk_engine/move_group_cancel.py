"""Cancel in-flight MoveGroup (/move_action) goals on e-stop."""

from __future__ import annotations

from typing import TYPE_CHECKING

from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient

if TYPE_CHECKING:
    from rclpy.node import Node


class MoveGroupCancelClient:
    """Fire-and-forget cancel for all active MoveGroup goals."""

    def __init__(self, node: Node, *, action_name: str = '/move_action') -> None:
        self._node = node
        self._client = ActionClient(node, MoveGroup, action_name)
        self._warned_unavailable = False

    def cancel_all(self) -> bool:
        if not self._client.server_is_ready():
            if not self._client.wait_for_server(timeout_sec=0.2):
                if not self._warned_unavailable:
                    self._node.get_logger().debug(
                        f'MoveGroup cancel skipped: {self._client._action_name} unavailable')
                    self._warned_unavailable = True
                return False

        self._client.cancel_all_goals_async()
        self._node.get_logger().warn('Canceled all MoveGroup goals (/move_action)')
        self._warned_unavailable = False
        return True
