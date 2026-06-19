"""Unit tests for Pick executor with mocked planner."""

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

from bridge_monitor_msgs.action import Pick
from manipulation_actions.pick_executor import PickExecutor


class _MockGoalHandle:
    def __init__(self) -> None:
        self.feedback: list = []
        self.terminal = None
        self._cancel_requested = False

    @property
    def is_cancel_requested(self) -> bool:
        return self._cancel_requested

    def publish_feedback(self, feedback) -> None:
        self.feedback.append(feedback)

    def succeed(self) -> None:
        self.terminal = 'succeed'

    def abort(self) -> None:
        self.terminal = 'abort'

    def canceled(self) -> None:
        self.terminal = 'canceled'


class _MockPlanner:
    def __init__(self) -> None:
        self.calls = 0

    def move_to_pose(self, pose, *, planning_group, end_effector_link, timeout_sec):
        del pose, planning_group, end_effector_link, timeout_sec
        self.calls += 1
        return True, None, 'ok'

    def cancel_all(self) -> None:
        pass


def _pick_goal() -> Pick.Goal:
    goal = Pick.Goal()
    goal.grasp_pose = PoseStamped()
    goal.grasp_pose.header.frame_id = 'base'
    goal.grasp_pose.pose = Pose(
        position=Point(x=0.4, y=0.0, z=0.3),
        orientation=Quaternion(w=1.0),
    )
    goal.planning_group = 'manipulator'
    goal.end_effector_link = 'lbr_iiwa_link_7'
    goal.pre_grasp_offset_m = 0.1
    goal.grasp_timeout_sec = 10.0
    return goal


def test_pick_executor_runs_three_phases():
    executor = PickExecutor()
    handle = _MockGoalHandle()
    planner = _MockPlanner()
    result = executor.execute(_pick_goal(), handle, planner)
    assert result.success is True
    assert planner.calls == 3
    assert handle.terminal == 'succeed'
    assert [fb.phase for fb in handle.feedback] == ['approach', 'grasp', 'lift']
