"""Unit tests for Place executor with mocked planner."""

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

from bridge_monitor_msgs.action import Place
from manipulation_actions.place_executor import PlaceExecutor


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


def _place_goal() -> Place.Goal:
    goal = Place.Goal()
    goal.place_pose = PoseStamped()
    goal.place_pose.header.frame_id = 'base'
    goal.place_pose.pose = Pose(
        position=Point(x=0.35, y=0.1, z=0.25),
        orientation=Quaternion(w=1.0),
    )
    goal.planning_group = 'manipulator'
    goal.retreat_offset_m = 0.08
    return goal


def test_place_executor_runs_three_phases():
    executor = PlaceExecutor()
    handle = _MockGoalHandle()
    planner = _MockPlanner()
    result = executor.execute(_place_goal(), handle, planner)
    assert result.success is True
    assert planner.calls == 3
    assert handle.terminal == 'succeed'
    assert [fb.phase for fb in handle.feedback] == ['approach', 'release', 'retreat']
