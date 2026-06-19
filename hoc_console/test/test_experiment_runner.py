"""Tests for ExperimentRunner scenario control."""

from __future__ import annotations

import threading
import time

import pytest
import rclpy
from bridge_monitor_msgs.action import ExecuteScenario
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from hoc_console.experiment_runner import ExperimentRunner


class _DummyNode(Node):
    def __init__(self) -> None:
        super().__init__('test_experiment_runner_node')


@pytest.fixture(scope='module')
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def runner(ros_context):
    node = _DummyNode()
    feedback: list[dict] = []
    runner = ExperimentRunner(
        node,
        get_latest_metrics=lambda: None,
        get_latest_risk=lambda: None,
        on_feedback=feedback.append,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield runner, feedback
    executor.shutdown()
    thread.join(timeout=2.0)
    node.destroy_node()


def test_start_scenario_runs_without_deadlock(runner):
    exp_runner, feedback = runner
    goal = ExecuteScenario.Goal()
    goal.scenario_id = 'SC-01'
    goal.random_seed = 7
    goal.randomization_strength = 0.3

    ok, message = exp_runner.start_scenario(goal)
    assert ok is True
    assert 'SC-01' in message

    deadline = time.time() + 8.0
    while time.time() < deadline and not feedback:
        time.sleep(0.1)

    assert feedback, 'expected experiment_progress feedback'
    assert feedback[-1]['scenario_id'] == 'SC-01'
    assert feedback[-1]['progress'] > 0.0

    deadline = time.time() + 8.0
    while time.time() < deadline and exp_runner.active:
        time.sleep(0.1)

    assert not exp_runner.active


def test_start_scenario_rejects_when_busy(runner):
    exp_runner, _feedback = runner
    goal = ExecuteScenario.Goal()
    goal.scenario_id = 'SC-02'
    ok1, _ = exp_runner.start_scenario(goal)
    assert ok1 is True
    ok2, message = exp_runner.start_scenario(goal)
    assert ok2 is False
    assert 'already running' in message.lower()
