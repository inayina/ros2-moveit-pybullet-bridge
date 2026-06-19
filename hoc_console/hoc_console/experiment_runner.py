"""ExecuteScenario action server for HOC experiment control."""

from __future__ import annotations

import threading
import time
import uuid
from typing import Any, Callable

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from bridge_monitor_msgs.action import ExecuteScenario
from bridge_monitor_msgs.msg import DistributionMetrics, RiskStatus

from hoc_console.ros_bridge import distribution_metrics_to_dict, risk_status_to_dict


class ExperimentRunner:
    """Run preset validation scenarios and stream progress feedback."""

    SCENARIOS = {
        'SC-01': '点到点运动',
        'SC-02': '轨迹跟踪',
        'SC-03': '碰撞规避',
        'SC-04': '域随机化扫描',
        'SC-05': '急停恢复',
    }

    PHASES = ('init', 'warmup', 'execute', 'cooldown', 'done')

    def __init__(
        self,
        node: Node,
        *,
        get_latest_metrics: Callable[[], DistributionMetrics | None],
        get_latest_risk: Callable[[], RiskStatus | None],
        on_feedback: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self._node = node
        self._get_latest_metrics = get_latest_metrics
        self._get_latest_risk = get_latest_risk
        self._on_feedback = on_feedback
        self._lock = threading.Lock()
        self._active = False
        self._action_server = ActionServer(
            node,
            ExecuteScenario,
            '/hoc/execute_scenario',
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    def _goal_callback(self, goal_request) -> GoalResponse:
        if goal_request.scenario_id not in self.SCENARIOS:
            self._node.get_logger().warn(
                f'Unknown scenario {goal_request.scenario_id!r}, accepting anyway.')
        with self._lock:
            if self._active:
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        with self._lock:
            self._active = True
        req = goal_handle.request
        experiment_id = f'exp_{time.strftime("%Y%m%d_%H%M%S")}_{uuid.uuid4().hex[:6]}'
        self._node.get_logger().info(
            f'Starting scenario {req.scenario_id} seed={req.random_seed} '
            f'strength={req.randomization_strength:.2f} id={experiment_id}')

        feedback = ExecuteScenario.Feedback()
        result = ExecuteScenario.Result()
        phase_count = len(self.PHASES)
        start = time.monotonic()

        try:
            for idx, phase in enumerate(self.PHASES):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.experiment_id = experiment_id
                    result.message = f'Scenario canceled during {phase}.'
                    return result

                phase_start = time.monotonic()
                while time.monotonic() - phase_start < 1.0:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.success = False
                        result.experiment_id = experiment_id
                        result.message = f'Scenario canceled during {phase}.'
                        return result
                    progress = (idx + (time.monotonic() - phase_start)) / phase_count
                    feedback.progress = min(progress, 1.0)
                    feedback.current_phase = phase
                    risk = self._get_latest_risk()
                    metrics = self._get_latest_metrics()
                    if risk:
                        feedback.current_risk = risk
                    if metrics:
                        feedback.current_metrics = metrics
                    goal_handle.publish_feedback(feedback)
                    if self._on_feedback:
                        payload: dict[str, Any] = {
                            'experiment_id': experiment_id,
                            'scenario_id': req.scenario_id,
                            'progress': feedback.progress,
                            'current_phase': phase,
                        }
                        if risk:
                            payload['current_risk'] = risk_status_to_dict(risk)
                        if metrics:
                            payload['current_metrics'] = distribution_metrics_to_dict(metrics)
                        self._on_feedback(payload)
                    time.sleep(0.2)

            result.success = True
            result.experiment_id = experiment_id
            result.message = (
                f'{req.scenario_id} completed in {time.monotonic() - start:.1f}s')
            metrics = self._get_latest_metrics()
            risk = self._get_latest_risk()
            if metrics:
                result.final_metrics = metrics
            if risk:
                result.final_risk = risk
            goal_handle.succeed()
            return result
        finally:
            with self._lock:
                self._active = False
