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
from std_srvs.srv import Trigger

from bridge_monitor_msgs.action import ExecuteScenario
from bridge_monitor_msgs.msg import DistributionMetrics, DomainRandomizationConfig, RiskStatus
from bridge_monitor_msgs.srv import SetRandomization

from hoc_console.ros_bridge import distribution_metrics_to_dict, risk_status_to_dict
from hoc_console.scenario_motion import (
    publish_iiwa_collision_avoid,
    publish_iiwa_estop_recovery,
    publish_iiwa_point_to_point,
    publish_iiwa_randomization_scan,
    publish_iiwa_trajectory_track,
)


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
        self._e_stop_client = node.create_client(Trigger, '/risk/force_e_stop')
        self._clear_e_stop_client = node.create_client(Trigger, '/risk/clear_e_stop')
        self._set_randomization_client = node.create_client(
            SetRandomization, '/bridge/set_randomization')
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

    def start_scenario(self, goal: ExecuteScenario.Goal) -> tuple[bool, str]:
        """Start a scenario without blocking the HOC node (WebSocket path)."""
        with self._lock:
            if self._active:
                return False, 'Scenario already running'
        thread = threading.Thread(
            target=self._run_scenario,
            args=(goal, None),
            daemon=True,
        )
        thread.start()
        return True, f'Scenario {goal.scenario_id} started'

    def _execute(self, goal_handle):
        return self._run_scenario(goal_handle.request, goal_handle)

    def _call_trigger(self, client, timeout_sec: float = 2.0) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=1.0):
            return False, 'trigger service unavailable'
        future = client.call_async(Trigger.Request())
        done = threading.Event()
        holder: dict[str, Any] = {'resp': None, 'error': None}

        def _on_done(fut) -> None:
            try:
                holder['resp'] = fut.result()
            except Exception as exc:  # noqa: BLE001
                holder['error'] = str(exc)
            done.set()

        future.add_done_callback(_on_done)
        if not done.wait(timeout_sec):
            return False, 'service call timed out'
        if holder['error']:
            return False, holder['error']
        resp = holder['resp']
        if resp is None:
            return False, 'empty service response'
        return bool(resp.success), resp.message

    def _apply_randomization(self, seed: int, strength: float) -> tuple[bool, str]:
        if not self._set_randomization_client.wait_for_service(timeout_sec=1.0):
            return False, '/bridge/set_randomization unavailable'
        req = SetRandomization.Request()
        cfg = DomainRandomizationConfig()
        cfg.random_seed = seed
        cfg.randomization_strength = strength
        cfg.joint_damping_min = 0.0
        cfg.joint_damping_max = 0.5
        cfg.joint_friction_min = 0.0
        cfg.joint_friction_max = 0.3
        cfg.motor_strength_min = 0.85
        cfg.motor_strength_max = 1.15
        cfg.position_noise_std = 0.01
        cfg.velocity_noise_std = 0.05
        cfg.time_delay_min_ms = 0.0
        cfg.time_delay_max_ms = 50.0
        cfg.payload_mass_min = 0.0
        cfg.payload_mass_max = 0.5
        req.config = cfg
        future = self._set_randomization_client.call_async(req)
        done = threading.Event()
        holder: dict[str, Any] = {'resp': None, 'error': None}

        def _on_done(fut) -> None:
            try:
                holder['resp'] = fut.result()
            except Exception as exc:  # noqa: BLE001
                holder['error'] = str(exc)
            done.set()

        future.add_done_callback(_on_done)
        if not done.wait(3.0):
            return False, 'set_randomization timed out'
        if holder['error']:
            return False, holder['error']
        resp = holder['resp']
        if resp is None:
            return False, 'empty set_randomization response'
        return bool(resp.success), resp.message

    def _run_scenario_motion(self, req: ExecuteScenario.Goal) -> None:
        scenario_id = req.scenario_id
        seed = req.random_seed
        strength = req.randomization_strength

        if scenario_id == 'SC-02':
            publish_iiwa_trajectory_track(
                self._node, seed=seed, strength=strength)
        elif scenario_id == 'SC-03':
            publish_iiwa_collision_avoid(
                self._node, seed=seed, strength=strength)
        elif scenario_id == 'SC-04':
            for idx, scan_strength in enumerate((0.25, 0.5, 0.75)):
                ok, msg = self._apply_randomization(seed + idx, scan_strength)
                if not ok:
                    self._node.get_logger().warn(
                        f'SC-04 randomization step {idx} failed: {msg}')
                publish_iiwa_randomization_scan(
                    self._node, seed=seed + idx, strength=scan_strength)
                time.sleep(0.3)
        elif scenario_id == 'SC-05':
            publish_iiwa_estop_recovery(
                self._node, seed=seed, strength=strength)
            time.sleep(1.5)
            ok, msg = self._call_trigger(self._e_stop_client)
            self._node.get_logger().info(f'SC-05 force e-stop: ok={ok} msg={msg}')
            time.sleep(1.0)
            ok, msg = self._call_trigger(self._clear_e_stop_client)
            self._node.get_logger().info(f'SC-05 clear e-stop: ok={ok} msg={msg}')
            time.sleep(0.5)
            publish_iiwa_estop_recovery(
                self._node, seed=seed + 1, strength=strength * 0.6)
        else:
            publish_iiwa_point_to_point(
                self._node, seed=seed, strength=strength)

    def _run_scenario(self, req: ExecuteScenario.Goal, goal_handle) -> ExecuteScenario.Result | None:
        with self._lock:
            self._active = True
        experiment_id = f'exp_{time.strftime("%Y%m%d_%H%M%S")}_{uuid.uuid4().hex[:6]}'
        self._node.get_logger().info(
            f'Starting scenario {req.scenario_id} seed={req.random_seed} '
            f'strength={req.randomization_strength:.2f} id={experiment_id}')

        feedback = ExecuteScenario.Feedback()
        result = ExecuteScenario.Result()
        phase_count = len(self.PHASES)
        start = time.monotonic()

        try:
            motion_sent = False
            for idx, phase in enumerate(self.PHASES):
                if phase == 'execute' and not motion_sent:
                    self._run_scenario_motion(req)
                    motion_sent = True

                if goal_handle is not None and goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.experiment_id = experiment_id
                    result.message = f'Scenario canceled during {phase}.'
                    return result

                phase_start = time.monotonic()
                while time.monotonic() - phase_start < 1.0:
                    if goal_handle is not None and goal_handle.is_cancel_requested:
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
                    if goal_handle is not None:
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
            if goal_handle is not None:
                goal_handle.succeed()
            if self._on_feedback:
                self._on_feedback({
                    'experiment_id': experiment_id,
                    'scenario_id': req.scenario_id,
                    'progress': 1.0,
                    'current_phase': 'done',
                    'completed': True,
                })
            return result
        finally:
            with self._lock:
                self._active = False
