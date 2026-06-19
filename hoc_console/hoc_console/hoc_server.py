"""HOC server: ROS 2 bridge + WebSocket dashboard backend."""

from __future__ import annotations

import asyncio
import json
import os
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from bridge_monitor_msgs.action import ExecuteScenario
from bridge_monitor_msgs.msg import DistributionMetrics, DomainRandomizationConfig, RiskStatus
from bridge_monitor_msgs.srv import AcknowledgeRisk, ExportExperiment, InjectShift, SetRandomization
from sensor_msgs.msg import JointState

from hoc_console.experiment_runner import ExperimentRunner
from hoc_console.http_static import resolve_frontend_dist, start_static_server
from hoc_console.report_html import render_html_report
from hoc_console.ros_bridge import (
    distribution_metrics_to_dict,
    risk_status_to_dict,
    tracking_error_to_dict,
)
from hoc_console.ws_hub import WsHub


@dataclass
class HistorySample:
    t: float
    risk: dict[str, Any] | None = None
    metrics: dict[str, Any] | None = None


@dataclass
class SessionHistory:
    start_time: float = field(default_factory=time.time)
    risk_timeline: deque = field(default_factory=lambda: deque(maxlen=3600))
    metrics_timeline: deque = field(default_factory=lambda: deque(maxlen=3600))
    alerts: list[dict[str, Any]] = field(default_factory=list)
    max_risk_level: int = 0
    max_composite_score: float = 0.0
    shift_detected_count: int = 0
    shift_total: int = 0
    kl_sum: float = 0.0
    kl_max: float = 0.0
    mmd_sum: float = 0.0
    mmd_max: float = 0.0
    metrics_count: int = 0

    def record_risk(self, payload: dict[str, Any]) -> None:
        t = time.time() - self.start_time
        self.risk_timeline.append({'t': t, 'level': payload['level'], 'score': payload['composite_score']})
        self.max_risk_level = max(self.max_risk_level, payload['level'])
        self.max_composite_score = max(self.max_composite_score, payload['composite_score'])

    def record_metrics(self, payload: dict[str, Any]) -> None:
        t = time.time() - self.start_time
        kl = payload.get('kl_divergence_mean', 0.0)
        mmd = payload.get('mmd_statistic', 0.0)
        self.metrics_timeline.append({
            't': t,
            'kl_mean': kl,
            'mmd_stat': mmd,
            'shift_detected': payload.get('shift_detected', False),
        })
        self.metrics_count += 1
        self.kl_sum += kl
        self.kl_max = max(self.kl_max, kl)
        self.mmd_sum += mmd
        self.mmd_max = max(self.mmd_max, mmd)
        self.shift_total += 1
        if payload.get('shift_detected'):
            self.shift_detected_count += 1

    def summary(self) -> dict[str, Any]:
        mean_kl = self.kl_sum / self.metrics_count if self.metrics_count else 0.0
        mean_mmd = self.mmd_sum / self.metrics_count if self.metrics_count else 0.0
        ratio = self.shift_detected_count / self.shift_total if self.shift_total else 0.0
        return {
            'max_risk_level': self.max_risk_level,
            'max_composite_score': self.max_composite_score,
            'shift_detected_count': self.shift_detected_count,
            'shift_detected_ratio': ratio,
            'mean_kl': mean_kl,
            'max_kl': self.kl_max,
            'mean_mmd': mean_mmd,
            'max_mmd': self.mmd_max,
        }


class HocServerNode(Node):
    """Bridge ROS 2 risk/monitor topics to WebSocket dashboard clients."""

    def __init__(self) -> None:
        super().__init__('hoc_server')

        self.declare_parameter('websocket_port', 8765)
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('push_frequency_hz', 5.0)
        self.declare_parameter('rosbag_output_dir', '~/ros2_ws/bags')
        self.declare_parameter('report_output_dir', '~/ros2_ws/reports')
        self.declare_parameter('serve_frontend', True)
        self.declare_parameter('frontend_dist_dir', '')

        self._ws_hub = WsHub()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._system_state = 'RUNNING'
        self._latest_risk: RiskStatus | None = None
        self._latest_metrics: DistributionMetrics | None = None
        self._latest_tracking = None
        self._history = SessionHistory()
        self._recording = False
        self._recording_proc: subprocess.Popen | None = None
        self._bag_path = ''
        self._experiment_metadata: dict[str, Any] = {}

        self.create_subscription(RiskStatus, '/risk/status', self._on_risk, 10)
        self.create_subscription(
            DistributionMetrics, '/monitor/distribution_metrics', self._on_metrics, 10)
        self.create_subscription(JointState, '/monitor/tracking_error', self._on_tracking, 10)
        self.create_subscription(String, '/risk/alerts', self._on_alert, 10)

        self._e_stop_client = self.create_client(Trigger, '/risk/force_e_stop')
        self._ack_client = self.create_client(AcknowledgeRisk, '/risk/acknowledge')
        self._clear_e_stop_client = self.create_client(Trigger, '/risk/clear_e_stop')
        self._set_randomization_client = self.create_client(
            SetRandomization, '/bridge/set_randomization')
        self._inject_shift_client = self.create_client(InjectShift, '/bridge/inject_shift')
        self._set_mode_client = self.create_client(SetBool, '/bridge/set_mode')
        self._scenario_client = ActionClient(self, ExecuteScenario, '/hoc/execute_scenario')

        self.create_service(ExportExperiment, '/hoc/export_experiment', self._handle_export)
        self.create_service(Trigger, '/hoc/start_recording', self._handle_start_recording)
        self.create_service(Trigger, '/hoc/stop_recording', self._handle_stop_recording)

        self._runner = ExperimentRunner(
            self,
            get_latest_metrics=lambda: self._latest_metrics,
            get_latest_risk=lambda: self._latest_risk,
            on_feedback=self._on_experiment_feedback,
        )

        hz = self.get_parameter('push_frequency_hz').value
        self._push_timer = self.create_timer(1.0 / hz, self._push_latest)
        self._state_timer = self.create_timer(1.0, self._push_system_state)

        self._ws_port = self.get_parameter('websocket_port').value
        self._http_port = self.get_parameter('http_port').value
        self._serve_frontend = self.get_parameter('serve_frontend').value
        self._frontend_dist = resolve_frontend_dist(
            self.get_parameter('frontend_dist_dir').value)
        self._ws_thread = threading.Thread(target=self._run_ws_server, daemon=True)
        self._ws_thread.start()

        self.get_logger().info(
            f'hoc_server started. WebSocket ws://0.0.0.0:{self._ws_port}'
            + (f', UI http://0.0.0.0:{self._http_port}' if self._serve_frontend else ''))

    def _on_experiment_feedback(self, payload: dict[str, Any]) -> None:
        loop = self._loop
        if loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._ws_hub.broadcast('experiment_progress', payload), loop)

    def _push_system_state(self) -> None:
        loop = self._loop
        if loop is None or not loop.is_running():
            return
        asyncio.run_coroutine_threadsafe(
            self._ws_hub.broadcast('system_state', {'state': self._system_state}),
            loop,
        )

    def _on_risk(self, msg: RiskStatus) -> None:
        self._latest_risk = msg
        self._history.record_risk(risk_status_to_dict(msg))

    def _on_metrics(self, msg: DistributionMetrics) -> None:
        self._latest_metrics = msg
        self._history.record_metrics(distribution_metrics_to_dict(msg))

    def _on_tracking(self, msg: JointState) -> None:
        self._latest_tracking = msg

    def _on_alert(self, msg: String) -> None:
        try:
            alert = json.loads(msg.data)
        except json.JSONDecodeError:
            alert = {'message': msg.data}
        alert['timestamp'] = time.time()
        self._history.alerts.append(alert)
        loop = self._loop
        if loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._ws_hub.broadcast('alert_event', alert), loop)

    def _push_latest(self) -> None:
        loop = self._loop
        if loop is None or not loop.is_running():
            return
        if self._latest_risk:
            asyncio.run_coroutine_threadsafe(
                self._ws_hub.broadcast(
                    'risk_status', risk_status_to_dict(self._latest_risk)),
                loop,
            )
        if self._latest_metrics:
            asyncio.run_coroutine_threadsafe(
                self._ws_hub.broadcast(
                    'distribution_metrics',
                    distribution_metrics_to_dict(self._latest_metrics)),
                loop,
            )
        if self._latest_tracking:
            asyncio.run_coroutine_threadsafe(
                self._ws_hub.broadcast(
                    'tracking_error',
                    tracking_error_to_dict(self._latest_tracking)),
                loop,
            )

    async def _handle_ws_message(self, websocket, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            await self._ws_hub.send_to(websocket, {
                'type': 'error',
                'message': 'Invalid JSON',
            })
            return

        msg_type = msg.get('type')
        if msg_type == 'ping':
            await self._ws_hub.send_to(websocket, {'type': 'pong'})
            return

        if msg_type == 'subscribe':
            topics = msg.get('topics', [])
            self._ws_hub.subscribe(websocket, topics)
            await self._ws_hub.send_to(websocket, {
                'type': 'subscribed',
                'topics': topics,
            })
            return

        if msg_type == 'command':
            result = await asyncio.get_event_loop().run_in_executor(
                None, self._handle_command, msg.get('action', ''), msg.get('params', {}))
            await self._ws_hub.send_to(websocket, {
                'type': 'command_result',
                'action': msg.get('action'),
                **result,
            })
            if msg.get('action') == 'export_report' and result.get('success'):
                await self._ws_hub.send_to(websocket, {
                    'type': 'report_ready',
                    'file_path': result.get('file_path', ''),
                    'format': result.get('format', 'html'),
                })
            return

        await self._ws_hub.send_to(websocket, {
            'type': 'error',
            'message': f'Unknown message type: {msg_type}',
        })

    def _handle_command(self, action: str, params: dict[str, Any]) -> dict[str, Any]:
        handlers = {
            'e_stop': self._cmd_e_stop,
            'acknowledge': self._cmd_acknowledge,
            'resume': self._cmd_resume,
            'start_recording': self._cmd_start_recording,
            'stop_recording': self._cmd_stop_recording,
            'export_report': self._cmd_export_report,
            'start_experiment': self._cmd_start_experiment,
            'set_randomization': self._cmd_set_randomization,
            'inject_shift': self._cmd_inject_shift,
            'pause': self._cmd_pause,
        }
        handler = handlers.get(action)
        if handler is None:
            return {'success': False, 'message': f'Unknown action: {action}'}
        return handler(params)

    def _cmd_e_stop(self, _params: dict[str, Any]) -> dict[str, Any]:
        if not self._e_stop_client.wait_for_service(timeout_sec=1.0):
            return {'success': False, 'message': '/risk/force_e_stop unavailable'}
        future = self._e_stop_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            return {'success': False, 'message': 'E-stop call timed out'}
        resp = future.result()
        return {'success': resp.success, 'message': resp.message}

    def _cmd_acknowledge(self, params: dict[str, Any]) -> dict[str, Any]:
        if not self._ack_client.wait_for_service(timeout_sec=1.0):
            return {'success': False, 'message': '/risk/acknowledge unavailable'}
        req = AcknowledgeRisk.Request()
        req.operator_id = str(params.get('operator_id', 'operator'))
        req.comment = str(params.get('comment', ''))
        req.from_level = int(params.get('from_level', 3))
        req.to_level = int(params.get('to_level', 2))
        future = self._ack_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            return {'success': False, 'message': 'Acknowledge call timed out'}
        resp = future.result()
        return {'success': resp.success, 'message': resp.message}

    def _cmd_pause(self, _params: dict[str, Any]) -> dict[str, Any]:
        if not self._set_mode_client.wait_for_service(timeout_sec=1.0):
            return {'success': False, 'message': '/bridge/set_mode unavailable'}
        req = SetBool.Request()
        req.data = False
        future = self._set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            return {'success': False, 'message': 'pause timed out'}
        resp = future.result()
        if resp.success:
            self._system_state = 'PAUSED'
        return {'success': resp.success, 'message': resp.message, 'state': self._system_state}

    def _cmd_resume(self, _params: dict[str, Any]) -> dict[str, Any]:
        if self._set_mode_client.wait_for_service(timeout_sec=1.0):
            mode_req = SetBool.Request()
            mode_req.data = True
            mode_future = self._set_mode_client.call_async(mode_req)
            rclpy.spin_until_future_complete(self, mode_future, timeout_sec=2.0)
        if not self._clear_e_stop_client.wait_for_service(timeout_sec=1.0):
            return {'success': False, 'message': '/risk/clear_e_stop unavailable'}
        future = self._clear_e_stop_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            return {'success': False, 'message': 'Clear e-stop timed out'}
        resp = future.result()
        if resp.success:
            self._system_state = 'RUNNING'
        return {'success': resp.success, 'message': resp.message, 'state': self._system_state}

    def _cmd_start_recording(self, _params: dict[str, Any]) -> dict[str, Any]:
        resp = Trigger.Response()
        self._handle_start_recording(Trigger.Request(), resp)
        return {
            'success': resp.success,
            'message': resp.message,
            'recording': self._recording,
            'bag_path': self._bag_path,
        }

    def _cmd_stop_recording(self, _params: dict[str, Any]) -> dict[str, Any]:
        resp = Trigger.Response()
        self._handle_stop_recording(Trigger.Request(), resp)
        return {
            'success': resp.success,
            'message': resp.message,
            'recording': self._recording,
            'bag_path': self._bag_path,
        }

    def _cmd_export_report(self, params: dict[str, Any]) -> dict[str, Any]:
        req = ExportExperiment.Request()
        req.experiment_id = str(params.get('experiment_id', f'exp_{int(time.time())}'))
        req.format = str(params.get('format', 'html'))
        req.output_path = str(params.get('path', ''))
        screenshot = params.get('screenshot_b64')
        resp = ExportExperiment.Response()
        self._generate_export(req, resp, screenshot_b64=screenshot)
        return {
            'success': resp.success,
            'message': resp.message,
            'file_path': resp.file_path,
            'format': req.format,
        }

    def _cmd_start_experiment(self, params: dict[str, Any]) -> dict[str, Any]:
        if not self._scenario_client.wait_for_server(timeout_sec=2.0):
            return {'success': False, 'message': '/hoc/execute_scenario action unavailable'}
        goal = ExecuteScenario.Goal()
        goal.scenario_id = str(params.get('scenario_id', 'SC-01'))
        goal.random_seed = int(params.get('seed', 42))
        goal.randomization_strength = float(params.get('strength', 0.5))
        goal.record_bag = bool(params.get('record', False))
        self._experiment_metadata = {
            'scenario_id': goal.scenario_id,
            'random_seed': goal.random_seed,
            'randomization_strength': goal.randomization_strength,
        }
        future = self._scenario_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            return {'success': False, 'message': 'Failed to send scenario goal'}
        goal_handle = future.result()
        if not goal_handle.accepted:
            return {'success': False, 'message': 'Scenario goal rejected'}
        return {
            'success': True,
            'message': f'Scenario {goal.scenario_id} started',
            'scenario_id': goal.scenario_id,
        }

    def _build_randomization_config(self, params: dict[str, Any]) -> DomainRandomizationConfig:
        cfg = DomainRandomizationConfig()
        cfg.random_seed = int(params.get('seed', 42))
        cfg.randomization_strength = float(params.get('strength', 0.5))
        cfg.joint_damping_min = float(params.get('joint_damping_min', 0.5))
        cfg.joint_damping_max = float(params.get('joint_damping_max', 2.0))
        cfg.joint_friction_min = float(params.get('joint_friction_min', 0.0))
        cfg.joint_friction_max = float(params.get('joint_friction_max', 0.2))
        cfg.motor_strength_min = float(params.get('motor_strength_min', 0.8))
        cfg.motor_strength_max = float(params.get('motor_strength_max', 1.2))
        cfg.position_noise_std = float(params.get('position_noise_std', 0.01))
        cfg.velocity_noise_std = float(params.get('velocity_noise_std', 0.05))
        cfg.time_delay_min_ms = float(params.get('time_delay_min_ms', 0.0))
        cfg.time_delay_max_ms = float(params.get('time_delay_max_ms', 50.0))
        cfg.payload_mass_min = float(params.get('payload_mass_min', 0.0))
        cfg.payload_mass_max = float(params.get('payload_mass_max', 5.0))
        return cfg

    def _cmd_set_randomization(self, params: dict[str, Any]) -> dict[str, Any]:
        if not self._set_randomization_client.wait_for_service(timeout_sec=1.0):
            return {'success': False, 'message': '/bridge/set_randomization unavailable'}
        req = SetRandomization.Request()
        req.config = self._build_randomization_config(params)
        future = self._set_randomization_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done():
            return {'success': False, 'message': 'set_randomization timed out'}
        resp = future.result()
        return {'success': resp.success, 'message': resp.message}

    def _cmd_inject_shift(self, params: dict[str, Any]) -> dict[str, Any]:
        if not self._inject_shift_client.wait_for_service(timeout_sec=1.0):
            return {'success': False, 'message': '/bridge/inject_shift unavailable'}
        req = InjectShift.Request()
        req.parameter_name = str(params.get('parameter', 'joint_damping'))
        req.delta_percent = float(params.get('delta_percent', 30.0))
        req.duration_sec = float(params.get('duration', 10.0))
        future = self._inject_shift_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done():
            return {'success': False, 'message': 'inject_shift timed out'}
        resp = future.result()
        return {'success': resp.success, 'message': resp.message}

    def _generate_export(
        self,
        request: ExportExperiment.Request,
        response: ExportExperiment.Response,
        *,
        screenshot_b64: str | None = None,
    ) -> None:
        report_dir = Path(os.path.expanduser(
            self.get_parameter('report_output_dir').value)).resolve()
        report_dir.mkdir(parents=True, exist_ok=True)

        experiment_id = request.experiment_id or f'exp_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        fmt = (request.format or 'html').lower()
        ext = 'html' if fmt in ('html', 'pdf') else 'json'
        out_path = request.output_path
        if not out_path:
            out_path = str(report_dir / f'{experiment_id}.{ext}')
        out_path = os.path.expanduser(out_path)

        summary = self._history.summary()
        metadata = {
            **self._experiment_metadata,
            'duration_sec': time.time() - self._history.start_time,
        }
        recommendation = ''
        if self._latest_risk:
            recommendation = self._latest_risk.recommendation

        if ext == 'json':
            payload = {
                'experiment_id': experiment_id,
                'metadata': metadata,
                'summary': summary,
                'risk_timeline': list(self._history.risk_timeline),
                'metrics_timeline': list(self._history.metrics_timeline),
                'alerts': self._history.alerts,
                'recommendation': recommendation,
            }
            Path(out_path).write_text(json.dumps(payload, indent=2, ensure_ascii=False))
        else:
            html = render_html_report(
                experiment_id=experiment_id,
                metadata=metadata,
                summary=summary,
                risk_timeline=list(self._history.risk_timeline),
                metrics_timeline=list(self._history.metrics_timeline),
                alerts=self._history.alerts,
                latest_risk=risk_status_to_dict(self._latest_risk) if self._latest_risk else None,
                latest_metrics=(
                    distribution_metrics_to_dict(self._latest_metrics)
                    if self._latest_metrics else None
                ),
                screenshot_b64=screenshot_b64,
                recommendation=recommendation,
            )
            Path(out_path).write_text(html, encoding='utf-8')

        response.success = True
        response.file_path = out_path
        response.message = f'Report exported to {out_path}'
        self.get_logger().info(response.message)

    def _handle_export(self, request, response):
        self._generate_export(request, response)
        return response

    def _handle_start_recording(self, _request, response):
        if self._recording:
            response.success = False
            response.message = f'Already recording: {self._bag_path}'
            return response
        bag_dir = Path(os.path.expanduser(
            self.get_parameter('rosbag_output_dir').value)).resolve()
        bag_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._bag_path = str(bag_dir / f'hoc_{stamp}')
        topics = [
            '/monitor/distribution_metrics',
            '/risk/status',
            '/risk/alerts',
        ]
        cmd = ['ros2', 'bag', 'record', '-o', self._bag_path, *topics]
        try:
            self._recording_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self._recording = True
            response.success = True
            response.message = f'Recording to {self._bag_path}'
        except OSError as exc:
            response.success = False
            response.message = f'Failed to start rosbag: {exc}'
        return response

    def _handle_stop_recording(self, _request, response):
        if not self._recording or self._recording_proc is None:
            response.success = False
            response.message = 'Not recording'
            return response
        self._recording_proc.terminate()
        try:
            self._recording_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._recording_proc.kill()
        self._recording = False
        self._recording_proc = None
        response.success = True
        response.message = f'Recording stopped: {self._bag_path}'
        loop = self._loop
        if loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._ws_hub.broadcast('recording_status', {
                    'recording': False,
                    'bag_path': self._bag_path,
                }),
                loop,
            )
        return response

    def _run_ws_server(self) -> None:
        try:
            import websockets
        except ImportError:
            self.get_logger().warn(
                'websockets not installed — WebSocket server disabled. '
                'pip install websockets')
            return

        async def handler(websocket):
            self._ws_hub.register(websocket)
            await self._ws_hub.send_to(websocket, {
                'type': 'connected',
                'message': 'HOC WebSocket ready',
            })
            try:
                async for message in websocket:
                    await self._handle_ws_message(websocket, message)
            finally:
                self._ws_hub.unregister(websocket)

        async def main():
            self._loop = asyncio.get_running_loop()
            if self._serve_frontend and self._frontend_dist:
                await start_static_server(self._frontend_dist, self._http_port)
            elif self._serve_frontend:
                self.get_logger().warn(
                    'serve_frontend=true but frontend/dist not found — '
                    'run: cd hoc_console/frontend && npm run build')
            async with websockets.serve(handler, '0.0.0.0', self._ws_port):
                await asyncio.Future()

        asyncio.run(main())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HocServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
