"""HOC server node: ROS 2 subscriptions + WebSocket broadcast (scaffold)."""

from __future__ import annotations

import asyncio
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from bridge_monitor_msgs.msg import DistributionMetrics, RiskStatus
from bridge_monitor_msgs.srv import ExportExperiment

from hoc_console.ros_bridge import distribution_metrics_to_dict, risk_status_to_dict
from hoc_console.ws_hub import WsHub


class HocServerNode(Node):
    """Bridge ROS 2 risk/monitor topics to WebSocket dashboard clients."""

    def __init__(self) -> None:
        super().__init__('hoc_server')

        self.declare_parameter('websocket_port', 8765)
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('push_frequency_hz', 5.0)
        self.declare_parameter('rosbag_output_dir', '~/ros2_ws/bags')

        self._ws_hub = WsHub()
        self._latest_risk: RiskStatus | None = None
        self._latest_metrics: DistributionMetrics | None = None

        self.create_subscription(RiskStatus, '/risk/status', self._on_risk, 10)
        self.create_subscription(
            DistributionMetrics, '/monitor/distribution_metrics', self._on_metrics, 10)
        self.create_subscription(String, '/risk/alerts', self._on_alert, 10)

        self.create_service(ExportExperiment, '/hoc/export_experiment', self._handle_export)
        self.create_service(Trigger, '/hoc/start_recording', self._handle_start_recording)
        self.create_service(Trigger, '/hoc/stop_recording', self._handle_stop_recording)

        hz = self.get_parameter('push_frequency_hz').value
        self._push_timer = self.create_timer(1.0 / hz, self._push_latest)

        self._ws_port = self.get_parameter('websocket_port').value
        self._ws_thread = threading.Thread(target=self._run_ws_server, daemon=True)
        self._ws_thread.start()

        self.get_logger().info(
            f'hoc_server started (scaffold). WebSocket port={self._ws_port}')

    def _on_risk(self, msg: RiskStatus) -> None:
        self._latest_risk = msg

    def _on_metrics(self, msg: DistributionMetrics) -> None:
        self._latest_metrics = msg

    def _on_alert(self, msg: String) -> None:
        self.get_logger().info(f'Alert: {msg.data}')

    def _push_latest(self) -> None:
        if self._latest_risk is None and self._latest_metrics is None:
            return
        loop = getattr(self, '_loop', None)
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
            try:
                async for message in websocket:
                    self.get_logger().debug(f'WS command: {message}')
            finally:
                self._ws_hub.unregister(websocket)

        async def main():
            self._loop = asyncio.get_running_loop()
            async with websockets.serve(handler, '0.0.0.0', self._ws_port):
                await asyncio.Future()

        asyncio.run(main())

    def _handle_export(self, request, response):
        response.success = True
        response.file_path = request.output_path or '/tmp/experiment_report.json'
        response.message = 'Scaffold: export not yet implemented.'
        return response

    def _handle_start_recording(self, request, response):
        response.success = True
        response.message = 'Scaffold: rosbag recording not yet implemented.'
        return response

    def _handle_stop_recording(self, request, response):
        response.success = True
        response.message = 'Scaffold: rosbag recording not yet implemented.'
        return response


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
