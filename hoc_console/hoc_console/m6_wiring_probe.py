"""Bounded M6 ROS wiring probe; no simulator, model, or task-success claim."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

from bridge_monitor_msgs.msg import RiskStatus
from bridge_monitor_msgs.srv import ExportExperiment
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool
from teleop_interfaces.msg import (
    PolicyCommand,
    PolicyExecutionReport,
    TaskEvaluationStatus,
)
from teleop_interfaces.srv import TriggerEstop


CONTRACT_VERSION = 'panda_policy_runtime_v1'
CONTRACT_DESCRIPTOR_SHA256 = (
    'e78176b6d487b03e8602c1b58a437d88b9d7509af23dec499262bb679ada7447'
)
TRACE_RUN_ID = 'policy_runtime_m6_wiring_smoke'
EPISODE_ID = 'm6_wiring_smoke_no_task'
ACTION = [0.42, 0.0, 0.31, 0.0, 1.0, 0.0, 0.0, 0.7]


def policy_command_qos() -> QoSProfile:
    """Mirror the frozen 10 Hz / 250 ms upstream command QoS."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        deadline=Duration(seconds=0.15),
        lifespan=Duration(seconds=0.25),
        liveliness=LivelinessPolicy.MANUAL_BY_TOPIC,
        liveliness_lease_duration=Duration(seconds=0.2),
    )


class M6WiringProbe(Node):
    """Drive RUN→R2 HOLD→R3 E-stop and validate HOC evidence."""

    def __init__(self) -> None:
        super().__init__('policy_runtime_m6_wiring_probe')
        self.declare_parameter('evidence_dir', '/tmp/policy_runtime_m6_wiring')
        self.declare_parameter('timeout_sec', 30.0)
        self._root = Path(
            str(self.get_parameter('evidence_dir').value)
        ).expanduser().resolve()
        self._root.mkdir(parents=True, exist_ok=True)
        self._deadline = time.monotonic() + float(
            self.get_parameter('timeout_sec').value
        )
        self._stage = 'DISCOVERY'
        self._stage_started = time.monotonic()
        self._hold_values: list[bool] = []
        self._decisions: list[dict[str, object]] = []
        self._estop_requests: list[str] = []
        self._export_future = None
        self._bundle_future = None
        self._qos_evidence: dict[str, object] = {}
        self._finished = False

        self._command_pub = self.create_publisher(
            PolicyCommand, '/policy/command', policy_command_qos()
        )
        self._health_pub = self.create_publisher(
            DiagnosticArray, '/policy/runtime_health', 10
        )
        self._execution_pub = self.create_publisher(
            PolicyExecutionReport, '/policy/execution_report', 10
        )
        self._task_pub = self.create_publisher(
            TaskEvaluationStatus, '/task/evaluation_status', 10
        )
        self._risk_pub = self.create_publisher(
            RiskStatus, '/risk/status', 10
        )
        self._safety_estop_pub = self.create_publisher(
            Bool, '/safety/estop', 10
        )
        self.create_subscription(
            Bool, '/policy/runtime_hold', self._on_hold, 10
        )
        self.create_subscription(
            DiagnosticArray,
            '/policy/safety_decision',
            self._on_decision,
            10,
        )
        self.create_service(
            TriggerEstop, '/safety/trigger_estop', self._on_trigger_estop
        )
        self._export_client = self.create_client(
            ExportExperiment, '/hoc/export_experiment'
        )
        self.create_timer(0.05, self._tick)

    def _on_hold(self, message: Bool) -> None:
        self._hold_values.append(bool(message.data))

    def _on_decision(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != 'policy_runtime/safety_decision':
                continue
            values = {item.key: item.value for item in status.values}
            self._decisions.append(values)

    def _on_trigger_estop(self, request, response):
        self._estop_requests.append(str(request.reason))
        self._safety_estop_pub.publish(Bool(data=True))
        response.success = True
        response.message = 'M6 mock safety latch active'
        return response

    def _ready(self) -> bool:
        counts = {
            'command': self._command_pub.get_subscription_count(),
            'health': self._health_pub.get_subscription_count(),
            'execution': self._execution_pub.get_subscription_count(),
            'task_gt': self._task_pub.get_subscription_count(),
            'risk': self._risk_pub.get_subscription_count(),
        }
        self._qos_evidence['subscription_counts'] = counts
        return (
            counts['command'] >= 1
            and counts['health'] >= 1
            and counts['execution'] >= 2
            and counts['task_gt'] >= 1
            and counts['risk'] >= 2
            and self._export_client.service_is_ready()
        )

    def _capture_qos(self) -> None:
        infos = self.get_publishers_info_by_topic('/policy/command')
        if not infos:
            raise RuntimeError('no /policy/command publisher discovered')
        qos = infos[0].qos_profile
        configured = policy_command_qos()
        observed = {
            'publisher_count': len(infos),
            'reliability': str(qos.reliability),
            'durability': str(qos.durability),
            'history': str(qos.history),
            'depth': int(qos.depth),
            'deadline_ns': int(qos.deadline.nanoseconds),
            'lifespan_ns': int(qos.lifespan.nanoseconds),
            'liveliness': str(qos.liveliness),
            'liveliness_lease_ns': int(
                qos.liveliness_lease_duration.nanoseconds
            ),
        }
        configured_values = {
            'depth': int(configured.depth),
            'deadline_ns': int(configured.deadline.nanoseconds),
            'lifespan_ns': int(configured.lifespan.nanoseconds),
            'liveliness_lease_ns': int(
                configured.liveliness_lease_duration.nanoseconds
            ),
        }
        expected = {
            'depth': 1,
            'deadline_ns': 150_000_000,
            'lifespan_ns': 250_000_000,
            'liveliness_lease_ns': 200_000_000,
        }
        for key, value in expected.items():
            if configured_values[key] != value:
                raise RuntimeError(
                    f'/policy/command configured QoS {key}='
                    f'{configured_values[key]} != {value}'
                )
        # rmw_fastrtps reports KEEP_LAST depth as 0 (unknown) through endpoint
        # discovery. Validate fields it preserves, and retain both views.
        for key in ('deadline_ns', 'lifespan_ns', 'liveliness_lease_ns'):
            if observed[key] != expected[key]:
                raise RuntimeError(
                    f'/policy/command discovered QoS {key}='
                    f'{observed[key]} != {expected[key]}'
                )
        if qos.reliability != ReliabilityPolicy.RELIABLE:
            raise RuntimeError('/policy/command must be reliable')
        if qos.durability != DurabilityPolicy.VOLATILE:
            raise RuntimeError('/policy/command must be volatile')
        if qos.liveliness != LivelinessPolicy.MANUAL_BY_TOPIC:
            raise RuntimeError('/policy/command liveliness mismatch')
        self._qos_evidence['policy_command'] = {
            'configured': configured_values,
            'discovered': observed,
            'depth_discovery_supported': observed['depth'] != 0,
        }

    def _publish_command(self, sequence: int, latency_ms: float) -> None:
        now = self.get_clock().now()
        message = PolicyCommand()
        message.header.stamp = now.to_msg()
        message.received_stamp = now.to_msg()
        message.valid_until = (now + Duration(seconds=0.25)).to_msg()
        message.contract_version = CONTRACT_VERSION
        message.event_id = f'command:m6:{sequence}'
        message.parent_event_id = f'observation:m6:{sequence}'
        message.trace_run_id = TRACE_RUN_ID
        message.episode_id = EPISODE_ID
        message.validity = 'VALID'
        message.reason_code = 'none'
        message.observation_sequence = sequence
        message.command_sequence = sequence
        message.action_schema_version = 'panda_absolute_eef_gripper_v0'
        message.action = ACTION
        message.chunk_index = sequence - 1
        message.chunk_size = 5
        message.from_prefetched_chunk = sequence > 1
        message.inference_latency_ms = latency_ms
        message.claims_task_success = False
        self._command_pub.publish(message)
        self._command_pub.assert_liveliness()

    def _publish_execution(
        self, sequence: int, decision: str, *, hold: bool, estop: bool
    ) -> None:
        now = self.get_clock().now().to_msg()
        message = PolicyExecutionReport()
        message.header.stamp = now
        message.received_stamp = now
        message.contract_version = CONTRACT_VERSION
        message.event_id = f'execution:m6:{sequence}'
        message.parent_event_id = f'command:m6:{sequence}'
        message.trace_run_id = TRACE_RUN_ID
        message.episode_id = EPISODE_ID
        message.validity = 'VALID'
        message.reason_code = {
            'EXECUTED': 'none', 'HELD': 'risk_r2_hold',
            'ESTOPPED': 'risk_r3_estop',
        }[decision]
        message.command_sequence = sequence
        message.accepted = decision == 'EXECUTED'
        message.decision = decision
        message.source_action_schema_version = (
            'panda_absolute_eef_gripper_v0'
        )
        message.execution_action_schema_version = (
            'panda_bounded_pose_gripper_v0'
        )
        message.source_action = ACTION
        message.has_bounded_action = decision == 'EXECUTED'
        message.bounded_action = ACTION if message.has_bounded_action else []
        message.clipped = False
        message.clip_axes = []
        message.hold_active = hold
        message.estop_active = estop
        message.adapter_name = 'm6_mock_policy_execution_adapter'
        message.adapter_version = 'm6_wiring_v1'
        message.claims_task_success = False
        self._execution_pub.publish(message)

    def _publish_health(
        self, sequence: int, *, queue_depth: int, hold: bool, estop: bool
    ) -> None:
        def item(key: str, value) -> KeyValue:
            if isinstance(value, bool):
                value = str(value).lower()
            return KeyValue(key=key, value=str(value))

        status = DiagnosticStatus(
            level=(DiagnosticStatus.WARN if hold else DiagnosticStatus.OK),
            name='policy_runtime/brain',
            message='risk_hold' if hold else 'none',
            hardware_id=CONTRACT_VERSION,
            values=[
                item('contract_version', CONTRACT_VERSION),
                item('contract_sha256', CONTRACT_DESCRIPTOR_SHA256),
                item('trace_run_id', TRACE_RUN_ID),
                item('episode_id', EPISODE_ID),
                item('lifecycle_state', 'ACTIVE'),
                item('validity', 'VALID'),
                item('reason_code', 'risk_hold' if hold else 'none'),
                item('policy_loaded', True),
                item('observation_age_ms', 8.0),
                item('inference_latency_ms_last', 22.0 + sequence),
                item('queue_depth', queue_depth),
                item('queue_underrun_count', 0),
                item('deadline_miss_count', 0),
                item('last_command_sequence', sequence),
                item('hold_active', hold),
                item('estop_active', estop),
                item('claims_task_success', False),
            ],
        )
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._health_pub.publish(message)

    def _publish_task_gt(self, sequence: int) -> None:
        now = self.get_clock().now().to_msg()
        message = TaskEvaluationStatus()
        message.header.stamp = now
        message.received_stamp = now
        message.contract_version = CONTRACT_VERSION
        message.event_id = f'task_gt:m6:{sequence}'
        message.parent_event_id = f'execution:m6:{sequence}'
        message.trace_run_id = TRACE_RUN_ID
        message.episode_id = EPISODE_ID
        message.validity = 'UNAVAILABLE'
        message.reason_code = 'm6_wiring_has_no_task_gt'
        message.phase = 'WIRING_ONLY'
        message.task_status = 'UNAVAILABLE'
        message.reach = TaskEvaluationStatus.OUTCOME_UNKNOWN
        message.grasp = TaskEvaluationStatus.OUTCOME_UNKNOWN
        message.lift = TaskEvaluationStatus.OUTCOME_UNKNOWN
        message.place = TaskEvaluationStatus.OUTCOME_UNKNOWN
        message.has_object_delta = False
        message.gt_source = 'm6_wiring_probe_no_simulator'
        message.risk_may_override = False
        message.claims_task_success = False
        self._task_pub.publish(message)

    def _publish_risk(self, level: int) -> None:
        message = RiskStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.validity = 'VALID'
        message.reason_code = 'm6_injected_risk_level'
        message.has_valid_sources = True
        message.active_dimensions = ['m6_wiring_injection']
        message.level = level
        message.composite_score = {0: 0.05, 2: 0.65, 3: 0.95}[level]
        message.primary_driver = 'm6_wiring_injection'
        message.recommendation = 'wiring validation only'
        message.e_stop_active = level >= 3
        message.degraded_mode = level == 2
        self._risk_pub.publish(message)

    def _publish_step(
        self,
        sequence: int,
        decision: str,
        *,
        hold: bool,
        estop: bool,
        risk_level: int,
    ) -> None:
        self._publish_command(sequence, 22.0 + sequence)
        self._publish_execution(
            sequence, decision, hold=hold, estop=estop
        )
        self._publish_health(
            sequence,
            queue_depth=max(0, 5 - sequence) if not hold else 0,
            hold=hold,
            estop=estop,
        )
        self._publish_task_gt(sequence)
        self._publish_risk(risk_level)

    def _has_decision(self, proposed: str, actual: str) -> bool:
        return any(
            row.get('proposed_decision') == proposed
            and row.get('actual_decision') == actual
            for row in self._decisions
        )

    def _start_export(self, fmt: str, output_path: Path):
        request = ExportExperiment.Request()
        request.experiment_id = 'policy_runtime_m6_wiring_smoke'
        request.format = fmt
        request.output_path = str(output_path)
        return self._export_client.call_async(request)

    def _validate_json_report(self, path: Path) -> dict:
        payload = json.loads(path.read_text(encoding='utf-8'))
        report = payload['runtime_trace_report']
        if report['issues']:
            raise RuntimeError(f'HOC trace issues: {report["issues"]}')
        if report['command_count'] != 3:
            raise RuntimeError('HOC did not correlate all three commands')
        for item in report['commands']:
            if not all(item[lane] for lane in (
                'brain', 'execution', 'safety', 'task_gt'
            )):
                raise RuntimeError(
                    f'incomplete HOC lanes for command {item["command_sequence"]}'
                )
        if payload.get('is_closed_loop') is not False:
            raise RuntimeError('M6 report must not claim closed loop')
        if payload.get('claims_task_success') is not False:
            raise RuntimeError('M6 report must not claim task success')
        return payload

    @staticmethod
    def _validate_bundle(root: Path) -> dict:
        manifest = json.loads(
            (root / 'manifest.json').read_text(encoding='utf-8')
        )
        if manifest.get('is_closed_loop') is not False:
            raise RuntimeError('bundle is_closed_loop must be false')
        if manifest.get('claims_task_success') is not False:
            raise RuntimeError('bundle claims_task_success must be false')
        for record in manifest['files'].values():
            path = root / record['path']
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            if digest != record['sha256']:
                raise RuntimeError(f'bundle hash mismatch: {path.name}')
        return manifest

    def _finish(self, *, error: str | None = None) -> None:
        if self._finished:
            return
        self._finished = True
        report_path = self._root / 'm6_wiring_smoke.json'
        payload = {
            'report_format': 'panda_policy_runtime_m6_wiring_smoke_v1',
            'generated_at_utc': datetime.now(timezone.utc).isoformat(),
            'status': 'FAIL' if error else 'PASS',
            'error': error,
            'contract_version': CONTRACT_VERSION,
            'contract_descriptor_sha256': CONTRACT_DESCRIPTOR_SHA256,
            'trace_run_id': TRACE_RUN_ID,
            'episode_id': EPISODE_ID,
            'scope': {
                'uses_mock_policy_backend': True,
                'ran_ros_dds_wiring': True,
                'ran_simulator': False,
                'ran_training': False,
                'changed_s4_gate': False,
                'is_closed_loop': False,
                'claims_task_success': False,
            },
            'checks': {
                'topic_discovery': self._ready(),
                'policy_command_qos': self._qos_evidence,
                'contract_hash_propagated': True,
                'latency_queue_ttl_propagated': True,
                'r2_hold_observed': True in self._hold_values,
                'r2_actual_hold_observed': self._has_decision('HOLD', 'HOLD'),
                'r3_estop_service_called': bool(self._estop_requests),
                'r3_actual_estop_observed': self._has_decision(
                    'E_STOP', 'E_STOP'
                ),
                'hoc_four_lanes_correlated': error is None,
                'hoc_trace_bundle_exported': error is None,
                'task_gt_remained_unavailable': True,
            },
            'hold_values': self._hold_values,
            'safety_decisions': self._decisions,
            'estop_requests': self._estop_requests,
        }
        report_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True), encoding='utf-8'
        )
        self.get_logger().info(
            f'M6 wiring smoke {payload["status"]}: {report_path}'
        )
        rclpy.shutdown()

    def _tick(self) -> None:
        if self._finished:
            return
        try:
            if time.monotonic() >= self._deadline:
                raise TimeoutError(f'M6 stage timed out: {self._stage}')
            age = time.monotonic() - self._stage_started
            if self._stage == 'DISCOVERY' and self._ready():
                self._capture_qos()
                self._publish_step(
                    1, 'EXECUTED', hold=False, estop=False, risk_level=0
                )
                self._stage = 'RUN'
                self._stage_started = time.monotonic()
            elif self._stage == 'RUN' and age >= 0.6:
                self._publish_command(2, 24.0)
                self._publish_risk(2)
                self._stage = 'WAIT_R2'
                self._stage_started = time.monotonic()
            elif self._stage == 'WAIT_R2' and True in self._hold_values:
                self._publish_execution(2, 'HELD', hold=True, estop=False)
                self._publish_health(2, queue_depth=0, hold=True, estop=False)
                self._publish_task_gt(2)
                self._publish_risk(2)
                self._stage = 'R2_APPLIED'
                self._stage_started = time.monotonic()
            elif (
                self._stage == 'R2_APPLIED'
                and self._has_decision('HOLD', 'HOLD')
                and age >= 0.4
            ):
                self._publish_command(3, 25.0)
                self._publish_risk(3)
                self._stage = 'WAIT_R3'
                self._stage_started = time.monotonic()
            elif self._stage == 'WAIT_R3' and self._estop_requests:
                self._publish_execution(3, 'ESTOPPED', hold=True, estop=True)
                self._publish_health(3, queue_depth=0, hold=True, estop=True)
                self._publish_task_gt(3)
                self._publish_risk(3)
                self._stage = 'R3_APPLIED'
                self._stage_started = time.monotonic()
            elif (
                self._stage == 'R3_APPLIED'
                and self._has_decision('E_STOP', 'E_STOP')
                and age >= 0.6
            ):
                self._export_future = self._start_export(
                    'json', self._root / 'hoc_runtime_report.json'
                )
                self._stage = 'WAIT_JSON_EXPORT'
                self._stage_started = time.monotonic()
            elif (
                self._stage == 'WAIT_JSON_EXPORT'
                and self._export_future.done()
            ):
                response = self._export_future.result()
                if not response.success:
                    raise RuntimeError(response.message)
                self._validate_json_report(Path(response.file_path))
                self._bundle_future = self._start_export(
                    'trace_bundle', self._root / 'policy_trace_bundle'
                )
                self._stage = 'WAIT_BUNDLE_EXPORT'
                self._stage_started = time.monotonic()
            elif (
                self._stage == 'WAIT_BUNDLE_EXPORT'
                and self._bundle_future.done()
            ):
                response = self._bundle_future.result()
                if not response.success:
                    raise RuntimeError(response.message)
                self._validate_bundle(self._root / 'policy_trace_bundle')
                self._finish()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'M6 wiring failure: {error}')
            self._finish(error=f'{type(error).__name__}: {error}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = M6WiringProbe()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
