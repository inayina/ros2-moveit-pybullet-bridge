#!/usr/bin/env python3
"""Benchmark PolicyRunner latency, resource usage, and health alarms."""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import rclpy
from bridge_monitor_msgs.msg import DistributionMetrics
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

try:
    import psutil
except ImportError:  # pragma: no cover - exercised via import guard in main
    psutil = None


def _stamp_sec(msg) -> float:
    stamp = msg.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _health_kv(status, key: str) -> str:
    for item in status.values:
        if item.key == key:
            return item.value
    return ''


def _diagnostic_level(level: Any) -> int:
    if isinstance(level, (bytes, bytearray)):
        return int(level[0])
    return int(level)


@dataclass
class TimeseriesRow:
    episode: int
    monotonic_sec: float
    latency_ms: Optional[float] = None
    cpu_percent: Optional[float] = None
    rss_mb: Optional[float] = None
    inference_latency_ms: Optional[float] = None
    kl_mean: Optional[float] = None
    w1_mean: Optional[float] = None
    mmd: Optional[float] = None


@dataclass
class HealthEvent:
    monotonic_sec: float
    level: int
    reason: str
    inference_latency_ms: float
    last_action_age_ms: float


class BenchmarkCollector(Node):
    def __init__(self) -> None:
        super().__init__('policy_runner_benchmark_collector')
        self.timeseries: list[TimeseriesRow] = []
        self.health_events: list[HealthEvent] = []
        self.episode_latencies: list[float] = []
        self._pending_cmd_stamp: Optional[float] = None
        self._pending_cmd_mono: Optional[float] = None
        self._latest_metrics: Optional[DistributionMetrics] = None
        self._latest_inference_latency_ms = 0.0

        self.create_subscription(
            JointTrajectory,
            '/bridge/command',
            self._on_command,
            10,
        )
        self.create_subscription(
            JointState,
            '/bridge/sim/joint_states',
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DistributionMetrics,
            '/monitor/distribution_metrics',
            self._on_metrics,
            10,
        )
        self.create_subscription(
            __import__('diagnostic_msgs.msg', fromlist=['DiagnosticArray']).DiagnosticArray,
            '/system_health',
            self._on_health,
            10,
        )

    def _on_command(self, msg: JointTrajectory) -> None:
        self._pending_cmd_stamp = _stamp_sec(msg)
        self._pending_cmd_mono = time.monotonic()

    def _on_joint_state(self, msg: JointState) -> None:
        if self._pending_cmd_mono is None:
            return
        mono_latency_ms = (time.monotonic() - self._pending_cmd_mono) * 1000.0
        header_latency_ms = None
        if self._pending_cmd_stamp is not None:
            header_latency_ms = (_stamp_sec(msg) - self._pending_cmd_stamp) * 1000.0
        latency_ms = mono_latency_ms
        if header_latency_ms is not None and header_latency_ms >= 0.0:
            latency_ms = max(mono_latency_ms, header_latency_ms)
        self.episode_latencies.append(latency_ms)
        self._pending_cmd_stamp = None
        self._pending_cmd_mono = None

    def _on_metrics(self, msg: DistributionMetrics) -> None:
        self._latest_metrics = msg

    def _on_health(self, msg) -> None:
        for status in msg.status:
            if status.name != 'policy_runner.health':
                continue
            inference_ms = float(_health_kv(status, 'inference_latency_ms') or 0.0)
            self._latest_inference_latency_ms = inference_ms
            self.health_events.append(
                HealthEvent(
                    monotonic_sec=time.monotonic(),
                    level=_diagnostic_level(status.level),
                    reason=_health_kv(status, 'reason'),
                    inference_latency_ms=inference_ms,
                    last_action_age_ms=float(_health_kv(status, 'last_action_age_ms') or 0.0),
                )
            )

    def sample_resources(self, process: Optional[psutil.Process]) -> tuple[Optional[float], Optional[float]]:
        if process is None:
            return None, None
        cpu = process.cpu_percent(interval=None)
        rss_mb = process.memory_info().rss / (1024.0 * 1024.0)
        return cpu, rss_mb

    def record_sample(self, episode: int, process: Optional[psutil.Process]) -> None:
        cpu, rss = self.sample_resources(process)
        metrics = self._latest_metrics
        self.timeseries.append(
            TimeseriesRow(
                episode=episode,
                monotonic_sec=time.monotonic(),
                latency_ms=self.episode_latencies[-1] if self.episode_latencies else None,
                cpu_percent=cpu,
                rss_mb=rss,
                inference_latency_ms=self._latest_inference_latency_ms,
                kl_mean=metrics.kl_divergence_mean if metrics else None,
                w1_mean=metrics.wasserstein_mean if metrics else None,
                mmd=metrics.mmd_statistic if metrics else None,
            )
        )

    def reset_episode(self) -> None:
        self.episode_latencies.clear()
        self._pending_cmd_stamp = None
        self._pending_cmd_mono = None


def _find_process(name: str) -> Optional[psutil.Process]:
    if psutil is None:
        return None
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            cmdline = ' '.join(proc.info.get('cmdline') or [])
            if name in cmdline and 'benchmark_system.py' not in cmdline:
                return psutil.Process(proc.info['pid'])
        except (psutil.Error, KeyError):
            continue
    return None


def _launch_stack(
    *,
    strategy: str,
    replay_path: str,
    seed: int,
    fault_injection: bool,
    inference_freq: int,
) -> tuple[subprocess.Popen, subprocess.Popen]:
    launch = subprocess.Popen(
        [
            'ros2', 'launch', 'pybullet_bridge', 'test_monitoring.launch.py',
            'sim_mode:=DIRECT',
            'robot:=planar_2dof',
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    time.sleep(3.0)

    runner_args = [
        'ros2', 'run', 'pybullet_bridge', 'policy_runner', '--ros-args',
        '-p', f'strategy_type:={strategy}',
        '-p', f'seed:={seed}',
        '-p', f'policy_inference_freq:={inference_freq}',
    ]
    if strategy == 'replay':
        runner_args.extend(['-p', f'replay_path:={replay_path}'])
    if fault_injection:
        runner_args.extend([
            '-p', 'fault_injection_enabled:=true',
            '-p', 'fault_sleep_probability:=1.0',
            '-p', 'fault_sleep_sec:=0.1',
            '-p', 'watchdog_timeout_sec:=0.05',
        ])

    runner = subprocess.Popen(
        runner_args,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    time.sleep(2.0)
    return launch, runner


def _stop_process(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)


def _wait_for_topics(timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        topics = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            check=False,
        )
        if '/bridge/sim/joint_states' in topics.stdout and '/bridge/command' in topics.stdout:
            return True
        time.sleep(0.5)
    return False


def _render_html(summary: dict[str, Any], output_dir: Path) -> str:
    return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8"/>
  <title>Policy Runner Benchmark — {summary.get('strategy', '')}</title>
  <style>
    body {{ font-family: system-ui, sans-serif; background: #141414; color: #e8e8e8; margin: 24px; }}
    h1, h2 {{ color: #69b1ff; }}
    table {{ border-collapse: collapse; width: 100%; margin: 12px 0; }}
    th, td {{ border: 1px solid #434343; padding: 8px; text-align: left; }}
    th {{ background: #1f1f1f; }}
    .metric-grid {{ display: grid; grid-template-columns: repeat(4, 1fr); gap: 12px; }}
    .metric-card {{ background: #1f1f1f; padding: 16px; border-radius: 8px; }}
    .metric-value {{ font-size: 22px; font-weight: bold; color: #95de64; }}
  </style>
</head>
<body>
  <h1>Policy Runner System Benchmark</h1>
  <p>Output directory: <code>{output_dir}</code></p>
  <div class="metric-grid">
    <div class="metric-card"><div>Mean latency</div><div class="metric-value">{summary.get('mean_latency_ms', 0):.3f} ms</div></div>
    <div class="metric-card"><div>Max latency</div><div class="metric-value">{summary.get('max_latency_ms', 0):.3f} ms</div></div>
    <div class="metric-card"><div>CPU peak</div><div class="metric-value">{summary.get('cpu_peak_percent', 0):.1f}%</div></div>
    <div class="metric-card"><div>RSS peak</div><div class="metric-value">{summary.get('rss_peak_mb', 0):.1f} MB</div></div>
  </div>
  <h2>Summary</h2>
  <pre>{json.dumps(summary, indent=2, ensure_ascii=False)}</pre>
  <h2>Artifacts</h2>
  <ul>
    <li>benchmark_timeseries.csv</li>
    <li>benchmark_summary.json</li>
    <li>system_health_events.csv</li>
  </ul>
</body>
</html>"""


def _write_timeseries_csv(path: Path, rows: list[TimeseriesRow]) -> None:
    with path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow([
            'episode',
            'monotonic_sec',
            'latency_ms',
            'cpu_percent',
            'rss_mb',
            'inference_latency_ms',
            'kl_mean',
            'w1_mean',
            'mmd',
        ])
        for row in rows:
            writer.writerow([
                row.episode,
                f'{row.monotonic_sec:.6f}',
                '' if row.latency_ms is None else f'{row.latency_ms:.6f}',
                '' if row.cpu_percent is None else f'{row.cpu_percent:.3f}',
                '' if row.rss_mb is None else f'{row.rss_mb:.3f}',
                '' if row.inference_latency_ms is None else f'{row.inference_latency_ms:.3f}',
                '' if row.kl_mean is None else f'{row.kl_mean:.6f}',
                '' if row.w1_mean is None else f'{row.w1_mean:.6f}',
                '' if row.mmd is None else f'{row.mmd:.6f}',
            ])


def _write_health_csv(path: Path, events: list[HealthEvent]) -> None:
    with path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow([
            'monotonic_sec',
            'level',
            'reason',
            'inference_latency_ms',
            'last_action_age_ms',
        ])
        for event in events:
            writer.writerow([
                f'{event.monotonic_sec:.6f}',
                event.level,
                event.reason,
                f'{event.inference_latency_ms:.3f}',
                f'{event.last_action_age_ms:.3f}',
            ])


def _health_alarm_latency_ms(
    events: list[HealthEvent],
    fault_start_mono: Optional[float],
) -> Optional[float]:
    if fault_start_mono is None:
        return None
    for event in events:
        if event.monotonic_sec >= fault_start_mono and event.level >= DiagnosticStatus.WARN:
            return (event.monotonic_sec - fault_start_mono) * 1000.0
    return None


def run_benchmark(args: argparse.Namespace) -> dict[str, Any]:
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    replay_path = args.replay_path
    if args.strategy == 'replay' and not replay_path:
        fixture = Path(__file__).resolve().parents[1] / 'pybullet_bridge' / 'test' / 'fixtures' / 'planar_2dof_replay.pkl'
        if not fixture.is_file():
            from pybullet_bridge.learning.benchmark_fixtures import write_planar_replay_fixture
            write_planar_replay_fixture(fixture)
        replay_path = str(fixture)

    launch_proc: subprocess.Popen | None = None
    runner_proc: subprocess.Popen | None = None
    if args.launch_stack:
        launch_proc, runner_proc = _launch_stack(
            strategy=args.strategy,
            replay_path=replay_path,
            seed=args.seed,
            fault_injection=args.fault_injection,
            inference_freq=args.inference_freq,
        )
        if not _wait_for_topics(args.startup_timeout_sec):
            raise RuntimeError('Timed out waiting for benchmark ROS topics')

    rclpy.init(args=[])
    collector = BenchmarkCollector()
    process = _find_process(args.process_name)
    if process is not None:
        process.cpu_percent(interval=None)

    fault_start_mono: Optional[float] = None
    completed_episodes = 0

    try:
        for episode in range(args.episodes):
            collector.reset_episode()
            if args.fault_injection and fault_start_mono is None:
                fault_start_mono = time.monotonic()

            ep_deadline = time.monotonic() + args.duration_sec
            while rclpy.ok() and time.monotonic() < ep_deadline:
                rclpy.spin_once(collector, timeout_sec=0.02)
                collector.record_sample(episode, process)

            completed_episodes += 1

        latencies = [
            row.latency_ms
            for row in collector.timeseries
            if row.latency_ms is not None
        ]
        cpu_values = [row.cpu_percent for row in collector.timeseries if row.cpu_percent is not None]
        rss_values = [row.rss_mb for row in collector.timeseries if row.rss_mb is not None]

        alarm_latency = _health_alarm_latency_ms(collector.health_events, fault_start_mono)
        health_within_1s = True
        if args.fault_injection:
            health_within_1s = alarm_latency is not None and alarm_latency <= 1000.0

        summary = {
            'strategy': args.strategy,
            'episodes': args.episodes,
            'completed_episodes': completed_episodes,
            'max_latency_ms': round(max(latencies), 3) if latencies else 0.0,
            'mean_latency_ms': round(statistics.fmean(latencies), 3) if latencies else 0.0,
            'std_latency_ms': round(statistics.pstdev(latencies), 3) if len(latencies) > 1 else 0.0,
            'cpu_peak_percent': round(max(cpu_values), 3) if cpu_values else 0.0,
            'rss_peak_mb': round(max(rss_values), 3) if rss_values else 0.0,
            'health_alarm_detected_within_1s': health_within_1s,
            'health_alarm_latency_ms': round(alarm_latency, 3) if alarm_latency is not None else None,
            'seed': args.seed,
            'fault_injection': args.fault_injection,
            'replay_path': replay_path if args.strategy == 'replay' else '',
            'timeseries_rows': len(collector.timeseries),
            'health_events': len(collector.health_events),
        }

        _write_timeseries_csv(output_dir / 'benchmark_timeseries.csv', collector.timeseries)
        _write_health_csv(output_dir / 'system_health_events.csv', collector.health_events)
        summary_path = output_dir / 'benchmark_summary.json'
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True, ensure_ascii=False) + '\n',
            encoding='utf-8',
        )
        (output_dir / 'benchmark_report.html').write_text(
            _render_html(summary, output_dir),
            encoding='utf-8',
        )
        return summary
    finally:
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        _stop_process(runner_proc)
        _stop_process(launch_proc)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Benchmark PolicyRunner system metrics.')
    parser.add_argument('--strategy', choices=['replay', 'sine_wave'], default='replay')
    parser.add_argument('--episodes', type=int, default=100)
    parser.add_argument('--duration-sec', type=float, default=10.0)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--replay-path', default='')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--fault-injection', action='store_true')
    parser.add_argument('--process-name', default='policy_runner')
    parser.add_argument('--inference-freq', type=int, default=20)
    parser.add_argument('--launch-stack', action='store_true', help='Launch bridge+monitor and policy_runner')
    parser.add_argument('--startup-timeout-sec', type=float, default=25.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    if psutil is None:
        print('psutil is required for benchmark_system.py. Install with: pip install psutil', file=sys.stderr)
        return 1

    args = build_parser().parse_args(argv)
    if args.episodes <= 0 or args.duration_sec <= 0:
        print('episodes and duration-sec must be positive', file=sys.stderr)
        return 1

    try:
        summary = run_benchmark(args)
    except Exception as exc:  # noqa: BLE001 - benchmark CLI reports failure and exits non-zero
        print(f'[benchmark_system] failed: {exc}', file=sys.stderr)
        return 1

    print(json.dumps(summary, indent=2, ensure_ascii=False))

    if summary['completed_episodes'] != args.episodes:
        print('[benchmark_system] incomplete episodes', file=sys.stderr)
        return 1
    if args.fault_injection and not summary['health_alarm_detected_within_1s']:
        print('[benchmark_system] fault injection health alarm not detected within 1s', file=sys.stderr)
        return 1
    if summary['timeseries_rows'] == 0:
        print('[benchmark_system] no timeseries samples collected', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
