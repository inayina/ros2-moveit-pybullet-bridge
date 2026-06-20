#!/usr/bin/env python3
"""Verify FR-HOC WebSocket, commands, and report export behavior."""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import statistics
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import websockets


@dataclass
class FrameStats:
    count: int = 0
    receive_times: list[float] = field(default_factory=list)
    latencies_ms: list[float] = field(default_factory=list)

    def add(self, frame: dict[str, Any], received_at: float) -> None:
        self.count += 1
        self.receive_times.append(received_at)
        stamp = frame.get('timestamp')
        if isinstance(stamp, dict):
            sent_at = float(stamp.get('sec', 0)) + float(stamp.get('nanosec', 0)) * 1e-9
            if sent_at > 0:
                self.latencies_ms.append((received_at - sent_at) * 1000.0)

    def summary(self) -> dict[str, Any]:
        hz = 0.0
        if len(self.receive_times) >= 2:
            duration = self.receive_times[-1] - self.receive_times[0]
            if duration > 0:
                hz = (len(self.receive_times) - 1) / duration
        return {
            'count': self.count,
            'mean_hz': round(hz, 3),
            'p95_latency_ms': round(_percentile(self.latencies_ms, 95), 3)
            if self.latencies_ms else None,
            'max_latency_ms': round(max(self.latencies_ms), 3) if self.latencies_ms else None,
        }


def _percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * pct / 100.0
    lo = int(rank)
    hi = min(lo + 1, len(ordered) - 1)
    frac = rank - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


class HocWsVerifier:
    def __init__(self, uri: str) -> None:
        self.uri = uri
        self.frames: list[dict[str, Any]] = []
        self.stats: dict[str, FrameStats] = {
            '/risk/status': FrameStats(),
            '/monitor/distribution_metrics': FrameStats(),
            '/monitor/tracking_error': FrameStats(),
        }
        self.latest_by_topic: dict[str, dict[str, Any]] = {}
        self.command_results: list[dict[str, Any]] = []
        self.report_ready: list[dict[str, Any]] = []
        self.system_states: list[dict[str, Any]] = []
        self.alert_events: list[dict[str, Any]] = []

    async def __aenter__(self):
        self.ws = await websockets.connect(self.uri, ping_interval=None)
        await self.ws.send(json.dumps({
            'type': 'subscribe',
            'topics': [
                '/risk/status',
                '/monitor/distribution_metrics',
                '/monitor/tracking_error',
            ],
        }))
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self.ws.close()

    async def recv_until(self, predicate, timeout_sec: float) -> dict[str, Any] | None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                raw = await asyncio.wait_for(self.ws.recv(), timeout=deadline - time.monotonic())
            except asyncio.TimeoutError:
                return None
            frame = json.loads(raw)
            self._record(frame)
            if predicate(frame):
                return frame
        return None

    async def collect(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            try:
                raw = await asyncio.wait_for(self.ws.recv(), timeout=deadline - time.monotonic())
            except asyncio.TimeoutError:
                break
            self._record(json.loads(raw))

    async def command(self, action: str, params: dict[str, Any] | None = None, timeout_sec: float = 5.0) -> dict[str, Any]:
        sent_at = time.monotonic()
        await self.ws.send(json.dumps({
            'type': 'command',
            'action': action,
            'params': params or {},
        }))

        def _matches(frame: dict[str, Any]) -> bool:
            return frame.get('type') == 'command_result' and frame.get('action') == action

        frame = await self.recv_until(_matches, timeout_sec)
        if frame is None:
            return {
                'action': action,
                'success': False,
                'latency_ms': None,
                'message': 'command_result timeout',
            }
        frame['latency_ms'] = round((time.monotonic() - sent_at) * 1000.0, 3)
        return frame

    def _record(self, frame: dict[str, Any]) -> None:
        self.frames.append(frame)
        kind = frame.get('type')
        if kind == 'data':
            topic = frame.get('topic')
            if topic in self.stats:
                self.stats[topic].add(frame, time.time())
                self.latest_by_topic[topic] = frame.get('payload', {})
        elif kind == 'command_result':
            self.command_results.append(frame)
        elif kind == 'report_ready':
            self.report_ready.append(frame)
        elif kind == 'system_state':
            self.system_states.append(frame)
        elif kind == 'alert_event':
            self.alert_events.append(frame)


def _read_report(path: str) -> dict[str, Any]:
    report = Path(path)
    if not report.is_file():
        return {'exists': False, 'path': path}
    if report.suffix == '.json':
        data = json.loads(report.read_text(encoding='utf-8'))
        return {
            'exists': True,
            'path': path,
            'risk_timeline_len': len(data.get('risk_timeline', [])),
            'metrics_timeline_len': len(data.get('metrics_timeline', [])),
            'has_summary': bool(data.get('summary')),
        }
    if report.suffix == '.csv':
        rows = list(csv.reader(report.read_text(encoding='utf-8').splitlines()))
        header = rows[0] if rows else []
        return {
            'exists': True,
            'path': path,
            'rows': max(len(rows) - 1, 0),
            'has_risk_columns': 'risk_level' in header and 'composite_score' in header,
            'has_metric_columns': 'kl_mean' in header and 'mmd_stat' in header,
        }
    return {'exists': True, 'path': path}


async def _run(args) -> int:
    async with HocWsVerifier(args.websocket_uri) as hoc:
        await hoc.recv_until(lambda f: f.get('type') == 'subscribed', timeout_sec=5.0)
        await hoc.collect(args.sample_sec)

        randomization = await hoc.command('set_randomization', {
            'seed': 123,
            'strength': 0.35,
            'joint_damping_min': 0.02,
            'joint_damping_max': 0.08,
        })
        inject = await hoc.command('inject_shift', {
            'parameter': 'joint_damping',
            'delta_percent': 20.0,
            'duration': 4.0,
        })
        pause = await hoc.command('pause')
        e_stop = await hoc.command('e_stop')
        acknowledge = await hoc.command('acknowledge', {
            'operator_id': 'fr_hoc_verify',
            'comment': 'FR-HOC verification',
        })
        resume = await hoc.command('resume')

        await hoc.collect(2.0)

        json_path = str(args.report_dir / 'fr_hoc_verify.json')
        csv_path = str(args.report_dir / 'fr_hoc_verify.csv')
        export_json = await hoc.command('export_report', {
            'experiment_id': 'fr_hoc_verify',
            'format': 'json',
            'path': json_path,
        })
        export_csv = await hoc.command('export_report', {
            'experiment_id': 'fr_hoc_verify',
            'format': 'csv',
            'path': csv_path,
        })

        risk = hoc.latest_by_topic.get('/risk/status', {})
        metrics = hoc.latest_by_topic.get('/monitor/distribution_metrics', {})
        tracking = hoc.latest_by_topic.get('/monitor/tracking_error', {})
        stream_stats = {topic: stat.summary() for topic, stat in hoc.stats.items()}
        max_ws_latency = max(
            (item.get('max_latency_ms') or 0.0 for item in stream_stats.values()),
            default=0.0,
        )
        command_map = {
            'set_randomization': randomization,
            'inject_shift': inject,
            'pause': pause,
            'e_stop': e_stop,
            'acknowledge': acknowledge,
            'resume': resume,
            'export_json': export_json,
            'export_csv': export_csv,
        }
        control_latencies = [
            item.get('latency_ms')
            for item in (pause, e_stop, acknowledge, resume)
            if item.get('latency_ms') is not None
        ]
        payload = {
            'timestamp_unix': round(time.time(), 3),
            'criteria': {
                'websocket_latency_ms_max': args.ws_latency_threshold_ms,
                'push_frequency_hz_min': args.push_frequency_min_hz,
                'control_latency_ms_max': args.control_latency_threshold_ms,
            },
            'streams': stream_stats,
            'latest_payloads': {
                'risk_has_five_dim_radar': len(risk.get('attribution', [])) == 5,
                'risk_level': risk.get('level'),
                'primary_driver': risk.get('primary_driver'),
                'metrics_has_distribution_boxplot': bool(metrics.get('sim_position_median_per_joint'))
                and bool(metrics.get('real_position_median_per_joint')),
                'metrics_has_curves': all(
                    key in metrics
                    for key in ('kl_divergence_mean', 'wasserstein_mean', 'mmd_statistic')
                ),
                'tracking_error_joints': len(tracking.get('joint_names', [])),
            },
            'commands': command_map,
            'reports': {
                'json': _read_report(json_path),
                'csv': _read_report(csv_path),
            },
        }
        payload['passes'] = {
            'websocket_latency': max_ws_latency <= args.ws_latency_threshold_ms,
            'push_frequency': all(
                stream_stats[topic]['mean_hz'] >= args.push_frequency_min_hz
                for topic in ('/risk/status', '/monitor/distribution_metrics')
            ),
            'risk_radar_payload': payload['latest_payloads']['risk_has_five_dim_radar'],
            'distribution_payload': (
                payload['latest_payloads']['metrics_has_distribution_boxplot']
                and payload['latest_payloads']['metrics_has_curves']
            ),
            'pause_zoom_contract': bool(pause.get('success')),
            'control_latency': all(
                value <= args.control_latency_threshold_ms
                for value in control_latencies
            ),
            'parameter_adjustment': bool(randomization.get('success')) and bool(inject.get('success')),
            'report_export': (
                bool(export_json.get('success'))
                and bool(export_csv.get('success'))
                and payload['reports']['json'].get('risk_timeline_len', 0) > 0
                and payload['reports']['json'].get('metrics_timeline_len', 0) > 0
                and payload['reports']['csv'].get('has_risk_columns')
                and payload['reports']['csv'].get('has_metric_columns')
            ),
        }
        payload['overall_passes'] = all(payload['passes'].values())
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        if not payload['overall_passes']:
            return 1
        return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Verify FR-HOC HOC console behavior.')
    parser.add_argument('--websocket-uri', default='ws://127.0.0.1:8765')
    parser.add_argument('--output', type=Path, default=Path('docs/samples/hoc-console-metrics.json'))
    parser.add_argument('--report-dir', type=Path, default=Path('docs/samples/hoc-verification-reports'))
    parser.add_argument('--sample-sec', type=float, default=6.0)
    parser.add_argument('--ws-latency-threshold-ms', type=float, default=200.0)
    parser.add_argument('--push-frequency-min-hz', type=float, default=4.5)
    parser.add_argument('--control-latency-threshold-ms', type=float, default=100.0)
    args = parser.parse_args(argv)
    args.report_dir.mkdir(parents=True, exist_ok=True)
    return asyncio.run(_run(args))


if __name__ == '__main__':
    raise SystemExit(main())
