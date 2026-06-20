#!/usr/bin/env python3
"""Verify FR-MON distribution monitoring evidence on a running demo."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from bridge_monitor_msgs.msg import DistributionMetrics
from bridge_monitor_msgs.srv import InjectShift
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_srvs.srv import Trigger


@dataclass
class TrialResult:
    index: int
    success: bool
    latency_sec: float | None
    max_kl: float
    max_w1: float
    max_mmd: float
    min_mmd_p: float
    method: str
    message: str


def _stamp_sec(msg) -> float:
    return float(msg.sec) + float(msg.nanosec) * 1e-9


def _percent_jitter(times: list[float], expected_hz: float) -> dict:
    if len(times) < 2:
        return {
            'samples': len(times),
            'mean_hz': 0.0,
            'mean_error_pct': 100.0,
            'jitter_pct': 100.0,
            'max_gap_ms': math.nan,
        }
    intervals = [b - a for a, b in zip(times, times[1:]) if b > a]
    mean_interval = statistics.fmean(intervals)
    mean_hz = 1.0 / mean_interval
    expected_period = 1.0 / expected_hz
    stdev = statistics.pstdev(intervals) if len(intervals) > 1 else 0.0
    return {
        'samples': len(times),
        'mean_hz': round(mean_hz, 3),
        'mean_error_pct': round(abs(mean_hz - expected_hz) / expected_hz * 100.0, 3),
        'jitter_pct': round(stdev / expected_period * 100.0, 3),
        'max_gap_ms': round(max(intervals) * 1000.0, 3),
    }


class MonitorMetricsChecker(Node):
    def __init__(self) -> None:
        super().__init__('monitor_metrics_checker')
        self.metrics: list[DistributionMetrics] = []
        self._reset_baseline = self.create_client(Trigger, '/monitor/reset_baseline')
        self._inject_shift = self.create_client(InjectShift, '/bridge/inject_shift')
        self._set_params = self.create_client(SetParameters, '/dist_monitor/set_parameters')
        self.create_subscription(
            DistributionMetrics,
            '/monitor/distribution_metrics',
            self._on_metrics,
            10,
        )

    def _on_metrics(self, msg: DistributionMetrics) -> None:
        self.metrics.append(msg)

    def wait_for_metrics(self, *, min_samples: int, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if len(self.metrics) >= min_samples:
                return True
        return False

    def spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def reset_baseline(self, timeout_sec: float) -> tuple[bool, str]:
        if not self._reset_baseline.wait_for_service(timeout_sec=timeout_sec):
            return False, '/monitor/reset_baseline unavailable'
        future = self._reset_baseline.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return False, 'reset_baseline timed out'
        response = future.result()
        return bool(response.success), response.message

    def set_thresholds(
        self,
        *,
        kl: float,
        w1: float,
        mmd: float,
        timeout_sec: float,
    ) -> tuple[bool, str]:
        if not self._set_params.wait_for_service(timeout_sec=timeout_sec):
            return False, '/dist_monitor/set_parameters unavailable'

        request = SetParameters.Request()
        request.parameters = [
            self._double_param('kl_threshold_mean', kl),
            self._double_param('w1_threshold_mean', w1),
            self._double_param('mmd_threshold', mmd),
        ]
        future = self._set_params.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return False, 'threshold update timed out'
        results = future.result().results
        ok = all(item.successful for item in results)
        reason = '; '.join(item.reason for item in results if item.reason)
        return ok, reason or f'kl={kl}, w1={w1}, mmd={mmd}'

    @staticmethod
    def _double_param(name: str, value: float) -> Parameter:
        param = Parameter()
        param.name = name
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        return param

    def inject_shift(
        self,
        *,
        parameter_name: str,
        delta_percent: float,
        duration_sec: float,
        timeout_sec: float,
    ) -> tuple[bool, str]:
        if not self._inject_shift.wait_for_service(timeout_sec=timeout_sec):
            return False, '/bridge/inject_shift unavailable'
        request = InjectShift.Request()
        request.parameter_name = parameter_name
        request.delta_percent = delta_percent
        request.duration_sec = duration_sec
        future = self._inject_shift.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return False, 'inject_shift timed out'
        response = future.result()
        return bool(response.success), response.message

    def run_injection_trial(
        self,
        *,
        index: int,
        parameter_name: str,
        delta_percent: float,
        duration_sec: float,
        detection_timeout_sec: float,
    ) -> TrialResult:
        start_idx = len(self.metrics)
        ok, message = self.inject_shift(
            parameter_name=parameter_name,
            delta_percent=delta_percent,
            duration_sec=duration_sec,
            timeout_sec=5.0,
        )
        if not ok:
            return TrialResult(index, False, None, 0.0, 0.0, 0.0, 1.0, 'none', message)

        start = time.monotonic()
        detected_at: float | None = None
        max_kl = 0.0
        max_w1 = 0.0
        max_mmd = 0.0
        min_p = 1.0
        methods: set[str] = set()
        deadline = start + detection_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            for item in self.metrics[start_idx:]:
                max_kl = max(max_kl, float(item.kl_divergence_mean))
                max_w1 = max(max_w1, float(item.wasserstein_mean))
                max_mmd = max(max_mmd, float(item.mmd_statistic))
                min_p = min(min_p, float(item.mmd_p_value))
                if item.detection_method and item.detection_method != 'none':
                    methods.add(item.detection_method)
                if item.shift_detected and detected_at is None:
                    detected_at = time.monotonic()
            if detected_at is not None:
                break

        return TrialResult(
            index=index,
            success=detected_at is not None,
            latency_sec=None if detected_at is None else round(detected_at - start, 3),
            max_kl=round(max_kl, 6),
            max_w1=round(max_w1, 6),
            max_mmd=round(max_mmd, 6),
            min_mmd_p=round(min_p, 6),
            method='+'.join(sorted(methods)) if methods else 'none',
            message=message,
        )


def _metrics_to_row(msg: DistributionMetrics, t0: float) -> dict:
    stamp = _stamp_sec(msg.header.stamp)
    return {
        't': round(stamp - t0, 6),
        'sample_count_sim': int(msg.sample_count_sim),
        'sample_count_real': int(msg.sample_count_real),
        'kl_mean': float(msg.kl_divergence_mean),
        'w1_mean': float(msg.wasserstein_mean),
        'mmd_statistic': float(msg.mmd_statistic),
        'mmd_p_value': float(msg.mmd_p_value),
        'mmd_threshold': float(msg.mmd_threshold),
        'w1_threshold': float(msg.w1_threshold),
        'shift_detected': bool(msg.shift_detected),
        'shift_detected_w1': bool(msg.shift_detected_w1),
        'detection_method': msg.detection_method,
    }


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Verify FR-MON distribution monitor evidence.')
    parser.add_argument('--output', type=Path, default=Path('docs/samples/monitor-metrics.json'))
    parser.add_argument('--timeline-csv', type=Path, default=Path('docs/samples/monitor-metrics-timeline.csv'))
    parser.add_argument('--warmup-sec', type=float, default=7.0)
    parser.add_argument('--expected-hz', type=float, default=10.0)
    parser.add_argument('--hz-tolerance-pct', type=float, default=10.0)
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--shift-parameter', default='joint_damping')
    parser.add_argument('--shift-delta-percent', type=float, default=20.0)
    parser.add_argument('--shift-duration-sec', type=float, default=4.0)
    parser.add_argument('--detection-timeout-sec', type=float, default=8.0)
    parser.add_argument('--kl-threshold', type=float, default=0.02)
    parser.add_argument('--w1-threshold', type=float, default=0.01)
    parser.add_argument('--mmd-threshold', type=float, default=0.005)
    args = parser.parse_args(argv)

    rclpy.init(args=[])
    node = MonitorMetricsChecker()
    try:
        if not node.wait_for_metrics(min_samples=5, timeout_sec=20.0):
            print('[FAIL] /monitor/distribution_metrics not available', file=sys.stderr)
            return 1

        node.spin_for(args.warmup_sec)
        reset_ok, reset_message = node.reset_baseline(timeout_sec=5.0)
        thresholds_ok, thresholds_message = node.set_thresholds(
            kl=args.kl_threshold,
            w1=args.w1_threshold,
            mmd=args.mmd_threshold,
            timeout_sec=5.0,
        )

        trials: list[TrialResult] = []
        for idx in range(args.trials):
            node.get_logger().info(f'FR-MON inject_shift trial {idx + 1}/{args.trials}')
            trials.append(
                node.run_injection_trial(
                    index=idx + 1,
                    parameter_name=args.shift_parameter,
                    delta_percent=args.shift_delta_percent,
                    duration_sec=args.shift_duration_sec,
                    detection_timeout_sec=args.detection_timeout_sec,
                ),
            )
            node.spin_for(0.5)

        times = [_stamp_sec(item.header.stamp) for item in node.metrics]
        hz = _percent_jitter(times, args.expected_hz)
        t0 = times[0] if times else 0.0
        rows = [_metrics_to_row(item, t0) for item in node.metrics]
        if rows:
            _write_csv(args.timeline_csv, rows)

        detections = sum(1 for item in trials if item.success)
        detection_rate = detections / len(trials) if trials else 0.0
        latest = node.metrics[-1] if node.metrics else DistributionMetrics()
        payload = {
            'timestamp_unix': round(time.time(), 3),
            'criteria': {
                'update_frequency_hz': args.expected_hz,
                'frequency_tolerance_pct': args.hz_tolerance_pct,
                'window_duration_sec_expected': 5.0,
                'injection_delta_percent': args.shift_delta_percent,
                'detection_rate_min': 0.90,
            },
            'frequency': {
                **hz,
                'passes': hz['mean_error_pct'] <= args.hz_tolerance_pct,
            },
            'latest_metrics': {
                'window_duration_sec': float(latest.window_duration_sec),
                'sample_count_sim': int(latest.sample_count_sim),
                'sample_count_real': int(latest.sample_count_real),
                'kl_mean': float(latest.kl_divergence_mean),
                'w1_mean': float(latest.wasserstein_mean),
                'mmd_statistic': float(latest.mmd_statistic),
                'mmd_p_value': float(latest.mmd_p_value),
                'mmd_threshold': float(latest.mmd_threshold),
                'shift_detected': bool(latest.shift_detected),
                'detection_method': latest.detection_method,
            },
            'threshold_hot_reload': {
                'passes': thresholds_ok,
                'message': thresholds_message,
                'values': {
                    'kl_threshold_mean': args.kl_threshold,
                    'w1_threshold_mean': args.w1_threshold,
                    'mmd_threshold': args.mmd_threshold,
                },
            },
            'baseline_reset': {
                'passes': reset_ok,
                'message': reset_message,
            },
            'injection_trials': [
                {
                    'index': item.index,
                    'success': item.success,
                    'latency_sec': item.latency_sec,
                    'max_kl': item.max_kl,
                    'max_w1': item.max_w1,
                    'max_mmd': item.max_mmd,
                    'min_mmd_p': item.min_mmd_p,
                    'method': item.method,
                    'message': item.message,
                }
                for item in trials
            ],
            'detection_rate': round(detection_rate, 3),
            'timeline_csv': str(args.timeline_csv),
            'overall_passes': (
                hz['mean_error_pct'] <= args.hz_tolerance_pct
                and reset_ok
                and thresholds_ok
                and detection_rate >= 0.90
                and bool(rows)
            ),
        }

        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        if not payload['overall_passes']:
            print(f'[FAIL] Monitor metrics criteria failed: {args.output}', file=sys.stderr)
            return 1
        print(f'[PASS] Monitor metrics criteria met: {args.output}')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
