"""Topic timing health monitor for Sim/Real joint state streams."""

from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass, field

import numpy as np
from builtin_interfaces.msg import Time as BuiltinTime
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


@dataclass
class _IntervalSample:
    interval_sec: float
    jitter_ms: float
    is_gap: bool


@dataclass
class _TopicCommStats:
    topic: str
    expected_hz: float
    gap_multiplier: float
    ewma_alpha: float
    latency_threshold_ms: float
    max_samples: int = 100
    _last_recv_mono: float | None = field(default=None, repr=False)
    _samples: deque = field(default_factory=deque, repr=False)
    latency_ewma_ms: float = 0.0

    def __post_init__(self) -> None:
        self._samples = deque(maxlen=self.max_samples)

    def record(self, recv_mono: float) -> None:
        expected_interval = 1.0 / self.expected_hz if self.expected_hz > 0 else 0.01
        if self._last_recv_mono is not None:
            interval = recv_mono - self._last_recv_mono
            if interval <= 0:
                return
            jitter_ms = abs(interval - expected_interval) * 1000.0
            is_gap = interval > self.gap_multiplier * expected_interval
            self._samples.append(
                _IntervalSample(interval_sec=interval, jitter_ms=jitter_ms, is_gap=is_gap),
            )
            self.latency_ewma_ms = (
                self.ewma_alpha * jitter_ms + (1.0 - self.ewma_alpha) * self.latency_ewma_ms
            )
        self._last_recv_mono = recv_mono

    @property
    def measured_hz(self) -> float:
        if not self._samples:
            return self.expected_hz
        mean_interval = float(np.mean([s.interval_sec for s in self._samples]))
        return 1.0 / mean_interval if mean_interval > 0 else 0.0

    @property
    def gap_ratio(self) -> float:
        if not self._samples:
            return 0.0
        gaps = sum(1 for s in self._samples if s.is_gap)
        return gaps / len(self._samples)

    def score(self) -> float:
        if not self._samples:
            return 0.0
        hz_error = (
            abs(self.measured_hz - self.expected_hz) / self.expected_hz
            if self.expected_hz > 0
            else 0.0
        )
        latency_norm = min(self.latency_ewma_ms / self.latency_threshold_ms, 1.0)
        return float(np.clip(
            0.4 * latency_norm + 0.3 * hz_error + 0.3 * self.gap_ratio,
            0.0,
            1.0,
        ))


@dataclass
class CommHealthMonitor:
    """Track publish timing for bridge joint-state topics."""

    expected_sim_hz: float = 100.0
    expected_real_hz: float = 100.0
    ewma_alpha: float = 0.2
    gap_multiplier: float = 2.0
    latency_threshold_ms: float = 100.0
    max_samples: int = 100

    def __post_init__(self) -> None:
        self._topics: dict[str, _TopicCommStats] = {
            '/bridge/sim/joint_states': _TopicCommStats(
                topic='/bridge/sim/joint_states',
                expected_hz=self.expected_sim_hz,
                gap_multiplier=self.gap_multiplier,
                ewma_alpha=self.ewma_alpha,
                latency_threshold_ms=self.latency_threshold_ms,
                max_samples=self.max_samples,
            ),
            '/bridge/real/joint_states': _TopicCommStats(
                topic='/bridge/real/joint_states',
                expected_hz=self.expected_real_hz,
                gap_multiplier=self.gap_multiplier,
                ewma_alpha=self.ewma_alpha,
                latency_threshold_ms=self.latency_threshold_ms,
                max_samples=self.max_samples,
            ),
        }

    def record(self, topic: str, recv_mono: float) -> None:
        stats = self._topics.get(topic)
        if stats is not None:
            stats.record(recv_mono)

    def aggregate_score(self) -> float:
        scores = [stats.score() for stats in self._topics.values() if stats._samples]
        if not scores:
            return 0.0
        return float(max(scores))

    def to_diagnostic_array(self, stamp: BuiltinTime) -> DiagnosticArray:
        array = DiagnosticArray()
        array.header.stamp = stamp
        for stats in self._topics.values():
            status = DiagnosticStatus()
            status.name = stats.topic
            status.hardware_id = 'dist_monitor'
            score = stats.score()
            if score >= 0.75:
                status.level = DiagnosticStatus.ERROR
            elif score >= 0.35:
                status.level = DiagnosticStatus.WARN
            else:
                status.level = DiagnosticStatus.OK
            payload = {
                'topic': stats.topic,
                'expected_hz': stats.expected_hz,
                'measured_hz': round(stats.measured_hz, 2),
                'latency_ewma_ms': round(stats.latency_ewma_ms, 3),
                'gap_ratio': round(stats.gap_ratio, 4),
                'score': round(score, 4),
            }
            status.message = json.dumps(payload, separators=(',', ':'))
            status.values = [
                KeyValue(key='score', value=f'{score:.4f}'),
                KeyValue(key='measured_hz', value=f'{stats.measured_hz:.2f}'),
            ]
            array.status.append(status)

        summary = DiagnosticStatus()
        summary.name = 'comm_health_aggregate'
        summary.hardware_id = 'dist_monitor'
        agg = self.aggregate_score()
        summary.level = (
            DiagnosticStatus.ERROR if agg >= 0.75
            else DiagnosticStatus.WARN if agg >= 0.35
            else DiagnosticStatus.OK
        )
        summary.message = json.dumps({'comm_health_score': round(agg, 4)})
        summary.values = [KeyValue(key='comm_health_score', value=f'{agg:.4f}')]
        array.status.append(summary)
        return array
