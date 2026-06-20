"""Sliding-window planning success/failure statistics for risk_engine."""

from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass, field

from builtin_interfaces.msg import Time as BuiltinTime
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


@dataclass
class PlanningStatsCollector:
    """Track recent manipulation / planning outcomes."""

    window_size: int = 20
    _results: deque[tuple[bool, str, str]] = field(default_factory=deque, repr=False)

    def __post_init__(self) -> None:
        self._results = deque(maxlen=max(int(self.window_size), 1))

    def record(self, *, success: bool, message: str = '', action: str = '') -> None:
        self._results.append((bool(success), str(message), str(action)))

    @property
    def sample_count(self) -> int:
        return len(self._results)

    def failure_rate(self) -> float:
        if not self._results:
            return 0.0
        failures = sum(1 for ok, _, _ in self._results if not ok)
        return failures / len(self._results)

    def last_error(self) -> str:
        for ok, message, action in reversed(self._results):
            if not ok:
                prefix = f'{action}: ' if action else ''
                return f'{prefix}{message}'.strip(': ')
        return ''

    def to_diagnostic_array(self, stamp: BuiltinTime) -> DiagnosticArray:
        rate = self.failure_rate()
        array = DiagnosticArray()
        array.header.stamp = stamp
        status = DiagnosticStatus()
        status.name = 'planning_stats'
        status.hardware_id = 'risk_engine'
        if rate >= 0.5:
            status.level = DiagnosticStatus.ERROR
        elif rate >= 0.1:
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        payload = {
            'failure_rate': round(rate, 4),
            'window': self.window_size,
            'sample_count': self.sample_count,
            'last_error': self.last_error(),
        }
        status.message = json.dumps(payload, separators=(',', ':'))
        status.values = [
            KeyValue(key='failure_rate', value=f'{rate:.4f}'),
            KeyValue(key='sample_count', value=str(self.sample_count)),
        ]
        array.status.append(status)
        return array
