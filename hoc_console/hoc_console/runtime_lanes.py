"""Freshness, ordering, and correlation for M3 HOC runtime lanes."""

from __future__ import annotations

from copy import deepcopy
import time


LANES = ('brain', 'execution', 'safety', 'task_gt')


class RuntimeLaneStore:
    """Keep four independent lanes without inventing missing data."""

    def __init__(self, stale_after_sec: float = 1.0) -> None:
        if stale_after_sec <= 0.0:
            raise ValueError('stale_after_sec must be positive')
        self.stale_after_sec = float(stale_after_sec)
        self._payloads: dict[str, dict] = {}
        self._received: dict[str, float] = {}
        self._last_execution_sequence: dict[str, int] = {}

    def update(
        self,
        lane: str,
        payload: dict,
        *,
        received_monotonic: float | None = None,
    ) -> bool:
        if lane not in LANES:
            raise ValueError(f'unknown runtime lane: {lane}')
        if lane == 'execution':
            episode = str(payload.get('episode_id', ''))
            sequence = int(payload.get('command_sequence', -1))
            previous = self._last_execution_sequence.get(episode)
            if previous is not None and sequence <= previous:
                return False
            self._last_execution_sequence[episode] = sequence
        self._payloads[lane] = deepcopy(payload)
        self._received[lane] = (
            time.monotonic()
            if received_monotonic is None else received_monotonic
        )
        return True

    def snapshot(self, *, now_monotonic: float | None = None) -> dict:
        now = time.monotonic() if now_monotonic is None else now_monotonic
        lanes: dict[str, dict] = {}
        for lane in LANES:
            payload = deepcopy(self._payloads.get(lane, {}))
            if not payload:
                lanes[lane] = {
                    'lane': lane,
                    'validity': 'UNAVAILABLE',
                    'reason_code': 'no_data',
                    'age_ms': None,
                }
                continue
            age = max(0.0, now - self._received[lane])
            payload['age_ms'] = age * 1000.0
            if age > self.stale_after_sec:
                payload['source_validity'] = payload.get(
                    'validity', 'UNAVAILABLE'
                )
                payload['validity'] = 'STALE'
                payload['reason_code'] = 'source_timeout'
            lanes[lane] = payload
        return {
            'lanes': lanes,
            'correlation': self._correlation(lanes),
        }

    @staticmethod
    def _correlation(lanes: dict[str, dict]) -> dict:
        trace_ids = {
            payload.get('trace_run_id')
            for payload in lanes.values()
            if payload.get('trace_run_id')
        }
        execution = lanes['execution']
        task = lanes['task_gt']
        return {
            'trace_run_ids': sorted(trace_ids),
            'trace_consistent': len(trace_ids) <= 1,
            'execution_event_id': execution.get('event_id'),
            'execution_parent_event_id': execution.get('parent_event_id'),
            'task_event_id': task.get('event_id'),
            'task_parent_event_id': task.get('parent_event_id'),
        }
