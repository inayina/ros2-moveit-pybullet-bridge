"""Tests for M3 four-lane HOC freshness and trace behavior."""

from hoc_console.runtime_lanes import RuntimeLaneStore


def test_missing_lanes_are_unavailable_not_green() -> None:
    snapshot = RuntimeLaneStore().snapshot(now_monotonic=10.0)
    assert all(
        lane['validity'] == 'UNAVAILABLE'
        for lane in snapshot['lanes'].values()
    )


def test_lane_becomes_stale_and_preserves_source_validity() -> None:
    store = RuntimeLaneStore(stale_after_sec=1.0)
    store.update(
        'brain',
        {'lane': 'brain', 'validity': 'VALID', 'reason_code': 'none'},
        received_monotonic=1.0,
    )
    lane = store.snapshot(now_monotonic=2.1)['lanes']['brain']
    assert lane['validity'] == 'STALE'
    assert lane['source_validity'] == 'VALID'
    assert lane['reason_code'] == 'source_timeout'


def test_regressed_execution_sequence_is_rejected() -> None:
    store = RuntimeLaneStore()
    first = {'episode_id': 'ep', 'command_sequence': 5}
    assert store.update('execution', first, received_monotonic=1.0)
    assert not store.update(
        'execution',
        {'episode_id': 'ep', 'command_sequence': 4},
        received_monotonic=2.0,
    )
    lane = store.snapshot(now_monotonic=2.0)['lanes']['execution']
    assert lane['command_sequence'] == 5


def test_trace_mismatch_is_explicit() -> None:
    store = RuntimeLaneStore()
    store.update(
        'execution',
        {'episode_id': 'ep', 'command_sequence': 0, 'trace_run_id': 'a'},
        received_monotonic=1.0,
    )
    store.update(
        'task_gt', {'trace_run_id': 'b'}, received_monotonic=1.0
    )
    assert store.snapshot(now_monotonic=1.0)['correlation'][
        'trace_consistent'
    ] is False
