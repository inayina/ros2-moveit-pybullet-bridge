"""Tests for M3 distribution validity semantics."""

from dist_monitor.metric_validity import evaluate_metric_validity
import pytest


@pytest.mark.parametrize(
    ('kwargs', 'validity', 'reason'),
    [
        ({'sources_fresh': False}, 'STALE', 'source_stale'),
        ({'sources_seen': False}, 'UNAVAILABLE', 'no_data'),
        (
            {'aligned_samples': 4},
            'WARMING_UP',
            'insufficient_aligned_samples',
        ),
        ({'baseline_ready': False}, 'WARMING_UP', 'baseline_warming_up'),
        ({'calibration_id': ''}, 'UNAVAILABLE', 'calibration_missing'),
        (
            {'reference_source': 'lerobot'},
            'UNAVAILABLE',
            'reference_not_runtime_calibration',
        ),
    ],
)
def test_invalid_states_never_claim_metric_valid(kwargs, validity, reason):
    values = {
        'aligned_samples': 50,
        'min_samples': 50,
        'baseline_ready': True,
        'calibration_id': 'panda_same_scene_20260726',
        'reference_source': 'topic',
        'sources_fresh': True,
        'sources_seen': True,
    }
    values.update(kwargs)
    result = evaluate_metric_validity(**values)
    assert result.validity == validity
    assert result.reason_code == reason
    assert result.metric_valid is False


def test_same_scene_calibration_is_valid() -> None:
    result = evaluate_metric_validity(
        aligned_samples=50,
        min_samples=50,
        baseline_ready=True,
        calibration_id='panda_same_scene_20260726',
        reference_source='topic',
        sources_fresh=True,
        sources_seen=True,
    )
    assert result.validity == 'VALID'
    assert result.metric_valid is True
