"""Unit tests for risk aggregation logic."""

import pytest

from risk_engine.aggregator import (
    AggregatedRisk,
    clip01,
    RiskAggregator,
    RiskWeights,
    score_to_level,
)


def test_clip01():
    assert clip01(-1.0) == 0.0
    assert clip01(0.5) == 0.5
    assert clip01(2.0) == 1.0


def test_score_to_level():
    thresholds = (0.25, 0.50, 0.75)
    assert score_to_level(0.0, thresholds) == 0
    assert score_to_level(0.24, thresholds) == 0
    assert score_to_level(0.25, thresholds) == 1
    assert score_to_level(0.60, thresholds) == 2
    assert score_to_level(0.90, thresholds) == 3


def test_aggregate_low_risk_level_zero():
    agg = RiskAggregator()
    result = agg.aggregate({})
    assert isinstance(result, AggregatedRisk)
    assert result.level == 0
    assert result.composite_score == 0.0


def test_aggregate_high_distribution_shift():
    agg = RiskAggregator()
    result = agg.aggregate({'distribution_shift': 1.0})
    assert result.level == 1
    assert result.composite_score == 0.30
    assert result.primary_driver == 'distribution_shift'
    assert '域随机化' in result.recommendation


def test_aggregate_all_dimensions_critical():
    agg = RiskAggregator()
    result = agg.aggregate({dim: 1.0 for dim in (
        'distribution_shift',
        'tracking_error',
        'dynamics_anomaly',
        'comm_health',
        'planning_failure',
        'resource_pressure',
    )})
    assert result.level == 3
    assert result.composite_score == 1.0


def test_aggregate_missing_dimensions_default_zero():
    agg = RiskAggregator(weights=RiskWeights())
    result = agg.aggregate({'tracking_error': 0.4})
    assert len(result.dimensions) == 6
    assert result.primary_driver == 'tracking_error'
    tracking = next(d for d in result.dimensions if d.dimension == 'tracking_error')
    assert tracking.raw_score == 0.4
    assert tracking.weighted_score == 0.4 * 0.25


def test_resource_pressure_is_exposed_as_sixth_dimension():
    result = RiskAggregator().aggregate({'resource_pressure': 1.0})
    assert result.primary_driver == 'resource_pressure'
    assert result.composite_score == 0.10
    assert len(result.dimensions) == 6


def test_invalid_sources_are_excluded_and_weights_are_renormalized():
    status = {
        name: {'valid': False}
        for name in (
            'distribution_shift', 'dynamics_anomaly', 'comm_health',
            'planning_failure', 'resource_pressure',
        )
    }
    status['distribution_shift'].update({
        'validity': 'UNAVAILABLE',
        'reason_code': 'calibration_missing',
        'provenance': '/monitor/distribution_metrics',
    })
    status['tracking_error'] = {
        'valid': True,
        'provenance': '/monitor/tracking_error',
    }
    result = RiskAggregator().aggregate(
        {'distribution_shift': 1.0, 'tracking_error': 0.5}, status
    )
    assert result.composite_score == pytest.approx(0.5)
    assert result.primary_driver == 'tracking_error'
    assert result.validity == 'DEGRADED'
    distribution = next(
        item for item in result.dimensions
        if item.dimension == 'distribution_shift'
    )
    assert distribution.weight == 0.0
    assert distribution.reason_code == 'calibration_missing'


def test_no_valid_sources_is_unavailable_not_green() -> None:
    status = {
        name: {'valid': False}
        for name in (
            'distribution_shift', 'tracking_error', 'dynamics_anomaly',
            'comm_health', 'planning_failure', 'resource_pressure',
        )
    }
    result = RiskAggregator().aggregate({}, status)
    assert result.validity == 'UNAVAILABLE'
    assert result.reason_code == 'no_valid_risk_sources'
    assert result.primary_driver == ''
