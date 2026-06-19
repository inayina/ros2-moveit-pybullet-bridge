"""Unit tests for risk aggregation logic."""

from risk_engine.aggregator import (
    AggregatedRisk,
    RiskAggregator,
    RiskWeights,
    clip01,
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
    assert result.composite_score == 0.35
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
    )})
    assert result.level == 3
    assert result.composite_score == 1.0


def test_aggregate_missing_dimensions_default_zero():
    agg = RiskAggregator(weights=RiskWeights())
    result = agg.aggregate({'tracking_error': 0.4})
    assert len(result.dimensions) == 5
    assert result.primary_driver == 'tracking_error'
    tracking = next(d for d in result.dimensions if d.dimension == 'tracking_error')
    assert tracking.raw_score == 0.4
    assert tracking.weighted_score == 0.4 * 0.25
