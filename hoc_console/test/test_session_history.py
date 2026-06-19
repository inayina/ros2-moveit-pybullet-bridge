"""Tests for SessionHistory aggregation."""

from hoc_console.hoc_server import SessionHistory


def test_session_history_records_w1_mean():
    history = SessionHistory()
    history.record_metrics({
        'kl_divergence_mean': 0.1,
        'wasserstein_mean': 0.04,
        'mmd_statistic': 0.02,
        'shift_detected': False,
    })
    history.record_metrics({
        'kl_divergence_mean': 0.2,
        'wasserstein_mean': 0.08,
        'mmd_statistic': 0.03,
        'shift_detected': True,
    })
    summary = history.summary()
    assert summary['mean_w1'] == 0.06
    assert summary['max_w1'] == 0.08
    assert len(history.metrics_timeline) == 2
    assert history.metrics_timeline[-1]['w1_mean'] == 0.08
