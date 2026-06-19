"""Tests for HTML report rendering."""

from hoc_console.report_html import render_html_report


def test_render_html_report_contains_experiment_id():
    html = render_html_report(
        experiment_id='exp_test',
        metadata={'scenario_id': 'SC-01'},
        summary={'max_risk_level': 1, 'max_composite_score': 0.2, 'mean_kl': 0.1, 'mean_w1': 0.05, 'mean_mmd': 0.02},
        risk_timeline=[],
        metrics_timeline=[{'t': 1.0, 'kl_mean': 0.1, 'w1_mean': 0.05, 'mmd_stat': 0.02, 'shift_detected': False}],
        alerts=[],
        latest_risk=None,
        latest_metrics=None,
        screenshot_b64=None,
        recommendation='ok',
    )
    assert 'exp_test' in html
    assert 'SC-01' in html
    assert 'Mean W1' in html
    assert '0.05' in html
