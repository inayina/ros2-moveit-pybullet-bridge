"""Tests for HTML report rendering."""

from hoc_console.report_html import render_html_report


def test_render_html_report_contains_experiment_id():
    html = render_html_report(
        experiment_id='exp_test',
        metadata={'scenario_id': 'SC-01'},
        summary={'max_risk_level': 1, 'max_composite_score': 0.2, 'mean_kl': 0.1, 'mean_mmd': 0.02},
        risk_timeline=[],
        metrics_timeline=[],
        alerts=[],
        latest_risk=None,
        latest_metrics=None,
        screenshot_b64=None,
        recommendation='ok',
    )
    assert 'exp_test' in html
    assert 'SC-01' in html
