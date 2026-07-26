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


def test_render_html_report_does_not_render_invalid_metrics_as_zero():
    html = render_html_report(
        experiment_id='exp_invalid',
        metadata={},
        summary={
            'max_risk_level': 0,
            'max_composite_score': 0.0,
            'mean_kl': None,
            'mean_w1': None,
            'mean_mmd': None,
            'shift_detected_ratio': None,
        },
        risk_timeline=[],
        metrics_timeline=[],
        alerts=[],
        latest_risk=None,
        latest_metrics=None,
        screenshot_b64=None,
        recommendation='no valid distribution samples',
    )
    assert html.count('unavailable') >= 4


def test_render_html_report_shows_command_correlation_without_success_claim():
    html = render_html_report(
        experiment_id='exp_trace', metadata={},
        summary={'max_risk_level': 0, 'max_composite_score': 0.0},
        risk_timeline=[], metrics_timeline=[], alerts=[], latest_risk=None,
        latest_metrics=None, screenshot_b64=None, recommendation='',
        runtime_trace_report={
            'issues': [],
            'commands': [{
                'command_sequence': 42,
                'execution': [{'decision': 'EXECUTED'}],
                'safety': [{'proposed_decision': 'RUN', 'actual_decision': 'RUN'}],
                'task_gt': [{'task_status': 'RUNNING'}],
            }],
        },
    )
    assert 'Policy Command Correlation' in html
    assert '<td>42</td>' in html
    assert 'Claims task success: false' in html
