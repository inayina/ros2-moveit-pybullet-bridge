"""Tests for CSV report export."""

from hoc_console.report_csv import render_csv_report


def test_render_csv_report_header_and_row():
    csv_text = render_csv_report(
        risk_timeline=[{'t': 1.0, 'level': 1, 'score': 0.3}],
        metrics_timeline=[{
            't': 1.0,
            'kl_mean': 0.12,
            'w1_mean': 0.05,
            'mmd_stat': 0.03,
            'shift_detected': True,
        }],
    )
    lines = csv_text.strip().splitlines()
    assert lines[0].startswith('t,risk_level')
    assert '1.000' in lines[1]
    assert 'True' in lines[1]
