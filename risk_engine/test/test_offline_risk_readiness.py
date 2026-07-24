"""Offline risk readiness对照 — pure unit tests (no ROS)."""

from __future__ import annotations

import csv
import json
from pathlib import Path

from risk_engine.offline_readiness import (
    appendix_for_unified_bundle,
    build_readiness_report,
    raw_scores_from_policyrunner_timeseries,
    raw_scores_from_s4_trial_reports,
    run_offline_readiness,
)


def test_policyrunner_timeseries_maps_comm_and_resource(tmp_path: Path) -> None:
    rows = [
        {
            "episode": "0",
            "latency_ms": "50.0",
            "cpu_percent": "90.0",
            "rss_mb": "100.0",
            "inference_latency_ms": "1.0",
            "kl_mean": "0.15",
            "w1_mean": "0.0",
            "mmd": "0.0",
        },
        {
            "episode": "0",
            "latency_ms": "120.0",
            "cpu_percent": "95.0",
            "rss_mb": "110.0",
            "inference_latency_ms": "2.0",
            "kl_mean": "0.30",
            "w1_mean": "0.0",
            "mmd": "0.0",
        },
    ]
    raw, provenance, stats = raw_scores_from_policyrunner_timeseries(rows)
    assert raw["distribution_shift"] == 1.0  # kl peak 0.30 / 0.30
    assert raw["comm_health"] > 0.0
    assert raw["resource_pressure"] > 0.0
    assert raw["tracking_error"] == 0.0
    assert "tracking_error" in provenance.unavailable
    assert stats["timeseries_rows"] == 2


def test_s4_companion_partial_scores() -> None:
    reports = [
        {
            "inference_latency_ms_p50": 50.0,
            "actions": [{"action_clipped": False}, {"action_clipped": True}],
            "final_safety_estop": False,
        },
        {
            "inference_latency_ms_p50": 150.0,
            "actions": [{"action_clipped": False}],
            "final_safety_estop": True,
        },
    ]
    raw, provenance, stats = raw_scores_from_s4_trial_reports(reports)
    assert raw["dynamics_anomaly"] >= 0.6  # estop bump
    assert raw["comm_health"] > 0.0
    assert raw["distribution_shift"] == 0.0
    assert stats["trials"] == 2
    assert "distribution_shift" in provenance.unavailable


def test_report_never_claims_task_success(tmp_path: Path) -> None:
    csv_path = tmp_path / "benchmark_timeseries.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "episode",
                "latency_ms",
                "cpu_percent",
                "rss_mb",
                "inference_latency_ms",
                "kl_mean",
                "w1_mean",
                "mmd",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "episode": "0",
                "latency_ms": "10.0",
                "cpu_percent": "0.0",
                "rss_mb": "30.0",
                "inference_latency_ms": "0.5",
                "kl_mean": "0.0",
                "w1_mean": "0.0",
                "mmd": "0.0",
            }
        )
    report = run_offline_readiness(
        timeseries_csv=csv_path,
        evaluation_run_id="unit_risk_offline",
    )
    assert report["claims_task_success"] is False
    assert report["claims_sim2real"] is False
    assert report["overrides_failure_lane"] is False
    assert report["overrides_continuous_task_evaluator"] is False
    assert report["use_as_task_go_no_go"] is False
    assert report["primary"]["aggregation"]["level"] == 0
    appendix = appendix_for_unified_bundle(report)
    assert appendix["overrides_failure_lane"] is False
    assert appendix["claims_task_success"] is False


def test_build_readiness_keeps_dimension_order() -> None:
    report = build_readiness_report(
        evaluation_run_id="x",
        primary_source="/tmp/ts.csv",
        primary_raw={
            "distribution_shift": 0.0,
            "tracking_error": 0.4,
            "dynamics_anomaly": 0.0,
            "comm_health": 0.0,
            "planning_failure": 0.0,
            "resource_pressure": 0.0,
        },
        primary_provenance=__import__(
            "risk_engine.offline_readiness", fromlist=["ScoreProvenance"]
        ).ScoreProvenance(available=["tracking_error"]),
        primary_stats={"timeseries_rows": 1},
    )
    dims = report["primary"]["aggregation"]["dimensions"]
    assert [d["dimension"] for d in dims] == [
        "distribution_shift",
        "tracking_error",
        "dynamics_anomaly",
        "comm_health",
        "planning_failure",
        "resource_pressure",
    ]
    assert report["primary"]["aggregation"]["primary_driver"] == "tracking_error"
