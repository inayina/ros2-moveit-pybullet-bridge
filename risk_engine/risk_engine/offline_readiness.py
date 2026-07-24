"""Offline risk readiness对照 from episode timeseries / trial logs.

Reuses RiskAggregator dimension weights for portfolio narrative only.
Does NOT override ContinuousTaskEvaluator, failure_lane, or task go/no-go.
"""

from __future__ import annotations

import csv
import json
import math
import statistics
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from risk_engine.aggregator import RiskAggregator, RiskWeights, clip01

# Match risk_node / risk_config.yaml normalization constants.
KL_NORM = 0.30
W1_NORM = 0.15
MMD_NORM = 0.10
COMM_LATENCY_THRESHOLD_MS = 100.0
RESOURCE_CPU_THRESHOLD_PERCENT = 85.0
TRACKING_RMSE_THRESHOLD = 0.05

ARTIFACT_TYPE = "risk_offline_readiness"
CONTRACT_VERSION = "risk_offline_readiness_v0"

DEFAULT_NON_CLAIMS = (
    "Risk readiness对照 only; not task go/no-go.",
    "Does not claim task success.",
    "Does not claim Sim2Real.",
    "Does not override ContinuousTaskEvaluator or failure_lane.",
    "Missing live monitor dimensions default to 0.0 and are listed under unavailable_dimensions.",
)


@dataclass
class ScoreProvenance:
    """Which fields fed each risk dimension."""

    available: list[str] = field(default_factory=list)
    unavailable: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)


def _finite_floats(values: Iterable[Any]) -> list[float]:
    out: list[float] = []
    for value in values:
        if value is None or value == "":
            continue
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            out.append(number)
    return out


def _percentile(sorted_vals: Sequence[float], q: float) -> float | None:
    if not sorted_vals:
        return None
    if len(sorted_vals) == 1:
        return float(sorted_vals[0])
    idx = (len(sorted_vals) - 1) * q
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return float(sorted_vals[lo])
    frac = idx - lo
    return float(sorted_vals[lo] * (1.0 - frac) + sorted_vals[hi] * frac)


def _peak(values: Sequence[float]) -> float | None:
    return max(values) if values else None


def _mean(values: Sequence[float]) -> float | None:
    return float(statistics.fmean(values)) if values else None


def load_timeseries_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def distribution_shift_from_metrics(
    *,
    kl_mean: float | None,
    w1_mean: float | None,
    mmd: float | None,
    shift_detected: bool = False,
) -> float:
    """Mirror RiskEngineNode distribution_shift normalization."""
    parts: list[float] = []
    if kl_mean is not None and KL_NORM > 0:
        parts.append(kl_mean / KL_NORM)
    if w1_mean is not None and W1_NORM > 0:
        parts.append(w1_mean / W1_NORM)
    if mmd is not None and MMD_NORM > 0:
        parts.append(mmd / MMD_NORM)
    score = max(parts) if parts else 0.0
    if shift_detected:
        score = max(score, 0.5)
    return clip01(score)


def resource_pressure_from_cpu(cpu_percent: float | None) -> float:
    """Mirror RiskEngineNode CPU component of resource_pressure."""
    if cpu_percent is None:
        return 0.0
    threshold = RESOURCE_CPU_THRESHOLD_PERCENT
    return clip01(
        max(0.0, (cpu_percent - threshold) / max(100.0 - threshold, 1.0))
    )


def comm_health_from_latency_ms(latency_ms: float | None) -> float:
    """Proxy: latency / configured comm threshold (risk_config.yaml)."""
    if latency_ms is None or COMM_LATENCY_THRESHOLD_MS <= 0:
        return 0.0
    return clip01(latency_ms / COMM_LATENCY_THRESHOLD_MS)


def tracking_error_from_rmse(rmse: float | None) -> float:
    if rmse is None or TRACKING_RMSE_THRESHOLD <= 0:
        return 0.0
    return clip01(rmse / TRACKING_RMSE_THRESHOLD)


def raw_scores_from_policyrunner_timeseries(
    rows: Sequence[Mapping[str, Any]],
    *,
    summary: Mapping[str, Any] | None = None,
) -> tuple[dict[str, float], ScoreProvenance, dict[str, Any]]:
    """Map PolicyRunner benchmark_timeseries.csv → risk raw_scores."""
    provenance = ScoreProvenance(
        notes=[
            "Primary input: PolicyRunner benchmark_timeseries.csv.",
            "tracking_error / dynamics_anomaly / planning_failure absent from this CSV → 0.0.",
        ]
    )
    latencies = _finite_floats(r.get("latency_ms") for r in rows)
    cpus = _finite_floats(r.get("cpu_percent") for r in rows)
    rss = _finite_floats(r.get("rss_mb") for r in rows)
    infer = _finite_floats(r.get("inference_latency_ms") for r in rows)
    kls = _finite_floats(r.get("kl_mean") for r in rows)
    w1s = _finite_floats(r.get("w1_mean") for r in rows)
    mmds = _finite_floats(r.get("mmd") for r in rows)

    kl_peak = _peak(kls)
    w1_peak = _peak(w1s)
    mmd_peak = _peak(mmds)
    lat_p95 = _percentile(sorted(latencies), 0.95) if latencies else None
    lat_peak = _peak(latencies)
    cpu_peak = _peak(cpus)
    if summary:
        cpu_peak = float(summary.get("cpu_peak_percent", cpu_peak or 0.0))
        if summary.get("max_latency_ms") is not None:
            lat_peak = float(summary["max_latency_ms"])

    dist = distribution_shift_from_metrics(
        kl_mean=kl_peak, w1_mean=w1_peak, mmd=mmd_peak
    )
    # Prefer p95 latency for steady-state comm proxy; fall back to peak.
    comm = comm_health_from_latency_ms(lat_p95 if lat_p95 is not None else lat_peak)
    resource = resource_pressure_from_cpu(cpu_peak)

    if kls or w1s or mmds:
        provenance.available.append("distribution_shift")
    else:
        provenance.unavailable.append("distribution_shift")
    provenance.available.append("comm_health")
    provenance.available.append("resource_pressure")
    provenance.unavailable.extend(
        ["tracking_error", "dynamics_anomaly", "planning_failure"]
    )

    raw = {
        "distribution_shift": dist,
        "tracking_error": 0.0,
        "dynamics_anomaly": 0.0,
        "comm_health": comm,
        "planning_failure": 0.0,
        "resource_pressure": resource,
    }
    stats = {
        "timeseries_rows": len(rows),
        "latency_ms_mean": _mean(latencies),
        "latency_ms_p95": lat_p95,
        "latency_ms_max": lat_peak,
        "cpu_percent_peak": cpu_peak,
        "rss_mb_peak": _peak(rss),
        "inference_latency_ms_mean": _mean(infer),
        "kl_mean_peak": kl_peak,
        "w1_mean_peak": w1_peak,
        "mmd_peak": mmd_peak,
    }
    return raw, provenance, stats


def raw_scores_from_s4_trial_reports(
    trial_reports: Sequence[Mapping[str, Any]],
) -> tuple[dict[str, float], ScoreProvenance, dict[str, Any]]:
    """Partial map from Isaac S4 policy report.json files (companion only)."""
    provenance = ScoreProvenance(
        notes=[
            "Companion input: Isaac S4 trial report.json (no KL/W1/MMD or tracking RMSE).",
            "comm_health from inference_latency_ms_p50; dynamics soft proxy from action_clipped.",
        ]
    )
    lat_p50s: list[float] = []
    clipped_rates: list[float] = []
    estop_count = 0
    for report in trial_reports:
        if report.get("inference_latency_ms_p50") is not None:
            lat_p50s.append(float(report["inference_latency_ms_p50"]))
        actions = report.get("actions") or []
        if actions:
            clipped = sum(1 for a in actions if a.get("action_clipped"))
            clipped_rates.append(clipped / len(actions))
        if report.get("final_safety_estop"):
            estop_count += 1

    lat_mean = _mean(lat_p50s)
    clip_mean = _mean(clipped_rates) or 0.0
    comm = comm_health_from_latency_ms(lat_mean)
    # Soft dynamics proxy: clipped ratio + hard bump on any E-stop.
    dynamics = clip01(clip_mean)
    if estop_count > 0:
        dynamics = max(dynamics, 0.6)

    provenance.available.append("comm_health")
    provenance.available.append("dynamics_anomaly")
    provenance.unavailable.extend(
        [
            "distribution_shift",
            "tracking_error",
            "planning_failure",
            "resource_pressure",
        ]
    )
    raw = {
        "distribution_shift": 0.0,
        "tracking_error": 0.0,
        "dynamics_anomaly": dynamics,
        "comm_health": comm,
        "planning_failure": 0.0,
        "resource_pressure": 0.0,
    }
    stats = {
        "trials": len(trial_reports),
        "inference_latency_ms_p50_mean": lat_mean,
        "action_clipped_rate_mean": clip_mean,
        "final_safety_estop_count": estop_count,
    }
    return raw, provenance, stats


def aggregate_readiness(
    raw_scores: Mapping[str, float],
    *,
    weights: RiskWeights | None = None,
) -> dict[str, Any]:
    result = RiskAggregator(weights=weights).aggregate(dict(raw_scores))
    return {
        "level": result.level,
        "composite_score": result.composite_score,
        "primary_driver": result.primary_driver,
        "recommendation": result.recommendation,
        "dimensions": [
            {
                "dimension": d.dimension,
                "raw_score": d.raw_score,
                "weight": d.weight,
                "weighted_score": d.weighted_score,
            }
            for d in result.dimensions
        ],
    }


def build_readiness_report(
    *,
    evaluation_run_id: str,
    primary_source: str,
    primary_raw: Mapping[str, float],
    primary_provenance: ScoreProvenance,
    primary_stats: Mapping[str, Any],
    companion_source: str | None = None,
    companion_raw: Mapping[str, float] | None = None,
    companion_provenance: ScoreProvenance | None = None,
    companion_stats: Mapping[str, Any] | None = None,
    companion_paths: Sequence[str] | None = None,
    notes: Sequence[str] | None = None,
) -> dict[str, Any]:
    primary_agg = aggregate_readiness(primary_raw)
    payload: dict[str, Any] = {
        "contract_version": CONTRACT_VERSION,
        "artifact_type": ARTIFACT_TYPE,
        "evaluation_run_id": evaluation_run_id,
        "claims_task_success": False,
        "claims_sim2real": False,
        "claims_online_autonomous_grasp": False,
        "overrides_failure_lane": False,
        "overrides_continuous_task_evaluator": False,
        "use_as_task_go_no_go": False,
        "primary": {
            "source_kind": "policyrunner_timeseries",
            "path": primary_source,
            "raw_scores": dict(primary_raw),
            "aggregation": primary_agg,
            "input_stats": dict(primary_stats),
            "provenance": asdict(primary_provenance),
        },
        "non_claims": list(DEFAULT_NON_CLAIMS),
        "notes": list(
            notes
            or [
                "Portfolio readiness对照 via offline RiskAggregator.",
                "Task success remains owned by ContinuousTaskEvaluator / S4 GT funnel.",
            ]
        ),
    }
    if companion_raw is not None and companion_source is not None:
        payload["companion"] = {
            "source_kind": "isaac_s4_trial_reports",
            "path": companion_source,
            "paths": list(companion_paths or []),
            "raw_scores": dict(companion_raw),
            "aggregation": aggregate_readiness(companion_raw),
            "input_stats": dict(companion_stats or {}),
            "provenance": asdict(
                companion_provenance or ScoreProvenance()
            ),
        }
    return payload


def load_s4_trial_reports(trials_dir: Path) -> tuple[list[dict[str, Any]], list[str]]:
    reports: list[dict[str, Any]] = []
    paths: list[str] = []
    for seed_dir in sorted(trials_dir.glob("seed_*")):
        report_path = seed_dir / "report.json"
        if not report_path.is_file():
            continue
        reports.append(json.loads(report_path.read_text(encoding="utf-8")))
        paths.append(str(report_path.resolve()))
    return reports, paths


def run_offline_readiness(
    *,
    timeseries_csv: Path,
    summary_json: Path | None = None,
    s4_trials_dir: Path | None = None,
    evaluation_run_id: str | None = None,
) -> dict[str, Any]:
    rows = load_timeseries_csv(timeseries_csv)
    summary = (
        json.loads(summary_json.read_text(encoding="utf-8"))
        if summary_json is not None and summary_json.is_file()
        else None
    )
    raw, provenance, stats = raw_scores_from_policyrunner_timeseries(
        rows, summary=summary
    )
    companion_raw = None
    companion_prov = None
    companion_stats = None
    companion_paths: list[str] = []
    companion_source = None
    if s4_trials_dir is not None and s4_trials_dir.is_dir():
        trial_reports, companion_paths = load_s4_trial_reports(s4_trials_dir)
        if trial_reports:
            companion_raw, companion_prov, companion_stats = (
                raw_scores_from_s4_trial_reports(trial_reports)
            )
            companion_source = str(s4_trials_dir.resolve())

    run_id = evaluation_run_id or timeseries_csv.parent.name
    return build_readiness_report(
        evaluation_run_id=run_id,
        primary_source=str(timeseries_csv.resolve()),
        primary_raw=raw,
        primary_provenance=provenance,
        primary_stats=stats,
        companion_source=companion_source,
        companion_raw=companion_raw,
        companion_provenance=companion_prov,
        companion_stats=companion_stats,
        companion_paths=companion_paths,
    )


def appendix_for_unified_bundle(report: Mapping[str, Any]) -> dict[str, Any]:
    """Compact appendix block for unified eval bundle (does not alter backends)."""
    primary = report.get("primary") or {}
    agg = primary.get("aggregation") or {}
    return {
        "artifact_type": ARTIFACT_TYPE,
        "contract_version": CONTRACT_VERSION,
        "evaluation_run_id": report.get("evaluation_run_id"),
        "claims_task_success": False,
        "overrides_failure_lane": False,
        "use_as_task_go_no_go": False,
        "risk_level": agg.get("level"),
        "composite_score": agg.get("composite_score"),
        "primary_driver": agg.get("primary_driver"),
        "recommendation": agg.get("recommendation"),
        "dimensions": {
            d["dimension"]: d["raw_score"]
            for d in (agg.get("dimensions") or [])
            if isinstance(d, dict) and "dimension" in d
        },
        "source_path": primary.get("path"),
        "companion_trials": (report.get("companion") or {})
        .get("input_stats", {})
        .get("trials"),
        "non_claims": list(report.get("non_claims") or DEFAULT_NON_CLAIMS),
    }
