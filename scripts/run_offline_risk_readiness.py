#!/usr/bin/env python3
"""Offline RiskAggregator readiness对照 from PolicyRunner timeseries (+ optional S4).

Example:
  PYTHONPATH=risk_engine python3 scripts/run_offline_risk_readiness.py \\
    --timeseries evidence/downstream/.../benchmark_timeseries.csv \\
    --summary evidence/downstream/.../benchmark_summary.json \\
    --s4-trials /path/to/smolvla_s4_bounded5_.../trials \\
    --out evidence/downstream/smolvla_v3_ep0_risk_offline_....json

Does NOT launch ROS, override failure_lane, or claim task success / Sim2Real.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RISK_PKG = ROOT / "risk_engine"
if str(RISK_PKG) not in sys.path:
    sys.path.insert(0, str(RISK_PKG))

from risk_engine.offline_readiness import run_offline_readiness  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--timeseries",
        type=Path,
        required=True,
        help="PolicyRunner benchmark_timeseries.csv",
    )
    parser.add_argument(
        "--summary",
        type=Path,
        default=None,
        help="Optional benchmark_summary.json (cpu/latency peaks)",
    )
    parser.add_argument(
        "--s4-trials",
        type=Path,
        default=None,
        help="Optional Isaac S4 trials/ dir with seed_*/report.json",
    )
    parser.add_argument("--evaluation-run-id", type=str, default=None)
    parser.add_argument("--out", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = run_offline_readiness(
        timeseries_csv=args.timeseries,
        summary_json=args.summary,
        s4_trials_dir=args.s4_trials,
        evaluation_run_id=args.evaluation_run_id,
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"wrote {args.out}")
    primary = report["primary"]["aggregation"]
    print(
        f"primary risk_level=R{primary['level']} "
        f"composite={primary['composite_score']:.4f} "
        f"driver={primary['primary_driver']}"
    )
    print(
        "non-claims: claims_task_success=false "
        "overrides_failure_lane=false use_as_task_go_no_go=false"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
