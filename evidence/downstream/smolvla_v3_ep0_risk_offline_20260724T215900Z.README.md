# SmolVLA v3 → offline RiskAggregator readiness对照 (P3)

**Date**: 2026-07-24  
**Kind**: offline risk readiness对照 (not task eval)

## What this is

Reuses downstream `RiskAggregator` (six weighted dimensions → R0–R3) on existing
PolicyRunner timeseries, with Isaac S4 trial reports as a **partial companion**.

- Primary: `smolvla_v3_ep0_policyrunner_20260724T213800Z/benchmark_timeseries.csv`
- Companion: `evidence/smolvla_s4_bounded5_20260724T203700Z/trials/seed_*/report.json`
- Aggregator: `risk_engine/risk_engine/aggregator.py::RiskAggregator`
- Runner: `scripts/run_offline_risk_readiness.py` (no ROS / no `risk_node`)

## Explicit non-claims

| Flag | Value |
| --- | --- |
| `claims_task_success` | `false` |
| `claims_sim2real` | `false` |
| `overrides_failure_lane` | `false` |
| `overrides_continuous_task_evaluator` | `false` |
| `use_as_task_go_no_go` | `false` |

Risk output is **portfolio readiness对照 only**. Isaac S4 task GT / `failure_lane`
remain authoritative for task outcomes.

## Command

```bash
cd ~/ros2_ws/src/ros2-moveit-pybullet-bridge
PYTHONPATH=risk_engine python3 scripts/run_offline_risk_readiness.py \
  --timeseries evidence/downstream/smolvla_v3_ep0_policyrunner_20260724T213800Z/benchmark_timeseries.csv \
  --summary evidence/downstream/smolvla_v3_ep0_policyrunner_20260724T213800Z/benchmark_summary.json \
  --s4-trials ~/robot-sim-lab/robot-arm-episode-data-lab/evidence/smolvla_s4_bounded5_20260724T203700Z/trials \
  --evaluation-run-id smolvla_v3_ep0_risk_offline_20260724T215900Z \
  --out evidence/downstream/smolvla_v3_ep0_risk_offline_20260724T215900Z.json
```
