# Current Status

Last audited: 2026-07-26.

## Portfolio Mainline

`ros2-moveit-pybullet-bridge` is the downstream **execution-validation** surface of
the three-repo Panda loop (Sim2Sim readiness — not task go/no-go).

Current mainline:

1. Load Panda `bridge_handoff` artifacts from `robot-arm-episode-data-lab`.
2. Replay `ee_delta_gripper[7]` (or midstream-exported) actions via `panda_jsonl_replay`.
3. Adapt actions into Panda joint targets with `PandaActionAdapter`.
4. Execute or observe the replay in PyBullet (`benchmark_system.py --launch-stack`
   brings up bridge + `dist_monitor` + `risk_engine`).
5. Produce offline RiskAggregator readiness JSON
   (`scripts/run_offline_risk_readiness.py`) for midstream
   `unified_eval_report` `appendix.risk_readiness`
   (`use_as_task_go_no_go=false`, `overrides_failure_lane=false`).
6. Strictly load a versioned `panda_policy_trace_bundle_v1` and replay native
   absolute EEF8 commands through `PolicyCommandReplayPolicy` plus its
   independent adapter. This M5 route is offline evidence only
   (`is_closed_loop=false`, `claims_task_success=false`).

## Supported Evidence

- Panda robot profile and launch defaults.
- Panda JSONL handoff loader and action adapter.
- PyBullet execution bridge with HOLD/PAUSE/E_STOP states.
- Distribution metrics: KL, Wasserstein-1, MMD.
- Online risk_engine on the monitoring launch path.
- Offline risk readiness smoke hung on the midstream unified envelope
  (SmolVLA v3 1-ep PolicyRunner + risk appendix).
- HOC command-sequence correlation across Brain, Execution, Safety, and Task GT,
  with fail-closed five-track trace-bundle export.
- Strict PolicyCommand trace loading: invalid hashes, action schema, sequence,
  parent links, or task-success claims are rejected before replay.
- M6 bounded ROS/DDS wiring smoke with a mock PolicyBackend: command QoS,
  R2 actual HOLD, R3 TriggerEstop/E_STOP, four-lane HOC correlation, strict
  trace reload, timeout, and clean process exit all passed.
- Legacy MoveIt / FollowJointTrajectory path for iiwa7 regression.

## Partial Or Future Work

- Full multi-episode / fault-injection risk campaign on a single canonical handoff run.
- Downstream physical object-grasp validation scene (explicitly out of task go/no-go scope).
- Online ACT/VLA action-chunk runtime as a closed-loop success claim.
- SmolVLA authoritative online cutover or any PyBullet/Isaac policy wiring run.
- Real Panda hardware source.
- Full Sim2Real claims.

## Wording Rules

Use:

- Panda handoff replay;
- risk-monitored PyBullet execution;
- Sim-to-Sim/domain-randomization readiness;
- legacy iiwa7 MoveIt regression.

Avoid:

- full Sim2Real;
- real-robot execution;
- downstream grasp validation platform;
- LeRobot replay as real robot;
- ACT runtime if only scripts/offline checkpoints are involved.
