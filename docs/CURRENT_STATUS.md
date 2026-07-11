# Current Status

Last audited: 2026-07-11.

## Portfolio Mainline

`ros2-moveit-pybullet-bridge` is the downstream execution and monitoring
repository for the three-repo Panda manipulation pipeline.

Current mainline:

1. Load Panda `bridge_handoff` artifacts from `robot-arm-episode-data-lab`.
2. Replay `ee_delta_gripper[7]` actions through `panda_jsonl_replay`.
3. Adapt actions into Panda joint targets with `PandaActionAdapter`.
4. Execute or observe the replay in PyBullet.
5. Monitor tracking error, distribution shift, risk level, system state, and HOC
   dashboard outputs.

## Supported Evidence

- Panda robot profile and launch defaults.
- Panda JSONL handoff loader and action adapter.
- PyBullet execution bridge with HOLD/PAUSE/E_STOP states.
- Distribution metrics: KL, Wasserstein-1, MMD.
- Risk engine aggregation and HOC dashboard bridge.
- Legacy MoveIt / FollowJointTrajectory path for iiwa7 regression.

## Partial Or Future Work

- Full ROS benchmark against the latest real midstream `bridge_handoff_panda`.
- Downstream physical object-grasp validation scene.
- Online ACT action chunk runtime.
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
