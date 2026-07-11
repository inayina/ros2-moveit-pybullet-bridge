# Panda Alignment Current Status

This document records the current facts for the Panda migration in
`ros2-moveit-pybullet-bridge`. It supersedes the older wording that described
Panda support as purely future work.

## Current Mainline

The portfolio mainline is now:

1. Upstream Panda collection in `ros2-arm-teleoperation-suite`.
2. Midstream schema adaptation, baseline replay, and `bridge_handoff_panda` in
   `robot-arm-episode-data-lab`.
3. Downstream Panda PyBullet replay and risk monitoring in this repository.

The downstream repository provides a Panda robot profile, Panda handoff loader,
`panda_jsonl_replay`, and `PandaActionAdapter`. Launch files default to
`robot_profile:=panda` for the portfolio path.

## What Is Implemented

- `panda` is registered in `robot_profiles.py`.
- `DEFAULT_PORTFOLIO_PROFILE` is `panda`.
- `bridge_config.yaml` and `robot_profiles.yaml` default to `panda`.
- `portfolio_demo.launch.py` and `hoc_experiment.launch.py` default to Panda.
- `JsonlActionReplayPolicy` can load midstream Panda handoff bundles.
- `PandaActionAdapter` converts `ee_delta_gripper[7]` actions into bridge joint
  targets using `hold`, `mock_ik`, or `pybullet_ik`.
- Distribution monitoring, tracking metrics, risk aggregation, and HOC remain
  robot-profile agnostic enough for Panda replay demos.

## Remaining Gaps

- Full ROS benchmark evidence should be regenerated against the latest
  `robot-arm-episode-data-lab/data/bridge_handoff_panda` before using it as
  final portfolio proof.
- Downstream does not yet provide a complete physical grasp scene with object
  contact validation.
- ACT online chunked inference is not implemented in the downstream runtime.
- `RealSource` remains randomized PyBullet or replay input, not a real robot.
- MoveIt / FollowJointTrajectory demos are legacy regression evidence, not the
  Panda portfolio mainline.

## Legacy Scope

KUKA iiwa7 stays in the repository for:

- MoveIt / RViz / FollowJointTrajectory regression;
- older screenshots and reports;
- compatibility tests around the original bridge architecture.

Do not describe iiwa7 as the current portfolio robot. Do not describe Panda as a
future-only backend.

## Safe Interview Wording

Current wording:

> The downstream repository executes Panda policy handoff replay in PyBullet and
> adds tracking, distribution-shift monitoring, risk aggregation, and an HOC
> dashboard. iiwa7 / MoveIt remains as a legacy regression path.

Avoid wording:

- completed real-robot Sim2Real;
- downstream grasp-success validation platform;
- ACT online controller;
- LeRobot replay as real robot execution.
