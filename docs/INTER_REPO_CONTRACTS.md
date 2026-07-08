# Inter-Repo Interface Contracts

This repository is the downstream bridge/runtime in the three-repo robot-arm
loop. It consumes middle-repo policy handoff bundles, validates runtime IO, runs
MoveIt/PyBullet replay or deployment-readiness checks, and sends lightweight
feedback upstream/middle. It does not clean datasets or train policies.

## Repository Roles

| Repository | Role | Owns | Does not own |
|---|---|---|---|
| `ros2-arm-teleoperation-suite` | Upstream runtime and capture | ROS 2/MuJoCo capture, recorder schema, raw episodes, upstream validation | Dataset release, policy training, downstream replay |
| `robot-arm-episode-data-lab` | Middle data/training repository | Schema adaptation, dataset release, ACT/Diffusion training, checkpoint/export, handoff manifests | ROS 2 runtime nodes, MoveIt/PyBullet execution |
| `ros2-moveit-pybullet-bridge` | Downstream bridge/runtime | Handoff loading, MoveIt/PyBullet replay, runtime IO validation, distribution/risk monitoring, feedback summaries | Raw episode collection, dataset cleaning/release, policy training |

## Gate A: Handoff Load

Producer: `robot-arm-episode-data-lab`

Expected bundle:

```text
bridge_handoff/
├── predicted_actions.jsonl
├── dataset_manifest.json
├── dataset_inspection_report.json
├── replay_check.json
└── handoff_manifest.json
```

Fail fast when:

- `handoff_manifest.json` is missing or has an unknown `handoff_format`;
- `schema_id` is not `panda_ee_delta_gripper_v0` for the Panda replay path;
- `action_type` is not `ee_delta_gripper`;
- action dimension is not `[7]`;
- replay rows contain NaN/Inf or missing `episode_index`, `frame_index`,
  `timestamp`, `release_id`, or `task`;
- manifest does not declare control rate, frame convention, or gripper range.

## Gate B: Runtime IO

Before executing a policy or replay stream, bridge code must resolve:

| Key | Expected value / rule |
|---|---|
| `command_topic` | `/bridge/command` unless explicitly overridden |
| `command_type` | `trajectory_msgs/msg/JointTrajectory` |
| `robot_profile` | `panda` for Panda handoff bundles |
| `input_action` | `ee_delta_gripper[7]`; never treat as joint target |
| `state_source` | `/bridge/sim/joint_states` aligned to Panda joint names |
| `control_rate_hz` | declared in manifest; adapter clamps to configured runtime rate |
| `position_units` | meters for delta xyz, radians for delta rpy/joints |
| `quaternion_order` | ROS `[qx, qy, qz, qw]` in manifests and summaries |
| `gripper_range_m` | manifest-declared range, normally `[0.0, 0.08]` |

Use `docs/templates/runtime_io_requirements.yaml` when requesting new fields
from middle or upstream repos.

## Gate C: Replay / Deployment Summary

Every Panda replay or deployment-readiness run that informs upstream/middle
work should produce a small summary based on:

```text
docs/templates/downstream_replay_summary.yaml
```

The summary should include:

- handoff id, release id, policy id, schema id, action type/dim;
- loaded/rejected status and rejection reason;
- planning failures, action limit violations, E-stop/HOLD events;
- max/mean tracking error and control-rate mismatch;
- likely owner for each failure: upstream, middle, downstream, or unknown.

Do not send large replay logs, videos, rollout datasets, checkpoint binaries, or
full generated benchmark directories back across repos.

## Gate D: Feedback Routing

Feedback to middle repo:

- handoff manifest ambiguity;
- replay JSONL schema/action errors;
- normalization or action distribution issues;
- policy artifact metadata gaps.

Feedback to upstream repo:

- collection quality or per-task success issues visible only during replay;
- frame/quaternion convention mismatch caused by capture;
- gripper command range or object placement behavior that should be fixed in
  upstream sim/recorder/batch generation.

Feedback files should be small, committed under docs/reports or docs/templates
in the owning repo, and referenced by issue/commit rather than copied as data.

## Related Docs

- `docs/PANDA_JSONL_REPLAY_ROADMAP.md`
- `docs/design/13-three-repo-integration-development-plan.md`
- `docs/ICD.md`
- `docs/INTEGRATION.md`
