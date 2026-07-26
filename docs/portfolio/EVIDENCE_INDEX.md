# Evidence Index - ros2-moveit-pybullet-bridge

阶段 1 证据资产索引。状态只能是 `keep`, `regenerate`, `relabel`, `move_to_legacy`, `archive`, `delete`。

| 资产 | 当前仓库 | 主线/Legacy | 数据来源 | 生成脚本 | 输入产物 | 能证明 | 不能证明 | 状态 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `docs/assets/dual-repo-cross-source-metrics.png` | 下游 | Legacy/data plot | old dual-repo validation | script 待确认 | old sample metrics | historical dual-repo comparison | Panda handoff replay canonical | move_to_legacy |
| `docs/assets/dual-repo-cross-source-overlay.png` | 下游 | Legacy/data plot | old dual-repo validation | script 待确认 | old sample trajectories | historical overlay | current Panda downstream facts | move_to_legacy |
| `docs/assets/dual-repo-integration-overview.png` | 下游 | Legacy/design | old dual-repo docs | script 待确认 | old docs | legacy integration design | Panda current mainline | move_to_legacy |
| `docs/assets/dual-repo-lerobot-trajectory.png` | 下游 | Legacy/data plot | old LeRobot trajectory | script 待确认 | old data | historical trajectory view | current handoff replay | move_to_legacy |
| `docs/assets/dual-repo-offline-self-metrics.png` | 下游 | Legacy/data plot | old offline metrics | script 待确认 | old metrics | historical offline metric | current benchmark | move_to_legacy |
| `docs/assets/m1-joint-sweep.gif` | 下游 | Legacy/demo | iiwa/legacy demo | script 待确认 | legacy sim | old joint sweep | Panda replay | move_to_legacy |
| `docs/assets/m1-pybullet.png` | 下游 | Legacy/screenshot | legacy PyBullet | script 待确认 | legacy sim | old PyBullet scene | Panda handoff replay | move_to_legacy |
| `docs/assets/m2-iiwa-pipeline.svg` | 下游 | Legacy/design | iiwa pipeline | script 待确认 | legacy docs | iiwa pipeline design | Panda pipeline | move_to_legacy |
| `docs/assets/m2-iiwa-pybullet.gif` | 下游 | Legacy/demo | iiwa PyBullet | script 待确认 | legacy sim | iiwa replay demo | Panda replay | move_to_legacy |
| `docs/assets/m2-iiwa-pybullet.png` | 下游 | Legacy/screenshot | iiwa PyBullet | script 待确认 | legacy sim | iiwa scene | Panda current state | move_to_legacy |
| `docs/assets/m2-iiwa-rviz.gif` | 下游 | Legacy/demo | iiwa RViz | script 待确认 | legacy ROS run | historical RViz demo | Panda handoff replay | move_to_legacy |
| `docs/assets/m2-moveit-pipeline.svg` | 下游 | Legacy/design | MoveIt legacy pipeline | script 待确认 | legacy docs | old MoveIt design | current Panda replay benchmark | move_to_legacy |
| `docs/assets/m3-dual-source.gif` | 下游 | Legacy/synthetic demo | synthetic/old dual-source demo | script 待确认 | old docs note | visual idea only | current runtime evidence | archive |
| `docs/assets/m4-monitor-metrics.png` | 下游 | Mainline/support | monitor metrics | plotting script 待确认 | distribution metrics output | metrics UI/plot example | canonical Panda numbers unless source linked | relabel |
| `docs/assets/m5-hoc-console.svg` | 下游 | Historical/design | 四泳道改版前 HOC console docs | script 待确认 | legacy docs | 历史 HOC concept/UI | 当前四泳道 HOC、Policy Runtime 或 M6 wiring | move_to_legacy |
| `docs/assets/m5-hoc-dashboard.png` | 下游 | Historical/screenshot | M3 四泳道改版前 HOC dashboard screenshot | manual capture | legacy MLP / five-risk dashboard run | 历史 dashboard UI 与演进过程 | 当前四泳道 HOC、Policy Runtime、安全回灌或 M6 wiring | move_to_legacy |
| `docs/assets/m5-hoc-dashboard.svg` | 下游 | Historical/design | 四泳道改版前 HOC dashboard diagram | script 待确认 | legacy docs | 历史 dashboard design | 当前 runtime evidence | move_to_legacy |
| `docs/assets/m6-pick-and-lift.gif` | 下游 | Legacy/demo | KUKA/pick-lift historical demo | script 待确认 | legacy sim | historical pick/lift demo | Panda downstream grasp validation | move_to_legacy |
| `docs/assets/panda_domain_randomization_distribution.png` | 下游 | Mainline/support | Panda domain randomization/distribution plot | plotting script 待确认 | benchmark/distribution JSON needed | distribution visualization | generalized Sim2Real success | relabel |
| `docs/assets/panda_fault_injection_safety_response.png` | 下游 | Mainline/data plot | fault injection benchmark | plotting script 待确认 | fault benchmark JSON needed | fault response if source exists | certified safety | regenerate |
| `docs/assets/panda_replay_control_latency.png` | 下游 | Mainline/data plot | Panda replay benchmark | plotting script 待确认 | benchmark summary/timeseries JSON | replay latency if tied to run ID | real robot latency | regenerate |
| `docs/assets/panda_replay_distribution_monitoring.png` | 下游 | Mainline/data plot | distribution monitor output | plotting script 待确认 | KL/W1/MMD metrics | monitor output | real/sim equivalence | relabel |
| `docs/assets/panda_replay_resource_usage.png` | 下游 | Mainline/data plot | benchmark resource metrics | plotting script 待确认 | benchmark summary/timeseries JSON | CPU/RSS for run | production capacity | regenerate |
| `docs/assets/panda_sim2sim_trajectory_alignment.png` | 下游 | Mainline/data plot | sim-to-sim trajectory alignment | plotting script 待确认 | replay trajectory logs | trajectory alignment | Sim2Real | relabel |
| `docs/assets/portfolio-overview.png` | 下游 | support/design | portfolio overview | script 待确认 | docs | broad portfolio framing | Panda runtime evidence | archive |
| `docs/assets/same-task-iiwa-metrics.png` | 下游 | Legacy/data plot | old same-task iiwa metrics | script 待确认 | legacy metrics | historical comparison | Panda handoff replay | move_to_legacy |
| `docs/assets/same-task-iiwa-overlay.png` | 下游 | Legacy/data plot | old same-task iiwa overlay | script 待确认 | legacy trajectories | historical comparison | current Panda result | move_to_legacy |
| `docs/assets/same-task-lerobot-metrics.png` | 下游 | Legacy/data plot | old LeRobot metrics | script 待确认 | legacy metrics | historical comparison | current Panda result | move_to_legacy |
| `docs/assets/same-task-lerobot-overlay.png` | 下游 | Legacy/data plot | old LeRobot overlay | script 待确认 | legacy trajectories | historical comparison | current Panda result | move_to_legacy |
| `docs/assets/three_repo_dataflow_diagram.png` | 下游 | Mainline/design | three-repo docs | canonical script should live in midstream | `THREE_REPO_CANONICAL_FACTS.md` | high-level dataflow | run evidence | regenerate |
| `docs/assets/three_repo_run_evidence.png` | 下游 | Mainline/evidence collage | three-repo run summary | canonical script should live in midstream | canonical metrics + benchmark JSON | summarized run evidence | original raw evidence or real robot | regenerate |
| `docs/samples/.capture_tmp/*` | 下游 | temporary | local capture temp | generated temp files | temp captures | nothing stable | README evidence | delete |
| `docs/samples/portfolio-demo-zh-*.mp4` | 下游 | support/video | portfolio demo video | script/manual capture 待确认 | demo materials | presentation walkthrough | current code behavior | archive |



| `docs/assets/three_repo_canonical_dataflow.svg` | 下游 | Mainline/design | phase-2 canonical facts | manual SVG from midstream canonical source | midstream `THREE_REPO_CANONICAL_FACTS.md` | 三仓职责边界与数据流 | run evidence or real robot capability | keep |

| `docs/assets/three_repo_canonical_run_evidence.svg` | 下游 | Mainline/evidence summary | phase-2 canonical facts and JSON artifacts | manual SVG from midstream canonical source | canonical manifests, metrics, handoff, latest benchmark JSON | README 级运行证据摘要 | original artifacts or real robot capability | keep |
| `docs/assets/hoc-runtime-four-lane-dashboard.png` | 下游 | Mainline/frontend screenshot | 2026-07-26 current HOC frontend | `hoc_console/frontend/e2e/portfolio-screenshot.spec.ts` + mock WebSocket | deterministic RUN→E_STOP→HOLD history；Final Decision、原因链、四泳道 state timeline 与连续诊断布局 | M6 live wiring、SmolVLA task performance、authoritative cutover、Sim2Real | keep |

## Notes

- Downstream README 主体应聚焦 Panda handoff loader, JSONL replay, PandaActionAdapter, PyBullet execution, distribution/risk benchmark。
- iiwa、dual-repo、same-task、portfolio-wide materials should move to Legacy/extended reading.
- Panda data plots need a run ID and original JSON/timeseries source before being used as README evidence.
