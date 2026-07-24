# SmolVLA Recovery v3 → PolicyRunner reuse smoke

Downstream mirror of midstream archive
`robot-arm-episode-data-lab/evidence/downstream/smolvla_v3_ep0_policyrunner_20260724T213800Z/`.

**Proves**: `load_handoff_bundle` + PolicyRunner `panda_jsonl_replay` + `pybullet_ik`
with `--launch-stack` completed 1 episode (latency/RSS timeseries collected).

**Does not prove**: task success, closed-loop grasp, Sim2Real, or Isaac Pass.
Handoff `is_closed_loop=false`.
