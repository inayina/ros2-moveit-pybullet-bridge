export const CANONICAL_RUN = {
  runId: 'panda_30_mlp_20260711',
  scope: 'Sim-to-Sim readiness',
  dataset: { episodes: 30, frames: 71_737, gate: '30/30 PASS' },
  policy: {
    name: 'MLP BC',
    split: '24 train / 6 test',
    trainLoss: 0.04914,
    testLoss: 0.23502,
  },
  handoff: { id: 'panda_30_mlp_bridge_v0', actions: 71_737, status: 'PASS' },
  replay: {
    strategy: 'panda_jsonl_replay',
    commandMode: 'pybullet_ik',
    completed: '1/1',
    meanLatencyMs: 17.626,
    maxLatencyMs: 49.508,
  },
  fault: { alarmLatencyMs: 94.399, detectedWithin1s: true },
  warnings: [
    'Low-dimensional baseline：scene / wrist 图像未用于本次训练',
    '3,275 个 gripper 输出越界，执行端必须 clamp / reject',
  ],
} as const;
