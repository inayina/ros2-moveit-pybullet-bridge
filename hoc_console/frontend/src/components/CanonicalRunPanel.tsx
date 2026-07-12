import { ArrowRightOutlined, CheckCircleFilled, WarningFilled } from '@ant-design/icons';
import { Tag, Tooltip } from 'antd';
import { CANONICAL_RUN } from '../data/canonicalRun';

const stages = [
  {
    key: 'dataset',
    eyebrow: 'G0 · UPSTREAM',
    title: 'Panda Dataset',
    value: CANONICAL_RUN.dataset.episodes + ' episodes',
    detail: CANONICAL_RUN.dataset.frames.toLocaleString() + ' frames · ' + CANONICAL_RUN.dataset.gate,
  },
  {
    key: 'training',
    eyebrow: 'G1 · MIDSTREAM',
    title: CANONICAL_RUN.policy.name,
    value: CANONICAL_RUN.policy.split,
    detail: 'loss ' + CANONICAL_RUN.policy.trainLoss + ' → ' + CANONICAL_RUN.policy.testLoss,
  },
  {
    key: 'handoff',
    eyebrow: 'CONTRACT',
    title: 'Bridge Handoff',
    value: CANONICAL_RUN.handoff.status,
    detail: CANONICAL_RUN.handoff.actions.toLocaleString() + ' actions',
  },
  {
    key: 'replay',
    eyebrow: 'G2 · DOWNSTREAM',
    title: 'Panda Replay',
    value: CANONICAL_RUN.replay.completed + ' complete',
    detail: CANONICAL_RUN.replay.meanLatencyMs + ' ms mean · ' + CANONICAL_RUN.replay.commandMode,
  },
  {
    key: 'fault',
    eyebrow: 'SAFETY',
    title: 'Fault Response',
    value: CANONICAL_RUN.fault.alarmLatencyMs + ' ms',
    detail: 'health alarm · within 1 s',
  },
];

export function CanonicalRunPanel() {
  return (
    <section className="canonical-run" aria-label="Canonical Panda closed-loop run">
      <div className="canonical-run__header">
        <div>
          <span className="canonical-run__kicker">CANONICAL PORTFOLIO RUN</span>
          <strong>{CANONICAL_RUN.runId}</strong>
        </div>
        <div className="canonical-run__badges">
          <Tag color="cyan">Franka Panda</Tag>
          <Tag color="geekblue">{CANONICAL_RUN.scope}</Tag>
          <Tooltip title={CANONICAL_RUN.warnings.join('；')}>
            <Tag icon={<WarningFilled />} color="warning">2 个已知边界</Tag>
          </Tooltip>
        </div>
      </div>
      <div className="canonical-run__flow">
        {stages.map((stage, index) => (
          <div className="canonical-run__stage-wrap" key={stage.key}>
            <article className="canonical-run__stage">
              <span>{stage.eyebrow}</span>
              <h3>{stage.title}</h3>
              <strong><CheckCircleFilled /> {stage.value}</strong>
              <small>{stage.detail}</small>
            </article>
            {index < stages.length - 1 ? <ArrowRightOutlined className="canonical-run__arrow" /> : null}
          </div>
        ))}
      </div>
    </section>
  );
}
