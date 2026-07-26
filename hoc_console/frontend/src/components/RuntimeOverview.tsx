import { Badge, Card, Tag } from 'antd';
import { useDashboardStore } from '../stores/dashboardStore';
import type { RuntimeLanePayload } from '../types/messages';
import { RuntimeStateTimeline } from './RuntimeStateTimeline';

const LANE_TITLES: Record<string, string> = {
  brain: 'Brain',
  execution: 'Execution',
  safety: 'Safety',
  task_gt: 'Task GT',
};

function validityColor(validity: string) {
  if (validity === 'VALID') return '#7f8b92';
  if (validity === 'DEGRADED' || validity === 'WARMING_UP') return '#c28a2c';
  if (validity === 'ERROR') return '#b84f4f';
  return '#69727a';
}

function lanePrimary(lane: RuntimeLanePayload) {
  if (lane.lane === 'brain') {
    return String(lane.lifecycle_state ?? lane.validity);
  }
  if (lane.lane === 'execution') {
    return String(lane.decision ?? lane.validity);
  }
  if (lane.lane === 'safety') {
    if (lane.proposed_decision) {
      return `${String(lane.proposed_decision)} → ${String(lane.actual_decision ?? 'UNAVAILABLE')}`;
    }
    return lane.has_valid_sources ? `R${Number(lane.level ?? 0)}` : 'NO DATA';
  }
  return String(lane.task_status ?? lane.phase ?? lane.validity);
}

function finalDecision(safety: RuntimeLanePayload) {
  if (safety.validity === 'ERROR' || Boolean(safety.e_stop_active)) return 'E-STOP';
  if (!safety.has_valid_sources || ['STALE', 'UNAVAILABLE'].includes(safety.validity)) {
    return 'NO DATA';
  }
  if (Number(safety.level ?? 0) >= 3) return 'E-STOP';
  if (Number(safety.level ?? 0) >= 2) return 'HOLD';
  return 'RUN';
}

export function RuntimeOverview() {
  const frame = useDashboardStore((state) => state.runtimeFrame);
  const lanes = frame?.lanes ?? {
    brain: { lane: 'brain', validity: 'UNAVAILABLE', reason_code: 'no_data' },
    execution: { lane: 'execution', validity: 'UNAVAILABLE', reason_code: 'no_data' },
    safety: { lane: 'safety', validity: 'UNAVAILABLE', reason_code: 'no_data' },
    task_gt: { lane: 'task_gt', validity: 'UNAVAILABLE', reason_code: 'no_data' },
  };
  const decision = finalDecision(lanes.safety);
  return (
    <section className={`runtime-overview runtime-overview--${decision.toLowerCase().replace(' ', '-')}`}>
      <div className="runtime-decision" data-testid="runtime-final-decision">
        <strong>Final Decision: {decision}</strong>
        <span>{lanes.safety.reason_code}</span>
        {frame && !frame.correlation.trace_consistent && (
          <Tag color="red">TRACE MISMATCH</Tag>
        )}
        {Boolean(lanes.safety.decision_mismatch) && (
          <Tag color="red">SAFETY DECISION MISMATCH</Tag>
        )}
      </div>
      <div className="runtime-lanes">
        {Object.values(lanes).map((lane) => (
          <Card key={lane.lane} size="small" className="runtime-lane">
            <div className="runtime-lane__title">
              <strong>{LANE_TITLES[lane.lane]}</strong>
              <Badge
                color={validityColor(lane.validity)}
                text={lane.validity}
              />
            </div>
            <div className="runtime-lane__primary">{lanePrimary(lane)}</div>
            <code>{lane.reason_code}</code>
            <small>
              {lane.age_ms == null ? 'age unavailable' : `${Number(lane.age_ms).toFixed(0)} ms`}
            </small>
          </Card>
        ))}
      </div>
      <RuntimeStateTimeline />
    </section>
  );
}
