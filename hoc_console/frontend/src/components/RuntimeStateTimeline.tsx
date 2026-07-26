import { useMemo } from 'react';
import { useDashboardStore } from '../stores/dashboardStore';
import type { RuntimeLanePayload } from '../types/messages';

const LANES = ['brain', 'execution', 'safety', 'task_gt'] as const;
const LABELS: Record<(typeof LANES)[number], string> = {
  brain: 'Brain',
  execution: 'Execution',
  safety: 'Safety',
  task_gt: 'Task GT',
};

function laneState(lane: RuntimeLanePayload): string {
  if (lane.lane === 'brain') return String(lane.lifecycle_state ?? lane.validity);
  if (lane.lane === 'execution') return String(lane.decision ?? lane.validity);
  if (lane.lane === 'safety') {
    return String(lane.actual_decision ?? lane.proposed_decision ?? `R${Number(lane.level ?? 0)}`);
  }
  return String(lane.task_status ?? lane.phase ?? lane.validity);
}

function stateTone(value: string): string {
  const state = value.toUpperCase().replace('-', '_');
  if (['ERROR', 'FAIL', 'FAILED', 'E_STOP', 'ESTOPPED'].includes(state)) return 'danger';
  if (['HOLD', 'HELD', 'DEGRADED', 'WARMING_UP', 'R2'].includes(state)) return 'warning';
  if (['UNAVAILABLE', 'STALE', 'NO_DATA', 'INACTIVE'].includes(state)) return 'missing';
  return 'healthy';
}

interface Segment {
  state: string;
  count: number;
}

export function RuntimeStateTimeline() {
  const history = useDashboardStore((state) => state.runtimeHistory);
  const frame = useDashboardStore((state) => state.runtimeFrame);

  const rows = useMemo(() => LANES.map((laneName) => {
    const source = history.length > 0
      ? history.map((point) => point.frame.lanes[laneName])
      : frame ? [frame.lanes[laneName]] : [];
    const segments: Segment[] = [];
    source.forEach((lane) => {
      const state = laneState(lane);
      const last = segments[segments.length - 1];
      if (last?.state === state) last.count += 1;
      else segments.push({ state, count: 1 });
    });
    return { laneName, segments };
  }), [frame, history]);

  if (!frame) return null;

  const execution = frame.lanes.execution;
  const safety = frame.lanes.safety;
  const task = frame.lanes.task_gt;
  const sequence = execution.command_sequence == null ? '—' : String(execution.command_sequence);

  return (
    <section className="runtime-state-history" aria-label="Command-correlated runtime state history">
      <div className="runtime-state-history__header">
        <div>
          <strong>Command-correlated state timeline</strong>
          <small>last 60 s · discrete states · missing data stays visible</small>
        </div>
        <code>selected command #{sequence}</code>
      </div>
      <div className="runtime-cause-chain">
        <span>Brain {laneState(frame.lanes.brain)}</span><b>→</b>
        <span>Execution {laneState(execution)}</span><b>→</b>
        <span>Safety {laneState(safety)}</span>
        <i>Task GT {laneState(task)} · independent</i>
      </div>
      <div className="runtime-state-history__rows">
        {rows.map(({ laneName, segments }) => (
          <div className="runtime-state-row" key={laneName}>
            <strong>{LABELS[laneName]}</strong>
            <div className="runtime-state-track">
              {segments.length === 0 ? (
                <span className="runtime-state-segment runtime-state-segment--missing">NO DATA</span>
              ) : segments.map((segment, index) => (
                <span
                  className={`runtime-state-segment runtime-state-segment--${stateTone(segment.state)}`}
                  key={`${segment.state}-${index}`}
                  style={{ flexGrow: segment.count }}
                  title={`${LABELS[laneName]}: ${segment.state}`}
                >
                  {segment.state}
                </span>
              ))}
            </div>
            <code>{segments[segments.length - 1]?.state ?? 'NO DATA'}</code>
          </div>
        ))}
      </div>
    </section>
  );
}
