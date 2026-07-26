import { memo, useMemo } from 'react';
import { useDashboardStore } from '../stores/dashboardStore';
import { DIMENSION_LABELS } from '../types/messages';
import { StableChart } from './StableChart';

const AXES = [
  'distribution_shift',
  'tracking_error',
  'dynamics_anomaly',
  'comm_health',
  'planning_failure',
];

export const RiskRadar = memo(function RiskRadar() {
  const risk = useDashboardStore((s) => s.risk);
  const riskHistory = useDashboardStore((s) => s.riskHistory);

  const { current, previous } = useMemo(() => {
    const scores = AXES.map((dim) => {
      const attr = risk?.attribution.find((a) => a.dimension === dim);
      return Math.min(attr?.raw_score ?? 0, 1);
    });
    const cutoff = Date.now() / 1000 - 30;
    const oldPoint = [...riskHistory].reverse().find((p) => p.t <= cutoff);
    const prevScores = oldPoint
      ? AXES.map((dim) => Math.min(oldPoint.attribution[dim] ?? 0, 1))
      : AXES.map(() => 0);
    return { current: scores, previous: prevScores };
  }, [risk, riskHistory]);

  const option = useMemo(
    () => ({
      backgroundColor: 'transparent',
      animation: false,
      tooltip: {},
      legend: {
        data: ['当前', '30s前'],
        textStyle: { color: '#a4adb4' },
        bottom: 0,
      },
      radar: {
        indicator: AXES.map((dim) => ({
          name: DIMENSION_LABELS[dim] ?? dim,
          max: 1,
        })),
        splitArea: { areaStyle: { color: ['#30363c', '#292e33'] } },
        splitLine: { lineStyle: { color: '#535b62' } },
        axisLine: { lineStyle: { color: '#535b62' } },
        axisName: { color: '#c6ccd0' },
      },
      series: [
        {
          type: 'radar',
          data: [
            {
              value: current,
              name: '当前',
              areaStyle: { color: 'rgba(100, 133, 163, 0.30)' },
              lineStyle: { color: '#6485a3' },
            },
            {
              value: previous,
              name: '30s前',
              lineStyle: { type: 'dashed', color: '#929ba2' },
              areaStyle: { opacity: 0 },
            },
          ],
        },
      ],
    }),
    [current, previous],
  );

  return (
    <div className="panel panel--chart">
      <h3>五维风险雷达</h3>
      <StableChart option={option} height={360} />
      <p className="panel-caption">
        主因: {risk?.primary_driver ?? '—'} · {risk?.recommendation ?? '系统运行正常'}
      </p>
    </div>
  );
});
