import ReactECharts from 'echarts-for-react';
import { useMemo } from 'react';
import { useDashboardStore } from '../stores/dashboardStore';
import { DIMENSION_LABELS } from '../types/messages';

const AXES = [
  'distribution_shift',
  'tracking_error',
  'dynamics_anomaly',
  'comm_health',
  'planning_failure',
];

export function RiskRadar() {
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
      ? scores.map((v) => Math.max(v * 0.85, 0))
      : scores.map(() => 0);
    return { current: scores, previous: prevScores };
  }, [risk, riskHistory]);

  const option = {
    backgroundColor: 'transparent',
    tooltip: {},
    legend: {
      data: ['当前', '30s前'],
      textStyle: { color: '#aaa' },
      bottom: 0,
    },
    radar: {
      indicator: AXES.map((dim) => ({
        name: DIMENSION_LABELS[dim] ?? dim,
        max: 1,
      })),
      splitArea: { areaStyle: { color: ['#1f1f1f', '#141414'] } },
      axisName: { color: '#ccc' },
    },
    series: [
      {
        type: 'radar',
        data: [
          {
            value: current,
            name: '当前',
            areaStyle: { color: 'rgba(105, 177, 255, 0.35)' },
            lineStyle: { color: '#69b1ff' },
          },
          {
            value: previous,
            name: '30s前',
            lineStyle: { type: 'dashed', color: '#8c8c8c' },
            areaStyle: { opacity: 0 },
          },
        ],
      },
    ],
  };

  return (
    <div className="panel">
      <h3>五维风险雷达</h3>
      <ReactECharts option={option} style={{ height: 280 }} notMerge lazyUpdate />
      <p className="panel-caption">
        主因: {risk?.primary_driver ?? '—'} · {risk?.recommendation ?? '系统运行正常'}
      </p>
    </div>
  );
}
