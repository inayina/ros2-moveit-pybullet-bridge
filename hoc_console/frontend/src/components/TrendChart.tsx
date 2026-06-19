import ReactECharts from 'echarts-for-react';
import { useMemo } from 'react';
import { useDashboardStore } from '../stores/dashboardStore';

export function TrendChart() {
  const history = useDashboardStore((s) => s.metricsHistory);

  const option = useMemo(() => {
    const labels = history.map((p) => {
      const age = Math.round(Date.now() / 1000 - p.t);
      return `-${age}s`;
    });
    return {
      backgroundColor: 'transparent',
      tooltip: { trigger: 'axis' },
      legend: {
        data: ['KL mean', 'MMD'],
        textStyle: { color: '#aaa' },
        top: 0,
      },
      grid: { left: 48, right: 16, top: 36, bottom: 28 },
      xAxis: {
        type: 'category',
        data: labels,
        axisLabel: { color: '#888', interval: Math.max(Math.floor(labels.length / 6), 1) },
      },
      yAxis: {
        type: 'value',
        axisLabel: { color: '#aaa' },
        splitLine: { lineStyle: { color: '#303030' } },
      },
      series: [
        {
          name: 'KL mean',
          type: 'line',
          smooth: true,
          showSymbol: false,
          data: history.map((p) => p.kl_mean),
          lineStyle: { color: '#69b1ff' },
          areaStyle: { color: 'rgba(105,177,255,0.15)' },
        },
        {
          name: 'MMD',
          type: 'line',
          smooth: true,
          showSymbol: false,
          data: history.map((p) => p.mmd_stat),
          lineStyle: { color: '#ffc53d' },
        },
      ],
    };
  }, [history]);

  return (
    <div className="panel panel--wide">
      <h3>KL / MMD 时序趋势（最近 60 秒）</h3>
      <ReactECharts option={option} style={{ height: 180 }} notMerge lazyUpdate />
    </div>
  );
}
