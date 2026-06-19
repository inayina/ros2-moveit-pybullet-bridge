import ReactECharts from 'echarts-for-react';
import { memo, useMemo } from 'react';
import { useDashboardStore } from '../stores/dashboardStore';

export const TrendChart = memo(function TrendChart() {
  const history = useDashboardStore((s) => s.metricsHistory);

  const option = useMemo(() => {
    const klData = history.map((p) => [p.t * 1000, p.kl_mean]);
    const w1Data = history.map((p) => [p.t * 1000, p.w1_mean]);
    const mmdData = history.map((p) => [p.t * 1000, p.mmd_stat]);
    return {
      backgroundColor: 'transparent',
      animation: false,
      tooltip: { trigger: 'axis' },
      legend: {
        data: ['KL mean', 'W1 mean', 'MMD'],
        textStyle: { color: '#aaa' },
        top: 0,
      },
      grid: { left: 48, right: 16, top: 36, bottom: 28 },
      xAxis: {
        type: 'time',
        axisLabel: { color: '#888' },
        splitLine: { show: false },
      },
      yAxis: {
        type: 'value',
        scale: true,
        axisLabel: { color: '#aaa' },
        splitLine: { lineStyle: { color: '#303030' } },
      },
      series: [
        {
          name: 'KL mean',
          type: 'line',
          smooth: true,
          showSymbol: false,
          data: klData,
          lineStyle: { color: '#69b1ff' },
          areaStyle: { color: 'rgba(105,177,255,0.15)' },
        },
        {
          name: 'W1 mean',
          type: 'line',
          smooth: true,
          showSymbol: false,
          data: w1Data,
          lineStyle: { color: '#b37feb' },
        },
        {
          name: 'MMD',
          type: 'line',
          smooth: true,
          showSymbol: false,
          data: mmdData,
          lineStyle: { color: '#ffc53d' },
        },
      ],
    };
  }, [history]);

  return (
    <div className="panel panel--wide panel--chart">
      <h3>KL / W1 / MMD 时序趋势（最近 60 秒）</h3>
      <ReactECharts
        option={option}
        style={{ height: 180, width: '100%' }}
        opts={{ renderer: 'canvas' }}
        notMerge
        lazyUpdate
      />
    </div>
  );
});
