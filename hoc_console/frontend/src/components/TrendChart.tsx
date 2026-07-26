import { memo, useMemo } from 'react';
import { useDashboardStore } from '../stores/dashboardStore';
import { StableChart } from './StableChart';

export const TrendChart = memo(function TrendChart() {
  const history = useDashboardStore((s) => s.metricsHistory);

  const option = useMemo(() => {
    const klData = history.map((p) => [p.t * 1000, p.kl_mean]);
    const w1Data = history.map((p) => [p.t * 1000, p.w1_mean]);
    const mmdData = history.map((p) => [p.t * 1000, p.mmd_stat]);
    const commData = history.map((p) => [p.t * 1000, p.comm_health_score]);
    return {
      backgroundColor: 'transparent',
      animation: false,
      tooltip: { trigger: 'axis' },
      legend: {
        data: ['KL mean', 'W1 mean', 'MMD', '通信健康'],
        textStyle: { color: '#a4adb4' },
        top: 0,
      },
      grid: { left: 48, right: 48, top: 36, bottom: 28 },
      dataZoom: [
        { type: 'inside', xAxisIndex: 0, filterMode: 'none' },
        { type: 'slider', xAxisIndex: 0, height: 16, bottom: 4, textStyle: { color: '#a4adb4' } },
      ],
      xAxis: {
        type: 'time',
        axisLabel: { color: '#9da6ad' },
        splitLine: { show: false },
      },
      yAxis: [
        {
          type: 'value',
          scale: true,
          axisLabel: { color: '#a4adb4' },
          splitLine: { lineStyle: { color: '#485057' } },
        },
        {
          type: 'value',
          min: 0,
          max: 1,
          axisLabel: { color: '#a4adb4' },
          splitLine: { show: false },
        },
      ],
      series: [
        {
          name: 'KL mean',
          type: 'line',
          smooth: true,
          showSymbol: false,
          data: klData,
          lineStyle: { color: '#5f7f9c' },
          areaStyle: { color: 'rgba(95,127,156,0.15)' },
        },
        {
          name: 'W1 mean',
          type: 'line',
          smooth: true,
          showSymbol: false,
          data: w1Data,
          lineStyle: { color: '#887a91' },
        },
        {
          name: 'MMD',
          type: 'line',
          smooth: true,
          showSymbol: false,
          data: mmdData,
          lineStyle: { color: '#a68a5d' },
        },
        {
          name: '通信健康',
          type: 'line',
          yAxisIndex: 1,
          smooth: true,
          showSymbol: false,
          data: commData,
          lineStyle: { color: '#9a6f6f', type: 'dashed' },
        },
      ],
    };
  }, [history]);

  return (
    <div className="panel panel--wide panel--chart">
      <h3>KL / W1 / MMD / 通信健康 时序趋势（最近 60 秒）</h3>
      <StableChart option={option} height={180} />
    </div>
  );
});
