import { memo, useMemo } from 'react';
import { useDashboardStore } from '../stores/dashboardStore';
import { StableChart } from './StableChart';

export const TrackingChart = memo(function TrackingChart() {
  const tracking = useDashboardStore((s) => s.tracking);

  const option = useMemo(() => {
    const joints = tracking?.joint_names ?? [];
    const errors = tracking?.errors ?? [];
    return {
      backgroundColor: 'transparent',
      animation: false,
      tooltip: { trigger: 'axis' },
      grid: { left: 48, right: 16, top: 24, bottom: 32 },
      xAxis: {
        type: 'category',
        data: joints.length ? joints : ['J1', 'J2', 'J3'],
        axisLabel: { color: '#aaa', rotate: joints.length > 4 ? 30 : 0 },
      },
      yAxis: {
        type: 'value',
        name: '关节误差 (rad)',
        axisLabel: { color: '#aaa' },
        splitLine: { lineStyle: { color: '#303030' } },
      },
      series: [
        {
          name: 'Sim − Real',
          type: 'line',
          data: errors.length ? errors : joints.map(() => 0),
          smooth: true,
          lineStyle: { color: '#b37feb' },
          areaStyle: { color: 'rgba(179,127,235,0.15)' },
        },
      ],
    };
  }, [tracking]);

  const rmse = useMemo(() => {
    const errors = tracking?.errors ?? [];
    if (!errors.length) return 0;
    const sq = errors.reduce((sum, e) => sum + e * e, 0);
    return Math.sqrt(sq / errors.length);
  }, [tracking]);

  return (
    <div className="panel panel--chart">
      <h3>关节跟踪误差</h3>
      <StableChart option={option} height={200} />
      <p className="panel-caption">RMSE: {rmse.toFixed(4)} rad</p>
    </div>
  );
});
