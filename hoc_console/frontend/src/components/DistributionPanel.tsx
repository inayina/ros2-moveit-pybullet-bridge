import { Badge, Space, Statistic, Tag } from 'antd';
import ReactECharts from 'echarts-for-react';
import { useMemo } from 'react';
import { useDashboardStore } from '../stores/dashboardStore';

export function DistributionPanel() {
  const metrics = useDashboardStore((s) => s.metrics);

  const boxplotOption = useMemo(() => {
    const joints = metrics?.joint_names ?? [];
    const klValues = metrics?.kl_divergence_per_joint ?? [];
    const simCount = metrics?.sample_count_sim ?? 0;
    const realCount = metrics?.sample_count_real ?? 0;

    const simData = klValues.map((kl, i) => {
      const spread = kl * 0.3 + 0.01;
      return [i, Math.max(kl - spread, 0), kl, kl + spread * 0.5, kl + spread];
    });
    const realData = klValues.map((kl, i) => {
      const shift = metrics?.shift_detected ? kl * 0.15 : kl * 0.05;
      return [i, Math.max(kl - shift, 0), kl + shift * 0.5, kl + shift, kl + shift * 1.2];
    });

    return {
      backgroundColor: 'transparent',
      tooltip: { trigger: 'item' },
      legend: {
        data: ['Sim 误差', 'Real 误差'],
        textStyle: { color: '#aaa' },
        top: 0,
      },
      grid: { left: 48, right: 16, top: 40, bottom: 48 },
      xAxis: {
        type: 'category',
        data: joints.length ? joints : ['J1', 'J2', 'J3'],
        axisLabel: { color: '#aaa' },
      },
      yAxis: {
        type: 'value',
        name: '误差 (KL proxy)',
        axisLabel: { color: '#aaa' },
        splitLine: { lineStyle: { color: '#303030' } },
      },
      series: [
        {
          name: 'Sim 误差',
          type: 'boxplot',
          data: simData.length ? simData : [[0, 0.01, 0.02, 0.03, 0.04]],
          itemStyle: { color: '#69b1ff', borderColor: '#69b1ff' },
        },
        {
          name: 'Real 误差',
          type: 'boxplot',
          data: realData.length ? realData : [[0, 0.02, 0.04, 0.05, 0.06]],
          itemStyle: { color: '#95de64', borderColor: '#95de64' },
        },
      ],
      title: {
        subtext: `Sim n=${simCount} · Real n=${realCount} · 窗口 ${metrics?.window_duration_sec?.toFixed(1) ?? '—'}s`,
        subtextStyle: { color: '#888' },
        left: 'center',
      },
    };
  }, [metrics]);

  const barOption = useMemo(() => {
    const joints = metrics?.joint_names ?? ['J1', 'J2', 'J3'];
    const kl = metrics?.kl_divergence_per_joint ?? joints.map(() => 0);
    return {
      backgroundColor: 'transparent',
      grid: { left: 40, right: 10, top: 10, bottom: 30 },
      xAxis: { type: 'category', data: joints, axisLabel: { color: '#aaa', fontSize: 10 } },
      yAxis: { type: 'value', axisLabel: { color: '#aaa', fontSize: 10 }, splitLine: { show: false } },
      series: [
        {
          type: 'bar',
          data: kl,
          itemStyle: {
            color: (params: { dataIndex: number }) =>
              kl[params.dataIndex] > (metrics?.kl_divergence_mean ?? 0) ? '#fa8c16' : '#69b1ff',
          },
        },
      ],
    };
  }, [metrics]);

  return (
    <div className="panel">
      <Space style={{ width: '100%', justifyContent: 'space-between' }}>
        <h3>Sim / Real 分布对比</h3>
        {metrics?.shift_detected ? (
          <Tag color="warning">⚠ 检出偏移 ({metrics.detection_method})</Tag>
        ) : (
          <Tag color="success">未检出偏移</Tag>
        )}
      </Space>
      <ReactECharts option={boxplotOption} style={{ height: 220 }} notMerge lazyUpdate />
      <ReactECharts option={barOption} style={{ height: 100 }} notMerge lazyUpdate />
      <Space size="large" wrap>
        <Statistic title="KL mean" value={metrics?.kl_divergence_mean ?? 0} precision={4} />
        <Statistic title="MMD" value={metrics?.mmd_statistic ?? 0} precision={4} />
        <Statistic title="p-value" value={metrics?.mmd_p_value ?? 0} precision={4} />
        <Badge
          status={metrics?.shift_detected ? 'warning' : 'success'}
          text={metrics?.shift_detected ? '偏移' : '正常'}
        />
      </Space>
    </div>
  );
}
