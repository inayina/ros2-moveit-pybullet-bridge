import { Badge, Space, Statistic, Tag } from 'antd';
import ReactECharts from 'echarts-for-react';
import { useMemo } from 'react';
import { useDashboardStore } from '../stores/dashboardStore';

function buildBoxplotSeries(
  joints: string[],
  mins: number[],
  q1s: number[],
  medians: number[],
  q3s: number[],
  maxs: number[],
): number[][] {
  const count = Math.min(joints.length, mins.length, q1s.length, medians.length, q3s.length, maxs.length);
  const data: number[][] = [];
  for (let i = 0; i < count; i += 1) {
    data.push([i, mins[i], q1s[i], medians[i], q3s[i], maxs[i]]);
  }
  return data;
}

export function DistributionPanel() {
  const metrics = useDashboardStore((s) => s.metrics);

  const boxplotOption = useMemo(() => {
    const joints = metrics?.joint_names ?? [];
    const simCount = metrics?.sample_count_sim ?? 0;
    const realCount = metrics?.sample_count_real ?? 0;
    const hasWindowBoxplot =
      (metrics?.sim_position_median_per_joint?.length ?? 0) > 0 &&
      (metrics?.real_position_median_per_joint?.length ?? 0) > 0;

    const simData = hasWindowBoxplot
      ? buildBoxplotSeries(
          joints,
          metrics?.sim_position_min_per_joint ?? [],
          metrics?.sim_position_q1_per_joint ?? [],
          metrics?.sim_position_median_per_joint ?? [],
          metrics?.sim_position_q3_per_joint ?? [],
          metrics?.sim_position_max_per_joint ?? [],
        )
      : [];
    const realData = hasWindowBoxplot
      ? buildBoxplotSeries(
          joints,
          metrics?.real_position_min_per_joint ?? [],
          metrics?.real_position_q1_per_joint ?? [],
          metrics?.real_position_median_per_joint ?? [],
          metrics?.real_position_q3_per_joint ?? [],
          metrics?.real_position_max_per_joint ?? [],
        )
      : [];

    return {
      backgroundColor: 'transparent',
      tooltip: { trigger: 'item' },
      legend: {
        data: ['Sim 位置', 'Real 位置'],
        textStyle: { color: '#aaa' },
        top: 0,
      },
      grid: { left: 48, right: 16, top: 40, bottom: 48 },
      xAxis: {
        type: 'category',
        data: joints.length ? joints : ['J1', 'J2', 'J3'],
        axisLabel: { color: '#aaa', fontSize: 10, rotate: joints.length > 5 ? 30 : 0 },
      },
      yAxis: {
        type: 'value',
        name: hasWindowBoxplot ? '关节位置 (rad)' : '等待窗口样本…',
        axisLabel: { color: '#aaa' },
        splitLine: { lineStyle: { color: '#303030' } },
      },
      series: [
        {
          name: 'Sim 位置',
          type: 'boxplot',
          data: simData.length ? simData : [],
          itemStyle: { color: '#69b1ff', borderColor: '#69b1ff' },
        },
        {
          name: 'Real 位置',
          type: 'boxplot',
          data: realData.length ? realData : [],
          itemStyle: { color: '#95de64', borderColor: '#95de64' },
        },
      ],
      title: {
        subtext: hasWindowBoxplot
          ? `窗口分布 · Sim n=${simCount} · Real n=${realCount} · ${metrics?.window_duration_sec?.toFixed(1) ?? '—'}s`
          : `样本不足 · Sim n=${simCount} · Real n=${realCount}`,
        subtextStyle: { color: '#888' },
        left: 'center',
      },
    };
  }, [metrics]);

  const barOption = useMemo(() => {
    const joints = metrics?.joint_names ?? ['J1', 'J2', 'J3'];
    const kl = metrics?.kl_divergence_per_joint ?? joints.map(() => 0);
    const w1 = metrics?.wasserstein_per_joint ?? joints.map(() => 0);
    return {
      backgroundColor: 'transparent',
      tooltip: { trigger: 'axis' },
      legend: {
        data: ['KL', 'W1'],
        textStyle: { color: '#aaa' },
        top: 0,
      },
      grid: { left: 40, right: 10, top: 28, bottom: 30 },
      xAxis: { type: 'category', data: joints, axisLabel: { color: '#aaa', fontSize: 10 } },
      yAxis: { type: 'value', axisLabel: { color: '#aaa', fontSize: 10 }, splitLine: { show: false } },
      series: [
        {
          name: 'KL',
          type: 'bar',
          data: kl,
          itemStyle: { color: '#69b1ff' },
        },
        {
          name: 'W1',
          type: 'bar',
          data: w1,
          itemStyle: { color: '#b37feb' },
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
        <Statistic title="W1 mean" value={metrics?.wasserstein_mean ?? 0} precision={4} />
        <Statistic title="MMD" value={metrics?.mmd_statistic ?? 0} precision={4} />
        <Statistic title="p-value" value={metrics?.mmd_p_value ?? 0} precision={4} />
        <Statistic
          title="通信健康"
          value={metrics?.comm_health_score ?? 0}
          precision={3}
          suffix="/ 1"
        />
        <Statistic
          title="动力学异常"
          value={metrics?.dynamics_anomaly_score ?? 0}
          precision={3}
          suffix="/ 1"
        />
        {metrics?.soft_limit_triggered ? (
          <Tag color="error">软限位触发</Tag>
        ) : (
          <Statistic
            title="软限位接近"
            value={metrics?.soft_limit_score ?? 0}
            precision={3}
            suffix="/ 1"
          />
        )}
        <Badge
          status={metrics?.shift_detected ? 'warning' : 'success'}
          text={metrics?.shift_detected ? '偏移' : '正常'}
        />
      </Space>
    </div>
  );
}
