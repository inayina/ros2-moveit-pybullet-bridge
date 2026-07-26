import { Badge, Space, Statistic, Tag } from 'antd';
import { memo, useMemo } from 'react';
import { useDashboardStore } from '../stores/dashboardStore';
import { StableChart } from './StableChart';

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

export const DistributionPanel = memo(function DistributionPanel() {
  const metrics = useDashboardStore((s) => s.metrics);
  const metricValid = metrics?.metric_valid === true;

  const boxplotOption = useMemo(() => {
    const joints = metrics?.joint_names ?? [];
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
      animation: false,
      tooltip: { trigger: 'item' },
      legend: {
        data: ['PyBullet 执行', 'Reference 回放'],
        textStyle: { color: '#a4adb4' },
        top: 0,
      },
      grid: { left: 48, right: 16, top: 40, bottom: 48 },
      dataZoom: [
        { type: 'inside', xAxisIndex: 0, filterMode: 'none' },
        { type: 'slider', xAxisIndex: 0, height: 14, bottom: 6, textStyle: { color: '#a4adb4' } },
      ],
      xAxis: {
        type: 'category',
        data: joints.length ? joints : ['J1', 'J2', 'J3'],
        axisLabel: { color: '#a4adb4', fontSize: 10, rotate: joints.length > 5 ? 30 : 0 },
      },
      yAxis: {
        type: 'value',
        name: hasWindowBoxplot ? '关节位置 (rad)' : '等待窗口样本…',
        axisLabel: { color: '#a4adb4' },
        splitLine: { lineStyle: { color: '#485057' } },
      },
      series: [
        {
          name: 'PyBullet 执行',
          type: 'boxplot',
          data: simData.length ? simData : [],
          itemStyle: { color: '#5f7f9c', borderColor: '#7895ae' },
        },
        {
          name: 'Reference 回放',
          type: 'boxplot',
          data: realData.length ? realData : [],
          itemStyle: { color: '#7f8970', borderColor: '#98a28a' },
        },
      ],
    };
  }, [metrics]);

  const barOption = useMemo(() => {
    const joints = metrics?.joint_names ?? ['J1', 'J2', 'J3'];
    const kl = metricValid ? metrics?.kl_divergence_per_joint ?? [] : [];
    const w1 = metricValid ? metrics?.wasserstein_per_joint ?? [] : [];
    return {
      backgroundColor: 'transparent',
      animation: false,
      tooltip: { trigger: 'axis' },
      legend: {
        data: ['KL', 'W1'],
        textStyle: { color: '#a4adb4' },
        top: 0,
      },
      grid: { left: 40, right: 10, top: 28, bottom: 30 },
      dataZoom: [
        { type: 'inside', xAxisIndex: 0, filterMode: 'none' },
      ],
      xAxis: { type: 'category', data: joints, axisLabel: { color: '#a4adb4', fontSize: 10 } },
      yAxis: { type: 'value', axisLabel: { color: '#a4adb4', fontSize: 10 }, splitLine: { show: false } },
      series: [
        {
          name: 'KL',
          type: 'bar',
          data: kl,
          itemStyle: { color: '#5f7f9c' },
        },
        {
          name: 'W1',
          type: 'bar',
          data: w1,
          itemStyle: { color: '#887a91' },
        },
      ],
    };
  }, [metricValid, metrics]);

  const unavailable = 'unavailable';

  return (
    <div className="panel panel--distribution">
      <Space style={{ width: '100%', justifyContent: 'space-between' }}>
        <h3>Panda Runtime / Reference 分布</h3>
        {!metricValid ? (
          <Tag>{metrics?.validity ?? 'NO DATA'} · {metrics?.reason_code ?? 'no_data'}</Tag>
        ) : metrics?.shift_detected ? (
          <Tag color="warning">⚠ 检出偏移 ({metrics.detection_method})</Tag>
        ) : (
          <Tag>未检出偏移</Tag>
        )}
      </Space>
      <p className="distribution-window">
        WINDOW {metrics?.window_duration_sec?.toFixed(1) ?? '—'} s
        {' · '}EXECUTION n={metrics?.sample_count_sim ?? 0}
        {' · '}REFERENCE n={metrics?.sample_count_real ?? 0}
      </p>
      <div className="distribution-charts">
        <StableChart option={boxplotOption} height={230} className="chart-boxplot" />
        <StableChart option={barOption} height={170} className="chart-bars" />
      </div>
      <div className="distribution-stats">
        <Statistic title="KL mean" value={metricValid ? metrics?.kl_divergence_mean ?? unavailable : unavailable} precision={metricValid ? 4 : undefined} />
        <Statistic title="W1 mean" value={metricValid ? metrics?.wasserstein_mean ?? unavailable : unavailable} precision={metricValid ? 4 : undefined} />
        <Statistic title="MMD" value={metricValid ? metrics?.mmd_statistic ?? unavailable : unavailable} precision={metricValid ? 4 : undefined} />
        <Statistic title="p-value" value={metricValid ? metrics?.mmd_p_value ?? unavailable : unavailable} precision={metricValid ? 4 : undefined} />
        <Statistic
          title="通信健康"
          value={metrics?.comm_health_valid ? metrics.comm_health_score : unavailable}
          precision={3}
          suffix="/ 1"
        />
        <Statistic
          title="动力学异常"
          value={metrics?.dynamics_valid ? metrics.dynamics_anomaly_score : unavailable}
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
          color={!metricValid ? '#69727a' : metrics?.shift_detected ? '#c28a2c' : '#7f8b92'}
          text={!metricValid ? '指标不可用' : metrics?.shift_detected ? '偏移' : '正常'}
        />
        <Tag>{metrics?.calibration_id || 'calibration unavailable'}</Tag>
      </div>
    </div>
  );
});
