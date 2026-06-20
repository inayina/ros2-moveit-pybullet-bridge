import { ArrowDownOutlined, ArrowUpOutlined, MinusOutlined } from '@ant-design/icons';
import { Badge, Space, Typography } from 'antd';
import { memo } from 'react';
import { useDashboardStore } from '../stores/dashboardStore';
import { SessionClock } from './SessionClock';
import { DIMENSION_LABELS, RISK_BG, RISK_COLORS } from '../types/messages';

const { Text, Title } = Typography;

const DRIVER_LABELS: Record<string, string> = {
  distribution_shift: '分布偏移',
  tracking_error: '跟踪误差',
  dynamics_anomaly: '动力学异常',
  comm_health: '通信异常',
  planning_failure: '规划失败',
  ...DIMENSION_LABELS,
};

export const RiskBanner = memo(function RiskBanner() {
  const risk = useDashboardStore((s) => s.risk);
  const trend = useDashboardStore((s) => s.trend);
  const connected = useDashboardStore((s) => s.connected);
  const sessionStart = useDashboardStore((s) => s.sessionStart);
  const systemState = useDashboardStore((s) => s.systemState);

  const level = risk?.level ?? 0;
  const blink = level >= 2;

  const TrendIcon =
    trend === 'up' ? ArrowUpOutlined : trend === 'down' ? ArrowDownOutlined : MinusOutlined;

  return (
    <div
      className={`risk-banner ${blink ? 'risk-banner--blink' : ''}`}
      style={{
        borderColor: RISK_COLORS[level] ?? RISK_COLORS[0],
        background: RISK_BG[level] ?? RISK_BG[0],
        ['--risk-accent' as string]: RISK_COLORS[level] ?? RISK_COLORS[0],
      }}
    >
      <Space size="large" wrap>
        <Title level={4} style={{ margin: 0, color: '#e8e8e8' }}>
          ◉ Sim2Real Monitor
        </Title>
        <Badge
          status={connected ? 'processing' : 'error'}
          text={connected ? 'WS 已连接' : 'WS 断开'}
        />
        <Space>
          <Text style={{ color: RISK_COLORS[level], fontSize: 18, fontWeight: 700 }}>
            {risk?.level_name ?? 'R0'} {level >= 1 ? '关注' : '正常'}
          </Text>
          <Text>综合风险 {(risk?.composite_score ?? 0).toFixed(2)}</Text>
          <TrendIcon style={{ color: trend === 'up' ? '#ff7875' : '#95de64' }} />
        </Space>
        <Text type="secondary">
          主因: {DRIVER_LABELS[risk?.primary_driver ?? ''] ?? risk?.primary_driver ?? '—'}
        </Text>
        {risk?.degraded_mode ? (
          <Text type="warning">降级运行 · 速度 50%</Text>
        ) : null}
        {systemState === 'HOLD' ? (
          <Text type="warning">看门狗 HOLD · 位置保持</Text>
        ) : null}
        {systemState === 'E_STOP' ? (
          <Text type="danger">急停 E_STOP</Text>
        ) : null}
        <SessionClock sessionStart={sessionStart} />
      </Space>
    </div>
  );
});
