import { List, Tag, Typography } from 'antd';
import { useDashboardStore } from '../stores/dashboardStore';
import { RISK_COLORS } from '../types/messages';

const { Text } = Typography;

function formatTime(ts?: number): string {
  if (!ts) return '--:--:--';
  const d = new Date(ts * 1000);
  return d.toLocaleTimeString('zh-CN', { hour12: false });
}

export function AlertTimeline() {
  const alerts = useDashboardStore((s) => s.alerts);

  return (
    <div className="panel panel--wide">
      <h3>告警时间线</h3>
      <List
        size="small"
        locale={{ emptyText: '暂无告警事件' }}
        dataSource={alerts}
        renderItem={(item) => {
          const level = item.to_level ?? 0;
          const isLevelChange = item.event_type === 'level_change';
          return (
            <List.Item>
              <Text type="secondary">{formatTime(item.timestamp)}</Text>
              <span style={{ margin: '0 12px' }}>
                {isLevelChange ? (
                  <Tag color={RISK_COLORS[item.from_level ?? 0]}>R{item.from_level}</Tag>
                ) : null}
                {isLevelChange ? '→' : null}
                {isLevelChange ? (
                  <Tag color={RISK_COLORS[level]}>R{level}</Tag>
                ) : (
                  <Tag>INFO</Tag>
                )}
              </span>
              <Text>
                {item.primary_driver ?? item.event_type ?? 'alert'}
                {item.message ? ` — ${item.message}` : ''}
              </Text>
            </List.Item>
          );
        }}
      />
    </div>
  );
}
