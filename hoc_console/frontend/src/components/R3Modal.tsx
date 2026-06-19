import { ExclamationCircleOutlined } from '@ant-design/icons';
import { Input, Modal, Space, Typography } from 'antd';
import { useState } from 'react';
import { useDashboardStore } from '../stores/dashboardStore';

const { Text, Paragraph } = Typography;

interface R3ModalProps {
  sendCommand: (action: string, params?: Record<string, unknown>) => Promise<{
    success: boolean;
    message?: string;
  }>;
}

export function R3Modal({ sendCommand }: R3ModalProps) {
  const risk = useDashboardStore((s) => s.risk);
  const dismissed = useDashboardStore((s) => s.r3ModalDismissed);
  const dismissR3Modal = useDashboardStore((s) => s.dismissR3Modal);
  const [operatorId, setOperatorId] = useState('');
  const [comment, setComment] = useState('');
  const [loading, setLoading] = useState(false);

  const level = risk?.level ?? 0;
  const visible = level >= 3 && !dismissed;

  const handleAck = async () => {
    setLoading(true);
    const res = await sendCommand('acknowledge', {
      operator_id: operatorId || 'operator',
      comment,
      from_level: 3,
      to_level: 2,
    });
    if (res.success) {
      dismissR3Modal();
      const resume = await sendCommand('resume');
      if (!resume.success) {
        setLoading(false);
        return;
      }
    }
    setLoading(false);
  };

  return (
    <Modal
      open={visible}
      title={
        <Space>
          <ExclamationCircleOutlined style={{ color: '#f5222d' }} />
          <span>严重风险 — 系统已急停</span>
        </Space>
      }
      closable={false}
      maskClosable={false}
      okText="确认已知悉"
      cancelText="暂不恢复"
      confirmLoading={loading}
      onOk={handleAck}
      onCancel={() => dismissR3Modal()}
      width={480}
      className="r3-modal"
    >
      <Paragraph>
        综合得分: <Text strong>{(risk?.composite_score ?? 0).toFixed(2)}</Text>
      </Paragraph>
      <Paragraph>
        主因: <Text type="danger">{risk?.primary_driver ?? '—'}</Text>
      </Paragraph>
      <Paragraph type="secondary">{risk?.recommendation ?? '请检查系统状态后再恢复。'}</Paragraph>
      <Space direction="vertical" style={{ width: '100%' }}>
        <Input
          placeholder="操作员 ID"
          value={operatorId}
          onChange={(e) => setOperatorId(e.target.value)}
        />
        <Input.TextArea
          placeholder="备注"
          value={comment}
          onChange={(e) => setComment(e.target.value)}
          rows={2}
        />
      </Space>
    </Modal>
  );
}
