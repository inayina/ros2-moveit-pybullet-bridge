import { Badge, Progress, Statistic, Tag } from 'antd';
import { useDashboardStore } from '../stores/dashboardStore';

export function GraspStatusPanel() {
  const grasp = useDashboardStore((s) => s.grasp);
  const confidence = Math.round((grasp?.confidence ?? 0) * 100);

  return (
    <div className="panel panel--grasp">
      <div className="panel-title-row">
        <h3>Panda 抓取 / 传感融合</h3>
        {!grasp ? (
          <Tag>等待 /bridge/sim/grasp_status</Tag>
        ) : grasp.object_slipped ? (
          <Tag color="error">检测到滑移</Tag>
        ) : grasp.grasp_established ? (
          <Tag color="success">接触已建立</Tag>
        ) : (
          <Tag color="processing">等待接触</Tag>
        )}
      </div>
      <div className="grasp-stats">
        <Statistic title="净接触力" value={grasp?.force_norm ?? 0} precision={2} suffix="N" />
        <Badge
          status={grasp?.object_slipped ? 'error' : grasp?.grasp_established ? 'success' : 'default'}
          text={grasp?.object_slipped ? 'SLIP' : grasp?.grasp_established ? 'GRASP' : 'OPEN'}
        />
      </div>
      <Progress
        percent={confidence}
        size="small"
        status={grasp?.object_slipped ? 'exception' : 'normal'}
        format={(value) => '置信度 ' + (value ?? 0) + '%'}
      />
      <p className="panel-caption">异频 joint / camera / FT 对齐后的接触与滑移估计；无消息时不伪造状态。</p>
    </div>
  );
}
