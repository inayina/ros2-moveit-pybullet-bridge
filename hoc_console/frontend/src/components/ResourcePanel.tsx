import { Progress, Space, Tag } from 'antd';
import { useDashboardStore } from '../stores/dashboardStore';

function boundedPercent(value: unknown): number {
  const numeric = Number(value ?? 0);
  return Math.max(0, Math.min(100, Number.isFinite(numeric) ? numeric : 0));
}

export function ResourcePanel() {
  const telemetry = useDashboardStore((state) => state.systemTelemetry);
  const recorder = useDashboardStore((state) => state.recorderDiagnostics);
  const history = useDashboardStore((state) => state.resourceHistory);
  const host = telemetry?.statuses.find((status) => status.name === 'system_telemetry/host');
  const recorderHealth = recorder?.statuses.find(
    (status) => status.name === 'lerobot_recorder/health',
  );
  const processes = telemetry?.statuses.filter(
    (status) => status.name.startsWith('system_telemetry/process/'),
  ) ?? [];
  const cpu = boundedPercent(host?.values.cpu_total_percent);
  const memory = boundedPercent(host?.values.memory_percent);
  const effectiveHz = Number(recorderHealth?.values.effective_hz ?? 0);
  const sceneAge = Number(recorderHealth?.values.scene_age_s ?? -1);
  const streams = Array.isArray(recorderHealth?.values.enabled_visual_streams)
    ? recorderHealth.values.enabled_visual_streams.map(String)
    : [];
  const peakCpu = history.reduce((value, sample) => Math.max(value, sample.cpuTotal), cpu);
  const activeHz = history.filter((sample) => sample.recorderHz > 0).map((sample) => sample.recorderHz);
  const minRecorderHz = activeHz.length ? Math.min(...activeHz) : effectiveHz;

  return (
    <section className="panel panel--resources">
      <div className="panel-title-row">
        <h3>录制与资源遥测</h3>
        <Tag color={cpu >= 85 ? 'red' : undefined}>{cpu.toFixed(1)}% CPU</Tag>
      </div>
      <div className="resource-bars">
        <div><span>系统 CPU</span><Progress percent={cpu} size="small" /></div>
        <div><span>系统内存</span><Progress percent={memory} size="small" /></div>
      </div>
      <Space wrap size={[6, 6]}>
        <Tag color={effectiveHz >= 8 ? undefined : 'orange'}>
          Recorder {effectiveHz.toFixed(1)} Hz
        </Tag>
        <Tag color={sceneAge >= 0 && sceneAge <= 0.5 ? undefined : 'orange'}>
          Scene age {sceneAge >= 0 ? sceneAge.toFixed(2) + ' s' : 'unseen'}
        </Tag>
        <Tag color={streams.includes('scene') ? undefined : 'default'}>Scene enabled</Tag>
        <Tag>Wrist disabled</Tag>
        <Tag>Tactile disabled</Tag>
      </Space>
      <div className="process-grid">
        {processes.map((status) => {
          const parts = status.name.split('/');
          const name = parts[parts.length - 1];
          const rawProcessCpu = Number(status.values.cpu_percent ?? 0);
          const processCpu = Number.isFinite(rawProcessCpu) ? Math.max(0, rawProcessCpu) : 0;
          return (
            <div key={status.name} className="process-row">
              <span>{name}</span>
              <strong>{processCpu.toFixed(1)}%</strong>
              <Progress percent={boundedPercent(processCpu)} size="small" showInfo={false} />
            </div>
          );
        })}
      </div>
      <p className="panel-caption">
        missing/stale/reused：{String(recorderHealth?.values.missing_rejects ?? 0)} /{' '}
        {String(recorderHealth?.values.stale_rejects ?? 0)} /{' '}
        {String(recorderHealth?.values.reused_rejects ?? 0)}
        {' '}· command missing {String(recorderHealth?.values.command_missing_rejects ?? 0)}
        {' '}· 60s peak CPU {peakCpu.toFixed(1)}% / min recorder {minRecorderHz.toFixed(1)} Hz
      </p>
    </section>
  );
}
