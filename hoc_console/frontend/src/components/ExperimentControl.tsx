import { DownloadOutlined, ExperimentOutlined, PlayCircleOutlined, VideoCameraOutlined } from '@ant-design/icons';
import { Button, InputNumber, Select, Slider, Space, message } from 'antd';
import { useState } from 'react';
import { useDashboardStore } from '../stores/dashboardStore';

const SCENARIOS = [
  { value: 'SC-01', label: 'SC-01 点到点' },
  { value: 'SC-02', label: 'SC-02 轨迹跟踪' },
  { value: 'SC-03', label: 'SC-03 碰撞规避' },
  { value: 'SC-04', label: 'SC-04 域随机化扫描' },
  { value: 'SC-05', label: 'SC-05 急停恢复' },
];

interface ExperimentControlProps {
  sendCommand: (action: string, params?: Record<string, unknown>) => Promise<{
    success: boolean;
    message?: string;
    file_path?: string;
  }>;
  dashboardRef: React.RefObject<HTMLDivElement>;
}

async function captureScreenshot(el: HTMLElement | null): Promise<string | undefined> {
  if (!el) return undefined;
  try {
    const html2canvas = (await import('html2canvas')).default;
    const canvas = await html2canvas(el, {
      backgroundColor: '#0a0a0a',
      scale: 1,
      logging: false,
    });
    return canvas.toDataURL('image/png').split(',')[1];
  } catch {
    return undefined;
  }
}

export function ExperimentControl({ sendCommand, dashboardRef }: ExperimentControlProps) {
  const [scenario, setScenario] = useState('SC-01');
  const [seed, setSeed] = useState(42);
  const [strengthPct, setStrengthPct] = useState(50);
  const [loading, setLoading] = useState<string | null>(null);
  const recording = useDashboardStore((s) => s.recording);
  const bagPath = useDashboardStore((s) => s.bagPath);
  const setRecording = useDashboardStore((s) => s.setRecording);

  const run = async (action: string, params: Record<string, unknown> = {}) => {
    setLoading(action);
    const result = await sendCommand(action, params);
    setLoading(null);
    if (result.success) {
      message.success(result.message ?? `${action} 成功`);
    } else {
      message.error(result.message ?? `${action} 失败`);
    }
    return result;
  };

  return (
    <div className="panel panel--wide experiment-control">
      <h3>实验控制</h3>
      <Space wrap size="middle" align="center">
        <span>场景</span>
        <Select
          value={scenario}
          onChange={setScenario}
          options={SCENARIOS}
          style={{ width: 200 }}
        />
        <span>种子</span>
        <InputNumber value={seed} onChange={(v) => setSeed(v ?? 42)} min={0} max={99999} />
        <span>随机化</span>
        <Slider
          min={0}
          max={100}
          value={strengthPct}
          onChange={setStrengthPct}
          onChangeComplete={(v) =>
            run('set_randomization', { seed, strength: v / 100 })
          }
          style={{ width: 160 }}
        />
        <span>{strengthPct}%</span>
        <Button
          type="primary"
          icon={<PlayCircleOutlined />}
          loading={loading === 'start_experiment'}
          onClick={() =>
            run('start_experiment', {
              scenario_id: scenario,
              seed,
              strength: strengthPct / 100,
            })
          }
        >
          开始实验
        </Button>
        <Button
          icon={<VideoCameraOutlined />}
          type={recording ? 'primary' : 'default'}
          danger={recording}
          loading={loading === 'start_recording' || loading === 'stop_recording'}
          onClick={async () => {
            if (recording) {
              const res = await run('stop_recording');
              if (res.success) setRecording(false, bagPath);
            } else {
              const res = await run('start_recording');
              if (res.success) setRecording(true);
            }
          }}
        >
          {recording ? '停止录制' : '开始录制'}
        </Button>
        <Button
          icon={<ExperimentOutlined />}
          loading={loading === 'inject_shift'}
          onClick={() =>
            run('inject_shift', {
              parameter: 'joint_damping',
              delta_percent: 30,
              duration: 10,
            })
          }
        >
          注入偏移
        </Button>
        <Button
          icon={<DownloadOutlined />}
          loading={loading === 'export_report'}
          onClick={async () => {
            const screenshot_b64 = await captureScreenshot(dashboardRef.current);
            const res = await run('export_report', {
              format: 'html',
              screenshot_b64,
            });
            if (res.success && res.file_path) {
              message.info(`报告已导出: ${res.file_path}`);
            }
          }}
        >
          导出报告
        </Button>
      </Space>
    </div>
  );
}