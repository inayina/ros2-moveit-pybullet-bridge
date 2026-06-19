import { ConfigProvider, Layout, Space, Button, theme } from 'antd';
import zhCN from 'antd/locale/zh_CN';
import { useEffect, useRef } from 'react';
import { AlertTimeline } from './components/AlertTimeline';
import { DistributionPanel } from './components/DistributionPanel';
import { EStopButton } from './components/EStopButton';
import { ExperimentControl } from './components/ExperimentControl';
import { R3Modal } from './components/R3Modal';
import { RiskBanner } from './components/RiskBanner';
import { RiskRadar } from './components/RiskRadar';
import { TrendChart } from './components/TrendChart';
import { TrackingChart } from './components/TrackingChart';
import { useWebSocket } from './hooks/useWebSocket';
import { useDashboardStore } from './stores/dashboardStore';
import './App.css';

const { Content } = Layout;

function App() {
  const dashboardRef = useRef<HTMLDivElement>(null);
  const { sendCommand } = useWebSocket();
  const risk = useDashboardStore((s) => s.risk);
  const level = risk?.level ?? 0;

  useEffect(() => {
    const handler = (e: KeyboardEvent) => {
      if (e.code !== 'Space' || e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) {
        return;
      }
      e.preventDefault();
      sendCommand('e_stop');
    };
    window.addEventListener('keydown', handler);
    return () => window.removeEventListener('keydown', handler);
  }, [sendCommand]);

  return (
    <ConfigProvider
      locale={zhCN}
      theme={{
        algorithm: theme.darkAlgorithm,
        token: { colorPrimary: '#1677ff', borderRadius: 8 },
      }}
    >
      <Layout className={`app-shell app-shell--r${level}`}>
        <div ref={dashboardRef} className="dashboard-root">
          <RiskBanner />
          <Content className="dashboard-content">
            <Space className="toolbar" wrap>
              <EStopButton onEStop={() => sendCommand('e_stop')} />
              <Button onClick={() => sendCommand('resume')}>恢复</Button>
            </Space>
            <div className="dashboard-grid dashboard-grid--3">
              <RiskRadar />
              <DistributionPanel />
              <TrackingChart />
            </div>
            <TrendChart />
            <ExperimentControl sendCommand={sendCommand} dashboardRef={dashboardRef} />
            <AlertTimeline />
          </Content>
        </div>
        <R3Modal sendCommand={sendCommand} />
      </Layout>
    </ConfigProvider>
  );
}

export default App;
