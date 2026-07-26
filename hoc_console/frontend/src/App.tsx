import { ConfigProvider, Layout, Space, Button, Tabs, theme, message } from 'antd';
import zhCN from 'antd/locale/zh_CN';
import { useEffect, useRef } from 'react';
import { AlertTimeline } from './components/AlertTimeline';
import { CanonicalRunPanel } from './components/CanonicalRunPanel';
import { DistributionPanel } from './components/DistributionPanel';
import { EStopButton } from './components/EStopButton';
import { ExperimentControl } from './components/ExperimentControl';
import { GraspStatusPanel } from './components/GraspStatusPanel';
import { RobotCameraPanel } from './components/RobotCameraPanel';
import { R3Modal } from './components/R3Modal';
import { RuntimeOverview } from './components/RuntimeOverview';
import { RiskRadar } from './components/RiskRadar';
import { ResourcePanel } from './components/ResourcePanel';
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

  const handleResume = async () => {
    if (level >= 3) {
      message.warning('请先确认 R3 告警后再恢复');
      return;
    }
    const res = await sendCommand('resume');
    if (res.success) {
      message.success(res.message ?? '系统已恢复');
    } else {
      message.error(res.message ?? '恢复失败');
    }
  };

  useEffect(() => {
    document.documentElement.dataset.riskLevel = String(level);
  }, [level]);

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
        token: {
          colorPrimary: '#6485a3',
          colorSuccess: '#7f9185',
          colorWarning: '#c28a2c',
          colorError: '#b84f4f',
          colorBgBase: '#202428',
          colorBgContainer: '#2a2f34',
          colorBorder: '#4a5158',
          colorText: '#e1e5e8',
          colorTextSecondary: '#a4adb4',
          borderRadius: 6,
        },
      }}
    >
      <Layout className="app-shell">
        <div ref={dashboardRef} className="dashboard-root">
          <RuntimeOverview />
          <Content className="dashboard-content">
            <Tabs
              className="hoc-tabs"
              defaultActiveKey="overview"
              tabBarExtraContent={(
                <Space className="toolbar">
                  <EStopButton onEStop={() => sendCommand('e_stop')} />
                  <Button onClick={() => sendCommand('pause')}>暂停</Button>
                  <Button onClick={handleResume}>恢复</Button>
                </Space>
              )}
              items={[
                {
                  key: 'overview',
                  label: 'Runtime Overview',
                  children: (
                    <div className="dashboard-grid dashboard-grid--3 overview-grid">
                      <RiskRadar />
                      <DistributionPanel />
                      <TrackingChart />
                    </div>
                  ),
                },
                {
                  key: 'diagnostics',
                  label: 'Diagnostics',
                  children: (
                    <div className="dashboard-grid diagnostics-grid">
                      <div className="dashboard-stack">
                        <RobotCameraPanel />
                        <GraspStatusPanel />
                      </div>
                      <div className="dashboard-stack">
                        <TrendChart />
                        <ResourcePanel />
                        <AlertTimeline />
                      </div>
                    </div>
                  ),
                },
                {
                  key: 'historical',
                  label: 'Historical / Evidence',
                  children: (
                    <div className="historical-tab">
                      <section className="historical-evidence">
                        <h2>Historical Evidence</h2>
                        <CanonicalRunPanel />
                      </section>
                      <ExperimentControl sendCommand={sendCommand} dashboardRef={dashboardRef} />
                    </div>
                  ),
                },
              ]}
            />
          </Content>
        </div>
        <R3Modal sendCommand={sendCommand} />
      </Layout>
    </ConfigProvider>
  );
}

export default App;
