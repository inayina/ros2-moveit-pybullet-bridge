# HOC Frontend

React 18 + TypeScript + Vite dashboard for Sim2Real monitoring.

## Setup

```bash
cd hoc_console/frontend
npm install
npm run dev
```

Open http://localhost:5173 — WebSocket connects to `ws://localhost:8765`.

## Stack

- React 18 + TypeScript + Vite
- Ant Design 5 (dark theme)
- ECharts 5 (radar, boxplot, line)
- Zustand (real-time state)
- html2canvas (report screenshots)

## Panels

| Panel | Description |
|-------|-------------|
| RiskBanner | R0–R3 level, trend arrow, primary driver |
| RiskRadar | Five-dimensional risk radar |
| DistributionPanel | Sim/Real error boxplot per joint |
| TrendChart | KL/MMD 60s rolling trend |
| ExperimentControl | Scenario run, record, export HTML report |
| R3Modal | Full-screen alert for R3 with acknowledge flow |

See [04-hoc-console-design.md](../../docs/design/04-hoc-console-design.md).
