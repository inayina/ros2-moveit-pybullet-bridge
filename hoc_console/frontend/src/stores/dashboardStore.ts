import { create } from 'zustand';
import type {
  AlertEvent,
  DistributionMetricsPayload,
  DiagnosticArrayPayload,
  ExperimentProgressPayload,
  GraspStatusPayload,
  MetricsHistoryPoint,
  RiskHistoryPoint,
  RiskStatusPayload,
  TrackingErrorPayload,
  TrendDirection,
  ResourceHistoryPoint,
  RuntimeFramePayload,
} from '../types/messages';
import { shouldThrottle } from '../utils/throttle';

const HISTORY_SECONDS = 60;
const METRICS_THROTTLE_MS = 300;
const TRACKING_THROTTLE_MS = 300;

interface DashboardState {
  connected: boolean;
  lastMessageAt: number | null;
  risk: RiskStatusPayload | null;
  metrics: DistributionMetricsPayload | null;
  tracking: TrackingErrorPayload | null;
  grasp: GraspStatusPayload | null;
  alerts: AlertEvent[];
  riskHistory: RiskHistoryPoint[];
  metricsHistory: MetricsHistoryPoint[];
  trend: TrendDirection;
  recording: boolean;
  bagPath: string;
  r3ModalDismissed: boolean;
  sessionStart: number;
  experiment: ExperimentProgressPayload | null;
  cameraFrame: string | null;
  systemState: string;
  systemTelemetry: DiagnosticArrayPayload | null;
  recorderDiagnostics: DiagnosticArrayPayload | null;
  resourceHistory: ResourceHistoryPoint[];
  runtimeFrame: RuntimeFramePayload | null;
  runtimeHistory: Array<{ t: number; frame: RuntimeFramePayload }>;
  setConnected: (connected: boolean) => void;
  ingestRisk: (payload: RiskStatusPayload) => void;
  ingestMetrics: (payload: DistributionMetricsPayload) => void;
  ingestTracking: (payload: TrackingErrorPayload) => void;
  ingestGrasp: (payload: GraspStatusPayload) => void;
  ingestAlert: (payload: AlertEvent) => void;
  setRecording: (recording: boolean, bagPath?: string) => void;
  dismissR3Modal: () => void;
  resetR3Modal: () => void;
  ingestExperimentProgress: (payload: ExperimentProgressPayload) => void;
  setCameraFrame: (dataUrl: string | null) => void;
  setSystemState: (state: string) => void;
  ingestSystemTelemetry: (payload: DiagnosticArrayPayload) => void;
  ingestRecorderDiagnostics: (payload: DiagnosticArrayPayload) => void;
  ingestRuntimeFrame: (payload: RuntimeFramePayload) => void;
}

function computeTrend(history: RiskHistoryPoint[]): TrendDirection {
  if (history.length < 2) return 'stable';
  const now = history[history.length - 1]?.score ?? 0;
  const cutoff = Date.now() / 1000 - 30;
  const old = [...history].reverse().find((p) => p.t <= cutoff);
  if (!old) return 'stable';
  const delta = now - old.score;
  if (delta > 0.03) return 'up';
  if (delta < -0.03) return 'down';
  return 'stable';
}

export const useDashboardStore = create<DashboardState>((set, get) => ({
  connected: false,
  lastMessageAt: null,
  risk: null,
  metrics: null,
  tracking: null,
  grasp: null,
  alerts: [],
  riskHistory: [],
  metricsHistory: [],
  trend: 'stable',
  recording: false,
  bagPath: '',
  r3ModalDismissed: false,
  sessionStart: Date.now(),
  experiment: null,
  cameraFrame: null,
  systemState: 'RUNNING',
  systemTelemetry: null,
  recorderDiagnostics: null,
  resourceHistory: [],
  runtimeFrame: null,
  runtimeHistory: [],

  setConnected: (connected) => set({ connected }),

  ingestRisk: (payload) => {
    const t = Date.now() / 1000;
    const attribution: Record<string, number> = {};
    for (const a of payload.attribution) {
      attribution[a.dimension] = a.raw_score;
    }
    const point: RiskHistoryPoint = {
      t,
      level: payload.level,
      score: payload.composite_score,
      attribution,
    };
    const riskHistory = [...get().riskHistory, point].filter((p) => p.t >= t - HISTORY_SECONDS);
    const trend = computeTrend(riskHistory);
    const updates: Partial<DashboardState> = {
      risk: payload,
      riskHistory,
      trend,
      lastMessageAt: Date.now(),
    };
    if (payload.level >= 3) {
      updates.r3ModalDismissed = false;
    }
    set(updates);
  },

  ingestMetrics: (payload) => {
    if (shouldThrottle('metrics', METRICS_THROTTLE_MS)) {
      return;
    }
    if (payload.metric_valid !== true) {
      set({ metrics: payload, lastMessageAt: Date.now() });
      return;
    }
    const t = Date.now() / 1000;
    const point: MetricsHistoryPoint = {
      t,
      kl_mean: payload.kl_divergence_mean,
      w1_mean: payload.wasserstein_mean,
      mmd_stat: payload.mmd_statistic,
      comm_health_score: payload.comm_health_score ?? 0,
    };
    const metricsHistory = [...get().metricsHistory, point].filter(
      (p) => p.t >= t - HISTORY_SECONDS,
    );
    set({
      metrics: payload,
      metricsHistory,
      lastMessageAt: Date.now(),
    });
  },

  ingestTracking: (payload) => {
    if (shouldThrottle('tracking', TRACKING_THROTTLE_MS)) {
      return;
    }
    set({ tracking: payload, lastMessageAt: Date.now() });
  },

  ingestGrasp: (payload) => set({ grasp: payload, lastMessageAt: Date.now() }),

  ingestAlert: (payload) =>
    set((state) => ({
      alerts: [{ ...payload, timestamp: payload.timestamp ?? Date.now() / 1000 }, ...state.alerts].slice(
        0,
        50,
      ),
    })),

  setRecording: (recording, bagPath = '') =>
    set({ recording, bagPath: bagPath || get().bagPath }),

  dismissR3Modal: () => set({ r3ModalDismissed: true }),

  resetR3Modal: () => set({ r3ModalDismissed: false }),

  ingestExperimentProgress: (payload) =>
    set({ experiment: payload, lastMessageAt: Date.now() }),

  setCameraFrame: (cameraFrame) => set({ cameraFrame }),

  setSystemState: (systemState) => set({ systemState }),

  ingestSystemTelemetry: (systemTelemetry) => {
    const host = systemTelemetry.statuses.find((status) => status.name === 'system_telemetry/host');
    const cpuTotal = Number(host?.values.cpu_total_percent ?? 0);
    const recorder = get().recorderDiagnostics?.statuses.find(
      (status) => status.name === 'lerobot_recorder/health',
    );
    const point: ResourceHistoryPoint = {
      t: Date.now() / 1000,
      cpuTotal,
      recorderHz: Number(recorder?.values.effective_hz ?? 0),
      sceneAge: Number(recorder?.values.scene_age_s ?? -1),
    };
    const resourceHistory = [...get().resourceHistory, point].filter(
      (sample) => sample.t >= point.t - HISTORY_SECONDS,
    );
    set({ systemTelemetry, resourceHistory, lastMessageAt: Date.now() });
  },

  ingestRecorderDiagnostics: (recorderDiagnostics) => {
    set({ recorderDiagnostics, lastMessageAt: Date.now() });
  },

  ingestRuntimeFrame: (runtimeFrame) => {
    const t = Date.now() / 1000;
    const runtimeHistory = [...get().runtimeHistory, { t, frame: runtimeFrame }]
      .filter((point) => point.t >= t - HISTORY_SECONDS)
      .slice(-1200);
    set({ runtimeFrame, runtimeHistory, lastMessageAt: Date.now() });
  },
}));
