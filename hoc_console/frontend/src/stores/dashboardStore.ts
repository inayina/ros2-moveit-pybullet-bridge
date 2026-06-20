import { create } from 'zustand';
import type {
  AlertEvent,
  DistributionMetricsPayload,
  ExperimentProgressPayload,
  MetricsHistoryPoint,
  RiskHistoryPoint,
  RiskStatusPayload,
  TrackingErrorPayload,
  TrendDirection,
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
  setConnected: (connected: boolean) => void;
  ingestRisk: (payload: RiskStatusPayload) => void;
  ingestMetrics: (payload: DistributionMetricsPayload) => void;
  ingestTracking: (payload: TrackingErrorPayload) => void;
  ingestAlert: (payload: AlertEvent) => void;
  setRecording: (recording: boolean, bagPath?: string) => void;
  dismissR3Modal: () => void;
  resetR3Modal: () => void;
  ingestExperimentProgress: (payload: ExperimentProgressPayload) => void;
  setCameraFrame: (dataUrl: string | null) => void;
  setSystemState: (state: string) => void;
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
}));
