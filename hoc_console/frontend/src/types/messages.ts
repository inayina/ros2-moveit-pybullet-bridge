export interface RosTimestamp {
  sec: number;
  nanosec: number;
}

export interface RiskAttribution {
  dimension: string;
  raw_score: number;
  weighted_score: number;
  weight: number;
  is_primary_driver: boolean;
}

export interface RiskStatusPayload {
  level: number;
  level_name: string;
  composite_score: number;
  primary_driver: string;
  recommendation: string;
  e_stop_active: boolean;
  degraded_mode: boolean;
  attribution: RiskAttribution[];
}

export interface DistributionMetricsPayload {
  joint_names: string[];
  kl_divergence_per_joint: number[];
  kl_divergence_mean: number;
  mmd_statistic: number;
  mmd_p_value: number;
  mmd_threshold: number;
  window_duration_sec: number;
  sample_count_sim: number;
  sample_count_real: number;
  shift_detected: boolean;
  detection_method: string;
}

export interface DataFrame {
  type: 'data';
  topic: string;
  timestamp?: RosTimestamp;
  payload: RiskStatusPayload | DistributionMetricsPayload | Record<string, unknown>;
}

export interface LegacyFrame {
  type: string;
  timestamp?: RosTimestamp;
  payload: RiskStatusPayload | DistributionMetricsPayload | Record<string, unknown>;
}

export type WsFrame = DataFrame | LegacyFrame | { type: string; [key: string]: unknown };

export interface TrackingErrorPayload {
  joint_names: string[];
  errors: number[];
}

export interface AlertEvent {
  timestamp?: number;
  event_type?: string;
  from_level?: number;
  to_level?: number;
  primary_driver?: string;
  message?: string;
}

export interface MetricsHistoryPoint {
  t: number;
  kl_mean: number;
  mmd_stat: number;
}

export interface RiskHistoryPoint {
  t: number;
  level: number;
  score: number;
}

export type TrendDirection = 'up' | 'down' | 'stable';

export const DIMENSION_LABELS: Record<string, string> = {
  distribution_shift: '关节误差KL',
  tracking_error: '跟踪延迟',
  dynamics_anomaly: '负载变化率',
  comm_health: '通信健康',
  planning_failure: '重复精度',
};

export const RISK_COLORS: Record<number, string> = {
  0: '#52c41a',
  1: '#faad14',
  2: '#fa8c16',
  3: '#f5222d',
};

export const RISK_BG: Record<number, string> = {
  0: '#162312',
  1: '#2b2111',
  2: '#2b1d11',
  3: '#2a1215',
};
