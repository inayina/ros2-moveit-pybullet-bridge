export interface RosTimestamp {
  sec: number;
  nanosec: number;
}

export interface ExperimentProgressPayload {
  experiment_id: string;
  scenario_id: string;
  progress: number;
  current_phase: string;
  current_risk?: RiskStatusPayload;
  current_metrics?: DistributionMetricsPayload;
}

export interface RiskAttribution {
  dimension: string;
  raw_score: number;
  weighted_score: number;
  weight: number;
  is_primary_driver: boolean;
  source_valid?: boolean;
  validity?: string;
  reason_code?: string;
  provenance?: string;
}

export interface RiskStatusPayload {
  validity?: string;
  reason_code?: string;
  has_valid_sources?: boolean;
  active_dimensions?: string[];
  invalid_dimensions?: string[];
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
  validity?: string;
  reason_code?: string;
  metric_valid?: boolean;
  baseline_ready?: boolean;
  calibration_id?: string;
  reference_source?: string;
  aligned_sample_count?: number;
  comm_health_valid?: boolean;
  dynamics_valid?: boolean;
  joint_names: string[];
  kl_divergence_per_joint: number[];
  kl_divergence_mean: number;
  wasserstein_per_joint: number[];
  wasserstein_mean: number;
  w1_threshold: number;
  shift_detected_w1: boolean;
  mmd_statistic: number;
  mmd_p_value: number;
  mmd_threshold: number;
  window_duration_sec: number;
  sample_count_sim: number;
  sample_count_real: number;
  sim_position_min_per_joint: number[];
  sim_position_q1_per_joint: number[];
  sim_position_median_per_joint: number[];
  sim_position_q3_per_joint: number[];
  sim_position_max_per_joint: number[];
  real_position_min_per_joint: number[];
  real_position_q1_per_joint: number[];
  real_position_median_per_joint: number[];
  real_position_q3_per_joint: number[];
  real_position_max_per_joint: number[];
  shift_detected: boolean;
  detection_method: string;
  /** D4 通信健康 [0,1]，越高越差 */
  comm_health_score?: number;
  /** D3 动力学异常 [0,1] */
  dynamics_anomaly_score?: number;
  velocity_jump_per_joint?: number[];
  /** 软限位接近度 [0,1] */
  soft_limit_score?: number;
  soft_limit_triggered?: boolean;
}

export interface DataFrame {
  type: 'data';
  topic: string;
  timestamp?: RosTimestamp;
  payload: RiskStatusPayload | DistributionMetricsPayload | Record<string, unknown>;
}

export type RuntimeValidity =
  | 'VALID'
  | 'DEGRADED'
  | 'WARMING_UP'
  | 'STALE'
  | 'UNAVAILABLE'
  | 'ERROR';

export interface RuntimeLanePayload {
  lane: 'brain' | 'execution' | 'safety' | 'task_gt';
  validity: RuntimeValidity | string;
  reason_code: string;
  age_ms?: number | null;
  trace_run_id?: string;
  episode_id?: string;
  event_id?: string;
  parent_event_id?: string;
  [key: string]: unknown;
}

export interface RuntimeFramePayload {
  lanes: {
    brain: RuntimeLanePayload;
    execution: RuntimeLanePayload;
    safety: RuntimeLanePayload;
    task_gt: RuntimeLanePayload;
  };
  correlation: {
    trace_run_ids: string[];
    trace_consistent: boolean;
    execution_event_id?: string | null;
    execution_parent_event_id?: string | null;
    task_event_id?: string | null;
    task_parent_event_id?: string | null;
  };
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

export interface GraspStatusPayload {
  grasp_established: boolean;
  object_slipped: boolean;
  force_xyz: number[];
  torque_xyz: number[];
  force_norm: number;
  confidence: number;
}

export interface DiagnosticStatusPayload {
  name: string;
  level: number;
  message: string;
  hardware_id: string;
  values: Record<string, unknown>;
}

export interface DiagnosticArrayPayload {
  statuses: DiagnosticStatusPayload[];
}

export interface ResourceHistoryPoint {
  t: number;
  cpuTotal: number;
  recorderHz: number;
  sceneAge: number;
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
  w1_mean: number;
  mmd_stat: number;
  comm_health_score: number;
}

export interface RiskHistoryPoint {
  t: number;
  level: number;
  score: number;
  attribution: Record<string, number>;
}

export type TrendDirection = 'up' | 'down' | 'stable';

export const DIMENSION_LABELS: Record<string, string> = {
  distribution_shift: '分布偏移',
  tracking_error: '跟踪误差',
  dynamics_anomaly: '动力学异常',
  comm_health: '通信健康',
  planning_failure: '规划失败',
  resource_pressure: '资源压力',
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
