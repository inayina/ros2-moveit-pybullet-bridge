import type { Page } from '@playwright/test';

const AXES = [
  'distribution_shift',
  'tracking_error',
  'dynamics_anomaly',
  'comm_health',
  'planning_failure',
];

function riskPayload(level: number, score: number) {
  return {
    validity: 'VALID',
    reason_code: 'none',
    has_valid_sources: true,
    active_dimensions: AXES,
    invalid_dimensions: [],
    level,
    level_name: `R${level}`,
    composite_score: score,
    primary_driver: 'distribution_shift',
    recommendation: '系统运行正常',
    e_stop_active: false,
    degraded_mode: false,
    attribution: AXES.map((dimension, idx) => ({
      dimension,
      raw_score: Math.min(0.15 + idx * 0.05 + score * 0.1, 1),
      weighted_score: 0.05,
      weight: 0.2,
      is_primary_driver: idx === 0,
    })),
  };
}

function metricsPayload(tick: number) {
  const joints = ['panda_joint1', 'panda_joint2', 'panda_joint3'];
  const wave = Math.sin(tick * 0.2) * 0.02;
  return {
    validity: 'VALID',
    reason_code: 'none',
    metric_valid: true,
    baseline_ready: true,
    calibration_id: 'mock_same_scene',
    reference_source: 'topic',
    aligned_sample_count: 118 + tick,
    comm_health_valid: true,
    dynamics_valid: true,
    joint_names: joints,
    kl_divergence_per_joint: joints.map((_, i) => 0.04 + wave + i * 0.01),
    kl_divergence_mean: 0.05 + wave,
    wasserstein_per_joint: joints.map((_, i) => 0.03 + wave + i * 0.008),
    wasserstein_mean: 0.04 + wave,
    w1_threshold: 0.1,
    shift_detected_w1: false,
    mmd_statistic: 0.12 + wave,
    mmd_p_value: 0.4,
    mmd_threshold: 0.2,
    window_duration_sec: 10,
    sample_count_sim: 120 + tick,
    sample_count_real: 118 + tick,
    sim_position_min_per_joint: [-0.5, -0.4, -0.3],
    sim_position_q1_per_joint: [-0.2, -0.1, 0],
    sim_position_median_per_joint: [0, 0.1, 0.2],
    sim_position_q3_per_joint: [0.2, 0.3, 0.4],
    sim_position_max_per_joint: [0.5, 0.6, 0.7],
    real_position_min_per_joint: [-0.48, -0.38, -0.28],
    real_position_q1_per_joint: [-0.18, -0.08, 0.02],
    real_position_median_per_joint: [0.02, 0.12, 0.22],
    real_position_q3_per_joint: [0.22, 0.32, 0.42],
    real_position_max_per_joint: [0.52, 0.62, 0.72],
    shift_detected: false,
    detection_method: 'none',
    comm_health_score: 0.95,
    dynamics_anomaly_score: 0.1,
    velocity_jump_per_joint: [0, 0, 0],
    soft_limit_score: 0.05,
    soft_limit_triggered: false,
  };
}

function trackingPayload() {
  return {
    joint_names: ['panda_joint1', 'panda_joint2', 'panda_joint3'],
    errors: [0.001, 0.002, 0.0015],
  };
}

/** Inject a mock WebSocket that streams dashboard data at high frequency. */
export async function installMockWebSocket(page: Page) {
  await page.addInitScript(() => {
    class MockWebSocket {
      static CONNECTING = 0;
      static OPEN = 1;
      static CLOSING = 2;
      static CLOSED = 3;
      static instances: MockWebSocket[] = [];

      readyState = MockWebSocket.CONNECTING;
      onopen: ((ev: Event) => void) | null = null;
      onmessage: ((ev: MessageEvent) => void) | null = null;
      onclose: ((ev: CloseEvent) => void) | null = null;
      onerror: ((ev: Event) => void) | null = null;

      private timer: number | null = null;
      private tick = 0;

      constructor(_url: string) {
        MockWebSocket.instances.push(this);
        (window as typeof window & {
          __hocMockSend?: (payload: Record<string, unknown>) => void;
          __hocMockPause?: () => void;
        }).__hocMockSend = (payload) => this.sendJson(payload);
        (window as typeof window & {
          __hocMockPause?: () => void;
        }).__hocMockPause = () => {
          MockWebSocket.instances.forEach((socket) => {
            if (socket.timer !== null) window.clearInterval(socket.timer);
            socket.timer = null;
          });
        };
        window.setTimeout(() => {
          this.readyState = MockWebSocket.OPEN;
          this.onopen?.(new Event('open'));
          this.sendJson({ type: 'connected', message: 'mock' });
          this.timer = window.setInterval(() => this.push(), 100);
        }, 50);
      }

      send(_data: string) {
        /* command acks handled in push for simplicity */
      }

      close() {
        if (this.timer) {
          clearInterval(this.timer);
        }
        this.readyState = MockWebSocket.CLOSED;
        this.onclose?.(new CloseEvent('close'));
      }

      private sendJson(payload: Record<string, unknown>) {
        this.onmessage?.({ data: JSON.stringify(payload) } as MessageEvent);
      }

      private push() {
        this.tick += 1;
        const risk = {
          validity: 'VALID',
          reason_code: 'none',
          has_valid_sources: true,
          active_dimensions: [
            'distribution_shift', 'tracking_error', 'dynamics_anomaly',
            'comm_health', 'planning_failure',
          ],
          invalid_dimensions: [],
          level: this.tick % 40 < 35 ? 0 : 2,
          level_name: this.tick % 40 < 35 ? 'R0' : 'R2',
          composite_score: 0.2 + (this.tick % 10) * 0.01,
          primary_driver: 'distribution_shift',
          recommendation: '系统运行正常',
          e_stop_active: false,
          degraded_mode: false,
          attribution: [
            'distribution_shift',
            'tracking_error',
            'dynamics_anomaly',
            'comm_health',
            'planning_failure',
          ].map((dimension, idx) => ({
            dimension,
            raw_score: 0.1 + idx * 0.05,
            weighted_score: 0.03,
            weight: 0.2,
            is_primary_driver: idx === 0,
          })),
        };
        const metrics = {
          validity: 'VALID',
          reason_code: 'none',
          metric_valid: true,
          baseline_ready: true,
          calibration_id: 'mock_same_scene',
          reference_source: 'topic',
          aligned_sample_count: 118,
          comm_health_valid: true,
          dynamics_valid: true,
          joint_names: ['panda_joint1', 'panda_joint2', 'panda_joint3'],
          kl_divergence_per_joint: [0.04, 0.05, 0.06],
          kl_divergence_mean: 0.05,
          wasserstein_per_joint: [0.03, 0.04, 0.05],
          wasserstein_mean: 0.04,
          w1_threshold: 0.1,
          shift_detected_w1: false,
          mmd_statistic: 0.12,
          mmd_p_value: 0.4,
          mmd_threshold: 0.2,
          window_duration_sec: 10,
          sample_count_sim: 120,
          sample_count_real: 118,
          sim_position_min_per_joint: [-0.5, -0.4, -0.3],
          sim_position_q1_per_joint: [-0.2, -0.1, 0],
          sim_position_median_per_joint: [0, 0.1, 0.2],
          sim_position_q3_per_joint: [0.2, 0.3, 0.4],
          sim_position_max_per_joint: [0.5, 0.6, 0.7],
          real_position_min_per_joint: [-0.48, -0.38, -0.28],
          real_position_q1_per_joint: [-0.18, -0.08, 0.02],
          real_position_median_per_joint: [0.02, 0.12, 0.22],
          real_position_q3_per_joint: [0.22, 0.32, 0.42],
          real_position_max_per_joint: [0.52, 0.62, 0.72],
          shift_detected: false,
          detection_method: 'none',
          comm_health_score: 0.95,
          dynamics_anomaly_score: 0.1,
          velocity_jump_per_joint: [0, 0, 0],
          soft_limit_score: 0.05,
          soft_limit_triggered: false,
        };
        this.sendJson({
          type: 'data',
          topic: '/risk/status',
          timestamp: { sec: this.tick, nanosec: 0 },
          payload: risk,
        });
        this.sendJson({
          type: 'data',
          topic: '/monitor/distribution_metrics',
          timestamp: { sec: this.tick, nanosec: 0 },
          payload: metrics,
        });
        this.sendJson({
          type: 'runtime_frame',
          payload: {
            lanes: {
              brain: {
                lane: 'brain', validity: 'VALID', reason_code: 'none',
                lifecycle_state: 'ACTIVE', queue_depth: 4, age_ms: 20,
              },
              execution: {
                lane: 'execution', validity: 'VALID', reason_code: 'none',
                decision: 'EXECUTED', command_sequence: this.tick, age_ms: 15,
              },
              safety: { lane: 'safety', age_ms: 10, ...risk },
              task_gt: {
                lane: 'task_gt', validity: 'VALID', reason_code: 'none',
                task_status: 'RUNNING', phase: 'reach', age_ms: 30,
              },
            },
            correlation: { trace_run_ids: ['mock'], trace_consistent: true },
          },
        });
        this.sendJson({
          type: 'data',
          topic: '/monitor/tracking_error',
          timestamp: { sec: this.tick, nanosec: 0 },
          payload: {
            joint_names: ['panda_joint1', 'panda_joint2', 'panda_joint3'],
            errors: [0.001, 0.002, 0.0015],
          },
        });
        this.sendJson({
          type: 'data',
          topic: '/bridge/sim/grasp_status',
          timestamp: { sec: this.tick, nanosec: 0 },
          payload: {
            grasp_established: true,
            object_slipped: false,
            force_xyz: [2.4, 0.3, 0.1],
            torque_xyz: [0.01, 0.02, 0.01],
            force_norm: 2.42,
            confidence: 0.95,
          },
        });
      }
    }

    window.WebSocket = MockWebSocket as unknown as typeof WebSocket;
  });
}

export { metricsPayload, riskPayload, trackingPayload };
