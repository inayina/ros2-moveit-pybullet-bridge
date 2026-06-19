import { useCallback, useEffect, useRef } from 'react';
import type {
  AlertEvent,
  DistributionMetricsPayload,
  ExperimentProgressPayload,
  RiskStatusPayload,
  TrackingErrorPayload,
  WsFrame,
} from '../types/messages';
import { useDashboardStore } from '../stores/dashboardStore';
import { resolveWsUrl } from '../utils/wsUrl';
const SUBSCRIBE_TOPICS = [
  '/monitor/distribution_metrics',
  '/risk/status',
  '/monitor/tracking_error',
];
const BACKOFF_MS = [1000, 2000, 4000, 8000, 16000, 30000];

function isRiskPayload(payload: unknown): payload is RiskStatusPayload {
  return (
    typeof payload === 'object' &&
    payload !== null &&
    'level' in payload &&
    'composite_score' in payload
  );
}

function isTrackingPayload(payload: unknown): payload is TrackingErrorPayload {
  return (
    typeof payload === 'object' &&
    payload !== null &&
    'joint_names' in payload &&
    'errors' in payload
  );
}

function isAlertPayload(payload: unknown): payload is AlertEvent {
  return typeof payload === 'object' && payload !== null;
}

function isMetricsPayload(payload: unknown): payload is DistributionMetricsPayload {
  return (
    typeof payload === 'object' &&
    payload !== null &&
    'kl_divergence_mean' in payload &&
    'mmd_statistic' in payload
  );
}

function isExperimentProgress(payload: unknown): payload is ExperimentProgressPayload {
  return (
    typeof payload === 'object' &&
    payload !== null &&
    'experiment_id' in payload &&
    'scenario_id' in payload &&
    'progress' in payload
  );
}

export function useWebSocket() {
  const wsRef = useRef<WebSocket | null>(null);
  const retryRef = useRef(0);
  const pingRef = useRef<number | null>(null);
  const mountedRef = useRef(true);

  const setConnected = useDashboardStore((s) => s.setConnected);
  const ingestRisk = useDashboardStore((s) => s.ingestRisk);
  const ingestMetrics = useDashboardStore((s) => s.ingestMetrics);
  const ingestTracking = useDashboardStore((s) => s.ingestTracking);
  const ingestAlert = useDashboardStore((s) => s.ingestAlert);
  const setRecording = useDashboardStore((s) => s.setRecording);
  const ingestExperimentProgress = useDashboardStore((s) => s.ingestExperimentProgress);
  const setCameraFrame = useDashboardStore((s) => s.setCameraFrame);

  const handleFrame = useCallback(
    (frame: WsFrame) => {
      if (frame.type === 'data' && 'topic' in frame && frame.payload) {
        if (frame.topic === '/risk/status' && isRiskPayload(frame.payload)) {
          ingestRisk(frame.payload);
        } else if (
          frame.topic === '/monitor/distribution_metrics' &&
          isMetricsPayload(frame.payload)
        ) {
          ingestMetrics(frame.payload);
        } else if (
          frame.topic === '/monitor/tracking_error' &&
          isTrackingPayload(frame.payload)
        ) {
          ingestTracking(frame.payload);
        } else if (frame.topic === 'alert_event' && isAlertPayload(frame.payload)) {
          ingestAlert(frame.payload);
        }
        return;
      }

      if (frame.type === 'risk_status' && 'payload' in frame && isRiskPayload(frame.payload)) {
        ingestRisk(frame.payload);
      } else if (
        frame.type === 'distribution_metrics' &&
        'payload' in frame &&
        isMetricsPayload(frame.payload)
      ) {
        ingestMetrics(frame.payload);
      } else if (
        frame.type === 'tracking_error' &&
        'payload' in frame &&
        isTrackingPayload(frame.payload)
      ) {
        ingestTracking(frame.payload);
      } else if (frame.type === 'alert_event' && 'payload' in frame && isAlertPayload(frame.payload)) {
        ingestAlert(frame.payload);
      } else if (frame.type === 'recording_status' && 'recording' in frame) {
        setRecording(Boolean(frame.recording), String(frame.bag_path ?? ''));
      } else if (frame.type === 'experiment_progress' && 'payload' in frame) {
        const payload = frame.payload;
        if (isExperimentProgress(payload)) {
          ingestExperimentProgress(payload);
          if (payload.current_metrics) {
            ingestMetrics(payload.current_metrics);
          }
          if (payload.current_risk) {
            ingestRisk(payload.current_risk);
          }
        }
      } else if (frame.type === 'camera_frame' && 'payload' in frame) {
        const payload = frame.payload;
        if (
          typeof payload === 'object' &&
          payload !== null &&
          'image_b64' in payload &&
          typeof payload.image_b64 === 'string'
        ) {
          setCameraFrame(`data:image/jpeg;base64,${payload.image_b64}`);
        }
      }
    },
    [ingestAlert, ingestExperimentProgress, ingestMetrics, ingestRisk, ingestTracking, setCameraFrame, setRecording],
  );

  const connect = useCallback(() => {
    if (!mountedRef.current) return;
    const wsUrl = resolveWsUrl();
    const ws = new WebSocket(wsUrl);
    wsRef.current = ws;

    ws.onopen = () => {
      retryRef.current = 0;
      setConnected(true);
      ws.send(
        JSON.stringify({
          type: 'subscribe',
          topics: SUBSCRIBE_TOPICS,
        }),
      );
      pingRef.current = window.setInterval(() => {
        if (ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify({ type: 'ping' }));
        }
      }, 30000);
    };

    ws.onmessage = (event) => {
      try {
        const frame = JSON.parse(event.data) as WsFrame;
        handleFrame(frame);
      } catch {
        /* ignore malformed frames */
      }
    };

    ws.onclose = () => {
      setConnected(false);
      if (pingRef.current) {
        clearInterval(pingRef.current);
        pingRef.current = null;
      }
      if (!mountedRef.current) return;
      const delay = BACKOFF_MS[Math.min(retryRef.current, BACKOFF_MS.length - 1)];
      retryRef.current += 1;
      window.setTimeout(connect, delay);
    };

    ws.onerror = () => ws.close();
  }, [handleFrame, setConnected]);

  useEffect(() => {
    mountedRef.current = true;
    connect();
    return () => {
      mountedRef.current = false;
      if (pingRef.current) clearInterval(pingRef.current);
      wsRef.current?.close();
    };
  }, [connect]);

  const sendCommand = useCallback((action: string, params: Record<string, unknown> = {}) => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      return Promise.resolve({ success: false, message: 'WebSocket not connected' });
    }
    return new Promise<{ success: boolean; message?: string; file_path?: string }>((resolve) => {
      const handler = (event: MessageEvent) => {
        try {
          const msg = JSON.parse(event.data);
          if (msg.type === 'command_result' && msg.action === action) {
            ws.removeEventListener('message', handler);
            resolve(msg);
          }
          if (msg.type === 'report_ready' && action === 'export_report') {
            ws.removeEventListener('message', handler);
            resolve({ success: true, file_path: msg.file_path, message: 'Report ready' });
          }
        } catch {
          /* ignore */
        }
      };
      ws.addEventListener('message', handler);
      ws.send(JSON.stringify({ type: 'command', action, params }));
      window.setTimeout(() => {
        ws.removeEventListener('message', handler);
        resolve({ success: false, message: 'Command timeout' });
      }, 10000);
    });
  }, []);

  return { sendCommand };
}
