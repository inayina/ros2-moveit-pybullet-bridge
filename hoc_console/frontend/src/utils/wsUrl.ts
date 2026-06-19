/** Resolve WebSocket URL for dev (Vite proxy), prod (:8080), or direct :8765. */
export function resolveWsUrl(): string {
  if (import.meta.env.VITE_WS_URL) {
    return import.meta.env.VITE_WS_URL;
  }

  const wsProto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const { host, port } = window.location;

  // Vite dev (:5173) or hoc_server static UI (:8080): same-origin /hoc-ws proxy
  if (import.meta.env.DEV || port === '8080') {
    return `${wsProto}//${host}/hoc-ws`;
  }

  // Fallback: direct backend port on same host (not browser localhost)
  const wsPort = import.meta.env.VITE_WS_PORT ?? '8765';
  return `${wsProto}//${window.location.hostname}:${wsPort}`;
}
