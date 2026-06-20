export function shouldThrottle(key: string, intervalMs: number): boolean {
  const storage = shouldThrottle as typeof shouldThrottle & {
    _last?: Map<string, number>;
  };
  if (!storage._last) {
    storage._last = new Map();
  }
  const now = Date.now();
  const last = storage._last.get(key) ?? 0;
  if (now - last < intervalMs) {
    return true;
  }
  storage._last.set(key, now);
  return false;
}
