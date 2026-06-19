import { memo, useEffect, useState } from 'react';
import { Typography } from 'antd';

const { Text } = Typography;

/** Isolated 1 Hz clock so the rest of the banner does not re-render every second. */
export const SessionClock = memo(function SessionClock({ sessionStart }: { sessionStart: number }) {
  const [elapsed, setElapsed] = useState('00:00:00');

  useEffect(() => {
    const tick = () => {
      const sec = Math.floor((Date.now() - sessionStart) / 1000);
      const h = Math.floor(sec / 3600);
      const m = Math.floor((sec % 3600) / 60);
      const s = sec % 60;
      setElapsed(
        `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`,
      );
    };
    tick();
    const id = window.setInterval(tick, 1000);
    return () => clearInterval(id);
  }, [sessionStart]);

  return <Text type="secondary">⏱ {elapsed}</Text>;
});
