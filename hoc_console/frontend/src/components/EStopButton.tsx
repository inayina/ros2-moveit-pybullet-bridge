import { Button } from 'antd';
import { useCallback, useRef, useState } from 'react';

interface EStopButtonProps {
  onEStop: () => void;
  disabled?: boolean;
}

export function EStopButton({ onEStop, disabled }: EStopButtonProps) {
  const [cooldown, setCooldown] = useState(false);
  const lastClick = useRef(0);

  const handleClick = useCallback(() => {
    const now = Date.now();
    if (now - lastClick.current < 1000) return;
    lastClick.current = now;
    setCooldown(true);
    onEStop();
    window.setTimeout(() => setCooldown(false), 1000);
  }, [onEStop]);

  return (
    <Button
      type="primary"
      danger
      size="large"
      className="estop-btn"
      disabled={disabled || cooldown}
      onClick={handleClick}
    >
      急停
    </Button>
  );
}
