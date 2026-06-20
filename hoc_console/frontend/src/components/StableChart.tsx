import ReactECharts from 'echarts-for-react';
import { memo } from 'react';

interface StableChartProps {
  option: Record<string, unknown>;
  height: number;
  className?: string;
}

/** Shared ECharts wrapper: merge updates, no animation flicker. */
export const StableChart = memo(function StableChart({
  option,
  height,
  className,
}: StableChartProps) {
  return (
    <ReactECharts
      className={className}
      option={option}
      style={{ height, width: '100%' }}
      opts={{ renderer: 'canvas' }}
      lazyUpdate
    />
  );
});
