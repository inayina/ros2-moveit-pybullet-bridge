import { expect, test } from '@playwright/test';
import path from 'node:path';
import { installMockWebSocket } from './mockWebSocket';

test('capture current four-lane HOC frontend for portfolio', async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
  await installMockWebSocket(page);
  await page.goto('/');
  await expect(page.getByTestId('runtime-final-decision')).toContainText('Final Decision: RUN');
  await page.waitForTimeout(1800);

  await page.evaluate(() => {
    const mock = window as typeof window & {
      __hocMockSend?: (payload: Record<string, unknown>) => void;
      __hocMockPause?: () => void;
    };
    mock.__hocMockPause?.();
  });
  await page.waitForTimeout(120);

  await page.evaluate(() => {
    const mock = window as typeof window & {
      __hocMockSend?: (payload: Record<string, unknown>) => void;
    };
    mock.__hocMockSend?.({
      type: 'runtime_frame',
      payload: {
        lanes: {
          brain: {
            lane: 'brain', validity: 'VALID', reason_code: 'none',
            lifecycle_state: 'ACTIVE', queue_depth: 4, age_ms: 17,
          },
          execution: {
            lane: 'execution', validity: 'VALID', reason_code: 'risk_estop',
            decision: 'ESTOPPED', command_sequence: 3, age_ms: 12,
          },
          safety: {
            lane: 'safety', validity: 'VALID', reason_code: 'risk.r3_estop',
            has_valid_sources: true, level: 3, level_name: 'R3',
            proposed_decision: 'E_STOP', actual_decision: 'E_STOP',
            primary_driver: 'tracking_error', age_ms: 8,
          },
          task_gt: {
            lane: 'task_gt', validity: 'UNAVAILABLE',
            reason_code: 'evaluator_not_running', task_status: 'UNAVAILABLE', age_ms: 23,
          },
        },
        correlation: {
          trace_run_ids: ['portfolio_frontend_fixture'], trace_consistent: true,
        },
      },
    });
  });
  await page.waitForTimeout(120);

  await page.evaluate(() => {
    const mock = window as typeof window & {
      __hocMockSend?: (payload: Record<string, unknown>) => void;
    };
    mock.__hocMockSend?.({
      type: 'runtime_frame',
      payload: {
        lanes: {
          brain: {
            lane: 'brain', validity: 'VALID', reason_code: 'none',
            lifecycle_state: 'ACTIVE', queue_depth: 4, age_ms: 18,
          },
          execution: {
            lane: 'execution', validity: 'VALID', reason_code: 'risk_hold',
            decision: 'HELD', command_sequence: 2, age_ms: 13,
          },
          safety: {
            lane: 'safety', validity: 'VALID', reason_code: 'risk.r2_hold',
            has_valid_sources: true, level: 2, level_name: 'R2',
            proposed_decision: 'HOLD', actual_decision: 'HOLD',
            primary_driver: 'distribution_shift', age_ms: 9,
          },
          task_gt: {
            lane: 'task_gt', validity: 'UNAVAILABLE',
            reason_code: 'evaluator_not_running', task_status: 'UNAVAILABLE', age_ms: 24,
          },
        },
        correlation: {
          trace_run_ids: ['portfolio_frontend_fixture'], trace_consistent: true,
        },
      },
    });
  });

  await expect(page.getByTestId('runtime-final-decision')).toContainText('Final Decision: HOLD');
  const runtimeLanes = page.locator('.runtime-lanes');
  await expect(runtimeLanes.getByText('HELD', { exact: true })).toBeVisible();
  await expect(runtimeLanes.getByText('HOLD → HOLD', { exact: true })).toBeVisible();
  await expect(page.getByText('UNAVAILABLE', { exact: true }).first()).toBeVisible();
  await page.waitForTimeout(250);

  const overviewFits = await page.locator('.overview-grid').evaluate((element) => {
    const rect = element.getBoundingClientRect();
    return rect.top >= 0 && rect.bottom <= window.innerHeight + 1;
  });
  expect(overviewFits).toBe(true);
  expect(await page.evaluate(
    () => document.documentElement.scrollHeight <= window.innerHeight + 1,
  )).toBe(true);

  await page.screenshot({
    path: path.resolve(process.cwd(), '../../docs/assets/hoc-runtime-four-lane-dashboard.png'),
    fullPage: false,
  });
});
