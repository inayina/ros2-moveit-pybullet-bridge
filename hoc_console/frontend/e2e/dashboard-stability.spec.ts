import { expect, test } from '@playwright/test';
import { installMockWebSocket } from './mockWebSocket';

test.describe('HOC dashboard stability', () => {
  test.beforeEach(async ({ page }) => {
    await installMockWebSocket(page);
    await page.goto('/');
    await expect(page.getByTestId('runtime-final-decision')).toContainText(
      'Final Decision: RUN',
      { timeout: 10_000 },
    );
  });

  test('main grid layout stays stable under rapid WS updates', async ({ page }) => {
    const mainGrid = page.locator('.overview-grid');
    await expect(mainGrid).toBeVisible();

    const initial = await mainGrid.boundingBox();
    expect(initial).not.toBeNull();

    await page.waitForTimeout(2500);

    const later = await mainGrid.boundingBox();
    expect(later).not.toBeNull();

    expect(Math.abs((initial?.height ?? 0) - (later?.height ?? 0))).toBeLessThan(8);
    expect(Math.abs((initial?.width ?? 0) - (later?.width ?? 0))).toBeLessThan(8);
  });

  test('panels keep fixed chart regions without vertical jump', async ({ page }) => {
    const radar = page.locator('.panel--chart').first();
    const boxA = await radar.boundingBox();
    await page.waitForTimeout(2000);
    const boxB = await radar.boundingBox();

    expect(Math.abs((boxA?.y ?? 0) - (boxB?.y ?? 0))).toBeLessThan(4);
    expect(Math.abs((boxA?.height ?? 0) - (boxB?.height ?? 0))).toBeLessThan(8);
  });

  test('distribution panel uses side-by-side chart layout on desktop', async ({ page }) => {
    await page.setViewportSize({ width: 1400, height: 900 });
    const charts = page.locator('.distribution-charts');
    await expect(charts).toBeVisible();

    const layout = await charts.evaluate((el) => {
      const style = window.getComputedStyle(el);
      return style.gridTemplateColumns;
    });
    expect(layout).not.toBe('none');
    expect(layout.split(' ').length).toBeGreaterThanOrEqual(2);
  });

  test('runtime decision does not use full-page opacity pulse', async ({ page }) => {
    await page.waitForTimeout(1500);
    const animation = await page.locator('.runtime-overview').evaluate((el) => {
      return window.getComputedStyle(el).animationName;
    });
    expect(animation).not.toBe('banner-pulse');
  });

  test('runtime lanes distinguish five validity states', async ({ page }) => {
    await page.evaluate(() => {
      const push = (window as typeof window & {
        __hocMockSend?: (payload: Record<string, unknown>) => void;
        __hocMockPause?: () => void;
      }).__hocMockSend;
      (window as typeof window & { __hocMockPause?: () => void }).__hocMockPause?.();
      const lane = (name: string, validity: string) => ({
        lane: name,
        validity,
        reason_code: `${validity.toLowerCase()}_fixture`,
        age_ms: 1,
      });
      push?.({
        type: 'runtime_frame',
        payload: {
          lanes: {
            brain: lane('brain', 'WARMING_UP'),
            execution: lane('execution', 'STALE'),
            safety: lane('safety', 'UNAVAILABLE'),
            task_gt: lane('task_gt', 'ERROR'),
          },
          correlation: { trace_consistent: true },
        },
      });
    });
    const badges = page.locator('.ant-badge-status-text');
    await expect(badges.filter({ hasText: /^WARMING_UP$/ })).toBeVisible();
    await expect(badges.filter({ hasText: /^STALE$/ })).toBeVisible();
    await expect(badges.filter({ hasText: /^UNAVAILABLE$/ })).toBeVisible();
    await expect(badges.filter({ hasText: /^ERROR$/ })).toBeVisible();
    await expect(page.getByTestId('runtime-final-decision')).toContainText('NO DATA');
  });

  test('dashboard renders key panels under load', async ({ page }) => {
    await page.waitForTimeout(2000);
    await expect(page.getByText('五维风险雷达')).toBeVisible();
    await expect(page.getByText('Panda Runtime / Reference 分布')).toBeVisible();
    await expect(page.getByText('Final Decision: RUN')).toBeVisible();
    await expect(page.getByText('Command-correlated state timeline')).toBeVisible();
    const runtimeLanes = page.locator('.runtime-lanes');
    await expect(runtimeLanes.getByText('Brain', { exact: true })).toBeVisible();
    await expect(runtimeLanes.getByText('Execution', { exact: true })).toBeVisible();
    await expect(runtimeLanes.getByText('Safety', { exact: true })).toBeVisible();
    await expect(runtimeLanes.getByText('Task GT', { exact: true })).toBeVisible();
    await expect(page.getByText('关节跟踪误差')).toBeVisible();
    await expect(page.locator('.distribution-charts')).toBeVisible();

    await page.getByRole('tab', { name: 'Diagnostics' }).click();
    await expect(page.getByText('Panda 抓取 / 传感融合')).toBeVisible();
    await expect(page.getByText('KL / W1 / MMD / 通信健康 时序趋势')).toBeVisible();

    await page.getByRole('tab', { name: 'Historical / Evidence' }).click();
    await expect(page.getByText('Panda Dataset')).toBeVisible();
  });

  test('level-one overview fits a 1920 by 1080 operator display', async ({ page }) => {
    await page.setViewportSize({ width: 1920, height: 1080 });
    const overview = page.locator('.overview-grid');
    await expect(overview).toBeVisible();
    const fit = await overview.evaluate((element) => {
      const rect = element.getBoundingClientRect();
      return rect.top >= 0 && rect.bottom <= window.innerHeight + 1;
    });
    expect(fit).toBe(true);
    const pageOverflow = await page.evaluate(
      () => document.documentElement.scrollHeight > window.innerHeight + 1,
    );
    expect(pageOverflow).toBe(false);
  });
});
