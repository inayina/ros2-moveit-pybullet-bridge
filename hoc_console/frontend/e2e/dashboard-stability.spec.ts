import { expect, test } from '@playwright/test';
import { installMockWebSocket } from './mockWebSocket';

test.describe('HOC dashboard stability', () => {
  test.beforeEach(async ({ page }) => {
    await installMockWebSocket(page);
    await page.goto('/');
    await expect(page.getByText('WS 已连接')).toBeVisible({ timeout: 10_000 });
  });

  test('main grid layout stays stable under rapid WS updates', async ({ page }) => {
    const mainGrid = page.locator('.dashboard-grid--main');
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

  test('risk banner does not use full-page opacity pulse', async ({ page }) => {
    await page.waitForTimeout(1500);
    const animation = await page.locator('.risk-banner').evaluate((el) => {
      return window.getComputedStyle(el).animationName;
    });
    expect(animation).not.toBe('banner-pulse');
  });

  test('dashboard renders key panels under load', async ({ page }) => {
    await page.waitForTimeout(2000);
    await expect(page.getByText('五维风险雷达')).toBeVisible();
    await expect(page.getByText('Sim / Real 分布对比')).toBeVisible();
    await expect(page.getByText('关节跟踪误差')).toBeVisible();
    await expect(page.getByText('KL / W1 / MMD / 通信健康 时序趋势')).toBeVisible();
    await expect(page.locator('.distribution-charts')).toBeVisible();
  });
});
