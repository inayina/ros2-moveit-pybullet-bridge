# 配图资源索引

PNG 标题使用 **Fig. 1–9**（matplotlib 英文字体）；HTML 报告使用 **图 1–9**（中文），二者一一对应。

对齐命令：`python3 scripts/regenerate_all_reports.py` → 产出 `docs/samples/report-manifest.json`

| 图号 | 文件 | 报告 |
|------|------|------|
| 图 1 | `dual-repo-integration-overview.png` | 双仓库架构 |
| 图 2 | `dual-repo-cross-source-overlay.png` | 跨源轨迹叠加 |
| 图 3 | `dual-repo-cross-source-metrics.png` | 跨源 KL/W1/MMD |
| 图 4 | `dual-repo-lerobot-trajectory.png` | LeRobot Real 全量 |
| 图 5 | `dual-repo-offline-self-metrics.png` | 负对照自对比 |
| 图 6 | `same-task-iiwa-overlay.png` | 同任务 A 轨迹 |
| 图 7 | `same-task-iiwa-metrics.png` | 同任务 A 指标 |
| 图 8 | `same-task-lerobot-overlay.png` | 同任务 B 轨迹 |
| 图 9 | `same-task-lerobot-metrics.png` | 同任务 B 指标 |

---

## README 展示 GIF/PNG（P1-1 · 真实数据）

```bash
./scripts/capture_readme_assets.sh
```

| 文件 | 数据源 |
|------|--------|
| `m3-dual-source.gif` | `same-task-iiwa-dual.npz` Sim vs Real |
| `m2-iiwa-pybullet.gif` | 同上 · PyBullet 帧序列 |
| `m2-iiwa-rviz.gif` | 同上 · 3D + 关节曲线分屏 |
| `m5-hoc-dashboard.png` | 实验 JSON 指标 / 可选 HOC Playwright 截图 |

合成兜底：`python3 scripts/generate_milestone_assets.py`

## 面试 Demo

1. 打开 `docs/samples/dual-repo-experiment-report.html`（图 1–5 + 6–9）
2. 打开 `docs/samples/same-task-calibration-report.html`（图 6–9 详解）
3. README 配图已由 `capture_readme_assets.sh` 从真实 NPZ 生成
