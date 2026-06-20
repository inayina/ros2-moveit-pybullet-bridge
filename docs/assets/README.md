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
python3 scripts/capture_pick_lift_asset.py
```

| 文件 | 数据源 |
|------|--------|
| `m6-pick-and-lift.gif` | episode-data-lab 成功 `pick_and_lift` episode（constraint grasp + object lift） |
| `m3-dual-source.gif` | `same-task-iiwa-dual.npz` Sim vs Real |
| `m2-iiwa-pybullet.gif` | 同上 · PyBullet 帧序列 |
| `m2-iiwa-rviz.gif` | 可选 RViz/MoveIt 本地录屏；不作为 README 主展示 |
| `m5-hoc-dashboard.png` | HOC `hoc_prod` 浏览器截图；失败时回退为实验 JSON 指标图 |

合成兜底：`python3 scripts/generate_milestone_assets.py`。该命令默认保护 README 真实截图/GIF，不会覆盖上表 4 个已存在资产；如确需重写兜底图，使用 `python3 scripts/generate_milestone_assets.py --force-readme-assets`。

## 面试 Demo

1. 打开 `docs/samples/dual-repo-experiment-report.html`（图 1–5 + 6–9）
2. 打开 `docs/samples/same-task-calibration-report.html`（图 6–9 详解）
3. README 配图已由 `capture_readme_assets.sh` 和 `capture_pick_lift_asset.py` 从真实 NPZ、HOC 浏览器与 pick-and-lift episode 生成
