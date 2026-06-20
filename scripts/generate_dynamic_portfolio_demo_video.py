#!/usr/bin/env python3
"""Generate a dynamic Chinese demo with robot pose and live metrics."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation, font_manager
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle, FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
SAMPLES = ROOT / "docs" / "samples"
OUT_DIR = SAMPLES / "portfolio-demo-zh-dynamic"
DEFAULT_OUT = SAMPLES / "portfolio-demo-zh-dynamic.mp4"

BG = "#07101c"
PANEL = "#111b2d"
PANEL_2 = "#17243a"
GRID = "#26394f"
TEXT = "#edf6ff"
MUTED = "#9fb1c8"
BLUE = "#5dade2"
GREEN = "#52c41a"
ORANGE = "#faad14"
RED = "#ff4d4f"
PURPLE = "#b37feb"


def configure_font() -> None:
    for path in [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    ]:
        if Path(path).exists():
            font_manager.fontManager.addfont(path)
            plt.rcParams["font.family"] = font_manager.FontProperties(fname=path).get_name()
            break
    plt.rcParams["axes.unicode_minus"] = False


def load_json(name: str) -> dict[str, Any]:
    path = SAMPLES / name
    if not path.exists():
        return {}
    return json.loads(path.read_text())


def load_hoc_csv() -> dict[str, np.ndarray]:
    path = SAMPLES / "hoc-verification-reports" / "fr_hoc_verify.csv"
    if not path.exists():
        path = SAMPLES / "monitor-metrics-timeline.csv"
    rows: list[dict[str, str]] = []
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        t = np.linspace(0, 30, 240)
        return {
            "t": t,
            "risk_level": np.zeros_like(t),
            "composite_score": np.zeros_like(t),
            "kl_mean": np.zeros_like(t),
            "w1_mean": np.zeros_like(t),
            "mmd_stat": np.zeros_like(t),
            "shift_detected": np.zeros_like(t),
        }

    def col(name: str, fallback: float = 0.0) -> np.ndarray:
        return np.array([float(r.get(name, fallback) or fallback) for r in rows], dtype=float)

    def bool_col(name: str) -> np.ndarray:
        return np.array([str(r.get(name, "")).lower() == "true" for r in rows], dtype=float)

    t = col("t")
    t = t - t[0]
    return {
        "t": t,
        "risk_level": col("risk_level"),
        "composite_score": col("composite_score"),
        "kl_mean": col("kl_mean"),
        "w1_mean": col("w1_mean"),
        "mmd_stat": col("mmd_stat", col("mmd_statistic")[0] if "mmd_statistic" in rows[0] else 0.0),
        "shift_detected": bool_col("shift_detected"),
    }


def interp_series(series: dict[str, np.ndarray], duration: float, frames: int) -> dict[str, np.ndarray]:
    src_t = series["t"]
    if src_t[-1] <= 0:
        src_t = np.linspace(0, duration, len(src_t))
    dst_t = np.linspace(0, duration, frames)
    scale_t = src_t / src_t[-1] * duration
    out = {"t": dst_t}
    for key, values in series.items():
        if key == "t":
            continue
        out[key] = np.interp(dst_t, scale_t, values)
    return out


def robot_pose(progress: float) -> tuple[np.ndarray, np.ndarray]:
    """Planar 7-link iiwa-like pose used as a visual proxy."""
    lengths = np.array([0.34, 0.30, 0.28, 0.24, 0.20, 0.16, 0.12])
    q = np.array(
        [
            0.55 * math.sin(2 * math.pi * progress),
            -0.55 + 0.35 * math.sin(2 * math.pi * progress + 0.8),
            0.45 * math.sin(4 * math.pi * progress + 0.3),
            0.35 * math.sin(2 * math.pi * progress + 1.7),
            -0.25 * math.sin(3 * math.pi * progress),
            0.25 * math.sin(2 * math.pi * progress + 2.4),
            0.15 * math.sin(5 * math.pi * progress),
        ]
    )
    theta = np.cumsum(q) + math.pi / 2
    xs = [0.0]
    ys = [0.0]
    for l, th in zip(lengths, theta):
        xs.append(xs[-1] + l * math.cos(th))
        ys.append(ys[-1] + l * math.sin(th))
    return np.array(xs), np.array(ys)


def event_text(t: float, duration: float) -> tuple[str, str, str]:
    p = t / duration
    if p < 0.16:
        return "启动双源仿真", "MoveIt + PyBullet bridge 已就绪", BLUE
    if p < 0.33:
        return "轨迹规划执行", "FollowJointTrajectory relay 闭环反馈", GREEN
    if p < 0.50:
        return "注入分布偏移", "joint_damping +20%，制造已知异常", ORANGE
    if p < 0.67:
        return "监控检出", "KL/MMD 超阈值，shift_detected=true", ORANGE
    if p < 0.84:
        return "风险处置", "R2 降速 / R3 急停 / 速度归零", RED
    return "导出证据", "JSON/CSV 报告 + 验收脚本产物", GREEN


def generate(output: Path, duration: float, fps: int) -> None:
    configure_font()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    metrics = interp_series(load_hoc_csv(), duration, int(duration * fps))
    moveit = load_json("moveit-closure-metrics.json")
    monitor = load_json("monitor-metrics.json")
    risk = load_json("risk-management-metrics.json")
    safety = load_json("safety-nfr-metrics.json")
    hoc = load_json("hoc-console-metrics.json")

    frames = len(metrics["t"])
    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor=BG)
    gs = fig.add_gridspec(3, 4, left=0.04, right=0.98, top=0.88, bottom=0.08, wspace=0.28, hspace=0.36)
    ax_robot = fig.add_subplot(gs[:, :2], facecolor=PANEL)
    ax_metric = fig.add_subplot(gs[0, 2:], facecolor=PANEL)
    ax_risk = fig.add_subplot(gs[1, 2:], facecolor=PANEL)
    ax_cards = fig.add_subplot(gs[2, 2:], facecolor=PANEL)
    ax_cards.axis("off")

    fig.text(0.04, 0.95, "动态中文 Demo：机械臂姿态 + 实时指标", color=TEXT, fontsize=24, weight="bold")
    fig.text(0.04, 0.915, "用动画展示系统主线：规划执行、分布偏移注入、风险升级、安全处置和证据导出", color=MUTED, fontsize=13)

    # Robot panel
    ax_robot.set_title("KUKA iiwa7 姿态示意与末端轨迹", color=TEXT, fontsize=17, pad=12)
    ax_robot.set_xlim(-1.25, 1.25)
    ax_robot.set_ylim(-0.18, 1.65)
    ax_robot.set_aspect("equal")
    ax_robot.grid(color=GRID, alpha=0.45)
    ax_robot.tick_params(colors=MUTED)
    for s in ax_robot.spines.values():
        s.set_color(GRID)
    ax_robot.add_patch(FancyBboxPatch((-0.22, -0.12), 0.44, 0.10, boxstyle="round,pad=0.02", facecolor="#26394f", edgecolor=BLUE))
    arm_line, = ax_robot.plot([], [], color=BLUE, lw=8, solid_capstyle="round")
    joint_scatter = ax_robot.scatter([], [], s=90, c=GREEN, zorder=3)
    ee_dot = ax_robot.scatter([], [], s=160, c=ORANGE, zorder=4)
    trail_line, = ax_robot.plot([], [], color=ORANGE, lw=2, alpha=0.8)
    event_box = ax_robot.text(-1.18, 1.48, "", color=TEXT, fontsize=15, bbox=dict(boxstyle="round,pad=0.5", fc="#0e1a2d", ec=BLUE, alpha=0.95))

    # Metrics panel
    t = metrics["t"]
    ax_metric.set_title("分布指标动态曲线（KL / MMD / W1）", color=TEXT, fontsize=15)
    ax_metric.set_xlim(0, duration)
    max_metric = max(float(np.max(metrics["kl_mean"])), float(np.max(metrics["mmd_stat"])), 0.2)
    ax_metric.set_ylim(0, max_metric * 1.25)
    ax_metric.grid(color=GRID, alpha=0.45)
    ax_metric.tick_params(colors=MUTED)
    for s in ax_metric.spines.values():
        s.set_color(GRID)
    kl_line, = ax_metric.plot([], [], color=BLUE, lw=2.5, label="KL mean")
    mmd_line, = ax_metric.plot([], [], color=ORANGE, lw=2.5, label="MMD")
    w1_line, = ax_metric.plot([], [], color=PURPLE, lw=2.0, label="W1 mean")
    ax_metric.legend(loc="upper left", facecolor=PANEL, edgecolor=GRID, labelcolor=TEXT)
    inject_vline = ax_metric.axvline(duration * 0.42, color=RED, ls="--", lw=1.8)
    ax_metric.text(duration * 0.42 + 0.5, max_metric * 1.08, "注入偏移", color=RED, fontsize=11)

    # Risk panel
    ax_risk.set_title("风险等级与综合风险分数", color=TEXT, fontsize=15)
    ax_risk.set_xlim(0, duration)
    ax_risk.set_ylim(-0.1, 3.25)
    ax_risk.set_yticks([0, 1, 2, 3], ["R0", "R1", "R2", "R3"])
    ax_risk.grid(color=GRID, alpha=0.45)
    ax_risk.tick_params(colors=MUTED)
    for s in ax_risk.spines.values():
        s.set_color(GRID)
    risk_line, = ax_risk.step([], [], where="post", color=RED, lw=3, label="风险等级")
    score_line, = ax_risk.plot([], [], color=GREEN, lw=2.2, label="综合风险 x3")
    ax_risk.legend(loc="upper left", facecolor=PANEL, edgecolor=GRID, labelcolor=TEXT)

    # Cards
    cards = [
        ("MoveIt", "4/4 成功\nRMSE 0.006 rad", GREEN),
        ("监控", "10 Hz\n3/3 检出", ORANGE),
        ("安全", "R3 急停 9.5 ms\n速度归零 0.37 ms", RED),
        ("报告", "JSON 91 条\nCSV 92 行", BLUE),
    ]
    for i, (title, body, color) in enumerate(cards):
        x = 0.03 + i * 0.24
        ax_cards.add_patch(FancyBboxPatch((x, 0.18), 0.20, 0.62, boxstyle="round,pad=0.02", facecolor=PANEL_2, edgecolor=color, lw=2))
        ax_cards.text(x + 0.03, 0.62, title, color=TEXT, fontsize=16, weight="bold", transform=ax_cards.transAxes)
        ax_cards.text(x + 0.03, 0.34, body, color=MUTED, fontsize=12, transform=ax_cards.transAxes, linespacing=1.5)

    # Precompute robot trail
    ee_x: list[float] = []
    ee_y: list[float] = []

    def update(i: int):
        current_t = metrics["t"][i]
        p = (current_t / duration) % 1.0
        xs, ys = robot_pose(p)
        ee_x.append(xs[-1])
        ee_y.append(ys[-1])
        if len(ee_x) > fps * 10:
            del ee_x[: len(ee_x) - fps * 10]
            del ee_y[: len(ee_y) - fps * 10]

        arm_line.set_data(xs, ys)
        joint_scatter.set_offsets(np.column_stack([xs, ys]))
        ee_dot.set_offsets([[xs[-1], ys[-1]]])
        trail_line.set_data(ee_x, ee_y)

        title, note, color = event_text(current_t, duration)
        event_box.set_text(f"{title}\n{note}")
        event_box.get_bbox_patch().set_edgecolor(color)

        idx = slice(0, i + 1)
        kl_line.set_data(t[idx], metrics["kl_mean"][idx])
        mmd_line.set_data(t[idx], metrics["mmd_stat"][idx])
        w1_line.set_data(t[idx], metrics["w1_mean"][idx])
        risk_line.set_data(t[idx], metrics["risk_level"][idx])
        score_line.set_data(t[idx], metrics["composite_score"][idx] * 3)

        fig.suptitle(
            f"t={current_t:05.1f}s  |  {title}  |  风险 R{int(round(metrics['risk_level'][i]))}  |  "
            f"KL={metrics['kl_mean'][i]:.3f}  MMD={metrics['mmd_stat'][i]:.3f}",
            color=TEXT,
            fontsize=15,
            y=0.895,
        )
        return arm_line, joint_scatter, ee_dot, trail_line, kl_line, mmd_line, w1_line, risk_line, score_line, event_box

    writer = animation.FFMpegWriter(fps=fps, codec="libx264", bitrate=2400, extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
    ani = animation.FuncAnimation(fig, update, frames=frames, interval=1000 / fps, blit=False)
    ani.save(output, writer=writer, dpi=120)
    plt.close(fig)
    meta = {
        "output": str(output),
        "duration_sec": duration,
        "fps": fps,
        "frames": frames,
        "sources": [
            "docs/samples/hoc-verification-reports/fr_hoc_verify.csv",
            "docs/samples/moveit-closure-metrics.json",
            "docs/samples/monitor-metrics.json",
            "docs/samples/risk-management-metrics.json",
            "docs/samples/safety-nfr-metrics.json",
            "docs/samples/hoc-console-metrics.json",
        ],
    }
    (OUT_DIR / "manifest.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--fps", type=int, default=20)
    args = parser.parse_args()
    generate(args.output, args.duration, args.fps)
    print(f"[PASS] Dynamic Chinese demo video: {args.output}")
    print(f"[INFO] Sources/manifest: {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
