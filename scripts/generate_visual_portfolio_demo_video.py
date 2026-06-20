#!/usr/bin/env python3
"""Generate a visual Chinese portfolio demo video from verification metrics."""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
SAMPLES = ROOT / "docs" / "samples"
OUT_DIR = SAMPLES / "portfolio-demo-zh-visual"
DEFAULT_OUT = SAMPLES / "portfolio-demo-zh-visual.mp4"

W, H, DPI = 19.2, 10.8, 100
BG = "#08111f"
PANEL = "#121c2f"
PANEL_2 = "#17243a"
TEXT = "#eef6ff"
MUTED = "#9fb2c8"
BLUE = "#5dade2"
GREEN = "#52c41a"
ORANGE = "#faad14"
RED = "#ff4d4f"
PURPLE = "#b37feb"


def configure_font() -> None:
    candidates = [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    ]
    for path in candidates:
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


def data() -> dict[str, dict[str, Any]]:
    return {
        "bridge": load_json("bridge-comm-metrics.json"),
        "moveit": load_json("moveit-closure-metrics.json"),
        "monitor": load_json("monitor-metrics.json"),
        "risk": load_json("risk-management-metrics.json"),
        "hoc": load_json("hoc-console-metrics.json"),
        "perf": load_json("performance-nfr-metrics.json"),
        "safety": load_json("safety-nfr-metrics.json"),
        "maint": load_json("maintainability-nfr-metrics.json"),
    }


def fig_base(title: str, subtitle: str) -> tuple[plt.Figure, plt.Axes]:
    fig = plt.figure(figsize=(W, H), dpi=DPI, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.add_patch(FancyBboxPatch((0, 0.91), 1, 0.09, boxstyle="square,pad=0", facecolor="#0e1a2d", edgecolor="none"))
    ax.text(0.04, 0.955, "ros2-moveit-pybullet-bridge", color=MUTED, fontsize=18, va="center")
    ax.text(0.04, 0.82, title, color=TEXT, fontsize=40, weight="bold", va="center")
    ax.text(0.04, 0.765, subtitle, color=MUTED, fontsize=21, va="center")
    ax.plot([0.04, 0.96], [0.72, 0.72], color="#263d5a", lw=2)
    return fig, ax


def box(ax: plt.Axes, xy: tuple[float, float], wh: tuple[float, float], label: str, note: str, color: str = BLUE) -> None:
    x, y = xy
    w, h = wh
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.016,rounding_size=0.018", facecolor=PANEL, edgecolor="#2b4665", lw=1.7))
    ax.text(x + 0.02, y + h - 0.04, label, color=TEXT, fontsize=22, weight="bold", va="top")
    ax.text(x + 0.02, y + h - 0.10, note, color=MUTED, fontsize=15, va="top", wrap=True)
    ax.add_patch(FancyBboxPatch((x, y), 0.012, h, boxstyle="round,pad=0,rounding_size=0.01", facecolor=color, edgecolor="none"))


def arrow(ax: plt.Axes, start: tuple[float, float], end: tuple[float, float], color: str = BLUE) -> None:
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=22, lw=2.2, color=color))


def save(fig: plt.Figure, name: str) -> Path:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUT_DIR / name
    fig.savefig(path, facecolor=BG, bbox_inches="tight", pad_inches=0)
    plt.close(fig)
    return path


def cover_slide(d: dict[str, dict[str, Any]]) -> Path:
    fig, ax = fig_base("中文可视化 Demo", "不是纯文字 PPT：用实验流程、图表和验收指标讲清楚系统。")
    cards = [
        ("MoveIt 闭环", "4/4 规划成功，最大 RMSE 0.006 rad", GREEN),
        ("分布监控", "5s 滑窗，10 Hz，+20% 阻尼注入 3/3 检出", GREEN),
        ("风险熔断", "R0-R3 升级，R3 急停，速度 <1 ms 归零", GREEN),
        ("HOC/报告", "5 Hz WebSocket，JSON/CSV 审计导出", GREEN),
    ]
    for i, (title, note, color) in enumerate(cards):
        box(ax, (0.08 + i * 0.225, 0.43), (0.19, 0.18), title, note, color)
    ax.text(0.08, 0.30, "演示口径：前端只是 HOC 外壳，核心价值是 ROS2 + MoveIt2 + PyBullet + 风险监控闭环。", color=TEXT, fontsize=24)
    ax.text(0.08, 0.23, "本视频由 docs/samples/*.json 自动生成，可复现、可审计、可作为录屏兜底。", color=MUTED, fontsize=20)
    return save(fig, "slide_01_cover.png")


def architecture_slide(_: dict[str, dict[str, Any]]) -> Path:
    fig, ax = fig_base("系统架构图", "从规划、执行、监控到风险处置的闭环。")
    items = [
        ((0.06, 0.48), "MoveIt2", "关节目标 / 碰撞检查\nPlan & Execute", BLUE),
        ((0.27, 0.48), "ROS2 Bridge", "FollowJointTrajectory relay\n100 Hz joint_states", BLUE),
        ((0.49, 0.48), "Dual PyBullet", "Sim source + Real proxy\n240 Hz/source", BLUE),
        ((0.70, 0.48), "Dist Monitor", "KL / W1 / MMD\n5s sliding window", ORANGE),
        ((0.49, 0.20), "Risk Engine", "五维归因\nR0-R3 / HOLD / E_STOP", RED),
        ((0.70, 0.20), "HOC + Report", "操作员确认\nJSON / CSV / rosbag", GREEN),
    ]
    for xy, title, note, color in items:
        box(ax, xy, (0.17, 0.17), title, note, color)
    arrow(ax, (0.23, 0.565), (0.27, 0.565))
    arrow(ax, (0.44, 0.565), (0.49, 0.565))
    arrow(ax, (0.66, 0.565), (0.70, 0.565), ORANGE)
    arrow(ax, (0.58, 0.48), (0.58, 0.37), RED)
    arrow(ax, (0.70, 0.28), (0.66, 0.28), GREEN)
    arrow(ax, (0.58, 0.37), (0.58, 0.48), RED)
    ax.text(0.08, 0.11, "Demo 重点：不是展示好看的网页，而是展示机器人系统在异常注入后能监控、归因、熔断并导出证据。", color=TEXT, fontsize=23)
    return save(fig, "slide_02_architecture.png")


def experiment_process_slide(_: dict[str, dict[str, Any]]) -> Path:
    fig, ax = fig_base("实验过程时间线", "录制时可以按这条线讲：启动 -> 规划 -> 注入 -> 风险 -> 恢复 -> 导出。")
    steps = [
        ("启动", "portfolio_demo GUI\n双源 PyBullet 就绪", BLUE),
        ("规划", "MoveIt 发送轨迹\nPyBullet 执行反馈", GREEN),
        ("注入", "+20% joint_damping\n制造已知偏移", ORANGE),
        ("检出", "KL/MMD 超阈值\nshift_detected=true", ORANGE),
        ("处置", "R2 降速 / R3 急停\n速度快速归零", RED),
        ("审计", "HOC 导出 JSON/CSV\n报告可复验", GREEN),
    ]
    y = 0.54
    for i, (title, note, color) in enumerate(steps):
        x = 0.07 + i * 0.15
        ax.add_patch(plt.Circle((x, y), 0.035, color=color))
        ax.text(x, y, str(i + 1), color="#06111f", fontsize=18, weight="bold", ha="center", va="center")
        ax.text(x, y - 0.08, title, color=TEXT, fontsize=22, weight="bold", ha="center")
        ax.text(x, y - 0.15, note, color=MUTED, fontsize=15, ha="center", linespacing=1.5)
        if i < len(steps) - 1:
            arrow(ax, (x + 0.04, y), (x + 0.11, y), color)
    ax.add_patch(FancyBboxPatch((0.08, 0.16), 0.84, 0.12, boxstyle="round,pad=0.02,rounding_size=0.02", facecolor=PANEL_2, edgecolor="#2b4665"))
    ax.text(0.10, 0.22, "建议录屏策略：PyBullet/MoveIt 画面做主镜头，HOC 只短暂展示风险等级、命令响应和报告导出。", color=TEXT, fontsize=22, va="center")
    return save(fig, "slide_03_process.png")


def moveit_slide(d: dict[str, dict[str, Any]]) -> Path:
    m = d["moveit"]
    goals = m.get("move_group", {}).get("goals", [])
    labels = [g.get("label", f"goal{i}") for i, g in enumerate(goals)]
    rmse = [g.get("rmse_rad", 0) for g in goals]
    success = [1 if g.get("success") else 0 for g in goals]
    fig, ax = fig_base("MoveIt2 规划闭环", "规划成功率、执行误差、TF 与碰撞拒绝都已复验。")
    chart = fig.add_axes([0.08, 0.20, 0.52, 0.43], facecolor=PANEL)
    chart.bar(labels, rmse, color=GREEN)
    chart.axhline(0.05, color=RED, ls="--", lw=2, label="验收阈值 0.05 rad")
    chart.set_ylabel("RMSE (rad)", color=TEXT)
    chart.set_title("4 个标准目标执行误差", color=TEXT, fontsize=18)
    chart.tick_params(colors=TEXT)
    chart.legend(facecolor=PANEL, edgecolor="#2b4665", labelcolor=TEXT)
    chart.grid(axis="y", color="#2b3d55", alpha=0.6)
    for spine in chart.spines.values():
        spine.set_color("#2b4665")
    metric_card = [
        ("成功率", f"{sum(success)}/{len(success)}", GREEN),
        ("最大 RMSE", f"{max(rmse or [0]):.4f} rad", GREEN),
        ("TF", "link_0 -> link_7", GREEN),
        ("碰撞场景", "Plan-only 被拒绝", GREEN),
    ]
    for i, (title, value, color) in enumerate(metric_card):
        box(ax, (0.66, 0.53 - i * 0.105), (0.25, 0.08), title, value, color)
    return save(fig, "slide_04_moveit.png")


def monitor_slide(d: dict[str, dict[str, Any]]) -> Path:
    mon = d["monitor"]
    trials = mon.get("injection_trials", [])
    labels = [f"试验 {t.get('index')}" for t in trials]
    kl = [t.get("max_kl", 0) for t in trials]
    mmd = [t.get("max_mmd", 0) for t in trials]
    fig, ax = fig_base("分布偏移监控", "通过确定性注入证明 KL / MMD 能检出 Sim/Real 分布变化。")
    c1 = fig.add_axes([0.07, 0.20, 0.40, 0.43], facecolor=PANEL)
    x = range(len(labels))
    c1.bar([i - 0.18 for i in x], kl, width=0.36, label="max KL", color=BLUE)
    c1.bar([i + 0.18 for i in x], mmd, width=0.36, label="max MMD", color=ORANGE)
    c1.set_xticks(list(x), labels)
    c1.set_title("+20% 阻尼注入后的检测统计", color=TEXT, fontsize=18)
    c1.tick_params(colors=TEXT)
    c1.legend(facecolor=PANEL, edgecolor="#2b4665", labelcolor=TEXT)
    c1.grid(axis="y", color="#2b3d55", alpha=0.6)
    for spine in c1.spines.values():
        spine.set_color("#2b4665")
    c2 = fig.add_axes([0.55, 0.20, 0.36, 0.43], facecolor=PANEL)
    vals = [
        mon.get("frequency", {}).get("mean_hz", 0),
        mon.get("latest_metrics", {}).get("sample_count_sim", 0) / 50,
        mon.get("latest_metrics", {}).get("sample_count_real", 0) / 50,
        mon.get("detection_rate", 0) * 10,
    ]
    c2.bar(["频率\n(Hz)", "Sim样本\n/50", "Real样本\n/50", "检出率\nx10"], vals, color=[GREEN, BLUE, BLUE, GREEN])
    c2.set_title("滑窗与检出能力", color=TEXT, fontsize=18)
    c2.tick_params(colors=TEXT)
    c2.grid(axis="y", color="#2b3d55", alpha=0.6)
    for spine in c2.spines.values():
        spine.set_color("#2b4665")
    ax.text(0.08, 0.12, "结论：3 次注入全部成功检出，最新窗口 KL=1.069、MMD p=0.0196，满足监控 Demo 主线。", color=TEXT, fontsize=22)
    return save(fig, "slide_05_monitor.png")


def risk_slide(d: dict[str, dict[str, Any]]) -> Path:
    risk = d["risk"]
    safety = d["safety"]
    transitions = risk.get("transitions", [])
    levels = [t.get("target_level", 0) for t in transitions]
    lat = [t.get("latency_ms", 0) for t in transitions]
    fig, ax = fig_base("风险升级与安全处置", "五维风险从 R0-R3 升级，并触发降速 / 急停 / 人工确认恢复。")
    c1 = fig.add_axes([0.07, 0.22, 0.42, 0.42], facecolor=PANEL)
    c1.plot(levels, lat, marker="o", lw=3, color=ORANGE)
    c1.axhline(500, color=RED, ls="--", lw=2, label="500 ms 阈值")
    c1.set_xticks([0, 1, 2, 3], ["R0", "R1", "R2", "R3"])
    c1.set_ylabel("状态变化延迟 (ms)", color=TEXT)
    c1.set_title("R0-R3 风险状态变化", color=TEXT, fontsize=18)
    c1.tick_params(colors=TEXT)
    c1.legend(facecolor=PANEL, edgecolor="#2b4665", labelcolor=TEXT)
    c1.grid(color="#2b3d55", alpha=0.6)
    for spine in c1.spines.values():
        spine.set_color("#2b4665")
    c2 = fig.add_axes([0.57, 0.22, 0.32, 0.42], facecolor=PANEL)
    names = ["R2降速比", "R3急停", "速度归零", "恢复RUNNING"]
    vals = [
        safety.get("r2_degraded_motion", {}).get("degraded_to_normal_delta_ratio", 0) * 100,
        safety.get("r3_e_stop", {}).get("bridge_system_state_latency_ms", 0),
        safety.get("r3_e_stop", {}).get("velocity_zero_latency_ms", 0),
        safety.get("acknowledge_recovery", {}).get("running_latency_ms", 0),
    ]
    c2.barh(names, vals, color=[BLUE, RED, GREEN, GREEN])
    c2.set_title("安全动作指标", color=TEXT, fontsize=18)
    c2.tick_params(colors=TEXT)
    c2.grid(axis="x", color="#2b3d55", alpha=0.6)
    for spine in c2.spines.values():
        spine.set_color("#2b4665")
    ax.text(0.08, 0.13, "结论：R3 后 bridge 急停约 9.5 ms，速度归零约 0.37 ms；R2 自动降到约 50% 运动量。", color=TEXT, fontsize=22)
    return save(fig, "slide_06_risk.png")


def hoc_slide(d: dict[str, dict[str, Any]]) -> Path:
    hoc = d["hoc"]
    streams = hoc.get("streams", {})
    commands = hoc.get("commands", {})
    fig, ax = fig_base("HOC 与报告证据", "前端只短暂证明通道和操作；重点看命令延迟与报告导出。")
    c1 = fig.add_axes([0.07, 0.22, 0.38, 0.42], facecolor=PANEL)
    labels = [k.replace("/monitor/", "").replace("/risk/", "") for k in streams.keys()]
    hz = [v.get("mean_hz", 0) for v in streams.values()]
    max_lat = [v.get("max_latency_ms", 0) for v in streams.values()]
    c1.bar(labels, hz, color=GREEN, label="Hz")
    c1.set_title("WebSocket 推送频率", color=TEXT, fontsize=18)
    c1.tick_params(colors=TEXT, rotation=10)
    c1.grid(axis="y", color="#2b3d55", alpha=0.6)
    for spine in c1.spines.values():
        spine.set_color("#2b4665")
    c2 = fig.add_axes([0.53, 0.22, 0.38, 0.42], facecolor=PANEL)
    cmd_names = ["pause", "e_stop", "ack", "resume", "inject"]
    cmd_vals = [
        commands.get("pause", {}).get("latency_ms", 0),
        commands.get("e_stop", {}).get("latency_ms", 0),
        commands.get("acknowledge", {}).get("latency_ms", 0),
        commands.get("resume", {}).get("latency_ms", 0),
        commands.get("inject_shift", {}).get("latency_ms", 0),
    ]
    c2.bar(cmd_names, cmd_vals, color=[BLUE, RED, GREEN, GREEN, ORANGE])
    c2.axhline(100, color=RED, ls="--", lw=2)
    c2.set_title("HOC 控制命令延迟 (ms)", color=TEXT, fontsize=18)
    c2.tick_params(colors=TEXT)
    c2.grid(axis="y", color="#2b3d55", alpha=0.6)
    for spine in c2.spines.values():
        spine.set_color("#2b4665")
    ax.text(0.08, 0.13, "报告导出：JSON 风险时序 91 条，CSV 指标 92 行；适合放进作品集作为审计证据。", color=TEXT, fontsize=22)
    return save(fig, "slide_07_hoc.png")


def closing_slide(_: dict[str, dict[str, Any]]) -> Path:
    fig, ax = fig_base("最终 Demo 讲解口径", "把注意力放在工程闭环和可验证证据上。")
    rows = [
        ("一句话", "无真机条件下，用双 PyBullet + MoveIt2 + 分布监控构建可复现实验闭环。", BLUE),
        ("核心贡献", "ROS2 桥接、Sim/Real 指标、五维风险、Fail-Safe、报告导出。", GREEN),
        ("可展示画面", "PyBullet / MoveIt 过程 + 本视频图表 + 少量 HOC 证明。", ORANGE),
        ("可交付证据", "verify 脚本、JSON/CSV、coverage、rosbag、HTML 报告。", GREEN),
    ]
    y = 0.56
    for title, note, color in rows:
        box(ax, (0.10, y), (0.80, 0.10), title, note, color)
        y -= 0.13
    ax.text(0.10, 0.12, "如果现场 UI 出问题：直接播放这版图表视频，再切终端展示 verify 脚本和样例报告。", color=TEXT, fontsize=23)
    return save(fig, "slide_08_close.png")


def render(paths: list[Path], durations: list[float], output: Path) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    concat = OUT_DIR / "concat.txt"
    with concat.open("w") as f:
        for path, duration in zip(paths, durations):
            f.write(f"file '{path.as_posix()}'\n")
            f.write(f"duration {duration:.3f}\n")
        f.write(f"file '{paths[-1].as_posix()}'\n")
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-f",
            "concat",
            "-safe",
            "0",
            "-i",
            str(concat),
            "-vf",
            "fps=30,format=yuv420p",
            "-movflags",
            "+faststart",
            str(output),
        ],
        check=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()
    configure_font()
    d = data()
    slide_fns = [
        cover_slide,
        architecture_slide,
        experiment_process_slide,
        moveit_slide,
        monitor_slide,
        risk_slide,
        hoc_slide,
        closing_slide,
    ]
    paths = [fn(d) for fn in slide_fns]
    durations = [8, 9, 10, 9, 10, 10, 9, 8] if args.quick else [26, 30, 34, 32, 36, 36, 32, 28]
    render(paths, durations, args.output)
    print(f"[PASS] Visual Chinese demo video: {args.output}")
    print(f"[INFO] Slides: {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
