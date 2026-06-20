#!/usr/bin/env python3
"""Generate milestone README assets (PNG/GIF/SVG) without GUI."""

from __future__ import annotations

import math
import os
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'docs' / 'assets'
ASSETS.mkdir(parents=True, exist_ok=True)

LINK1 = 0.35
LINK2 = 0.30


def _arm_xy(j1: float, j2: float) -> tuple[np.ndarray, np.ndarray]:
    x0, y0 = 0.0, 0.0
    x1 = LINK1 * math.cos(j1)
    y1 = LINK1 * math.sin(j1)
    x2 = x1 + LINK2 * math.cos(j1 + j2)
    y2 = y1 + LINK2 * math.sin(j1 + j2)
    return np.array([x0, x1, x2]), np.array([y0, y1, y2])


def generate_m1_gif() -> None:
    frames = 40
    amplitude = 0.8
    times = np.linspace(0, 4.0, frames)
    j1 = amplitude * np.sin(np.pi * times / 4.0)
    j2 = amplitude * np.cos(np.pi * times / 4.0)

    fig, (ax_arm, ax_plot) = plt.subplots(1, 2, figsize=(8, 3.5))
    fig.suptitle('M1: PyBullet 2-DOF Joint Sweep', fontsize=12, fontweight='bold')

    (line_arm,) = ax_arm.plot([], [], '-o', color='#e67e22', lw=3, markersize=6)
    base = patches.Rectangle((-0.09, -0.09), 0.18, 0.18, color='#7f8c8d')
    ax_arm.add_patch(base)
    ax_arm.set_xlim(-0.75, 0.75)
    ax_arm.set_ylim(-0.2, 0.75)
    ax_arm.set_aspect('equal')
    ax_arm.set_title('Planar 2-DOF Arm')
    ax_arm.grid(True, alpha=0.3)

    (line_j1,) = ax_plot.plot([], [], label='joint1', color='#3498db')
    (line_j2,) = ax_plot.plot([], [], label='joint2', color='#2ecc71')
    ax_plot.set_xlim(0, 4)
    ax_plot.set_ylim(-1.0, 1.0)
    ax_plot.set_xlabel('time (s)')
    ax_plot.set_ylabel('position (rad)')
    ax_plot.set_title('/bridge/sim/joint_states')
    ax_plot.legend(loc='upper right')
    ax_plot.grid(True, alpha=0.3)

    def _update(i: int):
        xs, ys = _arm_xy(j1[i], j2[i])
        line_arm.set_data(xs, ys)
        line_j1.set_data(times[: i + 1], j1[: i + 1])
        line_j2.set_data(times[: i + 1], j2[: i + 1])
        return line_arm, line_j1, line_j2

    import matplotlib.animation as animation

    anim = animation.FuncAnimation(fig, _update, frames=frames, interval=80, blit=True)
    out = ASSETS / 'm1-joint-sweep.gif'
    anim.save(out, writer='pillow', dpi=100)
    plt.close(fig)
    print(f'wrote {out}')


def generate_m1_pybullet_png() -> None:
    try:
        import pybullet as p
        import pybullet_data
    except ImportError:
        print('skip m1-pybullet.png (pybullet not installed)')
        return

    urdf = ROOT / 'pybullet_bridge' / 'urdf' / 'planar_2dof.urdf'
    if not urdf.is_file():
        print('skip m1-pybullet.png (URDF missing)')
        return

    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation(physicsClientId=client)
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    robot = p.loadURDF(str(urdf), useFixedBase=True, physicsClientId=client)
    for idx, angle in enumerate((0.8, -0.6)):
        p.resetJointState(robot, idx, angle, physicsClientId=client)

    view = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0.25, 0.0, 0.55],
        distance=1.35,
        yaw=50,
        pitch=-18,
        roll=0,
        upAxisIndex=2,
    )
    proj = p.computeProjectionMatrixFOV(fov=55, aspect=1.0, nearVal=0.1, farVal=3.0)
    _, _, rgba, _, _ = p.getCameraImage(
        640, 480, view, proj, renderer=p.ER_TINY_RENDERER, physicsClientId=client)
    img = np.reshape(rgba, (480, 640, 4))[:, :, :3].astype(np.uint8)
    out = ASSETS / 'm1-pybullet.png'
    plt.imsave(out, img)
    p.disconnect(client)
    print(f'wrote {out}')


def generate_m3_gif() -> None:
    frames = 36
    t = np.linspace(0, 2 * np.pi, frames)
    sim = 0.5 * np.sin(t)
    drift = np.linspace(0, 0.6, frames)
    real = 0.5 * np.sin(t + 0.15) + drift

    fig, ax = plt.subplots(figsize=(7, 3.5))
    ax.set_title('M3: Dual-Source Domain Randomization (Sim vs Real)', fontweight='bold')
    ax.set_xlabel('sample index')
    ax.set_ylabel('joint1 position (rad)')
    ax.grid(True, alpha=0.3)
    (line_sim,) = ax.plot([], [], color='#3498db', lw=2, label='Sim-Source (ideal)')
    (line_real,) = ax.plot([], [], color='#e74c3c', lw=2, label='Real-Source (noisy)')
    ax.legend()
    ax.set_xlim(0, frames - 1)
    ax.set_ylim(-0.2, 1.2)

    import matplotlib.animation as animation

    def _update(i: int):
        line_sim.set_data(np.arange(i + 1), sim[: i + 1])
        line_real.set_data(np.arange(i + 1), real[: i + 1])
        return line_sim, line_real

    anim = animation.FuncAnimation(fig, _update, frames=frames, interval=90, blit=True)
    out = ASSETS / 'm3-dual-source.gif'
    anim.save(out, writer='pillow', dpi=100)
    plt.close(fig)
    print(f'wrote {out}')


def generate_m4_png() -> None:
    joints = [f'iiwa_joint_{i}' for i in range(1, 8)]
    kl = np.array([0.08, 0.12, 0.21, 0.15, 0.09, 0.11, 0.10])
    w1 = np.array([0.04, 0.06, 0.09, 0.07, 0.05, 0.06, 0.05])
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.2))
    fig.suptitle('M4: Distribution Monitor Metrics (KL / W1 / MMD)', fontweight='bold')

    axes[0].bar(joints, kl, color='#9b59b6')
    axes[0].axhline(0.15, color='#e74c3c', ls='--', label='KL threshold')
    axes[0].set_title('KL divergence per joint')
    axes[0].set_ylabel('KL(P||Q)')
    axes[0].legend(fontsize=8)
    axes[0].grid(True, axis='y', alpha=0.3)
    plt.setp(axes[0].get_xticklabels(), rotation=30, ha='right')

    axes[1].bar(joints, w1, color='#8e44ad')
    axes[1].axhline(0.08, color='#e74c3c', ls='--', label='W1 threshold')
    axes[1].set_title('Wasserstein-1 per joint')
    axes[1].set_ylabel('W1')
    axes[1].legend(fontsize=8)
    axes[1].grid(True, axis='y', alpha=0.3)
    plt.setp(axes[1].get_xticklabels(), rotation=30, ha='right')

    labels = ['MMD stat', 'p-value', 'mean KL']
    values = [0.062, 0.018, kl.mean()]
    colors = ['#2980b9', '#27ae60', '#8e44ad']
    axes[2].bar(labels, values, color=colors)
    axes[2].set_title('Shift detection summary')
    axes[2].set_ylim(0, 0.15)
    axes[2].grid(True, axis='y', alpha=0.3)
    axes[2].text(1, 0.018, 'shift detected', ha='center', va='bottom', color='#c0392b', fontsize=8)

    fig.tight_layout()
    out = ASSETS / 'm4-monitor-metrics.png'
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f'wrote {out}')


def _write_svg(path: Path, body: str, width: int = 900, height: int = 320) -> None:
    svg = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#f8f9fb"/>
  {body}
</svg>
'''
    path.write_text(svg, encoding='utf-8')
    print(f'wrote {path}')


def generate_m2_svg() -> None:
    body = '''
  <text x="450" y="28" text-anchor="middle" font-size="18" font-weight="bold" fill="#2c3e50">M2: MoveIt2 Planning Loop (UR5 tutorial)</text>
  <rect x="30" y="70" width="130" height="60" rx="8" fill="#3498db" opacity="0.9"/>
  <text x="95" y="105" text-anchor="middle" fill="white" font-size="13">RViz / MoveIt2</text>
  <polygon points="170,100 200,100 200,90 220,110 200,130 200,120 170,120" fill="#95a5a6"/>
  <rect x="230" y="70" width="150" height="60" rx="8" fill="#e67e22" opacity="0.95"/>
  <text x="305" y="98" text-anchor="middle" fill="white" font-size="12">FollowJointTrajectory</text>
  <text x="305" y="116" text-anchor="middle" fill="white" font-size="11">trajectory_controller</text>
  <polygon points="390,100 420,100 420,90 440,110 420,130 420,120 390,120" fill="#95a5a6"/>
  <rect x="450" y="70" width="130" height="60" rx="8" fill="#2ecc71" opacity="0.95"/>
  <text x="515" y="105" text-anchor="middle" fill="white" font-size="13">PyBullet Bridge</text>
  <polygon points="590,100 620,100 620,90 640,110 620,130 620,120 590,120" fill="#95a5a6"/>
  <rect x="650" y="70" width="120" height="60" rx="8" fill="#9b59b6" opacity="0.95"/>
  <text x="710" y="98" text-anchor="middle" fill="white" font-size="12">UR5 Sim</text>
  <text x="710" y="116" text-anchor="middle" fill="white" font-size="11">/joint_states</text>
  <text x="450" y="190" text-anchor="middle" font-size="13" fill="#555">Plan → Execute → PyBullet executes trajectory → feedback to MoveIt</text>
  <rect x="120" y="220" width="660" height="70" rx="10" fill="#ecf0f1" stroke="#bdc3c7"/>
  <text x="450" y="248" text-anchor="middle" font-size="12" fill="#2c3e50">ros2 launch moveit_config m2_demo.launch.py sim_mode:=DIRECT ur_type:=ur5</text>
  <text x="450" y="272" text-anchor="middle" font-size="12" fill="#7f8c8d">Planning Group: ur_manipulator · Interact tool0 · Plan → Execute</text>
'''
    _write_svg(ASSETS / 'm2-moveit-pipeline.svg', body)


def generate_m2_iiwa_svg() -> None:
    body = '''
  <text x="450" y="28" text-anchor="middle" font-size="18" font-weight="bold" fill="#2c3e50">M2: MoveIt2 Planning Loop (KUKA iiwa7 — portfolio mainline)</text>
  <rect x="30" y="70" width="130" height="60" rx="8" fill="#3498db" opacity="0.9"/>
  <text x="95" y="105" text-anchor="middle" fill="white" font-size="13">RViz / MoveIt2</text>
  <polygon points="170,100 200,100 200,90 220,110 200,130 200,120 170,120" fill="#95a5a6"/>
  <rect x="230" y="70" width="150" height="60" rx="8" fill="#e67e22" opacity="0.95"/>
  <text x="305" y="98" text-anchor="middle" fill="white" font-size="12">FollowJointTrajectory</text>
  <text x="305" y="116" text-anchor="middle" fill="white" font-size="11">trajectory_controller</text>
  <polygon points="390,100 420,100 420,90 440,110 420,130 420,120 390,120" fill="#95a5a6"/>
  <rect x="450" y="70" width="130" height="60" rx="8" fill="#2ecc71" opacity="0.95"/>
  <text x="515" y="105" text-anchor="middle" fill="white" font-size="13">PyBullet Bridge</text>
  <polygon points="590,100 620,100 620,90 640,110 620,130 620,120 590,120" fill="#95a5a6"/>
  <rect x="650" y="70" width="120" height="60" rx="8" fill="#9b59b6" opacity="0.95"/>
  <text x="710" y="98" text-anchor="middle" fill="white" font-size="12">KUKA iiwa7</text>
  <text x="710" y="116" text-anchor="middle" fill="white" font-size="11">7-DOF · manipulator</text>
  <text x="450" y="190" text-anchor="middle" font-size="13" fill="#555">Plan → Execute → PyBullet iiwa profile → /bridge/sim/joint_states</text>
  <rect x="120" y="220" width="660" height="70" rx="10" fill="#ecf0f1" stroke="#bdc3c7"/>
  <text x="450" y="248" text-anchor="middle" font-size="12" fill="#2c3e50">ros2 launch moveit_config m2_iiwa_demo.launch.py sim_mode:=GUI</text>
  <text x="450" y="272" text-anchor="middle" font-size="12" fill="#7f8c8d">Planning Group: manipulator · Interact → Plan → Execute</text>
'''
    _write_svg(ASSETS / 'm2-iiwa-pipeline.svg', body)


def generate_m5_svg() -> None:
    body = '''
  <text x="450" y="28" text-anchor="middle" font-size="18" font-weight="bold" fill="#2c3e50">M5: HOC Console Architecture</text>
  <rect x="40" y="60" width="150" height="180" rx="10" fill="#34495e"/>
  <text x="115" y="90" text-anchor="middle" fill="white" font-size="13">ROS 2 Topics</text>
  <text x="115" y="120" text-anchor="middle" fill="#ecf0f1" font-size="11">/risk/status</text>
  <text x="115" y="142" text-anchor="middle" fill="#ecf0f1" font-size="11">/monitor/distribution_metrics</text>
  <text x="115" y="164" text-anchor="middle" fill="#ecf0f1" font-size="11">/risk/alerts</text>
  <polygon points="200,150 240,150 240,140 260,160 240,180 240,170 200,170" fill="#95a5a6"/>
  <rect x="270" y="95" width="160" height="90" rx="10" fill="#16a085"/>
  <text x="350" y="130" text-anchor="middle" fill="white" font-size="13">hoc_server</text>
  <text x="350" y="152" text-anchor="middle" fill="white" font-size="11">WebSocket :8765</text>
  <polygon points="440,140 480,140 480,130 500,150 480,170 480,160 440,160" fill="#95a5a6"/>
  <rect x="510" y="60" width="350" height="180" rx="12" fill="#ffffff" stroke="#bdc3c7" stroke-width="2"/>
  <text x="685" y="95" text-anchor="middle" fill="#2c3e50" font-size="14" font-weight="bold">Dashboard</text>
  <rect x="530" y="110" width="140" height="50" rx="6" fill="#fdecea" stroke="#e74c3c"/>
  <text x="600" y="140" text-anchor="middle" fill="#c0392b" font-size="13">R2 · shift detected</text>
  <rect x="690" y="110" width="150" height="50" rx="6" fill="#eaf4fb" stroke="#3498db"/>
  <text x="765" y="140" text-anchor="middle" fill="#2471a3" font-size="12">KL / MMD charts</text>
  <rect x="530" y="175" width="310" height="45" rx="6" fill="#f4f6f7"/>
  <text x="685" y="203" text-anchor="middle" fill="#566573" font-size="11">Export experiment · Acknowledge risk · E-Stop</text>
  <text x="450" y="290" text-anchor="middle" font-size="12" fill="#7f8c8d">Architecture diagram — see m5-hoc-dashboard.png for UI preview</text>
'''
    _write_svg(ASSETS / 'm5-hoc-console.svg', body, height=330)


def generate_m5_dashboard_svg() -> None:
    body = '''
  <rect width="100%" height="100%" fill="#0a0a0a"/>
  <text x="450" y="32" text-anchor="middle" font-size="18" font-weight="bold" fill="#e8e8e8">M5: HOC Dashboard (dark theme wireframe)</text>
  <rect x="20" y="48" width="860" height="44" rx="8" fill="#2b1d11" stroke="#fa8c16" stroke-width="2"/>
  <text x="40" y="76" fill="#faad14" font-size="14">R2 关注 · 综合风险 0.58 ↑ · 主因: distribution_shift</text>
  <rect x="20" y="104" width="270" height="200" rx="10" fill="#141414" stroke="#303030"/>
  <text x="155" y="128" text-anchor="middle" fill="#d9d9d9" font-size="13">五维风险雷达</text>
  <polygon points="155,190 195,160 175,220 135,220 115,160" fill="rgba(105,177,255,0.35)" stroke="#69b1ff"/>
  <rect x="306" y="104" width="270" height="200" rx="10" fill="#141414" stroke="#303030"/>
  <text x="441" y="128" text-anchor="middle" fill="#d9d9d9" font-size="13">Sim / Real 分布对比</text>
  <rect x="330" y="150" width="18" height="60" fill="#69b1ff"/><rect x="352" y="140" width="18" height="70" fill="#95de64"/>
  <rect x="380" y="150" width="18" height="55" fill="#69b1ff"/><rect x="402" y="145" width="18" height="65" fill="#95de64"/>
  <rect x="592" y="104" width="288" height="200" rx="10" fill="#141414" stroke="#303030"/>
  <text x="736" y="128" text-anchor="middle" fill="#d9d9d9" font-size="13">关节跟踪误差</text>
  <polyline points="620,260 660,220 700,230 740,200 780,210 820,190" fill="none" stroke="#b37feb" stroke-width="2"/>
  <rect x="20" y="318" width="860" height="90" rx="10" fill="#141414" stroke="#303030"/>
  <text x="40" y="342" fill="#d9d9d9" font-size="13">KL / MMD 时序（60s）</text>
  <polyline points="40,390 200,370 360,350 520,330 680,340 840,320" fill="none" stroke="#69b1ff" stroke-width="2"/>
  <text x="450" y="430" text-anchor="middle" fill="#8c8c8c" font-size="11">See m5-hoc-dashboard.png for rendered preview (generate_milestone_assets.py)</text>
'''
    _write_svg(ASSETS / 'm5-hoc-dashboard.svg', body, width=900, height=450)


IIWA_HOME = (0.0, 0.785398, 0.0, -1.570796, 0.0, 1.570796, 0.0)


def _iiwa_urdf_path() -> Path | None:
    bundled = ROOT / 'pybullet_bridge' / 'urdf' / 'kuka_iiwa' / 'model.urdf'
    if bundled.is_file():
        return bundled
    try:
        import pybullet_data

        fallback = Path(pybullet_data.getDataPath()) / 'kuka_iiwa' / 'model.urdf'
        if fallback.is_file():
            return fallback
    except ImportError:
        pass
    return None


def _render_pybullet_frame(
    client: int,
    robot: int,
    joint_positions: tuple[float, ...],
    *,
    width: int = 640,
    height: int = 480,
) -> np.ndarray:
    import pybullet as p

    for idx, angle in enumerate(joint_positions):
        p.resetJointState(robot, idx, angle, physicsClientId=client)
    view = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0.0, 0.0, 0.55],
        distance=1.85,
        yaw=48,
        pitch=-22,
        roll=0,
        upAxisIndex=2,
    )
    proj = p.computeProjectionMatrixFOV(fov=55, aspect=width / height, nearVal=0.1, farVal=4.0)
    _, _, rgba, _, _ = p.getCameraImage(
        width,
        height,
        view,
        proj,
        renderer=p.ER_TINY_RENDERER,
        physicsClientId=client,
    )
    return np.reshape(rgba, (height, width, 4))[:, :, :3].astype(np.uint8)


def generate_iiwa7_pybullet_png() -> None:
    try:
        import pybullet as p
        import pybullet_data
    except ImportError:
        print('skip iiwa7-pybullet.png (pybullet not installed)')
        return

    urdf = _iiwa_urdf_path()
    if urdf is None:
        print('skip iiwa7-pybullet.png (URDF missing)')
        return

    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation(physicsClientId=client)
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    robot = p.loadURDF(str(urdf), useFixedBase=True, physicsClientId=client)
    img = _render_pybullet_frame(client, robot, IIWA_HOME)
    out = ASSETS / 'm2-iiwa-pybullet.png'
    plt.imsave(out, img)
    p.disconnect(client)
    print(f'wrote {out}')


def generate_iiwa7_motion_gif() -> None:
    try:
        import pybullet as p
        import pybullet_data
    except ImportError:
        print('skip m2-iiwa-pybullet.gif (pybullet not installed)')
        return

    urdf = _iiwa_urdf_path()
    if urdf is None:
        print('skip m2-iiwa-pybullet.gif (URDF missing)')
        return

    frames = 24
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation(physicsClientId=client)
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    robot = p.loadURDF(str(urdf), useFixedBase=True, physicsClientId=client)

    images = []
    for i in range(frames):
        phase = 2.0 * math.pi * i / frames
        joints = list(IIWA_HOME)
        joints[1] += 0.35 * math.sin(phase)
        joints[3] += 0.25 * math.cos(phase)
        joints[5] += 0.2 * math.sin(phase * 1.5)
        img = _render_pybullet_frame(client, robot, tuple(joints), width=480, height=360)
        images.append(img)

    p.disconnect(client)

    import matplotlib.animation as animation
    from matplotlib.image import AxesImage

    fig, ax = plt.subplots(figsize=(6.4, 4.8))
    fig.patch.set_facecolor('#1a1a1a')
    ax.set_facecolor('#1a1a1a')
    ax.axis('off')
    ax.set_title('KUKA iiwa7 — PyBullet portfolio profile', color='#e8e8e8', fontsize=11)
    im: AxesImage = ax.imshow(images[0])

    def _update(i: int):
        im.set_array(images[i])
        return [im]

    anim = animation.FuncAnimation(fig, _update, frames=frames, interval=100, blit=True)
    out = ASSETS / 'm2-iiwa-pybullet.gif'
    anim.save(out, writer='pillow', dpi=100)
    plt.close(fig)
    print(f'wrote {out}')


def generate_m5_dashboard_png() -> None:
    """Dark-theme HOC dashboard preview (matplotlib; replace with browser screenshot when available)."""
    dims = ['dist_shift', 'tracking', 'dynamics', 'comm', 'planning']
    dim_labels = ['Distribution', 'Tracking', 'Dynamics', 'Comm', 'Planning']
    scores = np.array([0.72, 0.35, 0.28, 0.15, 0.08])
    angles = np.linspace(0, 2 * np.pi, len(dims), endpoint=False)
    radar = np.concatenate([scores, scores[:1]])
    ang_closed = np.concatenate([angles, angles[:1]])

    fig = plt.figure(figsize=(12, 7), facecolor='#0a0a0a')
    gs = fig.add_gridspec(2, 3, height_ratios=[0.12, 1], hspace=0.28, wspace=0.25)

    banner = fig.add_subplot(gs[0, :])
    banner.set_facecolor('#2b1d11')
    banner.set_xlim(0, 1)
    banner.set_ylim(0, 1)
    banner.axis('off')
    banner.text(
        0.02, 0.5,
        'R2 WARNING  ·  score 0.58 ↑  ·  driver: distribution_shift  ·  degraded 50%',
        color='#faad14', fontsize=12, va='center', fontweight='bold',
    )

    ax_radar = fig.add_subplot(gs[1, 0], polar=True, facecolor='#141414')
    ax_radar.set_theta_offset(np.pi / 2)
    ax_radar.set_theta_direction(-1)
    ax_radar.plot(ang_closed, radar, color='#69b1ff', linewidth=2)
    ax_radar.fill(ang_closed, radar, color='#69b1ff', alpha=0.25)
    ax_radar.set_xticks(angles)
    ax_radar.set_xticklabels(dim_labels, color='#d9d9d9', fontsize=9)
    ax_radar.set_ylim(0, 1)
    ax_radar.set_yticklabels([])
    ax_radar.grid(color='#434343', alpha=0.5)
    ax_radar.set_title('5D Risk Radar', color='#d9d9d9', pad=12)

    ax_kl = fig.add_subplot(gs[1, 1], facecolor='#141414')
    t = np.linspace(0, 60, 120)
    kl = 0.08 + 0.004 * t + 0.02 * np.sin(t / 5)
    mmd = 0.02 + 0.0015 * t
    comm = 0.05 + 0.003 * np.maximum(0, t - 35)
    ax_kl.plot(t, kl, color='#69b1ff', label='KL mean', lw=1.8)
    ax_kl.plot(t, mmd, color='#95de64', label='MMD', lw=1.5)
    ax_kl.plot(t, comm, color='#ffc069', label='comm_health', lw=1.5)
    ax_kl.axhline(0.15, color='#ff7875', ls='--', lw=1, alpha=0.8)
    ax_kl.set_xlabel('time (s)', color='#8c8c8c')
    ax_kl.set_ylabel('metric', color='#8c8c8c')
    ax_kl.set_title('KL / MMD / Comm Health', color='#d9d9d9')
    ax_kl.legend(facecolor='#141414', edgecolor='#434343', labelcolor='#d9d9d9', fontsize=8)
    ax_kl.tick_params(colors='#8c8c8c')
    ax_kl.grid(True, alpha=0.2)

    ax_bar = fig.add_subplot(gs[1, 2], facecolor='#141414')
    joints = [f'J{i}' for i in range(1, 8)]
    sim = np.array([0.12, 0.18, 0.25, 0.20, 0.14, 0.16, 0.13])
    real = sim + np.array([0.04, 0.06, 0.11, 0.08, 0.05, 0.06, 0.05])
    x = np.arange(len(joints))
    w = 0.35
    ax_bar.bar(x - w / 2, sim, w, label='Sim', color='#69b1ff')
    ax_bar.bar(x + w / 2, real, w, label='Real', color='#95de64')
    ax_bar.set_xticks(x)
    ax_bar.set_xticklabels(joints, color='#8c8c8c')
    ax_bar.set_title('Sim / Real Window Stats', color='#d9d9d9')
    ax_bar.legend(facecolor='#141414', edgecolor='#434343', labelcolor='#d9d9d9', fontsize=8)
    ax_bar.tick_params(colors='#8c8c8c')
    ax_bar.grid(True, axis='y', alpha=0.2)

    fig.suptitle('M5: HOC Dashboard Preview', color='#e8e8e8', fontsize=14, fontweight='bold', y=0.98)
    out = ASSETS / 'm5-hoc-dashboard.png'
    fig.savefig(out, dpi=120, facecolor=fig.get_facecolor(), bbox_inches='tight')
    plt.close(fig)
    print(f'wrote {out}')


def generate_portfolio_overview_png() -> None:
    """Single-page portfolio stack diagram for README / slides."""
    fig, ax = plt.subplots(figsize=(10, 5.5), facecolor='#f8f9fb')
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 5.5)
    ax.axis('off')
    ax.set_title('Portfolio Stack — iiwa7 Sim2Real Monitor', fontsize=14, fontweight='bold', pad=12)

    layers = [
        ('HOC Console', 'React + WebSocket', '#16a085', 4.6),
        ('Risk Engine', 'R0–R3 · 5D · Fail-Safe', '#c0392b', 3.7),
        ('dist_monitor', 'KL · W1 · MMD', '#8e44ad', 2.8),
        ('PyBullet Bridge', 'Dual-Source · iiwa7', '#27ae60', 1.9),
        ('MoveIt2 / Actions', 'Plan · Pick/Place', '#2980b9', 1.0),
    ]
    for title, subtitle, color, y in layers:
        rect = patches.FancyBboxPatch(
            (1.2, y - 0.32), 7.6, 0.64, boxstyle='round,pad=0.02,rounding_size=0.08',
            facecolor=color, edgecolor='white', alpha=0.92,
        )
        ax.add_patch(rect)
        ax.text(5.0, y + 0.06, title, ha='center', va='center', color='white', fontsize=12, fontweight='bold')
        ax.text(5.0, y - 0.12, subtitle, ha='center', va='center', color='white', fontsize=9, alpha=0.9)
        if y > 1.0:
            ax.annotate('', xy=(5.0, y - 0.38), xytext=(5.0, y - 0.55),
                        arrowprops=dict(arrowstyle='->', color='#95a5a6', lw=1.5))

    ax.text(5.0, 0.35, 'ros2 launch pybullet_bridge portfolio_demo.launch.py sim_mode:=GUI',
            ha='center', fontsize=10, color='#555')
    out = ASSETS / 'portfolio-overview.png'
    fig.savefig(out, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'wrote {out}')


def main() -> None:
    generate_m1_gif()
    generate_m1_pybullet_png()
    generate_m2_svg()
    generate_m2_iiwa_svg()
    generate_iiwa7_pybullet_png()
    generate_iiwa7_motion_gif()
    generate_m3_gif()
    generate_m4_png()
    generate_m5_svg()
    generate_m5_dashboard_svg()
    generate_m5_dashboard_png()
    generate_portfolio_overview_png()
    print(f'done — assets in {ASSETS}')


if __name__ == '__main__':
    main()
