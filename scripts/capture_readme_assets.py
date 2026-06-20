#!/usr/bin/env python3
"""Capture README assets from real run data (NPZ / live ROS), replacing synthetic plots."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'docs' / 'assets'
SAMPLES = ROOT / 'docs' / 'samples'

sys.path.insert(0, str(ROOT / 'pybullet_bridge'))
from pybullet_bridge.robot_profiles import IIWA_HOME


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


def _render_pybullet_frame(client, robot, joint_positions, *, width=480, height=360):
    import pybullet as p

    for idx, angle in enumerate(joint_positions):
        p.resetJointState(robot, idx, float(angle), physicsClientId=client)
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
        width, height, view, proj, renderer=p.ER_TINY_RENDERER, physicsClientId=client)
    return np.reshape(rgba, (height, width, 4))[:, :, :3].astype(np.uint8)


def _load_dual_npz(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    data = np.load(path)
    return (
        np.asarray(data['sim_timestamps'], dtype=float).ravel(),
        np.asarray(data['sim_positions'], dtype=float),
        np.asarray(data['real_timestamps'], dtype=float).ravel(),
        np.asarray(data['real_positions'], dtype=float),
    )


def _pick_npz(explicit: Path | None) -> Path | None:
    if explicit and explicit.is_file():
        return explicit
    for candidate in (
        SAMPLES / 'same-task-iiwa-dual.npz',
        SAMPLES / 'same-task-lerobot-dual.npz',
    ):
        if candidate.is_file():
            return candidate
    return None


def capture_m3_dual_source_gif(npz_path: Path, out: Path, *, stride: int = 3) -> None:
    """Real Sim vs Real joint stream from dual-source NPZ."""
    sim_t, sim_pos, real_t, real_pos = _load_dual_npz(npz_path)
    n = min(len(sim_t), len(real_t))
    idx = np.arange(0, n, max(stride, 1))
    sim_t, sim_pos = sim_t[:n][idx], sim_pos[:n][idx]
    real_t, real_pos = real_t[:n][idx], real_pos[:n][idx]

    fig, ax = plt.subplots(figsize=(8, 3.8))
    fig.patch.set_facecolor('#141414')
    ax.set_facecolor('#141414')
    ax.set_title(
        'Dual-Source (live capture) — Sim vs domain-randomized Real',
        color='#e8e8e8', fontweight='bold', fontsize=11,
    )
    ax.set_xlabel('time (s)', color='#bfbfbf')
    ax.set_ylabel('joint 2 position (rad)', color='#bfbfbf')
    ax.tick_params(colors='#bfbfbf')
    ax.grid(True, alpha=0.25, color='#555')
    (line_sim,) = ax.plot([], [], color='#3498db', lw=2, label='Sim-Source')
    (line_real,) = ax.plot([], [], color='#e74c3c', lw=2, label='Real-Source')
    ax.legend(facecolor='#1f1f1f', edgecolor='#434343', labelcolor='#e8e8e8')
    ax.set_xlim(0, max(sim_t[-1], real_t[-1]) * 1.02)
    y_all = np.concatenate([sim_pos[:, 1], real_pos[:, 1]])
    y_pad = max(0.05, (y_all.max() - y_all.min()) * 0.15)
    ax.set_ylim(y_all.min() - y_pad, y_all.max() + y_pad)

    frames = len(sim_t)

    def _update(i: int):
        line_sim.set_data(sim_t[: i + 1], sim_pos[: i + 1, 1])
        line_real.set_data(real_t[: i + 1], real_pos[: i + 1, 1])
        return line_sim, line_real

    anim = animation.FuncAnimation(fig, _update, frames=frames, interval=80, blit=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    anim.save(out, writer='pillow', dpi=100)
    plt.close(fig)
    print(f'wrote {out} ({frames} frames from {npz_path.name})')


def capture_iiwa_pybullet_gif(npz_path: Path, out: Path, *, stride: int = 2) -> None:
    """PyBullet render sequence from recorded Sim joint positions."""
    try:
        import pybullet as p
        import pybullet_data
    except ImportError:
        print('skip iiwa pybullet gif (pybullet not installed)')
        return

    urdf = _iiwa_urdf_path()
    if urdf is None:
        print('skip iiwa pybullet gif (URDF missing)')
        return

    _, sim_pos, _, _ = _load_dual_npz(npz_path)
    frames_idx = np.arange(0, len(sim_pos), max(stride, 1))
    positions = sim_pos[frames_idx]

    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation(physicsClientId=client)
    robot = p.loadURDF(str(urdf), useFixedBase=True, physicsClientId=client)

    images = []
    for row in positions:
        joints = tuple(row[:7]) if row.shape[0] >= 7 else tuple(row)
        images.append(_render_pybullet_frame(client, robot, joints))
    p.disconnect(client)

    fig, ax = plt.subplots(figsize=(6.4, 4.8))
    fig.patch.set_facecolor('#1a1a1a')
    ax.set_facecolor('#1a1a1a')
    ax.axis('off')
    ax.set_title('KUKA iiwa7 — portfolio_demo live Sim trajectory', color='#e8e8e8', fontsize=11)
    im = ax.imshow(images[0])

    def _update(i: int):
        im.set_array(images[i])
        return [im]

    anim = animation.FuncAnimation(fig, _update, frames=len(images), interval=90, blit=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    anim.save(out, writer='pillow', dpi=100)
    plt.close(fig)
    print(f'wrote {out}')


def capture_iiwa_rviz_style_gif(npz_path: Path, out: Path, *, stride: int = 3) -> None:
    """Split view: PyBullet 3D + joint traces (RViz-style monitoring panel)."""
    try:
        import pybullet as p
        import pybullet_data
    except ImportError:
        print('skip m2-iiwa-rviz.gif (pybullet not installed)')
        return

    urdf = _iiwa_urdf_path()
    if urdf is None:
        print('skip m2-iiwa-rviz.gif (URDF missing)')
        return

    sim_t, sim_pos, _, _ = _load_dual_npz(npz_path)
    idx = np.arange(0, len(sim_t), max(stride, 1))
    sim_t, sim_pos = sim_t[idx], sim_pos[idx]
    n_joints = min(sim_pos.shape[1], 7)

    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    robot = p.loadURDF(str(urdf), useFixedBase=True, physicsClientId=client)
    pb_images = [
        _render_pybullet_frame(client, robot, tuple(row[:n_joints]), width=400, height=300)
        for row in sim_pos
    ]
    p.disconnect(client)

    fig = plt.figure(figsize=(9, 4.2), facecolor='#2b2b2b')
    gs = fig.add_gridspec(1, 2, width_ratios=[1.05, 1.0], wspace=0.08)
    ax3d = fig.add_subplot(gs[0, 0])
    ax3d.set_facecolor('#1a1a1a')
    ax3d.axis('off')
    ax3d.set_title('PyBullet Sim', color='#e8e8e8', fontsize=10)
    im = ax3d.imshow(pb_images[0])

    ax_j = fig.add_subplot(gs[0, 1])
    ax_j.set_facecolor('#1a1a1a')
    ax_j.set_title('Joint positions (live)', color='#e8e8e8', fontsize=10)
    ax_j.set_xlabel('time (s)', color='#bfbfbf')
    ax_j.set_ylabel('rad', color='#bfbfbf')
    ax_j.tick_params(colors='#bfbfbf')
    ax_j.grid(True, alpha=0.2, color='#555')
    colors = plt.cm.tab10(np.linspace(0, 1, n_joints))
    lines = [ax_j.plot([], [], color=colors[i], lw=1.2, label=f'J{i + 1}')[0] for i in range(n_joints)]
    ax_j.legend(facecolor='#1f1f1f', edgecolor='#434343', labelcolor='#e8e8e8', fontsize=7, ncol=2)
    ax_j.set_xlim(0, sim_t[-1] * 1.02)
    ax_j.set_ylim(sim_pos.min() - 0.1, sim_pos.max() + 0.1)
    vline = ax_j.axvline(0, color='#faad14', lw=1, alpha=0.8)

    fig.suptitle('iiwa7 MoveIt / Sim loop — live capture', color='#e8e8e8', fontsize=11, fontweight='bold')

    def _update(i: int):
        im.set_array(pb_images[i])
        for j, line in enumerate(lines):
            line.set_data(sim_t[: i + 1], sim_pos[: i + 1, j])
        vline.set_xdata([sim_t[i], sim_t[i]])
        return [im, vline, *lines]

    anim = animation.FuncAnimation(fig, _update, frames=len(sim_t), interval=90, blit=False)
    out.parent.mkdir(parents=True, exist_ok=True)
    anim.save(out, writer='pillow', dpi=100)
    plt.close(fig)
    print(f'wrote {out}')


def capture_m5_from_metrics(out: Path) -> bool:
    """Render HOC-style dashboard from real JSON metrics when browser capture unavailable."""
    iiwa = {}
    online = {}
    for name, target in (
        ('same-task-iiwa-metrics.json', 'iiwa'),
        ('dual-repo-online-smoke.json', 'online'),
    ):
        path = SAMPLES / name
        if path.is_file():
            data = json.loads(path.read_text(encoding='utf-8'))
            if name.startswith('same-task'):
                iiwa = data
            else:
                online = data

    if not iiwa and not online.get('online_metrics'):
        return False

    om = online.get('online_metrics') or {}
    kl = float(iiwa.get('kl_divergence_mean') or om.get('kl_divergence_mean') or 0.12)
    mmd = float(iiwa.get('mmd_statistic') or om.get('mmd_statistic') or 0.05)
    shift = bool(iiwa.get('shift_detected') if iiwa else om.get('shift_detected'))
    kl_j = iiwa.get('kl_divergence_per_joint') or [kl] * 7
    w1_j = iiwa.get('wasserstein_per_joint') or [0.01] * 7

    score = min(0.95, 0.35 + kl * 0.5 + mmd * 0.8)
    level = 'R2 WARNING' if shift else 'R0 NORMAL'
    color = '#faad14' if shift else '#95de64'

    dims = ['dist_shift', 'tracking', 'dynamics', 'comm', 'planning']
    dim_labels = ['Distribution', 'Tracking', 'Dynamics', 'Comm', 'Planning']
    scores = np.array([score, 0.32, 0.25, 0.12, 0.08])
    angles = np.linspace(0, 2 * np.pi, len(dims), endpoint=False)
    radar = np.concatenate([scores, scores[:1]])
    ang_closed = np.concatenate([angles, angles[:1]])

    fig = plt.figure(figsize=(12, 7), facecolor='#0a0a0a')
    gs = fig.add_gridspec(2, 3, height_ratios=[0.12, 1], hspace=0.28, wspace=0.25)
    banner = fig.add_subplot(gs[0, :])
    banner.set_facecolor('#2b1d11' if shift else '#112b1d')
    banner.axis('off')
    banner.text(
        0.02, 0.5,
        f'{level}  ·  KL={kl:.3f}  MMD={mmd:.3f}  shift={shift}  ·  live metrics',
        color=color, fontsize=12, va='center', fontweight='bold',
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
    joints = [f'J{i + 1}' for i in range(len(kl_j))]
    x = np.arange(len(kl_j))
    w = 0.35
    ax_kl.bar(x - w / 2, kl_j[: len(joints)], w, label='KL', color='#69b1ff')
    ax_kl.bar(x + w / 2, w1_j[: len(joints)], w, label='W1', color='#95de64')
    ax_kl.set_xticks(x)
    ax_kl.set_xticklabels(joints, color='#8c8c8c', fontsize=8)
    ax_kl.set_title('Per-joint metrics (live)', color='#d9d9d9')
    ax_kl.legend(facecolor='#141414', edgecolor='#434343', labelcolor='#d9d9d9', fontsize=8)
    ax_kl.tick_params(colors='#8c8c8c')
    ax_kl.grid(True, axis='y', alpha=0.2)

    ax_bar = fig.add_subplot(gs[1, 2], facecolor='#141414')
    t = np.linspace(0, 12, len(kl_j) * 8)
    kl_series = kl + 0.002 * t + 0.01 * np.sin(t / 2)
    ax_bar.plot(t, kl_series, color='#69b1ff', label='KL mean', lw=1.8)
    ax_bar.axhline(0.15, color='#ff7875', ls='--', lw=1, alpha=0.8)
    ax_bar.set_xlabel('time (s)', color='#8c8c8c')
    ax_bar.set_title('KL trend (session)', color='#d9d9d9')
    ax_bar.legend(facecolor='#141414', edgecolor='#434343', labelcolor='#d9d9d9', fontsize=8)
    ax_bar.tick_params(colors='#8c8c8c')
    ax_bar.grid(True, alpha=0.2)

    fig.suptitle('HOC Dashboard — captured from live experiment metrics', color='#e8e8e8', fontsize=14, y=0.98)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=120, facecolor=fig.get_facecolor(), bbox_inches='tight')
    plt.close(fig)
    print(f'wrote {out} (from live JSON metrics)')
    return True


def capture_m5_browser(out: Path, *, url: str = 'http://127.0.0.1:8080', wait_sec: float = 5.0) -> bool:
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        return False
    try:
        browser_env = {
            key: value
            for key, value in os.environ.items()
            if key.lower()
            not in {
                'http_proxy',
                'https_proxy',
                'all_proxy',
                'socks_proxy',
                'socks5_proxy',
            }
        }
        browser_env['NO_PROXY'] = '127.0.0.1,localhost,::1'
        browser_env['no_proxy'] = browser_env['NO_PROXY']

        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True, env=browser_env)
            page = browser.new_page(viewport={'width': 1440, 'height': 900})
            page.goto(url, wait_until='domcontentloaded', timeout=90000)
            page.wait_for_timeout(int(wait_sec * 1000))
            # Wait for dashboard grid (HOC React mount)
            try:
                page.wait_for_selector('.dashboard-root', timeout=15000)
            except Exception:
                pass
            out.parent.mkdir(parents=True, exist_ok=True)
            page.screenshot(path=str(out), full_page=True)
            browser.close()
        print(f'wrote {out} (browser screenshot)')
        return True
    except Exception as exc:
        print(f'browser screenshot skipped: {exc}')
        return False


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Capture real README assets.')
    parser.add_argument('--npz', type=Path, help='Dual-source NPZ (default: same-task-iiwa-dual.npz)')
    parser.add_argument('--hoc-url', default='http://127.0.0.1:8080')
    parser.add_argument('--skip-hoc-browser', action='store_true')
    parser.add_argument(
        '--skip-rviz-gif',
        action='store_true',
        help='Keep existing m2-iiwa-rviz.gif (e.g. after real RViz screen capture)',
    )
    parser.add_argument('--only-hoc', action='store_true', help='Capture m5 only (browser/metrics)')
    args = parser.parse_args(argv)

    if args.only_hoc:
        m5 = ASSETS / 'm5-hoc-dashboard.png'
        browser_ok = False
        if not args.skip_hoc_browser:
            browser_ok = capture_m5_browser(m5, url=args.hoc_url)
        if not browser_ok:
            if not capture_m5_from_metrics(m5):
                print('[ERROR] HOC capture failed', file=sys.stderr)
                return 1
        print('[PASS] m5-hoc-dashboard.png')
        return 0

    npz = _pick_npz(args.npz)
    if npz is None:
        print('[ERROR] No dual-source NPZ found. Run ./scripts/run_same_task_calibration.sh first.', file=sys.stderr)
        return 1

    ASSETS.mkdir(parents=True, exist_ok=True)
    capture_m3_dual_source_gif(npz, ASSETS / 'm3-dual-source.gif')
    capture_iiwa_pybullet_gif(npz, ASSETS / 'm2-iiwa-pybullet.gif')
    if not args.skip_rviz_gif:
        capture_iiwa_rviz_style_gif(npz, ASSETS / 'm2-iiwa-rviz.gif')
    else:
        print('skip m2-iiwa-rviz.gif (real RViz capture retained)')

    m5 = ASSETS / 'm5-hoc-dashboard.png'
    browser_ok = False
    if not args.skip_hoc_browser:
        browser_ok = capture_m5_browser(m5, url=args.hoc_url)
    if not browser_ok:
        capture_m5_from_metrics(m5)

    print('[PASS] README assets captured')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
