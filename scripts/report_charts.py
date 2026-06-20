"""Shared chart generation and metric captions for experiment reports."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from dist_monitor.lerobot_loader import load_lerobot_dataset
from dist_monitor.offline_compare import _load_dual_npz, _load_npz_trajectory

# PNG titles use ASCII labels (matplotlib default font); HTML reports use 图 1–9.
_FIG = {
    '1': 'Fig. 1',
    '2': 'Fig. 2',
    '3': 'Fig. 3',
    '4': 'Fig. 4',
    '5': 'Fig. 5',
    '6': 'Fig. 6',
    '7': 'Fig. 7',
    '8': 'Fig. 8',
    '9': 'Fig. 9',
}


def fig_label(fig_num: str) -> str:
    """Map 图 N or N to matplotlib-safe Fig. N prefix."""
    num = fig_num.replace('图', '').strip().strip('.')
    return _FIG.get(num, fig_num if fig_num else '')


def metrics_summary_line(metrics: dict) -> str:
    """One-line summary matching chart footers and report tables."""
    return (
        f"KL mean={metrics.get('kl_divergence_mean', 0):.4f}  "
        f"W1 mean={metrics.get('wasserstein_mean', 0):.4f}  "
        f"MMD={metrics.get('mmd_statistic', 0):.4f} "
        f"(p={metrics.get('mmd_p_value', 1):.3f})  "
        f"n={metrics.get('sample_count', 0)}  "
        f"shift={metrics.get('shift_detected', False)} "
        f"({metrics.get('detection_method', 'none')})"
    )


def chart_metrics(metrics: dict, out: Path, *, title: str, fig_num: str = '') -> None:
    """Bar chart for per-joint KL/W1 with footer aligned to JSON metrics."""
    kl = metrics.get('kl_divergence_per_joint') or []
    w1 = metrics.get('wasserstein_per_joint') or []
    n = max(len(kl), len(w1), 1)
    labels = [f'J{i + 1}' for i in range(n)]
    x = np.arange(n)
    width = 0.35

    full_title = f'{fig_label(fig_num)} · {title}' if fig_num else title
    fig, ax = plt.subplots(figsize=(9, 4.8))
    if kl:
        ax.bar(x - width / 2, kl[:n], width, label='KL divergence', color='#3498db')
    if w1:
        ax.bar(x + width / 2, w1[:n], width, label='Wasserstein-1', color='#2ecc71')
    ax.set_xticks(x)
    ax.set_xticklabels(labels[:n])
    ax.set_ylabel('metric value')
    ax.set_title(full_title, fontsize=11, fontweight='bold')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3, axis='y')

    kl_mean = float(metrics.get('kl_divergence_mean', 0) or 0)
    w1_mean = float(metrics.get('wasserstein_mean', 0) or 0)
    if kl_mean < 1e-6 and w1_mean < 1e-6:
        mmd = float(metrics.get('mmd_statistic', 0) or 0)
        p_val = float(metrics.get('mmd_p_value', 1) or 1)
        ax.text(
            0.5, 0.55, f'KL/W1 ~ 0 (baseline)\nMMD={mmd:.4f}, p={p_val:.3f}',
            transform=ax.transAxes, ha='center', va='center', fontsize=11,
            bbox=dict(boxstyle='round', facecolor='#fff3cd', edgecolor='#856404', alpha=0.95),
        )

    fig.text(0.5, 0.02, metrics_summary_line(metrics), ha='center', fontsize=9, color='#333')
    fig.tight_layout(rect=[0, 0.06, 1, 1])
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=120, facecolor='white')
    plt.close(fig)


def chart_cross_source_overlay(
    sim_npz: Path,
    lerobot_root: Path,
    out: Path,
    *,
    title: str,
    fig_num: str = '',
    sample_count: int | None = None,
) -> None:
    sim_ts, sim_states = _load_npz_trajectory(sim_npz)
    sim_pos = sim_states[:, : sim_states.shape[1] // 2]
    real = load_lerobot_dataset(lerobot_root)
    t_max = float(sim_ts[-1]) if len(sim_ts) else 0.0
    mask = real.timestamps <= t_max + 0.05
    real_t = real.timestamps[mask]
    real_pos = real.positions[mask]

    n_joints = min(sim_pos.shape[1], real_pos.shape[1], 7)
    fig, axes = plt.subplots(n_joints, 1, figsize=(10, 2.0 * n_joints), sharex=True)
    if n_joints == 1:
        axes = [axes]
    names = real.joint_names or [f'J{i + 1}' for i in range(n_joints)]
    for i, ax in enumerate(axes):
        ax.plot(sim_ts, sim_pos[:, i], color='#e67e22', lw=1.8, label='Bridge Sim' if i == 0 else None)
        ax.plot(real_t, real_pos[:, i], color='#3498db', lw=1.2, alpha=0.85,
                label='LeRobot Real' if i == 0 else None)
        ax.set_ylabel(names[i] if i < len(names) else f'J{i + 1}', fontsize=9)
        ax.grid(True, alpha=0.25)
    axes[0].legend(loc='upper right', fontsize=9)
    axes[-1].set_xlabel('time (s)')

    n_line = f' · n={sample_count}' if sample_count else ''
    prefix = f'{fig_label(fig_num)} · ' if fig_num else ''
    fig.suptitle(
        f'{prefix}{title}\n0–{t_max:.1f}s · sim={len(sim_ts)} pts · real window={len(real_t)} pts{n_line}',
        fontsize=11,
        fontweight='bold',
    )
    fig.tight_layout()
    fig.savefig(out, dpi=120, facecolor='white')
    plt.close(fig)


def chart_dual_overlay(
    npz_path: Path,
    out: Path,
    *,
    title: str,
    fig_num: str = '',
    sample_count: int | None = None,
) -> None:
    sim_ts, sim_states, real_ts, real_states = _load_dual_npz(npz_path)
    sim_pos = sim_states[:, : sim_states.shape[1] // 2]
    real_pos = real_states[:, : real_states.shape[1] // 2]
    n_joints = min(sim_pos.shape[1], real_pos.shape[1], 7)
    t_max = max(float(sim_ts[-1]) if len(sim_ts) else 0, float(real_ts[-1]) if len(real_ts) else 0)

    fig, axes = plt.subplots(n_joints, 1, figsize=(10, 2.0 * n_joints), sharex=True)
    if n_joints == 1:
        axes = [axes]
    for i, ax in enumerate(axes):
        ax.plot(sim_ts, sim_pos[:, i], color='#e67e22', lw=1.8, label='Sim source' if i == 0 else None)
        ax.plot(real_ts, real_pos[:, i], color='#3498db', lw=1.2, alpha=0.85,
                label='Real (domain-randomized)' if i == 0 else None)
        ax.set_ylabel(f'J{i + 1}', fontsize=9)
        ax.grid(True, alpha=0.25)
    axes[0].legend(loc='upper right', fontsize=9)
    axes[-1].set_xlabel('time (s)')

    n_line = f' · aligned n={sample_count}' if sample_count else ''
    prefix = f'{fig_label(fig_num)} · ' if fig_num else ''
    fig.suptitle(
        f'{prefix}{title}\n0–{t_max:.1f}s · sim={len(sim_ts)} · real={len(real_ts)}{n_line}',
        fontsize=11,
        fontweight='bold',
    )
    fig.tight_layout()
    fig.savefig(out, dpi=120, facecolor='white')
    plt.close(fig)


def chart_lerobot_trajectory(lerobot_root: Path, out: Path, *, fig_num: str = '') -> None:
    traj = load_lerobot_dataset(lerobot_root)
    t = traj.timestamps
    pos = traj.positions
    n_joints = min(pos.shape[1], 7)
    fig, axes = plt.subplots(n_joints, 1, figsize=(10, 2.2 * n_joints), sharex=True)
    if n_joints == 1:
        axes = [axes]
    names = traj.joint_names or [f'joint_{i}' for i in range(n_joints)]
    for i, ax in enumerate(axes):
        ax.plot(t, pos[:, i], color='#3498db', lw=1.5)
        ax.set_ylabel(names[i] if i < len(names) else f'J{i + 1}', fontsize=9)
        ax.grid(True, alpha=0.25)
    axes[-1].set_xlabel('time (s)')
    prefix = f'{fig_label(fig_num)} · ' if fig_num else ''
    fig.suptitle(
        f'{prefix}LeRobot Real (episode-data-lab) — {len(t)} samples @ {traj.fps:.0f} Hz',
        fontsize=12,
        fontweight='bold',
    )
    fig.tight_layout()
    fig.savefig(out, dpi=120, facecolor='white')
    plt.close(fig)


def chart_dual_repo_overview(out: Path, *, fig_num: str = '图 1') -> None:
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.set_xlim(0, 11)
    ax.set_ylim(0, 5)
    ax.axis('off')
    ax.set_title(f'{fig_label(fig_num)} · Dual-Repo Integration — episode-data-lab + bridge', fontsize=13)

    boxes = [
        (0.3, 2.8, 2.4, 1.2, '#8e44ad', 'episode-data-lab\nbatch_collect\nLeRobot export'),
        (3.2, 2.8, 2.4, 1.2, '#2980b9', 'LEROBOT_EXPORT\nReal trajectory'),
        (6.1, 2.8, 2.4, 1.2, '#27ae60', 'bridge Sim NPZ\noffline_compare'),
        (8.9, 2.8, 1.8, 1.2, '#e67e22', 'KL/W1/MMD\nshift_detected'),
    ]
    for x, y, w, h, color, text in boxes:
        ax.add_patch(plt.Rectangle((x, y), w, h, facecolor=color, alpha=0.85, edgecolor='white'))
        ax.text(x + w / 2, y + h / 2, text, ha='center', va='center', color='white', fontsize=9)

    for x0, x1 in [(2.7, 3.2), (5.6, 6.1), (8.5, 8.9)]:
        ax.annotate('', xy=(x1, 3.4), xytext=(x0, 3.4),
                    arrowprops=dict(arrowstyle='->', color='#7f8c8d', lw=2))

    ax.text(
        5.5, 1.5,
        'Cross-source: bridge Sim NPZ vs LeRobot Real\nSame-task: dual-source NPZ + baseline_frac',
        ha='center', fontsize=10, color='#2c3e50',
        bbox=dict(boxstyle='round', facecolor='#ecf0f1', alpha=0.9),
    )
    fig.tight_layout()
    fig.savefig(out, dpi=120, facecolor='white')
    plt.close(fig)
