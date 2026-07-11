#!/usr/bin/env python3
"""Generate a portfolio plot illustrating downstream safety watchdog response to delay faults."""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

REPO_ROOT = Path("/home/ina/ros2_ws/src/ros2-moveit-pybullet-bridge")
CSV_PATH = Path("/tmp/benchmark_fault_out/system_health_events.csv")
OUTPUT_DIR = REPO_ROOT / "docs/assets"


def set_style():
    """Set clean, modern plotting style."""
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.size": 11,
        "axes.labelsize": 12,
        "axes.titlesize": 14,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "figure.titlesize": 16,
        "grid.color": "#e2e8f0",
        "grid.linewidth": 0.8,
        "axes.edgecolor": "#94a3b8",
        "axes.linewidth": 1.0,
    })


def main():
    if not CSV_PATH.exists():
        print(f"Error: Fault CSV not found at {CSV_PATH}.")
        return 1

    set_style()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(CSV_PATH)
    start_time = df["monotonic_sec"].min()
    df["relative_sec"] = df["monotonic_sec"] - start_time

    fig, ax = plt.subplots(figsize=(8.5, 5))
    ax.grid(True, linestyle="--", alpha=0.7, zorder=0)

    # Define color mapping for diagnostic levels
    # Level 0 = Green (OK), Level 1 = Orange (WARN), Level 2 = Red (ERROR)
    colors = {0: "#22c55e", 1: "#f97316", 2: "#ef4444"}
    labels = {0: "Level 0: OK", 1: "Level 1: WARN (Stalled)", 2: "Level 2: ERROR (E-Stop Triggered)"}

    # Plot background bands representing safety zones
    ax.axhspan(0, 1000, color="#f0fdf4", alpha=0.6, label="Safe Watchdog Zone (< 1.0s)")
    ax.axhspan(1000, 2000, color="#fff7ed", alpha=0.6, label="Warning Watchdog Zone (1.0s - 2.0s)")
    ax.axhspan(2000, 4500, color="#fef2f2", alpha=0.6, label="Critical E-Stop Zone (> 2.0s)")

    # Plot the stall age line
    ax.plot(df["relative_sec"], df["last_action_age_ms"], color="#475569", linestyle="-", linewidth=2.5, zorder=2, label="Stall Duration (ms)")

    # Plot transition scatter points
    # Level 1 WARN transition
    warn_df = df[df["level"] == 1]
    if not warn_df.empty:
        first_warn = warn_df.iloc[0]
        ax.scatter(
            first_warn["relative_sec"], 
            first_warn["last_action_age_ms"], 
            color="#f97316", 
            edgecolors="#1e293b", 
            s=120, 
            zorder=4, 
            label="Watchdog Warning (WARN)"
        )

    # Level 2 ERROR transition
    err_df = df[df["level"] == 2]
    if not err_df.empty:
        first_err = err_df.iloc[0]
        ax.scatter(
            first_err["relative_sec"], 
            first_err["last_action_age_ms"], 
            color="#ef4444", 
            edgecolors="#1e293b", 
            s=120, 
            zorder=4, 
            label="E-Stop Triggered (ERROR)"
        )

    # Draw thresholds
    ax.axhline(y=1000, color="#f97316", linestyle="--", linewidth=1.2, alpha=0.8)
    ax.axhline(y=2000, color="#ef4444", linestyle="--", linewidth=1.2, alpha=0.8)

    # Annotate transitions
    # Find transition point to level 1 and level 2
    transition_1 = df[df["level"] == 1]["relative_sec"].min()
    transition_2 = df[df["level"] == 2]["relative_sec"].min()

    ax.annotate("Heartbeat Timeout\nWARN State", 
                 xy=(0.0, 1766), 
                 xytext=(0.4, 1300),
                 arrowprops=dict(facecolor='#f97316', shrink=0.08, width=1.2, headwidth=5),
                 fontsize=10, color="#c2410c", fontweight="bold")

    ax.annotate("Stall > 2.0s\nE-Stop Triggered", 
                 xy=(transition_2, 2170), 
                 xytext=(transition_2 - 0.6, 2800),
                 arrowprops=dict(facecolor='#ef4444', shrink=0.08, width=1.2, headwidth=5),
                 fontsize=10, color="#b91c1c", fontweight="bold")

    ax.set_title("Downstream Watchdog Safety Response (Fault Injection Run)", pad=15)
    ax.set_xlabel("Time Since Fault Injection (seconds)")
    ax.set_ylabel("Command Stall Duration (ms)")
    ax.set_ylim(0, 4200)
    ax.set_xlim(-0.1, df["relative_sec"].max() + 0.2)
    
    # Clean up duplicate legend entries
    handles, labels_list = ax.get_legend_handles_labels()
    by_label = dict(zip(labels_list, handles))
    ax.legend(by_label.values(), by_label.keys(), frameon=True, facecolor="white", edgecolor="#cbd5e1", loc="upper left")

    plt.tight_layout()
    out_path = OUTPUT_DIR / "panda_fault_injection_safety_response.png"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_path}")
    return 0


if __name__ == "__main__":
    exit(main())
