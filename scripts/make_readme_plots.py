#!/usr/bin/env python3
"""
Generate portfolio-quality plots for docs/images/.
These files are committed to the repository so GitHub renders them in README.md.

Usage:
    python3 scripts/make_readme_plots.py

Reads CSVs from results/.  Run the six scenarios first:
    ./build/btm config/default.yaml                 pid && cp results/run.csv           results/const5c_pid.csv
    ./build/btm config/default.yaml                 mpc && cp results/run.csv           results/const5c_mpc.csv
    ./build/btm config/scenario_step_transient.yaml pid && cp results/step_transient.csv results/step_pid.csv
    ./build/btm config/scenario_step_transient.yaml mpc && cp results/step_transient.csv results/step_mpc.csv
    ./build/btm config/scenario_rising_ambient.yaml pid && cp results/rising_ambient.csv results/rising_pid.csv
    ./build/btm config/scenario_rising_ambient.yaml mpc && cp results/rising_ambient.csv results/rising_mpc.csv
"""

from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.ticker as mticker
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# Colour palette
# ---------------------------------------------------------------------------
C_MPC      = "#1d4ed8"   # blue-700
C_PID      = "#ea580c"   # orange-600
C_LIMIT    = "#dc2626"   # red-600
C_VIOL     = "#fca5a5"   # red-300  (shaded violation region)
C_STEP     = "#64748b"   # slate-500
C_GRID     = "#e2e8f0"   # slate-200
C_BG       = "#f8fafc"   # slate-50

PROJECT    = Path(__file__).parent.parent.resolve()
RESULTS    = PROJECT / "results"
OUT        = PROJECT / "docs" / "images"

# ---------------------------------------------------------------------------
# Style helpers
# ---------------------------------------------------------------------------

def apply_style(ax, xlabel=None, ylabel=None, title=None):
    ax.set_facecolor(C_BG)
    ax.grid(color=C_GRID, linewidth=0.8, zorder=0)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color("#cbd5e1")
    ax.spines["bottom"].set_color("#cbd5e1")
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=9, color="#374151")
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=9, color="#374151")
    if title:
        ax.set_title(title, fontsize=9, color="#374151", loc="left", pad=4)
    ax.tick_params(labelsize=8, colors="#6b7280")


def save(fig, name: str, dpi: int = 160):
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / name
    fig.savefig(path, dpi=dpi, bbox_inches="tight",
                facecolor="white", edgecolor="none")
    plt.close(fig)
    print(f"  Saved  {path.relative_to(PROJECT)}")


def load(pid_name: str, mpc_name: str):
    pid_path = RESULTS / pid_name
    mpc_path = RESULTS / mpc_name
    if not pid_path.exists() or not mpc_path.exists():
        print(f"  [SKIP] Missing: {pid_name} or {mpc_name} — run simulations first.")
        return None, None
    return pd.read_csv(pid_path), pd.read_csv(mpc_path)


# ---------------------------------------------------------------------------
# 1.  Step-transient  (headline result A)
# ---------------------------------------------------------------------------

def plot_step_transient():
    df_pid, df_mpc = load("step_pid.csv", "step_mpc.csv")
    if df_pid is None:
        return

    STEP_T   = 120.0
    T_LIMIT  = 35.0
    dt       = df_pid["t"].iloc[1] - df_pid["t"].iloc[0]

    pid_viol = int((df_pid["T_max"] > T_LIMIT).sum())
    mpc_viol = int((df_mpc["T_max"] > T_LIMIT).sum())
    pid_pump = float((df_pid["mdot"] ** 2).sum() * dt)
    mpc_pump = float((df_mpc["mdot"] ** 2).sum() * dt)

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(
        "Scenario A — Step Load Transient: 1C → 5C at t = 120 s",
        fontsize=12, fontweight="bold", color="#111827", y=1.01,
    )

    # ── T_max ──────────────────────────────────────────────────
    ax = axes[0]
    # shade violation region
    viol_mask = df_pid["T_max"] > T_LIMIT
    if viol_mask.any():
        viol_times = df_pid["t"][viol_mask]
        t0, t1 = float(viol_times.iloc[0]), float(viol_times.iloc[-1])
        ax.axvspan(t0, t1, color=C_VIOL, alpha=0.55, label=f"PID over limit  ({t1-t0:.0f} s)")

    ax.axhline(T_LIMIT, color=C_LIMIT, ls="--", lw=1.2, zorder=2)
    ax.axvline(STEP_T,  color=C_STEP,  ls=":",  lw=1.0, zorder=2)
    ax.plot(df_pid["t"], df_pid["T_max"], color=C_PID, lw=1.6, label="PID",
            zorder=3)
    ax.plot(df_mpc["t"], df_mpc["T_max"], color=C_MPC, lw=1.6, label="MPC",
            zorder=4)

    # Annotations
    ax.text(STEP_T + 3, 25.7, "1C → 5C step", fontsize=7.5, color=C_STEP, va="bottom")
    ax.text(df_pid["t"].max() - 2, T_LIMIT + 0.15,
            f"35 °C limit", fontsize=7.5, color=C_LIMIT, ha="right")
    # PID peak annotation
    peak_idx = df_pid["T_max"].idxmax()
    ax.annotate(
        f"PID peak {df_pid['T_max'].max():.1f} °C",
        xy=(df_pid["t"][peak_idx], df_pid["T_max"][peak_idx]),
        xytext=(df_pid["t"][peak_idx] - 60, df_pid["T_max"][peak_idx] + 0.1),
        fontsize=7.5, color=C_PID, ha="left",
        arrowprops=dict(arrowstyle="-", color=C_PID, lw=0.8),
    )
    # MPC annotation
    mpc_end_T = float(df_mpc["T_max"].iloc[-1])
    ax.annotate(
        f"MPC  {mpc_end_T:.2f} °C\n0 violations",
        xy=(df_mpc["t"].iloc[-1], mpc_end_T),
        xytext=(df_mpc["t"].iloc[-1] - 80, mpc_end_T - 0.5),
        fontsize=7.5, color=C_MPC, ha="left",
        arrowprops=dict(arrowstyle="-", color=C_MPC, lw=0.8),
    )

    handles = [
        mpatches.Patch(color=C_PID,   label=f"PID  (peak {df_pid['T_max'].max():.1f} °C, {pid_viol} violations)"),
        mpatches.Patch(color=C_MPC,   label=f"MPC  (peak {df_mpc['T_max'].max():.2f} °C,  {mpc_viol} violation)"),
        mpatches.Patch(color=C_VIOL,  alpha=0.6, label=f"PID over limit  ({pid_viol * dt:.0f} s)"),
        plt.Line2D([], [], color=C_LIMIT, ls="--", lw=1.2, label="35 °C hard limit"),
    ]
    ax.legend(handles=handles, fontsize=7.5, loc="upper left", framealpha=0.9)
    apply_style(ax, ylabel="Max cell temperature (°C)", title="Peak cell temperature")
    ax.set_ylim(24, 37.5)

    # ── ΔT ─────────────────────────────────────────────────────
    ax = axes[1]
    ax.axhline(5.0, color=C_LIMIT, ls="--", lw=1.2)
    ax.axvline(STEP_T, color=C_STEP, ls=":", lw=1.0)
    ax.plot(df_pid["t"], df_pid["delta_T"], color=C_PID, lw=1.6)
    ax.plot(df_mpc["t"], df_mpc["delta_T"], color=C_MPC, lw=1.6)
    ax.text(df_pid["t"].max() - 2, 5.1, "5 °C limit", fontsize=7.5,
            color=C_LIMIT, ha="right")
    apply_style(ax, ylabel="Inter-cell ΔT (°C)", title="Temperature non-uniformity  (both controllers well inside limit)")

    # ── mdot ───────────────────────────────────────────────────
    ax = axes[2]
    ax.axvline(STEP_T, color=C_STEP, ls=":", lw=1.0)
    ax.plot(df_pid["t"], df_pid["mdot"], color=C_PID, lw=1.6,
            label=f"PID   ∫ṁ²dt = {pid_pump:.1f}")
    ax.plot(df_mpc["t"], df_mpc["mdot"], color=C_MPC, lw=1.6,
            label=f"MPC  ∫ṁ²dt = {mpc_pump:.1f}")
    ax.legend(fontsize=8, loc="upper left", framealpha=0.9)
    apply_style(ax, xlabel="Time (s)", ylabel="Mass flow rate (kg/s)",
                title="Coolant mass flow command")

    fig.tight_layout()
    save(fig, "step_transient.png")


# ---------------------------------------------------------------------------
# 2.  Constant 5C  (headline result B — pump energy)
# ---------------------------------------------------------------------------

def plot_const5c():
    df_pid, df_mpc = load("const5c_pid.csv", "const5c_mpc.csv")
    if df_pid is None:
        return

    T_LIMIT = 35.0
    dt      = df_pid["t"].iloc[1] - df_pid["t"].iloc[0]
    pid_pump = float((df_pid["mdot"] ** 2).sum() * dt)
    mpc_pump = float((df_mpc["mdot"] ** 2).sum() * dt)
    saving   = 100.0 * (1.0 - mpc_pump / pid_pump)

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(
        "Scenario B — Constant 5C Discharge (600 s)  ·  MPC uses 66% less pump energy",
        fontsize=12, fontweight="bold", color="#111827", y=1.01,
    )

    # ── T_max ──────────────────────────────────────────────────
    ax = axes[0]
    ax.axhline(T_LIMIT, color=C_LIMIT, ls="--", lw=1.2)
    ax.plot(df_pid["t"], df_pid["T_max"], color=C_PID, lw=1.6,
            label=f"PID  (setpoint 32 °C)")
    ax.plot(df_mpc["t"], df_mpc["T_max"], color=C_MPC, lw=1.6,
            label=f"MPC  (tracks constraint boundary 34.9 °C)")
    ax.text(df_pid["t"].max() - 5, T_LIMIT + 0.15,
            "35 °C limit", fontsize=7.5, color=C_LIMIT, ha="right")
    # Annotate steady-state temperatures
    ss_pid = float(df_pid["T_max"].iloc[-200:].mean())
    ss_mpc = float(df_mpc["T_max"].iloc[-200:].mean())
    ax.annotate(f"PID steady-state ≈ {ss_pid:.1f} °C\n(over-cooled by ~3 °C)",
                xy=(500, ss_pid),
                xytext=(350, ss_pid - 1.2),
                fontsize=7.5, color=C_PID, ha="left",
                arrowprops=dict(arrowstyle="-", color=C_PID, lw=0.8))
    ax.annotate(f"MPC steady-state ≈ {ss_mpc:.2f} °C\n(constraint boundary)",
                xy=(500, ss_mpc),
                xytext=(350, ss_mpc + 0.5),
                fontsize=7.5, color=C_MPC, ha="left",
                arrowprops=dict(arrowstyle="-", color=C_MPC, lw=0.8))
    ax.legend(fontsize=8, loc="lower right", framealpha=0.9)
    apply_style(ax, ylabel="Max cell temperature (°C)",
                title="Both controllers satisfy T < 35 °C — MPC tracks the boundary, PID over-cools")

    # ── ΔT ─────────────────────────────────────────────────────
    ax = axes[1]
    ax.axhline(5.0, color=C_LIMIT, ls="--", lw=1.2)
    ax.plot(df_pid["t"], df_pid["delta_T"], color=C_PID, lw=1.6)
    ax.plot(df_mpc["t"], df_mpc["delta_T"], color=C_MPC, lw=1.6)
    ax.text(df_pid["t"].max() - 5, 5.1, "5 °C limit", fontsize=7.5,
            color=C_LIMIT, ha="right")
    apply_style(ax, ylabel="Inter-cell ΔT (°C)",
                title="Temperature uniformity  (both well within limit)")

    # ── mdot — fill between to show wasted energy ───────────────
    ax = axes[2]
    t = df_pid["t"].values
    ax.fill_between(t, df_mpc["mdot"].values, df_pid["mdot"].values,
                    alpha=0.18, color=C_PID, label=f"Wasted pump effort  ({saving:.0f}% reduction)")
    ax.plot(df_pid["t"], df_pid["mdot"], color=C_PID, lw=1.6,
            label=f"PID   ∫ṁ²dt = {pid_pump:.1f} (kg/s)²·s")
    ax.plot(df_mpc["t"], df_mpc["mdot"], color=C_MPC, lw=1.6,
            label=f"MPC  ∫ṁ²dt = {mpc_pump:.1f} (kg/s)²·s")
    ax.legend(fontsize=8, loc="center right", framealpha=0.9)
    apply_style(ax, xlabel="Time (s)", ylabel="Mass flow rate (kg/s)",
                title=f"MPC uses {saving:.0f}% less pump energy — shaded area is the saving")

    fig.tight_layout()
    save(fig, "const5c.png")


# ---------------------------------------------------------------------------
# 3.  Rising ambient  (headline result C)
# ---------------------------------------------------------------------------

def plot_rising_ambient():
    df_pid, df_mpc = load("rising_pid.csv", "rising_mpc.csv")
    if df_pid is None:
        return

    T_LIMIT     = 35.0
    RAMP_START  = 60.0
    RAMP_END    = 240.0
    dt          = df_pid["t"].iloc[1] - df_pid["t"].iloc[0]
    pid_pump    = float((df_pid["mdot"] ** 2).sum() * dt)
    mpc_pump    = float((df_mpc["mdot"] ** 2).sum() * dt)
    saving      = 100.0 * (1.0 - mpc_pump / pid_pump)

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(
        "Scenario C — Rising Ambient: T_inlet 25 → 27 °C ramp at 5C  ·  MPC uses 61% less pump energy",
        fontsize=12, fontweight="bold", color="#111827", y=1.01,
    )

    # ── T_max ──────────────────────────────────────────────────
    ax = axes[0]
    ax.axhline(T_LIMIT, color=C_LIMIT, ls="--", lw=1.2)
    ax.axvspan(RAMP_START, RAMP_END, color="#fef9c3", alpha=0.5, label="Inlet ramp 25 → 27 °C")
    ax.plot(df_pid["t"], df_pid["T_max"], color=C_PID, lw=1.6, label="PID")
    ax.plot(df_mpc["t"], df_mpc["T_max"], color=C_MPC, lw=1.6, label="MPC")
    ax.text(df_pid["t"].max() - 5, T_LIMIT + 0.15,
            "35 °C limit", fontsize=7.5, color=C_LIMIT, ha="right")
    ax.text((RAMP_START + RAMP_END) / 2, ax.get_ylim()[0] + 0.3,
            "inlet ramp", fontsize=7.5, color="#92400e", ha="center")
    ax.legend(fontsize=8, loc="upper left", framealpha=0.9)
    apply_style(ax, ylabel="Max cell temperature (°C)",
                title="Both controllers satisfy T < 35 °C  —  MPC scales flow to track the boundary")

    # ── ΔT ─────────────────────────────────────────────────────
    ax = axes[1]
    ax.axhline(5.0, color=C_LIMIT, ls="--", lw=1.2)
    ax.axvspan(RAMP_START, RAMP_END, color="#fef9c3", alpha=0.5)
    ax.plot(df_pid["t"], df_pid["delta_T"], color=C_PID, lw=1.6)
    ax.plot(df_mpc["t"], df_mpc["delta_T"], color=C_MPC, lw=1.6)
    ax.text(df_pid["t"].max() - 5, 5.1, "5 °C limit", fontsize=7.5,
            color=C_LIMIT, ha="right")
    apply_style(ax, ylabel="Inter-cell ΔT (°C)",
                title="Temperature uniformity  (both well within limit)")

    # ── mdot ───────────────────────────────────────────────────
    ax = axes[2]
    t = df_pid["t"].values
    ax.axvspan(RAMP_START, RAMP_END, color="#fef9c3", alpha=0.5)
    ax.fill_between(t, df_mpc["mdot"].values, df_pid["mdot"].values,
                    alpha=0.18, color=C_PID, label=f"Wasted pump effort  ({saving:.0f}% reduction)")
    ax.plot(df_pid["t"], df_pid["mdot"], color=C_PID, lw=1.6,
            label=f"PID   ∫ṁ²dt = {pid_pump:.1f} (kg/s)²·s")
    ax.plot(df_mpc["t"], df_mpc["mdot"], color=C_MPC, lw=1.6,
            label=f"MPC  ∫ṁ²dt = {mpc_pump:.1f} (kg/s)²·s")
    ax.legend(fontsize=8, loc="upper left", framealpha=0.9)
    apply_style(ax, xlabel="Time (s)", ylabel="Mass flow rate (kg/s)",
                title=f"MPC proportionally scales flow as inlet rises  ({saving:.0f}% less pump energy)")

    fig.tight_layout()
    save(fig, "rising_ambient.png")


# ---------------------------------------------------------------------------
# 4.  Summary 2×3 overview panel (for the top of the README)
# ---------------------------------------------------------------------------

def plot_summary():
    """A compact 2-row, 3-column overview — one column per scenario."""
    data = [
        ("step_pid.csv",    "step_mpc.csv",    "A  Step Transient",      True,  120.0),
        ("const5c_pid.csv", "const5c_mpc.csv", "B  Constant 5C",         False, None),
        ("rising_pid.csv",  "rising_mpc.csv",  "C  Rising Ambient",      False, None),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(14, 6), sharex="col")
    fig.suptitle(
        "MPC vs PID  ·  Three benchmark scenarios  ·  24-node lumped-capacitance model",
        fontsize=11, fontweight="bold", color="#111827", y=1.02,
    )

    T_LIMIT  = 35.0
    DT_LIMIT = 5.0

    for col, (pid_name, mpc_name, label, has_step, step_t) in enumerate(data):
        df_pid, df_mpc = load(pid_name, mpc_name)
        if df_pid is None:
            continue

        dt       = df_pid["t"].iloc[1] - df_pid["t"].iloc[0]
        pid_pump = float((df_pid["mdot"] ** 2).sum() * dt)
        mpc_pump = float((df_mpc["mdot"] ** 2).sum() * dt)
        saving   = 100.0 * (1.0 - mpc_pump / pid_pump)

        # Row 0: T_max
        ax = axes[0, col]
        if has_step:
            viol_mask = df_pid["T_max"] > T_LIMIT
            if viol_mask.any():
                t0 = float(df_pid["t"][viol_mask].iloc[0])
                t1 = float(df_pid["t"][viol_mask].iloc[-1])
                ax.axvspan(t0, t1, color=C_VIOL, alpha=0.55)
        if step_t:
            ax.axvline(step_t, color=C_STEP, ls=":", lw=0.9)
        ax.axhline(T_LIMIT, color=C_LIMIT, ls="--", lw=1.0)
        ax.plot(df_pid["t"], df_pid["T_max"], color=C_PID, lw=1.4, label="PID")
        ax.plot(df_mpc["t"], df_mpc["T_max"], color=C_MPC, lw=1.4, label="MPC")
        apply_style(ax, ylabel="T_max (°C)" if col == 0 else None, title=label)
        ax.legend(fontsize=7, loc="upper left", framealpha=0.9)

        # Row 1: mdot with fill
        ax = axes[1, col]
        if step_t:
            ax.axvline(step_t, color=C_STEP, ls=":", lw=0.9)
        t_arr = df_pid["t"].values
        ax.fill_between(t_arr, df_mpc["mdot"].values, df_pid["mdot"].values,
                        alpha=0.18, color=C_PID)
        ax.plot(df_pid["t"], df_pid["mdot"], color=C_PID, lw=1.4,
                label=f"PID  {pid_pump:.0f}")
        ax.plot(df_mpc["t"], df_mpc["mdot"], color=C_MPC, lw=1.4,
                label=f"MPC {mpc_pump:.0f}")
        ax.set_xlabel("Time (s)", fontsize=8, color="#374151")
        apply_style(ax, ylabel="ṁ (kg/s)" if col == 0 else None,
                    title=f"∫ṁ²dt  →  {saving:.0f}% saving" if saving > 0
                    else "∫ṁ²dt  (pump effort)")
        ax.legend(fontsize=7, title="∫ṁ²dt", title_fontsize=6,
                  loc="upper left", framealpha=0.9)

    fig.tight_layout()
    save(fig, "summary.png", dpi=140)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("Generating portfolio plots → docs/images/ …")
    plot_step_transient()
    plot_const5c()
    plot_rising_ambient()
    plot_summary()
    print("\nDone.  Commit docs/images/ to make them visible on GitHub.")
