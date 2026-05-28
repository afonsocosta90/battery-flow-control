#!/usr/bin/env python3
"""
T3 — MPC Horizon Sensitivity Analysis
======================================
Sweeps the MPC prediction horizon N over [5, 10, 20, 40, 60, 80] steps on all
three benchmark scenarios and reports:
  • Peak T_max (°C)
  • Constraint violations (count)
  • ∫ṁ² dt (pump control effort)
  • Wall-clock time per MPC step (ms)

Requires the btm binary to be built:
  cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j

Usage (from repo root):
  python3 scripts/horizon_analysis.py

Results are written to results/horizon_analysis.json and a plot to
docs/images/horizon_analysis.png (if matplotlib is available).

Methodology (documented in DESIGN.md §3.5):
  Thermal time constant: τ = C_th / (h·A)  [per cell]
    C_th  = m_cell · cp_cell = 0.070 kg × 900 J/(kg·K) = 63 J/K
    h·A   = 250 W/(m²·K) × 0.00475 m² = 1.1875 W/K
    τ     = 63 / 1.1875 ≈ 53 s
  Recommended horizon: ≥ 1τ ÷ dt = 53 s / 0.1 s = 530 steps for full settling.
  Practical compromise: N = 20 (2 s preview) provides sufficient advance
  notice of the 1C→5C load step (Δheat arrives in ~1 dt; coolant transport
  delay is ~0.5–2 s at nominal flow) with manageable O(N) solver cost.
  The sensitivity study below quantifies the cost of shorter / longer horizons.
"""

import json
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO_ROOT = Path(__file__).parent.parent
BTM_BIN   = REPO_ROOT / "build" / "btm"
RESULTS   = REPO_ROOT / "results"
SCENARIOS = [
    ("constant_c_rate",  REPO_ROOT / "config" / "default.yaml"),
    ("step_transient",   REPO_ROOT / "config" / "scenario_step_transient.yaml"),
    ("rising_ambient",   REPO_ROOT / "config" / "scenario_rising_ambient.yaml"),
]
HORIZONS = [5, 10, 20, 40, 60, 80]


def check_binary() -> None:
    if not BTM_BIN.exists():
        print(f"ERROR: btm binary not found at {BTM_BIN}")
        print("Build with: cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j")
        sys.exit(1)


def run_mpc(config_yaml: Path, horizon: int) -> dict:
    """Run MPC with the given horizon; return metrics dict."""
    # Write a temporary YAML with overridden horizon
    with open(config_yaml) as f:
        yaml_text = f.read()

    # Override horizon_steps (simple text substitution — safe for these YAMLs)
    yaml_text = re.sub(r"horizon_steps:\s*\d+", f"horizon_steps: {horizon}", yaml_text)

    # Override log path to a temp file to avoid polluting results/
    log_path = RESULTS / f"horizon_{horizon}.csv"
    yaml_text = re.sub(r"log_path:.*", f'log_path: "{log_path}"', yaml_text)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as tmp:
        tmp.write(yaml_text)
        tmp_path = tmp.name

    try:
        t0 = time.perf_counter()
        result = subprocess.run(
            [str(BTM_BIN), tmp_path, "mpc"],
            capture_output=True, text=True, cwd=REPO_ROOT, timeout=300
        )
        wall_s = time.perf_counter() - t0
    finally:
        os.unlink(tmp_path)

    if result.returncode != 0:
        print(f"WARNING: btm exited {result.returncode} for horizon={horizon}")
        print(result.stderr[:500])
        return {}

    # Parse JSON summary
    summary_path = str(log_path).replace(".csv", "_summary.json")
    if Path(summary_path).exists():
        with open(summary_path) as f:
            metrics = json.load(f)
        metrics["wall_s"] = wall_s
        metrics["steps_total"] = metrics.get("steps", 0)
        if metrics["steps_total"] > 0:
            metrics["wall_ms_per_step"] = (wall_s / metrics["steps_total"]) * 1000
        return metrics

    # Fallback: parse stdout
    m: dict = {"wall_s": wall_s}
    for line in result.stdout.splitlines():
        if "Peak T_cell" in line:
            m["peak_T_max_c"] = float(line.split(":")[-1].strip().replace("°C","").strip())
        elif "∫ṁ² dt" in line or "pump_integral" in line:
            val = re.search(r"[\d.]+", line.split(":")[-1])
            if val:
                m["pump_integral"] = float(val.group())
        elif "Violations" in line:
            val = re.search(r"(\d+)\s*steps", line)
            if val:
                m["violation_count"] = int(val.group(1))
    return m


def print_table(scenario_name: str, rows: list[dict]) -> None:
    print(f"\n{'─'*72}")
    print(f"  Scenario: {scenario_name}")
    print(f"{'─'*72}")
    print(f"  {'N':>4}  {'T_max_peak':>10}  {'violations':>10}  {'∫ṁ² dt':>10}  {'ms/step':>8}")
    for r in rows:
        n = r.get("horizon")
        t = r.get("peak_T_max_c", float("nan"))
        v = r.get("violation_count", -1)
        p = r.get("pump_integral", float("nan"))
        ms = r.get("wall_ms_per_step", float("nan"))
        print(f"  {n:>4}  {t:>10.2f}  {v:>10}  {p:>10.3f}  {ms:>8.2f}")
    print()


def plot_results(all_data: dict) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("matplotlib not available; skipping plot")
        return

    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    fig.suptitle("MPC Horizon Sensitivity — all three scenarios", fontsize=12, fontweight="bold")

    colors = {"constant_c_rate": "#2196F3", "step_transient": "#E91E63", "rising_ambient": "#4CAF50"}
    labels = {"constant_c_rate": "Constant 5C", "step_transient": "Step Transient",
              "rising_ambient": "Rising Ambient"}

    for sc_name, rows in all_data.items():
        Ns = [r["horizon"] for r in rows if "peak_T_max_c" in r]
        T  = [r["peak_T_max_c"] for r in rows if "peak_T_max_c" in r]
        P  = [r.get("pump_integral", float("nan")) for r in rows if "peak_T_max_c" in r]
        MS = [r.get("wall_ms_per_step", float("nan")) for r in rows if "peak_T_max_c" in r]

        c = colors.get(sc_name, "gray")
        lbl = labels.get(sc_name, sc_name)

        axes[0].plot(Ns, T, "o-", color=c, label=lbl)
        axes[1].plot(Ns, P, "s-", color=c, label=lbl)
        axes[2].plot(Ns, MS, "^-", color=c, label=lbl)

    axes[0].axhline(35.0, color="red", linestyle="--", alpha=0.6, label="T_max constraint")
    axes[0].set_xlabel("Horizon N (steps)"); axes[0].set_ylabel("Peak T_max (°C)")
    axes[0].set_title("Temperature"); axes[0].legend(fontsize=8); axes[0].grid(True, alpha=0.3)

    axes[1].set_xlabel("Horizon N (steps)"); axes[1].set_ylabel("∫ṁ² dt [(kg/s)²·s]")
    axes[1].set_title("Pump Control Effort"); axes[1].legend(fontsize=8); axes[1].grid(True, alpha=0.3)

    axes[2].set_xlabel("Horizon N (steps)"); axes[2].set_ylabel("Wall-clock per step (ms)")
    axes[2].set_title("Solver Cost"); axes[2].legend(fontsize=8); axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    out = REPO_ROOT / "docs" / "images" / "horizon_analysis.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Plot saved to {out}")


def main() -> None:
    check_binary()
    RESULTS.mkdir(parents=True, exist_ok=True)

    all_data: dict[str, list[dict]] = {}

    for sc_name, config_path in SCENARIOS:
        print(f"\nRunning scenario: {sc_name}")
        rows: list[dict] = []
        for N in HORIZONS:
            print(f"  horizon = {N:3d} ... ", end="", flush=True)
            m = run_mpc(config_path, N)
            m["horizon"] = N
            rows.append(m)
            print(f"T_peak={m.get('peak_T_max_c', '?'):.2f}°C  "
                  f"violations={m.get('violation_count','?')}  "
                  f"{m.get('wall_ms_per_step', float('nan')):.1f}ms/step")
        all_data[sc_name] = rows
        print_table(sc_name, rows)

    # Save JSON
    out_json = RESULTS / "horizon_analysis.json"
    with open(out_json, "w") as f:
        json.dump(all_data, f, indent=2)
    print(f"Full results written to {out_json}")

    plot_results(all_data)


if __name__ == "__main__":
    main()
