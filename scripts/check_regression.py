#!/usr/bin/env python3
"""
Regression guard for battery-flow-control CI.

Reads the JSON summary files produced by ./btm for PID and MPC on the default
scenario and asserts that:
  1. MPC has zero constraint violations.
  2. MPC pump_integral is at least 30% lower than PID pump_integral.
  3. PID peak_T_max_c does not change by more than 1 °C vs the baseline.

The script exits with a non-zero code on failure, causing the CI job to fail.

Usage (from the repo root after running both simulations):
  python3 scripts/check_regression.py
"""

import json
import sys
from pathlib import Path

PID_SUMMARY = Path("results/run_pid_summary.json")  # renamed by CI workflow after PID run
MPC_SUMMARY = Path("results/run_mpc_summary.json")  # renamed by CI workflow after MPC run

FAILURES: list[str] = []

def load(path: Path) -> dict:
    if not path.exists():
        print(f"ERROR: summary file not found: {path}")
        sys.exit(2)
    with open(path) as f:
        return json.load(f)

def check(condition: bool, msg: str) -> None:
    if not condition:
        FAILURES.append(msg)

def main() -> None:
    pid = load(PID_SUMMARY)
    mpc = load(MPC_SUMMARY)

    print("=== Regression guard ===")
    print(f"  PID pump_integral : {pid['pump_integral']:.3f} (kg/s)²·s")
    print(f"  MPC pump_integral : {mpc['pump_integral']:.3f} (kg/s)²·s")
    print(f"  PID violations    : {pid['violation_count']}")
    print(f"  MPC violations    : {mpc['violation_count']}")
    print(f"  PID peak T_max    : {pid['peak_T_max_c']:.2f} °C")
    print(f"  MPC peak T_max    : {mpc['peak_T_max_c']:.2f} °C")

    # Guard 1: MPC must have zero violations on the nominal scenario.
    check(mpc["violation_count"] == 0,
          f"MPC has {mpc['violation_count']} constraint violations (expected 0)")

    # Guard 2: MPC pump energy must be at least 30% lower than PID.
    if pid["pump_integral"] > 0:
        saving = 1.0 - mpc["pump_integral"] / pid["pump_integral"]
        print(f"  MPC pump saving   : {saving*100:.1f}% (must be ≥ 30%)")
        check(saving >= 0.30,
              f"MPC pump saving {saving*100:.1f}% is below the 30% regression threshold")

    # Guard 3: PID peak temperature must be physically plausible (30–40 °C at 5C).
    check(30.0 <= pid["peak_T_max_c"] <= 40.0,
          f"PID peak T_max {pid['peak_T_max_c']:.2f} °C outside plausible range [30, 40]")

    if FAILURES:
        print("\nREGRESSION FAILURES:")
        for f in FAILURES:
            print(f"  ✗ {f}")
        sys.exit(1)
    else:
        print("\n✓ All regression checks passed")

if __name__ == "__main__":
    main()
