#!/usr/bin/env python3
"""
Minimal controller comparison script for Battery Thermal MPC project.

Usage:
    python scripts/compare_controllers.py --pid results/pid.csv --mpc results/mpc.csv --out results/comparison.png
"""
import argparse
import pandas as pd
import matplotlib.pyplot as plt
import os

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pid", default="results/run.csv", help="PID run CSV")
    parser.add_argument("--mpc", default="results/run.csv", help="MPC run CSV")
    parser.add_argument("--out", default="results/comparison.png", help="Output plot path")
    args = parser.parse_args()

    if not os.path.exists(args.pid) or not os.path.exists(args.mpc):
        print("One or both result CSVs not found. Run the C++ binary first.")
        return

    df_pid = pd.read_csv(args.pid)
    df_mpc = pd.read_csv(args.mpc)

    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # Max temperature
    axs[0].plot(df_pid['t'], df_pid['T_max'], label='PID', alpha=0.8)
    axs[0].plot(df_mpc['t'], df_mpc['T_max'], label='MPC', alpha=0.8)
    axs[0].axhline(35, color='r', linestyle='--', label='Constraint')
    axs[0].set_ylabel('Max Cell Temp (°C)')
    axs[0].legend()
    axs[0].grid(True)

    # Delta T
    axs[1].plot(df_pid['t'], df_pid['delta_T'], label='PID')
    axs[1].plot(df_mpc['t'], df_mpc['delta_T'], label='MPC')
    axs[1].axhline(5, color='r', linestyle='--')
    axs[1].set_ylabel('ΔT (°C)')
    axs[1].grid(True)

    # Mass flow
    axs[2].plot(df_pid['t'], df_pid['mdot'], label='PID')
    axs[2].plot(df_mpc['t'], df_mpc['mdot'], label='MPC')
    axs[2].set_ylabel('Mass Flow (kg/s)')
    axs[2].set_xlabel('Time (s)')
    axs[2].grid(True)

    plt.tight_layout()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    plt.savefig(args.out, dpi=150)
    print(f"Saved comparison plot to {args.out}")

    # Simple headline metrics
    pid_violations = ((df_pid['T_max'] > 35) | (df_pid['delta_T'] > 5)).sum()
    mpc_violations = ((df_mpc['T_max'] > 35) | (df_mpc['delta_T'] > 5)).sum()
    pid_pump = (df_pid['mdot']**2).sum() * 0.1   # crude integral proxy
    mpc_pump = (df_mpc['mdot']**2).sum() * 0.1

    print(f"PID violations: {pid_violations}")
    print(f"MPC violations: {mpc_violations}")
    print(f"PID pump proxy: {pid_pump:.2f}")
    print(f"MPC pump proxy: {mpc_pump:.2f}")
    if mpc_violations == 0 and pid_violations > 0:
        print("Headline A satisfied (MPC holds while PID violates).")
    if mpc_pump < 0.9 * pid_pump:
        print("Headline B satisfied (MPC uses ≥10% less pump effort).")

if __name__ == "__main__":
    main()
