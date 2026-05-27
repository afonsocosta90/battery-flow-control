#!/usr/bin/env python3
"""Simple per-run plotting script."""
import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

if len(sys.argv) < 2:
    print("Usage: python plot_results.py results/run.csv")
    sys.exit(1)

df = pd.read_csv(sys.argv[1])
fig, axs = plt.subplots(3,1, figsize=(10,7), sharex=True)
axs[0].plot(df['t'], df['T_max'], label='T_max')
axs[0].axhline(35, color='r', ls='--')
axs[0].set_ylabel('Max Temp (°C)')
axs[1].plot(df['t'], df['delta_T'], label='ΔT')
axs[1].axhline(5, color='r', ls='--')
axs[1].set_ylabel('ΔT (°C)')
axs[2].plot(df['t'], df['mdot'])
axs[2].set_ylabel('ṁ (kg/s)')
axs[2].set_xlabel('Time (s)')
plt.tight_layout()
out = sys.argv[1].replace('.csv', '.png')
plt.savefig(out, dpi=120)
print(f"Saved {out}")
