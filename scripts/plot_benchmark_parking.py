"""Figure: median RMS lateral error of every path, controller and speed (parking family).

Reads data/results/benchmark_parking.csv, saves docs/figures/benchmark_parking.png.
Each cell is the median over the seeds. Paths marked * break a hard limit (curvature or
lateral acceleration) at some benchmark speed and are run anyway.
"""
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LogNorm

SRC = Path("data/results/benchmark_parking.csv")
OUT = Path("docs/figures/benchmark_parking.png")


def main():
    rows = list(csv.DictReader(open(SRC)))
    paths = list(dict.fromkeys(r["path"] for r in rows))
    ctrls = list(dict.fromkeys(r["controller"] for r in rows))
    speeds = sorted({float(r["speed"]) for r in rows})
    grid = np.full((len(paths), len(speeds) * len(ctrls)), np.nan)
    for i, p in enumerate(paths):
        for j, v in enumerate(speeds):
            for k, c in enumerate(ctrls):
                vals = [float(r["rms_lateral"]) for r in rows if r["path"] == p and r["controller"] == c
                        and float(r["speed"]) == v and r["finished"] == "True"]
                if vals:
                    grid[i, j * len(ctrls) + k] = np.median(vals)
    bad = {r["path"] for r in rows if r["drivable"] == "False"}
    fig, ax = plt.subplots(figsize=(13, 8))
    im = ax.imshow(np.maximum(grid, 1e-3), aspect="auto", cmap="viridis_r", norm=LogNorm(1e-3, 2.0))
    short = {"Pure Pursuit": "PP", "Stanley": "ST", "Kinematic LQR": "LQR", "Kinematic MPC": "MPC"}
    ax.set_xticks(range(grid.shape[1]))
    ax.set_xticklabels([short.get(c, c) for _ in speeds for c in ctrls], fontsize=8)
    ax.set_yticks(range(len(paths)))
    ax.set_yticklabels([p + (" *" if p in bad else "") for p in paths], fontsize=8)
    for j, v in enumerate(speeds):
        ax.text((j + 0.5) * len(ctrls) - 0.5, -1.2, f"{v:g} m/s", ha="center", fontsize=10)
        if j:
            ax.axvline(j * len(ctrls) - 0.5, color="white", lw=2)
    for i in range(grid.shape[0]):
        for jj in range(grid.shape[1]):
            if not np.isnan(grid[i, jj]):
                ax.text(jj, i, f"{grid[i, jj]:.2f}", ha="center", va="center", fontsize=6.5,
                        color="white" if grid[i, jj] > 0.05 else "black")
    fig.colorbar(im, ax=ax, label="median RMS lateral error (m), log scale", shrink=0.8)
    ax.set_title("Parking family: median RMS lateral error over 5 seeds (* = breaks a hard limit)",
                 fontsize=11, pad=36)
    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=130)
    print(f"saved {OUT}")


if __name__ == "__main__":
    main()
