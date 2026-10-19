"""Figure: thesis planner waypoints and the rounded paths made from them (Task 9).

Reads the files written by export_thesis_paths.py. Saves docs/figures/thesis_paths.png.
"""
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from trackloop.planner_export import load_path

DATA, OUT = Path("data/paths"), Path("docs/figures/thesis_paths.png")
SEEDS, NAMES = (0, 1, 2), ("hybrid", "theta_star_tight")


def main():
    fig, axes = plt.subplots(len(NAMES), len(SEEDS), figsize=(11, 7.5))
    for col, seed in enumerate(SEEDS):
        grid = np.load(DATA / f"map_seed{seed}.npy")
        n = grid.shape[0] - 0.5
        for row, name in enumerate(NAMES):
            stem = DATA / f"thesis_{name}_seed{seed}"
            path = load_path(stem)
            info = json.loads(stem.with_suffix(".json").read_text())
            cell, ax = info["cell_m"], axes[row][col]
            ax.imshow(grid, origin="lower", cmap="Greys", vmin=0, vmax=1.6,
                      extent=(-0.5, n, -0.5, n))
            wp = np.array(info["waypoints_cells"])
            ax.plot(wp[:, 0], wp[:, 1], "o-", color="tab:orange", lw=1.0, ms=2.5,
                    label="planner waypoints")
            ax.plot(path.x / cell, path.y / cell, color="tab:blue", lw=1.6,
                    label="rounded path")
            ax.set_title(f"{name}, map {seed}: {info['length_m']:.0f} m, "
                         f"{info['rounded_hits']} obstacle hits", fontsize=9)
            ax.set_xticks([])
            ax.set_yticks([])
    axes[0][0].legend(loc="upper left", fontsize=7)
    fig.suptitle("Thesis planner paths, 3 m per cell, corners rounded with 6 m arcs")
    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=130)
    print(f"saved {OUT}")


if __name__ == "__main__":
    main()
