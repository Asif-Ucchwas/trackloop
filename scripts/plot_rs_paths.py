"""Figure: the nine Reeds-Shepp reference paths with gear and cusps.

Reads data/paths/rs_*.npz/.json, saves docs/figures/rs_paths.png.
Blue is forward, orange is reverse, red crosses are cusps, arrows are start and goal.
"""
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from trackloop.planner_export import load_path

DATA = Path("data/paths")
OUT = Path("docs/figures/rs_paths.png")
ORDER = ["ahead", "lane_shift", "diag", "u_turn", "behind", "three_point",
         "parallel_park", "side_slot", "reverse_in"]


def arrow(ax, pose, color):
    x, y, deg = pose
    h = np.radians(deg)
    ax.annotate("", xy=(x + 2.5 * np.cos(h), y + 2.5 * np.sin(h)), xytext=(x, y),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.6))


def main():
    fig, axes = plt.subplots(3, 3, figsize=(11, 9.5))
    for ax, name in zip(axes.flat, ORDER):
        path = load_path(DATA / f"rs_{name}")
        info = json.load(open(DATA / f"rs_{name}.json"))
        for gear, color in ((1, "tab:blue"), (-1, "tab:orange")):
            m = path.direction == gear
            ax.plot(np.where(m, path.x, np.nan), np.where(m, path.y, np.nan),
                    color=color, lw=2.0)
        for s_c in info["cusp_s_m"]:
            i = int(np.argmin(np.abs(path.s - s_c)))
            ax.plot(path.x[i], path.y[i], "x", color="tab:red", ms=9, mew=2)
        ax.plot(*info["start"][:2], "o", color="0.2", ms=5)
        arrow(ax, info["start"], "0.2")
        arrow(ax, info["goal"], "tab:green")
        ax.set_aspect("equal", adjustable="datalim")
        ax.margins(0.15)
        ax.grid(alpha=0.25)
        n = info["n_cusps"]
        ax.set_title(f"{name}  ({info['length_m']:.1f} m, {n} cusp{'s' * (n != 1)})",
                     fontsize=10)
    fig.suptitle(f"Reeds-Shepp reference paths, radius {info['radius_m']:g} m "
                 "(blue forward, orange reverse, red x cusp, dark start, green goal)",
                 fontsize=10)
    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=130)
    print(f"saved {OUT}")


if __name__ == "__main__":
    main()
