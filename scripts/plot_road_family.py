"""Figure: the road family. Lateral acceleration against time (top) and geometry (bottom).

Reads the frozen files in data/paths/road, saves docs/figures/road_family.png. Top: the
maneuver is the same in time at 5, 10 and 20 m/s (same peak, same ramps); the curve and
s_curve hold longer at low speed because they turn a fixed angle. Bottom: the paths at
10 m/s, the middle design speed (minimum radius 50 m).
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from trackloop import road_paths as rp
from trackloop.testset import ROAD_DIR, load_test_set

OUT = Path("docs/figures/road_family.png")
COLORS = {5.0: "tab:blue", 10.0: "tab:orange", 20.0: "tab:green"}


def main():
    paths = dict(load_test_set(ROAD_DIR))
    fig, axes = plt.subplots(2, len(rp.SHAPES), figsize=(16, 7.5))
    for j, shape in enumerate(rp.SHAPES):
        top, bottom = axes[0, j], axes[1, j]
        for v in rp.SPEEDS:
            p = paths[rp.path_id(shape, v)]
            top.plot(p.s / v, v * v * p.curvature, color=COLORS[v], lw=1.6, label=f"{v:g} m/s")
        top.axhline(rp.A_Y, color="0.6", ls=":", lw=1)
        top.axhline(-rp.A_Y, color="0.6", ls=":", lw=1)
        top.set_title(shape, fontsize=11)
        top.set_xlabel("time at design speed (s)")
        top.grid(alpha=0.25)
        p = paths[rp.path_id(shape, 10.0)]
        bottom.plot(p.x, p.y, color=COLORS[10.0], lw=1.8)
        bottom.set_aspect("equal", adjustable="datalim")
        bottom.set_xlabel("x (m)")
        bottom.grid(alpha=0.25)
        bottom.set_title(f"10 m/s: length {p.s[-1]:.0f} m", fontsize=9)
    axes[0, 0].set_ylabel("lateral acceleration v$^2$ $\\kappa$ (m/s$^2$)")
    axes[0, 0].legend(fontsize=8)
    axes[1, 0].set_ylabel("y (m)")
    fig.suptitle("Road family: 12 paths, peak lateral acceleration 2.0 m/s$^2$ at every speed", fontsize=12)
    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=130)
    print(f"saved {OUT}")


if __name__ == "__main__":
    main()
