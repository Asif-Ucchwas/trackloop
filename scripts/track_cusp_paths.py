"""Run all four controllers on the nine Reeds-Shepp paths with the cusp runner.

Writes data/results/cusp_tracking.csv and docs/figures/cusp_tracking.png.
Exploratory: one run per pair, start exactly on the path, kinematic model, no noise.
"""
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from trackloop.controllers.kinematic_lqr import KinematicLQR
from trackloop.controllers.kinematic_mpc import KinematicMPC
from trackloop.controllers.pure_pursuit import PurePursuit
from trackloop.controllers.stanley import Stanley, StanleyParams
from trackloop.cusp_runner import track_with_cusps
from trackloop.planner_export import load_path

DATA = Path("data/paths")
CSV = Path("data/results/cusp_tracking.csv")
FIG = Path("docs/figures/cusp_tracking.png")
NAMES = ["ahead", "lane_shift", "diag", "u_turn", "behind", "three_point",
         "parallel_park", "side_slot", "reverse_in"]
SHOWN = ["three_point", "parallel_park", "side_slot", "reverse_in"]
MAKERS = {
    "Pure Pursuit": PurePursuit,
    "Stanley": lambda p: Stanley(p, StanleyParams(feedforward=bool(p.direction[0] < 0))),
    "Kinematic LQR": KinematicLQR,
    "Kinematic MPC": KinematicMPC,
}
COLORS = {"Pure Pursuit": "tab:blue", "Stanley": "tab:green",
          "Kinematic LQR": "tab:red", "Kinematic MPC": "tab:purple"}


def main():
    rows, runs = [], {}
    head = ["path", "controller", "finished", "time_s", "rms_cte_m", "max_cte_m",
            "end_pos_err_m", "worst_cusp_pos_err_m", "worst_settle_s"]
    print("%-14s %-14s %4s %6s %8s %8s %8s %9s %7s" % tuple(head[:2] + ["ok", "time", "rms", "max", "end", "cusp", "settle"]))
    for name in NAMES:
        path = load_path(DATA / f"rs_{name}")
        start = [path.x[0], path.y[0], path.heading[0], 0.0, 0.0]
        for cname, make in MAKERS.items():
            r = track_with_cusps(path, make, start)
            runs[(name, cname)] = (path, r)
            moving = np.abs(r.states[:, 3]) > 0.05
            c = np.abs(r.cte)[moving]
            cusp = max([k["position_error"] for k in r.cusps], default=0.0)
            settle = max([k["settle_time"] for k in r.cusps], default=0.0)
            row = [name, cname, r.finished, round(r.time[-1], 2),
                   round(math.sqrt((c ** 2).mean()), 4), round(c.max(), 4),
                   round(r.end_position_error, 4), round(cusp, 4), round(settle, 2)]
            rows.append(row)
            print("%-14s %-14s %4s %6.1f %8.3f %8.3f %8.3f %9.3f %7.2f" % (
                name, cname, "yes" if r.finished else "NO", row[3], row[4], row[5], row[6], row[7], row[8]))
    CSV.parent.mkdir(parents=True, exist_ok=True)
    with open(CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(head)
        w.writerows(rows)

    fig, axes = plt.subplots(2, 2, figsize=(11, 8.5))
    for ax, name in zip(axes.flat, SHOWN):
        path = runs[(name, "Pure Pursuit")][0]
        ax.plot(path.x, path.y, "k--", lw=1.6, label="reference", zorder=1)
        for cname in MAKERS:
            r = runs[(name, cname)][1]
            ax.plot(r.states[:, 0], r.states[:, 1], color=COLORS[cname], lw=1.3,
                    label=cname, alpha=0.9, zorder=2)
        cusp = np.flatnonzero(path.direction[1:] != path.direction[:-1]) + 1
        ax.plot(path.x[cusp], path.y[cusp], "x", color="k", ms=10, mew=2, zorder=3)
        ax.set_aspect("equal", adjustable="datalim")
        ax.margins(0.12)
        ax.grid(alpha=0.25)
        worst = max(np.abs(runs[(name, c)][1].cte)[np.abs(runs[(name, c)][1].states[:, 3]) > 0.05].max()
                    for c in MAKERS)
        ax.set_title(f"{name}  ({len(cusp)} cusp{'s' * (len(cusp) != 1)}, worst error {worst:.2f} m)",
                     fontsize=10)
    axes[0, 0].legend(fontsize=8, loc="best")
    fig.suptitle("Executed paths with cusp stops, 1.5 m/s, kinematic model, one run each "
                 "(dashed reference, black x cusp)", fontsize=10)
    fig.tight_layout()
    FIG.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG, dpi=130)
    print(f"saved {CSV} and {FIG}")


if __name__ == "__main__":
    main()
