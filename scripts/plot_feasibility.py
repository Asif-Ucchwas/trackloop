"""Figure: curvature and steering lag of every test-set path against the car's limits.

Reads data/results/feasibility.csv, saves docs/figures/feasibility.png.
Left: largest curvature per path with the car's limit. Right: metres driven with the
steering behind the path (largest event, at the benchmark speed).
"""
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SRC = Path("data/results/feasibility.csv")
OUT = Path("docs/figures/feasibility.png")
COLORS = {"rs_": "tab:orange", "thesis_hybrid": "tab:blue", "thesis_theta": "tab:green"}


def color(name):
    for prefix, c in COLORS.items():
        if name.startswith(prefix):
            return c
    return "0.5"


def main():
    rows = list(csv.DictReader(open(SRC)))[::-1]
    names = [r["id"] for r in rows]
    limit = float(rows[0]["curvature_limit"])
    speed = float(rows[0]["speed"])
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 6.5), sharey=True)
    ys = range(len(rows))
    cols = [color(n) for n in names]
    a.barh(ys, [float(r["max_abs_curvature"]) for r in rows], color=cols)
    a.axvline(limit, color="tab:red", ls="--", lw=1.5)
    a.set_title(f"Largest curvature (limit {limit:.3f} per m, red)", fontsize=10)
    a.set_xlabel("1/m")
    b.barh(ys, [float(r["worst_lag_distance"]) for r in rows], color=cols)
    b.set_title(f"Metres driven with the steering behind the path ({speed:g} m/s)", fontsize=10)
    b.set_xlabel("m")
    a.set_yticks(list(ys))
    a.set_yticklabels(names, fontsize=8)
    for ax in (a, b):
        ax.grid(axis="x", alpha=0.25)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in COLORS.values()]
    fig.legend(handles, ["Reeds-Shepp", "thesis, hybrid", "thesis, theta star"], fontsize=8,
               loc="lower center", ncol=3)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=130)
    print(f"saved {OUT}")


if __name__ == "__main__":
    main()
