"""Figure: curvature, steering lag and largest drivable speed of every test-set path.

Reads data/results/feasibility.csv, saves docs/figures/feasibility.png.
Left: largest curvature with the car's limit. Middle: metres driven with the steering
behind the path (largest event, at 1.5 m/s). Right: largest speed at which the path needs no
more than mu * g of lateral acceleration, against the benchmark speeds 5, 10 and 20 m/s.
"""
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SRC = Path("data/results/feasibility.csv")
OUT = Path("docs/figures/feasibility.png")
COLORS = {"rs_": "tab:orange", "thesis_hybrid": "tab:blue", "thesis_theta": "tab:green"}
BENCH_SPEEDS = (5.0, 10.0, 20.0)
SPEED_AXIS = 25.0


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
    mu_g = float(rows[0]["lateral_accel_limit"])
    fig, (a, b, c) = plt.subplots(1, 3, figsize=(15, 6.5), sharey=True)
    ys = range(len(rows))
    cols = [color(n) for n in names]
    a.barh(ys, [float(r["max_abs_curvature"]) for r in rows], color=cols)
    a.axvline(limit, color="tab:red", ls="--", lw=1.5)
    a.set_title(f"Largest curvature (limit {limit:.3f} per m, red)", fontsize=10)
    a.set_xlabel("1/m")
    b.barh(ys, [float(r["worst_lag_distance"]) for r in rows], color=cols)
    b.set_title(f"Metres driven with the steering behind\nthe path ({speed:g} m/s)", fontsize=10)
    b.set_xlabel("m")
    vmax = [float(r["max_feasible_speed"]) for r in rows]
    c.barh(ys, [min(v, SPEED_AXIS) for v in vmax], color=cols)
    for y, v in zip(ys, vmax):
        if v == float("inf"):
            c.text(SPEED_AXIS - 0.4, y, "no limit", ha="right", va="center", fontsize=8, color="white")
    for v in BENCH_SPEEDS:
        c.axvline(v, color="0.25", ls=":", lw=1.2)
        c.text(v, len(rows) - 0.3, f" {v:g} m/s", fontsize=8, va="bottom")
    c.set_xlim(0, SPEED_AXIS)
    c.set_title(f"Largest drivable speed (lateral acceleration\n<= {mu_g:.2f} m/s$^2$) and benchmark speeds", fontsize=10)
    c.set_xlabel("m/s")
    a.set_yticks(list(ys))
    a.set_yticklabels(names, fontsize=8)
    for ax in (a, b, c):
        ax.grid(axis="x", alpha=0.25)
    handles = [plt.Rectangle((0, 0), 1, 1, color=col) for col in COLORS.values()]
    fig.legend(handles, ["Reeds-Shepp", "thesis, hybrid", "thesis, theta star"], fontsize=8,
               loc="lower center", ncol=3)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=130)
    print(f"saved {OUT}")


if __name__ == "__main__":
    main()
