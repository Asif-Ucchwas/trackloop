"""Task 20: how much of the gap between the linear loop models and the simulation each steering limit causes.

Offsets 0.05 to 2 m on a straight path, worst case over speeds 5, 10, 20 m/s and steering lags 0, 0.1, 0.2 s.
Variants: default; plant steering rate limit removed; (MPC) its own constraints removed; both removed.
Writes data/results/linearization_limits.csv and docs/figures/linearization_limits.png.
"""
import csv

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from trackloop import linear_loops as ll
from trackloop import linearization as lz

OFFSETS = (0.05, 0.25, 0.5, 1.0, 2.0)
VARIANTS = {"default": {}, "plant rate limit removed": {"free_plant_rate": True},
            "controller limits removed (MPC only)": {"free_controller_limits": True},
            "both removed": {"free_plant_rate": True, "free_controller_limits": True}}


def main():
    rows = []
    for c in ll.CONTROLLERS:
        for name, kw in VARIANTS.items():
            if name.startswith("controller") and c != "Dynamic MPC":
                continue
            if name == "both removed" and c != "Dynamic MPC":
                continue
            for e0 in OFFSETS:
                gap, rate, angle = lz.worst_gap(c, e0, **kw)
                rows.append({"controller": c, "variant": name, "offset": e0, "worst_gap_share": round(gap, 5),
                             "peak_steer_rate": round(rate, 3), "peak_steer_angle": round(angle, 4)})
    with open("data/results/linearization_limits.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print("Worst gap between linear model and simulation, share of the offset (3 speeds x 3 lags)")
    for c in ll.CONTROLLERS:
        for name in VARIANTS:
            sel = [r for r in rows if r["controller"] == c and r["variant"] == name]
            if sel:
                print(f"{c:13s} {name:38s} " + "  ".join(f"{r['offset']:g} m: {r['worst_gap_share']:7.1%}" for r in sel))
    print("Peak steering rate demanded at 2 m (rad/s; plant limit 0.8) and peak steering angle (rad; limit 0.6)")
    for c in ll.CONTROLLERS:
        name = "both removed" if c == "Dynamic MPC" else "plant rate limit removed"
        r = [x for x in rows if x["controller"] == c and x["variant"] == name and x["offset"] == 2.0][0]
        print(f"{c:13s} rate {r['peak_steer_rate']:6.2f}   angle {r['peak_steer_angle']:.2f}")

    fig, ax = plt.subplots(1, 4, figsize=(15, 3.8), sharey=True)
    for a, c in zip(ax, ll.CONTROLLERS):
        for name in VARIANTS:
            sel = [r for r in rows if r["controller"] == c and r["variant"] == name]
            if sel:
                a.plot([r["offset"] for r in sel], [100 * max(r["worst_gap_share"], 1e-4) for r in sel], "o-", label=name)
        a.set(xscale="log", yscale="log", xlabel="start offset (m)", title=c)
        a.grid(alpha=0.3, which="both")
    ax[0].set_ylabel("largest gap to linear model (% of offset)")
    ax[0].legend(fontsize=7)
    ax[3].legend(fontsize=7)
    fig.suptitle("Where the linear model stops matching the simulation: the steering rate limit")
    fig.tight_layout()
    fig.savefig("docs/figures/linearization_limits.png", dpi=150)
    print("wrote data/results/linearization_limits.csv and docs/figures/linearization_limits.png")


if __name__ == "__main__":
    main()
