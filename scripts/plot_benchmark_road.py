"""Figure for the road-family benchmark (Task 14): error by controller and speed, and tire slip."""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

MU, G, MASS, LF, LR, CF, CR = 0.8, 9.81, 1500.0, 1.2, 1.5, 80000.0, 100000.0
# slip angle at which the linear tire force equals the friction limit mu*Fz (placeholder mu)
FRONT_LIMIT = np.degrees(MU * MASS * G * LR / (LF + LR) / CF)
REAR_LIMIT = np.degrees(MU * MASS * G * LF / (LF + LR) / CR)

rows = list(csv.DictReader(open("data/results/benchmark_road.csv")))
C = ["Pure Pursuit", "Stanley", "Dynamic LQR", "Dynamic MPC"]
V = [5.0, 10.0, 20.0]
col = dict(zip(C, ["tab:orange", "tab:green", "tab:blue", "tab:purple"]))


def pick(c, v, key):
    return np.array([float(r[key]) for r in rows if r["controller"] == c and float(r["speed"]) == v])


fig, ax = plt.subplots(1, 3, figsize=(15, 4.4))
w = 0.2
for i, c in enumerate(C):
    x = np.arange(3) + (i - 1.5) * w
    ax[0].bar(x, [np.median(pick(c, v, "rms_lateral")) for v in V], w, color=col[c], label=c)
    ax[1].boxplot([pick(c, v, "max_lateral") for v in V], positions=x, widths=w * 0.85,
                  patch_artist=True, boxprops=dict(facecolor=col[c], alpha=0.6),
                  medianprops=dict(color="k"), flierprops=dict(markersize=3))
    ax[2].bar(x, [pick(c, v, "max_slip_front_deg").max() for v in V], w, color=col[c])
    ax[2].plot(x, [pick(c, v, "max_slip_rear_deg").max() for v in V], "k_", ms=9, mew=2,
               label="rear axle" if i == 0 else None)
ax[2].axhline(FRONT_LIMIT, color="k", ls="--", lw=1, label="front: linear force = mu*Fz")
ax[2].axhline(REAR_LIMIT, color="k", ls=":", lw=1.2, label="rear: linear force = mu*Fz")
ax[2].legend(fontsize=7, loc="upper left")
for a, t, yl in zip(ax, ["", "Maximum lateral error, per run",
                         "Largest slip angle reached (bars: front)"], ["m", "m", "deg"]):
    a.set_xticks(range(3), [f"{int(v)} m/s" for v in V])
    a.set_title(t, fontsize=10)
    a.set_ylabel(yl)
    a.grid(axis="y", alpha=0.3)
ax[0].set_title("Median RMS lateral error (20 runs per bar)", fontsize=10)
ax[0].legend(fontsize=8)
fig.suptitle("Road family: 4 controllers x 12 paths x 5 seeds, dynamic model, constant design speed", fontsize=11)
fig.tight_layout()
Path("docs/figures").mkdir(parents=True, exist_ok=True)
fig.savefig("docs/figures/benchmark_road.png", dpi=130)
print("wrote docs/figures/benchmark_road.png")
