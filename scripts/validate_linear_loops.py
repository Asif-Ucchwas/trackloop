"""Task 17: compare the linear closed-loop models with the nonlinear simulation on a straight path.

For each controller, speed and steering lag, the vehicle starts offset by e0 metres with zero heading
error and the simulation (models/dynamic.py with the benchmark controllers) is compared with the
discrete linear model of linear_loops.py. Reports the largest difference in lateral error over 4 s.
Writes data/results/linear_loops_validation.csv and docs/figures/linear_loops.png.
"""
import csv

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from trackloop import harness_road as hr
from trackloop import linear_loops as ll
from trackloop.models import dynamic as dyn
from trackloop.paths import straight

DT, STEPS = ll.DT, 80
PATH = straight(400.0)
OFFSETS, SPEEDS, LAGS = (0.05, 0.25, 0.5, 1.0, 2.0), (5.0, 10.0, 20.0), (0.0, 0.1, 0.2)


def compare(controller, v, lag, e0):
    params = dyn.DynamicParams(steer_lag=lag)
    ctrl = hr.CONTROLLERS[controller](PATH)
    ctrl.reset()
    s = np.array([0.0, e0, 0.0, v, 0.0, 0.0, 0.0])
    sim = [s[1]]
    for _ in range(STEPS):
        s = dyn.step(s, ctrl.command(s), 0.0, DT, params)
        sim.append(s[1])
    system = ll.closed_loop(controller, v, lag, dt=DT)
    z = np.zeros(system.A.shape[0])
    z[0] = e0
    lin = [z[0]]
    for _ in range(STEPS):
        z = system.A @ z
        lin.append(z[0])
    return np.array(sim), np.array(lin)


if __name__ == "__main__":
    rows = []
    for c in ll.CONTROLLERS:
        for v in SPEEDS:
            for lag in LAGS:
                for e0 in OFFSETS:
                    sim, lin = compare(c, v, lag, e0)
                    d = float(np.max(np.abs(sim - lin)))
                    rows.append({"controller": c, "speed": v, "lag": lag, "offset": e0,
                                 "max_difference_m": round(d, 6), "share_of_offset": round(d / e0, 5)})
    with open("data/results/linear_loops_validation.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"{'controller':14s} {'offset (m)':>10s} {'worst share of offset over 3 speeds x 3 lags':>46s}")
    for c in ll.CONTROLLERS:
        for e0 in OFFSETS:
            worst = max(r["share_of_offset"] for r in rows if r["controller"] == c and r["offset"] == e0)
            print(f"{c:14s} {e0:10.2f} {worst:46.1%}")

    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    t = np.arange(STEPS + 1) * DT
    colors = dict(zip(ll.CONTROLLERS, ("tab:blue", "tab:orange", "tab:green", "tab:red")))
    for c in ll.CONTROLLERS:
        sim, lin = compare(c, 10.0, 0.1, 0.5)
        ax[0].plot(t, sim, color=colors[c], lw=2, label=f"{c} (simulation)")
        ax[0].plot(t, lin, color="k", lw=1, ls="--")
    ax[0].plot([], [], color="k", lw=1, ls="--", label="linear model")
    ax[0].set(xlabel="time (s)", ylabel="lateral error of the centre of gravity (m)",
              title="10 m/s, 0.1 s lag, 0.5 m start offset")
    ax[0].legend(fontsize=8)
    for c in ll.CONTROLLERS:
        share = [max(r["share_of_offset"] for r in rows if r["controller"] == c and r["offset"] == e0) for e0 in OFFSETS]
        ax[1].plot(OFFSETS, np.array(share) * 100, "o-", color=colors[c], label=c)
    ax[1].set(xscale="log", xlabel="start offset (m)", ylabel="largest difference from simulation (% of offset)",
              title="Where the linear model stops matching\n(worst of 3 speeds x 3 lags)")
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig("docs/figures/linear_loops.png", dpi=150)
    print("wrote docs/figures/linear_loops.png and data/results/linear_loops_validation.csv")
