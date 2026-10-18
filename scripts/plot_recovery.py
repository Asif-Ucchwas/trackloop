"""Figure: LQR and MPC recovering from a lateral offset with the real steering limits.

Straight path, 10 m/s, dt = 0.05 s, 12 s. Saves docs/figures/lqr_vs_mpc_recovery.png.
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from trackloop import paths
from trackloop.controllers.lqr import LQRController
from trackloop.controllers.mpc import MPCController
from trackloop.models.dynamic import DynamicParams
from trackloop.simulate_dynamic import simulate_dynamic

DT, T_END, OFFSET = 0.05, 12.0, 5.0
OUT = Path("docs/figures/lqr_vs_mpc_recovery.png")


def run(controller, path):
    state0 = [0.0, OFFSET, 0.0, 10.0, 0.0, 0.0, 0.0]
    return simulate_dynamic(controller, path, state0, DT, T_END, params=DynamicParams())


def main():
    path = paths.straight(300.0)
    runs = {"LQR": run(LQRController(path), path),
            "MPC": run(MPCController(path, dt=DT, horizon=20), path)}
    t = np.arange(int(round(T_END / DT)) + 1) * DT
    fig, (top, mid, ax) = plt.subplots(3, 1, figsize=(7, 8))
    for name, (traj, cte) in runs.items():
        top.plot(t, cte, label=name)
        mid.plot(t, traj[:, 6], label=name)
        ax.plot(traj[:, 0], traj[:, 1], label=name)
    top.axhline(0.0, color="0.6", lw=0.8)
    top.axhspan(-0.05, 0.05, color="0.9", zorder=0)
    top.set_ylabel("cross-track error (m)")
    mid.axhline(0.6, color="0.6", lw=0.8, ls="--")
    mid.axhline(-0.6, color="0.6", lw=0.8, ls="--")
    mid.set_ylabel("steering angle (rad)")
    for axis in (top, mid):
        axis.set_xlim(0.0, T_END)
        axis.set_xlabel("time (s)")
    ax.axhline(0.0, color="0.4", lw=1.0, ls=":", label="path")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    top.legend(loc="upper right")
    ax.legend(loc="upper right")
    top.set_title(f"Recovery from a {OFFSET:.0f} m offset at 10 m/s "
                  "(limits 0.6 rad, 0.8 rad/s)")
    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=130)
    print(f"saved {OUT}")


if __name__ == "__main__":
    main()
