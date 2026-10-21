"""Re-score the cusp runs with the Task 13 metrics.

Runs the four controllers on the nine Reeds-Shepp test-set paths (same runs as
track_cusp_paths.py) and scores each log with trackloop.metrics.
Writes data/results/cusp_metrics.csv and docs/figures/metrics.png.
Exploratory: one run per pair, start on the path, kinematic model, no noise.
"""
import csv
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
from trackloop.metrics import score_cusp_run
from trackloop.testset import load_test_set

CSV = Path("data/results/cusp_metrics.csv")
FIG = Path("docs/figures/metrics.png")
MAKERS = {
    "Pure Pursuit": PurePursuit,
    "Stanley": lambda p: Stanley(p, StanleyParams(feedforward=bool(p.direction[0] < 0))),
    "Kinematic LQR": KinematicLQR,
    "Kinematic MPC": KinematicMPC,
}
COLORS = {"Pure Pursuit": "tab:blue", "Stanley": "tab:green",
          "Kinematic LQR": "tab:red", "Kinematic MPC": "tab:purple"}


def main():
    rows = []
    for name, path in load_test_set():
        if not name.startswith("rs_"):
            continue
        start = [path.x[0], path.y[0], path.heading[0], 0.0, 0.0]
        for ctrl, make in MAKERS.items():
            res = track_with_cusps(path, make, start)
            m, extra = score_cusp_run(path, res)
            rows.append({"path": name, "controller": ctrl, **{k: round(v, 5) if isinstance(v, float) else v
                                                              for k, v in {**m.as_dict(), **extra}.items()}})
    CSV.parent.mkdir(parents=True, exist_ok=True)
    with open(CSV, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    names = list(dict.fromkeys(r["path"] for r in rows))
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.2), sharey=True)
    panels = [("rms_lateral", "RMS lateral error (m)"), ("steering_rate_rms", "RMS steering rate (rad/s)"),
              ("chatter_per_s", "Steering-rate reversals per second")]
    h = 0.2
    for ax, (key, title) in zip(axes, panels):
        for j, ctrl in enumerate(MAKERS):
            vals = [next(r[key] for r in rows if r["path"] == n and r["controller"] == ctrl) for n in names]
            ax.barh(np.arange(len(names)) + (j - 1.5) * h, vals, h, label=ctrl, color=COLORS[ctrl])
        ax.set_title(title, fontsize=10)
        ax.grid(axis="x", alpha=0.25)
    axes[0].set_yticks(range(len(names)))
    axes[0].set_yticklabels(names, fontsize=8)
    axes[0].invert_yaxis()
    axes[0].legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    FIG.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG, dpi=130)
    print(f"{len(rows)} runs scored; wrote {CSV} and {FIG}")


if __name__ == "__main__":
    main()
