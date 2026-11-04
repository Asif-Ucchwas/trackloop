"""Task 16 root-cause step 1: which parking paths drive the Kinematic MPC lag sensitivity?

Reruns Kinematic MPC and Kinematic LQR at 3 m/s, seeds 0-4, nominal and lag_0.2, and prints the
per-path ratio of RMS lateral error (lag / nominal), sorted by the MPC ratio. Evidence only.
"""
import sys
from multiprocessing import Pool

import numpy as np

from trackloop import harness as h

CTRL = ("Kinematic MPC", "Kinematic LQR")
SPEED = 3.0
SEEDS = (0, 1, 2, 3, 4)


def job(t):
    r = h.run_one(t)
    return t, r.get("rms_lateral"), r.get("finished")


if __name__ == "__main__":
    tasks = [(n, c, SPEED, s, k) for n in h.path_names() for c in CTRL for s in SEEDS for k in ("nominal", "lag_0.2")]
    with Pool() as p:
        out = p.map(job, tasks)
    res = {t: (rms, fin) for t, rms, fin in out}
    rows = []
    for n in h.path_names():
        ratio = {}
        for c in CTRL:
            rs = [res[(n, c, SPEED, s, "lag_0.2")][0] / res[(n, c, SPEED, s, "nominal")][0] for s in SEEDS
                  if res[(n, c, SPEED, s, "lag_0.2")][1] and res[(n, c, SPEED, s, "nominal")][1]]
            ratio[c] = float(np.median(rs)) if rs else float("nan")
        nom = np.median([res[(n, "Kinematic MPC", SPEED, s, "nominal")][0] for s in SEEDS])
        lag = np.median([res[(n, "Kinematic MPC", SPEED, s, "lag_0.2")][0] for s in SEEDS])
        rows.append((ratio["Kinematic MPC"], n, ratio["Kinematic LQR"], nom, lag))
    print(f"{'path':32s} {'MPC ratio':>9s} {'LQR ratio':>9s} {'MPC nominal rms':>16s} {'MPC lag0.2 rms':>15s}")
    for r in sorted(rows, reverse=True):
        print(f"{r[1]:32s} {r[0]:9.2f} {r[2]:9.2f} {r[3]:16.4f} {r[4]:15.4f}")
    unf = [t for t, (_, fin) in res.items() if not fin]
    print("runs not finished:", len(unf))
