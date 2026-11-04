"""Task 16 root-cause step 4: when does Stanley chatter on the road family?

Runs Stanley (feedforward off, as in the benchmark) on the four 5 m/s road paths, seeds 0-4, over a
range of speeds and cross-track gains k, and prints the median steering-rate reversals per second and
median RMS lateral error. The benchmark itself used k = 1.5 and speeds 5, 10, 20. Evidence only.
"""
from dataclasses import replace
from multiprocessing import Pool

import numpy as np

from trackloop import dynamic_runner as dr
from trackloop import harness as _unused  # noqa: F401  (keeps import order identical to the harnesses)
from trackloop import harness_road as hr
from trackloop.controllers.stanley import Stanley, StanleyParams
from trackloop.models.dynamic import DynamicParams

SPEEDS = (3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 15.0, 20.0)
GAINS = (0.75, 1.5, 3.0)
SEEDS = (0, 1, 2, 3, 4)
PLANT = DynamicParams()


def job(t):
    name, v, k, seed = t
    index, path, _ = hr._road_set()[name]
    e0, dpsi = hr.start_condition(seed, index)
    ctrl = dr.RearAxleAdapter(Stanley(path, StanleyParams(feedforward=False, k=k)), hr.LR)
    res = dr.run(path, ctrl, v, e0, dpsi, params=PLANT)
    m = dr.score_run(path, res, PLANT)
    return t, m.chatter_per_s, m.rms_lateral, res.status


if __name__ == "__main__":
    names = [n for n in hr.path_names() if n.endswith("_v5")]
    tasks = [(n, v, k, s) for n in names for v in SPEEDS for k in GAINS for s in SEEDS]
    with Pool() as p:
        out = p.map(job, tasks)
    print(f"paths: {names}")
    print("median chatter (reversals/s) | median RMS lateral (m); rows = speed, columns = k")
    print(f"{'m/s':>5s} " + " ".join(f"{'k=' + str(k):>18s}" for k in GAINS))
    for v in SPEEDS:
        cells = []
        for k in GAINS:
            sel = [(c, r) for (t, c, r, st) in out if t[1] == v and t[2] == k]
            cells.append(f"{np.median([c for c, _ in sel]):8.2f} | {np.median([r for _, r in sel]):7.3f}")
        print(f"{v:5.1f} " + " ".join(f"{c:>18s}" for c in cells))
    print("runs not finished:", sum(st != "finished" for *_, st in out))
