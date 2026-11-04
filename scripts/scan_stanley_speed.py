"""Task 16 root-cause step 5: does Stanley's chatter depend on how many path points one step covers?

The road paths have points every 0.1 m and a run steps 0.05 s, so one step covers speed / 2 points.
Scan speed in fine steps on the four 5 m/s road paths (k = 1.5, seeds 0-4) and print median
chatter against speed and against the fractional part of points-per-step. Evidence only.
"""
from multiprocessing import Pool

import numpy as np

from trackloop import dynamic_runner as dr
from trackloop import harness_road as hr
from trackloop.controllers.stanley import Stanley, StanleyParams
from trackloop.models.dynamic import DynamicParams

SPEEDS = tuple(np.round(np.arange(2.0, 8.01, 0.25), 2))
SEEDS = (0, 1, 2, 3, 4)
PLANT = DynamicParams()


def job(t):
    name, v, seed = t
    index, path, _ = hr._road_set()[name]
    e0, dpsi = hr.start_condition(seed, index)
    res = dr.run(path, dr.RearAxleAdapter(Stanley(path, StanleyParams(feedforward=False)), hr.LR), v, e0, dpsi, params=PLANT)
    return t, dr.score_run(path, res, PLANT).chatter_per_s


if __name__ == "__main__":
    names = [n for n in hr.path_names() if n.endswith("_v5")]
    with Pool() as p:
        out = p.map(job, [(n, float(v), s) for n in names for v in SPEEDS for s in SEEDS])
    print(f"{'m/s':>5s} {'points/step':>12s} {'median chatter /s':>18s}")
    for v in SPEEDS:
        c = np.median([x for t, x in out if abs(t[1] - v) < 1e-9])
        print(f"{v:5.2f} {v * dr.DT / 0.1:12.3f} {c:18.2f}")
