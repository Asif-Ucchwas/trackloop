"""Task 16 root-cause step 7: does path spacing change the road benchmark for Pure Pursuit and Stanley?

The two controllers that look up the nearest path point (Pure Pursuit, Stanley) on all 12 road paths at their
design speeds, seeds 0-4, with the controller's path at 0.1 m (as benchmarked) and 0.05 m spacing.
Prints median RMS lateral error and median chatter per controller and speed. Scoring uses the original path.
"""
from multiprocessing import Pool

import numpy as np

from trackloop import dynamic_runner as dr
from trackloop import harness_road as hr
from trackloop.models.dynamic import DynamicParams
from trackloop.paths import Path

PLANT, SEEDS, SPACINGS, CTRL = DynamicParams(), (0, 1, 2, 3, 4), (0.1, 0.05), ("Pure Pursuit", "Stanley")


def resample(p, ds):
    s = np.append(np.arange(0.0, p.s[-1], ds), p.s[-1])
    it = lambda a: np.interp(s, p.s, a)
    return Path(it(p.x), it(p.y), np.interp(s, p.s, np.unwrap(p.heading)), it(p.curvature), s, np.sign(it(p.direction)))


def job(t):
    name, c, ds, seed = t
    index, path, info = hr._road_set()[name]
    e0, dpsi = hr.start_condition(seed, index)
    cp = path if ds == 0.1 else resample(path, ds)
    res = dr.run(path, hr.CONTROLLERS[c](cp), float(info["design_speed_mps"]), e0, dpsi, params=PLANT)
    m = dr.score_run(path, res, PLANT)
    return t, float(info["design_speed_mps"]), m.rms_lateral, m.chatter_per_s


if __name__ == "__main__":
    with Pool() as p:
        out = p.map(job, [(n, c, ds, s) for n in hr.path_names() for c in CTRL for ds in SPACINGS for s in SEEDS])
    print(f"{'controller':14s} {'m/s':>4s} | {'rms 0.10':>9s} {'rms 0.05':>9s} | {'chat 0.10':>9s} {'chat 0.05':>9s}")
    for c in CTRL:
        for v in (5.0, 10.0, 20.0):
            row = []
            for k in (2, 3):
                for ds in SPACINGS:
                    row.append(np.median([x[k] for x in out if x[0][1] == c and x[0][2] == ds and x[1] == v]))
            print(f"{c:14s} {v:4.0f} | {row[0]:9.4f} {row[1]:9.4f} | {row[2]:9.2f} {row[3]:9.2f}")
