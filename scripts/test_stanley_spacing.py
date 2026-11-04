"""Task 16 root-cause step 6: does Stanley's chatter shrink when the path points are closer together?

Resamples the four 5 m/s road paths to 0.1 (as benchmarked), 0.05 and 0.02 m spacing (linear in arc
length; heading, curvature and direction interpolated the same way) and prints the median chatter at
3, 4 and 5 m/s (k = 1.5, seeds 0-4). If the chatter comes from the discrete nearest-point lookup it
should fall as the spacing falls. Scoring uses the original path. Evidence only.
"""
from multiprocessing import Pool

import numpy as np

from trackloop import dynamic_runner as dr
from trackloop import harness_road as hr
from trackloop.controllers.stanley import Stanley, StanleyParams
from trackloop.models.dynamic import DynamicParams
from trackloop.paths import Path

SPACINGS, SPEEDS, SEEDS = (0.1, 0.05, 0.02), (3.0, 4.0, 5.0), (0, 1, 2, 3, 4)
PLANT = DynamicParams()


def resample(p, ds):
    s = np.arange(0.0, p.s[-1], ds)
    s = np.append(s, p.s[-1])
    it = lambda a: np.interp(s, p.s, a)
    heading = np.interp(s, p.s, np.unwrap(p.heading))
    return Path(it(p.x), it(p.y), heading, it(p.curvature), s, np.sign(it(p.direction)))


def job(t):
    name, ds, v, seed = t
    index, path, _ = hr._road_set()[name]
    e0, dpsi = hr.start_condition(seed, index)
    ctrl_path = path if ds == 0.1 else resample(path, ds)
    ctrl = dr.RearAxleAdapter(Stanley(ctrl_path, StanleyParams(feedforward=False), search_window=int(30.0 / ds)), hr.LR)
    res = dr.run(path, ctrl, v, e0, dpsi, params=PLANT)
    m = dr.score_run(path, res, PLANT)
    return t, m.chatter_per_s, m.rms_lateral


if __name__ == "__main__":
    names = [n for n in hr.path_names() if n.endswith("_v5")]
    with Pool() as p:
        out = p.map(job, [(n, ds, v, s) for n in names for ds in SPACINGS for v in SPEEDS for s in SEEDS])
    print("median chatter (reversals/s) | median RMS lateral (m); rows = spacing, columns = speed")
    print(f"{'ds (m)':>7s} " + " ".join(f"{str(v) + ' m/s':>18s}" for v in SPEEDS))
    for ds in SPACINGS:
        cells = []
        for v in SPEEDS:
            sel = [(c, r) for t, c, r in out if t[1] == ds and t[2] == v]
            cells.append(f"{np.median([c for c, _ in sel]):8.2f} | {np.median([r for _, r in sel]):7.3f}")
        print(f"{ds:7.2f} " + " ".join(f"{c:>18s}" for c in cells))
