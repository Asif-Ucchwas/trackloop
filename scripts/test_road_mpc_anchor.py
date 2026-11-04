"""Task 16 root-cause step 9: do the Dynamic MPC divergences at 20 m/s with 0.2 s lag follow the same anchor?

The road MPC bounds its first steering step by one rate step from the measured steering angle
(state[6]). Wrap it so that angle is replaced by its own previous command (controller source untouched)
and compare, on the four 20 m/s road paths, seeds 0-4: runs that diverge, and median RMS lateral error,
for the standard and the anchored MPC under nominal, 0.1 s and 0.2 s steering lag. Evidence only.
"""
from multiprocessing import Pool

import numpy as np

from trackloop import dynamic_runner as dr
from trackloop import harness_road as hr
from trackloop.model_error import dynamic_plant

SEEDS, CASES = (0, 1, 2, 3, 4), ("nominal", "lag_0.1", "lag_0.2")


class Anchored:
    def __init__(self, inner):
        self.inner, self.prev = inner, None

    def reset(self):
        self.inner.reset()
        self.prev = None

    def command(self, state):
        s = np.array(state, dtype=float)
        if self.prev is not None:
            s[6] = self.prev
        self.prev = self.inner.command(s)
        return self.prev


def job(t):
    name, anchored, case, seed = t
    index, path, info = hr._road_set()[name]
    plant = dynamic_plant(None if case == "nominal" else case)
    e0, dpsi = hr.start_condition(seed, index)
    ctrl = hr.CONTROLLERS["Dynamic MPC"](path)
    ctrl = Anchored(ctrl) if anchored else ctrl
    res = dr.run(path, ctrl, float(info["design_speed_mps"]), e0, dpsi, params=plant)
    rms = dr.score_run(path, res, plant).rms_lateral
    return t, res.status, rms


if __name__ == "__main__":
    names = [n for n in hr.path_names() if n.endswith("_v20")]
    with Pool() as p:
        out = p.map(job, [(n, a, c, s) for n in names for a in (False, True) for c in CASES for s in SEEDS])
    print(f"{len(names)} paths x {len(SEEDS)} seeds = {len(names) * len(SEEDS)} runs per row")
    print(f"{'MPC':10s} {'case':8s} {'diverged':>9s} {'median RMS of finished (m)':>28s}")
    for a in (False, True):
        for c in CASES:
            sel = [(st, r) for t, st, r in out if t[1] == a and t[2] == c]
            ok = [r for st, r in sel if st == "finished"]
            print(f"{'anchored' if a else 'standard':10s} {c:8s} {sum(st != 'finished' for st, _ in sel):9d} {np.median(ok):28.4f}")
