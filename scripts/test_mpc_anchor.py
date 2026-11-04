"""Task 16 root-cause step 3: is the Kinematic MPC's lag damage caused by anchoring its first-step
rate bound to the measured steering angle?

Experiment (diagnostic only, the controller source is untouched): wrap the MPC so the steering angle it
sees is replaced by its own previous command. Compare RMS lateral error at 3 m/s, seeds 0-4, median per path:
standard MPC nominal, standard MPC lag 0.2, anchored MPC nominal, anchored MPC lag 0.2.
"""
from multiprocessing import Pool

import numpy as np

from trackloop import harness as h
from trackloop.cusp_runner import RunnerParams, track_with_cusps
from trackloop.metrics import score_cusp_run
from trackloop.model_error import kinematic_plant

SPEED, SEEDS = 3.0, (0, 1, 2, 3, 4)


class Anchored:
    def __init__(self, inner):
        self.inner, self.prev = inner, None

    def reset(self):
        self.inner.reset()
        self.prev = None

    def command(self, state):
        s = np.array(state, dtype=float)
        if self.prev is not None:
            s[4] = self.prev
        self.prev = self.inner.command(s)
        return self.prev


def run(task):
    name, anchored, case, seed = task
    index, path = h._test_set()[name]
    e0, dpsi = h.start_condition(seed, index)
    make = (lambda p: Anchored(h.CONTROLLERS["Kinematic MPC"](p))) if anchored else h.CONTROLLERS["Kinematic MPC"]
    res = track_with_cusps(path, make, h.start_state(path, e0, dpsi),
                           params=RunnerParams(speed=SPEED, max_time=h.max_time(path, SPEED)), kin_params=kinematic_plant(case))
    m, _ = score_cusp_run(path, res)
    return task, m.as_dict()["rms_lateral"], res.finished


if __name__ == "__main__":
    variants = [(False, "nominal"), (False, "lag_0.2"), (True, "nominal"), (True, "lag_0.2")]
    tasks = [(n, a, c, s) for n in h.path_names() for a, c in variants for s in SEEDS]
    with Pool() as p:
        out = p.map(run, tasks)
    res = {t: (r, f) for t, r, f in out}
    print(f"{'path':32s} {'std nom':>8s} {'std lag':>8s} {'anch nom':>9s} {'anch lag':>9s}")
    cols = []
    for n in h.path_names():
        row = [float(np.median([res[(n, a, c, s)][0] for s in SEEDS])) for a, c in variants]
        cols.append(row)
        print(f"{n:32s} " + " ".join(f"{v:8.3f}" for v in row))
    cols = np.array(cols)
    print(f"{'median over paths':32s} " + " ".join(f"{v:8.3f}" for v in np.median(cols, axis=0)))
    print("lag/nominal, standard MPC: %.2f   anchored MPC: %.2f" % (np.median(cols[:, 1] / cols[:, 0]), np.median(cols[:, 3] / cols[:, 2])))
    print("runs not finished:", sum(not f for _, f in res.values()))
