"""Task 16 root-cause step 2: trace one parking run, nominal against lag, for the MPC and the LQR.

Records the steering command each controller issues and the steering angle the plant reaches, then
reports signed-error zero crossings (oscillation rate), peak error, and how far the command leads the
steering angle. Evidence only: it states what was measured, not why.
"""
import sys

import numpy as np

from trackloop import harness as h
from trackloop.cusp_runner import RunnerParams, track_with_cusps
from trackloop.model_error import kinematic_plant

PATH, SPEED, SEED = "thesis_theta_star_tight_seed2", 3.0, 0


class Spy:
    def __init__(self, inner, log):
        self.inner, self.log = inner, log

    def reset(self):
        self.inner.reset()

    def command(self, state):
        c = self.inner.command(state)
        self.log.append((float(state[4]), c))
        return c

    def __getattr__(self, name):
        return getattr(self.inner, name)


def trace(controller, case):
    index, path = h._test_set()[PATH]
    e0, dpsi = h.start_condition(SEED, index)
    log = []
    make = lambda seg_path: Spy(h.CONTROLLERS[controller](seg_path), log)
    params = RunnerParams(speed=SPEED, max_time=h.max_time(path, SPEED))
    res = track_with_cusps(path, make, h.start_state(path, e0, dpsi), params=params, kin_params=kinematic_plant(case))
    cte = res.cte
    zc = int(np.sum(np.diff(np.sign(cte[np.abs(cte) > 0.02])) != 0))
    log = np.array(log)
    cmd_minus_delta = log[:, 1] - log[:, 0]
    cmd_flips = int(np.sum(np.diff(np.sign(np.diff(log[:, 1]))) != 0))
    dur = res.time[-1]
    return dict(duration_s=dur, finished=res.finished, rms=float(np.sqrt(np.mean(cte**2))), peak=float(np.max(np.abs(cte))),
                err_zero_crossings_per_s=zc / dur, cmd_direction_changes_per_s=cmd_flips / dur,
                rms_cmd_minus_angle=float(np.sqrt(np.mean(cmd_minus_delta**2))),
                frac_time_rate_limited=float(np.mean(np.abs(cmd_minus_delta) > 0.8 * 0.05)))


if __name__ == "__main__":
    print(f"path {PATH}, speed {SPEED}, seed {SEED}")
    keys = None
    for c in ("Kinematic MPC", "Kinematic LQR"):
        for case in ("nominal", "lag_0.2"):
            d = trace(c, case)
            if keys is None:
                keys = list(d)
                print(f"{'controller':16s} {'case':8s} " + " ".join(f"{k[:14]:>14s}" for k in keys))
            print(f"{c:16s} {case:8s} " + " ".join(f"{d[k]:14.3f}" if not isinstance(d[k], bool) else f"{str(d[k]):>14s}" for k in keys))
