"""Run a path-tracking controller on the dynamic model at constant speed, and log it.

The vehicle starts on (or offset from) the start of the path at the design speed with zero
sideslip and yaw rate, and the longitudinal acceleration is zero, so the speed stays at the
design speed. The run ends when the rear axle is within ARRIVE metres of the end of the path.
It is stopped early, and marked, if the rear axle is more than MAX_ERROR from the path
("diverged") or the time limit passes ("timeout").

The dynamic model logs the centre of gravity. Errors are scored at the rear axle (see
trackloop.metrics), the same point as for the kinematic runs. Controllers written for the
kinematic rear-axle state (Pure Pursuit, Stanley) are wrapped in RearAxleAdapter.
"""
import math
from dataclasses import dataclass

import numpy as np

from trackloop.metrics import compute, errors_on_path, rear_axle
from trackloop.models import dynamic as dyn
from trackloop.projection import project

DT = 0.05           # s, plant and controller step (the MPC is discretised at the same step)
ARRIVE = 0.5        # m, distance to the path end at which a run counts as finished
MAX_ERROR = 25.0    # m, rear-axle distance from the path beyond which a run is stopped


class RearAxleAdapter:
    """Lets a kinematic rear-axle controller steer the dynamic model.

    The controller sees [x, y, psi, vx, delta] with (x, y) the rear axle, which is lr behind
    the centre of gravity. Its steering command goes to the dynamic model unchanged.
    """

    def __init__(self, controller, lr):
        self.controller, self.lr = controller, lr

    def reset(self):
        self.controller.reset()

    def command(self, state):
        x, y = rear_axle(state[dyn.X], state[dyn.Y], state[dyn.PSI], self.lr)
        return self.controller.command([x, y, state[dyn.PSI], state[dyn.VX], state[dyn.DELTA]])


@dataclass
class DynamicRun:
    states: np.ndarray      # (n + 1, 7), dynamic model state (centre of gravity)
    time: np.ndarray
    status: str             # "finished", "diverged" or "timeout"


def time_limit(path, speed):
    return 3.0 * float(path.s[-1]) / speed + 10.0


def start_state(path, speed, e0, dpsi, params):
    """Centre-of-gravity state whose rear axle is e0 left of the path start, heading off by dpsi."""
    h = float(path.heading[0])
    psi = h + dpsi
    xr = float(path.x[0]) - e0 * math.sin(h)
    yr = float(path.y[0]) + e0 * math.cos(h)
    return [xr + params.lr * math.cos(psi), yr + params.lr * math.sin(psi), psi, speed, 0.0, 0.0, 0.0]


def run(path, controller, speed, e0=0.0, dpsi=0.0, params=None, dt=DT):
    params = params if params is not None else dyn.DynamicParams()
    state = np.array(start_state(path, speed, e0, dpsi, params), dtype=float)
    controller.reset()
    total, limit = float(path.s[-1]), time_limit(path, speed)
    rows, index, status, t = [state.copy()], 0, "finished", 0.0
    while True:
        xr, yr = rear_axle(state[dyn.X], state[dyn.Y], state[dyn.PSI], params.lr)
        proj = project(path, xr, yr, index, 300)
        index = proj.index
        if total - proj.s <= ARRIVE:
            break
        if abs(proj.error) > MAX_ERROR:
            status = "diverged"
            break
        if t >= limit:
            status = "timeout"
            break
        state = dyn.step(state, controller.command(state), 0.0, dt, params)
        rows.append(state.copy())
        t += dt
    states = np.array(rows)
    return DynamicRun(states, np.arange(len(states)) * dt, status)


def tire_use(states, params=None):
    """Largest front and rear slip angle (rad) and body lateral acceleration (m/s^2) of a run."""
    p = params if params is not None else dyn.DynamicParams()
    vx = np.maximum(states[:, dyn.VX], p.v_min)
    vy, r, delta = states[:, dyn.VY], states[:, dyn.R], states[:, dyn.DELTA]
    alpha_f = delta - np.arctan2(vy + p.lf * r, vx)
    alpha_r = -np.arctan2(vy - p.lr * r, vx)
    ay = (p.cf * alpha_f * np.cos(delta) + p.cr * alpha_r) / p.mass
    return float(np.abs(alpha_f).max()), float(np.abs(alpha_r).max()), float(np.abs(ay).max())


def score_run(path, result, params=None):
    """Metrics (trackloop.metrics.Metrics) of a dynamic run, measured at the rear axle."""
    p = params if params is not None else dyn.DynamicParams()
    s = result.states
    xr, yr = rear_axle(s[:, dyn.X], s[:, dyn.Y], s[:, dyn.PSI], p.lr)
    cte, head = errors_on_path(path, xr, yr, s[:, dyn.PSI])
    return compute(cte, head, result.time, s[:, dyn.DELTA], s[:, dyn.VX])
