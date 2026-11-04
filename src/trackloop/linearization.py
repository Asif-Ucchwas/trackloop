"""Where the linear loop models stop matching the nonlinear simulation, and why (Task 20).

compare() runs the nonlinear simulation and the sampled linear model of linear_loops.py from the same offset
on a straight path and returns how far apart they get. The steering rate limit can be removed from the plant
(`free_plant_rate`) and, for the MPC, from the controller's own constraints (`free_controller_limits`), to see
how much of the gap each one causes.
"""
import numpy as np

from trackloop import harness_road as hr
from trackloop import linear_loops as ll
from trackloop.controllers.mpc import MPCController
from trackloop.models import dynamic as dyn
from trackloop.paths import straight

FREE = {"max_steer_rate": 1e6, "max_steer": 10.0}
SPEEDS = (5.0, 10.0, 20.0)
LAGS = (0.0, 0.1, 0.2)


def compare(controller, vx, lag, e0, free_plant_rate=False, free_controller_limits=False, steps=80, dt=ll.DT):
    """Largest |simulation - linear model| of the lateral error over `steps` steps, as a share of e0.

    Returns (gap, peak steering rate demanded in rad/s, peak steering angle in rad) from the simulation.
    free_plant_rate removes the plant's steering rate limit; free_controller_limits removes the steering
    angle and rate constraints inside the MPC (it has no effect on the other controllers).
    """
    plant = dyn.DynamicParams(steer_lag=lag, **({"max_steer_rate": 1e6} if free_plant_rate else {}))
    path = straight(400.0)
    if controller == "Dynamic MPC" and free_controller_limits:
        ctrl = MPCController(path, params=dyn.DynamicParams(**FREE), dt=dt)
    else:
        ctrl = hr.CONTROLLERS[controller](path)
    ctrl.reset()
    s = np.array([0.0, e0, 0.0, vx, 0.0, 0.0, 0.0])
    sim, peak_rate, peak_angle, prev = [e0], 0.0, 0.0, 0.0
    for _ in range(steps):
        s = dyn.step(s, ctrl.command(s), 0.0, dt, plant)
        sim.append(s[1])
        peak_rate = max(peak_rate, abs(s[6] - prev) / dt)
        peak_angle = max(peak_angle, abs(s[6]))
        prev = s[6]
    lin = ll.closed_loop(controller, vx, lag, dt=dt)
    z = np.zeros(lin.A.shape[0])
    z[0] = e0
    ref = [e0]
    for _ in range(steps):
        z = lin.A @ z
        ref.append(z[0])
    gap = float(np.abs(np.array(sim) - np.array(ref)).max() / e0)
    return gap, float(peak_rate), float(peak_angle)


def worst_gap(controller, e0, speeds=SPEEDS, lags=LAGS, **kw):
    """Largest gap share, and the largest peak steering rate, over the speeds and lags."""
    runs = [compare(controller, v, lag, e0, **kw) for v in speeds for lag in lags]
    return max(r[0] for r in runs), max(r[1] for r in runs), max(r[2] for r in runs)
