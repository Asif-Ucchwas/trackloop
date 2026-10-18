"""Closed-loop simulation of a path-tracking controller on the kinematic model."""
import numpy as np

from trackloop.models import kinematic as kin
from trackloop.paths import cross_track_error


def _error_at(path, state, ref_offset):
    x = state[kin.X] + ref_offset * np.cos(state[kin.PSI])
    y = state[kin.Y] + ref_offset * np.sin(state[kin.PSI])
    return cross_track_error(path, x, y)


def simulate(controller, path, state0, dt, t_end, accel=0.0, params=None,
             ref_offset=0.0):
    """Returns the state trajectory (n + 1, 5) and the cross-track error (n + 1,).

    The error is measured at the point ref_offset ahead of the rear axle along the
    heading: 0 for the rear axle, the wheelbase for the front axle.
    """
    params = params if params is not None else kin.KinematicParams()
    n = int(round(t_end / dt))
    traj = np.zeros((n + 1, 5))
    cte = np.zeros(n + 1)
    traj[0] = state0
    controller.reset()
    cte[0] = _error_at(path, traj[0], ref_offset)
    for i in range(n):
        cmd = controller.command(traj[i])
        traj[i + 1] = kin.step(traj[i], cmd, accel, dt, params)
        cte[i + 1] = _error_at(path, traj[i + 1], ref_offset)
    return traj, cte
