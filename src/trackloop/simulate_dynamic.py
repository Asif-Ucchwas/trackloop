"""Closed-loop simulation of a path-tracking controller on the dynamic model."""
import numpy as np

from trackloop.models import dynamic as dyn
from trackloop.projection import project


def simulate_dynamic(controller, path, state0, dt, t_end, accel=0.0, params=None):
    """Returns the state trajectory (n + 1, 7) and the cross-track error (n + 1,).

    The state is the dynamic model state (position of the center of gravity), and the
    error is the signed distance of the center of gravity from the path.
    """
    params = params if params is not None else dyn.DynamicParams()
    n = int(round(t_end / dt))
    traj = np.zeros((n + 1, 7))
    cte = np.zeros(n + 1)
    traj[0] = state0
    controller.reset()
    index = 0
    for i in range(n + 1):
        proj = project(path, traj[i, dyn.X], traj[i, dyn.Y], index, 300)
        index, cte[i] = proj.index, proj.error
        if i < n:
            cmd = controller.command(traj[i])
            traj[i + 1] = dyn.step(traj[i], cmd, accel, dt, params)
    return traj, cte
