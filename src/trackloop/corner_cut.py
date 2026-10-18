"""Corner-cutting measurement for a path that goes from a straight into a left arc."""
import numpy as np

from trackloop import paths
from trackloop.controllers.pure_pursuit import PurePursuit, PurePursuitParams
from trackloop.models.kinematic import KinematicParams
from trackloop.simulate import simulate

CORNER = 30.0       # m, where the straight ends and the curve starts
THEORY_PEAK = 0.1039   # peak inward error in units of ld^2 / R (small-angle theory)
THEORY_WHERE = 0.571   # location of the peak past the corner, in units of ld
THEORY_OUTWARD = -0.0045   # first outward overshoot, in units of ld^2 / R


def measure(radius, ld, v, dt=0.01, params=None):
    """Peak inward error (m), its distance past the corner, and the next outward dip."""
    arc_length = 25.0 + 6.0 * ld
    path = paths.straight_then_arc(CORNER, radius, arc_length / radius)
    pp = PurePursuit(path, PurePursuitParams(ld_min=ld, k_v=0.0))
    t_end = (CORNER + 20.0 + 3.0 * ld) / v
    state0 = [0.0, 0.0, 0.0, v, 0.0]
    params = params if params is not None else KinematicParams(max_steer_rate=100.0)
    traj, cte = simulate(pp, path, state0, dt, t_end, params=params)
    i = int(np.argmax(cte))
    return float(cte[i]), float(traj[i, 0] - CORNER), float(cte[i:].min())
