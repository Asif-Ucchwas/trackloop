"""Road family: forward paths with the same lateral-acceleration profile at every speed.

A path for design speed v is built from a curvature profile kappa(s), piecewise linear in
arc length. Peak curvature is kappa_max = a_y / v^2, so driving the path at v needs a peak
lateral acceleration of a_y whatever v is. The curvature ramps between 0 and kappa_max
(a clothoid) over ramp_length = a_y * v / jerk metres, so the lateral acceleration changes
at the constant rate `jerk` (m/s^3) and the steering never has to jump.

Heading is the exact integral of the curvature. Position is the integral of the heading,
done numerically on a fine grid (step FINE_DS), then sampled every ds metres.

Shapes: curve (one 90 degree left turn), lateral_shift (a left pulse then a right pulse of
curvature: the heading returns to zero and the car has moved sideways, about 15 m at these
settings, far more than a lane), s_curve (60 degrees left, a straight, 60 degrees right) and
double_shift (a lateral shift, a straight, and the mirrored shift back to the start line).
"""
import math

import numpy as np

from trackloop.paths import Path

A_Y = 2.0               # m/s^2 peak lateral acceleration, a placeholder (about 1/4 of mu g)
JERK = 1.0              # m/s^3 lateral jerk while the curvature ramps, a placeholder
SPEEDS = (5.0, 10.0, 20.0)
SHAPES = ("curve", "lateral_shift", "s_curve", "double_shift")
LEAD_S = 3.0            # seconds of straight before and after the maneuver
GAP_S = 2.0             # seconds of straight between the two parts of s_curve / double_shift
FINE_DS = 0.005         # m, grid for the numerical integration of position
DS = 0.1                # m, sample spacing of the exported path


def peak_curvature(speed, a_y=A_Y):
    return a_y / speed ** 2


def ramp_length(speed, a_y=A_Y, jerk=JERK):
    return a_y * speed / jerk


def path_id(shape, speed):
    return f"road_{shape}_v{speed:g}"


def profile(shape, speed, a_y=A_Y, jerk=JERK):
    """Nodes (s, kappa) of the piecewise linear curvature profile."""
    if shape not in SHAPES:
        raise ValueError(f"unknown shape {shape!r}")
    kmax, sr = peak_curvature(speed, a_y), ramp_length(speed, a_y, jerk)
    nodes = [(0.0, 0.0)]

    def go(length, k_end):
        nodes.append((nodes[-1][0] + length, k_end))

    def straight(length):
        go(length, 0.0)

    def turn(sign, heading=None):
        hold = 0.0
        if heading is not None:
            hold = heading / kmax - sr
            if hold < -1e-9:
                raise ValueError("heading change is smaller than the two ramps alone")
        go(sr, sign * kmax)
        if hold > 1e-12:
            go(hold, sign * kmax)
        go(sr, 0.0)

    lead, gap = LEAD_S * speed, GAP_S * speed
    straight(lead)
    if shape == "curve":
        turn(+1, math.pi / 2)
    elif shape == "lateral_shift":
        turn(+1)
        turn(-1)
    elif shape == "s_curve":
        turn(+1, math.radians(60.0))
        straight(gap)
        turn(-1, math.radians(60.0))
    else:
        turn(+1)
        turn(-1)
        straight(gap)
        turn(-1)
        turn(+1)
    straight(lead)
    return np.array(nodes)


def _heading(nodes, s):
    """Exact integral of the piecewise linear curvature at arc lengths s."""
    sn, kn = nodes[:, 0], nodes[:, 1]
    theta_nodes = np.concatenate([[0.0], np.cumsum(0.5 * (kn[1:] + kn[:-1]) * np.diff(sn))])
    i = np.clip(np.searchsorted(sn, s, side="right") - 1, 0, len(sn) - 2)
    length = sn[i + 1] - sn[i]
    d = s - sn[i]
    slope = (kn[i + 1] - kn[i]) / length
    return theta_nodes[i] + kn[i] * d + 0.5 * slope * d * d


def build(shape, speed, a_y=A_Y, jerk=JERK, ds=DS, fine_ds=FINE_DS):
    """Return (Path, info) for the shape at the given design speed."""
    nodes = profile(shape, speed, a_y, jerk)
    total = float(nodes[-1, 0])
    fine = np.append(np.arange(0.0, total, fine_ds), total)
    theta = _heading(nodes, fine)
    ds_f = np.diff(fine)
    x = np.concatenate([[0.0], np.cumsum(0.5 * (np.cos(theta[1:]) + np.cos(theta[:-1])) * ds_f)])
    y = np.concatenate([[0.0], np.cumsum(0.5 * (np.sin(theta[1:]) + np.sin(theta[:-1])) * ds_f)])
    s = np.append(np.arange(0.0, total, ds), total)
    path = Path(np.interp(s, fine, x), np.interp(s, fine, y), _heading(nodes, s),
                np.interp(s, nodes[:, 0], nodes[:, 1]), s, np.ones(len(s)))
    info = {
        "name": path_id(shape, speed), "family": "road", "shape": shape,
        "design_speed_mps": speed, "peak_lateral_accel_mps2": a_y, "lateral_jerk_mps3": jerk,
        "peak_curvature_per_m": peak_curvature(speed, a_y),
        "min_radius_m": speed ** 2 / a_y, "ramp_length_m": ramp_length(speed, a_y, jerk),
        "lead_s": LEAD_S, "gap_s": GAP_S, "ds_m": ds, "fine_ds_m": fine_ds,
        "length_m": total, "end_heading_rad": float(path.heading[-1]),
        "end_x_m": float(path.x[-1]), "end_y_m": float(path.y[-1]),
        "profile_nodes": [[float(a), float(b)] for a, b in nodes],
    }
    return path, info
