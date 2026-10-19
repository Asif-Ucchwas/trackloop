"""Stanley path tracking for the rear-axle kinematic model.

Forward driving: the law references the front axle: delta = psi_e - atan(k e / (v + k_soft)),
where e is the front axle cross-track error (positive left of the path) and psi_e is the
path heading minus the vehicle heading at the path point nearest the front axle.

Reverse driving (the path's direction flag is -1): the leading axle is the rear axle, and
steering reaches it only through the heading rate, so the forward law diverges. The reverse
law references the rear axle and steers the body heading toward the heading that removes
the error: delta = k_heading * ((psi - h) - atan(k e / (|v| + k_soft))), with h the body
heading stored in the path. See docs/notes for the derivation.

With feedforward on, atan(wheelbase * curvature) is added, using the path's steering
curvature, which has the same sign in both gears.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StanleyParams:
    wheelbase: float = 2.7
    k: float = 1.5          # 1/s, cross-track gain
    k_soft: float = 1.0     # m/s, softening added to the speed
    max_steer: float = 0.6  # rad
    k_heading: float = 1.0  # reverse only: gain on the heading term
    feedforward: bool = False


def _wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class Stanley:
    def __init__(self, path, params=None, search_window=300):
        self.path = path
        self.params = params if params is not None else StanleyParams()
        self.search_window = search_window
        self._index = 0
        self.cross_track = 0.0
        self.heading_error = 0.0

    def reset(self):
        self._index = 0

    def _update_index(self, x, y):
        lo = self._index
        hi = min(lo + self.search_window, len(self.path))
        d2 = (self.path.x[lo:hi] - x) ** 2 + (self.path.y[lo:hi] - y) ** 2
        self._index = lo + int(np.argmin(d2))

    def command(self, state):
        p = self.params
        x, y, psi, v = state[0], state[1], state[2], state[3]
        reverse = self.path.direction[self._index] < 0
        reach = 0.0 if reverse else p.wheelbase
        xr = x + reach * np.cos(psi)
        yr = y + reach * np.sin(psi)
        self._update_index(xr, yr)
        i = self._index
        h = self.path.heading[i]
        e = -(xr - self.path.x[i]) * np.sin(h) + (yr - self.path.y[i]) * np.cos(h)
        psi_e = _wrap(h - psi)
        self.cross_track, self.heading_error = float(e), float(psi_e)
        if reverse:
            delta = p.k_heading * (-psi_e - np.arctan2(p.k * e, abs(v) + p.k_soft))
        else:
            delta = psi_e - np.arctan2(p.k * e, v + p.k_soft)
        if p.feedforward:
            delta += np.arctan(p.wheelbase * self.path.curvature[i])
        return float(np.clip(delta, -p.max_steer, p.max_steer))
