"""Stanley path tracking for the rear-axle kinematic model (forward driving).

The law references the front axle: delta = psi_e - atan(k e / (v + k_soft)), where
e is the front axle cross-track error (positive left of the path) and psi_e is the
path heading minus the vehicle heading at the path point nearest the front axle.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StanleyParams:
    wheelbase: float = 2.7
    k: float = 1.5          # 1/s, cross-track gain
    k_soft: float = 1.0     # m/s, softening added to the speed
    max_steer: float = 0.6  # rad


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
        xf = x + p.wheelbase * np.cos(psi)
        yf = y + p.wheelbase * np.sin(psi)
        self._update_index(xf, yf)
        i = self._index
        h = self.path.heading[i]
        e = -(xf - self.path.x[i]) * np.sin(h) + (yf - self.path.y[i]) * np.cos(h)
        psi_e = (h - psi + np.pi) % (2.0 * np.pi) - np.pi
        self.cross_track, self.heading_error = float(e), float(psi_e)
        delta = psi_e - np.arctan2(p.k * e, v + p.k_soft)
        return float(np.clip(delta, -p.max_steer, p.max_steer))
