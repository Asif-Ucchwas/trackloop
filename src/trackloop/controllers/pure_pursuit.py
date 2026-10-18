"""Pure Pursuit path tracking for the rear-axle kinematic model (forward driving).

The steering angle makes the vehicle follow the circle that passes through the rear
axle, is tangent to the heading, and reaches a goal point on the path a lookahead
distance ahead: delta = atan(2 L sin(alpha) / ld), where alpha is the angle between
the heading and the line to the goal point.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PurePursuitParams:
    wheelbase: float = 2.7
    ld_min: float = 2.0     # m, lookahead at zero speed
    k_v: float = 0.5        # s, lookahead growth per unit of speed
    ld_max: float = 30.0    # m

    def lookahead(self, v):
        return float(np.clip(self.ld_min + self.k_v * abs(v), self.ld_min, self.ld_max))


class PurePursuit:
    def __init__(self, path, params=None, search_window=300):
        self.path = path
        self.params = params if params is not None else PurePursuitParams()
        self.search_window = search_window
        self._index = 0
        self.goal = (path.x[0], path.y[0])

    def reset(self):
        self._index = 0

    def _update_index(self, x, y):
        lo = self._index
        hi = min(lo + self.search_window, len(self.path))
        d2 = (self.path.x[lo:hi] - x) ** 2 + (self.path.y[lo:hi] - y) ** 2
        self._index = lo + int(np.argmin(d2))

    def lookahead_point(self, x, y, ld):
        """First path point at distance ld from (x, y), ahead of the nearest point."""
        self._update_index(x, y)
        p = self.path
        i = self._index
        far = np.nonzero(np.hypot(p.x[i:] - x, p.y[i:] - y) >= ld)[0]
        if len(far) == 0:
            return float(p.x[-1]), float(p.y[-1])
        j = i + int(far[0])
        if j == i:
            return float(p.x[j]), float(p.y[j])
        ax, ay = p.x[j - 1], p.y[j - 1]
        dx, dy = p.x[j] - ax, p.y[j] - ay
        fx, fy = ax - x, ay - y
        a = dx * dx + dy * dy
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - ld * ld
        t = (-b + np.sqrt(max(b * b - 4.0 * a * c, 0.0))) / (2.0 * a)
        t = float(np.clip(t, 0.0, 1.0))
        return float(ax + t * dx), float(ay + t * dy)

    def command(self, state):
        x, y, psi, v = state[0], state[1], state[2], state[3]
        ld = self.params.lookahead(v)
        self.goal = self.lookahead_point(x, y, ld)
        alpha = np.arctan2(self.goal[1] - y, self.goal[0] - x) - psi
        alpha = (alpha + np.pi) % (2.0 * np.pi) - np.pi
        return float(np.arctan2(2.0 * self.params.wheelbase * np.sin(alpha), ld))
