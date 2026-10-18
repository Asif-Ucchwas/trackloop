"""Reference paths in the common format used by the controllers.

Arrays of equal length: x, y position (m), heading (rad), curvature (1/m, positive
for a left turn), s arc length from the first point (m), direction (+1 forward,
-1 reverse). Cross-track error is positive when the vehicle is left of the path.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Path:
    x: np.ndarray
    y: np.ndarray
    heading: np.ndarray
    curvature: np.ndarray
    s: np.ndarray
    direction: np.ndarray

    def __len__(self):
        return len(self.x)


def straight_then_arc(straight_length, radius, angle, ds=0.1):
    """A straight segment along +x, then an arc. Positive angle turns left."""
    sign = 1.0 if angle >= 0 else -1.0
    total = straight_length + radius * abs(angle)
    n = int(round(total / ds)) + 1
    s = np.linspace(0.0, total, n)
    theta = np.clip(s - straight_length, 0.0, None) / radius
    on_arc = s > straight_length
    x = np.where(on_arc, straight_length + radius * np.sin(theta), s)
    y = np.where(on_arc, sign * radius * (1.0 - np.cos(theta)), 0.0)
    heading = sign * theta
    curvature = np.where(on_arc, sign / radius, 0.0)
    return Path(x, y, heading, curvature, s, np.ones(n))


def straight(length, ds=0.1):
    return straight_then_arc(length, 1.0, 0.0, ds)


def arc(radius, angle, ds=0.1):
    return straight_then_arc(0.0, radius, angle, ds)


def nearest_index(path, x, y):
    return int(np.argmin((path.x - x) ** 2 + (path.y - y) ** 2))


def cross_track_error(path, x, y):
    i = nearest_index(path, x, y)
    h = path.heading[i]
    return float(-(x - path.x[i]) * np.sin(h) + (y - path.y[i]) * np.cos(h))
