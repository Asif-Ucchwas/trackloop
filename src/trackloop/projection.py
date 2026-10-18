"""Projection of a position onto a path, with interpolated heading and curvature."""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Projection:
    """Foot point of a position on the path, found by projecting onto a segment."""
    index: int          # first vertex of the segment holding the foot point
    t: float            # position along that segment, 0 to 1
    x: float
    y: float
    heading: float      # path heading at the foot point (rad)
    curvature: float    # path curvature at the foot point (1/m)
    s: float            # arc length of the foot point (m)
    error: float        # signed distance, positive when the point is left of the path


def _wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def _on_segment(path, i, x, y):
    ax, ay = path.x[i], path.y[i]
    dx, dy = path.x[i + 1] - ax, path.y[i + 1] - ay
    t = ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)
    t = min(max(t, 0.0), 1.0)
    px, py = ax + t * dx, ay + t * dy
    return (px - x) ** 2 + (py - y) ** 2, t, px, py


def project(path, x, y, start=0, window=None):
    """Project (x, y) onto the polyline. Searches vertices start .. start + window.

    Heading and curvature are interpolated linearly along the segment, and the error
    is the signed distance measured against the interpolated heading.
    """
    hi = len(path) if window is None else min(start + window, len(path))
    d2 = (path.x[start:hi] - x) ** 2 + (path.y[start:hi] - y) ** 2
    nearest = start + int(np.argmin(d2))
    best = None
    for i in (nearest - 1, nearest):
        if 0 <= i < len(path) - 1:
            cand = _on_segment(path, i, x, y)
            if best is None or cand[0] < best[1][0]:
                best = (i, cand)
    i, (_, t, px, py) = best
    heading = path.heading[i] + t * _wrap(path.heading[i + 1] - path.heading[i])
    curvature = path.curvature[i] + t * (path.curvature[i + 1] - path.curvature[i])
    s = path.s[i] + t * (path.s[i + 1] - path.s[i])
    error = -(x - px) * np.sin(heading) + (y - py) * np.cos(heading)
    return Projection(i, float(t), float(px), float(py), float(heading),
                      float(curvature), float(s), float(error))
