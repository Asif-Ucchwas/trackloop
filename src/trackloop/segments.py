"""Split a path at its cusps into segments that each have one gear."""
from dataclasses import dataclass

import numpy as np

from trackloop.paths import Path


@dataclass(frozen=True)
class Segment:
    path: Path      # one gear only; arc length starts at 0
    gear: int       # +1 forward, -1 reverse
    s_start: float  # arc length of its first sample on the full path
    first: int      # index of its first sample on the full path


def split_at_cusps(path):
    """Segments in driving order. Neighbours share the cusp sample.

    The cusp sample belongs to both segments, so each one is tracked up to the cusp.
    In the full path that sample carries the next piece's curvature, so the segment
    that ends there takes the curvature of the sample before it.
    """
    d = np.asarray(path.direction)
    if not np.all(np.abs(d) == 1.0):
        raise ValueError("direction must be +1 or -1 on every sample")
    cusps = np.flatnonzero(d[1:] != d[:-1]) + 1
    bounds = [0, *[int(c) for c in cusps], len(path) - 1]
    out = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        sl = slice(a, b + 1)
        k = np.array(path.curvature[sl])
        if b != len(path) - 1:
            k[-1] = k[-2]
        gear = int(d[a])
        out.append(Segment(
            Path(np.array(path.x[sl]), np.array(path.y[sl]), np.array(path.heading[sl]),
                 k, path.s[sl] - path.s[a], np.full(b - a + 1, float(gear))),
            gear, float(path.s[a]), a))
    return out
