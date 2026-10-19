"""Turn a Reeds-Shepp segment list into the common TrackLoop path format.

Input: a start pose (x, y, heading) in metres and radians, and a list of
(turn, signed_length) segments in units of the turning radius, as MultiPlan's
RSPath.segments stores them. turn is 'L', 'R' or 'S'. A negative length means
reverse gear. For L and R the length is the angle turned (radians); for S it is
the distance divided by the radius.

Conventions of the output Path:
  heading    body heading (where the car points), continuous, not wrapped
  curvature  steering curvature: +1/r for L, -1/r for R, 0 for S, in both gears
  direction  +1 forward, -1 reverse: the motion that leaves each sample
  so d(heading)/d(arc length) = curvature * direction
"""
import math

import numpy as np

from trackloop.paths import Path

_TOL = 1e-9


def _pieces(start, segments, radius):
    """Pieces as arrays: start arc length, pose at start, gear, signed curvature."""
    x, y, h = start
    s = 0.0
    rows = []
    for turn, length in segments:
        if turn not in ("L", "R", "S"):
            raise ValueError(f"unknown segment type {turn!r}")
        real = abs(length) * radius
        if real <= _TOL:
            continue
        gear = 1.0 if length > 0 else -1.0
        k = {"L": 1.0 / radius, "R": -1.0 / radius, "S": 0.0}[turn]
        rows.append((s, x, y, h, gear, k))
        if k == 0.0:
            x += gear * real * math.cos(h)
            y += gear * real * math.sin(h)
        else:
            h_end = h + gear * k * real
            x += (math.sin(h_end) - math.sin(h)) / k
            y -= (math.cos(h_end) - math.cos(h)) / k
            h = h_end
        s += real
    if not rows:
        raise ValueError("path has no length")
    return np.array(rows), s, (x, y, h)


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rs_to_path(start, segments, radius, ds=0.1, goal=None):
    """Sample the segments every ds metres. Returns (Path, info).

    Every gear change is sampled exactly: the cusp sample sits on the cusp and
    carries the new direction. If goal is given, info reports how far the
    computed end pose is from it.
    """
    rows, total, end = _pieces(start, segments, radius)
    starts = rows[:, 0]
    bounds = starts[1:]
    grid = np.arange(0.0, total, ds)
    if len(bounds):
        near = np.min(np.abs(grid[:, None] - bounds[None, :]), axis=1) <= _TOL
        grid = grid[~near]
    s = np.unique(np.concatenate([grid, bounds, [total]]))
    idx = np.clip(np.searchsorted(starts, s, side="right") - 1, 0, len(rows) - 1)
    _, x0, y0, h0, gear, k = rows[idx].T
    t = s - starts[idx]
    straight = k == 0.0
    kk = np.where(straight, 1.0, k)
    h = np.where(straight, h0, h0 + gear * k * t)
    x = np.where(straight, x0 + gear * t * np.cos(h0), x0 + (np.sin(h) - np.sin(h0)) / kk)
    y = np.where(straight, y0 + gear * t * np.sin(h0), y0 - (np.cos(h) - np.cos(h0)) / kk)
    cusps = np.flatnonzero(rows[1:, 4] != rows[:-1, 4]) + 1
    info = {"radius_m": radius, "ds_m": ds, "length_m": float(total),
            "n_segments": len(rows), "n_cusps": len(cusps),
            "cusp_s_m": [float(starts[c]) for c in cusps],
            "max_curvature": float(np.max(np.abs(k)))}
    if goal is not None:
        info["end_position_error_m"] = float(math.hypot(end[0] - goal[0], end[1] - goal[1]))
        info["end_heading_error_rad"] = float(abs(_wrap(end[2] - goal[2])))
    return Path(x, y, h, k, s, gear), info
