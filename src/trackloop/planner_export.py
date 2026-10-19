"""Turn a planner polyline into a drivable reference path in the common format.

The thesis planners return waypoints as (x, y) in grid cells. This module scales them
to metres, rounds every corner with a circular arc, and samples the result by arc
length. Heading and curvature are exact for the rounded path (lines and arcs), not
estimated by finite differences. Heading is continuous (not wrapped). Paths are
forward only (direction = +1).
"""
import json
import math
from pathlib import Path as FilePath

import numpy as np

from trackloop.paths import Path


def _unit(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = math.hypot(dx, dy)
    return (dx / n, dy / n), n


def _clean(points, cell):
    pts = [(float(points[0][0]) * cell, float(points[0][1]) * cell)]
    for p in points[1:]:
        q = (float(p[0]) * cell, float(p[1]) * cell)
        if math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1]) > 1e-9:
            pts.append(q)
    return pts


def _pieces(pts, radius):
    """Lines and arcs of the rounded path, with one stats tuple per corner."""
    pieces, stats, cur, s0 = [], [], pts[0], 0.0

    def line(a, b):
        nonlocal s0
        n = math.hypot(b[0] - a[0], b[1] - a[1])
        if n > 1e-12:
            heading = math.atan2(b[1] - a[1], b[0] - a[0])
            pieces.append(("line", s0, n, a[0], a[1], heading))
            s0 += n

    for i in range(1, len(pts) - 1):
        u1, l1 = _unit(pts[i - 1], pts[i])
        u2, l2 = _unit(pts[i], pts[i + 1])
        cross = u1[0] * u2[1] - u1[1] * u2[0]
        theta = math.atan2(abs(cross), u1[0] * u2[0] + u1[1] * u2[1])
        if theta < 1e-9:
            continue
        t_max = 0.5 * min(l1, l2)
        tight = radius * math.tan(theta / 2.0) > t_max
        r = t_max / math.tan(theta / 2.0) if tight else radius
        t, sg = r * math.tan(theta / 2.0), math.copysign(1.0, cross)
        a = (pts[i][0] - u1[0] * t, pts[i][1] - u1[1] * t)
        line(cur, a)
        h0 = math.atan2(u1[1], u1[0])
        c = (a[0] - sg * r * u1[1], a[1] + sg * r * u1[0])
        pieces.append(("arc", s0, r * theta, c[0], c[1], h0, sg, r))
        s0 += r * theta
        cur = (pts[i][0] + u2[0] * t, pts[i][1] + u2[1] * t)
        stats.append((r, theta, tight))
    line(cur, pts[-1])
    return pieces, stats


def _sample(pieces, s):
    x, y, h, k = (np.empty_like(s) for _ in range(4))
    starts = np.array([p[1] for p in pieces])
    idx = np.clip(np.searchsorted(starts, s, side="right") - 1, 0, len(pieces) - 1)
    for j, p in enumerate(pieces):
        m = idx == j
        d = s[m] - p[1]
        if p[0] == "line":
            x[m] = p[3] + d * math.cos(p[5])
            y[m] = p[4] + d * math.sin(p[5])
            h[m], k[m] = p[5], 0.0
        else:
            sg, r = p[6], p[7]
            hh = p[5] + sg * d / r
            x[m] = p[3] + sg * r * np.sin(hh)
            y[m] = p[4] - sg * r * np.cos(hh)
            h[m], k[m] = hh, sg / r
    return x, y, h, k


def export_path(points, cell=3.0, radius=6.0, ds=0.1):
    """Return (Path, info) for a polyline of (x, y) in cells. cell and radius in m."""
    pts = _clean(points, cell)
    if len(pts) < 2:
        raise ValueError("need at least two distinct points")
    pieces, stats = _pieces(pts, radius)
    total = pieces[-1][1] + pieces[-1][2]
    s = np.arange(0.0, total, ds)
    if total - s[-1] > 1e-9:
        s = np.append(s, total)
    x, y, h, k = _sample(pieces, s)
    info = {
        "cell_m": cell, "radius_m": radius, "ds_m": ds, "length_m": float(total),
        "n_vertices": len(pts), "n_corners": len(stats),
        "n_tight": sum(1 for _, _, tight in stats if tight),
        "min_radius_m": min((r for r, _, _ in stats), default=None),
        "max_cut_m": max((r * (1 / math.cos(t / 2) - 1) for r, t, _ in stats),
                         default=0.0),
        "max_curvature": float(np.max(np.abs(k))),
    }
    return Path(x, y, h, k, s, np.ones(len(s))), info


def save_path(path, info, stem):
    stem = FilePath(stem)
    stem.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(stem.with_suffix(".npz"), x=path.x, y=path.y,
                        heading=path.heading, curvature=path.curvature, s=path.s,
                        direction=path.direction)
    text = json.dumps(info, indent=2, sort_keys=True) + "\n"
    stem.with_suffix(".json").write_text(text)


def load_path(stem):
    d = np.load(FilePath(stem).with_suffix(".npz"))
    keys = ("x", "y", "heading", "curvature", "s", "direction")
    return Path(*(d[key] for key in keys))
