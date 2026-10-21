"""Tracking metrics computed from a logged run.

Every controller is scored by the same code from the same log: sample times, the
tracked point's position, heading, steering angle and speed. The controllers and
simulators are not touched, so a stored run can always be re-scored.

Conventions (see docs/notes/metrics_math_reference.md):
- Lateral error is the signed distance from the tracked point to the nearest point of
  the path polyline (projection onto the segments, not onto the nearest vertex),
  positive to the left of the path direction. The tracked point is the rear axle.
- Heading error is wrap(vehicle heading - path heading at the foot point).
- Steering metrics use only the samples where the vehicle is driving (|speed| above
  v_drive), so steering moved at a standstill (before a cusp) is not counted.
"""
import math
from dataclasses import asdict, dataclass

import numpy as np

from trackloop.projection import project

V_DRIVE = 0.01      # m/s, speed above which a sample counts as driving
DEADBAND = 0.01     # rad/s, steering rates below this are not counted as reversals


def _wrap(a):
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def rear_axle(x_cg, y_cg, psi, lr):
    """Rear-axle position from the centre of gravity (dynamic model), lr behind the CG."""
    return x_cg - lr * np.cos(psi), y_cg - lr * np.sin(psi)


def errors_on_path(path, x, y, psi, window=300):
    """Signed lateral error and heading error of each sample against one path."""
    n = len(x)
    cte, head = np.zeros(n), np.zeros(n)
    index = 0
    for i in range(n):
        p = project(path, x[i], y[i], index, window)
        index = p.index
        cte[i] = p.error
        head[i] = float(_wrap(psi[i] - p.heading))
    return cte, head


def chatter_reversals(rate, deadband=DEADBAND):
    """Number of times the steering rate changes sign, ignoring rates inside the deadband."""
    sig = rate[np.abs(rate) > deadband]
    return int(np.count_nonzero(np.sign(sig[1:]) != np.sign(sig[:-1])))


@dataclass(frozen=True)
class Metrics:
    n_samples: int
    total_time: float            # s, whole run including standstills
    driving_time: float          # s, samples with |speed| > V_DRIVE
    rms_lateral: float           # m
    max_lateral: float           # m, largest |error|
    rms_heading: float           # rad
    max_heading: float           # rad
    steering_effort: float       # rad, RMS of the steering angle while driving
    steering_rate_rms: float     # rad/s, RMS of d delta / dt while driving
    steering_rate_max: float     # rad/s
    chatter_per_s: float         # steering-rate sign reversals per second driving

    def as_dict(self):
        return asdict(self)


def compute(cte, head, t, delta, speed, v_drive=V_DRIVE, deadband=DEADBAND):
    """Metrics from error arrays and the steering log. All arrays have one entry per sample."""
    t, delta, speed = (np.asarray(a, dtype=float) for a in (t, delta, speed))
    dt = np.diff(t)
    rate = np.diff(delta) / dt
    moving = np.abs(speed[1:]) > v_drive
    driving_time = float(dt[moving].sum())
    r = rate[moving]
    on = np.abs(speed) > v_drive
    return Metrics(
        n_samples=len(t),
        total_time=float(t[-1] - t[0]),
        driving_time=driving_time,
        rms_lateral=float(np.sqrt(np.mean(np.square(cte)))),
        max_lateral=float(np.abs(cte).max()),
        rms_heading=float(np.sqrt(np.mean(np.square(head)))),
        max_heading=float(np.abs(head).max()),
        steering_effort=float(np.sqrt(np.mean(delta[on] ** 2))) if on.any() else 0.0,
        steering_rate_rms=float(np.sqrt(np.mean(r ** 2))) if r.size else 0.0,
        steering_rate_max=float(np.abs(r).max()) if r.size else 0.0,
        chatter_per_s=(chatter_reversals(r, deadband) / driving_time) if driving_time > 0 else 0.0,
    )


def score_segments(segment_paths, segment_ids, x, y, psi, t, delta, speed):
    """Score a run made of one path per segment (a path with cusps). Each sample is
    measured against the path of the segment that was active when it was logged."""
    x, y, psi, ids = (np.asarray(a) for a in (x, y, psi, segment_ids))
    cte, head = np.zeros(len(x)), np.zeros(len(x))
    for k, path in enumerate(segment_paths):
        m = np.flatnonzero(ids == k)
        if m.size:
            cte[m], head[m] = errors_on_path(path, x[m], y[m], psi[m])
    return compute(cte, head, t, delta, speed)


def score_cusp_run(path, result):
    """Score a cusp_runner.RunResult against the segments of its path, rear-axle point."""
    from trackloop.segments import split_at_cusps
    s = result.states
    seg_paths = [seg.path for seg in split_at_cusps(path)]
    m = score_segments(seg_paths, result.segment, s[:, 0], s[:, 1], s[:, 2],
                       result.time, s[:, 4], s[:, 3])
    cusp_pos = max((c["position_error"] for c in result.cusps), default=0.0)
    settle = max((c["settle_time"] for c in result.cusps if not math.isnan(c["settle_time"])), default=0.0)
    return m, {"finished": bool(result.finished), "end_position_error": result.end_position_error,
               "end_heading_error": result.end_heading_error,
               "worst_cusp_position_error": float(cusp_pos), "worst_settle_time": float(settle)}
