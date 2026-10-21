"""Benchmark harness, parking family: every kinematic controller on every frozen path.

One run is (path, controller, speed, seed). The seed fixes the start offset, which depends
on the seed and the path but not on the controller, so controllers are compared from the
same starting condition. Every run is scored by trackloop.metrics and written as one row.
Paths that fail a feasibility check at the run's speed are run anyway and flagged, never
dropped. "drivable" means the hard limits hold (curvature and lateral acceleration); the
steering-rate check is reported as a lag distance, since it fails on most paths at any speed.
A controller that raises is recorded as an error row, never skipped.
"""
import math
from dataclasses import asdict
from functools import lru_cache

import numpy as np

from trackloop.controllers.kinematic_lqr import KinematicLQR
from trackloop.controllers.kinematic_mpc import KinematicMPC
from trackloop.controllers.pure_pursuit import PurePursuit
from trackloop.controllers.stanley import Stanley, StanleyParams
from trackloop.cusp_runner import RunnerParams, track_with_cusps
from trackloop.feasibility import G, MU, check_path
from trackloop.metrics import DEADBAND, V_DRIVE, score_cusp_run
from trackloop.models.kinematic import KinematicParams
from trackloop.segments import split_at_cusps
from trackloop.testset import load_test_set

SPEEDS = (0.5, 1.5, 3.0)        # m/s, parking family
SEEDS = (0, 1, 2, 3, 4)
OFFSET_RANGE = 0.3              # m, start lateral offset uniform in +-
HEADING_RANGE = 0.1             # rad, start heading error uniform in +-
FAMILY = "parking"


def make_pure_pursuit(path):
    return PurePursuit(path)


def make_stanley(path):
    return Stanley(path, StanleyParams(feedforward=bool(path.direction[0] < 0)))


def make_lqr(path):
    return KinematicLQR(path)


def make_mpc(path):
    return KinematicMPC(path)


CONTROLLERS = {
    "Pure Pursuit": make_pure_pursuit,
    "Stanley": make_stanley,
    "Kinematic LQR": make_lqr,
    "Kinematic MPC": make_mpc,
}


@lru_cache(maxsize=1)
def _test_set():
    items = load_test_set()
    return {name: (i, path) for i, (name, path) in enumerate(items)}


def path_names():
    return list(_test_set())


def start_condition(seed, path_index):
    """Lateral offset (m, left positive) and heading error (rad) at the start."""
    rng = np.random.default_rng([seed, path_index])
    return (float(rng.uniform(-OFFSET_RANGE, OFFSET_RANGE)),
            float(rng.uniform(-HEADING_RANGE, HEADING_RANGE)))


def start_state(path, e0, dpsi):
    h = float(path.heading[0])
    return [float(path.x[0]) - e0 * math.sin(h), float(path.y[0]) + e0 * math.cos(h),
            h + dpsi, 0.0, 0.0]


def max_time(path, speed):
    """Time limit that scales with the path: three times the drive time plus allowances."""
    n_seg = len(split_at_cusps(path))
    return 3.0 * float(path.s[-1]) / speed + 60.0 + 20.0 * n_seg


def _reasons(f):
    out = []
    if not f.curvature_ok:
        out.append("curvature")
    if not f.rate_ok:
        out.append("steering_rate")
    if not f.lateral_ok:
        out.append("lateral_accel")
    return "+".join(out)


def run_one(task):
    """task = (path_name, controller, speed, seed). Returns one result row (a dict)."""
    name, controller, speed, seed = task
    index, path = _test_set()[name]
    e0, dpsi = start_condition(seed, index)
    feas = check_path(path, speed=speed)
    row = {"family": FAMILY, "path": name, "controller": controller, "speed": speed, "seed": seed,
           "e0": round(e0, 5), "dpsi0": round(dpsi, 5),
           "drivable": feas.curvature_ok and feas.lateral_ok, "feasible": feas.feasible,
           "infeasible_reasons": _reasons(feas), "steering_events": feas.steering_events,
           "lag_distance_m": round(feas.worst_lag_distance, 4),
           "max_lateral_accel": round(feas.max_lateral_accel, 3)}
    params = RunnerParams(speed=speed, max_time=max_time(path, speed))
    try:
        res = track_with_cusps(path, CONTROLLERS[controller], start_state(path, e0, dpsi), params=params)
        m, extra = score_cusp_run(path, res)
        row.update(status="ok", **m.as_dict(), **extra)
    except Exception as exc:    # recorded, not hidden
        row.update(status=f"error: {type(exc).__name__}", finished=False)
    return {k: (round(v, 5) if isinstance(v, float) else v) for k, v in row.items()}


def all_tasks(names=None, controllers=None, speeds=SPEEDS, seeds=SEEDS):
    names = names if names is not None else path_names()
    controllers = controllers if controllers is not None else list(CONTROLLERS)
    return [(n, c, v, s) for n in names for c in controllers for v in speeds for s in seeds]


def settings():
    """Everything that fixes the results apart from the code itself."""
    return {"family": FAMILY, "speeds": list(SPEEDS), "seeds": list(SEEDS),
            "offset_range_m": OFFSET_RANGE, "heading_range_rad": HEADING_RANGE,
            "controllers": list(CONTROLLERS), "paths": path_names(),
            "runner": asdict(RunnerParams()), "vehicle": asdict(KinematicParams()),
            "friction_mu": MU, "gravity": G, "metrics_v_drive": V_DRIVE, "metrics_deadband": DEADBAND}


def summarize(rows):
    """Per controller and speed: runs, finished share, median RMS error, 90th percentile of max
    error. Medians and percentiles are over the finished runs, for all paths and for the paths
    that are drivable (curvature and lateral acceleration within limits) at that speed."""
    out = []
    for scope in ("all", "drivable"):
        for c in dict.fromkeys(r["controller"] for r in rows):
            for v in sorted({r["speed"] for r in rows}):
                sel = [r for r in rows if r["controller"] == c and r["speed"] == v
                       and (scope == "all" or r["drivable"])]
                done = [r for r in sel if r.get("finished")]
                out.append({"scope": scope, "controller": c, "speed": v, "runs": len(sel),
                            "finished": len(done),
                            "median_rms": float(np.median([r["rms_lateral"] for r in done])) if done else math.nan,
                            "p90_max": float(np.percentile([r["max_lateral"] for r in done], 90)) if done else math.nan})
    return out
