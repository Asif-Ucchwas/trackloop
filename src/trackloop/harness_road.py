"""Benchmark harness, road family: every dynamic-model controller on every frozen road path.

One run is (road path, controller, seed). The path fixes the design speed (5, 10 or 20 m/s),
so the three speeds are in the paths. The seed fixes a start offset and heading error that
depend on the seed and the path, not on the controller, as in the parking harness. Every run
is scored by trackloop.metrics at the rear axle and also records the tire slip angles and
lateral acceleration it reached, to show whether the linear-tire model stayed in range.
A run that diverges or times out is recorded with its status, never dropped, and a controller
that raises is recorded as an error row.
"""
import json
import math
from functools import lru_cache

import numpy as np

from trackloop import dynamic_runner as dr
from trackloop.controllers.lqr import LQRController
from trackloop.controllers.mpc import MPCController
from trackloop.controllers.pure_pursuit import PurePursuit
from trackloop.controllers.stanley import Stanley, StanleyParams
from trackloop.feasibility import check_path
from trackloop.harness import HEADING_RANGE, OFFSET_RANGE, SEEDS, start_condition
from trackloop.metrics import DEADBAND, V_DRIVE
from trackloop.models.dynamic import DynamicParams
from trackloop.testset import ROAD_DIR, load_test_set

FAMILY = "road"
LR = DynamicParams().lr


def make_pure_pursuit(path):
    return dr.RearAxleAdapter(PurePursuit(path), LR)


def make_stanley(path):
    return dr.RearAxleAdapter(Stanley(path, StanleyParams(feedforward=False)), LR)


def make_lqr(path):
    return LQRController(path)


def make_mpc(path):
    return MPCController(path, dt=dr.DT)


CONTROLLERS = {
    "Pure Pursuit": make_pure_pursuit,
    "Stanley": make_stanley,
    "Dynamic LQR": make_lqr,
    "Dynamic MPC": make_mpc,
}


@lru_cache(maxsize=1)
def _road_set():
    items = load_test_set(ROAD_DIR)
    out = {}
    for i, (name, path) in enumerate(items):
        info = json.loads((ROAD_DIR / f"{name}.json").read_text())
        out[name] = (i, path, info)
    return out


def path_names():
    return list(_road_set())


def run_one(task):
    """task = (path_name, controller, seed). Returns one result row (a dict)."""
    name, controller, seed = task
    index, path, info = _road_set()[name]
    speed = float(info["design_speed_mps"])
    e0, dpsi = start_condition(seed, index)
    feas = check_path(path, speed=speed)
    row = {"family": FAMILY, "path": name, "shape": info["shape"], "controller": controller,
           "speed": speed, "seed": seed, "e0": round(e0, 5), "dpsi0": round(dpsi, 5),
           "drivable": feas.curvature_ok and feas.lateral_ok, "feasible": feas.feasible}
    try:
        res = dr.run(path, CONTROLLERS[controller](path), speed, e0, dpsi)
        m = dr.score_run(path, res)
        af, ar, ay = dr.tire_use(res.states)
        row.update(status=res.status, finished=res.status == "finished", **m.as_dict(),
                   max_slip_front_deg=math.degrees(af), max_slip_rear_deg=math.degrees(ar),
                   max_body_lateral_accel=ay)
    except Exception as exc:    # recorded, not hidden
        row.update(status=f"error: {type(exc).__name__}", finished=False)
    return {k: (round(v, 5) if isinstance(v, float) else v) for k, v in row.items()}


def all_tasks(names=None, controllers=None, seeds=SEEDS):
    names = names if names is not None else path_names()
    controllers = controllers if controllers is not None else list(CONTROLLERS)
    return [(n, c, s) for n in names for c in controllers for s in seeds]


def settings():
    """Everything that fixes the results apart from the code itself."""
    return {"family": FAMILY, "seeds": list(SEEDS), "offset_range_m": OFFSET_RANGE,
            "heading_range_rad": HEADING_RANGE, "controllers": list(CONTROLLERS),
            "paths": path_names(), "plant_dt_s": dr.DT, "arrive_m": dr.ARRIVE,
            "max_error_m": dr.MAX_ERROR, "dynamic_params": DynamicParams().__dict__,
            "metrics_v_drive": V_DRIVE, "metrics_deadband": DEADBAND}


def summarize(rows):
    """Per controller and design speed: runs, finished runs, median RMS lateral error and the
    90th percentile of the maximum error, over the finished runs."""
    out = []
    for c in dict.fromkeys(r["controller"] for r in rows):
        for v in sorted({r["speed"] for r in rows}):
            sel = [r for r in rows if r["controller"] == c and r["speed"] == v]
            done = [r for r in sel if r.get("finished")]
            out.append({"controller": c, "speed": v, "runs": len(sel), "finished": len(done),
                        "median_rms": float(np.median([r["rms_lateral"] for r in done])) if done else math.nan,
                        "p90_max": float(np.percentile([r["max_lateral"] for r in done], 90)) if done else math.nan,
                        "max_slip_deg": max((max(r["max_slip_front_deg"], r["max_slip_rear_deg"]) for r in done),
                                            default=math.nan)})
    return out
