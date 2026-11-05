"""Controllers designed on an identified model (Task 24).

For each Stage 4 model-error case the plant is the true vehicle. A steering-sweep experiment on that plant (the Task 21 data,
15 m/s, noisy yaw rate and lateral acceleration) is fitted with the Task 22 estimator, starting from the nominal model and
assuming the nominal mass. The LQR and MPC are then designed on one of three models and run on the true plant:

  nominal     the nominal model (the Stage 4 "wrong model" result)
  identified  the model fitted to the experiment on the true plant
  oracle      the true plant's own parameters (the best a redesign can do; not available in practice)

Pure Pursuit and Stanley are geometric: they use no tire or mass parameters, so their results do not depend on the design
model and are not re-run (their Stage 4 results stand for all three).
"""
import math
from dataclasses import replace
from functools import lru_cache

from trackloop import dynamic_runner as dr
from trackloop import identification as idn
from trackloop import sysid_data as sd
from trackloop.controllers.lqr import LQRController
from trackloop.controllers.mpc import MPCController
from trackloop.harness import start_condition
from trackloop.harness_road import FAMILY, _road_set
from trackloop.feasibility import check_path
from trackloop.model_error import dynamic_plant
from trackloop.models.dynamic import DynamicParams

VARIANTS = ("nominal", "identified", "oracle")
MODEL_BASED = ("Dynamic LQR", "Dynamic MPC")
EXPERIMENT = sd.Settings(kind="sweep", vx=15.0, duration=40.0, seed=21)


@lru_cache(maxsize=None)
def identified(case):
    """(DynamicParams fitted to a sweep experiment on the plant of `case`, the fit dict). Estimates cf, cr and Iz from the
    nominal starting point with the nominal mass assumed; everything else stays nominal."""
    plant = dynamic_plant(case)
    fit = idn.fit(sd.generate(EXPERIMENT, plant), base=idn.NOMINAL, start=(1.0, 1.0, 1.0))
    cf, cr, iz = fit["theta"]
    return replace(idn.NOMINAL, cf=float(cf), cr=float(cr), inertia=float(iz)), fit


def design_params(variant, case):
    if variant == "nominal":
        return DynamicParams()
    if variant == "identified":
        return identified(case)[0]
    if variant == "oracle":
        return dynamic_plant(case)
    raise ValueError(f"variant must be one of {VARIANTS}")


def make_controller(controller, path, params):
    if controller == "Dynamic LQR":
        return LQRController(path, params=params)
    if controller == "Dynamic MPC":
        return MPCController(path, params=params, dt=dr.DT)
    raise ValueError(f"{controller} does not depend on the design model")


def run_one(task):
    """task = (path name, controller, seed, case, variant). Same run as harness_road.run_one for that case, with the
    controller designed on the variant's model. Returns a result row."""
    name, controller, seed, case, variant = task
    plant = dynamic_plant(case)
    index, path, info = _road_set()[name]
    speed = float(info["design_speed_mps"])
    e0, dpsi = start_condition(seed, index)
    row = {"family": FAMILY, "path": name, "controller": controller, "speed": speed, "seed": seed, "case": case,
           "variant": variant, "e0": round(e0, 5), "dpsi0": round(dpsi, 5)}
    try:
        res = dr.run(path, make_controller(controller, path, design_params(variant, case)), speed, e0, dpsi, params=plant)
        m = dr.score_run(path, res, plant)
        row.update(status=res.status, finished=res.status == "finished", **m.as_dict())
    except Exception as exc:    # recorded, not hidden
        row.update(status=f"error: {type(exc).__name__}", finished=False)
    return {k: (round(v, 5) if isinstance(v, float) else v) for k, v in row.items()}


def all_tasks(cases, controllers=MODEL_BASED, variants=("identified", "oracle"), seeds=(0, 1, 2, 3, 4)):
    return [(n, c, s, k, v) for k in cases for v in variants for n in _road_set() for c in controllers for s in seeds]


def paired_ratio(rows, other, metric="rms_lateral", key=("path", "controller", "seed", "speed")):
    """Median over paired runs of metric(rows) / metric(other); both runs must have finished."""
    import numpy as np

    ref = {tuple(r[k] for k in key): r for r in other if r.get("finished")}
    vals = [r[metric] / ref[tuple(r[k] for k in key)][metric] for r in rows
            if r.get("finished") and tuple(r[k] for k in key) in ref and ref[tuple(r[k] for k in key)][metric] > 0]
    return float(np.median(vals)) if vals else math.nan


def load_stage4(path="data/results/model_error_road.csv"):
    """The committed Stage 4 road model-error runs (controllers designed on the nominal model), numbers and flags converted."""
    import csv

    rows = []
    for r in csv.DictReader(open(path)):
        r["finished"] = r["finished"] == "True"
        r["seed"], r["speed"] = int(r["seed"]), float(r["speed"])
        for k in ("rms_lateral", "steering_effort"):
            r[k] = float(r[k]) if r[k] else math.nan
        rows.append(r)
    return rows
