"""Model-error cases (Task 15): the plant differs from the model the controllers were designed on.

Controllers are always built on nominal parameters (DynamicParams() or the controllers' own
defaults). A case changes only the plant the run is simulated on, so the controller does not
know about the difference. The sizes below are placeholders chosen to be clearly larger than
estimation noise and clearly smaller than a different vehicle; they are not measured values.

Dynamic plant (road family): mass, tire cornering stiffness, steering lag.
  mass +-20 percent, with the yaw inertia scaled by the same factor (same radius of gyration)
  cornering stiffness +-30 percent on both axles together
  steering actuator lag, a first-order time constant of 0.1 s and 0.2 s
  combined: mass +20 percent, stiffness -30 percent and lag 0.1 s together

Kinematic plant (parking family): there is no mass or tire in the model, so only
  steering actuator lag (0.1 s, 0.2 s) and wheelbase +-5 percent (the one geometric parameter).
"""
import math
from dataclasses import replace

import numpy as np

from trackloop.models.dynamic import DynamicParams
from trackloop.models.kinematic import KinematicParams

DYNAMIC_CASES = {
    "nominal": {},
    "mass_p20": {"mass": 1.2, "inertia": 1.2},
    "mass_m20": {"mass": 0.8, "inertia": 0.8},
    "stiff_p30": {"cf": 1.3, "cr": 1.3},
    "stiff_m30": {"cf": 0.7, "cr": 0.7},
    "lag_0.1": {"steer_lag": 0.1},
    "lag_0.2": {"steer_lag": 0.2},
    "combined": {"mass": 1.2, "inertia": 1.2, "cf": 0.7, "cr": 0.7, "steer_lag": 0.1},
}
KINEMATIC_CASES = {
    "nominal": {},
    "lag_0.1": {"steer_lag": 0.1},
    "lag_0.2": {"steer_lag": 0.2},
    "wheelbase_p5": {"wheelbase": 1.05},
    "wheelbase_m5": {"wheelbase": 0.95},
}
LAG_FIELDS = ("steer_lag",)     # absolute values; every other field is a scale factor


def _apply(nominal, changes):
    out = {}
    for field, value in changes.items():
        out[field] = value if field in LAG_FIELDS else getattr(nominal, field) * value
    return replace(nominal, **out)


def dynamic_plant(case):
    """DynamicParams of the plant for a case; None or "nominal" gives the nominal plant."""
    return _apply(DynamicParams(), DYNAMIC_CASES[case or "nominal"])


def kinematic_plant(case):
    """KinematicParams of the plant for a case; None or "nominal" gives the nominal plant."""
    return _apply(KinematicParams(), KINEMATIC_CASES[case or "nominal"])


def summarize(rows, key_fields=("path", "controller", "seed")):
    """Per controller, speed and case: runs, finished runs, median RMS lateral error, and the
    median paired change against the nominal case (same path, controller, seed, speed).
    "ratio" is the median of case RMS divided by nominal RMS. Only pairs where both runs
    finished are used for the paired columns."""
    def key(r):
        return tuple(r[k] for k in key_fields) + (r["speed"],)

    nominal = {key(r): r for r in rows if r["case"] == "nominal" and r.get("finished")}
    out = []
    for c in dict.fromkeys(r["controller"] for r in rows):
        for v in sorted({r["speed"] for r in rows}):
            for k in dict.fromkeys(r["case"] for r in rows):
                sel = [r for r in rows if r["controller"] == c and r["speed"] == v and r["case"] == k]
                done = [r for r in sel if r.get("finished")]
                pairs = [(r, nominal[key(r)]) for r in done if key(r) in nominal]
                out.append({
                    "controller": c, "speed": v, "case": k, "runs": len(sel), "finished": len(done),
                    "median_rms": float(np.median([r["rms_lateral"] for r in done])) if done else math.nan,
                    "paired_change": float(np.median([a["rms_lateral"] - b["rms_lateral"] for a, b in pairs]))
                    if pairs else math.nan,
                    "ratio": float(np.median([a["rms_lateral"] / b["rms_lateral"] for a, b in pairs
                                              if b["rms_lateral"] > 0])) if pairs else math.nan})
    return out
