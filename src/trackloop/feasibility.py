"""Can the car follow a reference path? Curvature, steering-rate and lateral-acceleration checks.

Curvature: the tightest turn is kappa_max = tan(max_steer) / L. A sample beyond it cannot be
driven as drawn, at any speed.

Steering rate: the steering angle that holds the path's curvature is delta_ref = atan(L kappa).
Between two samples ds apart it changes by d. At speed v the car has ds / v seconds, in which
the steering can move by max_steer_rate * ds / v, so the step is followable when
|d| <= max_steer_rate * ds / v. Neighbouring steps that are not followable form one event. An
event with total change D needs D / max_steer_rate seconds, which is v * D / max_steer_rate
metres of driving with the steering behind the path. Steps across a cusp are not counted: the
car is stopped there and the steering settles before the next segment starts.

Lateral acceleration: driving curvature kappa at speed v needs a = v^2 |kappa|. Tires hold
about mu * g, so a path is drivable at speed v only if v^2 max|kappa| <= mu * g, which gives
the largest speed v_max = sqrt(mu * g / max|kappa|). mu = 0.8 is a placeholder.
"""
import math
from dataclasses import asdict, dataclass

import numpy as np

from trackloop.models.kinematic import KinematicParams
from trackloop.segments import split_at_cusps

G = 9.81
MU = 0.8


@dataclass(frozen=True)
class Feasibility:
    curvature_limit: float       # 1/m
    max_abs_curvature: float     # 1/m
    samples_over_limit: int
    max_excess: float            # 1/m beyond the limit, 0 if none
    curvature_ok: bool
    speed: float                 # m/s the rate and lateral checks were made for
    steering_events: int         # runs of steps the steering cannot follow at this speed
    largest_event_rad: float     # total steering change of the largest event
    worst_lag_distance: float    # m driven with the steering behind, for the largest event
    rate_ok: bool
    lateral_accel_limit: float   # m/s^2, mu * g
    max_lateral_accel: float     # m/s^2 at this speed
    max_feasible_speed: float    # m/s, inf for a straight path
    lateral_ok: bool

    @property
    def feasible(self):
        return self.curvature_ok and self.rate_ok and self.lateral_ok

    def as_dict(self):
        return {**asdict(self), "feasible": self.feasible}


def curvature_limit(limits=None):
    limits = limits if limits is not None else KinematicParams()
    return float(np.tan(limits.max_steer) / limits.wheelbase)


def max_speed_for_curvature(kappa, mu=MU):
    """Largest speed at which curvature kappa needs no more than mu * g of lateral acceleration."""
    return math.inf if kappa == 0.0 else math.sqrt(mu * G / abs(kappa))


def _event_sizes(delta, ds, allowed):
    """Total steering change of each run of consecutive steps larger than allowed."""
    step = np.abs(np.diff(delta))
    bad = step > allowed * (1.0 + 1e-9)
    sizes, i, n = [], 0, len(bad)
    while i < n:
        if not bad[i]:
            i += 1
            continue
        total = 0.0
        while i < n and bad[i]:
            total += step[i]
            i += 1
        sizes.append(total)
    return sizes


def check_path(path, limits=None, speed=1.5, mu=MU):
    if speed <= 0.0:
        raise ValueError("speed must be positive")
    limits = limits if limits is not None else KinematicParams()
    kmax = curvature_limit(limits)
    k = np.abs(np.asarray(path.curvature))
    over = k > kmax
    sizes = []
    for seg in split_at_cusps(path):
        delta = np.arctan(limits.wheelbase * np.asarray(seg.path.curvature))
        ds = np.diff(np.asarray(seg.path.s))
        sizes += _event_sizes(delta, ds, limits.max_steer_rate * ds / speed)
    largest = max(sizes, default=0.0)
    lat = float(speed * speed * k.max())
    return Feasibility(
        curvature_limit=kmax,
        max_abs_curvature=float(k.max()),
        samples_over_limit=int(over.sum()),
        max_excess=float(max(k.max() - kmax, 0.0)),
        curvature_ok=not bool(over.any()),
        speed=float(speed),
        steering_events=len(sizes),
        largest_event_rad=float(largest),
        worst_lag_distance=float(speed * largest / limits.max_steer_rate),
        rate_ok=len(sizes) == 0,
        lateral_accel_limit=float(mu * G),
        max_lateral_accel=lat,
        max_feasible_speed=float(max_speed_for_curvature(float(k.max()), mu)),
        lateral_ok=lat <= mu * G * (1.0 + 1e-12),
    )
