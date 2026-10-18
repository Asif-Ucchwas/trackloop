"""Matched-turn comparison of the kinematic and dynamic models.

Both models get the same constant steering angle, chosen so the kinematic model
has lateral acceleration ay. The dynamic model's center of gravity is converted
to the rear axle so both paths describe the same point on the car.
"""
import numpy as np

from trackloop.models import dynamic as dyn
from trackloop.models import kinematic as kin

DT = 0.01


def matched_turn(v, ay=2.0, duration=5.0, kp=None, dp=None):
    kp = kp if kp is not None else kin.KinematicParams()
    dp = dp if dp is not None else dyn.DynamicParams()
    assert kp.wheelbase == dp.wheelbase
    assert kp.max_steer == dp.max_steer and kp.max_steer_rate == dp.max_steer_rate
    wheelbase = dp.wheelbase

    delta = np.arctan(wheelbase * ay / v**2)
    if delta > dp.max_steer:
        return None
    n = int(round(duration / DT))
    k0 = np.array([0.0, 0.0, 0.0, v, 0.0])
    d0 = np.array([dp.lr, 0.0, 0.0, v, 0.0, 0.0, 0.0])
    kt = kin.rollout(k0, delta, 0.0, DT, n, kp)
    dt = dyn.rollout(d0, delta, 0.0, DT, n, dp)
    psi = dt[:, dyn.PSI]
    rear_x = dt[:, dyn.X] - dp.lr * np.cos(psi)
    rear_y = dt[:, dyn.Y] - dp.lr * np.sin(psi)
    err = np.hypot(kt[-1, kin.X] - rear_x[-1], kt[-1, kin.Y] - rear_y[-1])
    r_kin = v * np.tan(kt[-1, kin.DELTA]) / wheelbase
    r_dyn = dt[-1, dyn.R]
    k_us = dp.understeer_gradient
    return {
        "delta": delta,
        "ratio": r_dyn / r_kin,
        "formula": wheelbase / (wheelbase + k_us * v**2),
        "dpsi": kt[-1, kin.PSI] - dt[-1, dyn.PSI],
        "err": err,
        "beta": np.degrees(np.arctan2(dt[-1, dyn.VY], dt[-1, dyn.VX])),
    }
