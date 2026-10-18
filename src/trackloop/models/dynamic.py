"""Dynamic bicycle model with linear tires (lateral dynamics).

State layout: x, y, psi, vx, vy, r, delta
  x, y   position of the center of gravity, world frame (m)
  psi    heading (rad)
  vx, vy body-frame longitudinal and lateral velocity (m/s)
  r      yaw rate (rad/s)
  delta  front steering angle (rad)

Simplifications: accel is the body-x acceleration (the vy*r coupling is
dropped), and slip angles use max(vx, v_min) so they stay finite near zero
speed. Below roughly v_min the model is not trusted.
"""
from dataclasses import dataclass

import numpy as np

X, Y, PSI, VX, VY, R, DELTA = range(7)


@dataclass(frozen=True)
class DynamicParams:
    mass: float = 1500.0
    inertia: float = 2500.0
    lf: float = 1.2
    lr: float = 1.5
    cf: float = 80000.0
    cr: float = 100000.0
    max_steer: float = 0.6
    max_steer_rate: float = 0.8
    v_min: float = 1.0

    @property
    def wheelbase(self):
        return self.lf + self.lr

    @property
    def understeer_gradient(self):
        return self.mass / self.wheelbase * (self.lr / self.cf - self.lf / self.cr)


def _derivatives(q, delta, accel, params):
    x, y, psi, vx, vy, r = q
    vx_eff = max(vx, params.v_min)
    alpha_f = delta - np.arctan2(vy + params.lf * r, vx_eff)
    alpha_r = -np.arctan2(vy - params.lr * r, vx_eff)
    fyf = params.cf * alpha_f
    fyr = params.cr * alpha_r
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        vx * c - vy * s,
        vx * s + vy * c,
        r,
        accel,
        (fyf * np.cos(delta) + fyr) / params.mass - vx * r,
        (params.lf * fyf * np.cos(delta) - params.lr * fyr) / params.inertia,
    ])


def step(state, steer_cmd, accel, dt, params=DynamicParams()):
    state = np.asarray(state, dtype=float)
    delta = state[DELTA]
    max_change = params.max_steer_rate * dt
    delta = delta + np.clip(steer_cmd - delta, -max_change, max_change)
    delta = np.clip(delta, -params.max_steer, params.max_steer)

    q = state[:DELTA]
    k1 = _derivatives(q, delta, accel, params)
    k2 = _derivatives(q + 0.5 * dt * k1, delta, accel, params)
    k3 = _derivatives(q + 0.5 * dt * k2, delta, accel, params)
    k4 = _derivatives(q + dt * k3, delta, accel, params)
    q_next = q + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    return np.append(q_next, delta)


def rollout(state0, steer_cmd, accel, dt, n_steps, params=DynamicParams()):
    out = np.zeros((n_steps + 1, 7))
    out[0] = state0
    for i in range(n_steps):
        out[i + 1] = step(out[i], steer_cmd, accel, dt, params)
    return out
