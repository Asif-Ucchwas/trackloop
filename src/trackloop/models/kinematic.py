"""Kinematic bicycle model with the rear axle as the reference point.

State layout: [x, y, heading, speed, steering_angle]
Inputs: steering command (rad) and acceleration (m/s^2).
Negative speed (reverse driving) is supported without changes.
"""
from dataclasses import dataclass

import numpy as np

X, Y, PSI, V, DELTA = range(5)


@dataclass(frozen=True)
class KinematicParams:
    wheelbase: float = 2.7          # m, placeholder value
    max_steer: float = 0.6          # rad, steering angle limit, placeholder value
    max_steer_rate: float = 0.8     # rad/s, steering rate limit, placeholder value


def _derivatives(q, delta, accel, params):
    """Time derivative of [x, y, heading, speed] for a fixed steering angle."""
    _, _, psi, v = q
    return np.array([
        v * np.cos(psi),
        v * np.sin(psi),
        v * np.tan(delta) / params.wheelbase,
        accel,
    ])


def step(state, steer_cmd, accel, dt, params=KinematicParams()):
    """Advance one time step: steering actuator first, then RK4 on pose and speed."""
    state = np.asarray(state, dtype=float)
    max_change = params.max_steer_rate * dt
    delta = state[DELTA] + np.clip(steer_cmd - state[DELTA], -max_change, max_change)
    delta = float(np.clip(delta, -params.max_steer, params.max_steer))

    q = state[:4]
    k1 = _derivatives(q, delta, accel, params)
    k2 = _derivatives(q + 0.5 * dt * k1, delta, accel, params)
    k3 = _derivatives(q + 0.5 * dt * k2, delta, accel, params)
    k4 = _derivatives(q + dt * k3, delta, accel, params)
    q_next = q + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    return np.append(q_next, delta)


def rollout(state0, steer_cmd, accel, dt, n_steps, params=KinematicParams()):
    """Simulate n_steps with constant inputs. Returns an array of shape (n_steps + 1, 5)."""
    traj = np.empty((n_steps + 1, 5))
    traj[0] = state0
    for k in range(n_steps):
        traj[k + 1] = step(traj[k], steer_cmd, accel, dt, params)
    return traj
