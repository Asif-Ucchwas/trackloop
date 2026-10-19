"""LQR for the kinematic error model, written per metre driven instead of per second.

Error state x = [e, psi_e]: rear-axle offset left of the path (m) and body heading minus
path heading (rad). With gear g = +1 forward and -1 reverse, and u = tan(delta), the
derivatives per metre driven are
    de/ds       = g sin(psi_e)
    dpsi_e/ds   = g (u / L - kappa)
where kappa is the path's steering curvature. Linearised at zero error:
    dx/ds = g A x + g B (u - L kappa),   A = [[0, 1], [0, 0]],   B = [0, 1/L].
The speed does not appear, so one gain per gear holds at any speed down to a standstill.
The command is delta = atan(L kappa - K_g x). Reverse gear flips the sign of the heading
gain: K_reverse = [k_e, -k_psi] where [k_e, k_psi] is the forward gain.
"""
from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_continuous_are

from trackloop.projection import project

A0 = np.array([[0.0, 1.0], [0.0, 0.0]])


def error_model(gear, wheelbase=2.7):
    """A, B of dx/ds = A x + B (u - L kappa) for gear +1 or -1."""
    return gear * A0, gear * np.array([[0.0], [1.0 / wheelbase]])


def kinematic_gain(gear, wheelbase=2.7, x_max=(0.5, 0.1), u_max=0.2):
    """LQR gain (length 2) with Bryson weights Q = diag(1 / x_max^2), R = 1 / u_max^2."""
    q = np.diag(1.0 / np.asarray(x_max, dtype=float) ** 2)
    r = np.array([[1.0 / u_max**2]])
    a, b = error_model(gear, wheelbase)
    riccati = solve_continuous_are(a, b, q, r)
    return np.linalg.solve(r, b.T @ riccati)[0]


@dataclass(frozen=True)
class KinematicLQRParams:
    wheelbase: float = 2.7
    max_steer: float = 0.6
    x_max: tuple = (0.5, 0.1)
    u_max: float = 0.2
    feedforward: bool = True


class KinematicLQR:
    def __init__(self, path, params=None, search_window=300):
        self.path = path
        self.params = params if params is not None else KinematicLQRParams()
        p = self.params
        self.gains = {g: kinematic_gain(g, p.wheelbase, p.x_max, p.u_max) for g in (1, -1)}
        self.search_window = search_window
        self._index = 0
        self.errors = np.zeros(2)

    def reset(self):
        self._index = 0

    def command(self, state):
        p = self.params
        x, y, psi = float(state[0]), float(state[1]), float(state[2])
        proj = project(self.path, x, y, self._index, self.search_window)
        self._index = proj.index
        gear = 1 if self.path.direction[proj.index] > 0 else -1
        psi_e = (psi - proj.heading + np.pi) % (2.0 * np.pi) - np.pi
        self.errors = np.array([proj.error, psi_e])
        u = -self.gains[gear] @ self.errors
        if p.feedforward:
            u += p.wheelbase * proj.curvature
        return float(np.clip(np.arctan(u), -p.max_steer, p.max_steer))
