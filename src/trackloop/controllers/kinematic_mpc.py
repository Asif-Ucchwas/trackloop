"""Linear MPC on the kinematic error model, for forward and reverse driving, with OSQP.

Error state x = [e, psi_e] (rear-axle offset left of the path, body heading minus path
heading), input u = tan(delta). At a design speed |v| and gear g the signed speed is
v = g |v|, and the model discretised at dt is
    x+ = [[1, v dt], [0, 1]] x + [[v^2 dt^2 / (2 L)], [v dt / L]] (u - L kappa),
written in deviations from the steady turn at the current path curvature kappa, so the
steady-turn steering needs no feedforward. The cost is x^T Q x + R w^2 over the horizon
(w = u - L kappa) with the discrete Riccati solution as terminal weight. Constraints:
|delta| <= max_steer and |change in u| <= steering rate limit * dt. The first step is
also limited by the steering angle the vehicle has now. The QP is built once per gear.
"""
import time
from dataclasses import dataclass

import numpy as np
import osqp
import scipy.sparse as sp

from trackloop.controllers.lqr import bryson_weights
from trackloop.controllers.mpc import discrete_lqr
from trackloop.projection import project


@dataclass(frozen=True)
class KinematicMPCParams:
    wheelbase: float = 2.7
    max_steer: float = 0.6
    max_steer_rate: float = 0.8
    x_max: tuple = (0.5, 0.1)
    u_max: float = 0.2


def model(gear, speed, dt, wheelbase):
    v = gear * speed
    ad = np.array([[1.0, v * dt], [0.0, 1.0]])
    bu = np.array([[v * v * dt * dt / (2.0 * wheelbase)], [v * dt / wheelbase]])
    return ad, bu


def prediction_matrices(ad, bu, n):
    """X = sx x0 + su W for the states x_1 .. x_N and inputs w_0 .. w_{N-1}."""
    sx = np.vstack([np.linalg.matrix_power(ad, k) for k in range(1, n + 1)])
    su = np.zeros((2 * n, n))
    for i in range(n):
        for j in range(i + 1):
            su[2 * i:2 * i + 2, j] = (np.linalg.matrix_power(ad, i - j) @ bu)[:, 0]
    return sx, su


class KinematicMPC:
    def __init__(self, path, params=None, speed=1.5, dt=0.1, horizon=40, search_window=300):
        self.path = path
        self.params = params if params is not None else KinematicMPCParams()
        self.speed, self.dt, self.n = speed, dt, horizon
        self.search_window = search_window
        self.errors = np.zeros(2)
        self._qps = {}
        self.reset()

    def reset(self):
        self.solve_times = []
        self.failures = 0
        self._index = 0
        self._last = 0.0

    def _build(self, gear):
        p, n = self.params, self.n
        ad, bu = model(gear, self.speed, self.dt, p.wheelbase)
        q = np.diag(1.0 / np.asarray(p.x_max, dtype=float) ** 2)
        r = np.array([[1.0 / p.u_max**2]])
        p_term, _ = discrete_lqr(ad, bu, q, r)
        sx, su = prediction_matrices(ad, bu, n)
        qbar = np.kron(np.eye(n), q)
        qbar[-2:, -2:] = p_term
        hess = su.T @ qbar @ su + np.kron(np.eye(n), r)
        diff = sp.diags([-np.ones(n - 1), np.ones(n - 1)], [0, 1], shape=(n - 1, n))
        cons = sp.vstack([sp.identity(n), diff]).tocsc()
        qp = osqp.OSQP()
        qp.setup(sp.csc_matrix(hess), np.zeros(n), cons, np.zeros(2 * n - 1),
                 np.zeros(2 * n - 1), verbose=False, eps_abs=1e-9, eps_rel=1e-9,
                 max_iter=20000, polishing=False)
        self._qps[gear] = (qp, su.T @ qbar @ sx)

    def command(self, state):
        p, n = self.params, self.n
        x, y, psi, delta = float(state[0]), float(state[1]), float(state[2]), float(state[4])
        proj = project(self.path, x, y, self._index, self.search_window)
        self._index = proj.index
        gear = 1 if self.path.direction[proj.index] > 0 else -1
        psi_e = (psi - proj.heading + np.pi) % (2.0 * np.pi) - np.pi
        self.errors = np.array([proj.error, psi_e])
        if gear not in self._qps:
            self._build(gear)
        qp, sq = self._qps[gear]
        lk = p.wheelbase * proj.curvature
        u_lim = np.tan(p.max_steer)
        rate = p.max_steer_rate * self.dt
        u_now = np.tan(delta)
        lows, highs = np.full(n, -u_lim - lk), np.full(n, u_lim - lk)
        lows[0] = max(lows[0], u_now - lk - rate)
        highs[0] = min(highs[0], u_now - lk + rate)
        lower = np.concatenate([lows, np.full(n - 1, -rate)])
        upper = np.concatenate([highs, np.full(n - 1, rate)])
        t0 = time.perf_counter()
        qp.update(q=sq @ self.errors, l=lower, u=upper)
        result = qp.solve(raise_error=False)
        self.solve_times.append(time.perf_counter() - t0)
        if result.info.status != "solved":
            self.failures += 1
        if not np.all(np.isfinite(result.x)):
            return self._last
        self._last = float(np.clip(np.arctan(lk + result.x[0]), -p.max_steer, p.max_steer))
        return self._last
