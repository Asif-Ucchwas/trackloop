"""Linear MPC for lateral path tracking on the LQR error model, solved with OSQP.

The model is the error model of lqr.py discretised with a zero-order hold at the
control period dt and the current speed. It is written in deviations from the steady
turn at the path curvature: x_tilde = x - x_ss, u_tilde = delta - delta_ss, so the
steady-turn steering needs no feedforward. The cost is the sum of x^T Q x + u^T R u
over the horizon plus a terminal cost x^T P x, with P from the discrete Riccati
equation. Constraints: |delta| <= max_steer and |delta change| <= rate limit * dt.
"""
import time

import numpy as np
import osqp
import scipy.sparse as sp
from scipy.linalg import expm, solve_discrete_are

from trackloop.controllers.lqr import bryson_weights, error_model
from trackloop.models.dynamic import DynamicParams
from trackloop.projection import project

MIN_SPEED = 2.0


def discretize(vx, dt, p=None):
    """Zero-order-hold model x+ = Ad x + Bu delta of the error model at speed vx."""
    a, b, _ = error_model(vx, p)
    m = np.zeros((5, 5))
    m[:4, :4], m[:4, 4:] = a * dt, b * dt
    e = expm(m)
    return e[:4, :4], e[:4, 4:]


def steady_turn(vx, curvature, p=None):
    """Steering delta_ss and error state x_ss of a steady turn at the curvature."""
    p = p if p is not None else DynamicParams()
    delta = (p.wheelbase + p.understeer_gradient * vx**2) * curvature
    beta = (p.lr - p.lf * p.mass * vx**2 / (p.cr * p.wheelbase)) * curvature
    return delta, np.array([0.0, 0.0, -beta, 0.0])


def discrete_lqr(ad, bu, q, r):
    """Terminal weight P and gain K of the infinite-horizon discrete problem."""
    p = solve_discrete_are(ad, bu, q, r)
    k = np.linalg.solve(r + bu.T @ p @ bu, bu.T @ p @ ad)
    return p, k


def prediction_matrices(ad, bu, n):
    """X = sx x0 + su U for the states x_1 .. x_N and inputs u_0 .. u_{N-1}."""
    sx = np.vstack([np.linalg.matrix_power(ad, k) for k in range(1, n + 1)])
    su = np.zeros((4 * n, n))
    for i in range(n):
        for j in range(i + 1):
            su[4 * i:4 * i + 4, j] = (np.linalg.matrix_power(ad, i - j) @ bu)[:, 0]
    return sx, su


class MPCController:
    def __init__(self, path, params=None, dt=0.05, horizon=20, q=None, r=None,
                 search_window=300):
        self.path = path
        self.params = params if params is not None else DynamicParams()
        self.dt, self.n = dt, horizon
        self.q, self.r = (q, r) if q is not None else bryson_weights()
        self.search_window = search_window
        self.errors = np.zeros(4)
        self.reset()

    def reset(self):
        self.solve_times = []      # seconds per QP update + solve
        self.failures = 0          # solves that did not report "solved"
        self._index = 0
        self._speed = None
        self._qp = None
        self._last = 0.0

    def _build(self, vx):
        ad, bu = discretize(vx, self.dt, self.params)
        p_term, _ = discrete_lqr(ad, bu, self.q, self.r)
        sx, su = prediction_matrices(ad, bu, self.n)
        qbar = np.kron(np.eye(self.n), self.q)
        qbar[-4:, -4:] = p_term
        hess = su.T @ qbar @ su + np.kron(np.eye(self.n), self.r)
        self._sq = su.T @ qbar @ sx
        diff = sp.diags([-np.ones(self.n - 1), np.ones(self.n - 1)], [0, 1],
                        shape=(self.n - 1, self.n))
        cons = sp.vstack([sp.identity(self.n), diff]).tocsc()
        lo, hi = np.zeros(2 * self.n - 1), np.zeros(2 * self.n - 1)
        qp = osqp.OSQP()
        qp.setup(sp.csc_matrix(hess), np.zeros(self.n), cons, lo, hi, verbose=False,
                 eps_abs=1e-9, eps_rel=1e-9, max_iter=20000, polishing=False)
        self._qp, self._speed = qp, vx

    def command(self, state):
        p = self.params
        x, y, psi, vx, vy, r = (float(v) for v in state[:6])
        proj = project(self.path, x, y, self._index, self.search_window)
        self._index = proj.index
        kappa, e1 = proj.curvature, proj.error
        e2 = (psi - proj.heading + np.pi) % (2.0 * np.pi) - np.pi
        e1_dot = vx * np.sin(e2) + vy * np.cos(e2)
        psi_dot_des = kappa * (vx * np.cos(e2) - vy * np.sin(e2)) / (1.0 - kappa * e1)
        self.errors = np.array([e1, e1_dot, e2, r - psi_dot_des])

        v_model = max(vx, MIN_SPEED)
        if self._qp is None or abs(v_model - self._speed) > 0.05:
            self._build(v_model)
        delta_ss, x_ss = steady_turn(v_model, kappa, p)
        rate = p.max_steer_rate * self.dt
        lows = np.full(self.n, -p.max_steer - delta_ss)
        highs = np.full(self.n, p.max_steer - delta_ss)
        lows[0] = max(lows[0], state[6] - delta_ss - rate)
        highs[0] = min(highs[0], state[6] - delta_ss + rate)
        lower = np.concatenate([lows, np.full(self.n - 1, -rate)])
        upper = np.concatenate([highs, np.full(self.n - 1, rate)])
        t0 = time.perf_counter()
        self._qp.update(q=self._sq @ (self.errors - x_ss), l=lower, u=upper)
        result = self._qp.solve(raise_error=False)
        self.solve_times.append(time.perf_counter() - t0)
        if result.info.status != "solved":
            self.failures += 1
        if not np.all(np.isfinite(result.x)):
            return self._last
        self._last = float(np.clip(delta_ss + result.x[0], -p.max_steer, p.max_steer))
        return self._last
