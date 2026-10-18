"""LQR design for lateral path tracking: error model, Riccati solution, gain schedule.

Error state x = [e1, e1_dot, e2, e2_dot]: lateral position error of the center of
gravity (m, positive left of the path), its rate (m/s), heading error (rad, vehicle
heading minus path heading) and its rate (rad/s). Input u = front steering angle.
The model is linear for constant speed vx and small errors. psi_dot_des is the
heading rate of the path (curvature times speed).
"""
import numpy as np
from scipy.linalg import solve_continuous_are

from trackloop.models.dynamic import DynamicParams
from trackloop.projection import project

DEFAULT_X_MAX = (0.5, 1.0, 0.1, 0.3)   # acceptable size of each error state
DEFAULT_U_MAX = 0.2                    # acceptable steering angle (rad)


def bryson_weights(x_max=DEFAULT_X_MAX, u_max=DEFAULT_U_MAX):
    """Q = diag(1 / x_max^2), R = 1 / u_max^2."""
    q = np.diag(1.0 / np.asarray(x_max, dtype=float) ** 2)
    return q, np.array([[1.0 / u_max**2]])


def error_model(vx, p=None):
    """Matrices A, B, Bd of x_dot = A x + B delta + Bd psi_dot_des at speed vx."""
    p = p if p is not None else DynamicParams()
    m, iz, lf, lr, cf, cr = p.mass, p.inertia, p.lf, p.lr, p.cf, p.cr
    a = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, -(cf + cr) / (m * vx), (cf + cr) / m, (-cf * lf + cr * lr) / (m * vx)],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, (-lf * cf + lr * cr) / (iz * vx), (lf * cf - lr * cr) / iz,
         -(lf**2 * cf + lr**2 * cr) / (iz * vx)],
    ])
    b = np.array([[0.0], [cf / m], [0.0], [lf * cf / iz]])
    bd = np.array([[0.0], [(-cf * lf + cr * lr) / (m * vx) - vx], [0.0],
                   [-(lf**2 * cf + lr**2 * cr) / (iz * vx)]])
    return a, b, bd


def lqr_gain(vx, q=None, r=None, p=None):
    """Returns the gain K (1 x 4), the Riccati solution P and the model (A, B)."""
    if q is None or r is None:
        q, r = bryson_weights()
    a, b, _ = error_model(vx, p)
    riccati = solve_continuous_are(a, b, q, r)
    gain = np.linalg.solve(r, b.T @ riccati)
    return gain, riccati, a, b


def default_speeds():
    """Speed grid for the schedule: fine below 6 m/s, where the gains change fastest."""
    return np.concatenate([np.arange(2.0, 6.0, 0.25), np.arange(6.0, 30.01, 1.0)])


class GainSchedule:
    """Gains computed on a grid of speeds and interpolated linearly in between."""

    def __init__(self, speeds=None, q=None, r=None, p=None):
        self.speeds = np.asarray(speeds if speeds is not None else default_speeds())
        self.table = np.array([lqr_gain(v, q, r, p)[0][0] for v in self.speeds])

    def gain(self, vx):
        """Gain row at speed vx, held constant outside the grid."""
        return np.array([np.interp(vx, self.speeds, self.table[:, i])
                         for i in range(self.table.shape[1])])


class LQRController:
    """Scheduled LQR steering for the dynamic model, with curvature feedforward.

    State layout is that of the dynamic model. Errors are measured at the center of
    gravity against the foot point of the path. The command is
    delta = -K(vx) x + delta_ff, with delta_ff the steering that holds a steady turn
    at the path curvature with zero error.
    """

    def __init__(self, path, schedule=None, params=None, feedforward=True,
                 search_window=300):
        self.path = path
        self.params = params if params is not None else DynamicParams()
        self.schedule = schedule if schedule is not None else GainSchedule(p=self.params)
        self.feedforward = feedforward
        self.search_window = search_window
        self._index = 0
        self.errors = np.zeros(4)
        self.steer_ff = 0.0

    def reset(self):
        self._index = 0

    def feedforward_steer(self, vx, curvature):
        """Steady-turn steering minus k3 times the steady heading error."""
        p = self.params
        k3 = self.schedule.gain(vx)[2]
        beta = (p.lr - p.lf * p.mass * vx**2 / (p.cr * p.wheelbase)) * curvature
        steady = (p.wheelbase + p.understeer_gradient * vx**2) * curvature
        return steady - k3 * beta

    def command(self, state):
        x, y, psi, vx, vy, r = (float(v) for v in state[:6])
        proj = project(self.path, x, y, self._index, self.search_window)
        self._index = proj.index
        kappa = proj.curvature
        e1 = proj.error
        e2 = (psi - proj.heading + np.pi) % (2.0 * np.pi) - np.pi
        e1_dot = vx * np.sin(e2) + vy * np.cos(e2)
        psi_dot_des = kappa * (vx * np.cos(e2) - vy * np.sin(e2)) / (1.0 - kappa * e1)
        e2_dot = r - psi_dot_des
        self.errors = np.array([e1, e1_dot, e2, e2_dot])
        self.steer_ff = self.feedforward_steer(vx, kappa) if self.feedforward else 0.0
        delta = -self.schedule.gain(vx) @ self.errors + self.steer_ff
        return float(np.clip(delta, -self.params.max_steer, self.params.max_steer))
