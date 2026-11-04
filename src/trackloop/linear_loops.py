"""Linear closed-loop models of the four road controllers around straight-line driving (Task 17).

Error state x = [e1, e1_dot, e2, e2_dot] of the LQR error model (lqr.py): lateral error of the
centre of gravity, its rate, heading error (vehicle minus path) and its rate. On a straight path
every controller is a fixed state feedback delta_cmd = -K x, with K depending on the speed:

  Pure Pursuit  delta = atan(2 L sin(alpha) / ld), small angles, goal point ld ahead of the rear
                axle on the path: K = [2L/ld^2, 0, (2L/ld)(1 - lr/ld), 0], ld = lookahead(v).
  Stanley       delta = psi_e - atan(k e_front / (v + k_soft)), small angles, e_front = e1 + lf e2:
                K = [s, 0, 1 + s lf, 0] with s = k / (v + k_soft).
  Dynamic LQR   K from the controller's gain schedule: Riccati gains on a speed grid, interpolated,
                held constant above 30 m/s (equal to the Riccati gain on the grid, e.g. at 10 m/s).
  Dynamic MPC   with no constraint active and the Riccati terminal cost, the first move equals the
                infinite-horizon discrete LQR gain at the control period dt (discrete_lqr).

The steering actuator is a first-order lag with time constant `lag` seconds (0 means none), as in
models/dynamic.py. Limits (steering angle, steering rate) are not in a linear model.
"""
import control as ct
import numpy as np

from trackloop.controllers.lqr import GainSchedule, bryson_weights, error_model
from trackloop.controllers.mpc import MIN_SPEED, discrete_lqr, discretize
from trackloop.controllers.pure_pursuit import PurePursuitParams
from trackloop.controllers.stanley import StanleyParams
from trackloop.models.dynamic import DynamicParams

CONTROLLERS = ("Pure Pursuit", "Stanley", "Dynamic LQR", "Dynamic MPC")
DT = 0.05   # s, control period of the dynamic runner


_SCHEDULES = {}


def _schedule(p):
    key = repr(p)
    if key not in _SCHEDULES:
        _SCHEDULES[key] = GainSchedule(p=p)
    return _SCHEDULES[key]


def feedback_gain(controller, vx, params=None, dt=DT):
    """Row K (shape (4,)) with delta_cmd = -K x at speed vx, as the controller computes it.

    Dynamic LQR uses its gain schedule (Riccati gains on a speed grid, interpolated, held constant
    above the grid's top speed, 30 m/s), not the exact Riccati gain at vx.
    """
    p = params if params is not None else DynamicParams()
    if controller == "Pure Pursuit":
        pp = PurePursuitParams(wheelbase=p.wheelbase)
        ld = pp.lookahead(vx)
        return np.array([2 * p.wheelbase / ld**2, 0.0, 2 * p.wheelbase / ld * (1.0 - p.lr / ld), 0.0])
    if controller == "Stanley":
        sp = StanleyParams(wheelbase=p.wheelbase)
        s = sp.k / (vx + sp.k_soft)
        return np.array([s, 0.0, 1.0 + s * p.lf, 0.0])
    if controller == "Dynamic LQR":
        return _schedule(p).gain(vx)
    if controller == "Dynamic MPC":
        ad, bu = discretize(max(vx, MIN_SPEED), dt, p)
        q, r = bryson_weights()
        return discrete_lqr(ad, bu, q, r)[1][0]
    raise ValueError(f"unknown controller {controller!r}")


def closed_loop(controller, vx, lag=0.0, dt=None, params=None, plant_params=None):
    """Closed loop as a python-control StateSpace; input: additive steering disturbance (rad),
    outputs: all states. States are x, plus the steering angle when lag > 0.

    dt=None gives a continuous-time model (not available for the MPC, a sampled controller).
    With dt the model is the exact discrete model of the simulation step: the actuator lag
    becomes delta+ = a delta + (1 - a) u with a = exp(-dt / lag), and the plant holds delta+
    over the step. `params` are the parameters the controller is designed on; `plant_params`
    (default: the same) are the plant's.
    """
    design = params if params is not None else DynamicParams()
    plant = plant_params if plant_params is not None else design
    if controller == "Dynamic MPC" and dt is None:
        raise ValueError("the MPC is a sampled controller: give dt")
    k = feedback_gain(controller, vx, design, dt if dt is not None else DT)
    n = 4 if lag <= 0.0 else 5
    if dt is None:
        a, b, _ = error_model(vx, plant)
        if lag <= 0.0:
            return ct.StateSpace(a - b @ k[None, :], b, np.eye(4), np.zeros((4, 1)))
        a_cl = np.zeros((5, 5))
        a_cl[:4, :4], a_cl[:4, 4:] = a, b
        a_cl[4, :4], a_cl[4, 4] = -k / lag, -1.0 / lag
        b_cl = np.zeros((5, 1))
        b_cl[4, 0] = 1.0 / lag
        return ct.StateSpace(a_cl, b_cl, np.eye(5), np.zeros((5, 1)))
    ad, bu = discretize(vx, dt, plant)
    if lag <= 0.0:
        return ct.StateSpace(ad - bu @ k[None, :], bu, np.eye(4), np.zeros((4, 1)), dt)
    g = np.exp(-dt / lag)
    a_cl = np.zeros((5, 5))
    a_cl[:4, :4] = ad - (1.0 - g) * bu @ k[None, :]
    a_cl[:4, 4:] = g * bu
    a_cl[4, :4], a_cl[4, 4] = -(1.0 - g) * k, g
    b_cl = np.zeros((5, 1))
    b_cl[:4, 0] = (1.0 - g) * bu[:, 0]
    b_cl[4, 0] = 1.0 - g
    return ct.StateSpace(a_cl, b_cl, np.eye(5), np.zeros((5, 1)), dt)


def is_stable(system):
    """True when every pole is inside the stability region (left half plane, or unit disc if sampled)."""
    poles = np.linalg.eigvals(system.A)
    return bool(np.all(np.abs(poles) < 1.0)) if system.dt else bool(np.all(poles.real < 0.0))


def open_loop(controller, vx, lag=0.0, dt=DT, params=None, plant_params=None):
    """Sampled loop transfer function L(z), broken at the steering command (Task 18).

    Input: the commanded steering angle delta_cmd (after the controller, before the actuator lag);
    output: the controller's feedback value K x, so that delta_cmd = -L delta_cmd closes the loop.
    The closed loop of `closed_loop(..., dt)` is A_ol - B_ol C_ol exactly. Break point and sign are
    a choice: margins computed here are margins to a gain or phase change at the steering command.
    """
    design = params if params is not None else DynamicParams()
    plant = plant_params if plant_params is not None else design
    k = feedback_gain(controller, vx, design, dt)
    ad, bu = discretize(vx, dt, plant)
    if lag <= 0.0:
        return ct.StateSpace(ad, bu, k[None, :], np.zeros((1, 1)), dt)
    g = np.exp(-dt / lag)
    a = np.zeros((5, 5))
    a[:4, :4], a[:4, 4:], a[4, 4] = ad, g * bu, g
    b = np.zeros((5, 1))
    b[:4, 0], b[4, 0] = (1.0 - g) * bu[:, 0], 1.0 - g
    c = np.zeros((1, 5))
    c[0, :4] = k
    return ct.StateSpace(a, b, c, np.zeros((1, 1)), dt)
