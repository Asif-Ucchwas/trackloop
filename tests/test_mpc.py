import numpy as np

from trackloop import paths
from trackloop.controllers.lqr import LQRController, bryson_weights, error_model
from trackloop.controllers.mpc import (MPCController, discrete_lqr, discretize,
                                       steady_turn)
from trackloop.models.dynamic import DynamicParams
from trackloop.simulate_dynamic import simulate_dynamic

DT = 0.05
FAST = DynamicParams(max_steer_rate=100.0)
LOOSE = DynamicParams(max_steer=10.0, max_steer_rate=1000.0)


def test_zero_order_hold_model_matches_its_series_for_a_short_period():
    dt = 1e-4
    a, b, _ = error_model(10.0)
    ad, bu = discretize(10.0, dt)
    series_a = np.eye(4) + a * dt + a @ a * dt**2 / 2 + a @ a @ a * dt**3 / 6
    series_b = (b * dt + a @ b * dt**2 / 2 + a @ a @ b * dt**3 / 6)
    assert np.allclose(ad, series_a, atol=1e-8)
    assert np.allclose(bu, series_b, atol=1e-8)


def test_steady_turn_solves_the_error_model():
    for v, kappa in ((10.0, 0.02), (20.0, 0.01), (5.0, 0.05), (30.0, -0.004)):
        a, b, bd = error_model(v)
        delta, x_ss = steady_turn(v, kappa)
        residual = a @ x_ss + b[:, 0] * delta + bd[:, 0] * kappa * v
        assert np.abs(residual).max() < 1e-12


def test_hand_calculated_steady_turn():
    delta, x_ss = steady_turn(10.0, 0.02)
    assert np.isclose(delta, 0.0615)
    assert np.allclose(x_ss, [0.0, 0.0, -0.016667, 0.0], atol=1e-6)


def _state_near_the_arc():
    path = paths.arc(50.0, 1.0)
    i = 200
    state = [path.x[i] + 0.05, path.y[i] + 0.3, path.heading[i] + 0.04,
             10.0, 0.1, 0.2, 0.01]
    return path, state


def test_unconstrained_mpc_equals_discrete_lqr_for_any_horizon():
    q, r = bryson_weights()
    ad, bu = discretize(10.0, DT)
    kd = discrete_lqr(ad, bu, q, r)[1]
    path, state = _state_near_the_arc()
    delta_ss, x_ss = steady_turn(10.0, 0.02)
    for horizon in (3, 20):
        c = MPCController(path, params=LOOSE, dt=DT, horizon=horizon)
        delta = c.command(state)
        expected = delta_ss - (kd @ (c.errors - x_ss))[0]
        assert abs(delta - expected) < 1e-7


def test_error_state_matches_the_lqr_controller():
    path, state = _state_near_the_arc()
    mpc, lqr = MPCController(path, params=LOOSE), LQRController(path, params=LOOSE)
    mpc.command(state)
    lqr.command(state)
    assert np.allclose(mpc.errors, lqr.errors, atol=1e-12)


def test_rate_limit_binds_for_a_large_error():
    path = paths.straight(100.0)
    c = MPCController(path, dt=DT)
    assert np.isclose(c.command([5.0, 2.0, 0.0, 10.0, 0.0, 0.0, 0.0]), -0.04, atol=1e-6)
    assert np.isclose(c.command([5.0, 2.0, 0.0, 10.0, 0.0, 0.0, 0.5]), 0.46, atol=1e-6)


def test_rate_limit_also_applies_when_the_plan_unwinds_the_steering():
    c = MPCController(paths.straight(100.0), dt=DT)
    delta = c.command([5.0, 5.0, 0.0, 10.0, 0.0, 0.0, -0.59])
    assert np.isclose(delta, -0.55, atol=1e-6)


def test_straight_line_response_matches_the_discrete_closed_loop_prediction():
    q, r = bryson_weights()
    ad, bu = discretize(10.0, DT)
    kd = discrete_lqr(ad, bu, q, r)[1]
    path = paths.straight(120.0)
    c = MPCController(path, params=FAST, dt=DT)
    _, cte = simulate_dynamic(c, path, [0.0, 0.2, 0.0, 10.0, 0.0, 0.0, 0.0], DT, 6.0,
                              params=FAST)
    x, predicted = np.array([0.2, 0.0, 0.0, 0.0]), []
    for _ in cte:
        predicted.append(x[0])
        x = (ad - bu @ kd) @ x
    assert np.max(np.abs(cte - np.array(predicted))) < 2e-4
    assert c.failures == 0


def test_circle_has_a_small_steady_error_without_feedforward_terms():
    path = paths.arc(50.0, 6.0)
    c = MPCController(path, params=FAST, dt=DT)
    _, cte = simulate_dynamic(c, path, [0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0], DT, 25.0,
                              params=FAST)
    assert abs(cte[-80:].mean()) < 5e-4 and np.ptp(cte[-80:]) < 1e-5


def _large_offset(controller, y0=5.0):
    path = paths.straight(300.0)
    controller.path = path
    traj, cte = simulate_dynamic(controller, path, [0.0, y0, 0.0, 10.0, 0.0, 0.0, 0.0],
                                 DT, 12.0)
    return traj, cte


def test_mpc_respects_the_rate_limit_and_does_not_overshoot_a_large_offset():
    c = MPCController(paths.straight(300.0), dt=DT)
    traj, cte = _large_offset(c)
    assert np.abs(np.diff(traj[:, 6])).max() <= 0.8 * DT + 1e-9
    assert cte.min() > -0.01
    assert np.abs(cte[int(5.0 / DT):]).max() < 0.05
    assert c.failures == 0


def test_lqr_with_the_same_actuator_limits_overshoots_the_large_offset():
    traj, cte = _large_offset(LQRController(paths.straight(300.0)))
    assert cte.min() < -1.0


def test_solve_times_are_logged_once_per_command():
    path = paths.straight(120.0)
    c = MPCController(path, dt=DT)
    simulate_dynamic(c, path, [0.0, 0.5, 0.0, 10.0, 0.0, 0.0, 0.0], DT, 2.0)
    assert len(c.solve_times) == 40 and min(c.solve_times) > 0.0
