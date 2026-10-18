import numpy as np
from scipy.linalg import expm

from trackloop import paths
from trackloop.controllers.lqr import LQRController, lqr_gain
from trackloop.models.dynamic import DynamicParams
from trackloop.simulate_dynamic import simulate_dynamic

FAST = DynamicParams(max_steer_rate=100.0)
L = 2.7


def test_hand_calculated_steering_for_a_pure_cross_track_error():
    c = LQRController(paths.straight(100.0), params=FAST)
    assert np.isclose(c.command([5.0, 0.5, 0.0, 10.0, 0.0, 0.0, 0.0]), -0.2)


def test_heading_error_is_assembled_from_the_state_and_steers_back():
    c = LQRController(paths.straight(100.0), params=FAST)
    delta = c.command([5.0, 0.0, 0.1, 10.0, 0.0, 0.0, 0.0])
    expected_errors = [0.0, 10.0 * np.sin(0.1), 0.1, 0.0]
    assert np.allclose(c.errors, expected_errors)
    assert np.isclose(delta, -lqr_gain(10.0)[0][0] @ expected_errors)
    assert delta < 0


def test_steering_is_limited():
    c = LQRController(paths.straight(100.0), params=FAST)
    assert c.command([5.0, 5.0, 0.0, 10.0, 0.0, 0.0, 0.0]) == -0.6
    assert c.command([5.0, -5.0, 0.0, 10.0, 0.0, 0.0, 0.0]) == 0.6


def test_feedforward_alone_gives_the_steady_turn_steering_at_steady_state():
    radius, vx = 50.0, 10.0
    path = paths.arc(radius, 1.0)
    c = LQRController(path, params=FAST)
    kappa = 1.0 / radius
    beta = (1.5 - 1.2 * 1500.0 * vx**2 / (100000.0 * L)) * kappa
    assert np.isclose(beta, 0.016667, atol=1e-6)
    i = 100                          # a vertex on the arc, so the error is exactly 0
    psi = path.heading[i] - beta
    vy = vx * np.tan(beta)
    r = kappa * (vx * np.cos(-beta) - vy * np.sin(-beta))
    delta = c.command([path.x[i], path.y[i], psi, vx, vy, r, 0.0])
    assert np.allclose(c.errors, [0.0, 0.0, -beta, 0.0], atol=1e-9)
    assert np.isclose(delta, (L + 0.00375 * vx**2) * kappa)
    assert np.isclose(delta, 0.0615)


def test_straight_line_response_matches_the_linear_closed_loop_prediction():
    vx, y0, dt = 10.0, 0.2, 0.01
    path = paths.straight(120.0)
    traj, cte = simulate_dynamic(LQRController(path, params=FAST), path,
                                 [0.0, y0, 0.0, vx, 0.0, 0.0, 0.0], dt, 6.0, params=FAST)
    k, _, a, b = lqr_gain(vx)
    step = expm((a - b @ k) * dt)
    x = np.array([y0, 0.0, 0.0, 0.0])
    predicted = np.zeros(len(cte))
    for i in range(len(cte)):
        predicted[i] = x[0]
        x = step @ x
    assert np.max(np.abs(cte - predicted)) < 2e-3
    assert abs(cte[-1]) < 2e-3


def _circle_error(radius, vx, feedforward):
    path = paths.arc(radius, 6.0)
    c = LQRController(path, params=FAST, feedforward=feedforward)
    _, cte = simulate_dynamic(c, path, [0.0, 0.0, 0.0, vx, 0.0, 0.0, 0.0], 0.01, 25.0,
                              params=FAST)
    return cte[-400:].mean(), np.ptp(cte[-400:])


def _predicted_error(radius, vx):
    k = lqr_gain(vx)[0][0]
    kappa = 1.0 / radius
    steady = (L + 0.00375 * vx**2) * kappa
    beta = (1.5 - 1.2 * 1500.0 * vx**2 / (100000.0 * L)) * kappa
    return -(steady - k[2] * beta) / k[0]


def test_circle_without_feedforward_has_the_predicted_steady_error():
    for radius, vx in ((50.0, 10.0), (100.0, 20.0)):
        mean, spread = _circle_error(radius, vx, False)
        assert abs(mean - _predicted_error(radius, vx)) < 1e-3
        assert spread < 1e-4
    assert np.isclose(_predicted_error(50.0, 10.0), -0.0499, atol=1e-4)


def test_circle_with_feedforward_has_nearly_zero_steady_error():
    for radius, vx in ((50.0, 10.0), (100.0, 20.0)):
        mean, spread = _circle_error(radius, vx, True)
        assert abs(mean) < 5e-4 and spread < 1e-4
