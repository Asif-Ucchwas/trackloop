import numpy as np
from scipy.linalg import eig, expm

from trackloop.controllers.lqr import GainSchedule, bryson_weights, error_model, lqr_gain
from trackloop.models.dynamic import DynamicParams

SPEEDS = (2.0, 5.0, 10.0, 20.0, 30.0)


def test_error_model_matches_hand_calculation_at_10_mps():
    a, b, bd = error_model(10.0)
    assert np.allclose(a[:, 0], 0.0)
    assert np.isclose(a[1, 1], -12.0) and np.isclose(a[1, 2], 120.0)
    assert np.isclose(a[1, 3], 3.6) and np.isclose(a[3, 1], 2.16)
    assert np.isclose(a[3, 2], -21.6) and np.isclose(a[3, 3], -13.608)
    assert np.allclose(b.ravel(), [0.0, 160.0 / 3.0, 0.0, 38.4])
    assert np.allclose(bd.ravel(), [0.0, -6.4, 0.0, -13.608])


def test_open_loop_has_two_poles_at_the_origin_and_two_stable_poles():
    for v in SPEEDS:
        poles = np.linalg.eigvals(error_model(v)[0])
        assert np.sum(np.abs(poles) < 1e-9) == 2
        assert np.all(poles.real[np.abs(poles) >= 1e-9] < 0)


def test_riccati_solution_satisfies_the_equation_and_is_positive_definite():
    q, r = bryson_weights()
    for v in SPEEDS:
        k, p, a, b = lqr_gain(v)
        residual = a.T @ p + p @ a - p @ b @ np.linalg.solve(r, b.T @ p) + q
        assert np.abs(residual).max() < 1e-9
        assert np.allclose(p, p.T) and np.all(np.linalg.eigvalsh(p) > 0)


def test_riccati_solution_matches_the_hamiltonian_method():
    q, r = bryson_weights()
    for v in SPEEDS:
        k, p, a, b = lqr_gain(v)
        h = np.block([[a, -b @ np.linalg.solve(r, b.T)], [-q, -a.T]])
        values, vectors = eig(h)
        stable = vectors[:, values.real < 0]
        p_ham = np.real(stable[4:, :] @ np.linalg.inv(stable[:4, :]))
        assert np.allclose(p, p_ham, rtol=1e-7, atol=1e-9)


def test_cost_to_go_equals_the_integrated_closed_loop_cost():
    q, r = bryson_weights()
    k, p, a, b = lqr_gain(10.0)
    x0 = np.array([0.5, 0.0, 0.05, 0.0])
    dt, n = 0.001, 40000
    step = expm((a - b @ k) * dt)
    xs = np.zeros((n + 1, 4))
    xs[0] = x0
    for i in range(n):
        xs[i + 1] = step @ xs[i]
    u = -(xs @ k.T)[:, 0]
    rate = np.einsum("ij,jk,ik->i", xs, q, xs) + r[0, 0] * u**2
    cost = dt * (rate.sum() - 0.5 * (rate[0] + rate[-1]))
    assert np.isclose(cost, x0 @ p @ x0, rtol=1e-5)


def test_cross_track_gain_is_sqrt_q11_over_r_at_every_speed():
    q, r = bryson_weights()
    for v in np.arange(2.0, 30.5, 2.0):
        assert np.isclose(lqr_gain(v)[0][0, 0], np.sqrt(q[0, 0] / r[0, 0]), rtol=1e-9)


def test_closed_loop_is_stable_at_every_speed():
    for v in np.arange(2.0, 30.5, 1.0):
        k, p, a, b = lqr_gain(v)
        assert np.linalg.eigvals(a - b @ k).real.max() < -0.3


def test_gain_values_at_10_mps():
    k = lqr_gain(10.0)[0][0]
    assert np.allclose(k, [0.4, 0.1034, 2.4929, 0.4552], rtol=2e-3)


def test_schedule_matches_direct_solution_on_and_between_grid_points():
    sched = GainSchedule()
    for v in (2.0, 10.0, 30.0):
        assert np.allclose(sched.gain(v), lqr_gain(v)[0][0])
    for v in (2.1, 3.7, 5.1, 8.3, 12.7, 17.1, 24.9):
        assert np.allclose(sched.gain(v), lqr_gain(v)[0][0], rtol=2e-3)


def test_schedule_holds_the_end_gains_outside_the_grid():
    sched = GainSchedule()
    assert np.allclose(sched.gain(0.5), sched.gain(2.0))
    assert np.allclose(sched.gain(50.0), sched.gain(30.0))
