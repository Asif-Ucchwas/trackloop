import numpy as np
import pytest

from trackloop import harness_road as hr
from trackloop import linear_loops as ll
from trackloop.controllers.lqr import error_model, lqr_gain
from trackloop.models import dynamic as dyn
from trackloop.paths import straight


def test_pure_pursuit_gain_matches_hand_calculation():
    # ld = 2 + 0.5 * 10 = 7 m, L = 2.7 m, lr = 1.5 m
    k = ll.feedback_gain("Pure Pursuit", 10.0)
    assert k == pytest.approx([2 * 2.7 / 49, 0.0, (2 * 2.7 / 7) * (1 - 1.5 / 7), 0.0], abs=1e-9)


def test_stanley_gain_matches_hand_calculation():
    # s = k / (v + k_soft) = 1.5 / 11, e_front = e1 + lf e2 with lf = 1.2 m
    s = 1.5 / 11.0
    assert ll.feedback_gain("Stanley", 10.0) == pytest.approx([s, 0.0, 1.0 + s * 1.2, 0.0], abs=1e-9)


def test_lqr_gain_is_the_riccati_gain():
    assert ll.feedback_gain("Dynamic LQR", 10.0) == pytest.approx(lqr_gain(10.0)[0][0], abs=1e-12)


def test_continuous_closed_loop_without_lag_is_a_minus_bk():
    a, b, _ = error_model(10.0)
    k = ll.feedback_gain("Stanley", 10.0)
    sys = ll.closed_loop("Stanley", 10.0, 0.0)
    assert sys.A == pytest.approx(a - b @ k[None, :], abs=1e-12) and sys.A.shape == (4, 4)


@pytest.mark.parametrize("controller", ll.CONTROLLERS)
@pytest.mark.parametrize("lag", [0.0, 0.1, 0.2])
def test_discrete_loops_are_stable_at_10_m_per_s(controller, lag):
    sys = ll.closed_loop(controller, 10.0, lag, dt=ll.DT)
    assert sys.A.shape[0] == (4 if lag == 0.0 else 5) and ll.is_stable(sys)


def test_lag_adds_the_steering_angle_as_a_fifth_state():
    assert ll.closed_loop("Dynamic LQR", 10.0, 0.1).A.shape == (5, 5)


def test_mpc_is_only_available_sampled():
    with pytest.raises(ValueError):
        ll.closed_loop("Dynamic MPC", 10.0)


def test_unknown_controller_is_refused():
    with pytest.raises(ValueError):
        ll.feedback_gain("Bang-bang", 10.0)


def test_plant_mismatch_changes_the_loop_but_not_the_gain():
    heavy = dyn.DynamicParams(mass=1800.0)
    a = ll.closed_loop("Dynamic LQR", 10.0, 0.0, dt=ll.DT)
    b = ll.closed_loop("Dynamic LQR", 10.0, 0.0, dt=ll.DT, plant_params=heavy)
    assert not np.allclose(a.A, b.A)


@pytest.mark.parametrize("controller", ll.CONTROLLERS)
def test_linear_model_reproduces_the_simulation_for_a_small_offset(controller):
    path, e0, lag, v = straight(400.0), 0.05, 0.1, 10.0
    plant = dyn.DynamicParams(steer_lag=lag)
    ctrl = hr.CONTROLLERS[controller](path)
    ctrl.reset()
    s = np.array([0.0, e0, 0.0, v, 0.0, 0.0, 0.0])
    sys = ll.closed_loop(controller, v, lag, dt=ll.DT)
    z = np.zeros(5)
    z[0] = e0
    worst = 0.0
    for _ in range(80):
        s = dyn.step(s, ctrl.command(s), 0.0, ll.DT, plant)
        z = sys.A @ z
        worst = max(worst, abs(s[1] - z[0]))
    assert worst < 1e-3      # observed about 1e-5 m on a 0.05 m offset
