import numpy as np
import pytest

from trackloop import linear_loops as ll
from trackloop import stability_limit as sl
from trackloop.controllers.lqr import lqr_gain


def test_lqr_gain_is_held_constant_above_the_schedule_grid():
    # the schedule stops at 30 m/s; above it the controller holds that gain
    assert ll.feedback_gain("Dynamic LQR", 50.0) == pytest.approx(ll.feedback_gain("Dynamic LQR", 30.0), abs=1e-12)
    assert not np.allclose(ll.feedback_gain("Dynamic LQR", 50.0), lqr_gain(50.0)[0][0])


@pytest.mark.parametrize("controller", ll.CONTROLLERS)
def test_all_loops_are_stable_at_10_m_per_s(controller):
    assert sl.spectral_radius(controller, 10.0, 0.1) < 1.0
    assert sl.linear_rate(controller, 10.0, 0.1) < 0.0


def test_linear_limit_is_the_boundary_between_stable_and_unstable():
    lim = sl.linear_limit("Stanley", 0.1)
    assert 60.0 < lim < 61.5      # about 60.7 m/s
    assert sl.spectral_radius("Stanley", lim - 0.05, 0.1) < 1.0 <= sl.spectral_radius("Stanley", lim + 0.05, 0.1)


def test_linear_limit_is_none_when_the_search_stops_below_it():
    assert sl.linear_limit("Stanley", 0.1, v_hi=50.0) is None


def test_amplitude_ratio_of_a_growing_exponential():
    t = np.arange(0.0, 60.0, 0.05)
    # late window max at t just under 60, early window (5 to 15 s) max at just under 15
    assert sl.amplitude_ratio(t, np.exp(0.1 * t)) == pytest.approx(np.exp(0.1 * 45.0), rel=0.02)
    assert sl.amplitude_ratio(t, np.exp(-0.1 * t)) < 1.0


def test_linear_trajectory_starts_at_the_offset_and_decays_when_stable():
    lin = sl.linear_trajectory("Dynamic LQR", 10.0, 0.1, 0.5, 400)
    assert lin[0] == 0.5 and abs(lin[-1]) < 1e-3 * 0.5


@pytest.mark.parametrize("fraction, grows", [(0.9, False), (1.1, True)])
def test_simulation_agrees_with_the_linear_model_near_the_stanley_limit(fraction, grows):
    lim = sl.linear_limit("Stanley", 0.1)
    r = sl.compare_run("Stanley", lim * fraction, 0.1, e0=0.001, duration=60.0)
    assert (r["ratio_lin"] > 1.0) == grows and (r["ratio_sim"] > 1.0) == grows
    assert r["trajectory_gap"] < 0.05       # observed 2e-5 below the limit and 2e-2 above it
