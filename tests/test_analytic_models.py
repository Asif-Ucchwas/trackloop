"""Closed-form checks of the two vehicle models."""
import math

import numpy as np
import pytest

from trackloop.models import dynamic as dyn
from trackloop.models import kinematic as kin


def test_kinematic_straight_line_is_constant_acceleration_motion():
    s = kin.rollout([0, 0, 0, 3.0, 0], 0.0, 0.5, 0.01, 400)[-1]
    assert s[0] == pytest.approx(3.0 * 4 + 0.5 * 0.5 * 16, abs=1e-6)
    assert s[1] == pytest.approx(0.0, abs=1e-9)
    assert s[3] == pytest.approx(3.0 + 0.5 * 4, abs=1e-9)


def test_kinematic_constant_steering_drives_a_circle_of_radius_L_over_tan_delta():
    p = kin.KinematicParams()
    delta, v = 0.2, 2.0
    radius = p.wheelbase / math.tan(delta)
    n = round(2 * math.pi * radius / v / 0.005)
    traj = kin.rollout([0, 0, 0, v, delta], delta, 0.0, 0.005, n)
    centre = np.array([0.0, radius])
    assert np.allclose(np.hypot(traj[:, 0] - centre[0], traj[:, 1] - centre[1]), radius, atol=1e-4)
    assert traj[-1, 2] == pytest.approx(2 * math.pi, abs=2e-3)


def _steady_yaw_rate(params, vx, delta, seconds=15.0, dt=0.005):
    traj = dyn.rollout([0, 0, 0, vx, 0, 0, delta], delta, 0.0, dt, round(seconds / dt), params)
    return traj[-1, dyn.R]


@pytest.mark.parametrize("vx", [8.0, 15.0, 25.0])
def test_dynamic_steady_state_yaw_rate_matches_the_understeer_formula(vx):
    p = dyn.DynamicParams()
    delta = 0.01
    expected = vx * delta / (p.wheelbase + p.understeer_gradient * vx ** 2)
    assert _steady_yaw_rate(p, vx, delta) == pytest.approx(expected, rel=0.01)


@pytest.mark.parametrize("vx", [10.0, 25.0])
def test_neutral_steer_vehicle_has_yaw_rate_gain_v_over_L_at_any_speed(vx):
    base = dyn.DynamicParams()
    p = dyn.DynamicParams(cf=100000.0, cr=100000.0 * base.lf / base.lr)
    assert p.understeer_gradient == pytest.approx(0.0, abs=1e-12)
    delta = 0.01
    assert _steady_yaw_rate(p, vx, delta) == pytest.approx(vx * delta / p.wheelbase, rel=0.01)


def test_understeer_vehicle_turns_less_than_the_kinematic_model_and_oversteer_more():
    base = dyn.DynamicParams()
    kinematic_gain = 15.0 * 0.01 / base.wheelbase
    assert base.understeer_gradient > 0 and _steady_yaw_rate(base, 15.0, 0.01) < kinematic_gain
    over = dyn.DynamicParams(cf=100000.0, cr=60000.0)
    assert over.understeer_gradient < 0 and _steady_yaw_rate(over, 15.0, 0.01) > 15.0 * 0.01 / over.wheelbase


def test_dynamic_straight_motion_with_zero_steering_stays_straight():
    traj = dyn.rollout([0, 0, 0, 12.0, 0, 0, 0.0], 0.0, 0.0, 0.01, 500)
    assert np.allclose(traj[:, [1, 2, 4, 5]], 0.0, atol=1e-12)
    assert traj[-1, 0] == pytest.approx(60.0)
