import numpy as np

from trackloop.models import dynamic as dyn

P = dyn.DynamicParams()
DT = 0.01


def start(vx):
    return np.array([0.0, 0.0, 0.0, vx, 0.0, 0.0, 0.0])


def test_understeer_gradient_value():
    assert np.isclose(P.understeer_gradient, 0.00375)


def test_straight_line_stays_straight():
    traj = dyn.rollout(start(10.0), 0.0, 0.0, DT, 500)
    assert np.allclose(traj[:, dyn.Y], 0.0)
    assert np.allclose(traj[:, dyn.VY], 0.0)
    assert np.allclose(traj[:, dyn.R], 0.0)
    assert np.isclose(traj[-1, dyn.X], 50.0)


def test_steady_state_yaw_rate_matches_understeer_formula():
    delta = 0.02
    for vx in (10.0, 20.0, 30.0):
        traj = dyn.rollout(start(vx), delta, 0.0, DT, 1000)
        expected = vx * delta / (P.wheelbase + P.understeer_gradient * vx**2)
        assert np.isclose(traj[-1, dyn.R], expected, rtol=2e-3)


def test_understeer_turns_less_than_kinematic():
    delta, vx = 0.02, 20.0
    traj = dyn.rollout(start(vx), delta, 0.0, DT, 1000)
    kinematic_rate = vx * np.tan(delta) / P.wheelbase
    assert traj[-1, dyn.R] < kinematic_rate


def test_left_and_right_are_mirror_images():
    left = dyn.rollout(start(15.0), 0.05, 0.0, DT, 300)
    right = dyn.rollout(start(15.0), -0.05, 0.0, DT, 300)
    assert left[-1, dyn.Y] > 0
    assert np.allclose(left[:, dyn.Y], -right[:, dyn.Y])
    assert np.allclose(left[:, dyn.PSI], -right[:, dyn.PSI])
    assert np.allclose(left[:, dyn.R], -right[:, dyn.R])


def test_steering_limits_apply():
    traj = dyn.rollout(start(10.0), 2.0, 0.0, DT, 300)
    assert np.isclose(traj[-1, dyn.DELTA], P.max_steer)
    first = dyn.step(start(10.0), 2.0, 0.0, DT)
    assert np.isclose(first[dyn.DELTA], P.max_steer_rate * DT)
