import numpy as np
import pytest

from trackloop.models.kinematic import DELTA, PSI, X, Y, KinematicParams, rollout, step


def test_straight_line_constant_speed():
    p = KinematicParams()
    traj = rollout([0, 0, 0, 5.0, 0.0], 0.0, 0.0, 0.01, 1000, p)
    t = 10.0
    assert traj[-1, X] == pytest.approx(5.0 * t)
    assert traj[-1, Y] == pytest.approx(0.0, abs=1e-12)
    assert traj[-1, PSI] == pytest.approx(0.0, abs=1e-12)


def test_straight_line_with_acceleration():
    p = KinematicParams()
    traj = rollout([0, 0, 0, 2.0, 0.0], 0.0, 1.0, 0.01, 500, p)
    t = 5.0
    assert traj[-1, X] == pytest.approx(2.0 * t + 0.5 * 1.0 * t**2, abs=1e-9)


def test_constant_steer_matches_analytic_circle():
    p = KinematicParams()
    delta, v, dt, n = 0.3, 5.0, 0.01, 1000
    traj = rollout([0, 0, 0, v, delta], delta, 0.0, dt, n, p)
    radius = p.wheelbase / np.tan(delta)
    omega = v / radius
    t = np.arange(n + 1) * dt
    assert np.allclose(traj[:, X], radius * np.sin(omega * t), atol=1e-5)
    assert np.allclose(traj[:, Y], radius * (1 - np.cos(omega * t)), atol=1e-5)
    assert np.allclose(traj[:, PSI], omega * t, atol=1e-8)


def test_steering_angle_limit():
    p = KinematicParams(max_steer=0.6, max_steer_rate=100.0)
    s = step([0, 0, 0, 5.0, 0.55], 2.0, 0.0, 0.01, p)
    assert s[DELTA] == pytest.approx(0.6)
    s = step([0, 0, 0, 5.0, -0.55], -2.0, 0.0, 0.01, p)
    assert s[DELTA] == pytest.approx(-0.6)


def test_steering_rate_limit():
    p = KinematicParams(max_steer=0.6, max_steer_rate=0.8)
    s = np.array([0, 0, 0, 5.0, 0.0])
    for _ in range(50):
        s = step(s, 0.6, 0.0, 0.01, p)
    assert s[DELTA] == pytest.approx(0.4)
    for _ in range(50):
        s = step(s, 0.6, 0.0, 0.01, p)
    assert s[DELTA] == pytest.approx(0.6)


def test_left_and_right_steer_turn_the_right_way():
    p = KinematicParams()
    left = rollout([0, 0, 0, 5.0, 0.2], 0.2, 0.0, 0.01, 100, p)
    right = rollout([0, 0, 0, 5.0, -0.2], -0.2, 0.0, 0.01, 100, p)
    assert left[-1, PSI] > 0 and left[-1, Y] > 0
    assert right[-1, PSI] < 0 and right[-1, Y] < 0


def test_no_motion_at_zero_speed():
    p = KinematicParams()
    traj = rollout([1.0, 2.0, 0.5, 0.0, 0.4], 0.4, 0.0, 0.01, 100, p)
    assert np.allclose(traj[-1, :3], [1.0, 2.0, 0.5])


def test_reverse_driving_flips_heading_change():
    p = KinematicParams()
    traj = rollout([0, 0, 0, -2.0, 0.2], 0.2, 0.0, 0.01, 100, p)
    assert traj[-1, X] < 0
    assert traj[-1, PSI] < 0
