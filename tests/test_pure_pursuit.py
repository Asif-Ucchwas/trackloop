import numpy as np

from trackloop import paths
from trackloop.controllers.pure_pursuit import PurePursuit, PurePursuitParams
from trackloop.models.kinematic import KinematicParams
from trackloop.simulate import simulate

FAST_STEERING = KinematicParams(max_steer_rate=100.0)


def test_lookahead_grows_with_speed_and_clips():
    p = PurePursuitParams(ld_min=2.0, k_v=0.5, ld_max=10.0)
    assert [p.lookahead(v) for v in (0.0, 4.0, 10.0, 30.0)] == [2.0, 4.0, 7.0, 10.0]
    assert p.lookahead(-4.0) == 4.0


def test_goal_point_is_at_the_lookahead_distance():
    pp = PurePursuit(paths.straight(30.0))
    gx, gy = pp.lookahead_point(3.0, 1.0, 5.0)
    assert np.isclose(gx, 3.0 + np.sqrt(24.0)) and np.isclose(gy, 0.0)
    assert np.isclose(np.hypot(gx - 3.0, gy - 1.0), 5.0)


def test_no_steering_when_on_path_and_aligned():
    pp = PurePursuit(paths.straight(30.0))
    assert abs(pp.command([3.0, 0.0, 0.0, 5.0, 0.0])) < 1e-12


def test_steering_points_back_toward_the_path():
    pp = PurePursuit(paths.straight(30.0))
    assert pp.command([3.0, -1.0, 0.0, 5.0, 0.0]) > 0
    assert pp.command([3.0, 1.0, 0.0, 5.0, 0.0]) < 0


def test_hand_calculated_steering_value():
    params = PurePursuitParams(ld_min=5.0, k_v=0.0)
    pp = PurePursuit(paths.straight(30.0), params)
    expected = np.arctan(2 * 2.7 * 0.2 / 5.0)
    assert np.isclose(pp.command([3.0, -1.0, 0.0, 5.0, 0.0]), expected)


def _theory_gap(dt, v=5.0, ld=5.0, y0=0.1):
    """Largest gap between the simulated and the damped second-order response."""
    path = paths.straight(60.0)
    pp = PurePursuit(path, PurePursuitParams(ld_min=ld, k_v=0.0))
    _, cte = simulate(pp, path, [0, y0, 0, v, 0], dt, 6.0, params=FAST_STEERING)
    t = np.arange(len(cte)) * dt
    wn = np.sqrt(2.0) * v / ld
    w = wn / np.sqrt(2.0)
    expected = y0 * np.exp(-w * t) * (np.cos(w * t) + np.sin(w * t))
    return np.max(np.abs(cte - expected))


def test_straight_line_response_matches_damped_second_order_theory():
    assert _theory_gap(0.005) < 2.5e-4


def test_gap_to_theory_is_a_sampling_effect_that_halves_with_dt():
    ratio = _theory_gap(0.02) / _theory_gap(0.01)
    assert 1.8 < ratio < 2.2


def test_overshoot_does_not_depend_on_speed_or_lookahead():
    for v, ld in ((5.0, 5.0), (10.0, 10.0), (10.0, 5.0), (3.0, 6.0)):
        path = paths.straight(120.0)
        pp = PurePursuit(path, PurePursuitParams(ld_min=ld, k_v=0.0))
        _, cte = simulate(pp, path, [0, 0.1, 0, v, 0], 0.01, 8.0, params=FAST_STEERING)
        assert np.isclose(cte.min() / 0.1, -np.exp(-np.pi), atol=3e-3)


def test_circle_is_followed_with_no_steady_error():
    radius = 20.0
    path = paths.arc(radius, 2 * np.pi)
    delta0 = np.arctan(2.7 / radius)
    for ld in (3.0, 5.0, 10.0):
        pp = PurePursuit(path, PurePursuitParams(ld_min=ld, k_v=0.0))
        _, cte = simulate(pp, path, [0, 0, 0, 5.0, delta0], 0.01, 20.0)
        assert np.max(np.abs(cte)) < 1e-3
