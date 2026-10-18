import numpy as np

from trackloop import paths
from trackloop.controllers.stanley import Stanley, StanleyParams
from trackloop.models.kinematic import KinematicParams
from trackloop.simulate import simulate

L = 2.7
FAST_STEERING = KinematicParams(max_steer_rate=100.0)


def test_hand_calculated_steering_value_at_zero_speed():
    st = Stanley(paths.straight(30.0))
    expected = -np.arctan(1.5 * 0.1 / 1.0)
    assert np.isclose(st.command([3.0, 0.1, 0.0, 0.0, 0.0]), expected)


def test_no_steering_when_on_path_and_aligned():
    st = Stanley(paths.straight(30.0))
    assert abs(st.command([3.0, 0.0, 0.0, 5.0, 0.0])) < 1e-12


def test_heading_term_acts_alone_when_front_axle_is_on_the_path():
    st = Stanley(paths.straight(30.0))
    state = [3.0, -L * np.sin(0.1), 0.1, 5.0, 0.0]
    assert np.isclose(st.command(state), -0.1, atol=1e-9)


def test_steering_is_limited():
    st = Stanley(paths.straight(30.0))
    assert st.command([3.0, 5.0, 0.0, 0.5, 0.0]) == -0.6
    assert st.command([3.0, -5.0, 0.0, 0.5, 0.0]) == 0.6


def test_zero_speed_without_softening_stays_finite():
    st = Stanley(paths.straight(30.0), StanleyParams(k_soft=0.0))
    assert st.command([3.0, 0.1, 0.0, 0.0, 0.0]) == -0.6


def _front_error(v, dt, k_soft=0.0, y0=0.1):
    path = paths.straight(120.0)
    st = Stanley(path, StanleyParams(k=1.0, k_soft=k_soft))
    _, cte = simulate(st, path, [0, y0, 0, v, 0], dt, 4.0, params=FAST_STEERING,
                      ref_offset=L)
    return cte, np.arange(len(cte)) * dt


def test_front_axle_error_decays_exponentially():
    cte, t = _front_error(5.0, 0.005)
    assert np.max(np.abs(cte - 0.1 * np.exp(-t))) < 3e-4


def test_gap_to_theory_is_a_sampling_effect_that_halves_with_dt():
    gaps = []
    for dt in (0.01, 0.005):
        cte, t = _front_error(5.0, dt)
        gaps.append(np.max(np.abs(cte - 0.1 * np.exp(-t))))
    assert 1.8 < gaps[0] / gaps[1] < 2.2


def test_error_never_overshoots():
    cte, _ = _front_error(5.0, 0.01)
    assert np.all(np.diff(cte) < 0) and cte.min() > 0


def test_decay_rate_does_not_depend_on_speed():
    values = [_front_error(v, 0.01)[0][200] for v in (3.0, 5.0, 10.0)]
    assert np.allclose(values, values[1], rtol=0.02)


def test_softening_slows_the_decay_by_v_over_v_plus_k_soft():
    cte, t = _front_error(5.0, 0.005, k_soft=1.0)
    assert np.max(np.abs(cte - 0.1 * np.exp(-5.0 / 6.0 * t))) < 3e-4


def test_on_a_circle_the_rear_axle_runs_inside_and_the_front_axle_on_the_path():
    radius = 20.0
    path = paths.arc(radius, 4 * np.pi)
    st = Stanley(path, StanleyParams(k=1.0, k_soft=1.0))
    state0 = [0, 0, 0, 5.0, np.arctan(L / radius)]
    traj, rear = simulate(st, path, state0, 0.01, 40.0)
    offset = radius - np.sqrt(radius**2 - L**2)
    assert np.allclose(rear[-500:], offset, atol=1e-3)
    front = [paths.cross_track_error(path, s[0] + L * np.cos(s[2]),
                                     s[1] + L * np.sin(s[2])) for s in traj[-500:]]
    assert np.max(np.abs(front)) < 2e-3
