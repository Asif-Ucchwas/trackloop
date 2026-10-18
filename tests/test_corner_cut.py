import numpy as np

from trackloop.corner_cut import THEORY_OUTWARD, THEORY_PEAK, THEORY_WHERE, measure
from trackloop.models.kinematic import KinematicParams


def test_peak_matches_small_angle_theory():
    for radius, ld in ((40.0, 2.0), (40.0, 4.0), (80.0, 4.0), (80.0, 8.0)):
        peak, _, _ = measure(radius, ld, 5.0)
        assert np.isclose(peak * radius / ld**2, THEORY_PEAK, rtol=0.03)


def test_peak_is_reached_a_fixed_fraction_of_the_lookahead_past_the_corner():
    for radius, ld in ((40.0, 4.0), (80.0, 8.0)):
        _, where, _ = measure(radius, ld, 5.0)
        assert np.isclose(where / ld, THEORY_WHERE, atol=0.03)


def test_doubling_the_lookahead_quadruples_the_cut():
    small, _, _ = measure(80.0, 4.0, 5.0)
    large, _, _ = measure(80.0, 8.0, 5.0)
    assert 3.8 < large / small < 4.2


def test_doubling_the_radius_halves_the_cut():
    tight, _, _ = measure(40.0, 4.0, 5.0)
    gentle, _, _ = measure(80.0, 4.0, 5.0)
    assert 0.45 < gentle / tight < 0.55


def test_cut_barely_depends_on_speed():
    slow, _, _ = measure(40.0, 4.0, 3.0)
    fast, _, _ = measure(40.0, 4.0, 10.0)
    assert np.isclose(slow, fast, rtol=0.04)


def test_outward_overshoot_after_the_cut_is_small():
    for radius, ld in ((80.0, 4.0), (80.0, 8.0), (40.0, 4.0)):
        _, _, dip = measure(radius, ld, 5.0)
        assert np.isclose(dip * radius / ld**2, THEORY_OUTWARD, atol=0.0008)


def test_the_steering_rate_limit_does_not_change_the_cut():
    ideal, _, _ = measure(40.0, 4.0, 5.0)
    limited, _, _ = measure(40.0, 4.0, 5.0, params=KinematicParams())
    assert np.isclose(ideal, limited, rtol=0.01)
