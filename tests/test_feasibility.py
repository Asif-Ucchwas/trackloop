import math

import numpy as np
import pytest

from trackloop import paths
from trackloop.feasibility import check_path, curvature_limit
from trackloop.rs_export import rs_to_path
from trackloop.testset import load_test_set

START = (0.0, 0.0, 0.0)


def rs(segs, radius=5.0):
    return rs_to_path(START, segs, radius=radius)[0]


def ramp(n, kappa_end=0.2):
    """Straight-line samples 0.1 m apart whose curvature rises linearly from 0."""
    s = np.arange(n) * 0.1
    zeros = np.zeros(n)
    return paths.Path(s, zeros, zeros, np.linspace(0.0, kappa_end, n), s, np.ones(n))


def test_curvature_limit_follows_from_the_steering_limit():
    assert curvature_limit() == pytest.approx(math.tan(0.6) / 2.7)
    assert curvature_limit() == pytest.approx(0.2534, abs=1e-4)


def test_a_straight_line_passes_both_checks():
    r = check_path(paths.straight(20.0))
    assert r.feasible and r.steering_events == 0 and r.worst_lag_distance == 0.0


def test_an_arc_tighter_than_the_limit_fails_the_curvature_check_only():
    tight = check_path(rs([("L", 1.0)], radius=3.0))
    assert not tight.curvature_ok and tight.rate_ok
    assert tight.samples_over_limit == len(rs([("L", 1.0)], radius=3.0))
    assert tight.max_excess == pytest.approx(1.0 / 3.0 - curvature_limit())
    assert check_path(rs([("L", 1.0)], radius=5.0)).curvature_ok


def test_a_curvature_step_is_an_event_with_the_expected_lag():
    r = check_path(rs([("S", 1.0), ("L", 1.0)]), speed=1.5)
    assert r.steering_events == 1 and not r.rate_ok and r.curvature_ok
    assert r.largest_event_rad == pytest.approx(math.atan(2.7 * 0.2))
    assert r.worst_lag_distance == pytest.approx(1.5 * math.atan(2.7 * 0.2) / 0.8)


def test_the_same_step_is_followable_at_a_low_enough_speed():
    path = rs([("S", 1.0), ("L", 1.0)])
    assert check_path(path, speed=0.1).rate_ok
    assert not check_path(path, speed=1.5).rate_ok


def test_a_left_to_right_flip_is_one_event_of_twice_the_size():
    r = check_path(rs([("L", 1.0), ("R", 1.0)]), speed=1.5)
    assert r.steering_events == 1
    assert r.largest_event_rad == pytest.approx(2.0 * math.atan(2.7 * 0.2))


def test_steps_across_a_cusp_are_not_counted():
    r = check_path(rs([("L", 1.0), ("S", -1.0)]), speed=1.5)
    assert r.steering_events == 0 and r.rate_ok


def test_a_slow_curvature_ramp_is_followable_and_a_fast_one_is_not():
    assert check_path(ramp(21), speed=1.5).rate_ok
    fast = check_path(ramp(6), speed=1.5)
    assert fast.steering_events == 1
    assert fast.largest_event_rad == pytest.approx(math.atan(2.7 * 0.2))


def test_speed_must_be_positive():
    with pytest.raises(ValueError):
        check_path(paths.straight(5.0), speed=0.0)


def test_test_set_facts_that_the_notes_rely_on():
    results = {name: check_path(path, speed=1.5) for name, path in load_test_set()}
    over = {n for n, r in results.items() if not r.curvature_ok}
    assert over == {"thesis_hybrid_seed3", "thesis_theta_star_tight_seed3"}
    assert results["rs_three_point"].feasible and results["rs_ahead"].feasible
    park = results["rs_parallel_park"]
    assert park.steering_events == 1
    assert park.largest_event_rad == pytest.approx(0.990, abs=1e-3)
    assert park.worst_lag_distance == pytest.approx(1.857, abs=1e-2)
