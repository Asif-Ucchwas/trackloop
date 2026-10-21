import math

import pytest

from trackloop import paths
from trackloop.feasibility import G, MU, check_path, max_speed_for_curvature
from trackloop.rs_export import rs_to_path
from trackloop.testset import load_test_set


def arc(radius):
    return rs_to_path((0.0, 0.0, 0.0), [("L", 1.0)], radius=radius)[0]


def test_max_speed_follows_from_the_friction_limit():
    assert max_speed_for_curvature(0.2) == pytest.approx(math.sqrt(MU * G / 0.2))
    assert max_speed_for_curvature(0.2) == pytest.approx(6.264, abs=1e-3)
    assert max_speed_for_curvature(0.0) == math.inf


def test_a_straight_path_has_no_speed_limit():
    r = check_path(paths.straight(20.0), speed=20.0)
    assert r.lateral_ok and r.max_feasible_speed == math.inf and r.max_lateral_accel == 0.0


def test_a_5_m_arc_is_fine_at_5_m_s_and_impossible_at_10_m_s():
    path = arc(5.0)
    slow, fast = check_path(path, speed=5.0), check_path(path, speed=10.0)
    assert slow.lateral_ok and slow.max_lateral_accel == pytest.approx(5.0)
    assert not fast.lateral_ok and fast.max_lateral_accel == pytest.approx(20.0)
    assert not fast.feasible


def test_the_largest_feasible_speed_is_the_boundary():
    path = arc(5.0)
    vmax = check_path(path).max_feasible_speed
    assert check_path(path, speed=vmax * 0.999).lateral_ok
    assert not check_path(path, speed=vmax * 1.001).lateral_ok


def test_friction_coefficient_is_a_parameter():
    path = arc(5.0)
    assert not check_path(path, speed=5.0, mu=0.4).lateral_ok
    assert check_path(path, speed=5.0, mu=0.8).lateral_ok


def test_test_set_facts_for_the_benchmark_speeds():
    results = {n: {v: check_path(p, speed=v) for v in (1.5, 5.0, 10.0, 20.0)}
               for n, p in load_test_set()}
    assert all(r[1.5].lateral_ok for r in results.values())
    assert all(r[10.0].lateral_ok is False for n, r in results.items() if n != "rs_ahead")
    assert [n for n, r in results.items() if r[20.0].lateral_ok] == ["rs_ahead"]
    assert results["rs_parallel_park"][5.0].max_feasible_speed == pytest.approx(6.264, abs=1e-3)
