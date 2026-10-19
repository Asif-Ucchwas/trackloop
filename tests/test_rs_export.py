import math

import numpy as np
import pytest

from trackloop.planner_export import load_path, save_path
from trackloop.rs_export import rs_to_path

START = (0.0, 0.0, 0.0)


def test_forward_straight():
    p, info = rs_to_path(START, [("S", 5.0)], radius=2.0)
    assert info["length_m"] == pytest.approx(10.0)
    assert np.allclose(p.x, p.s) and np.allclose(p.y, 0.0)
    assert np.all(p.direction == 1.0) and np.all(p.curvature == 0.0)


def test_reverse_straight_moves_backwards_with_same_heading():
    p, info = rs_to_path(START, [("S", -5.0)], radius=2.0)
    assert p.x[-1] == pytest.approx(-10.0)
    assert np.allclose(p.heading, 0.0)
    assert np.all(p.direction == -1.0)


def test_forward_left_quarter_turn():
    p, info = rs_to_path(START, [("L", math.pi / 2)], radius=4.0)
    assert (p.x[-1], p.y[-1]) == pytest.approx((4.0, 4.0))
    assert p.heading[-1] == pytest.approx(math.pi / 2)
    assert info["length_m"] == pytest.approx(2 * math.pi)
    assert np.allclose(p.curvature, 0.25)


def test_reverse_left_quarter_turn():
    p, info = rs_to_path(START, [("L", -math.pi / 2)], radius=4.0)
    assert (p.x[-1], p.y[-1]) == pytest.approx((-4.0, 4.0))
    assert p.heading[-1] == pytest.approx(-math.pi / 2)
    assert np.allclose(p.curvature, 0.25)
    assert np.all(p.direction == -1.0)


def test_cusp_is_sampled_exactly_and_carries_new_direction():
    p, info = rs_to_path(START, [("S", 2.0), ("S", -1.0)], radius=1.0)
    assert info["n_cusps"] == 1
    assert info["cusp_s_m"] == pytest.approx([2.0])
    i = int(np.flatnonzero(np.isclose(p.s, 2.0, atol=1e-12))[0])
    assert p.x[i] == pytest.approx(2.0)
    assert p.direction[i] == -1.0 and p.direction[i - 1] == 1.0
    assert p.x[-1] == pytest.approx(1.0)


def test_steps_are_small_and_heading_is_continuous_across_a_cusp():
    segs = [("L", 1.0), ("S", -1.0), ("R", 0.5)]
    p, info = rs_to_path(START, segs, radius=4.0, ds=0.1)
    assert np.all(np.diff(p.s) > 0.0) and np.all(np.diff(p.s) <= 0.1 + 1e-9)
    assert np.all(np.hypot(np.diff(p.x), np.diff(p.y)) <= 0.1 + 1e-9)
    assert np.all(np.abs(np.diff(p.heading)) <= 0.1 / 4.0 + 1e-9)
    assert info["n_cusps"] == 2


def test_heading_rate_equals_curvature_times_direction():
    segs = [("L", 1.0), ("S", -1.0), ("R", -0.5), ("S", 0.7)]
    p, _ = rs_to_path(START, segs, radius=3.0)
    ds = np.diff(p.s)
    for i in range(len(ds)):
        assert p.heading[i + 1] - p.heading[i] == pytest.approx(
            p.curvature[i] * p.direction[i] * ds[i], abs=1e-9)


def test_end_pose_error_against_goal():
    _, info = rs_to_path(START, [("L", math.pi / 2)], radius=4.0,
                         goal=(4.0, 4.0, math.pi / 2))
    assert info["end_position_error_m"] < 1e-9
    assert info["end_heading_error_rad"] < 1e-9


def test_zero_length_segments_are_dropped():
    p, info = rs_to_path(START, [("S", 0.0), ("L", -1e-12), ("S", 1.0)], radius=1.0)
    assert info["n_cusps"] == 0 and info["n_segments"] == 1


def test_empty_path_and_bad_type_rejected():
    with pytest.raises(ValueError):
        rs_to_path(START, [("S", 0.0)], radius=1.0)
    with pytest.raises(ValueError):
        rs_to_path(START, [("X", 1.0)], radius=1.0)


def test_save_load_round_trip(tmp_path):
    p, info = rs_to_path(START, [("L", 1.0), ("S", -1.0)], radius=4.0)
    save_path(p, info, tmp_path / "rs")
    q = load_path(tmp_path / "rs")
    for name in ("x", "y", "heading", "curvature", "s", "direction"):
        assert np.array_equal(getattr(p, name), getattr(q, name))
