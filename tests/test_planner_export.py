import math

import numpy as np
import pytest

from trackloop.planner_export import export_path, load_path, save_path

CORNER = [(0, 0), (100, 0), (100, 100)]


def test_straight_polyline_is_unchanged():
    path, info = export_path([(0, 0), (10, 0), (20, 0)], cell=1.0)
    assert path.s[-1] == pytest.approx(20.0)
    assert np.allclose(path.y, 0.0) and np.allclose(path.heading, 0.0)
    assert np.allclose(path.curvature, 0.0)
    assert info["n_corners"] == 0 and info["min_radius_m"] is None


def test_right_angle_corner_matches_analytic_length():
    # legs of 100 m, radius 10: tangent length 10, arc length 10 * pi / 2
    path, info = export_path(CORNER, cell=1.0, radius=10.0)
    assert path.s[-1] == pytest.approx(200.0 - 20.0 + 10.0 * math.pi / 2)
    assert path.x[-1] == pytest.approx(100.0) and path.y[-1] == pytest.approx(100.0)
    assert path.heading[-1] == pytest.approx(math.pi / 2)
    assert np.max(path.curvature) == pytest.approx(0.1)
    assert info["max_cut_m"] == pytest.approx(10.0 * (math.sqrt(2.0) - 1.0))


def test_samples_are_evenly_spaced_in_arc_length():
    path, _ = export_path(CORNER, cell=1.0, radius=10.0)
    gaps = np.diff(path.s)
    assert np.all(gaps <= 0.1 + 1e-9) and np.allclose(gaps[:-1], 0.1)


def test_heading_matches_the_chord_of_the_positions():
    path, _ = export_path(CORNER, cell=1.0, radius=10.0)
    chord = np.arctan2(np.diff(path.y), np.diff(path.x))
    assert np.max(np.abs(chord - path.heading[:-1])) < 0.01


def test_right_turn_has_negative_curvature():
    path, _ = export_path([(0, 0), (50, 0), (50, -50)], cell=1.0, radius=5.0)
    assert np.min(path.curvature) == pytest.approx(-0.2)
    assert path.heading[-1] == pytest.approx(-math.pi / 2)


def test_short_legs_get_a_smaller_radius_and_are_flagged():
    path, info = export_path([(0, 0), (4, 0), (4, 4)], cell=1.0, radius=10.0)
    assert info["n_tight"] == 1
    assert info["min_radius_m"] == pytest.approx(2.0)  # half of the 4 m legs
    assert info["max_curvature"] == pytest.approx(0.5)


def test_adjacent_corners_share_a_short_leg_without_overlap():
    path, info = export_path([(0, 0), (10, 0), (10, 2), (20, 2)], cell=1.0, radius=6.0)
    assert info["n_tight"] == 2
    assert np.max(np.hypot(np.diff(path.x), np.diff(path.y))) <= 0.1 + 1e-9
    assert path.x[-1] == pytest.approx(20.0) and path.y[-1] == pytest.approx(2.0)


def test_cell_size_scales_the_path():
    a, _ = export_path([(0, 0), (10, 0), (10, 10)], cell=1.0, radius=2.0)
    b, _ = export_path([(0, 0), (10, 0), (10, 10)], cell=3.0, radius=6.0)
    assert b.s[-1] == pytest.approx(3.0 * a.s[-1])
    assert b.x[-1] == pytest.approx(30.0)


def test_repeated_points_are_ignored_and_short_input_is_rejected():
    path, _ = export_path([(0, 0), (0, 0), (5, 0)], cell=1.0)
    assert path.s[-1] == pytest.approx(5.0)
    with pytest.raises(ValueError):
        export_path([(1, 1), (1, 1)])


def test_save_and_load_round_trip(tmp_path):
    path, info = export_path([(0, 0), (30, 0), (30, 30)], cell=1.0, radius=5.0)
    save_path(path, info, tmp_path / "demo")
    back = load_path(tmp_path / "demo")
    for name in ("x", "y", "heading", "curvature", "s", "direction"):
        assert np.array_equal(getattr(back, name), getattr(path, name))
    assert (tmp_path / "demo.json").read_text().endswith("\n")
