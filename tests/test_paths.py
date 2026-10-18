import numpy as np

from trackloop import paths


def test_straight_path_fields():
    p = paths.straight(10.0)
    assert len(p) == 101
    assert np.isclose(p.s[-1], 10.0)
    assert np.allclose(p.y, 0.0) and np.allclose(p.heading, 0.0)
    assert np.allclose(p.curvature, 0.0) and np.all(p.direction == 1.0)


def test_left_quarter_circle_ends_where_geometry_says():
    p = paths.arc(10.0, np.pi / 2)
    assert np.isclose(p.x[-1], 10.0) and np.isclose(p.y[-1], 10.0)
    assert np.isclose(p.heading[-1], np.pi / 2)
    assert np.allclose(p.curvature[1:], 0.1)


def test_right_turn_mirrors_left_turn():
    left = paths.arc(10.0, 1.5)
    right = paths.arc(10.0, -1.5)
    assert np.allclose(left.x, right.x) and np.allclose(left.y, -right.y)
    assert np.allclose(left.heading, -right.heading)
    assert np.allclose(left.curvature, -right.curvature)


def test_straight_then_arc_joins_without_a_jump_in_position():
    p = paths.straight_then_arc(20.0, 10.0, 1.0)
    join = int(np.argmax(p.s > 20.0))
    assert np.isclose(p.x[join - 1], 20.0, atol=0.1)
    assert p.curvature[join - 1] == 0.0 and p.curvature[join] == 0.1


def test_arc_length_matches_point_spacing():
    p = paths.straight_then_arc(5.0, 10.0, 1.5)
    steps = np.hypot(np.diff(p.x), np.diff(p.y))
    assert np.allclose(steps, np.diff(p.s), atol=1e-5)


def test_cross_track_sign_positive_left():
    p = paths.straight(10.0)
    assert np.isclose(paths.cross_track_error(p, 5.0, 0.3), 0.3)
    assert np.isclose(paths.cross_track_error(p, 5.0, -0.2), -0.2)
