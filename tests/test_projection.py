import numpy as np

from trackloop import paths
from trackloop.projection import project


def test_foot_point_between_vertices_on_a_straight_path():
    pr = project(paths.straight(30.0), 3.04, 0.2)
    assert pr.index == 30 and np.isclose(pr.t, 0.4)
    assert np.isclose(pr.s, 3.04) and np.isclose(pr.x, 3.04)
    assert np.isclose(pr.error, 0.2) and np.isclose(pr.heading, 0.0)


def test_error_is_negative_on_the_right_of_the_path():
    assert np.isclose(project(paths.straight(30.0), 3.04, -0.2).error, -0.2)


def _beside_arc(radius, phi, offset):
    """A point at angle phi along a left arc of the given radius, offset outward."""
    return (radius + offset) * np.sin(phi), radius - (radius + offset) * np.cos(phi)


def test_arc_foot_point_matches_the_exact_geometry_between_vertices():
    path = paths.arc(20.0, 1.5)
    phi = 0.5037                     # vertices are 0.005 rad apart
    pr = project(path, *_beside_arc(20.0, phi, 0.3))
    assert abs(pr.error - (-0.3)) < 2e-4       # outside a left turn is the right side
    assert abs(pr.heading - phi) < 1e-4
    assert abs(pr.s - 20.0 * phi) < 1e-3
    assert np.isclose(pr.curvature, 0.05)


def test_error_is_smooth_while_moving_around_a_circle():
    path = paths.arc(20.0, 1.5)
    errors = [project(path, *_beside_arc(20.0, phi, 0.3)).error
              for phi in np.linspace(0.1, 0.2, 200)]
    assert np.ptp(errors) < 2e-4


def test_point_past_the_end_projects_onto_the_last_vertex():
    path = paths.straight(30.0)
    pr = project(path, 31.0, 0.2)
    assert pr.index == len(path) - 2 and np.isclose(pr.t, 1.0)
    assert np.isclose(pr.error, 0.2)


def test_search_window_starts_at_the_given_index():
    pr = project(paths.straight(30.0), 3.0, 0.0, start=200, window=50)
    assert pr.index >= 199
