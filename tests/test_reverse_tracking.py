import math

import numpy as np

from trackloop.controllers.pure_pursuit import PurePursuit
from trackloop.controllers.stanley import Stanley, StanleyParams
from trackloop.models import kinematic as kin
from trackloop.paths import Path, cross_track_error
from trackloop.rs_export import rs_to_path

DT = 0.05
BEHIND = [("L", -1.0), ("S", -1.0), ("R", -1.0)]


def run(controller, path, state0, t_end):
    state = np.array(state0, dtype=float)
    controller.reset()
    errors = []
    for _ in range(int(round(t_end / DT))):
        state = kin.step(state, controller.command(state), 0.0, DT)
        errors.append(cross_track_error(path, state[0], state[1]))
    return np.abs(np.array(errors))


def offset_start(path, offset=0.5, speed=-1.5):
    h = path.heading[0]
    return [path.x[0] - offset * math.sin(h), path.y[0] + offset * math.cos(h), h, speed, 0.0]


def reverse_straight():
    return rs_to_path((0.0, 0.0, 0.0), [("S", -30.0)], radius=1.0)[0]


def test_reverse_stanley_converges_on_a_reverse_straight():
    path = reverse_straight()
    err = run(Stanley(path), path, offset_start(path, 1.0), 40.0)
    assert err[:5].max() > 0.9 and err[-20:].max() < 0.01


def test_forward_stanley_law_diverges_in_reverse():
    rev = reverse_straight()
    as_forward = Path(rev.x, rev.y, rev.heading, rev.curvature, rev.s, np.ones(len(rev)))
    err = run(Stanley(as_forward), as_forward, offset_start(rev, 1.0), 15.0)
    assert err.max() > 5.0


def test_reverse_stanley_with_feedforward_tracks_a_reverse_curve():
    path = rs_to_path((0.0, 0.0, 0.0), BEHIND, radius=5.0)[0]
    ctrl = Stanley(path, StanleyParams(feedforward=True))
    n = int(path.s[-1] / 1.5 / DT)
    err = run(ctrl, path, offset_start(path), path.s[-1] / 1.5)[:n]
    assert err.max() < 0.6 and math.sqrt((err ** 2).mean()) < 0.3


def test_pure_pursuit_needs_no_change_in_reverse():
    path = rs_to_path((0.0, 0.0, 0.0), BEHIND, radius=5.0)[0]
    n = int(path.s[-1] / 1.5 / DT)
    err = run(PurePursuit(path), path, offset_start(path), path.s[-1] / 1.5)[:n]
    assert err.max() < 0.6 and math.sqrt((err ** 2).mean()) < 0.3


def test_pure_pursuit_forward_law_equals_the_reverse_law():
    ld, wheelbase = 4.0, 2.7
    for alpha in np.linspace(-3.0, 3.0, 25):
        forward = math.atan(2 * wheelbase * math.sin(alpha) / ld)
        reverse = -math.atan(2 * wheelbase * math.sin(alpha - math.pi) / ld)
        assert math.isclose(forward, reverse, abs_tol=1e-12)
