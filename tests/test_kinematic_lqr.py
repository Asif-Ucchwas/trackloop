import math

import numpy as np

from trackloop.controllers.kinematic_lqr import KinematicLQR, error_model, kinematic_gain
from trackloop.models import kinematic as kin
from trackloop.paths import cross_track_error
from trackloop.rs_export import rs_to_path

DT = 0.05
ARCS = [("L", 1.0), ("S", 1.0), ("R", 1.0)]


def run(path, state0, t_end):
    controller = KinematicLQR(path)
    state = np.array(state0, dtype=float)
    errors = []
    for _ in range(int(round(t_end / DT))):
        state = kin.step(state, controller.command(state), 0.0, DT)
        errors.append(cross_track_error(path, state[0], state[1]))
    return np.abs(np.array(errors))


def start(path, speed, offset=0.5):
    h = path.heading[0]
    return [path.x[0] - offset * math.sin(h), path.y[0] + offset * math.cos(h), h, speed, 0.0]


def test_reverse_gain_is_the_forward_gain_with_the_heading_sign_flipped():
    forward, reverse = kinematic_gain(1), kinematic_gain(-1)
    assert np.allclose(reverse, [forward[0], -forward[1]])


def test_closed_loop_is_stable_in_both_gears():
    for gear in (1, -1):
        a, b = error_model(gear)
        k = kinematic_gain(gear)
        assert np.all(np.linalg.eigvals(a - b @ k[None, :]).real < 0.0)


def test_recovers_from_an_offset_driving_forward_and_in_reverse():
    for segs, speed in (([("S", 30.0)], 2.0), ([("S", -30.0)], -1.5)):
        path = rs_to_path((0.0, 0.0, 0.0), segs, radius=1.0)[0]
        err = run(path, start(path, speed, 1.0), 40.0)
        assert err[:5].max() > 0.9 and err[-20:].max() < 0.01


def test_command_does_not_depend_on_speed_even_at_a_standstill():
    path = rs_to_path((0.0, 0.0, 0.0), [("S", -30.0)], radius=1.0)[0]
    state = start(path, -1.5)
    still = list(state)
    still[3] = 0.0
    assert KinematicLQR(path).command(state) == KinematicLQR(path).command(still)


def test_tracks_curves_in_both_gears():
    for sign, speed in ((1.0, 1.5), (-1.0, -1.5)):
        path = rs_to_path((0.0, 0.0, 0.0), [(t, sign * v) for t, v in ARCS], radius=5.0)[0]
        n = int(path.s[-1] / abs(speed) / DT)
        err = run(path, start(path, speed), path.s[-1] / abs(speed))[:n]
        assert err.max() < 0.6 and math.sqrt((err ** 2).mean()) < 0.3
