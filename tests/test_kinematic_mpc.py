import math

import numpy as np

from trackloop.controllers.kinematic_mpc import KinematicMPC
from trackloop.models import kinematic as kin
from trackloop.paths import cross_track_error
from trackloop.rs_export import rs_to_path

DT = 0.05
ARCS = [("L", 1.0), ("S", 1.0), ("R", 1.0)]


def run(controller, path, state0, t_end):
    state = np.array(state0, dtype=float)
    controller.reset()
    errors, commands = [], []
    for _ in range(int(round(t_end / DT))):
        cmd = controller.command(state)
        commands.append(cmd)
        state = kin.step(state, cmd, 0.0, DT)
        errors.append(cross_track_error(path, state[0], state[1]))
    return np.abs(np.array(errors)), np.array(commands)


def start(path, speed, offset=0.5):
    h = path.heading[0]
    return [path.x[0] - offset * math.sin(h), path.y[0] + offset * math.cos(h), h, speed, 0.0]


def test_recovers_from_an_offset_driving_forward_and_in_reverse():
    for segs, speed in (([("S", 30.0)], 1.5), ([("S", -30.0)], -1.5)):
        path = rs_to_path((0.0, 0.0, 0.0), segs, radius=1.0)[0]
        err, _ = run(KinematicMPC(path), path, start(path, speed, 1.0), 40.0)
        assert err[:5].max() > 0.9 and err[-20:].max() < 0.01


def test_tracks_curves_in_both_gears_without_solver_failures():
    for sign, speed in ((1.0, 1.5), (-1.0, -1.5)):
        path = rs_to_path((0.0, 0.0, 0.0), [(t, sign * v) for t, v in ARCS], radius=5.0)[0]
        n = int(path.s[-1] / abs(speed) / DT)
        mpc = KinematicMPC(path)
        err, _ = run(mpc, path, start(path, speed), path.s[-1] / abs(speed))
        assert mpc.failures == 0
        assert err[:n].max() < 0.6 and math.sqrt((err[:n] ** 2).mean()) < 0.3


def test_steering_stays_within_the_limit_for_a_large_offset():
    path = rs_to_path((0.0, 0.0, 0.0), [("S", -30.0)], radius=1.0)[0]
    mpc = KinematicMPC(path)
    err, cmd = run(mpc, path, start(path, -1.5, 4.0), 20.0)
    assert np.abs(cmd).max() <= 0.6 + 1e-9 and mpc.failures == 0
    assert err[-20:].max() < 0.5


def test_forward_and_reverse_commands_mirror_each_other():
    forward = rs_to_path((0.0, 0.0, 0.0), [("S", 30.0)], radius=1.0)[0]
    reverse = rs_to_path((0.0, 0.0, 0.0), [("S", -30.0)], radius=1.0)[0]
    cf = KinematicMPC(forward).command([0.0, 0.5, 0.1, 1.5, 0.0])
    cr = KinematicMPC(reverse).command([0.0, 0.5, -0.1, -1.5, 0.0])
    assert math.isclose(cf, cr, abs_tol=1e-7)
