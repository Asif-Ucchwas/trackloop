import numpy as np

from trackloop import paths
from trackloop.controllers.kinematic_lqr import KinematicLQR
from trackloop.controllers.kinematic_mpc import KinematicMPC
from trackloop.controllers.pure_pursuit import PurePursuit
from trackloop.controllers.stanley import Stanley, StanleyParams
from trackloop.cusp_runner import RunnerParams, track_with_cusps
from trackloop.rs_export import rs_to_path

SEGS = [("L", 1.0), ("S", -1.0), ("R", 0.5)]


def two_cusp_path():
    return rs_to_path((0.0, 0.0, 0.0), SEGS, radius=4.0)[0]


def start_of(path):
    return [path.x[0], path.y[0], path.heading[0], 0.0, 0.0]


def stanley(p):
    return Stanley(p, StanleyParams(feedforward=bool(p.direction[0] < 0)))


def test_path_without_cusps_finishes_with_no_cusp_records():
    path = paths.straight(20.0)
    result = track_with_cusps(path, KinematicLQR, start_of(path))
    assert result.finished and result.cusps == []
    assert result.end_position_error < 0.15 and np.abs(result.cte).max() < 1e-6


def test_all_four_controllers_finish_a_path_with_two_cusps():
    path = two_cusp_path()
    for make in (PurePursuit, stanley, KinematicLQR, KinematicMPC):
        result = track_with_cusps(path, make, start_of(path))
        assert result.finished and len(result.cusps) == 2
        assert all(c["position_error"] < 0.3 for c in result.cusps)
        assert result.end_position_error < 0.3


def test_vehicle_drives_each_segment_in_its_own_gear_and_stops_between():
    path = two_cusp_path()
    result = track_with_cusps(path, KinematicLQR, start_of(path))
    gears = {0: 1.0, 1: -1.0, 2: 1.0}
    moving = np.abs(result.states[:, 3]) > 0.01
    expected = np.array([gears[k] for k in result.segment])
    assert np.all(np.sign(result.states[moving, 3]) == expected[moving])
    last = [int(np.flatnonzero(result.segment == k)[-1]) for k in (0, 1)]
    assert all(result.states[i, 3] == 0.0 for i in last)


def test_steering_settles_before_each_new_segment():
    path = two_cusp_path()
    result = track_with_cusps(path, KinematicLQR, start_of(path))
    assert all(np.isfinite(c["settle_time"]) and c["settle_time"] > 0.0 for c in result.cusps)


def test_a_run_that_runs_out_of_time_is_reported_unfinished():
    path = two_cusp_path()
    result = track_with_cusps(path, KinematicLQR, start_of(path), RunnerParams(max_time=3.0))
    assert not result.finished
