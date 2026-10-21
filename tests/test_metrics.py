import math

import numpy as np
import pytest

from trackloop import paths
from trackloop.controllers.kinematic_lqr import KinematicLQR
from trackloop.cusp_runner import track_with_cusps
from trackloop.metrics import (chatter_reversals, compute, errors_on_path, rear_axle,
                               score_cusp_run)
from trackloop.rs_export import rs_to_path


def test_errors_are_signed_left_positive_and_heading_error_is_signed():
    path = paths.straight(10.0)
    x, y, psi = np.array([2.0, 5.0]), np.array([0.3, -0.2]), np.array([0.1, -0.05])
    cte, head = errors_on_path(path, x, y, psi)
    assert cte == pytest.approx([0.3, -0.2]) and head == pytest.approx([0.1, -0.05])


def test_error_is_distance_to_the_segment_not_to_the_nearest_vertex():
    path = paths.straight(10.0, ds=2.0)      # vertices every 2 m
    cte, _ = errors_on_path(path, np.array([3.0]), np.array([0.25]), np.array([0.0]))
    assert cte[0] == pytest.approx(0.25)


def test_error_on_a_circular_arc_is_the_radial_distance():
    path = paths.arc(10.0, math.pi / 2, ds=0.05)     # left turn, centre (0, 10)
    a = 0.7
    r = 10.0 + 0.3                                   # outside the turn = right of the path
    x, y = r * math.sin(a), 10.0 - r * math.cos(a)
    cte, _ = errors_on_path(path, np.array([x]), np.array([y]), np.array([a]))
    assert cte[0] == pytest.approx(-0.3, abs=1e-3)


def test_rear_axle_is_lr_behind_the_cg():
    x, y = rear_axle(10.0, 5.0, math.pi / 2, 1.5)
    assert (x, y) == pytest.approx((10.0, 3.5))


def synthetic(n=101, dt=0.1):
    t = np.arange(n) * dt
    return t, np.full(n, 2.0)


def test_constant_error_and_constant_steering():
    t, v = synthetic()
    m = compute(np.full(101, 0.2), np.full(101, -0.05), t, np.full(101, 0.1), v)
    assert m.rms_lateral == pytest.approx(0.2) and m.max_lateral == pytest.approx(0.2)
    assert m.rms_heading == pytest.approx(0.05)
    assert m.steering_effort == pytest.approx(0.1)
    assert m.steering_rate_rms == 0.0 and m.chatter_per_s == 0.0
    assert m.total_time == pytest.approx(10.0) and m.driving_time == pytest.approx(10.0)


def test_chatter_counts_sign_reversals_outside_the_deadband():
    rate = np.array([0.5, -0.5, 0.5, -0.5, 0.001, -0.001, 0.0])
    assert chatter_reversals(rate) == 3                  # the last three are in the deadband
    assert chatter_reversals(np.full(10, 0.4)) == 0


def test_a_zigzag_steering_log_has_high_chatter_a_smooth_ramp_has_none():
    t, v = synthetic()
    z = 0.05 * (np.arange(101) % 2)
    ramp = np.linspace(0.0, 0.2, 101)
    zig = compute(np.zeros(101), np.zeros(101), t, z, v)
    smooth = compute(np.zeros(101), np.zeros(101), t, ramp, v)
    assert zig.chatter_per_s == pytest.approx(10.0, abs=0.2)
    assert smooth.chatter_per_s == 0.0
    assert smooth.steering_rate_rms == pytest.approx(0.02)


def test_standstill_samples_are_left_out_of_the_steering_metrics():
    t = np.arange(101) * 0.1
    v = np.where(np.arange(101) < 50, 0.0, 2.0)
    delta = np.where(np.arange(101) < 50, np.linspace(0.0, 0.5, 101), 0.1)
    m = compute(np.zeros(101), np.zeros(101), t, delta, v)
    assert m.steering_effort == pytest.approx(0.1)
    assert m.driving_time == pytest.approx(5.0, abs=0.11)


def test_scoring_a_cusp_run_is_close_to_the_runner_numbers():
    path = rs_to_path((0.0, 0.0, 0.0), [("L", 1.0), ("S", -1.0), ("R", 0.5)], radius=4.0)[0]
    res = track_with_cusps(path, KinematicLQR, [path.x[0], path.y[0], path.heading[0], 0.0, 0.0])
    m, extra = score_cusp_run(path, res)
    assert extra["finished"] and extra["worst_cusp_position_error"] == pytest.approx(
        max(c["position_error"] for c in res.cusps))
    assert m.rms_lateral == pytest.approx(math.sqrt(np.mean(res.cte ** 2)), abs=0.01)
    assert m.max_lateral == pytest.approx(np.abs(res.cte).max(), abs=0.02)
    assert m.total_time == pytest.approx(res.time[-1])
