import math

import pytest

from trackloop import harness
from trackloop.harness import (CONTROLLERS, OFFSET_RANGE, all_tasks, max_time, path_names,
                               run_one, start_condition, start_state, summarize)


def test_start_condition_is_fixed_by_seed_and_path_and_within_range():
    assert start_condition(3, 7) == start_condition(3, 7)
    assert start_condition(3, 7) != start_condition(4, 7)
    assert start_condition(3, 7) != start_condition(3, 8)
    for seed in range(20):
        e, d = start_condition(seed, 0)
        assert abs(e) <= OFFSET_RANGE and abs(d) <= harness.HEADING_RANGE


def test_start_state_is_offset_to_the_left_of_the_path_start():
    _, path = harness._test_set()["rs_ahead"]
    x, y, psi, v, delta = start_state(path, 0.2, 0.05)
    h = path.heading[0]
    assert (x - path.x[0]) * -math.sin(h) + (y - path.y[0]) * math.cos(h) == pytest.approx(0.2)
    assert psi == pytest.approx(h + 0.05) and v == 0.0 and delta == 0.0


def test_the_task_list_covers_every_path_controller_speed_and_seed():
    tasks = all_tasks()
    assert len(tasks) == 19 * 4 * 3 * 5 == len(set(tasks))
    assert {t[0] for t in tasks} == set(path_names())


def test_a_run_is_reproducible_and_paired_across_controllers():
    a = run_one(("rs_lane_shift", "Kinematic LQR", 1.5, 2))
    b = run_one(("rs_lane_shift", "Kinematic LQR", 1.5, 2))
    assert a == b and a["status"] == "ok" and a["finished"]
    other = run_one(("rs_lane_shift", "Stanley", 1.5, 2))
    assert (other["e0"], other["dpsi0"]) == (a["e0"], a["dpsi0"])


def test_an_infeasible_path_is_run_and_flagged_not_dropped():
    row = run_one(("rs_parallel_park", "Kinematic LQR", 3.0, 0))
    assert row["status"] == "ok" and row["finished"] and "rms_lateral" in row
    fast = run_one(("rs_parallel_park", "Pure Pursuit", 10.0, 0))
    assert fast["feasible"] is False and "lateral_accel" in fast["infeasible_reasons"]
    assert row["drivable"] is True and fast["drivable"] is False
    sharp = run_one(("thesis_hybrid_seed3", "Kinematic LQR", 1.5, 0))
    assert sharp["drivable"] is False and "curvature" in sharp["infeasible_reasons"]
    assert sharp["status"] == "ok" and sharp["lag_distance_m"] > 0.0


def test_a_controller_that_raises_gives_an_error_row(monkeypatch):
    def boom(path):
        raise RuntimeError("solver failed")
    monkeypatch.setitem(CONTROLLERS, "Boom", boom)
    row = run_one(("rs_ahead", "Boom", 1.5, 0))
    assert row["status"] == "error: RuntimeError" and row["finished"] is False


def test_time_limit_scales_with_path_length_and_inverse_speed():
    _, long_path = harness._test_set()["thesis_hybrid_seed0"]
    assert max_time(long_path, 0.5) > 3.0 * long_path.s[-1] / 0.5
    expected = 3.0 * long_path.s[-1] * (1.0 / 0.5 - 1.0 / 1.5)
    assert max_time(long_path, 0.5) - max_time(long_path, 1.5) == pytest.approx(expected)


def test_summary_counts_finished_runs_and_scopes():
    rows = [run_one(("rs_lane_shift", "Kinematic LQR", 1.5, s)) for s in (0, 1)]
    rows += [run_one(("rs_parallel_park", "Kinematic LQR", 1.5, 0))]
    s = {(x["scope"], x["speed"]): x for x in summarize(rows)}
    assert s[("all", 1.5)]["runs"] == 3 and s[("all", 1.5)]["finished"] == 3
    assert s[("drivable", 1.5)]["runs"] == 3


def test_settings_record_what_fixes_the_results():
    s = harness.settings()
    assert s["speeds"] == [0.5, 1.5, 3.0] and s["seeds"] == [0, 1, 2, 3, 4]
    assert len(s["paths"]) == 19 and "runner" in s and s["vehicle"]["wheelbase"] == 2.7
