import math

import numpy as np
import pytest

from trackloop import dynamic_runner as dr
from trackloop import harness_road as hr
from trackloop.models.dynamic import DynamicParams


def test_all_controllers_registered():
    assert list(hr.CONTROLLERS) == ["Pure Pursuit", "Stanley", "Dynamic LQR", "Dynamic MPC"]


def test_task_count():
    assert len(hr.all_tasks()) == 12 * 4 * 5


def test_row_is_deterministic():
    task = ("road_lateral_shift_v10", "Dynamic LQR", 2)
    assert hr.run_one(task) == hr.run_one(task)


def test_start_condition_independent_of_controller():
    a = hr.run_one(("road_curve_v5", "Pure Pursuit", 1))
    b = hr.run_one(("road_curve_v5", "Dynamic MPC", 1))
    assert (a["e0"], a["dpsi0"]) == (b["e0"], b["dpsi0"])


def test_start_state_places_rear_axle_offset():
    _, path, _ = hr._road_set()["road_curve_v5"]
    p = DynamicParams()
    st = dr.start_state(path, 5.0, 0.2, 0.0, p)
    h = float(path.heading[0])
    xr, yr = st[0] - p.lr * math.cos(st[2]), st[1] - p.lr * math.sin(st[2])
    assert xr == pytest.approx(path.x[0] - 0.2 * math.sin(h))
    assert yr == pytest.approx(path.y[0] + 0.2 * math.cos(h))


def test_adapter_passes_rear_axle_state():
    seen = {}

    class Spy:
        def reset(self): pass
        def command(self, s):
            seen["s"] = list(s)
            return 0.0

    dr.RearAxleAdapter(Spy(), 1.5).command([10.0, 0.0, 0.0, 5.0, 0.1, 0.0, 0.03])
    assert seen["s"] == pytest.approx([8.5, 0.0, 0.0, 5.0, 0.03])


def test_every_run_finishes_on_a_sample():
    for c in hr.CONTROLLERS:
        assert hr.run_one(("road_s_curve_v10", c, 0))["status"] == "finished"


def test_summary_counts():
    rows = [hr.run_one(("road_curve_v5", c, 0)) for c in hr.CONTROLLERS]
    s = hr.summarize(rows)
    assert len(s) == 4 and all(x["runs"] == 1 and x["finished"] == 1 for x in s)


def test_tire_use_zero_on_straight_state():
    st = np.array([[0, 0, 0, 10.0, 0, 0, 0]] * 3, dtype=float)
    assert dr.tire_use(st) == (0.0, 0.0, 0.0)
