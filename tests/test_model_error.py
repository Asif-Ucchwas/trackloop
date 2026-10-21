import csv
from pathlib import Path

import numpy as np
import pytest

from trackloop import harness as parking
from trackloop import harness_road as road
from trackloop import model_error as me
from trackloop.models import dynamic as dyn
from trackloop.models import kinematic as kin

RESULTS = Path("data/results")


def test_nominal_plants_are_the_default_parameters():
    assert me.dynamic_plant(None) == dyn.DynamicParams() == me.dynamic_plant("nominal")
    assert me.kinematic_plant(None) == kin.KinematicParams() == me.kinematic_plant("nominal")


def test_zero_lag_leaves_dynamic_step_unchanged():
    s = np.array([0.0, 0.0, 0.1, 10.0, 0.2, 0.05, 0.02])
    a = dyn.step(s, 0.1, 0.0, 0.05, dyn.DynamicParams())
    b = dyn.step(s, 0.1, 0.0, 0.05, dyn.DynamicParams(steer_lag=0.0))
    assert np.array_equal(a, b)


def test_zero_lag_leaves_kinematic_step_unchanged():
    s = np.array([0.0, 0.0, 0.1, 2.0, 0.02])
    assert np.array_equal(kin.step(s, 0.1, 0.0, 0.05), kin.step(s, 0.1, 0.0, 0.05, kin.KinematicParams(steer_lag=0.0)))


def test_lag_follows_first_order_response_when_rate_limit_is_not_active():
    p = kin.KinematicParams(steer_lag=0.2, max_steer_rate=100.0)
    s = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    dt, n = 0.01, 100
    for _ in range(n):
        s = kin.step(s, 0.1, 0.0, dt, p)
    assert s[4] == pytest.approx(0.1 * (1 - np.exp(-n * dt / 0.2)), rel=1e-9)


def test_dynamic_lag_slows_the_steering_angle():
    s = np.zeros(7)
    s[dyn.VX] = 10.0
    lagged = dyn.DynamicParams(steer_lag=0.2, max_steer_rate=100.0)
    free = dyn.DynamicParams(max_steer_rate=100.0)
    assert 0.0 < dyn.step(s, 0.1, 0.0, 0.05, lagged)[dyn.DELTA] < dyn.step(s, 0.1, 0.0, 0.05, free)[dyn.DELTA]


def test_dynamic_cases_change_what_they_say():
    nom = dyn.DynamicParams()
    heavy = me.dynamic_plant("mass_p20")
    assert heavy.mass == pytest.approx(1.2 * nom.mass) and heavy.inertia == pytest.approx(1.2 * nom.inertia)
    assert heavy.cf == nom.cf and heavy.steer_lag == 0.0
    soft = me.dynamic_plant("stiff_m30")
    assert soft.cf == pytest.approx(0.7 * nom.cf) and soft.cr == pytest.approx(0.7 * nom.cr) and soft.mass == nom.mass
    assert me.dynamic_plant("lag_0.2").steer_lag == 0.2
    comb = me.dynamic_plant("combined")
    assert (comb.mass, comb.cf, comb.steer_lag) == pytest.approx((1.2 * nom.mass, 0.7 * nom.cf, 0.1))


def test_kinematic_wheelbase_case():
    assert me.kinematic_plant("wheelbase_p5").wheelbase == pytest.approx(2.7 * 1.05)


def test_road_nominal_case_reproduces_the_task_14_row():
    task = ("road_s_curve_v10", "Dynamic LQR", 3)
    a, b = road.run_one(task), road.run_one(task + ("nominal",))
    b.pop("case")
    assert a == b


def test_parking_nominal_case_reproduces_the_task_14_row():
    names = parking.path_names()
    task = (names[0], "Pure Pursuit", 0.5, 1)
    a, b = parking.run_one(task), parking.run_one(task + ("nominal",))
    b.pop("case")
    assert a == b


def test_controller_is_built_on_nominal_parameters_in_every_case():
    # the case must reach the plant only: the same start row, different outcome
    a = road.run_one(("road_curve_v10", "Dynamic MPC", 0, "nominal"))
    b = road.run_one(("road_curve_v10", "Dynamic MPC", 0, "stiff_m30"))
    assert (a["e0"], a["dpsi0"]) == (b["e0"], b["dpsi0"])
    assert a["rms_lateral"] != b["rms_lateral"]


def test_summary_pairs_against_nominal():
    rows = [road.run_one(("road_curve_v5", "Pure Pursuit", 0, k)) for k in ("nominal", "lag_0.2")]
    s = {x["case"]: x for x in me.summarize(rows)}
    assert s["nominal"]["ratio"] == pytest.approx(1.0) and s["nominal"]["paired_change"] == 0.0
    assert s["lag_0.2"]["ratio"] == pytest.approx(rows[1]["rms_lateral"] / rows[0]["rms_lateral"])


@pytest.mark.skipif(not (RESULTS / "benchmark_road.csv").exists(), reason="Task 14 results not present")
def test_nominal_case_in_model_error_matches_frozen_benchmark():
    frozen = {(r["path"], r["controller"], r["seed"]): float(r["rms_lateral"])
              for r in csv.DictReader(open(RESULTS / "benchmark_road.csv"))}
    for k in [("road_curve_v20", "Stanley", "2"), ("road_double_shift_v5", "Dynamic MPC", "4")]:
        assert road.run_one((k[0], k[1], int(k[2]), "nominal"))["rms_lateral"] == pytest.approx(frozen[k], abs=1e-5)
