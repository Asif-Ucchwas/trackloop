import numpy as np

from trackloop.benchmark import BENCHMARK_SPEEDS
from trackloop.models.comparison import matched_turn
from trackloop.models.dynamic import DynamicParams

P = DynamicParams()
L = P.wheelbase
K = P.understeer_gradient


def test_ratio_matches_closed_form_at_moderate_and_high_speed():
    for v in (10.0, 15.0, 20.0, 25.0, 30.0):
        o = matched_turn(v)
        assert np.isclose(o["ratio"], L / (L + K * v**2), rtol=5e-3)


def test_kinematic_model_adequate_at_slowest_benchmark_speed():
    assert matched_turn(BENCHMARK_SPEEDS[0])["ratio"] >= 0.95


def test_kinematic_model_inadequate_at_fastest_benchmark_speed():
    assert matched_turn(BENCHMARK_SPEEDS[-1])["ratio"] < 0.7


def test_ratio_decreases_with_speed():
    speeds = (4.0, 5.0, 6.0, 8.0, 10.0, 15.0, 20.0, 25.0, 30.0)
    ratios = [matched_turn(v)["ratio"] for v in speeds]
    assert np.all(np.diff(ratios) < 0)


def test_sideslip_sign_flips_at_zero_sideslip_speed():
    v0 = np.sqrt(P.lr * L * P.cr / (P.mass * P.lf))
    assert np.isclose(v0, 15.0)
    assert abs(matched_turn(v0)["beta"]) < 0.01
    assert matched_turn(10.0)["beta"] > 0
    assert matched_turn(20.0)["beta"] < 0


def test_turn_beyond_steering_limit_is_reported():
    assert matched_turn(2.0) is None
