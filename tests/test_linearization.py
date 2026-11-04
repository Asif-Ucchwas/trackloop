import pytest

from trackloop import linearization as lz


def test_small_offset_gap_is_tiny_for_every_controller():
    for controller in ("Pure Pursuit", "Stanley", "Dynamic LQR", "Dynamic MPC"):
        gap, _, _ = lz.compare(controller, 10.0, 0.1, 0.05)
        assert gap < 1e-3


def test_the_rate_limit_causes_most_of_the_gap_for_stanley():
    default, rate, _ = lz.compare("Stanley", 5.0, 0.0, 1.0)
    freed, freed_rate, _ = lz.compare("Stanley", 5.0, 0.0, 1.0, free_plant_rate=True)
    assert default > 0.03 and freed < 0.01 and freed < default / 5
    assert rate == pytest.approx(0.8, abs=1e-6) and freed_rate > 2.0        # demanded rate far above the 0.8 limit


def test_the_mpc_gap_needs_both_rate_limits_removed():
    default, _, _ = lz.compare("Dynamic MPC", 10.0, 0.1, 1.0)
    plant_only, _, _ = lz.compare("Dynamic MPC", 10.0, 0.1, 1.0, free_plant_rate=True)
    controller_only, _, _ = lz.compare("Dynamic MPC", 10.0, 0.1, 1.0, free_controller_limits=True)
    both, _, _ = lz.compare("Dynamic MPC", 10.0, 0.1, 1.0, free_plant_rate=True, free_controller_limits=True)
    assert plant_only == pytest.approx(default, rel=0.05)       # the MPC's own constraint binds first
    assert controller_only < default and both < 0.01
