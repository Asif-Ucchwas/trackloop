"""Numbers quoted in the README, recomputed from the committed result CSVs.

docs/TRACEABILITY.md maps each README claim to its source; this file checks the ones that live in a CSV.
"""
import csv
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "data" / "results"
README = (ROOT / "README.md").read_text()
TRACE = (ROOT / "docs" / "TRACEABILITY.md").read_text()
PATH = re.compile(r"(?<![\w~/.-])((?:scripts|docs|data|src|tests)/[\w./-]*[\w/])")


def rows(name):
    with open(RESULTS / name) as fh:
        return list(csv.DictReader(fh))


def pick(rs, **kw):
    out = [r for r in rs if all(str(r[k]) == str(v) or (_num(r[k]) and _num(v) and float(r[k]) == float(v)) for k, v in kw.items())]
    assert len(out) == 1, (kw, len(out))
    return out[0]


def _num(x):
    try:
        float(x)
        return True
    except (TypeError, ValueError):
        return False


def test_every_path_in_the_traceability_table_exists():
    paths = sorted({m.group(1) for m in PATH.finditer(TRACE) if TRACE[m.end():m.end() + 1] != "*"})
    assert len(paths) >= 40
    assert [p for p in paths if not (ROOT / p).exists()] == []


def test_road_means_at_20_ms_in_the_readme():
    rs = rows("mc_summary.csv")
    for controller, value in (("Pure Pursuit", 0.257), ("Stanley", 0.198), ("Dynamic LQR", 0.045), ("Dynamic MPC", 0.054)):
        assert float(pick(rs, family="road", scope="all", controller=controller, speed=20.0)["mean_rms"]) == pytest.approx(value, abs=5e-4)


def test_parking_means_in_the_readme():
    rs = rows("mc_summary.csv")
    for speed, pp, lqr in ((1.5, 0.084, 0.132), (3.0, 0.109, 0.286)):
        assert float(pick(rs, family="parking", scope="all", controller="Pure Pursuit", speed=speed)["mean_rms"]) == pytest.approx(pp, abs=5e-4)
        assert float(pick(rs, family="parking", scope="all", controller="Kinematic LQR", speed=speed)["mean_rms"]) == pytest.approx(lqr, abs=5e-4)


def test_every_monte_carlo_cell_meets_the_five_percent_target():
    rs = rows("mc_summary.csv")
    assert len(rs) == 36
    assert all(r["meets_target"] == "True" for r in rs)
    assert max(float(r["rel_half_width"]) for r in rs) == pytest.approx(0.049, abs=5e-4)


def test_phase_margins_at_10_ms_with_0p1_s_lag():
    rs = rows("margins_vs_speed.csv")
    expected = {"Pure Pursuit": 33, "Stanley": 37, "Dynamic LQR": 54, "Dynamic MPC": 60}
    for controller, degrees in expected.items():
        assert float(pick(rs, controller=controller, lag=0.1, speed=10.0)["phase_margin_deg"]) == pytest.approx(degrees, abs=0.5)
    assert "MPC 60 degrees" in README and "60 for MPC" in README


def test_linear_model_gap_at_a_2_m_offset_is_16_to_161_percent():
    rs = rows("linear_loops_validation.csv")
    for controller, share in (("Pure Pursuit", 22), ("Stanley", 16), ("Dynamic LQR", 28), ("Dynamic MPC", 161)):
        worst = max(float(r["share_of_offset"]) for r in rs if r["controller"] == controller and float(r["offset"]) == 2.0)
        assert 100 * worst == pytest.approx(share, abs=0.5)


def test_removing_the_rate_limit_leaves_a_gap_of_0p4_to_2p6_percent():
    rs = rows("linearization_limits.csv")
    shares = []
    for r in rs:
        if float(r["offset"]) != 2.0:
            continue
        if (r["controller"] == "Dynamic MPC" and r["variant"] == "both removed") or \
                (r["controller"] != "Dynamic MPC" and r["variant"] == "plant rate limit removed"):
            shares.append(100 * float(r["worst_gap_share"]))
    assert len(shares) >= 4
    assert min(shares) == pytest.approx(0.4, abs=0.05) and max(shares) == pytest.approx(2.6, abs=0.05)


def test_stability_limit_table_in_the_readme():
    rs = [r for r in rows("stability_limits.csv") if r["linear_limit"]]
    got = {(r["controller"], float(r["lag"])): float(r["linear_limit"]) for r in rs}
    want = {("Pure Pursuit", 0.1): 268.5, ("Pure Pursuit", 0.2): 134.2, ("Stanley", 0.0): 166.7, ("Stanley", 0.1): 60.7,
            ("Stanley", 0.2): 55.0, ("Dynamic LQR", 0.2): 92.1, ("Dynamic MPC", 0.2): 63.2}
    assert set(got) == set(want)
    for key, value in want.items():
        assert got[key] == pytest.approx(value, abs=0.05)
    assert sum(r["kind"] == "small" for r in rows("stability_limits.csv")) == 38


def test_identification_numbers_in_the_readme():
    rs = rows("identification.csv")
    assert len(rs) == 6
    assert max(abs(float(r["error_pct"])) for r in rs) < 0.2
    std = [float(r["mc_predicted_std_pct"]) for r in rs]
    assert round(min(std), 2) == 0.11 and round(max(std), 2) == 0.21
    assert all(0.93 <= float(r["mc_coverage_95"]) <= 0.97 for r in rs)


def test_controllers_on_the_identified_model_at_20_ms_combined():
    rs = rows("identified_design.csv")
    for controller, wrong, identified in (("Dynamic LQR", 2.6, 1.4), ("Dynamic MPC", 2.8, 1.6)):
        for variant, value in (("nominal", wrong), ("identified", identified)):
            got = float(pick(rs, controller=controller, speed=20.0, case="combined", variant=variant)["rms_ratio_vs_nominal_plant"])
            assert got == pytest.approx(value, abs=0.05)
