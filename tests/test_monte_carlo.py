import csv
import math
from pathlib import Path

import numpy as np
import pytest

from trackloop import monte_carlo as mc

RESULTS = Path("data/results")


def rows_for(values, controller="A", speed=1.0, finished=True):
    return [{"controller": controller, "speed": speed, "seed": s, "rms_lateral": v, "finished": finished}
            for s, v in enumerate(values)]


def test_mean_ci_matches_the_textbook_formula():
    v = [1.0, 2.0, 3.0, 4.0, 5.0]
    mean, half = mc.mean_ci(v)
    assert mean == pytest.approx(3.0)
    assert half == pytest.approx(2.7764451 * np.std(v, ddof=1) / math.sqrt(5), rel=1e-6)


def test_mean_ci_is_nan_with_one_value():
    assert math.isnan(mc.mean_ci([1.0])[1])


def test_seed_mean_averages_over_paths_and_skips_unfinished():
    rows = [{"controller": "A", "speed": 1.0, "seed": 0, "rms_lateral": 1.0, "finished": True},
            {"controller": "A", "speed": 1.0, "seed": 0, "rms_lateral": 3.0, "finished": "True"},
            {"controller": "A", "speed": 1.0, "seed": 0, "rms_lateral": 99.0, "finished": False},
            {"controller": "B", "speed": 1.0, "seed": 0, "rms_lateral": 7.0, "finished": True}]
    assert mc.seed_means(rows, "A", 1.0) == {0: 2.0}


def test_paired_difference_removes_the_shared_start_variation():
    shared = [1.0, 5.0, 2.0, 9.0, 4.0, 6.0]            # large seed-to-seed spread both controllers share
    a = {s: v + 0.1 for s, v in enumerate(shared)}
    b = {s: v for s, v in enumerate(shared)}
    diff, half = mc.paired_difference(a, b)
    assert diff == pytest.approx(0.1) and half == pytest.approx(0.0, abs=1e-12)
    assert mc.mean_ci(list(a.values()))[1] > 1.0        # the unpaired interval is far wider


def test_n_required_matches_the_formula():
    v = [0.8, 1.0, 1.2, 0.9, 1.1]
    cv = np.std(v, ddof=1) / np.mean(v)
    assert mc.n_required(v, target=0.05) == math.ceil((1.959964 * cv / 0.05) ** 2)


def test_more_seeds_narrow_the_interval():
    rng = np.random.default_rng(0)
    vals = rng.normal(1.0, 0.2, 400)
    assert mc.mean_ci(vals)[1] < mc.mean_ci(vals[:20])[1]


def test_cell_table_flags_the_target():
    tight = mc.cell_table(rows_for([1.00, 1.01, 0.99, 1.02, 0.98, 1.0]))[0]
    loose = mc.cell_table(rows_for([0.2, 2.0, 0.5, 1.8, 0.1, 1.3]))[0]
    assert tight["meets_target"] and not loose["meets_target"]
    assert tight["seeds"] == 6 and tight["finished"] == 6


def test_pair_table_excludes_zero_only_for_a_real_difference():
    rows = rows_for([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], "A") + rows_for([1.5, 2.5, 3.5, 4.5, 5.5, 6.5], "B") \
        + rows_for([1.0, 2.1, 2.9, 4.2, 4.8, 6.1], "C")
    p = {(x["a"], x["b"]): x for x in mc.pair_table(rows, [("A", "B"), ("A", "C")])}
    assert p[("A", "B")]["excludes_zero"] and not p[("A", "C")]["excludes_zero"]


@pytest.mark.skipif(not (RESULTS / "mc_road.csv").exists(), reason="Monte Carlo results not present")
def test_first_five_seeds_reproduce_the_task_14_rows():
    frozen = {(r["path"], r["controller"], r["seed"]): float(r["rms_lateral"])
              for r in csv.DictReader(open(RESULTS / "benchmark_road.csv"))}
    now = {(r["path"], r["controller"], r["seed"]): float(r["rms_lateral"])
           for r in csv.DictReader(open(RESULTS / "mc_road.csv")) if int(r["seed"]) < 5}
    assert set(frozen) == set(now)
    assert max(abs(frozen[k] - now[k]) for k in frozen) < 1e-5
