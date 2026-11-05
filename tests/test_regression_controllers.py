"""Frozen regression runs: each of the four controllers on two reference paths per family, seed 0.

tests/reference_runs.json holds the scores of these runs from the committed code. A change that moves a score by
more than the tolerance fails here; if the change is intended, regenerate the file and say why in the DEVLOG.
The same runs are also checked against the committed benchmark CSVs, so the numbers in the README trace to the code.
"""
import csv
import json
from pathlib import Path

import pytest

from trackloop import harness as parking
from trackloop import harness_road as road

HERE = Path(__file__).resolve().parent
RESULTS = HERE.parent / "data" / "results"
REFERENCE = json.loads((HERE / "reference_runs.json").read_text())
FIELDS = ("rms_lateral", "max_lateral", "steering_effort")


def _run(ref):
    if ref["family"] == "road":
        return road.run_one((ref["path"], ref["controller"], ref["seed"]))
    return parking.run_one((ref["path"], ref["controller"], ref["speed"], ref["seed"]))


def _ids():
    return [f"{r['family']}-{r['path']}-{r['controller'].replace(' ', '_')}" for r in REFERENCE]


def test_reference_covers_every_controller_in_both_families():
    assert {(r["family"], r["controller"]) for r in REFERENCE} == \
        {("road", c) for c in road.CONTROLLERS} | {("parking", c) for c in parking.CONTROLLERS}
    assert len(REFERENCE) == 16


@pytest.mark.parametrize("ref", REFERENCE, ids=_ids())
def test_run_reproduces_frozen_scores(ref):
    row = _run(ref)
    assert bool(row["finished"]) == ref["finished"]
    for key in FIELDS:
        assert row[key] == pytest.approx(ref[key], rel=5e-3, abs=2e-5), key


@pytest.mark.parametrize("family, name", [("road", "benchmark_road.csv"), ("parking", "benchmark_parking.csv")])
def test_reference_runs_match_the_committed_benchmark_csv(family, name):
    path = RESULTS / name
    if not path.exists():
        pytest.skip(f"{name} not present")
    with open(path) as fh:
        rows = {(r["path"], r["controller"], float(r["speed"]), int(r["seed"])): r for r in csv.DictReader(fh)}
    checked = 0
    for ref in (r for r in REFERENCE if r["family"] == family):
        key = (ref["path"], ref["controller"], float(ref["speed"]), ref["seed"])
        assert key in rows, key
        for field in FIELDS:
            assert float(rows[key][field]) == pytest.approx(ref[field], rel=5e-3, abs=2e-5), (key, field)
        checked += 1
    assert checked == 8
