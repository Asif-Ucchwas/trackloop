import json

import numpy as np
import pytest

from trackloop import road_paths as rp
from trackloop.feasibility import check_path
from trackloop.planner_export import load_path
from trackloop.testset import (ROAD_DIR, ROAD_RULE, _sha256, build_manifest, load_manifest,
                               load_test_set, path_ids)

CASES = [(shape, v) for shape in rp.SHAPES for v in rp.SPEEDS]


def speed_of(name):
    return float(name.rsplit("_v", 1)[1])


def test_ids_follow_the_shape_and_speed():
    assert rp.path_id("curve", 5.0) == "road_curve_v5"
    assert rp.path_id("double_shift", 20.0) == "road_double_shift_v20"
    with pytest.raises(ValueError):
        rp.profile("figure_eight", 5.0)


@pytest.mark.parametrize("shape,v", CASES)
def test_peak_lateral_acceleration_is_the_same_at_every_speed(shape, v):
    path, _ = rp.build(shape, v)
    assert v * v * np.abs(path.curvature).max() == pytest.approx(rp.A_Y, rel=1e-12)


def test_curvature_ramps_at_the_set_lateral_jerk():
    for v in rp.SPEEDS:
        nodes = rp.profile("curve", v)
        s, k = nodes[:, 0], nodes[:, 1]
        i = int(np.argmax(k > 0)) - 1                      # start of the first ramp
        assert s[i + 1] - s[i] == pytest.approx(rp.ramp_length(v))
        assert (k[i + 1] - k[i]) / (s[i + 1] - s[i]) * v ** 3 == pytest.approx(rp.JERK)


@pytest.mark.parametrize("shape,v", CASES)
def test_heading_is_the_integral_of_the_curvature(shape, v):
    path, _ = rp.build(shape, v)
    mean_k = 0.5 * (path.curvature[1:] + path.curvature[:-1])
    err = np.abs(np.diff(path.heading) / np.diff(path.s) - mean_k)
    # Exact except in a sample interval that holds a kink of the curvature profile, where the
    # interval average differs from the mean of its end values by at most slope * ds / 8.
    kink_bound = rp.JERK / v ** 3 * rp.DS / 8.0
    assert err.max() <= kink_bound * (1.0 + 1e-6) and np.median(err) < 1e-12


@pytest.mark.parametrize("v", rp.SPEEDS)
def test_shapes_end_where_their_definition_says(v):
    curve, _ = rp.build("curve", v)
    assert curve.heading[-1] == pytest.approx(np.pi / 2, abs=1e-9)
    s_curve, _ = rp.build("s_curve", v)
    assert s_curve.heading[-1] == pytest.approx(0.0, abs=1e-9)
    assert np.abs(s_curve.heading).max() == pytest.approx(np.pi / 3, abs=1e-6)
    shift, _ = rp.build("lateral_shift", v)
    assert shift.heading[-1] == pytest.approx(0.0, abs=1e-9) and shift.y[-1] > 5.0
    double, _ = rp.build("double_shift", v)
    assert double.heading[-1] == pytest.approx(0.0, abs=1e-9) and double.y[-1] == pytest.approx(0.0, abs=1e-6)


def test_the_lateral_shift_is_nearly_the_same_at_every_speed():
    offsets = [rp.build("lateral_shift", v)[0].y[-1] for v in rp.SPEEDS]
    assert max(offsets) / min(offsets) < 1.10


@pytest.mark.parametrize("shape,v", [("curve", 5.0), ("double_shift", 20.0)])
def test_position_integration_is_accurate(shape, v):
    coarse, _ = rp.build(shape, v)
    fine, _ = rp.build(shape, v, fine_ds=0.001)
    assert np.abs(coarse.x - fine.x).max() < 1e-5 and np.abs(coarse.y - fine.y).max() < 1e-5


def test_the_road_set_is_frozen_and_complete():
    ids = path_ids(ROAD_DIR)
    manifest = load_manifest(ROAD_DIR / "manifest.json")
    assert [e["id"] for e in manifest["paths"]] == ids == sorted(rp.path_id(s, v) for s, v in CASES)
    assert manifest["count"] == 12 and manifest["rule"] == ROAD_RULE
    assert {e["group"] for e in manifest["paths"]} == {"road"}
    for e in manifest["paths"]:
        assert e["sha256"]["npz"] == _sha256(ROAD_DIR / f"{e['id']}.npz")
        assert e["sha256"]["json"] == _sha256(ROAD_DIR / f"{e['id']}.json")
    assert json.loads(json.dumps(build_manifest(ROAD_DIR, ROAD_RULE))) == manifest


def test_the_frozen_files_still_match_what_the_code_generates():
    for name, stored in load_test_set(ROAD_DIR):
        info = json.loads((ROAD_DIR / f"{name}.json").read_text())
        fresh, _ = rp.build(info["shape"], info["design_speed_mps"], info["peak_lateral_accel_mps2"],
                            info["lateral_jerk_mps3"])
        for a in ("x", "y", "heading", "curvature", "s"):
            assert np.allclose(getattr(stored, a), getattr(fresh, a), atol=1e-6), (name, a)


def test_every_road_path_is_forward_only_and_passes_every_check_at_its_design_speed():
    for name, path in load_test_set(ROAD_DIR):
        assert np.all(path.direction == 1.0)
        f = check_path(path, speed=speed_of(name))
        assert f.curvature_ok and f.rate_ok and f.lateral_ok, name
        assert f.max_lateral_accel == pytest.approx(rp.A_Y, rel=1e-9)


def test_the_parking_test_set_is_unchanged_by_the_road_family():
    ids = path_ids()
    assert len(ids) == 19 and not any(i.startswith("road_") for i in ids)
    assert load_path(ROAD_DIR / "road_curve_v5").x.size > 0
