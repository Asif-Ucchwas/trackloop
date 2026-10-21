import json

import numpy as np

from trackloop.testset import (DATA_DIR, MANIFEST, _sha256, build_manifest, load_manifest,
                               load_test_set, path_ids)


def test_manifest_lists_every_exported_path_and_nothing_else():
    manifest = load_manifest()
    ids = [e["id"] for e in manifest["paths"]]
    assert ids == path_ids() and len(set(ids)) == len(ids)
    assert manifest["count"] == len(ids) == 19


def test_both_sources_and_both_planners_are_present():
    groups = {e["group"] for e in load_manifest()["paths"]}
    assert groups == {"thesis_hybrid", "thesis_theta_star_tight", "reeds_shepp"}


def test_recorded_hashes_match_the_files_on_disk():
    for e in load_manifest()["paths"]:
        assert e["sha256"]["npz"] == _sha256(DATA_DIR / f"{e['id']}.npz")
        assert e["sha256"]["json"] == _sha256(DATA_DIR / f"{e['id']}.json")


def test_manifest_on_disk_equals_a_fresh_build():
    assert json.loads(json.dumps(build_manifest())) == load_manifest()
    assert MANIFEST.read_text().endswith("\n")


def test_loaded_paths_match_the_recorded_geometry():
    manifest = load_manifest()
    loaded = load_test_set()
    assert [i for i, _ in loaded] == [e["id"] for e in manifest["paths"]]
    for (_, path), e in zip(loaded, manifest["paths"]):
        assert len(path) == e["samples"]
        assert round(float(path.s[-1]), 4) == e["length_m"]
        assert np.all(np.isfinite(path.x)) and np.all(np.diff(path.s) > 0.0)
