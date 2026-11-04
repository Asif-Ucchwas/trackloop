import importlib.util
from pathlib import Path

SPEC = importlib.util.spec_from_file_location("make_benchmarks_md", Path("scripts/make_benchmarks_md.py"))
mod = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mod)


def summary_row(fam, scope, c, v, mean, half, finished=10, runs=10, ok=True):
    return {"family": fam, "scope": scope, "controller": c, "speed": str(v), "seeds": "5", "runs": str(runs),
            "finished": str(finished), "mean_rms": str(mean), "half_width": str(half),
            "rel_half_width": str(half / mean), "meets_target": str(ok), "n_required": "3"}


SUMMARY = [summary_row("road", "all", "A", 5.0, 0.1, 0.002), summary_row("road", "all", "A", 10.0, 0.2, 0.004),
           summary_row("road", "all", "B", 5.0, 0.2, 0.002), summary_row("road", "all", "B", 10.0, 0.4, 0.004)]
PAIRS = [{"family": "road", "scope": "all", "a": "A", "b": "B", "speed": "5.0", "difference": "-0.1",
          "half_width": "0.001", "excludes_zero": "True"}]
CFG = {"git_commit": "abcdef1234567", "git_dirty": False, "n_seeds": {"road": 5}, "target_rel_half_width": 0.05}


def test_cells_show_mean_and_half_width():
    t = mod.speed_table(SUMMARY)
    assert "| A | 0.1000 +- 0.0020 | 0.2000 +- 0.0040 |" in t


def test_pair_row_has_relative_size_to_b():
    row = mod.pair_table(PAIRS, SUMMARY)[2]
    assert "-0.1000 +- 0.0010" in row and "-50.0%" in row and row.endswith("yes |")


def test_render_reports_commit_seeds_and_target():
    text = mod.render(SUMMARY, PAIRS, CFG)
    assert "abcdef1" in text and "road 5" in text and "cells missing the target: 0 of 4" in text
    assert "0 did not finish" in text and "A negative difference means a has the lower error." in text


def test_render_counts_a_cell_missing_the_target():
    bad = SUMMARY + [summary_row("road", "all", "C", 5.0, 0.1, 0.02, ok=False)]
    assert "cells missing the target: 1 of 5" in mod.render(bad, PAIRS, CFG)


def test_relative_column_uses_the_mean_of_the_same_scope():
    rows = [summary_row("parking", "all", "A", 1.0, 0.2, 0.002), summary_row("parking", "all", "B", 1.0, 0.4, 0.004),
            summary_row("parking", "drivable", "A", 1.0, 0.2, 0.002), summary_row("parking", "drivable", "B", 1.0, 0.8, 0.004)]
    pairs = [{"family": "parking", "scope": s, "a": "A", "b": "B", "speed": "1.0", "difference": "-0.2",
              "half_width": "0.001", "excludes_zero": "True"} for s in ("all", "drivable")]
    all_row = mod.pair_table([pairs[0]], rows)[2]
    drivable_row = mod.pair_table([pairs[1]], rows)[2]
    assert "-50.0%" in all_row and "-25.0%" in drivable_row


def test_render_counts_pairs_that_exclude_zero_and_names_the_smallest():
    text = mod.render(SUMMARY, PAIRS, CFG)
    assert "1 of 1 paired differences exclude zero, the smallest being 50.0% of the error." in text


def test_render_states_the_rerun_history():
    assert "rerun with 150 seeds" in mod.render(SUMMARY, PAIRS, CFG)


def test_render_points_to_the_root_cause_note():
    text = mod.render(SUMMARY, PAIRS, CFG)
    assert "docs/notes/root_cause_note.md" in text and "No cause is claimed for any other difference" in text
