"""Monte Carlo benchmark (Task 16): many seeds on both families, nominal plant, with intervals.

Writes compact per-run CSVs (the full row of any run can be regenerated from its path,
controller, speed and seed, because runs are deterministic), a summary of means with 95 percent
intervals, the paired controller differences, and the settings.
"""
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from trackloop import harness as parking
from trackloop import harness_road as road
from trackloop import monte_carlo as mc
from trackloop.results_io import environment, git, write_csv

OUT = Path("data/results")
N_SEEDS = 120
SEEDS = tuple(range(N_SEEDS))
KEEP = ("path", "controller", "speed", "seed", "drivable", "status", "finished", "rms_lateral", "max_lateral")
PAIRS = {
    "road": [("Dynamic LQR", "Dynamic MPC"), ("Pure Pursuit", "Dynamic LQR"), ("Stanley", "Dynamic LQR"),
             ("Pure Pursuit", "Stanley")],
    "parking": [("Kinematic LQR", "Kinematic MPC"), ("Pure Pursuit", "Kinematic LQR"),
                ("Stanley", "Kinematic LQR"), ("Pure Pursuit", "Stanley")],
}


def run_family(name, module, pool):
    rows = list(pool.map(module.run_one, module.all_tasks(seeds=SEEDS), chunksize=16))
    compact = [{k: r.get(k, "") for k in KEEP} for r in rows]
    write_csv(OUT / f"mc_{name}.csv", compact)
    return rows


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    summary, pairs, bad = [], [], 0
    with ProcessPoolExecutor(os.cpu_count() or 1) as pool:
        for name, module in (("road", road), ("parking", parking)):
            rows = run_family(name, module, pool)
            errors = sum(str(r["status"]).startswith("error") for r in rows)
            bad += errors
            scopes = {"all": rows}
            if name == "parking":   # some parking paths cannot be driven at the higher speeds
                scopes["drivable"] = [x for x in rows if x["drivable"]]
            for scope, sel in scopes.items():
                cells = mc.cell_table(sel)
                summary += [{"family": name, "scope": scope, **c} for c in cells]
                pairs += [{"family": name, "scope": scope, **p} for p in mc.pair_table(sel, PAIRS[name])]
                print(f"\n{name} [{scope}]: {len(sel)} runs, {errors} errors, {sum(not x.get('finished') for x in sel)} unfinished")
                print(f"{'controller':16}{'speed':>6}{'seeds':>6}{'mean RMS':>10}{'+-95%':>9}{'rel':>7}{'ok':>4}{'n needed':>9}")
                for c in cells:
                    print(f"{c['controller']:16}{c['speed']:>6g}{c['seeds']:>6}{c['mean_rms']:>10.4f}{c['half_width']:>9.4f}"
                          f"{c['rel_half_width']:>7.3f}{'y' if c['meets_target'] else 'N':>4}{c['n_required']:>9}")
                print("paired differences (a - b), mean RMS, 95% interval")
                for p in (x for x in pairs if x["family"] == name and x["scope"] == scope):
                    print(f"  {p['a']} - {p['b']} @ {p['speed']:g}: {p['difference']:+.4f} +- {p['half_width']:.4f}"
                          f"{'' if p['excludes_zero'] else '   (interval includes 0)'}")
    r = lambda rows: [{k: round(v, 6) if isinstance(v, float) else v for k, v in x.items()} for x in rows]
    write_csv(OUT / "mc_summary.csv", r(summary))
    write_csv(OUT / "mc_pairs.csv", r(pairs))
    status = git("status", "--porcelain", "--", "src", "scripts", "data/paths")
    cfg = {"seeds": [0, N_SEEDS - 1], "n_seeds": N_SEEDS, "target_rel_half_width": mc.TARGET, "level": mc.LEVEL,
           "pairs": PAIRS, "road": road.settings(), "parking": parking.settings(),
           "git_commit": git("rev-parse", "HEAD"), "git_dirty": bool(status), "environment": environment()}
    (OUT / "mc_settings.json").write_text(json.dumps(cfg, indent=2, default=str) + "\n")
    miss = [c for c in summary if not c["meets_target"]]
    print(f"\ncells missing the {mc.TARGET:.0%} target: {len(miss)} of {len(summary)}")
    return 1 if bad else 0


if __name__ == "__main__":
    t = time.time()
    code = main()
    print(f"\n{time.time() - t:.1f} s")
    sys.exit(code)
