"""Run the model-error cases (Task 15) on both families; write results, summary and settings."""
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from trackloop import harness as parking
from trackloop import harness_road as road
from trackloop import model_error as me
from trackloop.results_io import environment, git, write_csv

OUT = Path("data/results")
FAMILIES = {
    "road": (road, me.DYNAMIC_CASES, "model_error_road.csv"),
    "parking": (parking, me.KINEMATIC_CASES, "model_error_parking.csv"),
}


def run_family(name, pool):
    module, cases, filename = FAMILIES[name]
    tasks = module.all_tasks(cases=list(cases))
    rows = list(pool.map(module.run_one, tasks, chunksize=8))
    write_csv(OUT / filename, rows)
    summary = [{"family": name, **{k: round(v, 5) if isinstance(v, float) else v for k, v in s.items()}}
               for s in me.summarize(rows)]
    return rows, summary


def show(name, rows, summary):
    errors = sum(str(r["status"]).startswith("error") for r in rows)
    print(f"\n{name}: {len(rows)} rows, {errors} errors, {sum(not r.get('finished') for r in rows)} unfinished")
    cases = list(dict.fromkeys(s["case"] for s in summary))
    print("median paired ratio (case RMS / nominal RMS); (n) = runs that did not finish")
    print(f"{'controller':16}{'speed':>6}" + "".join(f"{k:>14}" for k in cases))
    for c in dict.fromkeys(s["controller"] for s in summary):
        for v in sorted({s["speed"] for s in summary}):
            cells = []
            for k in cases:
                s = next(x for x in summary if x["controller"] == c and x["speed"] == v and x["case"] == k)
                bad = s["runs"] - s["finished"]
                cells.append(f"{s['ratio']:.2f}" + (f" ({bad})" if bad else ""))
            print(f"{c:16}{v:>6}" + "".join(f"{x:>14}" for x in cells))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    allsum, bad = [], 0
    with ProcessPoolExecutor(os.cpu_count() or 1) as pool:
        for name in FAMILIES:
            rows, summary = run_family(name, pool)
            allsum += summary
            show(name, rows, summary)
            bad += sum(str(r["status"]).startswith("error") for r in rows)
    write_csv(OUT / "model_error_summary.csv", allsum)
    status = git("status", "--porcelain", "--", "src", "scripts", "data/paths")
    cfg = {"dynamic_cases": me.DYNAMIC_CASES, "kinematic_cases": me.KINEMATIC_CASES,
           "road": road.settings(), "parking": parking.settings(),
           "git_commit": git("rev-parse", "HEAD"), "git_dirty": bool(status), "environment": environment()}
    (OUT / "model_error_settings.json").write_text(json.dumps(cfg, indent=2, default=str) + "\n")
    return 1 if bad else 0


if __name__ == "__main__":
    t = time.time()
    code = main()
    print(f"\n{time.time() - t:.1f} s")
    sys.exit(code)
