"""Run the parking-family benchmark and write the results table.

Writes data/results/benchmark_parking.csv (one row per path, controller, speed, seed),
data/results/benchmark_parking_settings.json (everything that fixes the results, with the
git commit and the package versions) and data/results/benchmark_parking_summary.csv. Runs are
independent, so they run in parallel; the table does not depend on the number of workers.
Usage: python scripts/run_benchmark_parking.py [--workers N]
"""
import argparse
import csv
import importlib.metadata as md
import json
import os
import platform
import subprocess
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from trackloop import harness

OUT = Path("data/results")


def git(*args):
    try:
        return subprocess.run(["git", *args], capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return "unknown"


def version(package):
    try:
        return md.version(package)
    except md.PackageNotFoundError:
        return "not installed"


def environment():
    """Interpreter and package versions the results were produced with."""
    return {"python": platform.python_version(),
            **{p: version(p) for p in ("numpy", "scipy", "osqp", "control", "matplotlib")}}


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for r in rows for k in r)), restval="")
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    args = ap.parse_args()
    tasks = harness.all_tasks()
    print(f"{len(tasks)} runs on {args.workers} workers")
    with ProcessPoolExecutor(args.workers) as pool:
        rows = list(pool.map(harness.run_one, tasks, chunksize=8))
    OUT.mkdir(parents=True, exist_ok=True)
    write_csv(OUT / "benchmark_parking.csv", rows)
    summary = harness.summarize(rows)
    write_csv(OUT / "benchmark_parking_summary.csv", summary)
    meta = harness.settings()
    meta["git_commit"] = git("rev-parse", "HEAD")
    meta["git_dirty"] = bool(git("status", "--porcelain", "--untracked-files=no"))
    meta["environment"] = environment()
    (OUT / "benchmark_parking_settings.json").write_text(json.dumps(meta, indent=2) + "\n")
    bad = [r for r in rows if r["status"] != "ok"]
    print(f"{len(rows)} rows, {len(bad)} errors, "
          f"{sum(1 for r in rows if not r.get('finished'))} unfinished, "
          f"{sum(1 for r in rows if not r['drivable'])} not drivable (curvature or lateral acceleration), "
          f"{sum(1 for r in rows if not r['feasible'])} fail at least one of the three checks")
    print("%-9s %-14s %6s %5s %8s %9s %9s" % ("scope", "controller", "speed", "runs", "finished", "med rms", "p90 max"))
    for s in summary:
        print("%-9s %-14s %6.1f %5d %8d %9.3f %9.3f" % (s["scope"], s["controller"], s["speed"], s["runs"],
                                                       s["finished"], s["median_rms"], s["p90_max"]))


if __name__ == "__main__":
    main()
