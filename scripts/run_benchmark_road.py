"""Run the road-family benchmark (Task 14) and write results, summary and settings."""
import csv
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from trackloop import harness_road as hr
from trackloop.results_io import environment, git, write_csv

OUT = Path("data/results")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    tasks = hr.all_tasks()
    workers = min(len(tasks), os.cpu_count() or 1)
    with ProcessPoolExecutor(workers) as pool:
        rows = list(pool.map(hr.run_one, tasks, chunksize=4))
    write_csv(OUT / "benchmark_road.csv", rows)
    summary = hr.summarize(rows)
    write_csv(OUT / "benchmark_road_summary.csv", [{k: round(v, 5) if isinstance(v, float) else v
                                                    for k, v in s.items()} for s in summary])
    status = git("status", "--porcelain", "--", "src", "scripts", "data/paths")
    cfg = {**hr.settings(), "git_commit": git("rev-parse", "HEAD"), "git_dirty": bool(status),
           "environment": environment()}
    (OUT / "benchmark_road_settings.json").write_text(json.dumps(cfg, indent=2) + "\n")
    errors = sum(r["status"].startswith("error") for r in rows)
    unfinished = sum(not r.get("finished") for r in rows)
    print(f"{len(rows)} runs on {workers} workers")
    print(f"{len(rows)} rows, {errors} errors, {unfinished} unfinished, "
          f"{sum(not r['drivable'] for r in rows)} not drivable")
    print(f"{'controller':14}{'speed':>6}{'runs':>6}{'done':>6}{'med rms':>9}{'p90 max':>9}{'max slip':>9}")
    for s in summary:
        print(f"{s['controller']:14}{s['speed']:>6}{s['runs']:>6}{s['finished']:>6}"
              f"{s['median_rms']:>9.3f}{s['p90_max']:>9.3f}{s['max_slip_deg']:>9.2f}")
    return 1 if errors else 0


if __name__ == "__main__":
    t = time.time()
    code = main()
    print(f"{time.time() - t:.1f} s")
    sys.exit(code)
