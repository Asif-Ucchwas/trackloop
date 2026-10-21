"""Check every test-set path against the car's curvature limit and steering rate.

Writes data/results/feasibility.csv. The steering-rate check is made at the benchmark cruise
speed (1.5 m/s). The car is the placeholder one in models/kinematic.py.
"""
import csv
from pathlib import Path

from trackloop.feasibility import check_path
from trackloop.testset import load_test_set

OUT = Path("data/results/feasibility.csv")
SPEED = 1.5


def main():
    rows = []
    print("%-32s %8s %6s %7s %7s %9s %6s" % ("id", "max |k|", "over", "events", "event", "lag m", "ok"))
    for name, path in load_test_set():
        r = check_path(path, speed=SPEED)
        rows.append({"id": name, **r.as_dict()})
        print("%-32s %8.3f %6d %7d %7.3f %9.2f %6s" % (
            name, r.max_abs_curvature, r.samples_over_limit, r.steering_events,
            r.largest_event_rad, r.worst_lag_distance, "yes" if r.feasible else "NO"))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    ok = sum(1 for r in rows if r["feasible"])
    print(f"{ok} of {len(rows)} paths pass both checks at {SPEED} m/s; wrote {OUT}")


if __name__ == "__main__":
    main()
