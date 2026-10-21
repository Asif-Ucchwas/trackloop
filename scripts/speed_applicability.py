"""Which benchmark speeds can each test-set path be driven at, by lateral acceleration?

Writes data/results/speed_applicability.csv. A path is applicable at speed v when
v^2 * max|curvature| <= mu * g (mu = 0.8, a placeholder). Curvature and steering-rate
feasibility are separate checks (check_testset_feasibility.py).
"""
import csv
from pathlib import Path

from trackloop.feasibility import check_path
from trackloop.testset import load_test_set

OUT = Path("data/results/speed_applicability.csv")
SPEEDS = (1.5, 5.0, 10.0, 20.0)


def main():
    rows = []
    print("%-32s %8s %9s " % ("id", "max |k|", "v max") + " ".join("%5gm/s" % v for v in SPEEDS))
    for name, path in load_test_set():
        checks = {v: check_path(path, speed=v) for v in SPEEDS}
        first = checks[SPEEDS[0]]
        row = {"id": name, "max_abs_curvature": round(first.max_abs_curvature, 4),
               "max_feasible_speed": round(first.max_feasible_speed, 3)}
        for v in SPEEDS:
            row[f"ok_{v:g}"] = checks[v].lateral_ok
        rows.append(row)
        vmax = "no limit" if first.max_feasible_speed == float("inf") else "%.2f" % first.max_feasible_speed
        print("%-32s %8.3f %9s " % (name, first.max_abs_curvature, vmax)
              + " ".join("%8s" % ("yes" if row[f"ok_{v:g}"] else "NO") for v in SPEEDS))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print("paths drivable at each speed: " + ", ".join(
        "%g m/s: %d of %d" % (v, sum(r[f"ok_{v:g}"] for r in rows), len(rows)) for v in SPEEDS))


if __name__ == "__main__":
    main()
