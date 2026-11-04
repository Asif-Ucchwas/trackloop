"""Task 16 root-cause step 10: how many road runs exceed the tire friction limit?

Reads data/results/model_error_road.csv (peak front and rear slip angle per run, 5 seeds) and counts, per
controller, speed and case, the runs whose peak slip angle is above the angle at which the plant's linear
tire force reaches mu * Fz. That angle depends on the case: mu * Fz / C per axle, with Fz from the
static load split (mass changes it, stiffness changes C). No new simulation. Evidence only.
"""
import csv
from collections import defaultdict
from math import degrees

from trackloop.feasibility import G, MU
from trackloop.model_error import DYNAMIC_CASES, dynamic_plant

rows = list(csv.DictReader(open("data/results/model_error_road.csv")))
limits = {}
for case in ["nominal"] + [c for c in DYNAMIC_CASES if c != "nominal"]:
    p = dynamic_plant(None if case == "nominal" else case)
    L = p.lf + p.lr
    limits[case] = (degrees(MU * p.mass * G * p.lr / L / p.cf), degrees(MU * p.mass * G * p.lf / L / p.cr))

print("friction-limit slip angle by case (front deg, rear deg)")
for c, (f, r) in limits.items():
    print(f"  {c:10s} {f:5.2f} {r:5.2f}")
print()
cases = list(limits)
cell = defaultdict(lambda: [0, 0])
for r in rows:
    over = (float(r["max_slip_front_deg"]) > limits[r["case"]][0]) or (float(r["max_slip_rear_deg"]) > limits[r["case"]][1])
    k = (r["controller"], float(r["speed"]), r["case"])
    cell[k][0] += over
    cell[k][1] += 1
print("runs over the friction limit (front or rear), out of 20 per cell (4 paths x 5 seeds)")
print(f"{'controller':14s} {'m/s':>4s} " + " ".join(f"{c[:9]:>9s}" for c in cases))
for ctrl in dict.fromkeys(r["controller"] for r in rows):
    for v in (5.0, 10.0, 20.0):
        print(f"{ctrl:14s} {v:4.0f} " + " ".join(f"{cell[(ctrl, v, c)][0]:>9d}" for c in cases))
