"""Task 16 root-cause step 8: why is road_curve_v20 seed 2 the worst start for every controller?

For each seed on road_curve_v20 and each road controller, prints the start (offset e0, heading error
dpsi), the largest rear-axle error, the time it occurs, and the error 0.5 s and 1 s into the run.
The column 'drift 1s' is e0 + v*sin(dpsi)*1: where the vehicle would be after 1 s if its heading error
carried it away and nothing steered it. Evidence only.
"""
import math

import numpy as np

from trackloop import dynamic_runner as dr
from trackloop import harness_road as hr
from trackloop.metrics import errors_on_path, rear_axle
from trackloop.models import dynamic as dyn
from trackloop.models.dynamic import DynamicParams

PATH, PLANT, SEEDS = "road_curve_v20", DynamicParams(), (0, 1, 2, 3, 4)

if __name__ == "__main__":
    index, path, info = hr._road_set()[PATH]
    v = float(info["design_speed_mps"])
    print(f"{PATH}, design speed {v} m/s")
    print(f"{'seed':>4s} {'e0 (m)':>8s} {'dpsi (rad)':>10s} {'drift 1s':>9s} | {'controller':14s} {'max |e| (m)':>11s} {'at t (s)':>9s} {'e 0.5s':>8s} {'e 1s':>8s}")
    for seed in SEEDS:
        e0, dpsi = hr.start_condition(seed, index)
        for c in hr.CONTROLLERS:
            res = dr.run(path, hr.CONTROLLERS[c](path), v, e0, dpsi, params=PLANT)
            s = res.states
            xr, yr = rear_axle(s[:, dyn.X], s[:, dyn.Y], s[:, dyn.PSI], PLANT.lr)
            cte, _ = errors_on_path(path, xr, yr, s[:, dyn.PSI])
            k = int(np.argmax(np.abs(cte)))
            at = lambda t: cte[min(int(round(t / dr.DT)), len(cte) - 1)]
            print(f"{seed:4d} {e0:8.3f} {dpsi:10.3f} {e0 + v * math.sin(dpsi):9.2f} | {c:14s} {abs(cte[k]):11.3f} {res.time[k]:9.2f} {at(0.5):8.3f} {at(1.0):8.3f}")

    print()
    print("Which part of the start matters? Seed 2's e0 and dpsi taken one at a time, together, and with a sign flipped")
    e0s, d0s = hr.start_condition(2, index)
    print(f"{'e0':>7s} {'dpsi':>7s} | " + " ".join(f"{c[:12]:>16s}" for c in hr.CONTROLLERS))
    for e0, dpsi in ((e0s, 0.0), (0.0, d0s), (e0s, d0s), (e0s, -d0s), (-e0s, d0s)):
        cells = []
        for c in hr.CONTROLLERS:
            res = dr.run(path, hr.CONTROLLERS[c](path), v, e0, dpsi, params=PLANT)
            s = res.states
            xr, yr = rear_axle(s[:, dyn.X], s[:, dyn.Y], s[:, dyn.PSI], PLANT.lr)
            cte, _ = errors_on_path(path, xr, yr, s[:, dyn.PSI])
            k = int(np.argmax(np.abs(cte[: int(3.0 / dr.DT)])))
            cells.append(f"{abs(cte[k]):7.3f} m @{res.time[k]:4.2f}s")
        print(f"{e0:7.3f} {dpsi:7.3f} | " + " ".join(f"{x:>16s}" for x in cells))
    print("(largest error within the first 3 s)")
