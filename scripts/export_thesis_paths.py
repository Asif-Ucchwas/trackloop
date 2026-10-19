"""Task 9: export thesis planner paths into the common TrackLoop path format.

Runs the thesis Hybrid planner (A*-seeded corridor RRT*) and Theta* + tighten on seeded
50 x 50 maps under the exact collision rule (margin 0.05 cell), rounds the corners,
and saves each path with its provenance. Needs a clone of the thesis repository.
"""
import argparse
import json
import random
import subprocess
import sys
from pathlib import Path

import numpy as np

from trackloop.planner_export import export_path, save_path

START, GOAL, SIZE, DENSITY, MARGIN = (2, 2), (47, 47), 50, 0.20, 0.05
THESIS = "~/projects/hybrid-a-star-rrtstar-path-planning"


def thesis_commit(root):
    out = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"],
                         capture_output=True, text=True)
    return out.stdout.strip() or "unknown"


def hits(points, grid, line_free):
    """Number of consecutive point pairs whose segment touches an obstacle."""
    return sum(0 if line_free(a, b, grid) else 1 for a, b in zip(points, points[1:]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--thesis", default=THESIS)
    ap.add_argument("--cell", type=float, default=3.0)
    ap.add_argument("--radius", type=float, default=6.0)
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--out", default="data/paths")
    args = ap.parse_args()
    root = Path(args.thesis).expanduser()
    sys.path.insert(0, str(root))
    from baselines.astar_family import ThetaStarTight
    from planner_core import HybridAStar
    from planner_core.utils import (line_collision_free, make_grid,
                                    set_collision_mode, set_start_goal_free)
    set_collision_mode("exact", MARGIN)
    commit, out = thesis_commit(root), Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    planners = {"hybrid": HybridAStar().plan, "theta_star_tight": ThetaStarTight().plan}
    print("planner            seed verts length_m  min_r tight cut_m  wp_hits rd_hits")
    for seed in range(args.seeds):
        grid = make_grid(SIZE, DENSITY, seed=seed)
        set_start_goal_free(grid, START, GOAL)
        np.save(out / f"map_seed{seed}.npy", grid)
        for name, plan in planners.items():
            random.seed(seed)
            waypoints = plan(START, GOAL, grid)
            if waypoints is None:
                print(f"{name:<18} {seed:>4}  no path")
                continue
            path, info = export_path(waypoints, args.cell, args.radius)
            rounded = list(zip(path.x[::5] / args.cell, path.y[::5] / args.cell))
            info.update(
                planner=name, map_seed=seed, planner_seed=seed, density=DENSITY,
                grid_size=SIZE, start_cell=START, goal_cell=GOAL, thesis_commit=commit,
                collision_rule=f"exact, margin {MARGIN} cell",
                python=sys.version.split()[0],
                waypoints_cells=[[float(x), float(y)] for x, y in waypoints],
                waypoint_hits=hits(waypoints, grid, line_collision_free),
                rounded_hits=hits(rounded, grid, line_collision_free))
            save_path(path, info, out / f"thesis_{name}_seed{seed}")
            r = info["min_radius_m"]
            r = "-" if r is None else format(r, ".2f")
            print(f"{name:<18} {seed:>4} {info['n_vertices']:>5} "
                  f"{info['length_m']:>8.1f} {r:>6} {info['n_tight']:>5} "
                  f"{info['max_cut_m']:>5.2f} {info['waypoint_hits']:>8} "
                  f"{info['rounded_hits']:>7}")


if __name__ == "__main__":
    main()
