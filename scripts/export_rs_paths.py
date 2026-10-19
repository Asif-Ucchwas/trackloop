"""Export Reeds-Shepp reference paths from MultiPlan into the TrackLoop path format.

Poses are (x, y, heading in degrees) in metres. Before saving, every path must pass
three checks: it ends on the requested goal pose, its length equals MultiPlan's, and
its end agrees with MultiPlan's own sampler. Writes data/paths/rs_<name>.npz/.json.
"""
import argparse
import math
import platform
import subprocess
import sys
from pathlib import Path as FilePath

from trackloop.planner_export import save_path
from trackloop.rs_export import rs_to_path

MULTIPLAN = "~/projects/multiplan"
RADIUS = 5.0
CASES = {
    "ahead": ((0, 0, 0), (40, 0, 0)),
    "lane_shift": ((0, 0, 0), (40, 8, 0)),
    "u_turn": ((0, 0, 0), (0, 12, 180)),
    "behind": ((0, 0, 0), (-10, 4, 0)),
    "parallel_park": ((0, 0, 0), (-6, -3, 0)),
    "reverse_in": ((0, 0, 0), (-12, 8, 90)),
    "side_slot": ((0, 0, 90), (6, 4, 90)),
    "three_point": ((0, 0, 0), (2, 6, 180)),
    "diag": ((0, 0, 20), (25, -15, -100)),
}


def git(repo, *args):
    out = subprocess.run(["git", "-C", str(repo), *args], capture_output=True, text=True)
    return out.stdout.strip()


def pose(p):
    return (float(p[0]), float(p[1]), math.radians(p[2]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--multiplan", default=MULTIPLAN)
    ap.add_argument("--radius", type=float, default=RADIUS)
    ap.add_argument("--out", default="data/paths")
    args = ap.parse_args()
    mp = FilePath(args.multiplan).expanduser()
    sys.path.insert(0, str(mp))
    from path_sampling import path_endpoint
    from reeds_shepp_path import shortest_reeds_shepp_path

    out = FilePath(args.out)
    out.mkdir(parents=True, exist_ok=True)
    commit = git(mp, "rev-parse", "--short", "HEAD")
    dirty = bool(git(mp, "status", "--porcelain"))
    print(f"MultiPlan {commit}{' (modified)' if dirty else ''}  radius {args.radius} m")
    print(f"{'case':14s} {'word':20s} {'length':>8s} {'cusps':>5s} {'pos err':>8s} {'head err':>8s}")
    for name, (a, b) in CASES.items():
        start, goal = pose(a), pose(b)
        length, word, rs = shortest_reeds_shepp_path(start, goal, args.radius)
        path, info = rs_to_path(start, rs.segments, args.radius, goal=goal)
        mpx, mpy, _ = path_endpoint(start, rs.segments, args.radius)
        mp_gap = math.hypot(mpx - path.x[-1], mpy - path.y[-1])
        if (info["end_position_error_m"] > 1e-6 or info["end_heading_error_rad"] > 1e-6
                or abs(info["length_m"] - length) > 1e-6 or mp_gap > 1e-3):
            raise SystemExit(f"{name}: conversion check failed: {info}, mp gap {mp_gap}")
        info.update(name=name, word=word, start=list(a), goal=list(b),
                    segments=[[t, float(v)] for t, v in rs.segments],
                    multiplan_commit=commit, multiplan_modified=dirty,
                    multiplan_length_m=float(length), multiplan_end_gap_m=float(mp_gap),
                    python=platform.python_version())
        save_path(path, info, out / f"rs_{name}")
        print(f"{name:14s} {word:20s} {info['length_m']:8.3f} {info['n_cusps']:5d} "
              f"{info['end_position_error_m']:8.1e} {info['end_heading_error_rad']:8.1e}")


if __name__ == "__main__":
    main()
