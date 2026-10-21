"""Export the road family to data/paths/road/ and write its manifest.

Run this once. The files are then frozen: no controller is run on a path before it is
exported, and the tests fail if a file changes. To change a setting, edit
trackloop/road_paths.py, re-export on purpose and say so in DEVLOG.md.
"""
from trackloop import road_paths as rp
from trackloop.feasibility import check_path
from trackloop.planner_export import save_path
from trackloop.testset import ROAD_DIR, ROAD_RULE, write_manifest


def main():
    print("%-30s %8s %8s %8s %8s" % ("id", "len m", "R min m", "end y m", "lat acc"))
    for shape in rp.SHAPES:
        for v in rp.SPEEDS:
            path, info = rp.build(shape, v)
            save_path(path, info, ROAD_DIR / info["name"])
            f = check_path(path, speed=v)
            print("%-30s %8.1f %8.1f %8.2f %8.3f %s" % (info["name"], info["length_m"], info["min_radius_m"],
                                                        info["end_y_m"], f.max_lateral_accel,
                                                        "ok" if f.feasible else "NOT FEASIBLE"))
    m = write_manifest(ROAD_DIR, ROAD_DIR / "manifest.json", ROAD_RULE)
    print(f"{m['count']} paths written to {ROAD_DIR / 'manifest.json'}")


if __name__ == "__main__":
    main()
