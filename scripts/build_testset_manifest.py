"""Write data/paths/manifest.json: the frozen test set (every exported path, with hashes)."""
from trackloop.testset import write_manifest


def main():
    m = write_manifest()
    print("%-32s %-22s %7s %8s %6s" % ("id", "group", "len m", "max |k|", "cusps"))
    for e in m["paths"]:
        print("%-32s %-22s %7.1f %8.3f %6d" % (e["id"], e["group"], e["length_m"],
                                               e["max_abs_curvature"], e["cusps"]))
    print(f"{m['count']} paths written to data/paths/manifest.json")


if __name__ == "__main__":
    main()
