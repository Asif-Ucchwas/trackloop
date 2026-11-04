"""Monte Carlo statistics for the benchmark (Task 16): seed-level confidence intervals and sizing.

The random draw is the seed (it fixes the start offset and heading error on every path). The paths
are a fixed set, not a sample, so a result holds for these paths. For each controller and design
speed, each seed gives one number: the mean RMS lateral error over the paths of that seed (finished
runs only). The confidence interval is a Student t interval over the seeds, which is the standard
way to put an interval on a mean of independent draws. Controllers share the same starts for a
given seed and path, so a difference between two controllers is taken seed by seed (a paired
difference), which removes the start-to-start variation they have in common.

Sizing: for a target relative half-width h (0.05 means the interval is +-5 percent of the mean),
the number of seeds is n = (z * cv / h)^2, with cv the coefficient of variation of the seed means
and z = 1.96 for 95 percent. cv comes from a pilot and is itself uncertain with few seeds, so the
sizing is checked on the full run, not trusted.
"""
import math
from collections import defaultdict

import numpy as np
from scipy import stats

TARGET = 0.05       # relative half-width of the 95 percent interval on a controller's mean RMS
LEVEL = 0.95


def _finished(r):
    return r.get("finished") in (True, "True", "true", 1, "1")


def seed_means(rows, controller, speed, key="rms_lateral"):
    """{seed: mean of key over the finished runs of that seed} for one controller and speed."""
    by = defaultdict(list)
    for r in rows:
        if r["controller"] == controller and float(r["speed"]) == float(speed) and _finished(r):
            by[int(r["seed"])].append(float(r[key]))
    return {s: float(np.mean(v)) for s, v in sorted(by.items())}


def mean_ci(values, level=LEVEL):
    """(mean, half-width) of a Student t interval; half-width is nan with fewer than 2 values."""
    v = np.asarray(list(values), dtype=float)
    if len(v) < 2:
        return (float(v.mean()) if len(v) else math.nan), math.nan
    half = stats.t.ppf(0.5 + level / 2.0, len(v) - 1) * v.std(ddof=1) / math.sqrt(len(v))
    return float(v.mean()), float(half)


def paired_difference(a_by_seed, b_by_seed, level=LEVEL):
    """(mean of a - b, half-width) over the seeds both have."""
    seeds = sorted(set(a_by_seed) & set(b_by_seed))
    return mean_ci([a_by_seed[s] - b_by_seed[s] for s in seeds], level)


def n_required(values, target=TARGET, level=LEVEL):
    """Seeds needed for a relative half-width of target, from the spread of these seed means."""
    v = np.asarray(list(values), dtype=float)
    if len(v) < 2 or v.mean() == 0:
        return math.nan
    z = stats.norm.ppf(0.5 + level / 2.0)
    return int(math.ceil((z * v.std(ddof=1) / v.mean() / target) ** 2))


def cell_table(rows, target=TARGET):
    """One row per controller and speed: seeds, finished share, mean RMS with its interval,
    relative half-width, whether it meets the target, and the seeds the target needs."""
    out = []
    for c in dict.fromkeys(r["controller"] for r in rows):
        for v in sorted({float(r["speed"]) for r in rows}):
            sel = [r for r in rows if r["controller"] == c and float(r["speed"]) == v]
            by = seed_means(rows, c, v)
            mean, half = mean_ci(by.values())
            rel = half / mean if mean else math.nan
            out.append({"controller": c, "speed": v, "seeds": len(by), "runs": len(sel),
                        "finished": sum(_finished(r) for r in sel), "mean_rms": mean,
                        "half_width": half, "rel_half_width": rel, "meets_target": rel <= target,
                        "n_required": n_required(by.values(), target)})
    return out


def pair_table(rows, pairs):
    """Paired differences (a - b) for the listed (controller a, controller b) pairs, per speed."""
    out = []
    for a, b in pairs:
        for v in sorted({float(r["speed"]) for r in rows}):
            diff, half = paired_difference(seed_means(rows, a, v), seed_means(rows, b, v))
            out.append({"a": a, "b": b, "speed": v, "difference": diff, "half_width": half,
                        "excludes_zero": bool(abs(diff) > half)})
    return out
