"""Task 19: speed at which each linear steering loop loses stability, checked in the nonlinear simulation.

For each controller and steering lag, find the lowest speed where the sampled linear loop is unstable (search
up to 300 m/s). Where there is one, run the nonlinear simulation from a 1 mm offset at 0.9, 0.97, 1.03 and 1.1 times
that speed and compare it with the linear model (does the error grow or decay the same way, how far apart are the
trajectories); also run it from a 5 cm offset at 1.1 and 1.3 times the speed, where steering limits can act.
Where there is none, run it at 100 and 300 m/s. Writes data/results/stability_limits.csv and
docs/figures/stability_limit_check.png.
"""
import csv

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from trackloop import linear_loops as ll
from trackloop import stability_limit as sl

LAGS = (0.0, 0.1, 0.2)
FRACTIONS = (0.9, 0.97, 1.03, 1.1)
LARGE_FRACTIONS = (1.1, 1.3)
COLORS = dict(zip(ll.CONTROLLERS, ("tab:blue", "tab:orange", "tab:green", "tab:red")))


def main():
    rows, limits = [], {}
    for c in ll.CONTROLLERS:
        for lag in LAGS:
            limits[(c, lag)] = sl.linear_limit(c, lag)
    for (c, lag), lim in limits.items():
        if lim is None:
            for v in (100.0, 300.0):
                r = sl.compare_run(c, v, lag, e0=0.001, duration=60.0)
                rows.append({"controller": c, "lag": lag, "kind": "small", "fraction": "", "speed": v, "linear_limit": "",
                             "trajectory_gap": r["trajectory_gap"], "ratio_sim": r["ratio_sim"],
                             "ratio_lin": r["ratio_lin"], "max_error": r["max_error"], "end_error": ""})
            continue
        for f in FRACTIONS:
            r = sl.compare_run(c, lim * f, lag, e0=0.001, duration=120.0)
            rows.append({"controller": c, "lag": lag, "kind": "small", "fraction": f, "speed": lim * f,
                         "linear_limit": lim, "trajectory_gap": r["trajectory_gap"], "ratio_sim": r["ratio_sim"],
                         "ratio_lin": r["ratio_lin"], "max_error": r["max_error"], "end_error": ""})
        for f in LARGE_FRACTIONS:
            t, e = sl.simulate(c, lim * f, lag, e0=0.05, duration=120.0)
            rows.append({"controller": c, "lag": lag, "kind": "large", "fraction": f, "speed": lim * f,
                         "linear_limit": lim, "trajectory_gap": "", "ratio_sim": "", "ratio_lin": "",
                         "max_error": float(np.abs(e).max()), "end_error": float(np.abs(e[t >= t[-1] - 10.0]).max())})
    fields = list(rows[0])
    with open("data/results/stability_limits.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.6g}" if isinstance(v, float) else v) for k, v in r.items()})

    print("Linear stability limit (lowest speed where the sampled loop is unstable, search up to 300 m/s)")
    print(f"{'':13s} " + "".join(f"{'lag ' + str(lag) + ' s':>14s}" for lag in LAGS))
    for c in ll.CONTROLLERS:
        print(f"{c:13s} " + "".join(f"{(f'{limits[(c, lag)]:.1f} m/s' if limits[(c, lag)] else 'none'):>14s}" for lag in LAGS))
    small = [r for r in rows if r["kind"] == "small"]
    agree = sum(1 for r in small if (np.log(r["ratio_sim"]) > 0) == (np.log(r["ratio_lin"]) > 0))
    print(f"\nSmall offset (1 mm): growth/decay agrees with the linear model in {agree} of {len(small)} runs")
    for name, sel in (("0.9 and 0.97 x limit", lambda r: r["fraction"] in (0.9, 0.97)),
                      ("1.03 x limit", lambda r: r["fraction"] == 1.03), ("1.1 x limit", lambda r: r["fraction"] == 1.1),
                      ("no limit (100, 300 m/s)", lambda r: r["fraction"] == "")):
        g = [r["trajectory_gap"] for r in small if sel(r)]
        print(f"  trajectory gap, {name:24s}: largest {max(g):.1e}")
    print("Large offset (5 cm), nonlinear simulation only: largest |error| (m) over 120 s")
    for (c, lag), lim in limits.items():
        if lim is not None:
            big = {r["fraction"]: r["max_error"] for r in rows if r["kind"] == "large" and r["controller"] == c and r["lag"] == lag}
            print(f"  {c:13s} lag {lag}: limit {lim:6.1f} m/s   1.1x: {big[1.1]:.3g}   1.3x: {big[1.3]:.3g}")

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.8))
    for c in ll.CONTROLLERS:
        pts = [(np.log(r["ratio_lin"]), np.log(r["ratio_sim"])) for r in small if r["controller"] == c]
        if pts:
            x, y = zip(*pts)
            ax[0].plot(np.clip(x, -25, 25), np.clip(y, -25, 25), "o", color=COLORS[c], label=c, alpha=0.8)
    ax[0].plot([-25, 25], [-25, 25], "k", lw=0.8)
    ax[0].axhline(0, color="gray", lw=0.5)
    ax[0].axvline(0, color="gray", lw=0.5)
    ax[0].set(xlabel="linear model: ln(amplitude late / early)", ylabel="nonlinear simulation: ln(amplitude late / early)",
              title="1 mm offset: simulation against linear model\n(each point one run, 120 s; above 0 = growing)")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)
    for (c, lag), lim in limits.items():
        if lim is None:
            continue
        pts = [(1.0, None)]
        big = sorted((r["fraction"], r["max_error"]) for r in rows if r["kind"] == "large" and r["controller"] == c and r["lag"] == lag)
        ax[1].plot([p[0] for p in big], [p[1] for p in big], "o-", color=COLORS[c], alpha=0.5 + 0.25 * LAGS.index(lag),
                   label=f"{c}, lag {lag}")
    ax[1].set(yscale="log", xlabel="speed / linear stability limit", ylabel="largest |error| over 120 s (m)",
              title="5 cm offset, nonlinear simulation, past the limit")
    ax[1].legend(fontsize=7)
    ax[1].grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig("docs/figures/stability_limit_check.png", dpi=150)
    print("wrote data/results/stability_limits.csv and docs/figures/stability_limit_check.png")


if __name__ == "__main__":
    main()
