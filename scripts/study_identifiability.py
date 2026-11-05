"""Task 23: how excitation, noise and assumptions change the identification of cf, cr and Iz (simulated data).

1. For 27 experiments (noise level, steering amplitude, record length, sweep bandwidth, random-signal hold time, speed)
   the covariance the estimator would report at the true parameters; checked against the real spread over 50 noise
   seeds for six of them.
2. A wrong assumed mass, a large steering amplitude (model difference), and noise on the measured steering angle.
Writes data/results/identifiability_conditions.csv and identifiability_other.csv, docs/figures/identifiability.png.
"""
import csv
from dataclasses import replace

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from trackloop import identifiability as ib
from trackloop import identification as idn
from trackloop import sysid_data as sd

N_SEEDS = 50
BASE = sd.Settings(kind="sweep", vx=15.0, duration=40.0, seed=21)
PARAMS = ("cf", "cr", "Iz")


def conditions():
    c = [("base", "sweep", BASE), ("base", "prbs", replace(BASE, kind="prbs"))]
    c += [("noise", f"x{m:g}", replace(BASE, noise_r=BASE.noise_r * m, noise_ay=BASE.noise_ay * m)) for m in (0.25, 0.5, 2, 4, 8)]
    c += [("amplitude", f"{a:g} rad", replace(BASE, amplitude=a)) for a in (0.01, 0.02, 0.05, 0.1)]
    c += [("duration", f"{t:g} s", replace(BASE, duration=t)) for t in (10, 20, 80)]
    c += [("sweep top frequency", f"{f:g} Hz", replace(BASE, f1=f)) for f in (0.5, 1, 2, 5)]
    c += [("prbs hold", f"{h:g} s", replace(BASE, kind="prbs", hold=h)) for h in (0.1, 0.5, 1.0, 2.0)]
    c += [("speed", f"{v:g} m/s", replace(BASE, vx=v)) for v in (5, 10, 25, 30)]
    return c


CHECKED = {("base", "sweep"), ("noise", "x8"), ("amplitude", "0.01 rad"), ("speed", "5 m/s"), ("prbs hold", "2 s"),
           ("duration", "10 s")}


def main():
    rows, pred = [], {}
    print(f"{'experiment':32s} predicted std % (cf, cr, Iz)     eigenvalue ratio   weakest direction (cf, cr, Iz)")
    for group, label, s in conditions():
        p = ib.predicted(s)
        pred[(group, label)] = p
        row = {"group": group, "setting": label, **{f"std_pct_{n}": round(v, 4) for n, v in zip(PARAMS, p["std_pct"])},
               "eig_ratio": round(p["eig_ratio"], 1), **{f"weakest_{n}": round(v, 3) for n, v in zip(PARAMS, p["weakest"])}}
        if (group, label) in CHECKED:
            est, std = idn.monte_carlo(s, N_SEEDS)
            err = 100 * (est / np.array([getattr(idn.NOMINAL, n) for n in idn.DEFAULT_NAMES]) - 1)
            cover = (np.abs(est - np.array([getattr(idn.NOMINAL, n) for n in idn.DEFAULT_NAMES])) < 1.96 * std).mean(axis=0)
            for i, n in enumerate(PARAMS):
                row[f"mc_spread_pct_{n}"] = round(float(err[:, i].std(ddof=1)), 4)
                row[f"mc_bias_pct_{n}"] = round(float(err[:, i].mean()), 4)
                row[f"mc_coverage_{n}"] = round(float(cover[i]), 2)
        rows.append(row)
        print(f"{group + ' ' + label:32s} {p['std_pct'][0]:7.3f} {p['std_pct'][1]:7.3f} {p['std_pct'][2]:7.3f}   "
              f"{p['eig_ratio']:12.1f}   {np.round(p['weakest'], 2).tolist()}"
              + (f"   [{N_SEEDS} seeds: spread {err.std(axis=0, ddof=1).round(3).tolist()}, bias {err.mean(axis=0).round(3).tolist()}]"
                 if (group, label) in CHECKED else ""))
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with open("data/results/identifiability_conditions.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    other = []

    def add(study, setting, metric, values):
        for n, v in zip(PARAMS if len(values) == 3 else ("all",), values):
            other.append({"study": study, "setting": setting, "metric": metric, "parameter": n, "value": round(float(v), 4)})

    print("\nAssumed mass (estimate over true value for cf, cr, Iz; chi2 per degree of freedom)")
    mass = {}
    for k in (0.9, 0.95, 0.98, 1.02, 1.05, 1.1):
        ratio, chi = ib.fit_with_assumed_mass(BASE, k)
        mass[k] = ratio
        add("assumed mass", f"x{k:g}", "estimate_over_true", ratio)
        add("assumed mass", f"x{k:g}", "chi2_dof", [chi])
        print(f"  mass x{k:<5g} {np.round(ratio, 4).tolist()}  chi2 {chi:.3f}")

    print("\nSteering amplitude: error of the fit on noise-free data (model difference) and the statistical std, percent")
    amp = {}
    for a in (0.01, 0.03, 0.05, 0.1, 0.2):
        s = replace(BASE, amplitude=a)
        bias = ib.noise_free_bias(s)
        std = ib.predicted(s)["std_pct"]
        amp[a] = (bias, std)
        add("amplitude", f"{a:g} rad", "noise_free_bias_pct", bias)
        add("amplitude", f"{a:g} rad", "std_pct", std)
        print(f"  {a:5g} rad  bias {np.round(bias, 3).tolist()}  std {np.round(std, 3).tolist()}")

    print(f"\nNoise on the measured steering angle ({N_SEEDS} seeds; percent): bias, spread, predicted std, chi2")
    steer = {}
    for sg in (0.0, 0.0005, 0.001, 0.002):
        err, pstd, chi = ib.steering_noise_monte_carlo(BASE, sg, N_SEEDS)
        steer[sg] = (err.mean(axis=0), err.std(axis=0, ddof=1), pstd)
        add("steering noise", f"{sg:g} rad", "bias_pct", err.mean(axis=0))
        add("steering noise", f"{sg:g} rad", "spread_pct", err.std(axis=0, ddof=1))
        add("steering noise", f"{sg:g} rad", "predicted_std_pct", pstd)
        add("steering noise", f"{sg:g} rad", "chi2_dof", [chi])
        print(f"  {sg:g} rad: bias {err.mean(axis=0).round(3).tolist()}  spread {err.std(axis=0, ddof=1).round(3).tolist()}  "
              f"predicted {pstd.round(3).tolist()}  chi2 {chi:.2f}")
    with open("data/results/identifiability_other.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["study", "setting", "metric", "parameter", "value"])
        w.writeheader()
        w.writerows(other)

    fig, ax = plt.subplots(2, 4, figsize=(18, 8.5))

    def lines(a, group, labels, xs, xlabel, title, logx=False):
        for i, n in enumerate(PARAMS):
            a.plot(xs, [pred[(group, lb)]["std_pct"][i] for lb in labels], "o-", label=n)
        a.set(xlabel=xlabel, title=title, yscale="log", ylabel="predicted std (% of true value)", xscale="log" if logx else "linear")
        a.grid(alpha=0.3, which="both")

    speeds = [("speed", "5 m/s"), ("speed", "10 m/s"), ("base", "sweep"), ("speed", "25 m/s"), ("speed", "30 m/s")]
    for i, n in enumerate(PARAMS):
        ax[0, 0].plot([5, 10, 15, 25, 30], [pred[k]["std_pct"][i] for k in speeds], "o-", label=n)
    ax[0, 0].set(xlabel="speed (m/s)", title="speed: the rear stiffness needs speed", yscale="log", ylabel="predicted std (% of true value)")
    ax[0, 0].grid(alpha=0.3, which="both")
    ax[0, 0].legend(fontsize=8)
    f1s = [("sweep top frequency", "0.5 Hz"), ("sweep top frequency", "1 Hz"), ("sweep top frequency", "2 Hz"), ("base", "sweep"),
           ("sweep top frequency", "5 Hz")]
    for i, n in enumerate(PARAMS):
        ax[0, 1].plot([0.5, 1, 2, 3, 5], [pred[k]["std_pct"][i] for k in f1s], "o-", label=n)
    ax[0, 1].set(xlabel="sweep top frequency (Hz)", title="sweep bandwidth", yscale="log", xscale="log")
    ax[0, 1].grid(alpha=0.3, which="both")
    holds = [("prbs hold", "0.1 s"), ("base", "prbs"), ("prbs hold", "0.5 s"), ("prbs hold", "1 s"), ("prbs hold", "2 s")]
    for i, n in enumerate(PARAMS):
        ax[0, 2].plot([0.1, 0.25, 0.5, 1, 2], [pred[k]["std_pct"][i] for k in holds], "o-", label=n)
    ax[0, 2].set(xlabel="random signal hold time (s)", title="random signal: slower switching is worse", yscale="log", xscale="log")
    ax[0, 2].grid(alpha=0.3, which="both")
    # scaling laws for cf
    noise_f = [0.25, 0.5, 1, 2, 4, 8]
    nl = [("noise", f"x{m:g}") if m != 1 else ("base", "sweep") for m in noise_f]
    ref = pred[("base", "sweep")]["std_pct"][0]
    ax[0, 3].plot(noise_f, [pred[k]["std_pct"][0] / ref for k in nl], "o", label="noise level (x)")
    amps = [0.01, 0.02, 0.03, 0.05, 0.1]
    al = [("amplitude", f"{a:g} rad") if a != 0.03 else ("base", "sweep") for a in amps]
    ax[0, 3].plot([a / 0.03 for a in amps], [pred[k]["std_pct"][0] / ref for k in al], "s", label="steering amplitude (x)")
    durs = [10, 20, 40, 80]
    dl = [("duration", f"{t:g} s") if t != 40 else ("base", "sweep") for t in durs]
    ax[0, 3].plot([t / 40 for t in durs], [pred[k]["std_pct"][0] / ref for k in dl], "^", label="record length (x)")
    xs = np.logspace(-0.6, 0.95, 50)
    ax[0, 3].plot(xs, xs, "k:", lw=0.8)
    ax[0, 3].plot(xs, 1 / xs, "k:", lw=0.8)
    ax[0, 3].plot(xs, xs**-0.5, "k:", lw=0.8)
    ax[0, 3].set(xscale="log", yscale="log", xlabel="factor on the base experiment", ylabel="std of cf / base std",
                 title="scaling: 1/amplitude, noise, 1/sqrt(length)\n(dotted: slopes 1, -1, -1/2)")
    ax[0, 3].legend(fontsize=7)
    ax[0, 3].grid(alpha=0.3, which="both")
    ks = sorted(mass)
    for i, n in enumerate(PARAMS):
        ax[1, 0].plot(ks, [mass[k][i] for k in ks], "o-", label=n)
    ax[1, 0].plot(ks, ks, "k:", lw=0.8, label="y = x")
    ax[1, 0].set(xlabel="assumed mass / true mass", ylabel="estimate / true value", title="a wrong mass shifts all three estimates equally")
    ax[1, 0].legend(fontsize=8)
    ax[1, 0].grid(alpha=0.3)
    aa = sorted(amp)
    ax[1, 1].plot(aa, [abs(amp[a][0][0]) for a in aa], "o-", label="|bias| of cf (model difference)")
    ax[1, 1].plot(aa, [amp[a][1][0] for a in aa], "s-", label="std of cf (noise)")
    ax[1, 1].set(xscale="log", yscale="log", xlabel="steering amplitude (rad)", ylabel="percent of true value",
                 title="amplitude: noise shrinks, model bias grows")
    ax[1, 1].legend(fontsize=8)
    ax[1, 1].grid(alpha=0.3, which="both")
    sgs = sorted(steer)
    for i, n in enumerate(PARAMS):
        ax[1, 2].errorbar(np.array(sgs) * 1000, [steer[s][0][i] for s in sgs], yerr=[steer[s][2][i] for s in sgs], fmt="o-", capsize=3, label=n)
    ax[1, 2].set(xlabel="steering measurement noise (mrad)", ylabel="mean error (%)  ± predicted std",
                 title="noise on the measured steering biases the estimates")
    ax[1, 2].legend(fontsize=8)
    ax[1, 2].grid(alpha=0.3)
    wd = [pred[("base", "sweep")]["weakest"], pred[("base", "prbs")]["weakest"], pred[("speed", "5 m/s")]["weakest"]]
    xw = np.arange(3)
    for j, (v, lab) in enumerate(zip(wd, ("sweep 15 m/s", "random 15 m/s", "sweep 5 m/s"))):
        ax[1, 3].bar(xw + 0.25 * (j - 1), v, 0.25, label=lab)
    ax[1, 3].set(xticks=xw, xticklabels=PARAMS, title="weakest direction of the data (unit vector)", ylabel="component")
    ax[1, 3].legend(fontsize=8)
    ax[1, 3].grid(alpha=0.3, axis="y")
    fig.suptitle("What changes how well cf, cr and Iz can be identified (simulated data; mass fixed)")
    fig.tight_layout()
    fig.savefig("docs/figures/identifiability.png", dpi=140)
    print("\nwrote data/results/identifiability_conditions.csv, identifiability_other.csv and docs/figures/identifiability.png")


if __name__ == "__main__":
    main()
