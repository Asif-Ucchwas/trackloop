"""Task 22: least-squares identification of cf, cr and Iz from the Task 21 data, with the parameter covariance.

Fits the two committed datasets (data/sysid/*.csv), then repeats the fit on 100 datasets that differ only in the
noise seed, to check the predicted standard deviations against the real spread. SIMULATED data; mass fixed at its
true value because it cannot be identified from these signals (Task 21). Writes data/results/identification.csv
and docs/figures/identification.png.
"""
import csv
from dataclasses import replace

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from trackloop import identification as idn

N_SEEDS = 100
LABEL = {"cf": "cf (N/rad)", "cr": "cr (N/rad)", "inertia": "Iz (kg m²)"}


def ellipse(center, cov, ax, **kw):
    w, v = np.linalg.eigh(cov)
    t = np.linspace(0, 2 * np.pi, 100)
    pts = (v * np.sqrt(5.991 * w)) @ np.array([np.cos(t), np.sin(t)])      # 95 percent region for 2 parameters
    ax.plot(center[0] + pts[0], center[1] + pts[1], **kw)


def main():
    rows, fits, mcs = [], {}, {}
    for kind in ("sweep", "prbs"):
        data = idn.load_dataset(f"data/sysid/{kind}_v15")
        f = idn.fit(data)
        est, std = idn.monte_carlo(replace(data["settings"], kind=kind), N_SEEDS)
        fits[kind], mcs[kind] = (data, f), (est, std)
        print(f"\n{kind}: 4000 samples, chi2 per degree of freedom {f['chi2_dof']:.3f}, {f['nfev']} function evaluations")
        print(f"  {'':8s} {'true':>9s} {'estimate':>11s} {'std':>8s} {'error %':>8s} {'error/std':>9s}   "
              f"{N_SEEDS}-seed: bias %  spread %  predicted %  inside 95% interval")
        for i, nm in enumerate(f["names"]):
            tr, th, sd_ = f["true"][i], f["theta"][i], f["std"][i]
            bias = 100 * (est[:, i].mean() / tr - 1)
            spread = 100 * est[:, i].std(ddof=1) / tr
            pred = 100 * std[:, i].mean() / tr
            cover = float((np.abs(est[:, i] - tr) < 1.96 * std[:, i]).mean())
            print(f"  {nm:8s} {tr:9.1f} {th:11.1f} {sd_:8.1f} {100 * (th / tr - 1):8.3f} {(th - tr) / sd_:9.2f}   "
                  f"{bias:19.3f} {spread:9.3f} {pred:12.3f} {cover:14.0%}")
            rows.append({"dataset": kind, "parameter": nm, "true": tr, "estimate": round(th, 2), "std": round(sd_, 2),
                         "error_pct": round(100 * (th / tr - 1), 4), "mc_bias_pct": round(bias, 4),
                         "mc_spread_pct": round(spread, 4), "mc_predicted_std_pct": round(pred, 4), "mc_coverage_95": round(cover, 3)})
        print("  correlation (cf, cr, Iz):", np.round(f["corr"][np.triu_indices(3, 1)], 3).tolist(), "for cf-cr, cf-Iz, cr-Iz")
    with open("data/results/identification.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    fig, ax = plt.subplots(2, 3, figsize=(15, 8))
    for i, kind in enumerate(("sweep", "prbs")):
        (data, f), (est, _) = fits[kind], mcs[kind]
        s = data["settings"]
        window = (data["t"] > 20) & (data["t"] < 25) if kind == "sweep" else (data["t"] > 10) & (data["t"] < 15)
        r, _ = idn.predict(f["theta"], f["names"], data["delta"], s.vx, s.dt)
        ax[i, 0].plot(data["t"][window], data["r_meas"][window], ".", ms=3, color="0.6", label="measured")
        ax[i, 0].plot(data["t"][window], r[window], lw=1.2, label="fitted model")
        ax[i, 0].set(title=f"{kind}: yaw rate, 5 s of the fit", xlabel="time (s)", ylabel="rad/s")
        ax[i, 0].legend(fontsize=8)
        for a, (p, q) in zip(ax[i, 1:], ((0, 1), (0, 2))):
            a.plot(est[:, p] / f["true"][p], est[:, q] / f["true"][q], ".", color="0.5", label=f"{N_SEEDS} noise seeds")
            a.plot(1, 1, "k+", ms=12, label="true value")
            c = f["cov"][np.ix_([p, q], [p, q])] / np.outer(f["true"][[p, q]], f["true"][[p, q]])
            ellipse((1, 1), c, a, color="C3", lw=1.2, label="95% from covariance")
            a.set(xlabel=LABEL[f["names"][p]] + " / true", ylabel=LABEL[f["names"][q]] + " / true",
                  title=f"{kind}: estimates over noise seeds")
            a.grid(alpha=0.3)
        ax[i, 1].legend(fontsize=8)
        ax[i, 0].grid(alpha=0.3)
    fig.suptitle("Least-squares identification of cf, cr and Iz from simulated noisy data (mass fixed)")
    fig.tight_layout()
    fig.savefig("docs/figures/identification.png", dpi=150)
    print("\nwrote data/results/identification.csv and docs/figures/identification.png")


if __name__ == "__main__":
    main()
