"""Task 18: Bode plot at 10 m/s and gain/phase margins against speed for the four controllers.

Loop broken at the steering command (linear_loops.open_loop), sampled at 0.05 s. Writes
data/results/margins_vs_speed.csv, docs/figures/bode_10ms.png and docs/figures/margins_vs_speed.png.
"""
import csv

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from trackloop import linear_loops as ll
from trackloop import margins as mg

SPEEDS = np.arange(2.0, 71.0, 1.0)
LAGS = (0.0, 0.1, 0.2)
COLORS = dict(zip(ll.CONTROLLERS, ("tab:blue", "tab:orange", "tab:green", "tab:red")))


def main():
    rows = []
    for c in ll.CONTROLLERS:
        for lag in LAGS:
            for v in SPEEDS:
                m = mg.margins(c, float(v), lag)
                rows.append({"controller": c, "lag": lag, "speed": float(v),
                             "gain_up": round(m["gain_up"], 4), "gain_down": round(m["gain_down"], 4),
                             "phase_margin_deg": round(m["phase_margin_deg"], 3),
                             "crossover_rad_s": round(m["crossover_rad_s"], 4),
                             "delay_margin_s": round(m["delay_margin_s"], 5)})
    with open("data/results/margins_vs_speed.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    def pick(c, lag, key):
        return np.array([r[key] for r in rows if r["controller"] == c and r["lag"] == lag], dtype=float)

    print("Phase margin (deg) at 5, 10, 20, 40 m/s; first grid speed with phase margin <= 0 (Task 19 refines it)")
    for lag in LAGS:
        print(f"-- steering lag {lag} s")
        for c in ll.CONTROLLERS:
            pm = pick(c, lag, "phase_margin_deg")
            at = {v: pm[list(SPEEDS).index(v)] for v in (5.0, 10.0, 20.0, 40.0)}
            bad = SPEEDS[pm <= 0.0]
            first = f"{bad[0]:.0f} m/s" if len(bad) else "none up to 70 m/s"
            print(f"{c:13s} " + "  ".join(f"{v:.0f}: {x:5.1f}" for v, x in at.items()) + f"   first <= 0: {first}")

    # figure 1: Bode at 10 m/s, 0.1 s lag
    w = np.geomspace(0.05, np.pi / ll.DT * 0.999, 800)
    fig, ax = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    for c in ll.CONTROLLERS:
        system = ll.open_loop(c, 10.0, 0.1)
        resp = mg.frequency_response(system, w)
        ax[0].semilogx(w, 20 * np.log10(np.abs(resp)), color=COLORS[c], label=c)
        ax[1].semilogx(w, np.degrees(np.unwrap(np.angle(resp))), color=COLORS[c])
        pm, wc = mg.phase_margin(system)
        ax[0].plot([wc], [0], "o", color=COLORS[c])
        ax[1].plot([wc], [pm - 180.0], "o", color=COLORS[c])
    ax[0].axhline(0, color="k", lw=0.8)
    ax[1].axhline(-180, color="k", lw=0.8)
    ax[0].set(ylabel="loop gain (dB)", title="Loop broken at the steering command: 10 m/s, 0.1 s steering lag, 0.05 s sampling")
    ax[1].set(xlabel="frequency (rad/s)", ylabel="loop phase (deg)")
    ax[0].legend(fontsize=8)
    for a in ax:
        a.grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig("docs/figures/bode_10ms.png", dpi=150)

    # figure 2: margins against speed
    fig, ax = plt.subplots(2, 3, figsize=(13, 6.5), sharex=True)
    for j, lag in enumerate(LAGS):
        for c in ll.CONTROLLERS:
            ax[0, j].plot(SPEEDS, pick(c, lag, "phase_margin_deg"), color=COLORS[c], label=c)
            up = pick(c, lag, "gain_up")
            ax[1, j].plot(SPEEDS, 20 * np.log10(up), color=COLORS[c])
        ax[0, j].axhline(0, color="k", lw=0.8)
        ax[0, j].set_title(f"steering lag {lag} s")
        ax[1, j].set_xlabel("speed (m/s)")
        for r in (0, 1):
            ax[r, j].grid(alpha=0.3)
    ax[0, 0].set_ylabel("phase margin (deg)")
    ax[1, 0].set_ylabel("upward gain margin (dB)\n(blank where the loop is unstable)")
    ax[0, 0].legend(fontsize=8)
    fig.suptitle("Margins of the linear steering loop against speed (speeds above about 20 m/s are model extrapolation)")
    fig.tight_layout()
    fig.savefig("docs/figures/margins_vs_speed.png", dpi=150)
    print("wrote data/results/margins_vs_speed.csv, docs/figures/bode_10ms.png, docs/figures/margins_vs_speed.png")


if __name__ == "__main__":
    main()
