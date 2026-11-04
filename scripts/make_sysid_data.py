"""Task 21: generate the simulated identification datasets (a steering sweep and a random binary steering signal).

15 m/s, 40 s, 100 Hz, noise on yaw rate and lateral acceleration. Writes data/sysid/{sweep,prbs}_v15.csv and .json
(the settings and the true parameters) and docs/figures/sysid_data.png. SIMULATED data only.
"""
import os
from dataclasses import replace

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import welch

from trackloop import sysid_data as sd
from trackloop.models import dynamic as dyn

BASE = sd.Settings(vx=15.0, duration=40.0, seed=21)


def lateral_poles(vx, p=dyn.DynamicParams()):
    """Eigenvalues of the (lateral velocity, yaw rate) dynamics at speed vx, in rad/s."""
    a = np.array([[-(p.cf + p.cr) / (p.mass * vx), (-p.lf * p.cf + p.lr * p.cr) / (p.mass * vx) - vx],
                  [(-p.lf * p.cf + p.lr * p.cr) / (p.inertia * vx), -(p.lf**2 * p.cf + p.lr**2 * p.cr) / (p.inertia * vx)]])
    return np.linalg.eigvals(a)


def main():
    os.makedirs("data/sysid", exist_ok=True)
    sets = {"sweep": sd.generate(BASE), "prbs": sd.generate(replace(BASE, kind="prbs"))}
    poles = lateral_poles(BASE.vx)
    print(f"Lateral poles at {BASE.vx:g} m/s: " + ", ".join(f"{p:.2f}" for p in poles) + " rad/s")
    for name, d in sets.items():
        sd.write(d, f"data/sysid/{name}_v{BASE.vx:g}")
        snr_r, snr_ay = sd.signal_to_noise(d)
        rate = np.abs(np.diff(d["delta"])).max() / BASE.dt
        print(f"{name:6s} {len(d['t'])} samples  peak steering {np.abs(d['delta']).max():.3f} rad  peak rate {rate:.2f} rad/s  "
              f"limited {d['limited']:.1%}  RMS r {np.sqrt(np.mean(d['r_true']**2)):.3f} rad/s  RMS ay {np.sqrt(np.mean(d['ay_true']**2)):.2f} m/s2  "
              f"SNR r {snr_r:.0f}  ay {snr_ay:.0f}")

    fig, ax = plt.subplots(4, 2, figsize=(13, 10))
    for j, (name, d) in enumerate(sets.items()):
        t = d["t"]
        ax[0, j].plot(t, d["steer_cmd"], lw=0.8, label="command")
        ax[0, j].plot(t, d["delta"], lw=0.8, label="applied")
        ax[0, j].set(title=f"{name}: steering (rad)")
        ax[1, j].plot(t, d["r_meas"], lw=0.4, color="0.6", label="measured")
        ax[1, j].plot(t, d["r_true"], lw=0.8, label="true")
        ax[1, j].set(title="yaw rate (rad/s)")
        ax[2, j].plot(t, d["ay_meas"], lw=0.4, color="0.6", label="measured")
        ax[2, j].plot(t, d["ay_true"], lw=0.8, label="true")
        ax[2, j].set(title="lateral acceleration (m/s²)", xlabel="time (s)")
        f, pxx = welch(d["delta"], fs=1 / BASE.dt, nperseg=1024)
        ax[3, j].semilogy(f, pxx)
        for p in poles:
            ax[3, j].axvline(abs(p) / (2 * np.pi), color="k", ls=":", lw=0.8)
        ax[3, j].set(title="steering spectrum; dotted: lateral pole frequencies", xlabel="frequency (Hz)", xlim=(0, 10))
        for a in ax[:3, j]:
            a.grid(alpha=0.3)
        ax[3, j].grid(alpha=0.3, which="both")
    ax[0, 0].legend(fontsize=8)
    ax[1, 0].legend(fontsize=8)
    fig.suptitle("Simulated identification data (15 m/s, noise on yaw rate and lateral acceleration)")
    fig.tight_layout()
    fig.savefig("docs/figures/sysid_data.png", dpi=150)
    print("wrote data/sysid/*.csv, *.json and docs/figures/sysid_data.png")


if __name__ == "__main__":
    main()
