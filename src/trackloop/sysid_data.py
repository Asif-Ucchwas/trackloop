"""Simulated, noisy data for system identification of the dynamic bicycle model (Task 21).

This is SIMULATED data. The "vehicle" is the dynamic model of models/dynamic.py with known parameters, driven by an
excitation signal at constant speed, and what the estimator is allowed to see is the measured yaw rate and lateral
acceleration (true value plus Gaussian noise) and the steering angle. Nothing here says anything about identifying a
real vehicle.

Row k of a dataset is a snapshot at time t_k = k dt:
  delta[k]   steering angle in force from t_k until t_{k+1} (the model holds it constant over the step)
  r_true[k]  yaw rate at t_k          r_meas[k]  the same plus noise
  ay_true[k] lateral acceleration (body frame, vy_dot + vx r = total tire lateral force / mass) at t_k
  vy_true[k] lateral velocity at t_k (never given to an estimator: it is not measured)
"""
import csv
import json
from dataclasses import asdict, dataclass

import numpy as np

from trackloop.models import dynamic as dyn

DT = 0.01
AMPLITUDE = 0.03     # rad, small enough that the rate limit is not reached by the sweep
NOISE_R = 0.003      # rad/s, standard deviation of the yaw rate measurement noise (a choice, not a sensor datasheet)
NOISE_AY = 0.05      # m/s^2, standard deviation of the lateral acceleration noise (a choice)
KINDS = ("sweep", "prbs")
COLUMNS = ("t", "steer_cmd", "delta", "r_meas", "ay_meas", "r_true", "ay_true", "vy_true")


@dataclass(frozen=True)
class Settings:
    kind: str = "sweep"
    vx: float = 15.0
    duration: float = 40.0
    dt: float = DT
    amplitude: float = AMPLITUDE
    noise_r: float = NOISE_R
    noise_ay: float = NOISE_AY
    seed: int = 1
    f0: float = 0.1          # sweep start frequency, Hz
    f1: float = 3.0          # sweep end frequency, Hz
    hold: float = 0.25       # prbs: time each random level is held, s


def sweep_command(t, duration, f0, f1, amplitude):
    """Linear chirp from f0 to f1 Hz over `duration` seconds."""
    phase = 2 * np.pi * (f0 * t + (f1 - f0) * t**2 / (2 * duration))
    return amplitude * np.sin(phase)


def prbs_command(t, hold, amplitude, rng):
    """Random binary steering: plus or minus `amplitude`, each level held `hold` seconds."""
    n_levels = int(np.ceil(t[-1] / hold)) + 1
    levels = amplitude * rng.choice([-1.0, 1.0], size=n_levels)
    return levels[(t / hold).astype(int)]


def command(settings):
    t = np.arange(int(round(settings.duration / settings.dt))) * settings.dt
    if settings.kind == "sweep":
        return t, sweep_command(t, settings.duration, settings.f0, settings.f1, settings.amplitude)
    if settings.kind == "prbs":
        return t, prbs_command(t, settings.hold, settings.amplitude, np.random.default_rng([settings.seed, 0]))
    raise ValueError(f"kind must be one of {KINDS}")


def run(cmd, vx, dt, params):
    """Drive the dynamic model with the steering command `cmd` at constant speed vx.

    Returns the true signals and the share of steps where the rate or angle limit was active.
    """
    n = len(cmd)
    s = np.array([0.0, 0.0, 0.0, vx, 0.0, 0.0, 0.0])
    delta, r, vy, ay = (np.zeros(n) for _ in range(4))
    limited = 0
    for k in range(n):
        prev = s[dyn.DELTA]
        want = prev + (cmd[k] - prev) * (1 - np.exp(-dt / params.steer_lag)) if params.steer_lag > 0 else cmd[k]
        s_next = dyn.step(s, cmd[k], 0.0, dt, params)
        delta[k] = s_next[dyn.DELTA]
        limited += abs(want - prev) > params.max_steer_rate * dt + 1e-12 or abs(want) > params.max_steer
        r[k], vy[k] = s[dyn.R], s[dyn.VY]
        d = dyn._derivatives(s[:dyn.DELTA], delta[k], 0.0, params)
        ay[k] = d[4] + s[dyn.VX] * s[dyn.R]
        s = s_next
    return {"delta": delta, "r_true": r, "vy_true": vy, "ay_true": ay, "limited": limited / n}


def generate(settings=Settings(), params=dyn.DynamicParams()):
    """One noisy dataset: a dict of arrays (COLUMNS), plus 'limited' (share of limited steps) and 'settings'."""
    t, cmd = command(settings)
    out = run(cmd, settings.vx, settings.dt, params)
    noise = np.random.default_rng([settings.seed, 1])        # independent of the excitation: same noise for either kind
    out["r_meas"] = out["r_true"] + noise.normal(0.0, settings.noise_r, len(t))
    out["ay_meas"] = out["ay_true"] + noise.normal(0.0, settings.noise_ay, len(t))
    out.update(t=t, steer_cmd=cmd, settings=settings)
    return out


def write(data, stem):
    """Write `stem`.csv (COLUMNS) and `stem`.json (the settings and the true parameters)."""
    with open(f"{stem}.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(COLUMNS)
        for k in range(len(data["t"])):
            w.writerow([f"{data[c][k]:.6f}" for c in COLUMNS])
    with open(f"{stem}.json", "w") as fh:
        json.dump({"settings": asdict(data["settings"]), "true_parameters": asdict(dyn.DynamicParams()),
                   "limited_share": data["limited"]}, fh, indent=1)


def load(stem):
    """Read `stem`.csv back as a dict of arrays."""
    with open(f"{stem}.csv") as fh:
        rows = list(csv.reader(fh))
    cols = rows[0]
    arr = np.array(rows[1:], dtype=float)
    return {c: arr[:, i] for i, c in enumerate(cols)}


def signal_to_noise(data):
    """RMS of the true signal over the noise standard deviation, for yaw rate and lateral acceleration."""
    s = data["settings"]
    rms = lambda x: float(np.sqrt(np.mean(np.square(x))))
    return rms(data["r_true"]) / s.noise_r, rms(data["ay_true"]) / s.noise_ay
