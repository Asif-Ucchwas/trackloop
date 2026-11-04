"""Speed at which the linear steering loop loses stability, and the same test in the nonlinear simulation (Task 19)."""
import numpy as np

from trackloop import harness_road as hr
from trackloop import linear_loops as ll
from trackloop.models import dynamic as dyn
from trackloop.paths import straight


def spectral_radius(controller, vx, lag=0.0, dt=ll.DT):
    """Largest closed-loop pole magnitude of the sampled linear model."""
    return float(np.max(np.abs(np.linalg.eigvals(ll.closed_loop(controller, vx, lag, dt=dt).A))))


def linear_rate(controller, vx, lag=0.0, dt=ll.DT):
    """Growth rate (1/s) of the slowest closed-loop mode: ln(spectral radius) / dt; negative = decays."""
    return float(np.log(spectral_radius(controller, vx, lag, dt)) / dt)


def linear_limit(controller, lag=0.0, v_lo=2.0, v_hi=300.0, step=1.0, tol=0.01):
    """Lowest speed (m/s) where the linear loop is unstable, found on a `step` grid then bisected.

    None if the loop is stable at every grid speed up to v_hi, or already unstable at v_lo.
    """
    if spectral_radius(controller, v_lo, lag) >= 1.0:
        return None
    lo = v_lo
    for v in np.arange(v_lo + step, v_hi + step / 2, step):
        if spectral_radius(controller, float(v), lag) >= 1.0:
            hi = float(v)
            while hi - lo > tol:
                mid = 0.5 * (lo + hi)
                if spectral_radius(controller, mid, lag) >= 1.0:
                    hi = mid
                else:
                    lo = mid
            return 0.5 * (lo + hi)
        lo = float(v)
    return None


def simulate(controller, vx, lag, e0=0.01, duration=60.0, dt=ll.DT):
    """Nonlinear dynamic simulation on a straight path from an offset e0; returns (t, lateral error)."""
    path = straight(vx * duration + 200.0)
    ctrl = hr.CONTROLLERS[controller](path)
    ctrl.reset()
    params = dyn.DynamicParams(steer_lag=lag)
    s = np.array([0.0, e0, 0.0, vx, 0.0, 0.0, 0.0])
    e = [s[1]]
    for _ in range(int(duration / dt)):
        s = dyn.step(s, ctrl.command(s), 0.0, dt, params)
        e.append(s[1])
    return np.arange(len(e)) * dt, np.array(e)


def linear_trajectory(controller, vx, lag, e0, steps, dt=ll.DT):
    """Lateral error of the sampled linear model started offset by e0 (other states zero)."""
    a = ll.closed_loop(controller, vx, lag, dt=dt).A
    z = np.zeros(a.shape[0])
    z[0] = e0
    out = [z[0]]
    for _ in range(steps):
        z = a @ z
        out.append(z[0])
    return np.array(out)


def amplitude_ratio(t, e, window=10.0, start=5.0):
    """max |e| over the last `window` seconds divided by max |e| over `window` seconds from `start`.
    Above 1: the oscillation is larger at the end than near the start (growing)."""
    late = np.abs(e[t >= t[-1] - window]).max()
    early = np.abs(e[(t >= start) & (t < start + window)]).max()
    return float(late / early) if early > 0 else float("nan")


def compare_run(controller, vx, lag, e0=0.001, duration=120.0):
    """Nonlinear simulation against the linear model for a small offset.

    trajectory_gap: largest |e_sim - e_lin| over the run, as a share of the largest |e_lin|, computed up to the
    first time |e_sim| exceeds 0.05 m (beyond that the linear regime is over). ratio_sim and ratio_lin:
    amplitude_ratio of each series (over the whole run). max_error: largest |e_sim|.
    """
    t, e = simulate(controller, vx, lag, e0, duration)
    lin = linear_trajectory(controller, vx, lag, e0, len(t) - 1)
    over = np.where(np.abs(e) > 0.05)[0]
    n = int(over[0]) if len(over) else len(e)
    gap = float(np.abs(e[:n] - lin[:n]).max() / max(np.abs(lin[:n]).max(), 1e-300))
    return {"trajectory_gap": gap, "ratio_sim": amplitude_ratio(t, e), "ratio_lin": amplitude_ratio(t, lin),
            "max_error": float(np.abs(e).max())}
