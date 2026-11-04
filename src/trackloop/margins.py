"""Gain and phase margins of the sampled steering loops (Task 18).

The loop is broken at the steering command (linear_loops.open_loop). Gain margins are found by
scaling the loop gain and testing the closed-loop poles, so they are exact for the model and need
no choice of crossover: `gain_down` is the factor the gain can fall to before instability,
`gain_up` the factor it can grow to. The phase margin is read from the frequency response up to
the Nyquist frequency at the unity-gain crossings (smallest if there are several).
"""
import numpy as np

from trackloop.linear_loops import DT, open_loop


def _stable(a, b, c, s):
    return bool(np.max(np.abs(np.linalg.eigvals(a - s * b @ c))) < 1.0 - 1e-9)


def gain_margins(system, lo=1e-3, hi=1e4, n=1200):
    """(down, up): loop-gain factors, below and above 1, at which the closed loop first goes unstable.

    down = 0.0 if it is stable all the way to `lo`; up = inf if stable all the way to `hi`.
    Returns (nan, nan) if the loop is not stable at gain 1.
    """
    a, b, c = system.A, system.B, system.C
    if not _stable(a, b, c, 1.0):
        return float("nan"), float("nan")
    grid = np.geomspace(lo, hi, n)
    i1 = int(np.searchsorted(grid, 1.0))

    def edge(inside, outside):
        for _ in range(60):
            mid = np.sqrt(inside * outside)
            if _stable(a, b, c, mid):
                inside = mid
            else:
                outside = mid
        return inside

    up = float("inf")
    prev = 1.0
    for s in grid[i1:]:
        if not _stable(a, b, c, s):
            up = edge(prev, s)
            break
        prev = s
    down = 0.0
    prev = 1.0
    for s in grid[:i1][::-1]:
        if not _stable(a, b, c, s):
            down = edge(prev, s)
            break
        prev = s
    return down, up


def frequency_response(system, w):
    """L(e^{jw dt}) for a SISO sampled StateSpace, vectorised."""
    z = np.exp(1j * w * system.dt)
    n = system.A.shape[0]
    out = np.empty(len(w), dtype=complex)
    for i, zi in enumerate(z):
        out[i] = (system.C @ np.linalg.solve(zi * np.eye(n) - system.A, system.B))[0, 0] + system.D[0, 0]
    return out


def phase_margin(system, n=4000):
    """(pm_deg, crossover_rad_per_s): smallest phase margin over the unity-gain crossings."""
    w = np.geomspace(1e-3, np.pi / system.dt * 0.999, n)
    resp = frequency_response(system, w)
    mag = np.log(np.abs(resp))
    best = (float("nan"), float("nan"))
    for i in np.where(np.sign(mag[:-1]) != np.sign(mag[1:]))[0]:
        t = mag[i] / (mag[i] - mag[i + 1])
        wc = float(np.exp(np.log(w[i]) + t * (np.log(w[i + 1]) - np.log(w[i]))))
        lc = frequency_response(system, np.array([wc]))[0]
        pm = (np.degrees(np.angle(lc)) + 180.0 + 180.0) % 360.0 - 180.0
        if np.isnan(best[0]) or pm < best[0]:
            best = (float(pm), wc)
    return best


def margins(controller, vx, lag=0.0, dt=DT):
    """dict: gain_down, gain_up (factors), phase_margin_deg, crossover_rad_s, delay_margin_s."""
    system = open_loop(controller, vx, lag, dt)
    down, up = gain_margins(system)
    pm, wc = phase_margin(system)
    return {"gain_down": down, "gain_up": up, "phase_margin_deg": pm, "crossover_rad_s": wc,
            "delay_margin_s": float(np.radians(pm) / wc) if wc == wc and wc > 0 else float("nan")}
