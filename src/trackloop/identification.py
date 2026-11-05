"""Least-squares identification of the dynamic bicycle model's parameters from the Task 21 data (Task 22).

The estimator sees only the steering angle and the noisy measured yaw rate and lateral acceleration. Its model is
the SAMPLED LINEAR (small-angle) lateral model with the steering angle held over each step, which is exact for
the data's linear tires up to the atan in the slip angles and the cos(delta) in the front force (the model-data
difference is about 0.03 percent of the signal peak at the settings used here). The estimate minimises the sum of
squared prediction errors of both outputs, each divided by its noise standard deviation:

    theta_hat = argmin  sum_k [ (r_k - r_model_k(theta))/sigma_r ]^2 + [ (ay_k - ay_model_k(theta))/sigma_ay ]^2

The covariance is (J^T J)^-1 with J the Jacobian of those weighted residuals at theta_hat: the Cramer-Rao bound for
Gaussian noise of known standard deviation, which is also what the estimator's covariance equals when the model is
right. Parameters are optimised as factors of their nominal values so the three have comparable scale.
"""
from dataclasses import replace

import numpy as np
from scipy.linalg import expm
from scipy.optimize import least_squares

from trackloop.models import dynamic as dyn

NOMINAL = dyn.DynamicParams()
DEFAULT_NAMES = ("cf", "cr", "inertia")          # the mass is fixed at its known value (it is not identifiable)
START = (0.6, 1.5, 0.7)                          # starting point as factors of the true values
BOUNDS = (0.1, 10.0)                             # factors of the true values; keeps every trial model stable


def predict(theta, names, delta, vx, dt, base=NOMINAL):
    """Yaw rate and lateral acceleration of the sampled linear model for parameter values `theta` (physical units)."""
    p = replace(base, **dict(zip(names, theta)))
    m, iz, lf, lr, cf, cr = p.mass, p.inertia, p.lf, p.lr, p.cf, p.cr
    a = np.array([[-(cf + cr) / (m * vx), (-lf * cf + lr * cr) / (m * vx) - vx],
                  [(-lf * cf + lr * cr) / (iz * vx), -(lf**2 * cf + lr**2 * cr) / (iz * vx)]])
    aug = np.zeros((3, 3))
    aug[:2, :2] = a * dt
    aug[:2, 2] = np.array([cf / m, lf * cf / iz]) * dt
    e = expm(aug)
    ad, bd = e[:2, :2], e[:2, 2]
    a1, a2, b = -(cf + cr) / (m * vx), (-lf * cf + lr * cr) / (m * vx), cf / m
    n = len(delta)
    x = np.zeros(2)
    r, ay = np.zeros(n), np.zeros(n)
    for k in range(n):
        r[k] = x[1]
        ay[k] = a1 * x[0] + a2 * x[1] + b * delta[k]
        x = ad @ x + bd * delta[k]
    return r, ay


def residuals(factors, names, data, base=NOMINAL):
    s = data["settings"]
    truth = np.array([getattr(base, nm) for nm in names])
    r, ay = predict(np.asarray(factors) * truth, names, data["delta"], s.vx, s.dt, base)
    return np.concatenate([(r - data["r_meas"]) / s.noise_r, (ay - data["ay_meas"]) / s.noise_ay])


def fit(data, names=DEFAULT_NAMES, start=START, base=NOMINAL):
    """Estimate the parameters `names` (others stay at `base`). Returns a dict with the estimate and its covariance.

    Keys: names, theta (estimates), std (from the covariance), cov, corr, chi2_dof (about 1 if the model and the
    noise levels are right), true (the base values), nfev.
    """
    truth = np.array([getattr(base, nm) for nm in names])
    sol = least_squares(residuals, np.asarray(start, dtype=float)[: len(names)], args=(names, data, base), bounds=BOUNDS)
    cov_f = np.linalg.inv(sol.jac.T @ sol.jac)
    cov = cov_f * np.outer(truth, truth)
    std = np.sqrt(np.diag(cov))
    return {"names": names, "theta": sol.x * truth, "std": std, "cov": cov, "corr": cov / np.outer(std, std),
            "chi2_dof": float(2 * sol.cost / (len(sol.fun) - len(names))), "true": truth, "nfev": int(sol.nfev)}


def information_singular_values(data, names, base=NOMINAL):
    """Singular values of the weighted Jacobian at the true parameters (a near-zero one means a parameter direction
    the data cannot see)."""
    h = 1e-4
    cols = []
    for i in range(len(names)):
        up, down = np.ones(len(names)), np.ones(len(names))
        up[i] += h
        down[i] -= h
        cols.append((residuals(up, names, data, base) - residuals(down, names, data, base)) / (2 * h))
    return np.linalg.svd(np.array(cols).T, compute_uv=False)


def load_dataset(stem):
    """Read a Task 21 dataset (CSV and JSON) from disk in the form fit() takes."""
    import json

    from trackloop import sysid_data as sd

    data = sd.load(stem)
    with open(f"{stem}.json") as fh:
        data["settings"] = sd.Settings(**json.load(fh)["settings"])
    return data


def monte_carlo(settings, n, names=DEFAULT_NAMES, params=NOMINAL):
    """Fit n datasets that differ only in the noise seed. Returns (estimates, predicted std per fit), shape (n, p)."""
    from trackloop import sysid_data as sd

    est, std = [], []
    for seed in range(1, n + 1):
        f = fit(sd.generate(replace(settings, seed=seed), params), names, base=params)
        est.append(f["theta"])
        std.append(f["std"])
    return np.array(est), np.array(std)
