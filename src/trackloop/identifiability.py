"""How excitation, noise and assumptions change the identification of cf, cr and Iz (Task 22's estimator, studied in Task 23).

Everything here is SIMULATED data from the dynamic model. predicted() gives the covariance the estimator would report at
the TRUE parameters for a given experiment (no fitting needed), the other functions fit data made with a deliberate
change: a wrong assumed mass, a noisy steering measurement, or a large steering amplitude.
"""
from dataclasses import replace

import numpy as np

from trackloop import identification as idn
from trackloop import sysid_data as sd

NAMES = idn.DEFAULT_NAMES


def jacobian(data, names=NAMES, base=idn.NOMINAL, h=1e-4):
    """Jacobian of the weighted residuals with respect to the parameters as factors of `base`, at the base values."""
    cols = []
    for i in range(len(names)):
        up, down = np.ones(len(names)), np.ones(len(names))
        up[i] += h
        down[i] -= h
        cols.append((idn.residuals(up, names, data, base) - idn.residuals(down, names, data, base)) / (2 * h))
    return np.array(cols).T


def predicted(settings, names=NAMES, base=idn.NOMINAL):
    """What an experiment can determine: relative standard deviations (percent), correlation, and the weakest direction.

    The covariance is (J^T J)^-1 at the true parameters; it does not depend on the noise realisation. `weakest` is the
    unit vector in (cf, cr, Iz) factor space along which the data say least, `eig_ratio` the ratio of the largest to the
    smallest eigenvalue of J^T J (the square root of it is the ratio of the best to the worst determined direction).
    """
    info = jacobian(sd.generate(settings, base), names, base)
    info = info.T @ info
    cov = np.linalg.inv(info)
    std = np.sqrt(np.diag(cov))
    w, v = np.linalg.eigh(info)
    weakest = v[:, 0] * np.sign(v[np.argmax(np.abs(v[:, 0])), 0])
    return {"std_pct": 100 * std, "corr": cov / np.outer(std, std), "eig_ratio": float(w[-1] / w[0]), "weakest": weakest}


def fit_with_assumed_mass(settings, mass_factor):
    """Fit data made with the true mass while the estimator assumes `mass_factor` times it. Returns estimates over the
    true values (cf, cr, Iz) and the chi-squared per degree of freedom."""
    base = replace(idn.NOMINAL, mass=idn.NOMINAL.mass * mass_factor)
    f = idn.fit(sd.generate(settings), base=base)
    nominal = np.array([getattr(idn.NOMINAL, n) for n in NAMES])
    return f["theta"] / nominal, f["chi2_dof"]


def noise_free_bias(settings):
    """Percent error of the estimates on noise-free data: what the difference between the linear model and the
    nonlinear simulation alone does."""
    quiet = replace(settings, noise_r=1e-9, noise_ay=1e-9)
    f = idn.fit(sd.generate(quiet))
    return 100 * (f["theta"] / f["true"] - 1)


def steering_noise_monte_carlo(settings, sigma_delta, n):
    """Fit n datasets whose steering angle the estimator sees with Gaussian noise of standard deviation sigma_delta (rad).

    Returns (percent errors, shape (n, 3); the mean predicted standard deviation in percent; the mean chi-squared per
    degree of freedom). The plant gets the true steering angle.
    """
    err, std, chi = [], [], []
    for seed in range(1, n + 1):
        d = sd.generate(replace(settings, seed=seed))
        d["delta"] = d["delta"] + np.random.default_rng([seed, 2]).normal(0.0, sigma_delta, len(d["delta"]))
        f = idn.fit(d)
        err.append(100 * (f["theta"] / f["true"] - 1))
        std.append(100 * f["std"] / f["true"])
        chi.append(f["chi2_dof"])
    return np.array(err), np.mean(std, axis=0), float(np.mean(chi))
