from dataclasses import replace

import numpy as np

from trackloop import identification as idn
from trackloop import sysid_data as sd

S = sd.Settings(duration=20.0, seed=21)
NAMES = ("cf", "cr", "inertia")


def test_the_linear_model_matches_the_nonlinear_simulation_at_the_true_parameters():
    d = sd.generate(replace(S, noise_r=1e-9, noise_ay=1e-9))
    r, ay = idn.predict([idn.NOMINAL.cf, idn.NOMINAL.cr, idn.NOMINAL.inertia], NAMES, d["delta"], S.vx, S.dt)
    assert np.abs(r - d["r_true"]).max() < 1e-3 * np.abs(d["r_true"]).max()
    assert np.abs(ay - d["ay_true"]).max() < 1e-3 * np.abs(d["ay_true"]).max()


def test_noise_free_data_gives_back_the_true_parameters_from_a_poor_start():
    f = idn.fit(sd.generate(replace(S, noise_r=1e-9, noise_ay=1e-9)))
    assert np.abs(f["theta"] / f["true"] - 1).max() < 2e-3


def test_the_mass_direction_is_invisible_and_the_three_parameters_are_not():
    d = sd.generate(S)
    s3 = idn.information_singular_values(d, NAMES)
    s4 = idn.information_singular_values(d, ("mass",) + NAMES)
    assert s3[-1] / s3[0] > 0.05
    assert s4[-1] / s4[0] < 1e-8


def test_estimate_is_within_a_few_standard_deviations_and_residuals_match_the_noise():
    for kind in sd.KINDS:
        f = idn.fit(sd.generate(replace(S, kind=kind)))
        assert (np.abs(f["theta"] - f["true"]) < 4 * f["std"]).all()
        assert 0.9 < f["chi2_dof"] < 1.1
        assert np.linalg.eigvalsh(f["corr"]).min() > 0


def test_the_predicted_standard_deviation_matches_the_spread_over_noise_seeds():
    est, std = idn.monte_carlo(S, 25)
    ratio = est.std(axis=0, ddof=1) / std.mean(axis=0)
    assert (ratio > 0.6).all() and (ratio < 1.5).all()
