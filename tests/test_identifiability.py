from dataclasses import replace

import numpy as np

from trackloop import identifiability as ib
from trackloop import sysid_data as sd

S = sd.Settings(duration=10.0, seed=21)


def test_standard_deviations_scale_with_noise_amplitude_and_record_length():
    base = ib.predicted(S)["std_pct"]
    noisy = ib.predicted(replace(S, noise_r=4 * S.noise_r, noise_ay=4 * S.noise_ay))["std_pct"]
    small = ib.predicted(replace(S, amplitude=S.amplitude / 3))["std_pct"]
    long = ib.predicted(replace(S, duration=40.0))["std_pct"]
    assert np.allclose(noisy / base, 4.0, rtol=1e-4)
    assert np.allclose(small / base, 3.0, rtol=1e-3)
    assert np.allclose(long / base, 0.5, rtol=0.05)


def test_the_rear_stiffness_is_poorly_determined_at_low_speed_only():
    low = ib.predicted(replace(S, duration=40.0, vx=5.0))["std_pct"]
    normal = ib.predicted(replace(S, duration=40.0))["std_pct"]
    assert low[1] / low[0] > 4 and normal[1] / normal[0] < 2


def test_a_wrong_assumed_mass_moves_every_estimate_by_the_same_factor_and_the_fit_cannot_tell():
    ok, chi_ok = ib.fit_with_assumed_mass(replace(S, duration=20.0), 1.0)
    off, chi_off = ib.fit_with_assumed_mass(replace(S, duration=20.0), 1.05)
    assert np.allclose(off / ok, 1.05, atol=2e-3)
    assert abs(chi_off - chi_ok) < 0.01


def test_a_large_steering_amplitude_biases_the_estimates_faster_than_it_shrinks_their_spread():
    small = np.abs(ib.noise_free_bias(replace(S, amplitude=0.03))).max()
    large = np.abs(ib.noise_free_bias(replace(S, amplitude=0.1))).max()
    assert large > 4 * small


def test_noise_on_the_measured_steering_angle_biases_the_estimates_downward():
    err, _, chi = ib.steering_noise_monte_carlo(replace(S, duration=20.0), 0.001, 8)
    assert err.mean(axis=0).max() < -0.3 and chi > 1.2
