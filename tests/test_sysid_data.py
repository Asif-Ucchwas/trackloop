from dataclasses import replace

import numpy as np
import pytest

from trackloop import sysid_data as sd
from trackloop.models import dynamic as dyn

S = sd.Settings(duration=20.0)


def test_same_seed_gives_the_same_data_and_a_new_seed_a_new_noise():
    a, b, c = sd.generate(S), sd.generate(S), sd.generate(replace(S, seed=2))
    assert np.array_equal(a["r_meas"], b["r_meas"]) and np.array_equal(a["ay_meas"], b["ay_meas"])
    assert not np.array_equal(a["r_meas"], c["r_meas"])


def test_noise_has_the_requested_standard_deviation_and_no_noise_means_exact_data():
    d = sd.generate(replace(S, duration=40.0))
    assert np.std(d["r_meas"] - d["r_true"]) == pytest.approx(sd.NOISE_R, rel=0.05)
    assert np.std(d["ay_meas"] - d["ay_true"]) == pytest.approx(sd.NOISE_AY, rel=0.05)
    z = sd.generate(replace(S, noise_r=0.0, noise_ay=0.0))
    assert np.array_equal(z["r_meas"], z["r_true"]) and np.array_equal(z["ay_meas"], z["ay_true"])


def test_the_noise_does_not_depend_on_the_excitation():
    a, b = sd.generate(S), sd.generate(replace(S, kind="prbs"))
    assert np.allclose(a["r_meas"] - a["r_true"], b["r_meas"] - b["r_true"], atol=1e-12)


def test_the_sweep_stays_inside_the_steering_limits_and_the_prbs_stays_at_its_levels():
    sweep, prbs = sd.generate(S), sd.generate(replace(S, kind="prbs"))
    assert sweep["limited"] == 0.0 and np.abs(sweep["delta"]).max() <= sd.AMPLITUDE + 1e-12
    assert np.abs(prbs["delta"]).max() <= sd.AMPLITUDE + 1e-12 and 0.0 < prbs["limited"] < 0.2


def test_steady_state_matches_the_understeer_formula():
    p = dyn.DynamicParams()
    vx, delta = 15.0, 0.02
    out = sd.run(np.full(2000, delta), vx, 0.01, p)
    r_ss = vx * delta / (p.wheelbase + p.understeer_gradient * vx**2)
    assert out["r_true"][-1] == pytest.approx(r_ss, rel=1e-3)
    assert out["ay_true"][-1] == pytest.approx(vx * r_ss, rel=1e-3)


def test_scaling_mass_inertia_and_both_stiffnesses_together_changes_nothing():
    k = 1.7
    p = dyn.DynamicParams()
    q = replace(p, mass=k * p.mass, inertia=k * p.inertia, cf=k * p.cf, cr=k * p.cr)
    a, b = sd.generate(S, p), sd.generate(S, q)
    for key in ("r_true", "ay_true", "vy_true"):
        assert np.allclose(a[key], b[key], rtol=1e-9, atol=1e-12)


def test_csv_round_trip(tmp_path):
    d = sd.generate(replace(S, duration=2.0))
    stem = str(tmp_path / "d")
    sd.write(d, stem)
    back = sd.load(stem)
    assert list(back) == list(sd.COLUMNS)
    for c in sd.COLUMNS:
        assert np.allclose(back[c], d[c], atol=1e-6)
