import numpy as np
import control as ct
import pytest

from trackloop import linear_loops as ll
from trackloop import margins as mg


@pytest.mark.parametrize("controller", ll.CONTROLLERS)
@pytest.mark.parametrize("lag", [0.0, 0.1])
def test_open_loop_closes_to_the_closed_loop_model(controller, lag):
    ol = ll.open_loop(controller, 10.0, lag)
    cl = ll.closed_loop(controller, 10.0, lag, dt=ll.DT)
    assert ol.A - ol.B @ ol.C == pytest.approx(cl.A, abs=1e-12)


def test_phase_margin_of_a_sampled_integrator_matches_the_formula():
    wc, dt = 2.0, 0.05
    system = ct.StateSpace([[1.0]], [[wc * dt]], [[1.0]], [[0.0]], dt)      # L = wc dt / (z - 1)
    theta = 2.0 * np.arcsin(wc * dt / 2.0)                                # crossover: |e^{j theta} - 1| = wc dt
    pm, w = mg.phase_margin(system)
    assert pm == pytest.approx(90.0 - np.degrees(theta / 2.0), abs=1e-3)
    assert w == pytest.approx(theta / dt, rel=1e-3)


def test_frequency_response_matches_python_control():
    system = ll.open_loop("Stanley", 10.0, 0.1)
    w = np.array([0.5, 3.0, 20.0])
    ours = mg.frequency_response(system, w)
    ref = np.array([system(np.exp(1j * wi * system.dt)) for wi in w]).ravel()
    assert ours == pytest.approx(ref, abs=1e-9)


@pytest.mark.parametrize("controller", ll.CONTROLLERS)
def test_gain_margin_edges_separate_stable_from_unstable(controller):
    system = ll.open_loop(controller, 10.0, 0.1)
    _, up = mg.gain_margins(system)
    assert 1.0 < up < np.inf
    a, b, c = system.A, system.B, system.C
    assert mg._stable(a, b, c, up * 0.99) and not mg._stable(a, b, c, up * 1.01)


def test_unstable_loop_gives_nan_margins():
    system = ll.open_loop("Stanley", 80.0, 0.2)       # phase margin is negative there
    down, up = mg.gain_margins(system)
    assert np.isnan(down) and np.isnan(up)
    assert mg.phase_margin(system)[0] < 0.0


@pytest.mark.parametrize("controller", ll.CONTROLLERS)
def test_margins_are_positive_at_10_m_per_s(controller):
    m = mg.margins(controller, 10.0, 0.1)
    assert m["phase_margin_deg"] > 20.0 and m["gain_up"] > 1.5 and m["delay_margin_s"] > 0.0


def test_added_delay_agrees_with_the_delay_margin():
    # Pure Pursuit at 10 m/s, lag 0.1 s: one crossover, delay margin about 0.227 s (4.5 samples).
    system = ll.open_loop("Pure Pursuit", 10.0, 0.1)
    dm = mg.margins("Pure Pursuit", 10.0, 0.1)["delay_margin_s"]
    one = ct.tf([1.0], [1.0, 0.0], ll.DT)

    def stable_with(samples):
        loop = system
        for _ in range(samples):
            loop = ct.series(loop, one)
        closed = ct.feedback(ct.ss(loop), 1)
        return bool(np.all(np.abs(np.linalg.eigvals(closed.A)) < 1.0))

    assert stable_with(int(0.7 * dm / ll.DT)) and not stable_with(int(1.4 * dm / ll.DT))
