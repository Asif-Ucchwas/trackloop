import numpy as np

from trackloop import harness_road as hr
from trackloop import identified_design as idz
from trackloop.model_error import dynamic_plant

PATH = "road_curve_v20"


def rms(controller, case, variant, seed=1):
    return idz.run_one((PATH, controller, seed, case, variant))["rms_lateral"]


def test_the_nominal_variant_reproduces_the_stage_4_run():
    for c in idz.MODEL_BASED:
        assert rms(c, "stiff_m30", "nominal") == hr.run_one((PATH, c, 1, "stiff_m30"))["rms_lateral"]


def test_the_identified_model_has_the_same_dynamics_as_the_plant_even_with_the_mass_wrong():
    for case in ("mass_p20", "combined"):
        plant, model = dynamic_plant(case), idz.identified(case)[0]
        got = np.array([model.cf, model.cr, model.inertia]) / model.mass
        true = np.array([plant.cf, plant.cr, plant.inertia]) / plant.mass
        assert np.abs(got / true - 1).max() < 5e-3


def test_a_controller_designed_on_the_identified_model_behaves_like_the_oracle_design():
    for c in idz.MODEL_BASED:
        for case in ("mass_p20", "stiff_m30"):
            assert abs(rms(c, case, "identified") / rms(c, case, "oracle") - 1) < 5e-3


def test_the_nominal_design_on_a_much_softer_plant_does_worse_at_speed():
    for c in idz.MODEL_BASED:
        assert rms(c, "stiff_m30", "nominal") > 1.2 * rms(c, "stiff_m30", "oracle")


def test_paired_ratio_uses_only_finished_pairs():
    a = [{"path": "p", "controller": "c", "seed": s, "speed": 5.0, "rms_lateral": v, "finished": f}
         for s, v, f in ((0, 2.0, True), (1, 3.0, True), (2, 9.0, False))]
    b = [{**r, "rms_lateral": 1.0, "finished": True} for r in a]
    assert idz.paired_ratio(a, b) == 2.5
