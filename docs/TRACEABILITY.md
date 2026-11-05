# Where each README number comes from

Every result in the README is produced by a script in `scripts/`, stored in a CSV in `data/results/` (or in `data/paths/`, `data/sysid/`), and discussed in a note in `docs/notes/`. The table lists them by README section. "Checked by test" means `tests/test_traceability.py` recomputes the quoted numbers from the committed CSV, so a README number that disagrees with its table fails a test.

| README section | Numbers quoted | Stored in | Discussed in | Script | Checked by test |
| --- | --- | --- | --- | --- | --- |
| Paths from my planners | 10 thesis paths of 195 to 208 m, 9 of 10 clip an obstacle (cut 0.21 to 1.13 m), curvature 0.294 and 0.279 on map 3; 9 Reeds-Shepp paths, 7 cusps | `data/paths/` (JSON per path), `data/results/feasibility.csv` (curvature) | `docs/notes/planner_export_math_reference.md`, `docs/notes/rs_export_math_reference.md` | `scripts/export_thesis_paths.py`, `scripts/export_rs_paths.py` | path hashes in `tests/test_testset.py` |
| Tracking paths with cusps | max cross-track error table, 36 of 36 runs finish, 1 m errors on `parallel_park` and `side_slot` | `data/results/cusp_tracking.csv` | `docs/notes/cusp_tracking_math_reference.md` | `scripts/track_cusp_paths.py` | no |
| Test set and feasibility | 2 of 19 paths pass, lag distances 0.93 and 1.86 m, curvature limit 0.2534 | `data/results/feasibility.csv` | `docs/notes/feasibility_math_reference.md` | `scripts/check_testset_feasibility.py` | `tests/test_feasibility.py` |
| Speed applicability | 5, 20 and 80 m/s^2 against 7.8 | `data/results/speed_applicability.csv`, `data/results/feasibility.csv` | `docs/notes/feasibility_math_reference.md` (addendum) | `scripts/speed_applicability.py` | `tests/test_feasibility.py` |
| Metrics | Stanley chatter 3.6 per second against about 0.33 | `data/results/cusp_metrics.csv` | `docs/notes/metrics_math_reference.md` | `scripts/score_cusp_runs.py` | no |
| Benchmark, parking family | 1140 runs, Pure Pursuit lowest on 18 of 19 paths at 3 m/s | `data/results/benchmark_parking.csv`, `data/results/benchmark_parking_summary.csv` | `docs/notes/benchmark_design.md` | `scripts/run_benchmark_parking.py` | `tests/test_harness.py` |
| Road family | 12 paths, 2.0 m/s^2, slip about 1 degree | `data/paths/road/` | `docs/notes/road_family_math_reference.md` | `scripts/export_road_paths.py` | `tests/test_road_paths.py` |
| Road-family benchmark | 240 runs, front slip up to 7 degrees at 20 m/s | `data/results/benchmark_road.csv`, `data/results/benchmark_road_summary.csv` | `docs/notes/benchmark_road_note.md` | `scripts/run_benchmark_road.py` | `tests/test_regression_controllers.py` (reference runs) |
| Model-error cases | ratios against nominal (up to 2.8 for the dynamic MPC at 20 m/s), 5 runs unfinished | `data/results/model_error_road.csv`, `data/results/model_error_parking.csv`, `data/results/model_error_summary.csv` | `docs/notes/model_error_note.md` | `scripts/run_model_error.py` | `tests/test_model_error.py` |
| Monte Carlo benchmark | road 20 m/s means 0.045, 0.054, 0.198, 0.257 m; parking 0.084, 0.109 and 0.132, 0.286 m; 34,560 runs; largest half-width 4.9 percent | `data/results/mc_summary.csv`, `data/results/mc_pairs.csv`, `BENCHMARKS.md` | `docs/notes/root_cause_note.md` | `scripts/run_monte_carlo.py`, `scripts/make_benchmarks_md.py` | yes |
| Linear closed-loop models | gap at a 2 m offset 22, 16, 28 and 161 percent | `data/results/linear_loops_validation.csv` | `docs/notes/linear_loops_math_reference.md` (Checks) | `scripts/validate_linear_loops.py` | yes |
| Gain and phase margins | phase margin at 10 m/s, 0.1 s lag: 33, 37, 54, 60 degrees | `data/results/margins_vs_speed.csv` | `docs/notes/linear_loops_math_reference.md` (Margins) | `scripts/plot_margins.py` | yes |
| Stability limits | table of lowest unstable speeds, 38 of 38 runs agree | `data/results/stability_limits.csv` | `docs/notes/linear_loops_math_reference.md` (Stability limits) | `scripts/check_stability_limits.py` | yes |
| Where linearisation stops | gap 16 to 161 percent falls to 0.4 to 2.6 percent | `data/results/linearization_limits.csv` | `docs/notes/linear_loops_math_reference.md` (part 6) | `scripts/linearization_limits.py` | yes |
| Simulated identification data | 15 m/s, noise 0.003 rad/s and 0.05 m/s^2 | `data/sysid/` | `docs/notes/linear_loops_math_reference.md` (System identification data) | `scripts/make_sysid_data.py` | `tests/test_sysid_data.py` |
| Least-squares identification | within 0.2 percent, std 0.11 to 0.21 percent, coverage 93 to 97 percent, correlations 0.68 to 0.82 | `data/results/identification.csv` | `docs/notes/linear_loops_math_reference.md` (Least-squares identification) | `scripts/run_identification.py` | yes (all but the correlations) |
| Identification quality | cr std 1.9 percent at 5 m/s and 0.2 at 15 m/s, wrong mass shifts all estimates equally, 1 mrad steering noise biases by 0.8 to 0.9 percent | `data/results/identifiability_conditions.csv`, `data/results/identifiability_other.csv` | `docs/notes/linear_loops_math_reference.md` (Identification quality) | `scripts/study_identifiability.py` | `tests/test_identifiability.py` |
| Controllers on the identified model | combined case at 20 m/s: LQR 2.6 to 1.4, MPC 2.8 to 1.6; identified within 0.07 percent of the true-parameter design | `data/results/identified_models.csv`, `data/results/identified_design.csv`, `data/results/identified_design_runs.csv` | `docs/notes/linear_loops_math_reference.md` (Controllers designed on the identified model) | `scripts/run_identified_design.py` | yes (the ratios) |

## Not in a table

These README numbers are printed by their script but not saved to a CSV, so no test recomputes them: the recovery times in "Current results" (`scripts/plot_recovery.py`), the cusp stop overshoot of about 0.07 m and the 0.2 m and 0.056 m actuator comparisons (`docs/notes/cusp_tracking_math_reference.md`) and the correlations (0.68 to 0.82) in the identification section. Anything in the README without a source in this file should be treated as unverified.

## Corrections found while building this table

- The MPC phase margin at 10 m/s with a 0.1 s lag is 60.49 degrees in `margins_vs_speed.csv` (the note gives 60.5). The README said 61, a double rounding; it now says 60.
