# DEVLOG

## 2026-10-18: environment setup
- Project venv created with Python 3.12.3 on Ubuntu 24.04.
- Installed numpy 2.5.4, scipy 1.18.1, matplotlib 3.11.2, control 0.10.2, osqp 1.1.3, pytest 9.1.1.
- pip printed dependency warnings about two ROS packages that are not part of this project. Checking whether the shell exposes ROS Python paths inside the venv.
- Finding: the shell exports PYTHONPATH with the ROS Jazzy install and the planner_nodes workspace, so ROS Python folders are visible inside the venv. Imports of our packages still resolved to the venv copies, but the setup is not reproducible.
- Fix: run `unset PYTHONPATH` after activating the venv. tests/test_environment.py fails if a ROS path is on sys.path.

## 2026-10-18: Task 2, kinematic bicycle model
- Model in src/trackloop/models/kinematic.py. State is [x, y, heading, speed, steering angle]; the steering angle is a state so the rate limit has memory.
- Integration is RK4 on pose and speed with the steering angle held constant across each step.
- Default parameters (wheelbase 2.7 m, steering limit 0.6 rad, rate limit 0.8 rad/s) are placeholders, not a specific car.
- Reverse driving (negative speed) works with the same equations.
- Environment lesson: with the ROS PYTHONPATH exported, pytest crashed on a missing yaml module from a ROS pytest plugin. Run `unset PYTHONPATH` after activating the venv.
- Wrote docs/notes/kinematic_math_reference.md with a worked example whose numbers were printed from the code (R 8.7284 m, omega 0.57284 rad/s, circle position error 6.581e-12 m over 10 s at dt 0.01 s).
- Added src/trackloop/models/dynamic.py (dynamic bicycle model, linear tires, 7 states) and tests/test_dynamic.py. Steady-state yaw rate matches the closed-form understeer formula within 0.02% at 10, 20 and 30 m/s (K = 0.00375). 15 tests pass.
- Wrote docs/notes/dynamic_math_reference.md.
- Cross-validated kinematic and dynamic models with matched constant-steering turns.
- Yaw rate ratio matches L/(L+K v^2) from 10 m/s up; kinematic within 5% below 5.89 m/s.
- Sideslip sign flips at 15.00 m/s, matching the closed form.
- Fixed benchmark speeds at 5, 10 and 20 m/s (src/trackloop/benchmark.py). 21 tests pass.
- Wrote docs/notes/model_validation_note.md.
- Added Pure Pursuit with speed-dependent lookahead (src/trackloop/controllers).
- Straight-line response matches the damped second-order theory (zeta 0.707, 4.32%).
- Circle is tracked with no steady error in the kinematic model, for any lookahead.
- Corner cut peaks at 0.1039 ld^2/R, 0.571 ld past the start of the curve.
- Measured constant 0.1018 to 0.1046 over R 20 to 80 m, ld 2 to 8 m. 43 tests pass.
- Wrote docs/notes/pure_pursuit_math_reference.md.
- Added Stanley (front-axle law with low-speed softening and a steering limit).
- Front axle error decays as exp(-k v t / (v + k_soft)) with no overshoot.
- On a circle the rear axle runs inside by R - sqrt(R^2 - L^2): 0.1831 m at R = 20 m.
- Simulation agrees: 0.18308 m at 0.02 m path spacing. 54 tests pass.
- Wrote docs/notes/stanley_math_reference.md.
- Added LQR lateral tracking: error model on the dynamic bicycle model, Riccati gains
  with Bryson weights, and a gain schedule on 41 speeds.
- The cross-track gain is sqrt(Q11 / R) = 0.4 at every speed (seen in the table, then
  derived).
- Added segment projection, a dynamic-model simulator and curvature feedforward.
- R = 50 m at 10 m/s: steady error -0.0499 m without feedforward (predicted -0.0499)
  and -6e-5 m with it. 77 tests pass.
- Wrote docs/notes/lqr_math_reference.md.
- Added linear MPC (condensed QP in deviations from the steady turn, OSQP, N = 20,
  dt = 0.05 s) with steering and steering-rate limits.
- Unconstrained MPC equals discrete LQR: 1e-15 at N = 3, 1.2e-9 at N = 20.
- Discrete gains at dt = 0.05 s are lower than the continuous LQR gains (K1 0.236
  against 0.400) and approach them as dt shrinks. Not derived.
- 5 m offset with the real rate limit: LQR overshoots 1.75 m and settles in 8.15 s,
  MPC does not overshoot and settles in 3.65 s. 89 tests pass.
- Wrote docs/notes/mpc_math_reference.md.
- Exported thesis planner paths into the common path format (Task 9). planner_export.py
  rounds each corner with a circular arc and samples every 0.1 m, so heading and
  curvature are exact for lines and arcs.
- The thesis Hybrid planner returns a plain list of (x, y) grid cells with no heading
  or curvature, and it draws from Python's global random generator: an unseeded run
  differs every time. Each export seeds it and saves the seed, the thesis commit and
  the Python version.
- Chose 3 m per cell (paths of 195 to 208 m) and a 6 m corner radius.
- 10 paths (two planners, five maps): the thesis waypoints never touch an obstacle but
  9 of 10 rounded paths clip one at 0.5 m sampling (corner cut 0.21 to 1.13 m). Map 3
  needs curvature 0.294 and 0.279 against the car's limit of 0.253.
- The export table came out identical on Python 3.13 and 3.14.4. 99 tests pass.
- Wrote docs/notes/planner_export_math_reference.md.
- Generated Reeds-Shepp reference paths from MultiPlan in the common path format
  (Task 10). rs_export.py turns MultiPlan's segment list into samples every 0.1 m
  with exact formulas for lines and arcs, a direction flag for reverse gear, and one
  sample exactly on each cusp.
- Convention: heading is the body heading, curvature is the steering curvature
  (positive for left, the same sign in both gears), so the heading changes at
  curvature times direction per metre.
- 9 paths at a 5 m radius from hand-chosen poses; 4 have cusps (7 in all). Each
  converted path ends on the requested goal pose to within 1e-14 m and 1e-15 rad and
  has MultiPlan's length. MultiPlan's own sampler ends up to 4.4e-5 m from my end
  point; I did not test which side of that gap is its step size.
- The u_turn word contains | but the path has no cusp, because its first piece has
  zero length. The cusp count follows the pieces, not the word.
- 110 tests pass. Wrote docs/notes/rs_export_math_reference.md.
- Added cusp-aware tracking (Task 11). segments.py splits a path at its cusps into
  single-gear segments that share the cusp sample. cusp_runner.py tracks one segment
  at a time: it lets the steering settle at a standstill, drives the segment, brakes to
  a stop at the cusp, and starts the next segment in the other gear.
- Reverse driving, per controller: Pure Pursuit needs no change (its forward formula is
  the reverse formula when the goal point is chosen in travel order). Stanley's forward
  law diverges in reverse (positive feedback on the heading), so it has a reverse branch
  that references the rear axle. The existing LQR and MPC use the dynamic model, which
  divides by speed (minimum 2 m/s), so they cannot be used at cusp speeds. I had said
  they would need only small changes; that was wrong. Wrote kinematic LQR and MPC in
  distance driven instead.
- All four controllers finish all nine Reeds-Shepp paths. On parallel_park and side_slot
  the kinematic LQR and MPC reach 1.0 to 1.1 m of cross-track error, which is the
  steering rate limit meeting an instant curvature step: 0.2 m at 0.5 m/s and 0.056 m
  with a 5 rad/s actuator. Feedforward made it worse at the rate limit; not
  investigated. Stanley is the weakest on forward arcs; not diagnosed.
- One run per controller and path, starting on the path, no noise: exploratory, not the
  benchmark. A progress message of mine said every cusp stop was within 0.55 m; the real
  worst case is 1.1 m (parallel_park and side_slot, LQR and MPC).
- 136 tests pass. Wrote docs/notes/cusp_tracking_math_reference.md. Results table:
  data/results/cusp_tracking.csv.
- Fixed the test set and added feasibility checks (Task 12). The set is every exported
  path (10 thesis, 9 Reeds-Shepp, 19 in all); no path is chosen or dropped by controller
  scores. data/paths/manifest.json holds the sha256 of each path file, and a test fails
  if a file changes or a path is added or removed.
- The Reeds-Shepp paths were used while developing the Task 11 controllers, so they are
  development paths, not held-out ones. No controller was run on a thesis path in Task 11.
- feasibility.py checks each path against the car's curvature limit (tan(0.6) / 2.7 =
  0.2534 per metre) and its steering rate (0.8 rad/s). A curvature step that the steering
  cannot follow at the benchmark speed is an event; its lag distance is the metres driven
  with the steering behind the path. Steps across a cusp are not counted.
- Result at 1.5 m/s: 2 of 19 paths pass both checks (rs_ahead and rs_three_point). Only
  the two seed-3 thesis paths exceed the curvature limit. Lag is 0.93 m on most
  Reeds-Shepp paths, 1.86 m on parallel_park and side_slot, and 0.79 to 1.76 m on the
  thesis paths. Failing paths stay in the set and are flagged.
- 151 tests pass. Wrote docs/notes/feasibility_math_reference.md. Results:
  data/results/feasibility.csv.
- Added a lateral-acceleration check to feasibility.py after comparing the Task 4
  benchmark speeds (5, 10, 20 m/s) with the frozen paths: a path is drivable at speed v
  when v^2 times its largest curvature is at most mu g (mu = 0.8, a placeholder).
- Result: 19 of 19 paths are drivable at 1.5 and 5 m/s, but only rs_ahead at 10 and 20
  m/s (a 5 m arc needs 20 and 80 m/s^2 there, against 7.85). I missed this in Task 12.
- Decision: two path families. Parking: the 19 frozen paths at low speed with the
  kinematic controllers. Road: paths scaled so each speed sees the same lateral
  acceleration, at 5, 10 and 20 m/s with the dynamic controllers. The road family is
  not built yet and must be frozen before any benchmark run.
- 157 tests pass. Results: data/results/speed_applicability.csv. Figure now has a third
  panel (largest drivable speed against the benchmark speeds).
- Task 13: wrote src/trackloop/metrics.py and docs/notes/metrics_math_reference.md. The
  repository measured lateral error two ways (nearest vertex in the kinematic runs,
  projection onto segments at the centre of gravity in the dynamic ones), so one definition
  now scores everything: signed distance to the path polyline at the rear axle, from the
  logged run only.
- Metrics: lateral and heading error (RMS, max), steering effort, steering-rate RMS and max,
  steering chatter (rate reversals per second), driving and total time, plus the cusp
  position error and settle time from the runner.
- Old and new lateral error agree to 0.0005 m on all 36 cusp runs, so Task 11 results stand.
  Stanley reverses its steering rate 3.6 times a second on average against about 0.33 for the
  other three controllers, which lateral error alone did not show. Cause not diagnosed.
- The dynamic-model path (centre of gravity to rear axle) is written down but not yet run.
  Also corrected the feasibility note: linear tires are confirmed from the dynamic note.
- 166 tests pass. Results: data/results/cusp_metrics.csv, docs/figures/metrics.png.
- Task 14 (parking family): wrote src/trackloop/harness.py and scripts/run_benchmark_parking.py.
  19 frozen paths x 4 kinematic controllers x 3 speeds (0.5, 1.5, 3.0 m/s) x 5 seeds = 1140
  runs, one row each, scored by trackloop.metrics. The seed fixes a start offset (up to 0.3 m)
  and heading error (up to 0.1 rad) that depend on the seed and path but not the controller.
- Runs are flagged, not dropped. The steering-rate check fails on 15 of 19 paths at every
  speed, so it is reported as a lag distance instead of a flag; "drivable" uses curvature and
  lateral acceleration only (2 paths fail it). Settings, git commit and package versions go
  to a JSON file. The time limit now scales with path length, since 300 s would cut off
  0.5 m/s thesis runs.
- Result: all 1140 finish, no errors, run is deterministic. LQR and MPC are not separable
  (within 0.007 m). At 3.0 m/s Pure Pursuit is lowest on 18 of 19 paths; LQR and MPC reach
  1.0 and 1.5 m on parallel_park and side_slot. Suspected cause is the steering rate limit
  meeting a curvature step; not shown. Road family and dynamic controllers are not built yet.
- 175 tests pass. Results: data/results/benchmark_parking*.csv, docs/figures/benchmark_parking.png.
- Road family (second half of Task 14): src/trackloop/road_paths.py builds 12 forward paths,
  4 shapes (curve, lateral_shift, s_curve, double_shift) x design speeds 5, 10 and 20 m/s,
  each with peak lateral acceleration 2.0 m/s^2 and lateral jerk 1.0 m/s^3 (placeholders).
  Curvature is v^2-scaled and ramps as a clothoid, so the steering never makes an instant
  step. Exported to data/paths/road/ with its own manifest and frozen before any controller
  was run on it. The 19-path parking manifest is unchanged.
- All 12 pass curvature, steering-rate and lateral-acceleration checks at their design speed.
  Derived: tire slip angles at 2.0 m/s^2 are about 1 degree, well inside the linear-tire
  range of the dynamic model. Renamed lane_change to lateral_shift: the sideways move is
  about 15 m, not a lane.
- 211 tests pass (36 new). Next: dynamic controllers on the road family.

## Task 14 closed: road-family benchmark

- Built a dynamic-model runner (constant speed, stops at the path end), a rear-axle adapter for
  Pure Pursuit and Stanley, and a road harness with the same paired seeded starts as parking.
- 240 runs, 0 errors, 0 unfinished. Results at commit a737cc6.
- Found that LQR and MPC at 20 m/s exceed the linear-tire friction limit in 10 and 5 of 20 runs;
  the figure shows the limits and the note says what that does to the claim.
- Not yet shown: whether Pure Pursuit and Stanley are tuned fairly at 20 m/s, why Stanley chatters
  at 5 m/s, why the seed 2 curve_v20 start is the worst for all four. All three go to Task 16.
- Next: Task 15, model-error cases on both families.

## Task 15 closed: model-error cases

- Added a first-order steering lag to both plants (0 by default, so earlier results are unchanged;
  a test checks this) and case definitions that change only the plant, never the controller.
- Road: 1,920 runs, 0 errors, 5 did not finish. Parking: 5,700 runs, 0 errors, all finished.
  Results at commit 636f30c, clean tree. 232 tests pass.
- Found: mass and tire stiffness errors grow with speed on the road, and the combined case is worst.
  In parking, steering lag hurts only the kinematic MPC (about 7 times nominal at 3 m/s, 0.2 s).
- Not yet shown: why the kinematic MPC is lag-sensitive and the LQR is not, why 5 dynamic MPC runs
  diverged at 20 m/s with 0.2 s lag, whether the softer-tire cases push more runs past the
  tire friction limit. All go to Task 16.
- Next: Task 16, Monte Carlo sized by confidence-interval width, BENCHMARKS.md, root causes.

## Task 16 closed: Monte Carlo, BENCHMARKS.md, root causes

- Seeds sized by interval width: 150 road, 120 parking. The first full run used 120 for both and
  four road cells (dynamic LQR and MPC at 10 and 20 m/s) were at 5.1 to 5.4 percent, so the road
  family was rerun at 150. All 36 cells are within 5 percent. Results at commit d44377d, clean tree.
- BENCHMARKS.md is generated from the result CSVs. I found a bug in my own generator (the relative
  column in parking used the wrong scope) and fixed it with a test. 249 tests pass.
- Root causes, each with a test and its limits in `docs/notes/root_cause_note.md`: the Kinematic MPC
  and the road MPC fail under steering lag because their first-step rate bound is measured from the
  lagging steering angle; Stanley's 5 m/s chatter is a path-spacing artifact (RMS unchanged); the
  worst start (`road_curve_v20`, seed 2) is driven by the heading error.
- Wrong hypothesis: I guessed that softer tires would push more runs past the friction limit. The
  data say fewer, because the limit angle rises as stiffness falls. The Task 15 note is corrected.
- Not shown: the mechanisms inside the dynamic model, and the open items listed in the note (why
  Pure Pursuit beats the kinematic LQR and MPC in parking, tuning fairness at 20 m/s, why lag helps
  Pure Pursuit and Stanley, why the LQR's mean exceeds its median at 20 m/s).
- Next: Stage 5, Task 17, linear closed-loop models of each controller in python-control.

## Task 17: linear closed-loop models

- Built `linear_loops.py`: gains for the four controllers, continuous and exact-discrete closed loops with an
  optional first-order steering lag state, `is_stable`. 24 tests (273 in total).
- Validated against the nonlinear simulation: about 2e-5 m at a 0.05 m offset; the difference grows to
  22/16/28/161 percent of a 2 m offset (Pure Pursuit/Stanley/LQR/MPC, worst of 9 speed-lag cases).
  The growth is the steering rate limit (found in Task 20).
- The MPC gain equals discrete LQR at dt: derived from the cost, supported by the simulation agreement.
- Next: Task 18, Bode plots and margins against speed.

## Task 18: Bode plots and margins against speed

- Added `open_loop` (loop broken at the steering command) and `margins.py`: gain margin by scaling the gain and
  checking the poles, phase margin from the frequency response. 20 tests (293 in total), including an analytic
  phase margin and an added-delay check against the delay margin.
- Phase margin falls with speed for every controller and lag. Zero phase margin appears only with a steering lag,
  at 56 to 64 m/s on the 1 m/s grid (Stanley 61 at 0.1 s; Stanley 56, MPC 64 at 0.2 s; the LQR figure above
  30 m/s was wrong and is corrected in Task 19). The nonlinear check is Task 19.
- LQR with no lag has the smallest upward gain margin (about 2 dB at 40 m/s); its crossover is near Nyquist. Sampling
  delay as the cause is my reading, not tested.
- Next: Task 19, find the zero-margin speeds precisely and check them in the nonlinear simulation.

## Task 19: stability limits against the nonlinear simulation

- Added `stability_limit.py`: lowest unstable speed of each sampled linear loop, and a comparison of the nonlinear
  simulation with the linear model near that speed. 11 tests (304 in total).
- Limits: Stanley 166.7 (no lag), 60.7 (0.1 s), 55.0 (0.2 s); Pure Pursuit 268.5 and 134.2 (0.1, 0.2 s); LQR 92.1 and
  MPC 63.2 (0.2 s); none up to 300 m/s for the rest. The simulation agrees on growth or decay in 38 of 38 runs.
- Found and fixed an error in Task 18: LQR holds its gain above 30 m/s, my model used the exact gain. LQR limit 54 -> 92.1.
- Wrong first: an envelope-slope growth estimate disagreed for Stanley without lag; the trajectories agree to 1e-5 and
  the estimator was at fault.
- Not found: why LQR and MPC grow faster than the linear model past the limit (departure near 1e-2 m).
- Next: Task 20, derivations and the limits of linearisation in the math note.

## Task 20: derivations and where linearisation stops

- Wrote the derivations (error model, sampling and lag, LQR and MPC gains, margins) with worked examples in the math
  note, and `linearization.py` to measure the model-versus-simulation gap with each limit removed. 3 tests (307 in total).
- Answered the Task 17 open question: the gap growing with offset is the 0.8 rad/s steering rate limit (and, for the
  MPC, its own rate constraint). With them removed the worst gap at 2 m is 0.4 to 2.6 percent instead of 16 to 161.
- Steering angle limit: not a cause. Tire saturation: cannot be tested, the simulation has linear tires.
- Still open: why LQR and MPC leave the linear model near 1e-2 m past the stability limit (Task 19).
- Stage 5 done. Next: Stage 6, Task 21, system identification.

## Task 21: simulated identification data

- `sysid_data.py` generates noisy yaw rate and lateral acceleration from the dynamic model under a sweep or a random
  binary steering signal; seeded, with the settings and true parameters written beside each CSV. 7 tests (314 total).
- Finding before estimating: m, Iz, cf and cr scaled together change nothing, so mass is not identifiable from these
  signals. The syllabus's four parameters become three (cf, cr, Iz with mass fixed) in Task 22.
- The random signal is rate limited on 14.8 percent of steps; the file holds the applied steering angle, not the command.
- Steering is treated as measured exactly (an easier problem than a real one); noise levels are my choice.

## Task 22: least-squares identification

- `identification.py`: sampled linear model, weighted least squares on yaw rate and lateral acceleration, covariance from
  (J^T J)^-1, a Monte Carlo over noise seeds, a mass-direction check. 5 tests (319 total).
- Estimates within 0.2 percent; predicted standard deviations match the spread over 100 seeds (13 percent smaller to 1 percent larger
  than predicted) and 95 percent coverage is 93 to 97 percent. cf shows a small bias of -0.04 percent from the model difference.
- Mass check confirmed numerically: the smallest singular value is 4e-11 of the largest with mass as a parameter.
- Best case only: same model class and known noise. An optimiser overflow warning from a trial with negative parameters (first
  version without bounds) was fixed with bounds.

## Task 23: identification quality

- `identifiability.py`: the covariance at the true parameters for any experiment, plus fits with a wrong mass, steering noise and large
  amplitudes. 5 tests (324 total). The study script takes about 4 minutes (about 300 fits).
- Findings: scaling laws hold (amplitude law fails once the rate limit is active, 0.1 rad); cr is poorly identified at low speed (7 times
  cf's std at 5 m/s); wrong mass shifts all estimates exactly 1:1 with unchanged chi-squared; steering noise gives a downward bias that
  reaches the reported std from 0.5 mrad (4 to 7 times it at 1 mrad); bias from the model difference crosses the std near 0.05 rad.
- Not explained: the covariance overestimates the sweep Iz spread by 13 to 21 percent; the size of the steering-noise bias was not derived.
- The Task 21 note said Task 23 could relax exact steering measurement; this is where it was relaxed, and it matters.

## Task 24: controllers on the identified model

- `identified_design.py`: identify each Stage 4 plant from a sweep (nominal mass assumed), design LQR and MPC on the nominal, identified
  and true parameters, run on the true plant. 1680 new runs (about 1 minute on 2 cores), 5 tests (329 total). Pure Pursuit and Stanley use
  no vehicle parameters and were not re-run.
- Identified equals oracle: worst median ratio 1.0007, worst single run 0.49 percent. Wrong-model penalty (up to 1.9 times at 20 m/s) is removed.
- Task 23's prediction that a wrong mass is harmless held (mass +/-20 percent cases identical to oracle).
- Lag is not identified: lag cases unchanged, the same 5 MPC runs diverge at 0.2 s.
- Unexplained: at 5 m/s the nominal design beats the oracle on RMS error in softer-plant cases (0.91 to 0.95) with the same effort.
- Stage 6 done. Next: Stage 7, packaging (Task 25).

## Task 25: README overview

- Added an overview, an architecture diagram (`docs/figures/architecture.png`, drawn with mermaid), a main-results table and one
  reproduce command per stage at the top of the README, and replaced the old "benchmarks are still to come" intro, which had been
  out of date since Task 14. `tests/test_readme.py` fails if the README names a file that does not exist. 332 tests.
- I have not run every reproduce command in the table end to end; the test only checks that each named script exists.

## Task 26: tests and CI

- `tests/test_analytic_models.py`: closed-form checks of both models (circle radius, constant-acceleration line, the understeer
  formula at three speeds, neutral steer, oversteer). `tests/test_regression_controllers.py`: 16 frozen runs, four controllers on
  two road and two parking paths, and the same runs against the committed benchmark CSVs. 360 tests.
- `.github/workflows/ci.yml` runs pytest on Python 3.12 for every push to `main` and pull request. Not yet seen to pass on GitHub.

## Task 27: traceability

- `docs/TRACEABILITY.md` maps each README section to the CSV, note and script behind its numbers, and
  `tests/test_traceability.py` recomputes the CSV-backed ones.
- Found one error this way: the README gave the MPC phase margin as 61 degrees at 10 m/s with 0.1 s lag; the CSV has 60.49 (the
  note rounds to 60.5, and I rounded that again). Now 60.
- Every other number I checked agrees with its CSV: the road and parking means, the 36 of 36 interval target, the Task 17 gap
  percentages, the stability limits, the 0.4 to 2.6 percent gap, the identification ranges and the identified-design ratios.

## Failures, wrong hypotheses and open items (index of this log)

Mistakes of mine that were caught, with the task that fixed them:
- Task 2 (environment): the shell's ROS `PYTHONPATH` leaked into the venv; fixed with `unset PYTHONPATH` and a test.
- Task 11: I said the dynamic LQR and MPC would need only small changes for reverse driving. Wrong: they divide by speed, so
  kinematic versions in distance driven were written. I also reported every cusp stop within 0.55 m; the worst case is 1.1 m.
- Task 12 to 14: I missed that the 5, 10 and 20 m/s benchmark speeds need far more lateral acceleration than the frozen paths allow
  at those speeds, which led to the parking and road families.
- Task 16: a bug in my BENCHMARKS.md generator (wrong scope in the parking relative column), fixed with a test. The first Monte Carlo
  run missed the 5 percent target in four road cells, so the road family was rerun with 150 seeds.
- Task 16, wrong hypothesis: I expected softer tires to push more runs past the friction limit; the data say fewer.
- Task 18: the LQR model used the exact gain above 30 m/s where the controller holds a constant one (limit 54 m/s, correct 92.1).
- Task 19: an envelope-slope growth estimate disagreed with the simulation for Stanley; the estimator was at fault.
- Task 22: the first fit had no bounds and tried negative parameters; fixed with bounds.
- Task 27: the MPC phase margin was quoted as 61 degrees, the table says 60.49.

Open and not explained:
- Why Pure Pursuit beats the kinematic LQR and MPC in parking at 1.5 and 3 m/s; whether Pure Pursuit and Stanley are tuned fairly at 20 m/s.
- Why a steering lag helps Pure Pursuit and Stanley; why the dynamic LQR's mean exceeds its median at 20 m/s.
- Why Stanley is weakest on forward arcs and chatters at 3.6 reversals per second in the cusp runs (the 5 m/s road chatter is a path-spacing effect).
- Why feedforward made the rate-limited parking cases worse; why the discrete gains at 0.05 s are lower than the continuous ones (not derived).
- Why the LQR and MPC leave the linear model near 1e-2 m past the stability limit; the sampling-delay explanation of the LQR's small gain margin was not tested.
- Why the covariance overestimates the sweep Iz spread by 13 to 21 percent; the size of the steering-noise bias was not derived.
- Why at 5 m/s the nominal design beats the true-parameter design on RMS error in softer plants.
- Tire saturation cannot be tested: the simulated tires are linear.
- Everything here is simulation with placeholder vehicle parameters; nothing was tested on a real vehicle.

## Task 28: repository audit and release

- Audit of the tracked files (grep, read-only): no local paths or machine names, no unfinished-work markers, no tokens or passwords, no mentions
  of assistants, no attribution trailers in the whole history. The "personal data" pattern matched only digits inside numbers.
  The largest tracked files are `mc_parking.csv` (1.8 MB) and `model_error_parking.csv` (1.2 MB). CI passed on GitHub for Tasks 26 and 27.
- Found and fixed: no LICENSE (added MIT, copyright Md Asifuzzaman), no authors, license, readme or URL in `pyproject.toml`, two README
  sentences that still said the road family "will be" built and "comes next", and the GitHub description ended in "Work in progress".
- `tests/test_repo_hygiene.py` keeps the first two results true: no local paths or unfinished markers in text files, licence and metadata
  present, no attribution trailers in the history. 373 tests.
- Left as they are: four diagnostic scripts named `scripts/test_*.py` (they are run by hand, not collected by pytest, and the notes
  refer to them by name); pyflakes was not run because it is not installed in the venv.
- Tag `v1.0-portfolio` is created on this commit once CI passes on it.
