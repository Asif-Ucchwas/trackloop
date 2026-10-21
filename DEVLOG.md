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
