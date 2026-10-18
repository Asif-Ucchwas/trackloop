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
