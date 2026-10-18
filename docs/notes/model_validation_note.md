# Model validation: kinematic versus dynamic

Code: src/trackloop/models/comparison.py (matched_turn),
scripts/model_comparison.py (table), src/trackloop/benchmark.py (benchmark speeds).
Tests: tests/test_model_agreement.py.

## 1. Question

How wrong is the kinematic model at each speed, taking the dynamic model as the more
faithful of the two? The answer fixes the speeds used to compare controllers.

## 2. Matched turn procedure

For each speed v:

1. Choose a constant steering angle delta = atan(L ay / v^2) with ay = 2 m/s^2. This
   gives the kinematic model a lateral acceleration of 2 m/s^2, so every speed sees an
   equally demanding turn. If delta exceeds the 0.6 rad steering limit (speeds below
   about 2.8 m/s), the turn is reported as not possible.
2. Start both models at the same pose and speed. The kinematic state is the rear axle.
   The dynamic state is the center of gravity (CG), which sits lr ahead of the rear
   axle, so its start is shifted by lr.
3. Run both for 5 s at dt = 0.01 s with the same steering command and limits.
4. Convert the dynamic path to the rear axle, rear = CG - lr (cos psi, sin psi), before
   comparing. Without this the paths differ by lr = 1.5 m from the first step, which is
   bookkeeping, not model error.

## 3. Metrics (defined here because they are computed only in code)

- ratio = r_dyn(T) / r_kin(T), where r_kin = v tan(delta_final) / L and r_dyn is the
  dynamic yaw rate at T = 5 s. A ratio below 1 means the kinematic model overstates
  turning.
- dpsi = psi_kin(T) - psi_dyn(T), the heading difference in rad.
- err = distance between the kinematic rear axle and the dynamic rear axle at T. It
  depends on the 5 s horizon because a faster car travels farther, so it is not a
  model-quality number on its own.
- beta = atan2(vy, vx) of the dynamic model at T, the body sideslip angle at the CG.
- 0.95 crossing: ratio computed on a 0.1 m/s grid from 3 to 30 m/s and interpolated
  linearly to find the speed where it falls to 0.95. The 0.95 level is a chosen
  threshold, not a derived one.

## 4. Closed-form ratio

From the steady-state result in dynamic_math_reference.md, r_dyn = v delta / (L + K v^2),
and with small angles r_kin = v delta / L, so

    ratio = L / (L + K v^2)

At 20 m/s: 2.7 / (2.7 + 0.00375 x 400) = 2.7 / 4.2 = 0.6429. The simulation gives
0.6428.

Heading difference: r_kin = ay / v = 0.1 rad/s at 20 m/s, so the steady-rate part of
dpsi is (0.1 - 0.06428) x 5 = 0.1786 rad. The table shows 0.1835. The remaining
0.005 rad is expected to be the short build-up of yaw rate after the steering step;
it was not measured separately.

The formula assumes small angles. Below about 10 m/s the required steering is large
and the simulation departs from it: at 6 m/s the formula gives 0.9524 and the
simulation 0.9485. The 0.95 crossing is therefore 5.89 m/s from the simulation against
6.16 m/s from the formula, and the simulated value is the one used.

## 5. Sideslip derivation

In a steady turn the rear slip angle is alpha_r = -(vy - lr r) / vx = -beta + lr r / vx.
The rear axle force balance gives alpha_r = Fyr / Cr = m ay lf / (L Cr). With ay = vx r
and R = vx / r:

    beta = lr / R - m vx^2 lf / (R L Cr)

The first term is geometry: the CG sits ahead of the rear axle, so it slides toward the
inside of the turn. The second term is rear tire slip, which pushes it outward. They
cancel when

    vx^2 = lr L Cr / (m lf)
    vx   = sqrt(1.5 x 2.7 x 100000 / (1500 x 1.2)) = sqrt(225) = 15 m/s

The table shows beta = -0.000 deg at 15 m/s, +0.837 deg at 10 m/s and -0.215 deg at
20 m/s.

Worked example at 10 m/s:

    delta = atan(2.7 x 2 / 100) = 0.053948 rad
    r = 10 x 0.053948 / 3.075 = 0.17544 rad/s
    R = v / r = 57.00 m
    beta = 1.5 / 57.00 - 1500 x 100 x 1.2 / (57.00 x 2.7 x 100000)
         = 0.026316 - 0.011696 = 0.014620 rad = 0.838 deg

The simulation gives 0.837 deg; the formula drops the nonlinear terms.

## 6. Results (placeholder vehicle parameters)

    v (m/s)   ratio    mismatch
    5         0.9605   4%
    10        0.8770   12%
    20        0.6428   36%

The kinematic model is within 5% of the dynamic model below 5.89 m/s. The yaw rate
gain peaks at the characteristic speed sqrt(L / K) = 26.83 m/s.

## 7. Benchmark speed rule

Take one speed per regime: kinematic model adequate (5 m/s), moderate mismatch with
small angles (10 m/s), clear mismatch (20 m/s). All three are below the characteristic
speed, so yaw rate gain still rises across the benchmark. These are stored in
BENCHMARK_SPEEDS.

## 8. Limits of these findings

- Parameters are placeholders. If mass, tire stiffness or geometry change, rerun
  scripts/model_comparison.py and revisit the speeds.
- Linear tires are assumed. All turns here are at 2 m/s^2, well below tire saturation,
  so the findings say nothing about hard cornering.
- The dynamic model is itself an approximation of a real car. "Mismatch" means the gap
  between two models, not between a model and a vehicle.
- The 0.95 threshold, the 2 m/s^2 turn and the 5 s duration are chosen values.
