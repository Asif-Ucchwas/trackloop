# LQR lateral tracking: math reference

Code: src/trackloop/controllers/lqr.py, src/trackloop/projection.py,
src/trackloop/simulate_dynamic.py. Tests: tests/test_lqr.py,
tests/test_lqr_tracking.py, tests/test_projection.py.

## 1. Assumptions

- The dynamic bicycle model with linear tires (see dynamic_math_reference.md), with
  the placeholder parameters in DynamicParams: m = 1500 kg, Iz = 2500 kg m^2,
  lf = 1.2 m, lr = 1.5 m, cf = 80000 N/rad, cr = 100000 N/rad (axle stiffnesses).
- The design model is linear. It holds for constant speed vx, small angles and a
  path of constant curvature.
- Errors are measured at the center of gravity (CG) against the foot point of the
  path, found by projecting onto the segment between two path points.
- Forward driving only. Below 2 m/s the 2 m/s gains are used.

## 2. Error model

State x = [e1, e1_dot, e2, e2_dot]:

    e1       distance of the CG from the path, positive left of the path (m)
    e1_dot   its rate (m/s)
    e2       vehicle heading minus path heading (rad)
    e2_dot   its rate (rad/s)

The input is the front steering angle delta. Let psi_dot_des be the heading rate of
the path, kappa * vx for path curvature kappa.

Small angles make the rate of e1 the velocity across the path: e1_dot = vy + vx * e2.
Differentiate at constant vx, using e2_dot = r - psi_dot_des:

    e1_ddot = vy_dot + vx * (r - psi_dot_des)

The dynamic model has

    m (vy_dot + vx r) = Fyf + Fyr          Iz r_dot = lf Fyf - lr Fyr
    Fyf = cf (delta - (vy + lf r) / vx)    Fyr = -cr (vy - lr r) / vx

so e1_ddot = (Fyf + Fyr) / m - vx * psi_dot_des. Put vy = e1_dot - vx * e2 and
r = e2_dot + psi_dot_des into the forces, treat psi_dot_des as constant (so
r_dot = e2_ddot) and collect terms:

    e1_ddot = -(cf + cr) / (m vx) * e1_dot + (cf + cr) / m * e2
              + (cr lr - cf lf) / (m vx) * e2_dot + cf / m * delta
              + [(cr lr - cf lf) / (m vx) - vx] * psi_dot_des
    e2_ddot = (cr lr - cf lf) / (Iz vx) * e1_dot + (cf lf - cr lr) / Iz * e2
              - (cf lf^2 + cr lr^2) / (Iz vx) * e2_dot + lf cf / Iz * delta
              - (cf lf^2 + cr lr^2) / (Iz vx) * psi_dot_des

These are rows 2 and 4 of x_dot = A x + B delta + Bd psi_dot_des. Rows 1 and 3 are
e1_dot and e2_dot. Column 1 of A is zero because no force depends on e1: moving the
whole car sideways changes nothing.

Worked example at vx = 10 m/s (rows and columns numbered from 1):

    A[2,2] = -(180000) / (1500 * 10) = -12        A[2,3] = 180000 / 1500 = 120
    A[2,4] = (150000 - 96000) / (1500 * 10) = 3.6
    A[4,2] = (150000 - 96000) / (2500 * 10) = 2.16
    A[4,3] = (96000 - 150000) / 2500 = -21.6
    A[4,4] = -(1.44 * 80000 + 2.25 * 100000) / (2500 * 10) = -13.608
    B  = [0, 80000 / 1500, 0, 1.2 * 80000 / 2500] = [0, 53.333, 0, 38.4]
    Bd = [0, 3.6 - 10, 0, -13.608] = [0, -6.4, 0, -13.608]

The open loop poles are 0, 0 and -12.804 +/- 3.630j. The two zero poles are the two
position errors, which nothing pushes back.

## 3. The LQR design

Choose delta = -K x to minimise J = integral of (x^T Q x + R delta^2) dt. The
minimum cost from x0 is x0^T P x0, where P solves the Riccati equation

    A^T P + P A - P B R^-1 B^T P + Q = 0        K = R^-1 B^T P

(scipy solve_continuous_are). The tests check the residual (below 1e-9), a second
solution from the eigenvectors of the Hamiltonian matrix, and the cost by
integrating the closed loop (trapezoid, dt = 0.001 s, 40 s, agreement to 1e-5).

Bryson's rule sets each weight to 1 / (acceptable size)^2: x_max = (0.5 m, 1 m/s,
0.1 rad, 0.3 rad/s) and u_max = 0.2 rad give Q = diag(4, 1, 100, 11.11), R = 25.
These sizes are a starting guess, not tuned.

Worked example at 10 m/s:

    K = [0.40000, 0.10341, 2.49286, 0.45515]      P[1,1] = 3.52694
    x0 = [0.5, 0, 0.05, 0]:   x0^T P x0 = 1.17554

## 4. Why the cross-track gain is sqrt(Q11 / R)

The gain table gave K1 = 0.4000 at every speed. This was seen in the output first and
derived afterwards. Entry (1,1) of the Riccati equation is

    (A^T P)11 + (P A)11 - (P B)1^2 / R + Q11 = 0

Column 1 of A is zero, so (A^T P)11 = sum over k of A[k,1] P[k,1] = 0, and the same
for (P A)11. What is left is (P B)1^2 = Q11 * R, so K1 = (P B)1 / R = sqrt(Q11 / R),
taking the positive root, which is the one the solver returns.

Check at 10 m/s: (P B)1 = 10.000 = sqrt(4 * 25), so K1 = 10 / 25 = 0.4. This depends
on Q11 and R only, not on the vehicle or the speed. A different weight changes it.

## 5. Closed loop poles

Eigenvalues of A - B K (the solution at each speed, tests require every real part
below -0.3):

    vx (m/s)   slowest pole   fastest pole   other poles
    2          -0.393         -78.315        -56.649, -1.321
    5          -0.937         -33.822        -30.895, -2.353
    10         -1.544         -29.161        -14.322, -3.573
    20         -1.884         -27.864        -5.475 +/- 3.728j
    30         -1.955         -27.617        -4.557 +/- 4.681j

The slowest pole sets how fast the cross-track error dies: time constant 1 / 0.393 =
2.5 s at 2 m/s (about 10 s to settle within 2%) and 0.65 s at 10 m/s. The weights do
not change with speed. The likely cause of slow low-speed tracking is e1_dot = vx *
e2: a given heading error moves the car sideways in proportion to speed. This was
not tested separately.

## 6. Gain schedule

Solving the Riccati equation every control step is avoidable: the gains depend on vx
only. They are solved on 41 speeds (every 0.25 m/s from 2 to 6 m/s, every 1 m/s from
6 to 30 m/s) and interpolated linearly. Outside 2 to 30 m/s the end gains are used.

Schedule error = the largest, over the four gains, of |interpolated - exact| / |exact|.
Evaluated at speeds from 2.05 to 29.9 m/s in steps of 0.37 m/s, the worst is 1.8e-3,
at 6.49 m/s. A uniform 1 m/s grid gave 2.5% near 2.5 m/s, because the gains change
fastest at low speed.

## 7. Measuring the errors from the vehicle state

Let (x, y) be the CG, psi the heading, and the foot point have heading psi_p and
curvature kappa (both interpolated along the segment).

    e1      = signed distance to the foot point, positive left
    e2      = wrap(psi - psi_p)
    e1_dot  = vx sin(e2) + vy cos(e2)
    e2_dot  = r - psi_dot_des

The foot point moves along the path at (vx cos(e2) - vy sin(e2)) / (1 - kappa * e1):
the velocity along the path tangent, divided by a factor that is below 1 on the inside
of a curve, because the inner arc is shorter. So

    psi_dot_des = kappa * (vx cos(e2) - vy sin(e2)) / (1 - kappa * e1)

For small angles this is kappa * vx, the value used in the design model.

## 8. Curvature feedforward

On a circle of curvature kappa at speed vx, with the car on the path, a steady turn
needs (dynamic_math_reference.md):

    delta_ss = (L + K_us vx^2) kappa       K_us = (m / L)(lr / cf - lf / cr)
    beta_ss  = (lr - lf m vx^2 / (cr L)) kappa

The velocity then follows the path tangent, so the heading is the path heading minus
the sideslip angle: e2 = -beta_ss, with e1 = 0 and e1_dot = 0. The feedback term
-K3 * e2 = +K3 * beta_ss would add steering the turn does not need, so the
feedforward removes it:

    delta_ff = delta_ss - K3 * beta_ss

and the total at steady state, delta_ff + K3 * beta_ss, is exactly delta_ss.

Without feedforward the steady state has e1 != 0. Setting delta = delta_ss in
delta = -K1 e1 - K3 e2 with e2 = -beta_ss gives

    e1_ss = -(delta_ss - K3 * beta_ss) / K1

Worked example, R = 50 m (kappa = 0.02), vx = 10 m/s, K3 = 2.49286, K1 = 0.4:

    delta_ss = (2.7 + 0.00375 * 100) * 0.02 = 0.0615 rad
    beta_ss  = (1.5 - 1.2 * 1500 * 100 / (100000 * 2.7)) * 0.02 = 0.016667 rad
    K3 * beta_ss = 0.04155       delta_ff = 0.0615 - 0.04155 = 0.01995 rad
    e1_ss = -0.01995 / 0.4 = -0.04988 m   (without feedforward)

## 9. Results in simulation (placeholder parameters, steering rate limit removed)

Straight path, vx = 10 m/s, start 0.2 m left of the path, dt = 0.01 s. Prediction:
x(t) = expm((A - B K) t) x0.

    t (s)      1         2         3
    simulated  0.06604   0.01481   0.00319
    predicted  0.06658   0.01492   0.00321

The largest gap over 6 s is 7.9e-4 m, on a 0.2 m start error (0.4%). The simulated
error stays positive: no overshoot. The gap was not traced to a cause. The model
drops the nonlinear terms (sin, atan, delta in the lateral force) and the simulation
holds each command for one step.

Circle (arc long enough for 25 s), car starts on the path at speed. Steady error is
the mean of the last 4 s:

    R (m)  vx (m/s)   no feedforward   predicted    feedforward
    50     10         -0.04989         -0.04988     -6.3e-5
    100    20         -0.2011          -0.2014      -1.4e-4

At R = 50, vx = 10 with feedforward the mean steering is 0.06152 rad (delta_ss is
0.0615) and the mean sideslip angle 0.016667 rad (beta_ss is 0.016667).

## 10. Quantities computed only in code

- LQRController.errors: [e1, e1_dot, e2, e2_dot] as in section 7, from the last state.
- LQRController.steer_ff: delta_ff of section 8 (0 when feedforward is off).
- Steady error in section 9: mean of the cross-track error of the CG over the last
  400 samples (4 s at dt = 0.01 s), measured with project() by simulate_dynamic.
- Spread in the tests: max minus min of the same 400 samples.
- Gap: max over the samples of |simulated - predicted| cross-track error.
- Schedule error: defined in section 6.

## 11. Limits of these findings

- The simulation plant is the same model family as the design (dynamic bicycle, linear
  tires, same parameters). Agreement with the linear prediction shows the code matches
  the derivation. It does not show the controller works on a real car or on a plant
  with tire saturation, load transfer or actuator lag.
- Constant speed. The gains are scheduled on vx but the design assumes vx does not
  change. Braking or accelerating while turning was not tested.
- Small errors. The linear model drops nonlinear terms; large initial errors were not
  tested. The steering limit of 0.6 rad clips the command (tested on a hand case).
- psi_dot_des is treated as constant. A change of curvature (a straight into a
  curve) produces a transient this derivation does not cover. That comes with the
  benchmark paths.
- Tests use a steering rate limit of 100 rad/s. The default 0.8 rad/s limit was not
  tested for this controller.
- The weights (Bryson sizes) and every parameter are placeholders, not tuned.
- Below 2 m/s the 2 m/s gains are held and the model is not trusted.
