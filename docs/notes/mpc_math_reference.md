# Linear MPC: math reference

Code: src/trackloop/controllers/mpc.py, scripts/mpc_vs_lqr.py. Tests:
tests/test_mpc.py. Builds on lqr_math_reference.md (error model in section 2, steady
turn in section 8).

## 1. Assumptions

- The error model of lqr_math_reference.md, with the same placeholder parameters and
  the same Bryson weights Q = diag(4, 1, 100, 11.11), R = 25.
- Over the horizon the speed is held at its current value (at least 2 m/s) and the
  path curvature at its value at the current foot point. There is no preview of a
  curve ahead.
- Control period dt = 0.05 s, horizon N = 20 steps (1 s). The steering is held
  constant between samples (zero-order hold).
- The model input is the steering angle itself. The steering actuator is not a model
  state: its limits enter only as constraints.
- The model is linear and the solver is OSQP, run to a tolerance of 1e-9.

## 2. Prediction model

With a constant steering over one period, x(t + dt) = Ad x(t) + Bu delta, where
Ad = exp(A dt) and Bu is the integral of exp(A s) B over 0 to dt. Both come from one
matrix exponential:

    exp( [[A, B], [0, 0]] * dt ) = [[Ad, Bu], [0, 1]]

Worked example at 10 m/s, dt = 0.05 s (rows and columns numbered from 1). The series
Ad = I + A dt + (A dt)^2 / 2 + ... gives, after one, two and three terms:

    Ad[1,2]:  0.05000   0.03500   0.03816    exact 0.03775
    Ad[2,2]:  0.40000   0.58972   0.55303    exact 0.55739
    Bu[2]:    2.66667   2.03947   2.23036    exact 2.19733
    Bu[4]:    1.92000   1.41082   1.48644    exact 1.48086

Ad[1,2] after two terms is dt + A[2,2] dt^2 / 2 = 0.05 - 12 * 0.00125 = 0.035. The
series converges slowly because |A[2,3]| dt = 6 is large, which is why the code uses
the matrix exponential. Full result: Bu = [0.05826, 2.19733, 0.04040, 1.48086].

Deviation coordinates. Let (x_ss, delta_ss) be the steady turn of lqr_math_reference.md
section 8, with delta_ss = (L + K_us vx^2) kappa and x_ss = [0, 0, -beta_ss, 0]. It is
an equilibrium of the continuous model for the constant heading rate of the path, so
it is also an equilibrium of the one-step model. With x~ = x - x_ss and
u~ = delta - delta_ss the path term cancels and

    x~(k + 1) = Ad x~(k) + Bu u~(k)

The residual A x_ss + B delta_ss + Bd kappa vx is about 1e-16 at (10 m/s, 0.02),
(20, 0.01), (5, 0.05) and (30, -0.004). At R = 50 m and 10 m/s, delta_ss = 0.0615 rad
and x_ss = [0, 0, -0.016667, 0].

## 3. The optimisation

Minimise over u~(0) .. u~(N-1)

    sum for k = 0 .. N-1 of ( x~(k)^T Q x~(k) + R u~(k)^2 )  +  x~(N)^T P x~(N)

subject to the model and the limits of section 5. Stack X = [x~(1); ...; x~(N)]. Then

    X = Sx x~(0) + Su U      Sx = [Ad; Ad^2; ...; Ad^N]
                             Su has the block Ad^(i-j) Bu in row i, column j (j <= i)

For N = 20 the shapes are Sx 80 x 4 and Su 80 x 20. The first column of Su starts
with Bu = [0.05826, 2.19733, 0.04040, 1.48086], then Ad Bu = [0.15326, 1.68964,
0.09704, 0.83820]. Check of the first entry:

    1 * 0.05826 + 0.03775 * 2.19733 + 0.12252 * 0.04040 + 0.00479 * 1.48086
      = 0.15325   (0.15326 from unrounded entries)

Substituting X, and keeping half the cost (the minimiser does not change), gives the
quadratic program OSQP solves:

    minimise  (1/2) U^T H U + f^T U      H = Su^T Qbar Su + Rbar
                                         f = Su^T Qbar Sx x~(0)

Qbar is block diagonal with Q for x~(1) .. x~(N-1) and P for x~(N). H depends on the
speed only, so it is built when the speed changes by more than 0.05 m/s. f depends on
the state and is updated every step with the bounds.

## 4. Terminal cost and the link to discrete LQR

P solves the discrete Riccati equation for (Ad, Bu, Q, R), and

    K_d = (R + Bu^T P Bu)^-1 Bu^T P Ad

By Bellman's principle x~^T P x~ is the optimal cost to go of the infinite-horizon
problem. So with no constraint active the N-step problem has the same optimum for any
N, and the first move is u~ = -K_d x~. The test compares the MPC command with
delta_ss - K_d (x~) at N = 3 and N = 20: the difference is 1e-15 and 1.2e-9. The
second is the solver tolerance.

Numbers at 10 m/s, dt = 0.05 s:

    K_d = [0.23641, 0.06188, 1.53959, 0.21505]      P[1,1] = 73.038
    x~ = [0.5, 0, 0.05, 0]:   x~^T P x~ = 24.448

The discrete cost is a sum over steps, so 24.448 * dt = 1.222 should be near the
continuous cost to go. It is 4% above the 1.1755 of lqr_math_reference.md section 3.

The discrete gains are not the continuous ones. This was seen in the output, not
expected, and is not derived:

    dt (s)    K1       K3
    0.1       0.1649   1.1576
    0.05      0.2364   1.5396
    0.02      0.3190   2.0139
    0.01      0.3567   2.2360
    0.002     0.3909   2.4388
    0.0005    0.3977   2.4792
    continuous 0.4000  2.4929

The gap closes roughly in proportion to dt. The closed loops are closer than the gains.
The discrete closed loop poles at dt = 0.05 s, written as ln(lambda) / dt, are -1.544,
-3.572, -14.38 and -27.14 per second. The continuous LQR poles at 10 m/s are -1.544,
-3.573, -14.32 and -29.16. The three slow ones agree within 0.5%; the fast one differs
by 7%.

## 5. Constraints

The variables are u~(k) = delta(k) - delta_ss. With steering limit d_max = 0.6 rad,
rate limit 0.8 rad/s and rate step r = 0.8 * dt = 0.04 rad:

    -d_max - delta_ss <= u~(k) <= d_max - delta_ss               k = 0 .. N-1
    -r <= u~(k) - u~(k-1) <= r                                   k = 1 .. N-1
    |u~(0) - (delta_now - delta_ss)| <= r

where delta_now is the steering angle of the vehicle. The first row of the constraint
matrix combines the limit and the rate bound for u~(0). The matrix is [I; D], with D
holding -1, +1 pairs, so it has N + (N - 1) = 39 rows for N = 20 and never changes.
The bounds are never empty, because delta_now is inside the steering limits.

Hand examples (straight path, 10 m/s, delta_ss = 0, so u~ = delta):

    error 2 m left,  delta_now = 0      command -0.04    (0.8 * 0.05: the rate bound)
    error 2 m left,  delta_now = 0.5    command  0.46    (0.5 - 0.04)
    error 5 m left,  delta_now = -0.59  command -0.55    (-0.59 + 0.04)

In the third case the plan unwinds the steering by one rate step per step (-0.55,
-0.51, -0.47, ...) although the error is large. That was seen in the output and not
analysed. The tests check the three commands.

## 6. Results (placeholder parameters)

Straight path, 10 m/s, start 0.2 m left, dt = 0.05 s, rate limit removed. The
prediction is x(k) = (Ad - Bu K_d)^k x(0). The largest gap over 6 s is 1.3e-5 m.

Circle, car on the path at speed, last 4 s of 25 s:

    R (m)   vx (m/s)   MPC steady error   LQR with feedforward (LQR note, section 9)
    50      10         -1.1e-4 m          -6.3e-5 m
    100     20         -2.2e-4 m          -1.4e-4 m

Recovery from an offset on a straight path at 10 m/s, with the default limits
(0.6 rad, 0.8 rad/s) in the plant, dt = 0.05 s, 12 s (scripts/mpc_vs_lqr.py).
Settling means staying within 0.05 m.

    offset (m)   controller   settle (s)    overshoot (m)   max steer (rad)
    0.5          LQR          1.85          0.000           0.084
    0.5          MPC          1.90          0.000           0.080
    2.0          LQR          2.55          0.000           0.240
    2.0          MPC          2.90          0.000           0.200
    5.0          LQR          8.15          1.746           0.518
    5.0          MPC          3.65          0.000           0.311
    8.0          LQR          not settled   5.107           0.600
    8.0          MPC          4.10          0.000           0.375

Solve time of one update and solve at N = 20: mean 0.03 to 0.05 ms, largest 0.73 ms
(Legion Tower, one run). The largest value changes from run to run.

## 7. Quantities computed only in code

- MPCController.solve_times: seconds for one QP update (new f and bounds) plus one
  solve, per command. It excludes the projection and the rebuild of H.
- MPCController.failures: solves whose OSQP status was not "solved". If the result is
  not finite the previous command is repeated.
- scripts/mpc_vs_lqr.py. Settle: the time of the last sample outside 0.05 m, plus
  dt; "not settled" if the last sample is outside. Overshoot: max(0, -min error) for
  a start on the left. Max steer: the largest |delta| over the run.

## 8. Limits of these findings

- The plant is the same model family as the design. Offsets of 5 and 8 m are far
  outside the small-error range of the linear model. The LQR result says what an
  unconstrained gain does with a rate-limited steering actuator. It does not say the
  LQR is poor in its design range: at 0.5 and 2 m it settles faster than the MPC.
- The MPC does not model the steering actuator. It sees the limits, not the lag.
- Curvature and speed are frozen over the horizon. There is no preview.
- N = 20 and dt = 0.05 s are not tuned. In one sandbox run (not saved in the repo),
  dt = 0.02 s with N = 50 changed the settle times at 5, 8 and 12 m offsets by
  0.01 s or less.
- The weights are the LQR placeholders. The discrete gains differ from the continuous
  ones (section 4), so the MPC is gentler than the LQR for the same weights.
- Solve times come from one desktop and are not a statement about embedded hardware.
