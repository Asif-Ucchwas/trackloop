# Linear closed-loop models (Task 17)

How each controller becomes a linear system on a straight path, with a worked example at 10 m/s,
and what the models were checked against. Code: `src/trackloop/linear_loops.py`. Check:
`python scripts/validate_linear_loops.py` (figure `docs/figures/linear_loops.png`).

## Setup

- Error state x = [e1, e1_dot, e2, e2_dot]: lateral error of the centre of gravity (m), its rate,
  heading error (vehicle minus path, rad) and its rate. Plant on a straight path, constant speed
  v, from `lqr.error_model`: x_dot = A(v) x + B delta, with delta the front steering angle.
- Every controller is a fixed state feedback on a straight path, delta_cmd = -K x. The rows of K
  below follow from each controller's code with small angles (atan(x) = x, sin(x) = x).
- Steering actuator: first-order lag, delta_dot = (delta_cmd - delta) / T. In the simulation
  step this is delta+ = a delta + (1 - a) delta_cmd with a = exp(-dt / T), and the plant holds
  delta+ over the step. The sampled model in `closed_loop(..., dt)` is exactly that structure:
  with Ad, Bu the zero-order-hold discretisation of (A, B), the state z = [x, delta] evolves as
  x+ = Ad x + Bu delta+, delta+ = a delta - (1 - a) K x. Without lag z = x and x+ = (Ad - Bu K) x.
- Not in a linear model: the steering angle limit (0.6 rad), the rate limit (0.8 rad/s), path
  curvature (the feedforward term is zero on a straight path), measurement noise.

## Gains

- **Pure Pursuit** (`pure_pursuit.py`): goal point ld = clip(2 + 0.5 v, 2, 30) m ahead of the rear
  axle; delta = atan(2 L sin(alpha) / ld). On a straight path alpha = -e_r / ld - e2 with e_r
  the rear-axle offset, e_r = e1 - lr e2. So delta = -(2L/ld^2) e1 - (2L/ld)(1 - lr/ld) e2:
  K = [2L/ld^2, 0, (2L/ld)(1 - lr/ld), 0].
- **Stanley** (`stanley.py`): delta = psi_e - atan(k e_front / (v + k_soft)), psi_e = -e2, and the
  front-axle offset e_front = e_r + L e2 = e1 + lf e2 (because L - lr = lf). With s = k / (v + k_soft):
  K = [s, 0, 1 + s lf, 0].
- **Dynamic LQR** (`lqr.py`): K from the controller's gain schedule: Riccati gains on a speed grid (2 to 30 m/s),
  interpolated, held constant above 30 m/s. On the grid, e.g. at 10 m/s, it equals the Riccati gain. Above 30 m/s
  the exact Riccati gain is not what the controller uses (found in Task 19; Tasks 17 and 18 first used it there).
- **Dynamic MPC** (`mpc.py`): the terminal weight is the discrete Riccati solution, so with no
  constraint active the first move of the finite-horizon problem equals the infinite-horizon
  discrete LQR gain at the control period dt = 0.05 s. K is `discrete_lqr(Ad, Bu, Q, R)`.
  This is an equivalence I derived from the cost; the simulation check below is the evidence.

## Worked example, v = 10 m/s (L = 2.7, lf = 1.2, lr = 1.5)

| controller | K = [e1, e1_dot, e2, e2_dot] |
|---|---|
| Pure Pursuit (ld = 7 m) | 0.1102, 0, 0.6061, 0 |
| Stanley (s = 1.5 / 11 = 0.1364) | 0.1364, 0, 1.1636, 0 |
| Dynamic LQR | 0.4000, 0.1034, 2.4929, 0.4552 |
| Dynamic MPC | 0.2364, 0.0619, 1.5396, 0.2150 |

Slowest closed-loop pole of the sampled model, as a decay rate (1/s), without lag and with
T = 0.1 s: Pure Pursuit 1.18 and 1.09, Stanley 1.59 and 1.47, Dynamic LQR 1.53 and 1.48, Dynamic MPC
1.54 and 1.46. All four are stable at 5, 10 and 20 m/s with no lag (tests cover 10 m/s with lags
0, 0.1 and 0.2 s).

## Checks

- Gains for Pure Pursuit and Stanley match the hand calculation above (unit tests); the LQR gain
  equals `lqr_gain`.
- The sampled model against the nonlinear simulation on a straight path, starting offset by e0
  with zero heading error, over 4 s, for 4 controllers x 3 speeds (5, 10, 20 m/s) x 3 lags (0,
  0.1, 0.2 s): the largest difference in lateral error, as a share of the offset, in the worst
  of the 9 speed and lag combinations (`data/results/linear_loops_validation.csv`):

| offset | Pure Pursuit | Stanley | Dynamic LQR | Dynamic MPC |
|---|---|---|---|---|
| 0.05 m | 0.0% | 0.0% | 0.0% | 0.0% |
| 0.25 m | 1.8% | 1.4% | 2.2% | 2.6% |
| 0.5 m | 5.2% | 3.9% | 4.7% | 9.6% |
| 1 m | 11.6% | 8.6% | 7.8% | 25.2% |
| 2 m | 21.6% | 15.8% | 28.4% | 161.1% |

## What this does and does not show

- At 0.05 m the models reproduce the simulation to about 2e-5 m. That checks the gain formulas
  and the lag discretisation. It does not check them against a real vehicle, and the plant here
  is the same model the error equations were derived from.
- The difference grows with the offset size. Almost all of it is the 0.8 rad/s steering rate limit, which the
  linear model leaves out, and for the MPC its own QP rate constraint as well: with those removed the gap at 2 m
  falls from 16 to 161 percent to 0.4 to 2.6 percent (Task 20, part 6). Task 19 compares the linear
  predictions with the nonlinear simulation where it matters.
- Pure Pursuit and Stanley are linearised with small angles and the controller's nearest-point
  lookup is treated as continuous. The path-spacing effect found in the Task 16 notes
  (`root_cause_note.md`, section 2) is not in the linear model.
- Only straight-line driving at constant speed. Curves, speed changes and the gain schedule
  between grid speeds are not modelled.

## Margins (Task 18)

Code: `src/trackloop/margins.py` (`open_loop` in `linear_loops.py`). Figures: `docs/figures/bode_10ms.png`,
`docs/figures/margins_vs_speed.png`. Data: `data/results/margins_vs_speed.csv` (speeds 2 to 70 m/s,
lags 0, 0.1, 0.2 s). Script: `scripts/plot_margins.py`.

- **Where the loop is broken.** At the steering command: input delta_cmd, output K x, so the loop
  is L(z) = K (zI - A)^-1 B for the sampled model with the actuator lag included in A. A margin
  here is the margin to a gain or phase change at the steering command. Breaking the loop
  elsewhere (at the measured states, say) would give different numbers. This was my choice.
- **Gain margin** is found by scaling the loop gain and checking the closed-loop poles, so it needs
  no crossover definition. The upward margin is the factor the gain can grow by. Downward: the
  loop stays stable down to a gain factor of 0.001 (the lowest tried) wherever the phase margin
  is more than about 1 degree; very close to zero phase margin a lower limit appears (Dynamic LQR,
  0.2 s lag, 52 m/s: 0.67). The phase is within about 2 degrees of -180 at 0.05 rad/s (the plant
  behaves like a double integrator at low frequency, loop slope about -40 dB/decade), so the
  downward margin is not comfortable in practice even where it is formally unlimited.
- **Phase margin** is read from the response at the unity-gain crossings, up to the Nyquist
  frequency (62.8 rad/s); the smallest is reported. Delay margin = phase margin / crossover frequency.
- **Results.** Phase margin falls with speed for every controller and every lag. At 10 m/s with a 0.1 s
  lag: Pure Pursuit 33.0, Stanley 36.8, Dynamic LQR 54.0, Dynamic MPC 60.5 degrees. At 40 m/s with
  a 0.2 s lag: 15.2, 12.6, 7.1 and 9.1 degrees. On the 1 m/s grid the first speed with a phase margin
  at or below zero is 61 m/s (Stanley, 0.1 s lag) and 56 and 64 m/s (Stanley, MPC, 0.2 s lag);
  none for Pure Pursuit or LQR up to 70 m/s, none with no lag. Task 19 refines these and tests them in
  the nonlinear simulation. (The first version of these figures used the exact LQR gain above 30 m/s
  and gave 5.0 degrees and a zero at 54 m/s for LQR; corrected in Task 19.) The benchmark runs at most 20 m/s, so most of that range is
  extrapolation of the linear tire model.
- **Dynamic LQR with no lag** has the smallest upward gain margin (1.33 at 40 m/s, about 2.5 dB) because
  its crossover (34 rad/s) is close to Nyquist. A lag lowers the loop gain there and raises the
  margin (5.4 with 0.1 s) while lowering the phase margin. That the sampling delay is the cause is my
  reading of the crossover frequencies, not a separate test.
- Pure Pursuit curves bend at 56 m/s where the lookahead reaches its 30 m clip; the LQR curves flatten above
  30 m/s where its gain schedule ends.
- Not in the margins: the steering and rate limits, and for the MPC everything that makes it differ
  from its unconstrained first move.

## Stability limits (Task 19)

Code: `src/trackloop/stability_limit.py`. Script: `scripts/check_stability_limits.py`. Data:
`data/results/stability_limits.csv`. Figure: `docs/figures/stability_limit_check.png`.

- **Definition.** The stability limit of a controller at a steering lag is the lowest speed where the sampled
  linear loop of Task 17 (lag state included) has a pole outside the unit circle: found on a 1 m/s grid up to
  300 m/s, then bisected to 0.01 m/s. It is the same event as the phase margin reaching zero.

| | no lag | lag 0.1 s | lag 0.2 s |
|---|---|---|---|
| Pure Pursuit | none to 300 m/s | 268.5 m/s | 134.2 m/s |
| Stanley | 166.7 m/s | 60.7 m/s | 55.0 m/s |
| Dynamic LQR | none | none | 92.1 m/s |
| Dynamic MPC | none | none | 63.2 m/s |

- **Correction to Task 18.** My first nonlinear runs at the predicted LQR limit (54 m/s) were stable. The LQR
  controller's gain schedule ends at 30 m/s and holds that gain above it, but my model used the exact Riccati gain
  at each speed. `feedback_gain` now uses the schedule. LQR margins above 30 m/s changed (phase margin at 40 m/s with
  0.2 s lag: 5.0 to 7.1 degrees; limit: 54 to 92.1 m/s). Everything up to 30 m/s is unchanged, including the Task 17
  validation (5, 10, 20 m/s). Pure Pursuit, Stanley and MPC were not affected: their gains are computed at the
  actual speed.
- **Check in the nonlinear simulation.** From a 1 mm offset on a straight path, 120 s, at 0.9, 0.97, 1.03 and 1.1
  times each limit (7 cases with a limit), and at 100 and 300 m/s for the 5 cases without one (60 s). The error
  grows or decays as the linear model says in 38 of 38 runs, so the instability appears at the predicted speed to
  within 3 percent. The simulated and linear trajectories differ by at most 1e-4 of their size up to 0.97 times the
  limit, at most 2.4e-2 at 1.03 times, and at most 1.8e-5 for the cases without a limit.
- **Beyond the limit** the simulation leaves the linear model. At 1.1 times the limit the LQR and MPC runs grow much
  faster than predicted (natural log of late over early amplitude 15 against 6 for LQR, 14.5 against 12.5 for MPC), because at about
  1e-2 m something else takes over. Removing the 0.8 rad/s steering rate limit delays that for LQR (110 s to 128 s)
  without removing it, and for MPC neither limit matters. I did not find the cause.
- **Large offset (5 cm), 120 s, nonlinear simulation only.** At 1.1 and 1.3 times the limit Pure Pursuit stays within
  0.1 to 0.7 m, Stanley with lag within 1.4 to 3.4 m, Stanley without lag reaches 0.05 and 677 m, LQR and MPC reach 200 to 325 m.
  Bounded here means bounded over 120 s, not shown to stay bounded.
- **A measurement I got wrong first.** I first estimated growth rates from a running-maximum envelope. For Stanley
  without lag at 160 m/s it said -0.07 per second against -0.02 predicted, which looked like a model error. The
  trajectories agree to 1e-5 (the check above); the envelope slope is biased by beating between two lightly damped
  oscillations. I replaced it with comparing late and early amplitude of the two series.
- **Numbers that differ between machines.** Amplitude ratios below about 1e-6 are decays down to the round-off floor
  (late amplitude near 1e-16 m), and they differ in their leading digits between two machines (my sandbox and the
  desktop differed in 11 of 52 rows of `stability_limits.csv`, all of this kind). So do the final errors of runs that
  diverge by hundreds of metres. For these only growth or decay, and the trajectory gap, mean anything. The limits, the
  38 of 38 agreement and every other figure quoted here are the same on both machines to the digits given. The points
  at the far left of the figure's left panel are such round-off-floor cases.
- **What this does not show.** Both sides use the same plant: linear tires, no drag, constant speed, no tire
  saturation. The agreement checks the algebra and the discretisation, not a real vehicle. The limits of 55 to 270 m/s
  are properties of this model, far outside the 20 m/s benchmark (200 km/h is 56 m/s), not predictions for a car.

## Derivations, and where linearisation stops being valid (Task 20)

Code: `src/trackloop/linearization.py`. Script: `scripts/linearization_limits.py`. Data:
`data/results/linearization_limits.csv`. Figure: `docs/figures/linearization_limits.png`. Each part below gives the
formula, how it follows, and a worked example at 10 m/s (L = 2.7 m, lf = 1.2, lr = 1.5, m = 1500 kg, Iz = 2500,
cf = 80000, cr = 100000 N/rad).

### 1. The error model

**Formula.** State x = [e1, e1_dot, e2, e2_dot], input the front steering angle delta, constant speed v, straight path:
x_dot = A x + B delta with

    A = [[0, 1, 0, 0],
         [0, -(cf+cr)/(m v), (cf+cr)/m, (-cf lf + cr lr)/(m v)],
         [0, 0, 0, 1],
         [0, (-lf cf + lr cr)/(Iz v), (lf cf - lr cr)/Iz, -(lf^2 cf + lr^2 cr)/(Iz v)]]
    B = [0, cf/m, 0, lf cf/Iz]^T

**Derivation.** Linear tires and small angles give the front and rear lateral forces
Fyf = cf (delta - (vy + lf r)/v) and Fyr = cr (-(vy - lr r)/v). The vehicle equations are
m (vy_dot + v r) = Fyf + Fyr and Iz r_dot = lf Fyf - lr Fyr. On a straight path e2_dot = r and
e1_dot = vy + v e2, so vy = e1_dot - v e2 and e1_ddot = vy_dot + v r = (Fyf + Fyr)/m. Substituting vy:

    Fyf + Fyr = cf delta - (cf+cr)(e1_dot - v e2)/v + (-cf lf + cr lr) e2_dot / v
    Iz e2_ddot = lf cf delta + (-lf cf + lr cr)(e1_dot - v e2)/v - (lf^2 cf + lr^2 cr) e2_dot / v

Dividing by m and Iz gives rows 2 and 4 of A and B; rows 1 and 3 are the definitions of the rates.

**Worked example, v = 10 m/s.** A[1,1] = -180000/(1500 x 10) = -12; A[1,2] = 180000/1500 = 120;
A[1,3] = (-80000 x 1.2 + 100000 x 1.5)/15000 = 3.6; A[3,1] = 54000/25000 = 2.16; A[3,2] = -54000/2500 = -21.6;
A[3,3] = -(1.44 x 80000 + 2.25 x 100000)/25000 = -13.608; B = [0, 53.33, 0, 38.4]. The two integrators leave
A with eigenvalues 0, 0 and -12.80 +/- 3.63j (checked numerically against `error_model`).

### 2. Sampling and the steering lag

**Formula.** With the input held over a step dt (zero-order hold), exp([[A dt, B dt], [0, 0]]) = [[Ad, Bu], [0, I]].
A first-order actuator with time constant T, delta_dot = (u - delta)/T, over one step gives
delta+ = a delta + (1 - a) u with a = exp(-dt/T). The simulation holds delta+ over the step, so the closed loop is
x+ = Ad x + Bu delta+ with u = -K x (the matrices of `closed_loop`).

**Derivation.** The first is the standard solution of x_dot = A x + B u for constant u; the second is the solution
of the scalar lag equation for constant u. Both match `models/dynamic.py` (RK4 for the plant, exact lag step).

**Worked example.** At v = 10 m/s, dt = 0.05 s: Ad[1,1] = 0.5574, Ad[1,2] = 4.4261, Bu = [0.0583, 2.1973, 0.0404, 1.4809].
For T = 0.1 s, a = exp(-0.5) = 0.6065.

### 3. LQR and the MPC

**Formula.** Minimise the integral of x^T Q x + u^T R u. The gain is K = R^-1 B^T P with P the solution of
A^T P + P A - P B R^-1 B^T P + Q = 0. Weights as in the code (Bryson's rule): Q = diag(1/xmax^2) with
xmax = (0.5, 1.0, 0.1, 0.3), R = 1/0.2^2 = 25, so Q = diag(4, 1, 100, 11.1).

**Worked example, v = 10 m/s.** K = [0.4000, 0.1034, 2.4929, 0.4552]. The continuous closed-loop poles of A - B K are
-29.16, -14.32, -3.57 and -1.54 (decay rates, 1/s). The first gain, 0.4000, equals sqrt(Q11/R) = sqrt(4/25) at each of
2, 5, 10, 20 and 30 m/s that I checked; I have not derived why.

**MPC.** The terminal cost is the discrete Riccati solution, so while no constraint is active the first move of the
finite-horizon problem equals the infinite-horizon discrete LQR gain at dt (derived from the cost; the simulation
agreement of the tables above is the evidence). It is computed from (Ad, Bu) with `discrete_lqr`.

### 4. Pure Pursuit and Stanley

The formulas and the 10 m/s worked example are in "Gains" above. The step from the code to the gains is the small-angle
replacement atan(x) = x, sin(x) = x, which is what stops being exact at large errors (part 6).

### 5. Margins and the stability limit

**Formulas.** Broken at the steering command, the loop is L(z) = K (zI - A_o)^-1 B_o for the sampled model with the lag
state, so that the closed-loop poles are the roots of 1 + L(z) = 0. Phase margin: PM = 180 deg + arg L(e^{j wc dt}) at the unity-gain
frequency wc; delay margin = PM (rad) / wc; upward gain margin: the factor on L at which a closed-loop pole reaches the
unit circle. The stability limit is the lowest speed at which the largest pole magnitude reaches 1.

**Worked example, v = 10 m/s, T = 0.1 s, Pure Pursuit.** wc = 2.537 rad/s, PM = 32.99 deg = 0.5758 rad, so
the delay margin is 0.5758/2.537 = 0.227 s (4.5 samples; the added-delay test in `tests/test_margins.py` confirms
that 3 extra samples are stable and 6 are not), and the upward gain margin is 6.97. For Stanley at T = 0.1 s the largest
pole magnitude is 0.9972 at 57.7 m/s and 1.0026 at 63.8 m/s, and the bisected limit is 60.7 m/s.

### 6. Where linearisation stops being valid

Measured: the lateral error of the simulation against the sampled linear model, from an offset on a straight path
(`scripts/linearization_limits.py`), worst case over 3 speeds (5, 10, 20 m/s) and 3 steering lags (0, 0.1, 0.2 s), as
a share of the offset. "Rate limit removed" takes the 0.8 rad/s steering rate limit out of the plant; for the MPC the
second column removes its own constraints from the QP and the third removes both.

| offset | Pure Pursuit | | Stanley | | Dynamic LQR | | Dynamic MPC | | | |
|---|---|---|---|---|---|---|---|---|---|---|
| | default | rate limit removed | default | rate limit removed | default | rate limit removed | default | plant removed | MPC constraints removed | both removed |
| 0.25 m | 1.8% | 0.0% | 1.4% | 0.1% | 2.2% | 0.0% | 2.6% | 2.6% | 1.3% | 0.0% |
| 1 m | 11.6% | 0.6% | 8.6% | 0.7% | 7.8% | 0.4% | 25.2% | 25.2% | 6.5% | 0.1% |
| 2 m | 21.6% | 2.2% | 15.8% | 2.6% | 28.4% | 1.9% | 161.1% | 161.1% | 11.8% | 0.4% |

- **Steering rate limit (0.8 rad/s): the main cause.** This answers the question left open in the Task 17 section. For
  Pure Pursuit, Stanley and LQR the gap at 2 m falls from 16 to 28 percent to 2 to 3 percent when the plant rate limit is
  removed. At a 2 m offset the controllers ask for 9 to 12 rad/s (Pure Pursuit 9.8, Stanley 9.3, LQR 12.0, MPC 11.1 in the
  runs with the limit removed). A steering lag lowers the demanded rate (for Stanley the worst cases are at lag 0).
  For the MPC the plant limit alone changes nothing (the MPC's own rate constraint, also 0.8 rad/s, binds first); removing the MPC's constraints cuts 161 to 11.8 percent, and removing both gives 0.4 percent.
- **Steering angle limit (0.6 rad): not a cause here.** The peak steering angle at 2 m is 0.46 to 0.55 rad for Pure
  Pursuit, Stanley and MPC and just reaches 0.60 for LQR, whose own output clip stays on in these runs; its remaining
  gap is 1.9 percent. It would matter at larger offsets or higher gains.
- **Large errors (the small-angle terms).** With the rate limit removed the remaining gap is 0.4 to 2.6 percent at 2 m: the
  atan in the slip angles and in Stanley, the sin in Pure Pursuit, sin and cos of the heading error. It is small but
  grows roughly with the square of the offset (0.1 to 0.7 percent at 1 m, 2 to 3 percent at 2 m), as small-angle errors do.
- **Tire nonlinearity: not tested, and the simulation cannot test it.** The simulation's tires are linear in the slip
  angle (with an atan in the slip angle but no force saturation), so the linear models and the simulation share the plant
  up to those trigonometric terms. Real tires saturate near a slip angle of mu Fz/C; the Task 16 note
  (`root_cause_note.md`, section 5) counts how many benchmark runs exceed it (only dynamic LQR and MPC at 20 m/s, and
  two Stanley runs). Past it the linear tire model overstates the available force, so the stability limits and
  margins here are optimistic about large manoeuvres.
- **Constraints and limits of the controllers themselves.** The MPC's QP constraints are the reason for its 161 percent;
  the unconstrained first move that the linear model uses is not what it does at large offsets. The linear models also
  leave out the path-spacing artefact of the nearest-point lookup (Task 16), noise, curvature, speed changes and
  longitudinal dynamics. (The LQR model uses its gain schedule, held constant above 30 m/s, since Task 19.)
- **Past the stability limit** the Task 19 runs left the linear model at about 1e-2 m. The rate limit delays that for LQR
  but does not remove it, and does nothing for MPC, so it is not the whole story there; that stays open.

Summary: the linear models are good to a few percent for offsets up to about 0.25 m in all four controllers, and
for larger offsets they ignore the steering rate limit, which is the first nonlinearity to bite. They are not a model
of tire saturation or of anything beyond straight-line driving at constant speed.

## System identification data (Task 21)

Code: `src/trackloop/sysid_data.py`. Script: `scripts/make_sysid_data.py`. Data: `data/sysid/sweep_v15.csv`,
`prbs_v15.csv` and their `.json` files (settings and the true parameters). Figure: `docs/figures/sysid_data.png`.

**This is simulated data.** The vehicle is the dynamic model of `models/dynamic.py` with the nominal parameters
(m = 1500 kg, Iz = 2500 kg m^2, lf = 1.2 m, lr = 1.5 m, cf = 80000, cr = 100000 N/rad). Nothing here supports a
claim about identifying a real vehicle; that would need data from a real car.

### What the estimator is given and what it is not

- **Given** (columns of the CSV): time `t`, the steering angle `delta` in force over each 0.01 s step (the angle after
  the plant's rate and angle limits, held constant over the step), the measured yaw rate `r_meas` and the measured
  lateral acceleration `ay_meas`. The speed is constant and known (15 m/s). The steering angle is treated as measured
  exactly: this is an assumption that makes the problem easier than a real one, where the steering sensor also has
  noise and an offset. Task 23 can relax it.
- **Not given:** the lateral velocity (`vy_true` is in the file only to check results) and the true yaw rate and
  acceleration (`r_true`, `ay_true`, there to measure the noise and the fit).
- **Lateral acceleration** is the body-frame accelerometer reading, ay = vy_dot + vx r = (Fyf cos(delta) + Fyr)/m.
- **Noise:** independent Gaussian, standard deviation 0.003 rad/s on yaw rate and 0.05 m/s^2 on lateral acceleration
  (about 0.17 deg/s and 0.005 g). These are my choices for a plausible low-cost sensor, not figures from a datasheet.
  The noise has its own seeded random stream, so a dataset's noise is the same for either excitation signal.

### Excitation

| signal | definition | peak steering | RMS yaw rate | RMS ay | SNR (yaw rate, ay) | steps limited |
|---|---|---|---|---|---|---|
| sweep | linear chirp 0.1 to 3 Hz over 40 s, amplitude 0.03 rad | 0.030 rad | 0.070 rad/s | 0.92 m/s^2 | 23, 18 | 0 % |
| prbs | random plus or minus 0.03 rad, each level held 0.25 s | 0.030 rad | 0.102 rad/s | 1.34 m/s^2 | 34, 27 | 14.8 % |

SNR is the RMS of the true signal over the noise standard deviation, over the whole record. At 15 m/s the lateral
dynamics have a pair of poles at -8.54 +/- 4.23j rad/s (1.5 Hz natural frequency), inside the sweep band and in the
region where the random signal has most of its power (spectrum panel of the figure). The amplitude 0.03 rad keeps
the sweep's steering rate (peak 0.56 rad/s) below the 0.8 rad/s plant limit. The random signal switches by 0.06 rad,
which needs 0.075 s at the limit, so 14.8 percent of its steps are rate limited: the commanded and applied angles
differ at the edges. Because the data file holds the applied angle, this does not matter for identification, but the
command column should not be used as the input.

### What the data can and cannot tell: only three parameter combinations

**Formula.** With the geometry lf, lr known, the measured signals depend on the parameters only through
cf/m, cr/m and Iz/m. In particular, multiplying m, Iz, cf and cr by the same factor k leaves every signal unchanged.

**Derivation.** The model is vy_dot = (cf alpha_f cos(delta) + cr alpha_r)/m - vx r,
r_dot = (lf cf alpha_f cos(delta) - lr cr alpha_r)/Iz and ay = (cf alpha_f cos(delta) + cr alpha_r)/m, with the slip
angles alpha_f and alpha_r functions of velocities, yaw rate, delta and geometry only. The parameters appear only as
A = cf/m, B = cr/m, C = lf cf/Iz and D = lr cr/Iz. These four are not independent: C/A = lf m/Iz and D/B = lr m/Iz,
so (C/A)/(D/B) = lf/lr, which is known. Three numbers remain: cf/m, cr/m and Iz/m. The four unknowns m, Iz, cf, cr
therefore cannot all be recovered from these measurements, whatever the noise level or the excitation: one direction
in parameter space has no effect on the data.

**Worked example, 15 m/s.** cf/m = 53.33 and cr/m = 66.67 (N per rad per kg, that is m/s^2 per rad) and Iz/m = 1.667 m^2. Scaling
all four by k = 1.7 (m = 2550, Iz = 4250, cf = 136000, cr = 170000) gives the same three numbers and, in
`tests/test_sysid_data.py`, the same yaw rate, lateral acceleration and lateral velocity to a relative difference
of 1e-9.

**Consequence for Task 22.** The syllabus lists cornering stiffnesses, mass and yaw inertia. Mass cannot be estimated
separately from these signals. Task 22 will estimate cf, cr and Iz with the mass fixed at its known value (a
vehicle's mass can be weighed), and will report that choice. Whether the three combinations are well determined at
the noise level used here, and how correlated their estimates are, is for Tasks 22 and 23 to measure; the argument
above shows only that three is the most that can be determined.

### Reproducing

`python scripts/make_sysid_data.py` regenerates all four data files and the figure from the seed (21). The files are
rounded to six decimals, so a last digit could differ between machines: I check the line counts (4001 for each CSV)
rather than checksums.

## Least-squares identification (Task 22)

Code: `src/trackloop/identification.py`. Script: `scripts/run_identification.py`. Data: `data/results/identification.csv`.
Figure: `docs/figures/identification.png`. Input: the two simulated datasets of Task 21 (sweep and random steering, 15 m/s,
40 s, 100 Hz). **Simulated data throughout; nothing here is a claim about a real vehicle.**

### Method

**Model.** The estimator's model is the sampled linear lateral model (state: lateral velocity and yaw rate) with the steering
angle held over each step, the lateral part of the error model of Task 20 (Part 1) with the same exact discretisation
(Part 2):
x+ = Ad x + Bd delta, r = x2, ay = a1 x1 + a2 x2 + (cf/m) delta, with a1 = -(cf+cr)/(m v) and
a2 = (-lf cf + lr cr)/(m v). The data come from the nonlinear simulation, whose differences from this model (the atan in
the slip angles, cos(delta) in the front force) are 0.03 percent of the signal peak at these amplitudes
(`tests/test_identification.py`). So the model is almost exactly the model that made the data: this is the best case for the
estimator, not a test of how it copes with a model that is wrong.

**Estimate.** theta_hat minimises the sum over both outputs and all 4000 samples of the squared prediction error divided by
the noise standard deviation (0.003 rad/s for yaw rate, 0.05 m/s^2 for lateral acceleration). The parameters are
theta = (cf, cr, Iz); the mass is fixed at its true value because it cannot be identified (Task 21). The optimiser is
`scipy.optimize.least_squares` (trust region, bounds 0.1 to 10 times the true values to keep every trial model stable),
started from 0.6 cf, 1.5 cr and 0.7 Iz.

**Covariance.** With whitened residuals e(theta) and Jacobian J = de/dtheta at theta_hat, the covariance of the estimate is
(J^T J)^-1. This is the standard linearised result for Gaussian noise of known standard deviation; it is also the
Cramer-Rao bound for this model, so no unbiased estimator could do better. It is a local linear approximation, and I check it
against the real spread of the estimates over 100 noise seeds below instead of assuming it.

**Worked example (sweep dataset).** estimate cf = 79977.4 against a true value of 80000 (error -0.03 percent); the covariance
gives a standard deviation of 104.8 N/rad (0.13 percent), so the error is -0.22 standard deviations. The sum of the whitened
squared residuals per degree of freedom (chi-squared per degree of freedom) is 1.014; it would be 1 if the model and the
noise levels were exactly right.

### Results

| dataset | parameter | true | estimate | std (covariance) | error | spread over 100 seeds | mean predicted std | inside 95% interval |
|---|---|---|---|---|---|---|---|---|
| sweep | cf | 80000 | 79977 | 105 (0.13%) | -0.03% | 0.132% | 0.131% | 94% |
| sweep | cr | 100000 | 99901 | 214 (0.21%) | -0.10% | 0.199% | 0.214% | 95% |
| sweep | Iz | 2500 | 2499.7 | 4.5 (0.18%) | -0.01% | 0.157% | 0.180% | 97% |
| prbs | cf | 80000 | 79928 | 84 (0.11%) | -0.09% | 0.102% | 0.107% | 93% |
| prbs | cr | 100000 | 99912 | 158 (0.16%) | -0.09% | 0.147% | 0.160% | 95% |
| prbs | Iz | 2500 | 2495.9 | 3.8 (0.15%) | -0.16% | 0.152% | 0.155% | 94% |

- **The covariance is right.** Over 100 noise seeds, the real standard deviation of each estimate is within 13 percent of
  the predicted one (one is 1 percent larger, the other five 2 to 13 percent smaller), and the 95 percent interval from the covariance
  contains the true value in 93 to 97 percent of the runs. With 100 seeds the 95 percent coverage itself is uncertain by
  about plus or minus 2 percentage points.
- **A small bias exists.** The mean error over the seeds is -0.04 percent for cf (the other four are within 0.01 percent). The
  fit to noise-free data is off by -0.03 to -0.04 percent for cf, which is the model difference above: it is a third of the standard
  deviation and does not matter here, but it is not zero and would grow with a larger steering amplitude.
- **The estimates are strongly correlated.** The correlation coefficients are 0.68 (cf, cr), 0.78 (cf, Iz) and 0.75 (cr, Iz)
  for the sweep, and 0.79, 0.82 and 0.80 for the random signal (the figure's ellipses). A change in one stiffness is
  mostly compensated by changes in the others, so the three parameters are less separately identifiable than their individual
  standard deviations suggest. Task 23 studies what excitation and noise do to this.
- **The random signal gives smaller intervals** (0.11, 0.16, 0.15 percent against 0.13, 0.21, 0.18 percent) because it puts more
  power into the steering (RMS yaw rate 0.102 against 0.070 rad/s), and it is also the one with the strongest correlations.
  The data are different records of the same length, so this says what these two records gave, not that one signal type is
  always better.
- **Mass check.** With the mass added as a fourth parameter, the smallest singular value of the weighted Jacobian at the true
  parameters is 4e-11 (sweep) and 4e-11 (random) of the largest, a numerical zero; with three parameters it is 0.25 and
  0.23 of the largest. This confirms the Task 21 argument numerically.

### What this does not show

- The model that makes the data and the model that is fitted are almost the same, and the noise levels are known to the
  estimator (chi-squared per degree of freedom of 1.01 is a check on the code and the noise assumption, not a test of model
  error). A real vehicle has a model error, unknown noise, sensor bias and steering measured with error, and the intervals here would
  then be too small. The Stage 4 model-error cases and Task 23's noise study are where that starts to be tested.
- One speed (15 m/s) and one steering amplitude (0.03 rad), well inside the linear tire range.
- The nominal parameters are the true parameters in the fit (the starting point is 30 to 50 percent away from them, so it
  does not begin at the answer, but the bounds are centred on it).
