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
- **Dynamic LQR** (`lqr.py`): K from the Riccati solution at speed v, as the controller computes it.
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
- The difference grows with the offset size. I have not examined why (steering limits and the
  rate limit are the first candidates, not tested), nor why it reaches 161 percent for the MPC at 2 m.
  Task 19 compares the linear predictions with the nonlinear simulation where it matters.
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
  a 0.2 s lag: 15.2, 12.6, 5.0 and 9.1 degrees. On the 1 m/s grid the first speed with a phase margin
  at or below zero is 61 m/s (Stanley, 0.1 s lag) and 56, 54 and 64 m/s (Stanley, LQR, MPC, 0.2 s lag);
  none for Pure Pursuit up to 70 m/s, none with no lag. Task 19 refines these and tests them in
  the nonlinear simulation. The benchmark runs at most 20 m/s, so most of that range is
  extrapolation of the linear tire model.
- **Dynamic LQR with no lag** has the smallest upward gain margin (1.27 at 40 m/s, about 2 dB) because
  its crossover (36 rad/s) is close to Nyquist. A lag lowers the loop gain there and raises the
  margin (5.2 with 0.1 s) while lowering the phase margin. That the sampling delay is the cause is my
  reading of the crossover frequencies, not a separate test.
- Pure Pursuit curves bend at 56 m/s where the lookahead reaches its 30 m clip.
- Not in the margins: the steering and rate limits, and for the MPC everything that makes it differ
  from its unconstrained first move.
