# Road-family benchmark (Task 14)

What was run, what the numbers say, and what they do not say. Numbers are from
`data/results/benchmark_road.csv` (240 rows), produced at commit `a737cc6`.

## Setup

- 12 frozen road paths (4 shapes x design speeds 5, 10, 20 m/s), peak lateral acceleration
  2.0 m/s^2 (a placeholder), see `road_family_math_reference.md`.
- 4 controllers on the dynamic bicycle model (linear tires): Pure Pursuit, Stanley, Dynamic LQR,
  Dynamic MPC. Pure Pursuit and Stanley were written for the kinematic rear-axle state and run
  through an adapter (`RearAxleAdapter`); their command goes to the dynamic model unchanged.
  The kinematic LQR and MPC are not in this family.
- 5 seeds. A seed fixes a start offset (+-0.3 m) and heading error (+-0.1 rad) per path, the
  same for all four controllers. These are the parking-family ranges, reused unchanged.
- Speed is constant at the design speed (zero longitudinal acceleration). A run ends 0.5 m
  before the path end; 25 m off the path counts as diverged. Errors are measured at the rear
  axle with the Task 13 definitions.
- Result: 240 of 240 runs finished, 0 errors, 0 diverged, 0 timeouts.

## Results (median over 20 runs per cell)

| controller | 5 m/s RMS | 10 m/s RMS | 20 m/s RMS |
|---|---|---|---|
| Pure Pursuit | 0.085 | 0.109 | 0.251 |
| Stanley | 0.092 | 0.076 | 0.193 |
| Dynamic LQR | 0.066 | 0.046 | 0.034 |
| Dynamic MPC | 0.067 | 0.046 | 0.037 |

RMS lateral error in metres. Figure: `docs/figures/benchmark_road.png`.

## What the data support

- At 5 m/s the four controllers are within 0.07 to 0.09 m of each other. The 90th percentile of
  the maximum error is 0.287 m for all four, which is the size of the start offset, so maximum
  error at 5 m/s measures the start condition, not the controller.
- At 10 and 20 m/s the dynamic LQR and MPC have lower median RMS than Pure Pursuit and Stanley,
  and the gap widens with speed (at 20 m/s: 0.03 to 0.04 m against 0.19 to 0.25 m).
- LQR and MPC are close to each other everywhere. With 5 seeds and no intervals yet I do not
  rank them.

## What the data do not support (yet)

- **The 20 m/s LQR and MPC numbers are partly outside the tire model's valid range.** The linear
  tire force equals the friction limit (mu = 0.8, a placeholder) at a front slip angle of 4.68 deg
  and a rear slip angle of 3.00 deg. At 20 m/s the LQR exceeds the front limit in 10 of 20 runs
  (largest 7.01 deg) and the rear limit in 4 of 20; the MPC exceeds the front limit in 5 of 20
  (largest 5.54 deg). Their results there rely on forces a real tire at mu = 0.8 could not
  give. Nobody else crosses a limit. At 5 m/s all controllers stay below 2 deg. I have not
  checked whether the peaks occur in the start transient or on the path.
- **The gap to Pure Pursuit and Stanley is not shown to be the controller's fault.** I ran them
  with the parking-family settings and have not checked that those settings are sensible at
  20 m/s. Part of the gap may be tuning. Task 16 should test this before the gap is quoted.
- **The worst runs share a start condition.** The largest maximum errors (0.62 to 0.77 m) are
  all seed 2 on `road_curve_v20` (offset 0.237 m, heading error 0.071 rad), reached by all four
  controllers, so this is probably the start and not one controller's failure. The mechanism is
  not shown. Root cause is Task 16.
- **A heading error of 0.1 rad at 20 m/s is a hard start.** I reused the parking ranges without
  checking they suit 20 m/s.
- **Stanley's steering chatter at 5 m/s (median 1.67 reversals per second, others 0.2 to 0.3)
  is unexplained.** LQR is at 0.5 at 10 and 20 m/s. These are observations, not diagnoses.
- Most controllers reach the 0.8 rad/s steering-rate limit in the median run. I have not looked
  at where.
- 5 seeds per cell, no confidence intervals. Small differences between controllers are noise
  until Task 16 sizes the Monte Carlo.
