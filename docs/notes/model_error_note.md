# Model-error cases (Task 15)

What was run, what the numbers say, and what they do not say. Numbers are from
`data/results/model_error_summary.csv` (rows in `model_error_road.csv` and
`model_error_parking.csv`), produced at commit `636f30c` with a clean tree.
Figure: `docs/figures/model_error.png`.

## Setup

- The controllers are always built on the nominal parameters (`DynamicParams()` for the road
  family, the controllers' own defaults for parking). A case changes only the plant the run is
  simulated on, so the controller does not know about the difference.
- Same frozen paths, design speeds and seeds as the nominal benchmarks. Tests check that the
  nominal case reproduces the benchmark row for sample runs; they do not check every row.
- **The case sizes are placeholders.** They are meant to be clearly larger than estimation noise
  and clearly smaller than a different vehicle. They are not measured values for any real car.
- Road family (dynamic plant, 8 cases including nominal): mass +-20 percent with yaw inertia
  scaled by the same factor; cornering stiffness +-30 percent on both axles; steering actuator
  lag as a first-order time constant of 0.1 s and 0.2 s; and one combined case (mass +20 percent,
  stiffness -30 percent, lag 0.1 s).
- Parking family (kinematic plant, 5 cases including nominal): steering lag 0.1 s and 0.2 s, and
  wheelbase +-5 percent. The kinematic model has no mass or tire, so nothing else applies.
- Runs: road 1,920 rows, 0 errors, 5 did not finish. Parking 5,700 rows, 0 errors, all finished.
  The 5 road runs that did not finish are all Dynamic MPC at 20 m/s with 0.2 s lag (5 of that
  cell's 20 runs). Why they did not finish is examined in `docs/notes/root_cause_note.md`, section 4.
- Metric: for each run, RMS lateral error divided by the RMS of the nominal run with the same
  path, controller, seed and speed. The tables give the median of those per-pair ratios, not a
  ratio of medians. 1.00 means no change; above 1 is worse. Pairs where either run did not
  finish are left out.

## Results: road family (median paired ratio)

| controller | m/s | mass +20% | mass -20% | stiff +30% | stiff -30% | lag 0.1 s | lag 0.2 s | combined |
|---|---|---|---|---|---|---|---|---|
| Pure Pursuit | 5 | 1.05 | 0.97 | 0.96 | 1.11 | 0.89 | 0.87 | 1.15 |
| Pure Pursuit | 10 | 1.17 | 0.84 | 0.82 | 1.36 | 1.01 | 1.08 | 1.65 |
| Pure Pursuit | 20 | 1.20 | 0.80 | 0.77 | 1.44 | 1.02 | 1.10 | 1.80 |
| Stanley | 5 | 0.91 | 1.09 | 1.11 | 0.82 | 0.92 | 0.88 | 0.65 |
| Stanley | 10 | 1.25 | 0.78 | 0.75 | 1.54 | 1.01 | 1.05 | 1.92 |
| Stanley | 20 | 1.21 | 0.79 | 0.76 | 1.46 | 1.00 | 1.02 | 1.77 |
| Dynamic LQR | 5 | 0.94 | 1.07 | 1.09 | 0.89 | 0.91 | 0.84 | 0.81 |
| Dynamic LQR | 10 | 1.06 | 1.03 | 1.05 | 1.20 | 0.96 | 0.93 | 1.45 |
| Dynamic LQR | 20 | 1.31 | 0.95 | 0.99 | 1.84 | 1.00 | 1.00 | 2.61 |
| Dynamic MPC | 5 | 0.93 | 1.08 | 1.10 | 0.88 | 0.95 | 0.95 | 0.83 |
| Dynamic MPC | 10 | 1.08 | 1.05 | 1.06 | 1.26 | 1.10 | 1.25 | 1.70 |
| Dynamic MPC | 20 | 1.35 | 0.93 | 0.95 | 1.89 | 1.09 | 1.14 (5 not finished) | 2.76 |

## Results: parking family (median paired ratio)

| controller | m/s | lag 0.1 s | lag 0.2 s | wheelbase +5% | wheelbase -5% |
|---|---|---|---|---|---|
| Pure Pursuit | 0.5 | 0.97 | 0.94 | 1.04 | 0.98 |
| Pure Pursuit | 1.5 | 0.94 | 0.86 | 1.04 | 0.98 |
| Pure Pursuit | 3.0 | 0.90 | 0.79 | 1.04 | 0.98 |
| Stanley | 0.5 | 1.00 | 1.00 | 0.98 | 1.02 |
| Stanley | 1.5 | 0.99 | 0.97 | 0.97 | 1.04 |
| Stanley | 3.0 | 0.96 | 0.91 | 0.95 | 1.05 |
| Kinematic LQR | 0.5 | 1.00 | 1.02 | 1.07 | 1.05 |
| Kinematic LQR | 1.5 | 1.00 | 1.02 | 1.04 | 1.00 |
| Kinematic LQR | 3.0 | 1.00 | 1.00 | 1.03 | 0.99 |
| Kinematic MPC | 0.5 | 1.07 | 1.48 | 1.08 | 1.05 |
| Kinematic MPC | 1.5 | 1.52 | 3.11 | 1.04 | 1.00 |
| Kinematic MPC | 3.0 | 1.70 | 6.96 | 1.04 | 0.99 |

## What the data support

- **Road, 10 and 20 m/s: the stiffness-to-mass direction matters most.** For all four
  controllers, more mass or less stiffness raises the error (mass +20%: 1.06 to 1.35; stiffness
  -30%: 1.20 to 1.89) and less mass or more stiffness lowers or leaves it near 1 (0.75 to 1.06).
  The effect grows with speed. At 5 m/s the single-parameter effects are smaller and sometimes
  reversed (0.82 to 1.11).
- **Combined case: the worst for every controller at 10 and 20 m/s**, up to 2.61 for Dynamic LQR
  and 2.76 for Dynamic MPC at 20 m/s, against 1.77 to 1.80 for Pure Pursuit and Stanley.
- **Steering lag barely moves the road controllers at 10 and 20 m/s** except Dynamic MPC (1.10 and
  1.25 at 10 m/s, 1.09 and 1.14 at 20 m/s) and Pure Pursuit at 0.2 s (1.08 to 1.10). The other
  cells are within 7 percent of 1. At 5 m/s lag lowers the error by 5 to 16 percent for all four.
- **Parking: Kinematic MPC is the one controller that steering lag hurts badly**, from 1.07 and
  1.48 at 0.5 m/s to 1.70 and 6.96 at 3 m/s (0.1 s and 0.2 s). Kinematic LQR stays within 2
  percent of 1.00 for lag. The effect grows with speed.
- **Pure Pursuit and Stanley get slightly better with lag in parking** (0.79 to 1.00).
- **Wheelbase +-5 percent is small in parking.** Every cell is between 0.95 and 1.08.

## What the data do not support (yet)

- **Ratios are relative to each controller's own nominal error, not to each other.** The dynamic
  LQR and MPC have a very small nominal error at 20 m/s (0.034 and 0.037 m in Task 14), so a
  ratio of 2.6 is still only about 0.09 to 0.10 m, which is below Pure Pursuit's and Stanley's
  nominal error there (0.251 and 0.193 m). This is an approximation (a median of ratios times a
  median RMS); the exact absolute values are in the `median_rms` column. Read the tables as
  sensitivity, not as a ranking.
- **The sizes are placeholders**, so none of this says how a real vehicle would behave. It
  says which direction and roughly how steeply each controller responds.
- **No intervals.** There are 5 seeds. Cells within about 0.95 to 1.05 are not evidence of an
  effect, and I have not compared two controllers' ratios.
- **The 5 runs that did not finish are left out of the ratio.** If they are the worst runs, the
  1.14 for Dynamic MPC at 20 m/s with 0.2 s lag is flattering (they are all divergences; see
  `docs/notes/root_cause_note.md`, section 4).
- **The 20 m/s tire-limit caveat from Task 14 applies, but it is not worse under the softer-tire
  cases.** The plant keeps linear tires. The slip angle at which a tire reaches mu * Fz rises when
  stiffness falls (front 4.68 to 6.69 degrees at -30 percent), so fewer runs exceed it. The counts
  are in `docs/notes/root_cause_note.md`, section 5.
- **No mechanisms are shown.** I do not know why lower stiffness or more mass hurts more at
  speed, why Kinematic MPC is so sensitive to lag while Kinematic LQR is not (the MPC's
  prediction has no lag in it, but I have not tested that), why lag helps Pure Pursuit and Stanley
  (a low-pass effect on their command is plausible, not tested), or why the combined case improves
  Stanley, Dynamic LQR and Dynamic MPC at 5 m/s (0.65 to 0.83).
- **One change at a time, plus one combination.** I did not test whether the combined case is
  more than the sum of its parts.

## Reproduce

`python scripts/run_model_error.py` writes the four result files (about 90 s on the Legion);
`python scripts/plot_model_error.py` writes the figure.
