# Benchmark design, parking family: math reference (Task 14)

What the benchmark runs, how the runs are made repeatable, and what the first table does and
does not show. Predictions are marked "derived", things seen in runs "measured".

## 1. Scope

Two path families were chosen after the lateral-acceleration check (feasibility note, section
7). This task builds the parking family only: the 19 frozen paths at low speed, tracked with
the four kinematic controllers through the cusp runner. The road family (paths scaled so each
speed sees the same lateral acceleration, run with the dynamic controllers at 5, 10 and 20
m/s) does not exist yet and must be exported and frozen first. The dynamic LQR and MPC
therefore are not in this table.

## 2. The grid

19 paths x 4 controllers x 3 speeds x 5 seeds = 1140 runs. Speeds are 0.5, 1.5 and 3.0 m/s.
At 3.0 m/s the largest lateral acceleration of any path is about 2.6 m/s^2 (derived: v^2 x
0.294), well under the 7.85 limit. Five seeds is a placeholder: Task 16 sizes the seed count
from confidence-interval width.

## 3. Starting conditions

The seed fixes a start offset of up to 0.3 m to the left or right of the path start and a
heading error of up to 0.1 rad, both uniform. The generator is seeded by (seed, path index)
and not by the controller, so all four controllers start from the same condition and can be
compared pair by pair. Each row records the offset (e0) and the heading error (dpsi0).

## 4. Flags, not exclusions

Every path is run at every speed. Each row carries the Task 12 checks at that speed:

- `drivable`: curvature within the steering limit and lateral acceleration within mu g. Two
  paths fail it at every speed (`thesis_hybrid_seed3`, `thesis_theta_star_tight_seed3`):
  120 of 1140 rows.
- `steering_events` and `lag_distance_m`: the steering-rate check as a number. Measured: 15
  paths have an unfollowable curvature step at every benchmark speed, even 0.5 m/s, because
  the curvature jumps within a very short sample interval. A pass or fail flag would be set on
  most of the grid and tell nothing, so the lag distance (metres driven with the steering
  behind the path) is reported instead. It grows with speed: 3.7 m on `rs_side_slot` at 3.0
  m/s.
- `feasible`: all three checks pass. 120 of 1140 rows. It is kept for completeness; the
  summary uses `drivable`.

A controller that raises gives an error row (`status` starts with "error"), never a missing
row. In this run there were none.

## 5. Repeatability

- The run is deterministic: two full runs gave identical summary numbers. A test checks one
  run twice.
- `data/results/benchmark_parking_settings.json` records speeds, seeds, offset ranges,
  controller names, paths, runner and vehicle parameters, mu, the metric constants, the git
  commit (with a flag for uncommitted changes to tracked files) and the Python and package
  versions. The controller gains are the ones from earlier tasks; none were changed using
  these results.
- The time limit is 3 x length / speed + 60 s + 20 s per segment. The runner's fixed 300 s
  would have stopped a 195 m thesis path at 0.5 m/s (390 s of driving) and reported a healthy
  run as unfinished.
- Runs are independent and run in parallel; the table does not depend on the number of
  workers.

## 6. First results (measured, exploratory)

All 1140 runs finished with no errors. Medians over the drivable paths, all seeds
(`benchmark_parking_summary.csv`), median RMS lateral error in metres and the 90th percentile
of the maximum error:

| controller | 0.5 m/s | 1.5 m/s | 3.0 m/s |
| --- | --- | --- | --- |
| Pure Pursuit | 0.046 / 0.29 | 0.067 / 0.31 | 0.101 / 0.42 |
| Stanley | 0.124 / 0.65 | 0.131 / 0.67 | 0.143 / 0.64 |
| Kinematic LQR | 0.037 / 0.28 | 0.057 / 0.96 | 0.129 / 1.60 |
| Kinematic MPC | 0.037 / 0.28 | 0.058 / 0.96 | 0.130 / 1.60 |

What stands out:

- Kinematic LQR and MPC are not separable here: their median RMS differs by at most 0.007 m on
  any path and speed. Both are designed on the model that is also the plant.
- At 3.0 m/s Pure Pursuit has the lowest median on 18 of the 19 paths. LQR and MPC reach 1.02
  m on `rs_parallel_park` and 1.48 m on `rs_side_slot`, about 7 and 9 times Pure Pursuit
  (0.15 and 0.16 m). My reading is the steering rate limit meeting an instant curvature step,
  as in Task 11, which Pure Pursuit's lookahead smooths. I have not shown it. Task 16 root-
  causes the worst runs.
- At 0.5 m/s LQR and MPC are the best on the eight drivable thesis paths (0.02 to 0.03 m
  against 0.03 to 0.05 for Pure Pursuit and 0.10 to 0.17 for Stanley). On the two seed-3 paths
  that break the curvature limit they are at 0.06 m and Pure Pursuit is lower (0.04 to 0.05).
- Stanley is worst on `rs_three_point` (0.55, 0.49, 0.36 m against 0.08 to 0.11 for the other
  three), and has the highest steering chatter (Task 13).
- On `rs_ahead` all controllers show 0.04 to 0.07 m, which is only the start offset
  converging.

## 7. Limits

- The plant is the kinematic model the controllers are designed on: no model error, no noise,
  no sensor delay. That flatters LQR and MPC. Task 15 adds model error.
- A median over paths mixes paths of very different length and difficulty. The per-path
  table is the result; the summary is an overview. The figure shows every cell.
- Five seeds give no usable interval. Medians over seeds are shown, not intervals.
- The kinematic MPC is designed at 1.5 m/s and run at 0.5 and 3.0 m/s unchanged; it still
  tracks like LQR there, so the design speed does not decide the result.
- The family is low-speed and has cusp stops. It says nothing about highway-speed tracking.

## 8. Code

`src/trackloop/harness.py`, `scripts/run_benchmark_parking.py`, `tests/test_harness.py` (9
tests), `scripts/plot_benchmark_parking.py`; results `data/results/benchmark_parking.csv`
(1140 rows), `benchmark_parking_summary.csv`, `benchmark_parking_settings.json`; figure
`docs/figures/benchmark_parking.png`.
