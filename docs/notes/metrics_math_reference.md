# Tracking metrics: math reference (Task 13)

Defines every number the benchmark will report, with the formula, the sign convention and
the code that computes it. Predictions are marked "derived", things seen in runs "measured".

## 1. Why a definition is needed

Before this task the repository measured lateral error two ways. `simulate.py` and
`cusp_runner.py` use `cross_track_error`: the distance to the nearest path vertex, resolved
along that vertex's heading. `simulate_dynamic.py` uses `project`: the distance to the nearest
path segment, taken at the centre of gravity. A table mixing the two would compare unlike
numbers. All benchmark scores now come from `trackloop/metrics.py`, from the logged run only.

## 2. Tracked point and sign convention

- The tracked point is the rear axle (the point the kinematic model integrates). A dynamic
  run logs the centre of gravity; `rear_axle(x, y, psi, lr)` moves it lr = 1.5 m back.
- Lateral error e is the signed distance from the tracked point to the nearest point of the
  path polyline, positive to the left of the path heading (the path stores the body heading,
  also in reverse). The foot point comes from projection onto segments, so it does not depend
  on how finely the path is sampled.
- Heading error psi_e = wrap(psi - h(foot point)) in (-pi, pi], positive when the vehicle is
  rotated left of the path.
- On a path with cusps each sample is measured against the path of the segment active when it
  was logged.

## 3. Definitions

For N samples with times t_k, lateral errors e_k and heading errors psi_e,k:

| metric | formula | unit |
| --- | --- | --- |
| rms_lateral | sqrt(mean(e_k^2)) | m |
| max_lateral | max abs(e_k) | m |
| rms_heading | sqrt(mean(psi_e,k^2)) | rad |
| max_heading | max abs(psi_e,k) | rad |
| steering_effort | sqrt(mean(delta_k^2)) over driving samples | rad |
| steering_rate_rms | sqrt(mean(r_k^2)), r_k = (delta_k - delta_k-1) / dt, driving steps | rad/s |
| steering_rate_max | max abs(r_k) over driving steps | rad/s |
| chatter_per_s | sign reversals of r_k outside a deadband, divided by driving time | 1/s |
| total_time | t_N - t_0, standstills included | s |
| driving_time | sum of dt over steps with abs(v) > 0.01 m/s | s |

Choices that change the numbers:

- Steering metrics count only driving samples. Steering moved at a standstill before a cusp
  is settling, not tracking, and would add to the rate metrics.
- The deadband is 0.01 rad/s, so numerical noise is not counted as chatter. It is a
  parameter of `compute`, and the value is a choice, not a derived number.
- Lateral and heading errors are over all samples, standstills included. Samples at rest add
  repeated copies of the cusp error, so the RMS slightly favours runs that stop more
  accurately; the max is unaffected.

Cusp-aware metrics (from `cusp_runner.RunResult`, collected by `score_cusp_run`):

- worst_cusp_position_error: largest distance between the stopped vehicle and the cusp point.
- worst_settle_time: longest wait for the steering to reach the next segment's command.
- end_position_error and end_heading_error at the goal, and `finished`.

Any other quantity in a results table must be added here first.

## 4. Check against the old measure

Measured on 36 runs (4 controllers x 9 Reeds-Shepp paths): projection-based RMS and max
lateral error differ from the stored nearest-vertex values by at most 0.0005 m. The
cusp-tracking results from Task 11 therefore stand under the new definition.

## 5. What the metrics show (measured, exploratory)

`data/results/cusp_metrics.csv`, one run per pair, start on the path, no noise. Means over the
nine paths: steering-rate reversals per second are 0.32 (Pure Pursuit), 3.61 (Stanley), 0.34
(kinematic LQR) and 0.34 (kinematic MPC). Stanley chatters 2 to 9 times per second on six of
the nine paths, and the other three controllers stay below 0.7 on all nine. RMS steering rate
is 0.16, 0.21, 0.22 and 0.22 rad/s. Lateral error alone does not show this; it is the reason
chatter is its own metric. I have not diagnosed the Stanley chatter.

## 6. Limits

- The steering log is the steering angle state, not the controller's request. A request
  clipped by the rate limit is not visible.
- Reversals count a sign change of a finite difference at dt = 0.05 s, so the value depends on
  the time step. Compare runs only at the same dt.
- The dynamic path (centre of gravity to rear axle, steering in column 6, speed in column 3)
  is described here but not yet exercised by a test or a run.
- Metrics from one run are one sample. Intervals come with the Monte Carlo (Task 16).

## 7. Code

`src/trackloop/metrics.py`, `tests/test_metrics.py` (9 tests), `scripts/score_cusp_runs.py`,
`data/results/cusp_metrics.csv`, `docs/figures/metrics.png`.
