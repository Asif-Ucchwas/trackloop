# Test set and feasibility checks: math reference (Task 12)

Companion to `cusp_tracking_math_reference.md`. It explains what is in the fixed test set,
how a path is checked against the car's steering limit and steering rate, and what the first
check says about the 19 paths. Derived results and measured results are marked as such.

## 1. The test set

Rule: every exported path is in the set. No path is chosen or dropped by how a controller
scores on it, and a path that fails a feasibility check stays in and is flagged.

- 10 thesis paths (planners hybrid and theta_star_tight, five maps each) and 9 Reeds-Shepp
  paths (radius 5 m): 19 in all. Samples are 0.1 m apart, except that a sample sits exactly
  on each piece boundary, so some intervals are shorter (down to 0.0002 m).
- `data/paths/manifest.json` records, for each path, the sha256 of its `.npz` and `.json`
  files, the number of samples, the length, the largest curvature and the cusp count.
  `tests/test_testset.py` fails if a file changes, a path is added or removed, or the
  manifest no longer equals a fresh build. Changing the set means regenerating the manifest
  on purpose, in a commit that says so.

Disclosure for anyone reading the benchmark later: the nine Reeds-Shepp paths were used
while the Task 11 controllers and runner were developed (for example, Stanley's feedforward
is switched on for reverse segments only, a choice made while looking at these paths), so
they are development paths, not held-out ones. No controller was run on a thesis path during
Task 11. Earlier stages were not rechecked for this note.

## 2. Curvature limit

Rear-axle bicycle model: curvature kappa = tan(delta) / L. With |delta| <= 0.6 rad and
L = 2.7 m,

    kappa_max = tan(0.6) / 2.7 = 0.68414 / 2.7 = 0.25338 per metre,   R_min = 3.9466 m.

A sample with |kappa| above this cannot be driven as drawn at any speed. The steering that
a curvature needs is delta_ref = atan(L kappa).

Worked examples (measured on the exported paths, derived steering):

- `thesis_hybrid_seed3`: largest curvature 0.29368, so delta_ref = atan(2.7 x 0.29368)
  = 0.6704 rad, which is 0.0704 rad (4.0 deg) beyond the limit; 28 samples exceed it, by up
  to 0.0403 per metre.
- `thesis_theta_star_tight_seed3`: 0.27890, delta_ref = 0.6454 rad, 0.0454 rad over; 41
  samples, by up to 0.0255 per metre.
- A 6 m corner arc (the thesis rounding radius) is kappa = 0.1667, delta_ref = atan(0.45)
  = 0.4229 rad, well inside the limit. How the two seed-3 paths end up tighter than 6 m is
  not explained here.

## 3. Steering-rate check

Along a path the reference steering delta_ref changes between neighbouring samples by d,
over a distance ds. At speed v the car spends ds / v seconds on that step, and the steering
actuator, limited to r = 0.8 rad/s, can move by r ds / v. So the step is followable when

    |d| <= r ds / v.

At v = 1.5 m/s and ds = 0.1 m that is 0.8 x 0.1 / 1.5 = 0.0533 rad per sample.

Neighbouring steps that are not followable form one event, with total change D. The
steering needs D / r seconds to cover it, which is

    lag distance = v D / r

metres of driving with the steering behind the path. Worked example, a straight line into a
5 m arc: delta_ref jumps from 0 to atan(0.54) = 0.4951 rad in one step, 9.3 times the
allowed 0.0533 rad. D = 0.4951 rad, so 0.619 s and 0.93 m. A left-to-right flip is
D = 2 x 0.4951 = 0.9903 rad, so 1.238 s and 1.86 m. The same 0.4951 rad step becomes
followable only when v <= 0.8 x 0.1 / 0.4951 = 0.16 m/s (derived).

Choices and their effects:

- Steps across a cusp are not counted. The car is stopped there and the runner lets the
  steering settle before the next segment (Task 11).
- A gradual change is followable: a curvature ramp from 0 to 0.2 over 2 m (21 samples) has
  a first step of 0.027 rad, under 0.0533, so there are no events; over 0.5 m (6 samples)
  every step is above 0.0533 and it is one event of 0.4951 rad.
- A real curvature step is an event at any sample spacing; a ramp's classification depends
  on the spacing and on the speed. The 19 paths contain only constant-curvature pieces and
  steps between them, so their result does not depend on the spacing.
- For a path over the curvature limit the reference steering is not clipped, so the rate
  check sees the demand, not what the car could do.
- The lag is what the rate-limited actuator needs, starting from the old reference value.
  It measures the path's demand. It is not a prediction of the tracking error.

## 4. First results (measured)

At 1.5 m/s, car as in `models/kinematic.py` (`data/results/feasibility.csv`,
`docs/figures/feasibility.png`):

- 2 of 19 paths pass both checks: `rs_ahead` and `rs_three_point`. Every other path has at
  least one steering step the actuator cannot follow, because the planners' paths jump in
  curvature where a line meets an arc or an arc changes direction.
- Over the curvature limit: only the two seed-3 thesis paths.
- Lag distance, Reeds-Shepp: 0 (`ahead`, `three_point`), 0.93 m (`behind`, `diag`,
  `lane_shift`, `reverse_in`, `u_turn`), 1.86 m (`parallel_park`, `side_slot`).
- Lag distance, thesis: 0.79 to 1.76 m. The number of events on each thesis path equals the
  number of times its curvature changes. I checked this once with a separate script on all
  ten (for example 22 arcs and 41 events on `thesis_hybrid_seed0`); that script is not in
  the repo and no test covers it.
- Observation, not a law: on the Reeds-Shepp paths run in Task 11, the 0.93 m paths had
  kinematic LQR errors of 0.12 to 0.19 m, and the two 1.86 m paths had 1.0 to 1.1 m. That
  is two paths at the high end.

## 5. Limits

- Placeholder car (wheelbase, steering limit, rate limit), and one speed.
- A pure rate limiter; no steering acceleration limit, no actuator delay.
- The lag is a demand measure, not tracking error.
- Few paths: nine Reeds-Shepp paths are short and share one radius.

## 6. Metric definitions (computed only in code)

All from `trackloop/feasibility.py`, per path, at speed v:

- samples_over_limit: samples with |kappa| > kappa_max.
- max_excess: largest |kappa| minus kappa_max, 0 if none.
- steering step: |delta_ref[i+1] - delta_ref[i]| inside one gear segment, with
  delta_ref = atan(L kappa) and the segment's last curvature taken from the sample before
  it, as in `segments.py`.
- event: a run of consecutive steps each above r ds / v (with a 1e-9 relative tolerance).
- steering_events, largest_event_rad: the number of events and the largest total change D.
- worst_lag_distance: v D / r for the largest event.
- feasible: no samples over the limit and no events.

Code: `testset.py`, `feasibility.py`, `scripts/build_testset_manifest.py`,
`scripts/check_testset_feasibility.py`, `scripts/plot_feasibility.py`, and tests in
`tests/test_testset.py` and `tests/test_feasibility.py`.

## 7. Addendum: lateral acceleration and speed applicability

Added after Task 12 was pushed, when the benchmark speeds fixed in Task 4 (5, 10 and
20 m/s) were compared with the paths.

Driving curvature kappa at speed v needs a lateral acceleration a = v^2 |kappa| (kinematic
model, no slip). Tires hold about mu g; with mu = 0.8 (a placeholder) and g = 9.81 that is
7.848 m/s^2. A path is drivable at speed v when v^2 max|kappa| <= mu g, so its largest
drivable speed is

    v_max = sqrt(mu g / max|kappa|),

and a straight path has no limit.

Worked examples (derived):

- Reeds-Shepp, kappa = 0.2 (5 m radius): v_max = sqrt(7.848 / 0.2) = 6.264 m/s. At 5 m/s the
  path needs 25 x 0.2 = 5.0 m/s^2 (drivable); at 10 m/s, 20.0; at 20 m/s, 80.0, about 8 g.
- Thesis, 6 m corners, kappa = 0.1667: v_max = 6.86 m/s. `thesis_hybrid_seed0` (0.214): 6.05
  m/s. The two seed-3 paths (0.294 and 0.279): 5.17 and 5.30 m/s.

Result (measured, `data/results/speed_applicability.csv`): drivable at 1.5 m/s, 19 of 19
paths; at 5 m/s, 19 of 19; at 10 m/s, 1 of 19 (`rs_ahead`); at 20 m/s, 1 of 19 (`rs_ahead`).
The frozen paths therefore cannot be driven at the 10 and 20 m/s benchmark speeds. At 5 m/s
the seed-3 thesis paths pass this check but still fail the steering-limit check.

Why it matters: the syllabus describes the dynamic model as using linear tires, which do not
saturate, so it would simulate 8 g without complaint and return a tracking error that means
nothing. I have not confirmed this from the model's code.

Decision (the user's, taken after this check): two path families. A parking family, the 19
frozen paths at low speed, tracked with the kinematic controllers. A road family, paths whose
curve radius grows with the square of the speed so each benchmark speed sees the same lateral
acceleration, run at 5, 10 and 20 m/s with the dynamic controllers. The road family does not
exist yet. It has to be exported and frozen in the manifest before any benchmark run.

`check_path` now also returns lateral_accel_limit (mu g), max_lateral_accel (v^2 max|kappa|
at the given speed), max_feasible_speed and lateral_ok, and `feasible` needs all three
checks. mu is a parameter of `check_path`.

Limits: v^2 kappa ignores load transfer, banking and the shape of tire saturation; mu is a
placeholder; v_max comes from the sharpest corner only, so a path can be drivable faster over
most of its length.

Code: `scripts/speed_applicability.py` and `tests/test_lateral.py`; the third panel of
`docs/figures/feasibility.png` shows v_max against the benchmark speeds.
