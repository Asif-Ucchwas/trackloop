# Road family: math reference (Task 14, road paths)

How the road-family reference paths are built, why they exist, and what is frozen. Predictions
are marked "derived", things seen in runs or checks "measured".

## 1. Why a second family

The benchmark speeds fixed in Task 4 are 5, 10 and 20 m/s. The 19 parking paths need far more
lateral acceleration than tires give at 10 and 20 m/s (feasibility note, section 7): 18 of 19
paths are not drivable there. A road family fixes that by sizing each path to its speed. It is
run with the dynamic controllers, at 5, 10 and 20 m/s. No controller has been run on any road
path: the set was exported and frozen first.

## 2. Construction (derived)

A path for design speed v is a curvature profile kappa(s), piecewise linear in arc length.
Driving curvature kappa at speed v needs lateral acceleration a = v^2 kappa, so for a chosen
peak a_y:

- peak curvature kappa_max = a_y / v^2, minimum radius R = v^2 / a_y;
- lateral jerk j = da/dt = v^3 dkappa/ds, so a ramp from 0 to kappa_max has length
  a_y v / j, and lasts a_y / j seconds whatever the speed.

With a_y = 2.0 m/s^2 and j = 1.0 m/s^3 (both placeholders):

| speed | peak curvature | min radius | ramp length | peak steering | steering rate on the ramp |
| --- | --- | --- | --- | --- | --- |
| 5 m/s | 0.080 /m | 12.5 m | 10 m | 0.213 rad | 0.106 rad/s |
| 10 m/s | 0.020 /m | 50 m | 20 m | 0.054 rad | 0.027 rad/s |
| 20 m/s | 0.005 /m | 200 m | 40 m | 0.0135 rad | 0.007 rad/s |

Peak steering is atan(L kappa_max) with L = 2.7 m. The steering rate needed on a ramp is that
angle over 2 s, at most 13% of the 0.8 rad/s limit. Because the curvature ramps instead of
jumping, the steering never has to make the instant step that most parking paths contain.

Heading is the exact integral of the curvature. Position is the integral of the heading,
done numerically on a 5 mm grid and sampled every 0.1 m; against a 1 mm grid the position
differs by less than 10 micrometres (tested).

## 3. The four shapes

Each has 3 s of straight before and after (3 v metres).

- `curve`: one 90 degree left turn: ramp up, hold, ramp down.
- `lateral_shift`: a left ramp-up-and-down, then a right one. The heading returns to zero and
  the car ends 14.9 to 15.9 m to the left. That is far more than a lane (3.5 m); the name says
  shift, not lane change.
- `s_curve`: 60 degrees left, 2 s of straight, 60 degrees right.
- `double_shift`: a lateral shift, 2 s of straight, and the mirrored shift back to the start
  line (it ends at y = 0).

`lateral_shift` and `double_shift` have the same lateral-acceleration history in time at all
three speeds (the figure shows the lines on top of each other). `curve` and `s_curve` hold the
peak longer at lower speed, because they turn a fixed angle.

## 4. Results (measured)

12 paths: 4 shapes x 3 design speeds, ids like `road_curve_v10`. Lengths run from 60 m
(`road_curve_v5`) to 659 m (`road_s_curve_v20`). At its design speed every path passes all
three Task 12 checks: curvature within the limit, steering rate (zero lag distance) and lateral
acceleration (exactly 2.000 m/s^2 peak). All paths are forward only, with no cusps.

## 5. Is 2.0 m/s^2 inside the dynamic model's range? (derived)

The dynamic model has linear tires. At lateral acceleration a the front axle carries
m a l_r / L and the rear axle m a l_f / L; with m = 1500 kg, l_f = 1.2 m, l_r = 1.5 m and
cornering stiffnesses 80 000 and 100 000 N/rad:

| a | front slip angle | rear slip angle |
| --- | --- | --- |
| 2.0 m/s^2 | 1.19 deg | 0.76 deg |
| 4.0 m/s^2 | 2.39 deg | 1.53 deg |
| 7.85 m/s^2 (mu g) | 4.68 deg | 3.00 deg |

Slip angles of about 1 degree are well inside the range where a linear tire law holds; real
tires leave it as the friction limit nears. So the road results will describe the controller
and the model error, not tire saturation. This holds for these placeholder parameters.

## 6. Frozen and reproducible

- Files are in `data/paths/road/` with their own manifest (`manifest.json`, 12 entries,
  sha256 of every file), so the 19-path parking manifest and its tests are unchanged.
- The json files carry the design settings and no Python version or timestamp, so they are the
  same whichever machine exports them.
- A test compares the stored arrays with what the code generates now (to 1 micrometre), so a
  change to the generator is caught. Changing a setting means editing `road_paths.py`,
  re-exporting on purpose and saying so in DEVLOG.

## 7. Limits

- a_y = 2.0, j = 1.0, the 3 s and 2 s straights, and the 90 and 60 degree angles are choices.
- Constant design speed, no grade, banking or wind. The paths test lateral tracking only.
- No planner-generated paths at road scale (the thesis planners' output on a larger grid) are
  included; this family is analytic.
- A tracked speed other than the design speed is not part of the benchmark. At 10 m/s on a 5
  m/s path the lateral acceleration would be 8 m/s^2.

## 8. Code

`src/trackloop/road_paths.py`, `scripts/export_road_paths.py`, `tests/test_road_paths.py` (36
tests), `scripts/plot_road_family.py`; data `data/paths/road/` (12 paths and `manifest.json`);
figure `docs/figures/road_family.png`. `src/trackloop/testset.py` gained a road prefix, a
road group and a manifest rule argument.
