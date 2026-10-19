# Reeds-Shepp export: math reference

Code: src/trackloop/rs_export.py, scripts/export_rs_paths.py,
scripts/plot_rs_paths.py. Tests: tests/test_rs_export.py. Solver: MultiPlan,
reeds_shepp_path.py (verified there against 400 reference cases).

## 1. Input and conventions

MultiPlan returns a path as a list of segments (turn, signed_length), in units of the
turning radius r:

- turn is L (steer left), R (steer right) or S (straight).
- A negative length means reverse gear.
- For L and R the length is the angle turned, in radians, so the arc is r |length|
  metres long. For S the length is distance / r, so the line is r |length| metres.

The TrackLoop path stores, for each sample:

- heading: where the car points (body heading), continuous, not wrapped.
- curvature: steering curvature kappa, +1/r for L, -1/r for R, 0 for S, in both gears.
- direction: g = +1 forward, -1 reverse. Sample i carries the direction of the motion
  that leaves it; the last sample carries the last segment's.

Heading is not the direction of travel. In reverse the car moves along the opposite
way to the way it points, so the tangent of the driven curve is heading + pi.

## 2. Pose along one arc

Let the piece start at (x0, y0, h0) with gear g and curvature kappa, and let t be the
arc length driven on it (t >= 0). In reverse the car still turns by the angle the
steering produces per metre rolled, but in the opposite sense: with ds the distance
rolled, the kinematic bicycle gives d(heading) = g kappa ds. So

    h(t) = h0 + g kappa t
    d(x, y)/dt = g (cos h, sin h)

Integrate, using g kappa as the rate of change of h:

    x(t) = x0 + g * integral of cos(h0 + g kappa u) du
         = x0 + g * [sin(h) - sin(h0)] / (g kappa)
         = x0 + [sin(h) - sin(h0)] / kappa
    y(t) = y0 + g * integral of sin(h0 + g kappa u) du
         = y0 - [cos(h) - cos(h0)] / kappa

The factor g cancels because g = +-1 gives g / g = 1. So one formula covers both
gears; only h(t) depends on g. For a straight piece (kappa = 0):

    h = h0     x = x0 + g t cos(h0)     y = y0 + g t sin(h0)

Check, forward left turn of angle pi/2 and r = 4 from (0, 0, 0): kappa = 0.25,
t = 2 pi, h = pi/2, x = (1 - 0) / 0.25 = 4, y = -(0 - 1) / 0.25 = 4. This is a test.

Every piece starts where the last ended, so the pose is continuous. Heading cannot
jump, not even at a cusp: the car stops and reverses without rotating.

## 3. Why the curvature sign does not change with gear

The steering angle delta gives the curvature kappa = tan(delta) / wheelbase of the
rear-axle path. The yaw rate is v kappa with v the signed speed, so over a distance
ds rolled, d(heading) = g kappa ds. Left steering is kappa > 0 whether the car goes
forwards or backwards. So an L segment keeps kappa = +1/r in reverse. Its heading
then falls as the car rolls back: d(heading)/d(arc length) = kappa g = -1/r.

This is the quantity the Task 12 check compares with the steering limit, and it is
the same quantity as for the forward-only paths of Task 9.

## 4. Cusps and sampling

A cusp is a point where the gear changes between two pieces that have length. A
piece with length below 1e-9 m is dropped first, so a zero-length piece cannot create
or hide a cusp. The cusp arc length is the start of the piece after the change.

Samples are taken at s = 0, ds, 2 ds, ... and at the total length, plus at every
piece boundary. Grid points within 1e-9 m of a boundary are removed so that the
boundary is sampled once, exactly. The piece for a sample is the one that starts at
or before it, so the cusp sample is the first sample of the new gear and its direction
is the new gear. The sample before it still has the old gear.

## 5. Worked example: reverse_in (r = 5 m, MultiPlan commit 8be3e44)

Start (0, 0, 0 deg), goal (-12, 8, 90 deg). MultiPlan word CSC90|C. Segments:

    L -0.7925  (reverse)    S -0.8142 (reverse)    R -1.5708 (reverse)    L +0.7925 (forward)

Piece 1, L reverse: angle 0.7925 rad (45.41 deg), length 3.9626 m, kappa = 0.2,
g = -1. h = 0 + (-1)(0.2)(3.9626) = -0.7925. sin(0.7925) = 0.71213, cos = 0.70205:

    x = -0.71213 / 0.2 = -3.5606      y = -(0.70205 - 1) / 0.2 = 1.4897

End pose (-3.5606, 1.4897, -0.7925).

Piece 2, S reverse: 0.8142 x 5 = 4.0712 m, heading stays -0.7925, end pose
(-6.4189, 4.3890, -0.7925).

Piece 3, R reverse: angle pi/2, length 7.8540 m, kappa = -0.2, g = -1:
h = -0.7925 + (-1)(-0.2)(7.8540) = +0.7783, end pose (-13.4897, 4.4394, 0.7783).

Cusp: s = 3.9626 + 4.0712 + 7.8540 = 15.8878 m, at (-13.4897, 4.4394), body heading
0.7783 rad (44.59 deg). The cusp sample has direction +1, the one before it -1.

Piece 4, L forward: angle 0.7925, length 3.9626 m, kappa = 0.2, g = +1,
h = 0.7783 + 0.7925 = 1.5708 = 90 deg. End pose (-12.0000, 8.0000, 1.5708), which is
the goal. Total length 19.8505 m (the rounded piece lengths above add to 19.8504;
the table says 19.850).

The exported path has 203 samples: 161 in reverse, 42 forward.

## 6. Quantities computed only in code

- end_position_error_m: the distance from the end pose of the exact piece formulas to
  the goal position. It is a check of the converter, not of the planner.
- end_heading_error_rad: the absolute value of the wrapped angle between the end
  heading and the goal heading, wrapped to [-pi, pi).
- multiplan_end_gap_m: the distance between the last sample and the end that
  MultiPlan's own sampler (path_endpoint, 0.05 m steps) reaches for the same segments.
- n_cusps and cusp_s_m: the count and the arc lengths from section 4.
- max_curvature: the largest |kappa| over the samples, which is 1/r when any arc exists.

The export script stops without saving if the position or heading error exceeds 1e-6,
if the length differs from MultiPlan's by more than 1e-6 m, or if the gap to
MultiPlan's sampler exceeds 1e-3 m.

## 7. Limits of these findings

- Nine paths, with start and goal poses chosen by hand, one radius (5 m). They show
  that the conversion is consistent with MultiPlan. They do not show how often
  Reeds-Shepp needs a cusp for any real parking or driving task.
- Four of the nine have cusps (parallel_park 2, side_slot 2, three_point 2,
  reverse_in 1). In parallel_park the two cusps lie 0.29 m from the ends, and in
  side_slot 1.38 m from the ends: MultiPlan's solution rolls a short stub in the
  opposite gear at each end. That is how those solutions are, not an exporter fault.
- The u_turn word contains | but the path has no cusp: the first piece has zero
  length, so the drive is in reverse only. The cusp count follows the pieces, not the
  word.
- The end pose of every path matches the goal to about 1e-14 m. MultiPlan's own
  sampler ends up to 4.4e-5 m from my end point. My end point matches the goal and
  theirs does not, so the gap is their small-step integration, though I did not test
  that by changing their step.
- Curvature jumps from 0 to +-1/r at the end of an arc, as in Task 9, and a real
  steering wheel cannot do that. The paths ignore obstacles and steering rate.
- r = 5 m is a placeholder. The car's limit is 3.9466 m, kappa 0.2534. These paths
  use kappa 0.2.
