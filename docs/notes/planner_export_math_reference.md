# Planner export: math reference

Code: src/trackloop/planner_export.py, scripts/export_thesis_paths.py,
scripts/plot_thesis_paths.py. Tests: tests/test_planner_export.py.

## 1. Assumptions

- A planner path is a polyline of waypoints (x, y) in grid cells. Cell (x, y) is the
  unit square centred on (x, y) in the thesis code. One cell is `cell` metres
  (3 m here), so a waypoint is (3x, 3y) m.
- The vehicle drives the polyline forward only, so the direction flag is +1.
- Every interior vertex is replaced by a circular arc of radius r (6 m here) tangent
  to both legs. Everything else stays a straight line.
- Placeholder car: wheelbase 2.7 m, steering limit 0.6 rad.

## 2. Rounding one corner

Let u1 be the unit vector of the leg into the vertex P and u2 the unit vector of the
leg out of it. The turn angle and direction are

    theta = atan2(|u1 x u2|, u1 . u2)      sigma = sign(u1 x u2)

with sigma = +1 for a left turn and -1 for a right turn.

A circle of radius r tangent to both legs has its centre C on the bisector, at
distance r from each leg. The two tangent points A (first leg) and B (second leg) are
the same distance T from P, because the tangents from one point to a circle are equal.
The angle between the two legs at P is pi - theta, so the bisector makes an angle
(pi - theta) / 2 with each leg. Triangle P-A-C has a right angle at A (a radius is
perpendicular to its tangent), so

    tan((pi - theta) / 2) = r / T   =>   T = r tan(theta / 2)
    |PC| = r / sin((pi - theta) / 2) = r / cos(theta / 2)

The arc runs from A = P - T u1 to B = P + T u2 and turns the heading through exactly
theta, so its length is r theta.

Position on the arc. With n(h) = (-sin h, cos h) the left normal of heading h:

    C = A + sigma r n(h0)          h(s) = h0 + sigma s / r
    position(s) = C - sigma r n(h(s))        curvature = sigma / r

At s = 0 this gives C - sigma r n(h0) = A. Heading is kept continuous, not wrapped,
the same as the existing paths in paths.py.

## 3. When the arc does not fit

A leg between two corners is shared. Each corner may use at most half of each
neighbouring leg:

    T <= T_max = (1/2) min(L1, L2)

If r tan(theta / 2) > T_max the corner gets the smaller radius
r' = T_max / tan(theta / 2), and it is counted as tight. Two neighbouring arcs then
use T_i + T_{i+1} <= L_i, so they never overlap. The rule is conservative: a corner
next to a long leg could use more than half of it, and this code does not.

A smaller radius means a sharper turn. The car cannot drive a curvature above

    R_min = L / tan(delta_max) = 2.7 / tan(0.6) = 3.9466 m
    kappa_max = 1 / R_min = 0.2534 1/m

(kinematic bicycle at the steering limit, placeholder parameters).

## 4. Sampling and length

The pieces are lines and arcs with known start arc lengths. Samples are taken at
s = 0, ds, 2 ds, ... and at the total length. For each sample the code finds its piece
and evaluates the formulas above, so heading and curvature are exact, not finite
differences. Curvature is 0 on lines and sigma / r on arcs.

Rounded length. Each corner replaces two tangent pieces (total 2T) by an arc r theta:

    length = sum of leg lengths - sum over corners (2 T_i - r_i theta_i)

Since tan x > x for x > 0, T = r tan(theta / 2) > r theta / 2, so 2T > r theta: rounding
always shortens the path.

## 5. Worked examples (3 m per cell, r = 6 m, exported paths)

Example 1, theta_star_tight on map 2, first corner. Waypoints in cells (2, 2),
(7, 13), (13.674, 15.459), so in metres (6, 6), (21, 39), (41.021, 46.377).

    L1 = 36.249 m      L2 = 21.337 m
    h1 = 65.556 deg    h2 = 20.227 deg     right turn, theta = 45.329 deg
    T = 6 tan(22.665 deg) = 2.5055 m       T_max = 10.6685 m, so it fits
    arc length = 6 x 0.79114 = 4.7468 m    curvature = -1/6 = -0.1667
    cut = 6 (1 / cos(22.665 deg) - 1) = 0.5021 m

Length check on the same path (7 corners): leg sum 204.9752 m, sum of (2T - r theta)
0.9887 m, so 203.9866 m, which equals the exported length_m.

Example 2, hybrid on map 3, vertex 17 (a corner that does not fit).

    L1 = 6.000 m      L2 = 3.060 m      theta = 48.398 deg
    wanted T = 6 tan(24.199 deg) = 2.696 m   but T_max = 1.530 m
    r' = 1.530 / tan(24.199 deg) = 3.4051 m  (below R_min = 3.9466 m)
    curvature = 0.2937 1/m, above kappa_max = 0.2534 1/m
    cut = 3.4051 (1 / cos(24.199 deg) - 1) = 0.3280 m

## 6. Quantities computed only in code

- max_cut_m: the largest distance from a vertex to its arc. The vertex is |PC| from the
  centre and the arc is r from it, so cut = r (1 / cos(theta / 2) - 1), largest over
  corners (r is the radius used, after any reduction).
- min_radius_m: the smallest r used over all corners (none if there are no corners).
- n_tight: the number of corners that got a radius below the requested r.
- max_curvature: the largest |curvature| over the samples, 1 / min_radius_m.
- waypoint_hits and rounded_hits: the number of consecutive point pairs whose segment
  fails the thesis exact collision rule (obstacle cell inflated by 0.05 cell, touching
  counts as a hit). For the waypoints every pair is tested. For the rounded path every
  fifth sample (0.5 m) is converted back to cells and the pairs are tested. One clip can
  fail more than one pair, so the count is pairs, not obstacles.

## 7. Limits of these findings

- The rounded paths are not collision-free. In the 10 exported paths the thesis
  waypoints never touch an obstacle, but 9 of the 10 rounded paths do, at the 0.5 m
  sampling. The cut is 0.21 to 1.13 m while the planner's margin is 0.05 cell = 0.15 m.
  For tracking tests this does not matter, because the controllers have no obstacles.
  They are reference paths, not planner results.
- Curvature jumps from 0 to +-1/r at the ends of an arc (no clothoid). A real steering
  rate cannot follow a jump. This is left to the curvature check of Task 12.
- Map 3 needs curvature 0.294 (hybrid) and 0.279 (theta_star_tight), above the car's
  0.253. Those paths are exported unchanged and flagged, not repaired.
- 10 paths: two planners, five random maps of density 0.20, a fixed start (2, 2) and
  goal (47, 47). The thesis Hybrid planner uses Python's global random generator, so
  every export seeds it and saves the seed. The same table came out on Python 3.13 and
  3.14.4.
- The cell size (3 m), the radius (6 m) and the car parameters are choices and
  placeholders, not properties of the planners.
