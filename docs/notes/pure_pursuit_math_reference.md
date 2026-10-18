# Pure Pursuit: math reference

Code: src/trackloop/controllers/pure_pursuit.py (controller),
src/trackloop/corner_cut.py (corner measurement), src/trackloop/simulate.py,
src/trackloop/paths.py. Tests: tests/test_pure_pursuit.py, tests/test_corner_cut.py.
Script: scripts/pure_pursuit_corner.py.

## 1. Assumptions

- Rear-axle kinematic model (see kinematic_math_reference.md), forward driving only.
- The path is a list of points 0.1 m apart. The goal point is found by linear
  interpolation between two neighboring points.
- Sections 4 and 5 use small angles. Section 3 does not.

## 2. Control law

Pick a goal point G on the path at distance ld from the rear axle. Let alpha be the
angle between the vehicle heading and the line from the rear axle to G.

The circle that is tangent to the heading at the rear axle and passes through G has
a chord of length ld that makes angle alpha with its tangent. By the tangent-chord
relation the chord equals 2 Rc sin(alpha), so the curvature is

    kappa = 1 / Rc = 2 sin(alpha) / ld

and the kinematic model needs tan(delta) = L kappa, so

    delta = atan(2 L sin(alpha) / ld)
    ld    = clip(ld_min + k_v |v|, ld_min, ld_max)

Worked example: wheelbase L = 2.7 m, vehicle at (3, -1) heading along +x, path along
the x axis, ld = 5 m. The goal is at (3 + sqrt(24), 0) = (7.899, 0). The distance to
it is sqrt(24 + 1) = 5, and the line to it rises 1 m over a chord of 5 m, so
sin(alpha) = 0.2.

    delta = atan(2 * 2.7 * 0.2 / 5) = atan(0.216) = 0.21273 rad

The vehicle is right of the path, so the steering is to the left (positive).

## 3. Circle: zero steady-state error

Put the path on a circle of radius R and the vehicle on a concentric circle of
radius r. Place the rear axle at P = (r, 0) heading +y, and the goal at
G = (R cos(theta), R sin(theta)). Then

    ld^2 = R^2 + r^2 - 2 R r cos(theta)
    sin(alpha) = (r - R cos(theta)) / ld

A steady turn on radius r needs kappa = 1 / r, which is 2 sin(alpha) / ld = 1 / r:

    2 r (r - R cos(theta)) = ld^2 = R^2 + r^2 - 2 R r cos(theta)
    r^2 = R^2

so r = R for every lookahead. In the kinematic model Pure Pursuit has no steady
error on a circle. Worked example: R = 20 m needs delta = atan(2.7 / 20) = 0.13419 rad.
Starting on the circle with that steering, the cross-track error stays near 1e-4 m
for ld = 3, 5 and 10 m. That residual comes from the 0.1 m polyline path, not from
the controller (check script below).

## 4. Straight-line response

Use distance x = v t along the path, a path along the x axis, lateral offset y and
heading psi = dy/dx. The goal point has lateral position 0, so for small angles

    alpha = -y / ld - psi
    d(psi)/dx = 2 alpha / ld
    y'' + (2 / ld) y' + (2 / ld^2) y = 0         (derivatives with respect to x)

In time this is y'' + (2 v / ld) y' + (2 v^2 / ld^2) y = 0, a second-order system with

    wn   = sqrt(2) v / ld
    zeta = 1 / sqrt(2) = 0.7071

The damping ratio does not depend on speed or lookahead, so the overshoot is always
exp(-pi zeta / sqrt(1 - zeta^2)) = exp(-pi) = 4.32%. The natural frequency grows with
v / ld, which is why ld has to grow with speed (steering lag is not in this model,
so that limit is stated as reasoning and is not tested here).

Worked example: v = 5 m/s, ld = 5 m, initial offset y0 = 0.1 m, initial heading 0.
Then wn = sqrt(2), zeta wn = 1, wd = 1, so

    y(t) = y0 exp(-t) (cos t + sin t)
    y(1) = 0.05083 m     y(2) = 0.00667 m     y(pi) = -0.00432 m

The simulation (steering rate limit removed, dt = 0.01 s) gives a minimum of
-0.0043216 m, against -0.0043214 m from the formula.

The simulated curve differs from the formula by up to 3.4e-4 m at dt = 0.01 s. The
gap halves each time dt halves (6.9e-4, 3.4e-4, 1.7e-4, 8.6e-5 m at dt = 0.02, 0.01,
0.005, 0.0025 s). That first-order scaling matches the controller holding each
command for one step, an average delay of dt / 2. The test checks the scaling.

## 5. Corner cutting

Let the path be straight for x < 0 and a circle for x >= 0. For small angles the
circle is the parabola yp = kappa x^2 / 2 with kappa = 1 / R. The goal point is on
the curve as soon as x + ld > 0, so the vehicle sees the curve ld before it arrives:

    y'' + (2 / ld) y' + (2 / ld^2) y = (2 / ld^2) yp(x + ld)
    yp(x + ld) = kappa (x + ld)^2 / 2

for x >= -ld, with y = y' = 0 at x = -ld. One solution is y = yp (the path itself).
The homogeneous solutions are exp(-s) cos(s) and exp(-s) sin(s) with s = (x + ld) / ld.
Matching y = y' = 0 at s = 0 gives, for the cross-track error e = y - yp when x >= 0,

    e = kappa ld^2 * (1/2) exp(-s) (sin(s) - cos(s))

The peak is where the derivative exp(-s) cos(s) is zero, at s = pi / 2:

    position  x = (pi / 2 - 1) ld = 0.5708 ld past the start of the curve
    size      e = kappa ld^2 * (1/2) exp(-pi / 2) = 0.10394 ld^2 / R

The next extremum is at s = 3 pi / 2, an outward overshoot of
-(1/2) exp(-3 pi / 2) = -0.004492 ld^2 / R at x = 3.712 ld.

The size is proportional to ld^2, inversely proportional to R, and independent of
speed, because the equation in x has no v in it.

Worked example: R = 40 m, ld = 4 m.

    peak     = 0.10394 * 16 / 40 = 0.04158 m, at 0.5708 * 4 = 2.283 m past the corner
    outward  = -0.004492 * 16 / 40 = -0.0018 m

The simulation gives 0.0411 m (1.2% lower).

Measured peak * R / ld^2 over R = 20, 40, 80 m and ld = 2, 4, 6, 8 m at 5 m/s: 0.1018
to 0.1046 against 0.10394. It rises with ld / R (0.1046 at ld / R = 0.4), as expected
when the small-angle assumption weakens. The shortfall at small ld is consistent with
the one-step hold delay: at R = 40 and ld = 4 the value is 0.1033, 0.1028 and 0.1014
at v = 3, 5 and 10 m/s. Measured positions are 0.561 to 0.575 ld, and measured outward
overshoots are -0.0042 to -0.0051 ld^2 / R.

Design consequence: with ld = k v, the cut is 0.10394 k^2 v^2 / R = 0.10394 k^2 a,
where a is the lateral acceleration. With k = 0.5 s and a = 2 m/s^2 this is 5.2 cm.
This follows from the formula and was not simulated separately. It needs small
angles and a small ld_min.

## 6. Quantities computed only in code (corner_cut.measure)

- peak: the largest cross-track error over the run. Positive means left of the path,
  which is the inside of a left curve.
- where: the vehicle x position at the peak minus the corner position (30 m).
- dip: the smallest cross-track error from the peak to the end of the run.
- The path has 25 + 6 ld metres of curve and the run stops 3 ld before its end, so
  the goal point never reaches the end of the path.

## 7. Limits of these findings

- Kinematic model only. The dynamic model needs more steering for the same turn, so
  these results do not carry over to it. That comparison belongs to the benchmark.
- Small angles in sections 4 and 5. The corner formula was checked up to ld / R = 0.4.
- Forward driving only. Reverse driving is handled with the cusp work later.
- The steering rate limit (0.8 rad/s) made no visible difference to the corner cut
  at R = 40 m, ld = 2, 4 and 6 m.
- Parameters are placeholders: wheelbase 2.7 m. The defaults ld_min = 2 m, k_v = 0.5 s
  and ld_max = 30 m have not been tuned. The tests use a fixed lookahead (k_v = 0).
