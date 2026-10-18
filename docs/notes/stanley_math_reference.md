# Stanley: math reference

Code: src/trackloop/controllers/stanley.py, src/trackloop/simulate.py (ref_offset),
src/trackloop/paths.py. Tests: tests/test_stanley.py.

## 1. Assumptions

- Rear-axle kinematic model (see kinematic_math_reference.md), forward driving only.
- The control law references the front axle, which is one wheelbase L ahead of the
  rear axle along the heading.
- The path is a list of points 0.1 m apart. The nearest point is the nearest list
  point, not a projection onto the segment between two points.
- Section 3 uses small angles and removes the steering rate limit. Section 4 does not
  use small angles and keeps the default 0.8 rad/s limit.

## 2. Control law

Let e be the front axle cross-track error (positive left of the path), and psi_e the
path heading minus the vehicle heading at the path point nearest the front axle.

    delta = psi_e - atan2(k e, v + k_soft)        clipped to +/- max_steer

The first term lines the wheels up with the path. The second turns toward the path,
harder for a larger error and less hard at higher speed. The softening k_soft keeps
the second term finite as v goes to 0: with k_soft = 0 and v = 0 the term is
+/- pi / 2 (the clip then limits it), and with e = 0 it is 0. The value k_soft is a
tuning choice, not derived.

Worked example 1 (zero speed): rear axle at (3, 0.1), heading 0, path along the x
axis, v = 0, k = 1.5, k_soft = 1. The front axle is at (5.7, 0.1), so e = 0.1 and
psi_e = 0.

    delta = -atan(1.5 * 0.1 / (0 + 1.0)) = -atan(0.15) = -0.14889 rad

Worked example 2 (heading term alone): heading psi = 0.1 rad, with the rear axle at
y = -L sin(0.1) so the front axle is exactly on the path. Then e = 0 and

    delta = psi_e = 0 - 0.1 = -0.1 rad

## 3. Straight path: exponential convergence

For the kinematic model the front axle point moves at speed v / cos(delta) in the
direction psi + delta. This follows from adding the rotation term to the rear axle
velocity: v along the heading, plus L * (v tan(delta) / L) = v tan(delta) across it.

Take a path along the x axis, so that e is the front axle y position. The control law
makes the direction of the front axle velocity, relative to the path,

    psi + delta - 0 = psi + psi_e - atan(k e / (v + k_soft)) = -atan(k e / (v + k_soft))

because psi + psi_e = path heading = 0. So, exactly (with instant steering),

    de/dt = -(v / cos(delta)) sin(atan(k e / (v + k_soft)))

For small e and delta this becomes de/dt = -k (v / (v + k_soft)) e, and

    e(t) = e0 exp(-k v t / (v + k_soft))

This is first order: no overshoot. With k_soft = 0 the rate is k, whatever the speed.

Worked example: k = 1, k_soft = 0, e0 = 0.1 m (front axle offset), steering rate limit
removed.

    e(1) = 0.03679 m     e(2) = 0.01353 m     e(3) = 0.00498 m

Simulated at v = 5 m/s and dt = 0.005 s: 0.03665, 0.01348, 0.00497. The largest gap to
the formula over 4 s is 3.3e-4 m at dt = 0.01 s and 1.6e-4 m at dt = 0.005 s. It halves
with dt, which matches the controller holding each command for one step (an average
delay of dt / 2), as for Pure Pursuit. The simulated error stays positive and
decreases at every step.

With softening, k = 1, k_soft = 1, v = 5 m/s the rate is 5 / 6 = 0.8333 per second:
e(2) = 0.1 exp(-1.6667) = 0.01889 m. Simulated: 0.01883 m.

## 4. Circle: the rear axle runs inside the path

Let the path be a circle of radius R and the vehicle follow a concentric circle with
its rear axle at radius r. A kinematic steady turn needs tan(delta) = L / r. The front
axle is at radius sqrt(r^2 + L^2) = r / cos(delta), and the path tangent at the front
axle is rotated by delta from the heading. So psi_e = delta, and the control law

    delta = psi_e - atan(k e / (v + k_soft))

holds only when the atan term is 0, that is e = 0. The front axle sits on the path:

    sqrt(r^2 + L^2) = R        r = sqrt(R^2 - L^2)

The rear axle is inside the path by R - sqrt(R^2 - L^2), roughly L^2 / (2 R), and the
steady steering follows from sin(delta) = L / sqrt(r^2 + L^2) = L / R.

Worked example: R = 20 m, L = 2.7 m.

    r = sqrt(400 - 7.29) = 19.8169 m
    rear axle inside by 20 - 19.8169 = 0.18309 m   (L^2 / 2R gives 0.18225)
    delta = asin(2.7 / 20) = 0.13541 rad           (atan(2.7 / 20) = 0.13419)

Simulation at k = 1, k_soft = 1, v = 5 m/s over two laps (last 5 s): rear axle offset
0.18304 m on average, front axle error near 0. With 0.02 m path spacing the average
steering is 0.13541 rad and the offset 0.18308 m. With the default 0.1 m spacing the
steering varies by about 0.005 rad peak to peak, from the nearest-point lookup.

For R = 10 m the offset is 0.3714 m. It is a steady offset caused by referencing the
front axle, so it grows as the circle tightens. Pure Pursuit has none on a circle.

## 5. Quantities computed only in code

- simulate(..., ref_offset=a): the cross-track error is measured at the point a metres
  ahead of the rear axle along the heading. a = 0 is the rear axle, a = L the front.
- Stanley.cross_track and Stanley.heading_error store the last e and psi_e the
  controller used.

## 6. Limits of these findings

- Kinematic model only. The dynamic model needs more steering for the same turn, and
  these results do not carry over to it.
- The exponential result (section 3) assumes instant steering and small angles. It
  was tested with the rate limit removed. A 0.8 rad/s limit was not tested there.
- Corner entry (a straight into a curve) is not measured here. It comes with the
  benchmark paths.
- The gains k = 1 to 1.5 and k_soft = 1 are placeholders, not tuned.
- Forward driving only. Reverse driving is handled with the cusp work later.
- The nearest-point lookup on a 0.1 m path gives steering jitter on curves (section 4).
  Projecting onto the segment would remove most of it; that change is not made.
- Wheelbase 2.7 m is a placeholder.
