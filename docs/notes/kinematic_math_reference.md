# Kinematic bicycle model: math reference

Code: src/trackloop/models/kinematic.py. Tests: tests/test_kinematic.py.

## 1. Assumptions

- Two wheels: a fixed rear wheel and a steered front wheel, wheelbase L.
- The reference point is the center of the rear axle.
- No tire slip: each wheel moves only along the direction it points.
- Flat ground, rigid body.

These hold at low speed and low lateral acceleration. Section 8 says where they stop holding.

## 2. Equations

State: x, y (position, m), psi (heading, rad), v (speed, m/s), delta (steering angle, rad).
Inputs: steering command delta_cmd (rad) and acceleration a (m/s^2).

    x_dot   = v cos(psi)
    y_dot   = v sin(psi)
    psi_dot = (v / L) tan(delta)
    v_dot   = a

## 3. Derivation of the heading rate

With no slip, the rear wheel moves along the heading, so its velocity is perpendicular to the rear axle line. The front wheel moves along its own direction, which is the heading rotated by delta. Both velocity directions must be perpendicular to the line from one common point, the instantaneous center of rotation (ICR), to the wheel.

The ICR therefore lies on the rear axle line. Call its distance from the rear axle center R. The line from the ICR to the front wheel makes an angle delta with the line from the ICR to the rear axle center, so the right triangle with sides L (wheelbase) and R gives:

    tan(delta) = L / R        so        R = L / tan(delta)

The rear axle center moves at speed v on a circle of radius R, so its heading turns at

    psi_dot = v / R = (v / L) tan(delta)

A negative v (reverse driving) makes psi_dot negative for the same delta, with no change to the equations.

## 4. Worked example: constant steering on a circle

Given L = 2.7 m, delta = 0.3 rad, v = 5 m/s, start at the origin with heading 0.

    tan(0.3)           = 0.309336
    R = L / tan(delta) = 2.7 / 0.309336 = 8.7284 m
    omega = v / R      = 5 / 8.7284 = 0.57284 rad/s
    lateral accel      = v^2 / R = 25 / 8.7284 = 2.8642 m/s^2

The exact path with the center at (0, R) is

    x(t) = R sin(omega t)
    y(t) = R (1 - cos(omega t))
    psi(t) = omega t

At t = 10 s the code gives x = -4.59740, y = 1.30891, heading = 5.72845 rad, identical to the exact values to five decimals. The largest position error over the 10 s run, at dt = 0.01 s, is 6.581e-12 m. This is small because constant steering gives a constant-curvature arc, where RK4 is nearly exact. It is not an accuracy claim for arbitrary steering.

## 5. Worked examples in the reverse direction

Given a desired result, find the input.

Required steering for a 20 m radius turn:

    delta = atan(L / R) = atan(2.7 / 20) = 0.13419 rad = 7.69 degrees

Smallest radius the car can turn, at the steering limit of 0.6 rad:

    R_min = L / tan(0.6) = 2.7 / 0.684137 = 3.9466 m

Speed at which that tightest turn reaches a lateral acceleration of 3 m/s^2:

    v = sqrt(a_lat R_min) = sqrt(3 x 3.9466) = 3.44 m/s

Heading rate in reverse, v = -2 m/s with delta = 0.2 rad:

    psi_dot = (-2 / 2.7) tan(0.2) = -0.150156 rad/s

so the heading decreases while the car backs up with the wheels turned left.

## 6. Steering actuator

The steering angle is a state. Each step it moves toward the command by at most max_steer_rate x dt, then is clipped to plus or minus max_steer.

    delta_next = clip( delta + clip(delta_cmd - delta, -r dt, r dt), -d_max, d_max )

Worked example: with r = 0.8 rad/s and d_max = 0.6 rad, going from 0 to the limit takes 0.6 / 0.8 = 0.75 s, which is 75 steps at dt = 0.01 s.

## 7. Integration

Pose and speed are advanced with fourth-order Runge-Kutta. The steering angle is held at its new value for the whole step (zero-order hold), so a steering change inside a step is applied at the start of that step.

## 8. Where the model stops being valid

The no-slip assumption fails as lateral acceleration grows. In the example above, v^2 / R = 2.86 m/s^2 already approaches the range where tires slip on dry pavement. The valid speed range is measured in Task 4 by comparing this model against the dynamic model, and is not assumed here.

## 9. Parameters

wheelbase 2.7 m, steering limit 0.6 rad and steering rate limit 0.8 rad/s are placeholders, not a specific car.
