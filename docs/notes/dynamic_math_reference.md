# Dynamic bicycle model: math reference

Code: src/trackloop/models/dynamic.py. Tests: tests/test_dynamic.py.

## 1. Assumptions

- Same two-wheel geometry as the kinematic model, but the reference point is the center of gravity (CG). lf and lr are the distances from the CG to the front and rear axles, and L = lf + lr.
- Tires make lateral force in proportion to slip angle (linear tires): Fy = C x alpha, with cornering stiffness C per axle.
- Longitudinal speed vx is set by the acceleration input. The small vy x r coupling in the longitudinal equation is dropped, and tire force reduction from cornering is ignored.
- Flat ground, rigid body, no load transfer, no aerodynamic force.

## 2. State and inputs

State: x, y (CG position, m), psi (heading, rad), vx, vy (body-frame velocity, m/s), r (yaw rate, rad/s), delta (front steering angle, rad).
Inputs: steering command and body-x acceleration a.

## 3. Equations

Slip angles (angle between where a wheel points and where it moves):

    alpha_f = delta - atan((vy + lf r) / vx)
    alpha_r =       - atan((vy - lr r) / vx)

Tire forces and body dynamics:

    Fyf = Cf alpha_f           Fyr = Cr alpha_r
    m (vy_dot + vx r) = Fyf cos(delta) + Fyr
    Iz r_dot          = lf Fyf cos(delta) - lr Fyr
    vx_dot = a

Pose:

    x_dot   = vx cos(psi) - vy sin(psi)
    y_dot   = vx sin(psi) + vy cos(psi)
    psi_dot = r

The term vx r on the left of the lateral equation is the centripetal acceleration seen from the rotating body frame. It is why vy_dot is not simply force divided by mass.

## 4. Steady-state derivation (the understeer formula)

In a steady turn, vy_dot = 0, r_dot = 0, and with small angles cos(delta) = 1.

Moment balance gives lf Fyf = lr Fyr.
Force balance gives Fyf + Fyr = m vx r = m ay, where ay = vx r is the lateral acceleration.

Solving the two:

    Fyf = m ay lr / L        Fyr = m ay lf / L

Each axle carries lateral force in proportion to its share of the weight. Slip angles then follow from the linear tire law:

    alpha_f = m ay lr / (L Cf)        alpha_r = m ay lf / (L Cr)

Subtract the two slip angle definitions (the vy terms cancel):

    alpha_f - alpha_r = delta - L r / vx

so

    delta = L r / vx + alpha_f - alpha_r
          = L / R + K ay          with R = vx / r

    K = (m / L)(lr / Cf - lf / Cr)

K is the understeer gradient. K > 0 means the car needs more steering as lateral acceleration grows (understeer). With ay = vx r, solve for r:

    r = vx delta / (L + K vx^2)

The kinematic model is the K = 0 case, with the small difference that it uses tan(delta). The speed sqrt(L / K) is where the yaw rate gain vx / (L + K vx^2) peaks.

## 5. Worked example with real project numbers

Parameters: m = 1500 kg, Iz = 2500 kg m^2, lf = 1.2 m, lr = 1.5 m, Cf = 80000 N/rad, Cr = 100000 N/rad. L = 2.7 m.

    K = (1500 / 2.7)(1.5 / 80000 - 1.2 / 100000)
      = 555.56 x (1.875e-5 - 1.2e-5)
      = 0.00375 rad per (m/s^2)
    characteristic speed = sqrt(2.7 / 0.00375) = 26.83 m/s

Steady turn at vx = 10 m/s, delta = 0.02 rad:

    r = 10 x 0.02 / (2.7 + 0.00375 x 100) = 0.2 / 3.075 = 0.065041 rad/s
    ay = vx r = 0.65041 m/s^2
    Fyf = 1500 x 0.65041 x 1.5 / 2.7 = 542.0 N
    Fyr = 1500 x 0.65041 x 1.2 / 2.7 = 433.6 N
    alpha_f = 542.0 / 80000  = 0.006775 rad
    alpha_r = 433.6 / 100000 = 0.004336 rad
    delta check: L r / vx + alpha_f - alpha_r = 0.017561 + 0.002439 = 0.020000 rad

Simulation, 10 s at dt = 0.01 s, starting at vx = 10 m/s with zero yaw rate:

    vx (m/s)   simulated r   formula r   kinematic r
    10         0.065039      0.065041    0.074084
    20         0.095222      0.095238    0.148168
    30         0.098746      0.098765    0.222252

The simulation sits slightly below the formula, by 0.003% at 10 m/s and 0.019% at 30 m/s, because the code keeps cos(delta) and atan, which the formula drops. At 20 m/s the real model turns at 0.643 of the kinematic rate.

## 6. Low-speed guard

Slip angles divide by vx and are undefined at zero speed. The code uses max(vx, v_min) with v_min = 1 m/s inside the slip angle terms only. This keeps the integration finite but is not physics. The model is not trusted near or below v_min. Task 4 measures the speed range where this model and the kinematic model agree.

## 7. What these tests do and do not show

They show the code solves the stated equations correctly, by matching an independent closed-form result and a mirror symmetry check. They do not show the equations match a real car. The parameters are placeholders, and linear tires stop being accurate at high lateral acceleration, where real tires saturate.

## 8. Parameters

mass 1500 kg, inertia 2500 kg m^2, lf 1.2 m, lr 1.5 m, Cf 80000 N/rad, Cr 100000 N/rad, steering limit 0.6 rad, steering rate limit 0.8 rad/s, v_min 1 m/s. All placeholders, not a specific car.
