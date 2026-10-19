# Cusp tracking: math reference (Task 11)

Companion to `rs_export_math_reference.md`. It covers tracking a path that has cusps with a
kinematic rear-axle car model: how the path is split, what changes in reverse for each of
the four controllers, how the runner stops at a cusp, and what the first results do and do
not show. Predictions are marked "derived"; things seen in runs are marked "measured".
Code pointers are in section 11.

## 1. Conventions

- Wheelbase L = 2.7 m, steering limit 0.6 rad, steering rate limit 0.8 rad/s. All three are
  placeholders until a real car is chosen.
- State [x, y, psi, v, delta]. Gear g = +1 forward, -1 reverse, and v = g |v|.
- Path samples carry the body heading h (not the travel direction), the steering curvature
  kappa (positive left, the same sign in both gears), the arc length s and the gear g.
- Error state: e = rear-axle offset, positive to the left of the path heading h, and
  psi_e = psi - h wrapped to (-pi, pi]. The Stanley code stores h - psi, which is -psi_e.

## 2. Splitting at cusps

A cusp is a sample i with direction[i] != direction[i-1]. A path with k cusps splits into
k + 1 segments that share their cusp sample, so each segment is tracked up to the cusp.
The cusp sample is the first sample of the new gear and carries the new piece's curvature,
so the segment that ends there takes kappa[i-1] instead: the vehicle should arrive with the
old piece's steering, not the next one's.

Worked example, `parallel_park` (radius 5 m, word `C|CuCu|C`, 8.0023 m long):

- cusps at s = 0.2940 m and s = 7.7083 m;
- segment lengths 0.2940, 7.4144 and 0.2940 m, which add up to 8.0023 m within rounding;
- the short pieces are L 0.0588 rad x 5 m = 0.2940 m, and the middle is two arcs of
  0.7414 rad x 5 m = 3.707 m each, driven in reverse.

## 3. Error model per metre driven

Vehicle: dpsi/dt = v tan(delta) / L. With ds = |v| dt the distance driven,

    dpsi/ds = g tan(delta) / L.

The path heading obeys dh/ds = g kappa, because the exact arc formulas of the
export note have h = h0 + g kappa t. The offset changes with the component of the motion
across the path: de/dt = v sin(psi_e), so de/ds = g sin(psi_e). Together, with u = tan(delta):

    de/ds     = g sin(psi_e)
    dpsi_e/ds = g (u / L - kappa)

Linearised at zero error and written with w = u - L kappa:

    dx/ds = g A x + g B w,   A = [[0, 1], [0, 0]],   B = [0, 1/L]^T.

Zero error with w = 0 is the steady turn on the path circle. Example: kappa = 0.2 (radius
5 m) needs u = L kappa = 0.54, so delta = atan(0.54) = 0.4951 rad (28.4 deg).

Speed does not appear. One model per gear works at any speed down to a standstill. This is
why the repo's earlier LQR and MPC, built on the dynamic bicycle model (it divides by the
forward speed, minimum 2 m/s), cannot be used at cusp speeds, and why kinematic versions
were written.

## 4. Pure Pursuit in reverse

Forward law: the circle through the rear axle, tangent to the travel direction, that
reaches a goal point at distance ld has curvature 2 sin(beta) / ld, where beta is the angle
from the travel direction to the goal line. The code uses alpha, the angle from the body
heading axis to the goal line, and delta = atan(2 L sin(alpha) / ld).

Forward, beta = alpha. In reverse the travel direction is the body heading plus pi, so
beta = alpha - pi and sin(alpha) = -sin(beta). The yaw rate sign also flips: the travel
direction turns at -tan(delta) / L per metre. Setting that equal to 2 sin(beta) / ld gives

    tan(delta) = -2 L sin(beta) / ld = 2 L sin(alpha) / ld,

which is the forward formula unchanged. Pure Pursuit needs no reverse branch as long as the
goal point is chosen in travel order, which a single-gear segment guarantees.

Worked example: ld = 2 + 0.5 x 1.5 = 2.75 m. Forward with alpha = 0.3 rad:
delta = atan(2 x 2.7 x sin 0.3 / 2.75) = atan(0.5803) = 0.5258 rad. Reverse with the goal
0.3 rad to the left of the travel direction (beta = 0.3, alpha = pi + 0.3): the same code
gives delta = -0.5258 rad. The goal is on the left of the direction of travel, and steering
the other way is what turns a backward-moving car that way.

## 5. Stanley in reverse

Forward law (code): delta = -psi_e - atan(k e / (v + k_soft)), with e measured at the
front axle, which leads the motion.

Why it diverges in reverse (derived). With v = -|v|, the heading rate is
dpsi/dt = -|v| delta / L. The forward heading term gives delta = -psi_e, so
dpsi_e/dt = +(|v| / L) psi_e. That is positive feedback with time constant L / |v|
= 1.8 s at 1.5 m/s. In addition the forward cross-track term divides by v + k_soft
= -0.5, which flips its sign. The reverse leads with the rear axle, which steering reaches
only through the heading rate. Measured: the forward law applied in reverse gives a maximum
error above 5 m in `test_reverse_tracking.py`.

Reverse law (code): rear-axle reference (reach 0) and

    delta = k_h (psi_e - atan(k e / (|v| + k_soft))).

Then dpsi_e/dt = -(k_h |v| / L)(psi_e - psi*), with psi* = atan(k e / (|v| + k_soft)): the
heading error relaxes to psi*, and for e > 0 a positive psi_e makes de/dt = -|v| sin(psi_e)
negative, so the offset shrinks.

Linearised about a straight path at |v| = 1.5 m/s, k = 1.5, k_soft = 1, k_h = 1:

    d/dt [e, psi_e] = [[0, -1.5], [0.5, -0.5556]] [e, psi_e]
    lambda^2 + 0.5556 lambda + 0.5 = 0,   lambda = -0.278 +- 0.650 j,

stable and underdamped (damping ratio 0.39), with a 3.6 s time constant for the envelope.
The inner heading loop (rate k_h |v| / L = 0.556 per second) is slower than the outer
offset loop would be on its own (0.9 per second), so the two are not time-scale separated;
do not read 0.9 per second as the convergence rate.

Worked example: e = 0.3 m, psi_e = 0.1 rad, |v| = 1.5. Reverse law:
delta = 0.1 - atan(1.5 x 0.3 / 2.5) = 0.1 - 0.1781 = -0.0781 rad. The forward law on the
same state gives -2.509 rad, which the 0.6 rad limit clips.

Feedforward adds atan(L kappa). Steady turn needs u = L kappa in both gears (section 3), so
the same term serves both. On an arc of kappa = 0.2 it is 0.4951 rad.

## 6. Kinematic LQR in distance

Cost J = integral of (x^T Q x + r w^2) ds, with Bryson weights from the largest errors
accepted (0.5 m, 0.1 rad, u = 0.2): Q = diag(1/0.5^2, 1/0.1^2) = diag(4, 100), r = 1/0.2^2 = 25.

Forward gear, b = 1/L. Write P = [[p11, p12], [p12, p22]]. The Riccati equation
A^T P + P A - P B r^-1 B^T P + Q = 0 gives entry by entry:

    (1,1):  q1 - (b^2 / r) p12^2 = 0            ->  p12 = sqrt(q1 r) / b
    (2,2):  2 p12 + q2 - (b^2 / r) p22^2 = 0    ->  p22 = sqrt(r (2 p12 + q2)) / b

and K = r^-1 B^T P = (b / r) [p12, p22], so

    k_e   = sqrt(q1 / r)
    k_psi = sqrt((2 sqrt(q1 r) / b + q2) / r).

Numbers: k_e = sqrt(4 / 25) = 0.4 per metre. k_psi = sqrt((2 x 10 x 2.7 + 100) / 25)
= sqrt(154 / 25) = 2.4819, which equals the solver's gain. Closed loop per metre:
lambda^2 + (k_psi / L) lambda + k_e / L = lambda^2 + 0.9192 lambda + 0.1481 = 0, so
lambda = -0.2084 and -0.7108 per metre (e-folding distances 4.8 m and 1.4 m).

Reverse gear. With z = [e, -psi_e] = T x, T = diag(1, -1), the reverse model dx/ds = -A x - B w
becomes dz/ds = A z + B w, because T A T^-1 = -A and T B = [0, -b]^T. So z obeys the forward
model, its optimal law is w = -K z = -[k_e, -k_psi] x, and the cost is unchanged because Q
is diagonal. Reverse gain: [0.4, -2.4819], same closed-loop poles.

Command: delta = atan(L kappa - K x), clipped to +-0.6 rad. Worked example, e = 0.3 m,
psi_e = 0.1 rad, kappa = 0.2: forward u = 0.54 - (0.4 x 0.3 + 2.4819 x 0.1) = 0.1718, so
delta = 0.1701 rad. Reverse u = 0.54 - (0.12 - 0.2482) = 0.6682, so delta = 0.5891 rad.

The law assumes the steering angle equals the command at once. The plant has a rate limit
(section 9 shows what that does).

## 7. Kinematic MPC

The time-domain version of section 3 at a fixed design speed |v| = 1.5 m/s, sampled at
dt = 0.1 s with a zero-order hold on w:

    x+ = [[1, v dt], [0, 1]] x + [[v^2 dt^2 / (2 L)], [v dt / L]] w,   v = g |v|.

Horizon N = 40 (4 s, 6 m). Cost: sum of x^T Q x + r w^2 with the discrete Riccati solution
as the terminal weight. Constraints: |u| <= tan(0.6), |change of u per step| <= 0.8 x 0.1,
and the first step also limited by the steering angle the car has now. The change limit is
on u = tan(delta), not on delta; since tan is steeper than the identity, it is slightly
conservative. The QP is built once per gear and only its vector and bounds are updated
(OSQP). Curvature enters as a shift L kappa of the bounds, taken at the current projection
and held over the whole horizon: there is no curvature preview.

Differences from the LQR, so the two are not expected to be equal: discrete time at a fixed
design speed (the real speed ramps to and from zero in the runner), weights per sample
instead of per metre, and the constraints. With the constraints inactive and the Riccati
terminal weight, the solution is the infinite-horizon discrete LQR (standard result, not
re-derived here).

## 8. The runner

Per segment: build a controller, let the steering settle toward its command at standstill
(until |command - delta| < 0.02 rad, timeout 5 s), drive in the segment's gear, brake to a
standstill at the cusp, record the cusp errors. The next segment starts from that pose.

Speed profile with remaining arc length r on the segment, cruise 1.5 m/s, a = 1 m/s^2:

    v_ref = max(min(1.5, sqrt(2 a r)), 0.2),   change limited to a dt per step.

Constant deceleration gives v^2 = 2 a r, so the cruise speed is reached at r = 1.125 m and
the braking profile governs the last 1.125 m. At the arrival threshold r = 0.03 m the profile
is sqrt(0.06) = 0.245 m/s, above the 0.2 m/s floor, and the arrival test ends the approach
before the floor can bind; the floor only matters below r = 0.02 m. Braking then continues
at a from about 0.245 m/s. Estimated stop distance past the threshold: 0.245^2 / 2 = 0.03 m
plus one step of motion, so about 0.06 m past the end of the segment. Measured on the
straight `ahead` path: 0.074 m. The remaining 0.014 m is not explained; one candidate is
the 0.1 m sample spacing of the arc-length projection, not tested.

## 9. First results and what they show

Setup for every number here: nine Reeds-Shepp paths (radius 5 m), four controllers, start
exactly on the path, kinematic model with the steering rate limit, cruise 1.5 m/s, no noise,
one run per pair. Exploratory; this is not the Stage 4 benchmark. Full table:
`data/results/cusp_tracking.csv`. The four paths with cusps and real reverse driving:

    path           controller      max cte  worst cusp  end pos
    three_point    Pure Pursuit    0.135    0.130       0.144
    three_point    Stanley         0.669    0.553       0.526
    three_point    Kinematic LQR   0.024    0.082       0.077
    three_point    Kinematic MPC   0.025    0.083       0.078
    parallel_park  Pure Pursuit    0.319    0.085       0.045
    parallel_park  Stanley         0.991    0.992       0.935
    parallel_park  Kinematic LQR   1.007    1.007       0.964
    parallel_park  Kinematic MPC   1.007    1.008       0.964
    side_slot      Pure Pursuit    0.320    0.110       0.151
    side_slot      Stanley         0.495    0.428       0.487
    side_slot      Kinematic LQR   1.102    1.102       0.987
    side_slot      Kinematic MPC   1.103    1.103       0.992
    reverse_in     Pure Pursuit    0.201    0.139       0.137
    reverse_in     Stanley         0.208    0.090       0.206
    reverse_in     Kinematic LQR   0.189    0.082       0.102
    reverse_in     Kinematic MPC   0.193    0.081       0.104

All values in metres.

Measured:

1. All 36 runs finish, and every cusp is reached, stopped at and left in the other gear.
2. LQR and MPC agree to within 0.002 m rms on every path. MPC's constraints and rate
   knowledge do not help here; plausibly because it cannot see the curvature step coming
   (no preview), which was not tested.
3. The large errors on `parallel_park` and `side_slot` come from the path. Both flip the
   steering curvature instantly inside one gear. Steering rate limit test, kinematic LQR,
   max cross-track error in metres (`parallel_park` / `side_slot`):

       0.8 rad/s, 1.5 m/s (the runs above)   1.007 / 1.102
       0.8 rad/s, 0.5 m/s                    0.197 / 0.184
       5.0 rad/s, 1.5 m/s                    0.056 / 0.056
       0.8 rad/s, 1.5 m/s, no feedforward    0.571 / 0.705

   Scale of the effect (derived): the flip from kappa = -0.2 to +0.2 needs a steering swing
   of 2 atan(0.54) = 0.990 rad, which takes 1.24 s at 0.8 rad/s, that is 1.86 m of driving
   at 1.5 m/s, 0.62 m at 0.5 m/s, and 0.30 m with a 5 rad/s actuator. The error ordering
   follows that distance. This is not a fit, and only the speed and rate were varied.
4. At the rate limit, feedforward made it worse, not better (1.007 vs 0.571, 1.102 vs
   0.705), though both stay far above the 0.056 of a fast actuator. The cause was not
   investigated.
5. Pure Pursuit stays near 0.32 m on those two paths. Its lookahead smooths the step
   (the figure shows it leaving the reference between the two arcs) rather than tracking it.
6. Stanley is the weakest on forward arcs: 0.643 m max on `diag` (no cusps) and a large
   first-segment deviation on `three_point`. Not diagnosed. It runs without feedforward on
   forward segments and with it on reverse segments; forward with feedforward looked worse
   in an earlier probe and was not investigated.
7. On `three_point` the kinematic LQR and MPC are the best (rms 0.010 m).

## 10. Limits

- One start condition per run, exactly on the path (the unit tests use offsets of 0.5 to
  1 m); no noise, no model mismatch, no delay. The model used to control is the model that is simulated.
- The Reeds-Shepp paths have curvature steps, so they are not steering-rate feasible at
  1.5 m/s. Task 12's curvature check should test steering-rate feasibility, not only the
  curvature limit.
- Radius, cell size and car parameters are placeholders.
- The stop overshoots about 0.07 m along the path; the cusp position error includes it.
- MPC has a fixed design speed and no preview. LQR has no preview either.
- Kappa jumps at arc ends in the exported paths.
- The dynamic-model LQR and MPC are not used at cusps.

## 11. Metric definitions and code pointers

Metrics that exist only in code (`scripts/track_cusp_paths.py`, `cusp_runner.py`):

- cte: `cross_track_error(segment path, x, y)` of the rear axle against the active segment,
  evaluated at the nearest path sample (0.1 m spacing), positive left of the path heading.
- rms_cte and max_cte: root mean square and maximum of |cte| over the samples where
  |v| > 0.05 m/s, so the standstill settling samples are excluded.
- end_pos_err: distance from the rear axle, at standstill, to the last sample of the last
  segment.
- worst_cusp_pos_err: largest distance from the rear axle at standstill to the last sample
  of a segment, over all cusps. The heading error is recorded in the run result and not
  tabulated.
- worst_settle: largest steering settling time before a segment that follows a cusp. The
  settling before the first segment is not recorded.
- time_s: simulated time at the end of the run, including settling and braking.

Code: `segments.py` (split), `cusp_runner.py` (runner), `controllers/stanley.py` (reverse
branch), `controllers/kinematic_lqr.py`, `controllers/kinematic_mpc.py`,
`scripts/track_cusp_paths.py` (table and figure), tests in `tests/test_segments.py`,
`test_reverse_tracking.py`, `test_kinematic_lqr.py`, `test_kinematic_mpc.py`,
`test_cusp_runner.py`.
