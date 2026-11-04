# Root causes of the worst runs (Task 16)

What was investigated, what the evidence shows, and what is still open. Every number is from a
script in `scripts/` (named in each section), run on commit `587d83f` and rerun on a second machine,
where every table matched except one column of the trace table (command direction changes under lag:
2.44 against 2.51 per second; not investigated). Each section states what was measured and what was not.

## 1. Kinematic MPC and steering lag (parking family)

**Observation.** In Task 15, Kinematic MPC at 3 m/s with 0.2 s steering lag had a median paired
error ratio of 6.96 against its nominal run; Kinematic LQR was at 1.00. Across the 19 parking
paths (`scripts/diagnose_mpc_lag.py`, 3 m/s, seeds 0 to 4) the MPC ratio is above 2 on 17 paths and
near 1 on two (`rs_three_point` 1.04, `rs_ahead` 1.01); the LQR ratio stays between 0.79 and 1.27.
On the worst paths the MPC's median RMS goes from 0.1 to 0.3 m to 2 to 3.5 m.

**One run traced** (`scripts/trace_lag_run.py`, `thesis_theta_star_tight_seed2`, seed 0). With lag
the MPC's steering command changes direction about 5 times as often (0.45 to 2.5 per second), the
steering is rate-limited 68 percent of the time instead of 16, and the run takes 22 s longer. The
LQR barely changes. The MPC's command stays close to the measured steering angle in both cases
(RMS difference 0.03 and 0.06 rad), while the LQR's differs from it by 0.14 rad in both.

**Test.** In `kinematic_mpc.py` the first step of the MPC's steering-rate bound is measured from the
actual steering angle. With an actuator lag, that angle trails the command. The test
(`scripts/test_mpc_anchor.py`) wraps the MPC so that the angle it sees is its own previous command
(the controller source is not changed). Median over the 19 paths:

| | nominal plant | 0.2 s lag | lag / nominal (median of per-path ratios) |
|---|---|---|---|
| standard MPC | 0.190 m | 1.757 m | 6.85 |
| MPC anchored to its previous command | 0.220 m | 0.228 m | 1.10 |

**What this supports.** The lag damage follows the measured-angle anchor: removing it brings the
ratio from 6.85 to 1.10. The plausible mechanism is that the bound lets each new command differ by
only one rate step from an angle that has only partly responded, which slows the effective steering.
That mechanism is not shown separately.

**Limits.** Parking family, 3 m/s, 5 seeds only. The road MPC (which has its own failures under lag
at 20 m/s) was not examined. The anchored MPC is not a free improvement: nominal error rises on two
paths (`rs_u_turn` 0.122 to 0.451 m, `thesis_theta_star_tight_seed3` 0.345 to 0.451 m), and I have not
looked at why. A real controller would not know how its command acted on the plant, so this explains
the failure and is not a design proposal. The ratio 6.85 is a median of per-path ratios and differs
from the 6.96 of Task 15, which is a median over every path and seed.

## 2. Stanley's chatter at 5 m/s (road family)

**Observation.** In Task 14, Stanley's steering-rate reversals were 1.67 per second at 5 m/s against
0.2 to 0.3 for the other cells.

**Evidence** (`scripts/scan_stanley_chatter.py`, `scan_stanley_speed.py`, `test_stanley_spacing.py`,
`test_spacing_all_roads.py`):
- The chatter does not follow the cross-track gain (at 3 m/s: 3.21, 3.16 and 3.30 reversals per
  second for k of 0.75, 1.5 and 3).
- With the benchmark path (points every 0.1 m) it depends on how many path points one 0.05 s step
  covers (speed / 2): it peaks at half-integer counts (3.16 at 1.5 points per step, 1.67 at 2.5, 1.41
  at 3.5) and drops to 0.26 to 0.40 at whole numbers (1, 2, 3, 4 points).
- With the controller's path resampled to 0.05 m, chatter at 3, 4 and 5 m/s becomes 0.25, 0.20 and
  0.23; at 0.02 m, 0.19, 0.19 and 0.25. RMS lateral error does not change (0.118, 0.107, 0.092 m).
- On all 12 road paths at their design speeds, Stanley's chatter at 5 m/s falls from 1.67 to 0.23
  with 0.05 m spacing; at 10 and 20 m/s it is unchanged (0.22, 0.19). Median RMS is unchanged to
  four decimals for Stanley and for Pure Pursuit at every design speed, and Pure Pursuit's chatter
  does not change either.

**What this supports.** The 5 m/s chatter in the benchmark comes from sampling the path every 0.1 m,
not from Stanley's tuning, and it does not affect the RMS numbers in `BENCHMARKS.md`. Stanley
selects the nearest path point; the discrete jump of that choice is the likely mechanism, but I did not
isolate it from the other uses of the path.

**Limits.** The 10 and 20 m/s benchmark speeds cover 5 and 10 whole path points per step, so they
sit at the quiet points of the pattern by coincidence; the effect is not absent there in general. The
scan was run on the four 5 m/s road paths. LQR and MPC use a projection with its own handling and
were not tested. Other chatter values in earlier notes were not rechecked.

## 3. The worst start: `road_curve_v20`, seed 2 (road family)

**Observation.** In Task 14, seed 2 on `road_curve_v20` was the worst start for all four controllers
(offset 0.237 m, heading error +0.071 rad, both on the side leaving the path). Peak rear-axle
errors in `scripts/trace_worst_start.py`: Pure Pursuit 0.774 m, Stanley 0.644, dynamic LQR 0.616,
dynamic MPC 0.630, all reached 0.35 to 0.6 s after the start.

**Evidence.** Seed 2's offset and heading error applied one at a time (largest error in the first 3 s):

| e0 (m) | dpsi (rad) | Pure Pursuit | Stanley | Dynamic LQR | Dynamic MPC |
|---|---|---|---|---|---|
| 0.237 | 0 | 0.237 | 0.237 | 0.237 | 0.237 |
| 0 | 0.071 | 0.603 | 0.427 | 0.380 | 0.404 |
| 0.237 | 0.071 | 0.774 | 0.644 | 0.616 | 0.630 |
| 0.237 | -0.071 | 0.465 | 0.237 | 0.237 | 0.237 |

On the other four seeds the LQR's and MPC's largest error is the starting offset itself (0.10 to
0.29 m at t = 0). Pure Pursuit's (0.41 to 0.42 m) and Stanley's (0.31 m) largest errors on those
seeds occur 6 s or more into the run, on the curve, which is a different effect.

**What this supports.** The heading error, not the offset, drives the peak: the offset alone never
grows past its starting value, while 0.071 rad alone gives 0.38 to 0.60 m. At 20 m/s that heading
error is a sideways velocity of about 1.4 m/s that the steering has to turn around. The two effects
add for the LQR (0.380 + 0.237 = 0.617 against 0.616 measured) but not exactly for Pure Pursuit
(0.603 + 0.237 = 0.84 against 0.774). With the heading error pointing back toward the path, Stanley,
the LQR and the MPC stay at the starting offset; Pure Pursuit still reaches 0.465 m.

**Limits.** One path, one speed. Why a heading error of 0.071 rad needs about 0.4 s and 0.4 to
0.6 m to be corrected is not shown (the 0.8 rad/s steering-rate limit and the rate at which the
tires build force are candidates, neither tested). Seeds 0 to 4 only. The start range of 0.1 rad
for heading error was reused from parking without checking it against 20 m/s; this result is
consistent with it being a hard start there.

## 4. Dynamic MPC divergence at 20 m/s with 0.2 s lag (road family)

**Observation.** In Task 15, 5 of the 20 runs of Dynamic MPC at 20 m/s with 0.2 s steering lag did
not finish: `road_curve_v20` seed 2, `road_double_shift_v20` seed 1, and `road_s_curve_v20` seeds 0,
2 and 4. All five ended as diverged (error beyond 25 m), with peak front slip angles of 13.7 to 17.9
degrees, far above the 4.68 degrees at which the linear tire force reaches the friction limit.

**Test.** The road MPC bounds its first steering step by one rate step from the measured steering
angle (`state[6]`), the same structure as section 1. `scripts/test_road_mpc_anchor.py` replaces that
angle with the MPC's own previous command (controller source unchanged) on the four 20 m/s road
paths, seeds 0 to 4:

| MPC | plant | runs diverged (of 20) | median RMS of finished runs (m) |
|---|---|---|---|
| standard | nominal | 0 | 0.0374 |
| standard | lag 0.1 s | 0 | 0.0378 |
| standard | lag 0.2 s | 5 | 0.0382 |
| anchored | nominal | 0 | 0.0374 |
| anchored | lag 0.1 s | 0 | 0.0378 |
| anchored | lag 0.2 s | 0 | 0.0381 |

**What this supports.** All five divergences follow the measured-angle anchor, as in the parking
MPC. The divergence is a tail event: the median RMS of the finished runs hardly changes with lag,
so the Task 15 median ratio of 1.14 for this cell, computed without these five runs, understates
the damage.

**Limits.** I did not trace why a rate bound anchored to a lagging angle ends in divergence in
the dynamic model, so the mechanism is not shown. Only the 20 m/s paths and lags up to 0.2 s were
tested; the MPC's larger lag sensitivity at 10 m/s (median ratios 1.10 and 1.25 in Task 15) was not
examined. The anchored MPC is a diagnostic, not a proposed design.

## 5. Runs beyond the tire friction limit (road family)

**Question.** The plant keeps linear tires, so a run whose slip angle exceeds the angle at which
the tire force would reach mu * Fz is outside what a real tire could do. Task 15 guessed that the
softer-tire cases would push more runs past it. `scripts/tire_limit_exposure.py` counts them from
`data/results/model_error_road.csv` (peak front and rear slip per run), using the friction-limit angle
of each case: mu * Fz / C per axle, which depends on mass (load) and stiffness. Front limit in degrees:
4.68 nominal, 5.62 mass +20%, 3.75 mass -20%, 3.60 stiffness +30%, 6.69 stiffness -30%, 8.03 combined
(rear: 3.00, 3.60, 2.40, 2.31, 4.28, 5.14); the lag cases leave it at nominal.

**Evidence.** Runs over the limit (front or rear), out of 20 per cell. Every cell at 5 and 10 m/s is 0.
At 20 m/s:

| controller | nominal | mass +20% | mass -20% | stiff +30% | stiff -30% | lag 0.1 | lag 0.2 | combined |
|---|---|---|---|---|---|---|---|---|
| Pure Pursuit | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| Stanley | 0 | 0 | 2 | 2 | 0 | 0 | 0 | 0 |
| Dynamic LQR | 10 | 5 | 8 | 8 | 6 | 7 | 6 | 4 |
| Dynamic MPC | 5 | 2 | 6 | 6 | 1 | 1 | 5 | 0 |

**What this supports.** The guess was wrong. Lower stiffness raises the angle at which the tire
saturates, so fewer runs exceed it (LQR 10 to 6, MPC 5 to 1 with stiffness -30%), and the combined
case, which has the largest error ratios in Task 15 (2.61 and 2.76), has the fewest (4 and 0). The
exposure is almost entirely the dynamic LQR and MPC at 20 m/s; Pure Pursuit never exceeds the limit.

**Limits.** Each run counts once if its single largest slip angle is above the limit, however briefly.
The plant keeps linear tires, so nothing here says what a real tire would do past the limit, and the
20 m/s LQR and MPC numbers stay partly outside the model's validity. Five seeds; the counts are out
of 20 and carry no interval.

## Not yet investigated

- Why Pure Pursuit beats the kinematic LQR and MPC in parking at 1.5 and 3 m/s.
- Whether Pure Pursuit and Stanley are tuned fairly at 20 m/s.
- Why lag slightly helps Pure Pursuit and Stanley; why the combined model-error case improves
  Stanley, LQR and MPC at 5 m/s.
- Why the dynamic LQR's mean (0.045 m) exceeds its median (0.034 m) at 20 m/s.
