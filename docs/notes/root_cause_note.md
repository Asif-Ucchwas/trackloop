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

## Not yet investigated

- Why `road_curve_v20` seed 2 is the worst start for all four controllers.
- Why Pure Pursuit beats the kinematic LQR and MPC in parking at 1.5 and 3 m/s.
- Why 5 Dynamic MPC runs did not finish at 20 m/s with 0.2 s lag.
- Whether Pure Pursuit and Stanley are tuned fairly at 20 m/s.
- Whether the softer-tire cases push more runs past the tire friction limit.
- Why lag slightly helps Pure Pursuit and Stanley; why the combined model-error case improves
  Stanley, LQR and MPC at 5 m/s.
- Why the dynamic LQR's mean (0.045 m) exceeds its median (0.034 m) at 20 m/s.
