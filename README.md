# TrackLoop

Path-tracking controllers on a vehicle model, with stability analysis and system identification.

Status: in progress.

## Current results

Stages 1 and 2 are done: kinematic and dynamic bicycle models, and four tracking
controllers (Pure Pursuit, Stanley, scheduled LQR with curvature feedforward, and
linear MPC solved with OSQP). Benchmarks, stability analysis and system
identification are still to come.

![LQR and MPC recovering from a 5 m lateral offset](docs/figures/lqr_vs_mpc_recovery.png)

Recovery from a 5 m offset at 10 m/s on a straight path, with the plant limited to
0.6 rad steering and 0.8 rad/s steering rate. LQR is designed without the rate
limit and overshoots; MPC includes it and settles without overshoot. A 5 m offset
is far outside the small-error range LQR is linearized for, and the vehicle
parameters are placeholders, so this shows a limit of unconstrained LQR, not that
MPC is better in general. At 0.5 m and 2 m offsets the two settle at nearly the
same time (1.85 s vs 1.90 s, and 2.55 s vs 2.90 s).

Reproduce with `python scripts/plot_recovery.py`.

## Paths from my planners

Controllers are tested on paths from two planners, stored in one format: position, heading, curvature, arc length and a forward/reverse flag, sampled every 0.1 m.

**Thesis planner (Hybrid A*-RRT*, and Theta* with tightening).** The thesis code returns grid waypoints only, with no heading or curvature, so each corner is rounded with a circular arc (3 m per cell, 6 m radius). Ten paths: two planners on five random maps, 195 to 208 m long. The figure shows three of the five maps.

![Thesis planner paths](docs/figures/thesis_paths.png)

**Reeds-Shepp (from MultiPlan).** Nine paths from hand-chosen start and goal poses at a 5 m turning radius. Four of them reverse at least once, seven cusps in all. Blue is forward, orange is reverse, red crosses mark the cusps.

![Reeds-Shepp paths](docs/figures/rs_paths.png)

Limits:

- The rounded thesis paths are not collision-free: 9 of the 10 clip an obstacle when checked every 0.5 m (the arc cuts the corner by 0.21 to 1.13 m). They are reference paths for tracking tests, not planner results.
- Curvature jumps at the ends of arcs, and two paths on map 3 need curvature 0.294 and 0.279 against the placeholder car's limit of 0.253.
- Cell size, corner radius and car parameters are placeholders. Notes: `docs/notes/planner_export_math_reference.md` and `docs/notes/rs_export_math_reference.md`.

Reproduce: `python scripts/export_thesis_paths.py` (needs the thesis repo at `~/projects/hybrid-a-star-rrtstar-path-planning`), `python scripts/export_rs_paths.py` (needs MultiPlan at `~/projects/multiplan`), then `python scripts/plot_thesis_paths.py` and `python scripts/plot_rs_paths.py`.
