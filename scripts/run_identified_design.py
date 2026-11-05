"""Task 24: re-test the controllers designed on a model identified from simulated noisy data, against the Stage 4 results.

For each Stage 4 dynamic case (mass, stiffness, steering lag, and the combination) the plant is the true vehicle. A steering-sweep
experiment on it is fitted (cf, cr, Iz, nominal mass assumed). The LQR and MPC are designed on the identified model and on the
true plant's own parameters (oracle) and run on the true plant on the 12 road paths with 5 seeds. The nominal-design runs are the
committed Stage 4 results (data/results/model_error_road.csv). Pure Pursuit and Stanley use no vehicle parameters, so they are not
re-run. Writes data/results/identified_models.csv, identified_design_runs.csv, identified_design.csv and
docs/figures/identified_design.png. SIMULATED identification data.
"""
import csv
import os
import time
from concurrent.futures import ProcessPoolExecutor

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from trackloop import identified_design as idz
from trackloop.model_error import DYNAMIC_CASES, dynamic_plant

CASES = [c for c in DYNAMIC_CASES if c != "nominal"]
SPEEDS = (5.0, 10.0, 20.0)
CONTROLLERS = ("Pure Pursuit", "Stanley", "Dynamic LQR", "Dynamic MPC")


def write_rows(path, rows):
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def main():
    t0 = time.time()
    print("Identified models (nominal mass assumed). Dynamics error = error of cf/m, cr/m, Iz/m against the plant's, percent")
    print(f"{'case':10s} {'cf':>9s} {'cr':>9s} {'Iz':>7s}   chi2   dynamics error % (cf/m, cr/m, Iz/m)")
    models = []
    for case in ["nominal"] + CASES:
        plant, (model, fit) = dynamic_plant(case), idz.identified(case)
        err = 100 * (np.array([model.cf, model.cr, model.inertia]) / model.mass
                     / (np.array([plant.cf, plant.cr, plant.inertia]) / plant.mass) - 1)
        print(f"{case:10s} {model.cf:9.0f} {model.cr:9.0f} {model.inertia:7.0f}   {fit['chi2_dof']:.2f}   {np.round(err, 2).tolist()}")
        models.append({"case": case, "plant_mass": plant.mass, "plant_cf": plant.cf, "plant_cr": plant.cr, "plant_inertia": plant.inertia,
                       "identified_cf": round(model.cf, 1), "identified_cr": round(model.cr, 1), "identified_inertia": round(model.inertia, 1),
                       "chi2_dof": round(fit["chi2_dof"], 3), "dynamics_error_pct_cf": round(err[0], 3),
                       "dynamics_error_pct_cr": round(err[1], 3), "dynamics_error_pct_inertia": round(err[2], 3)})
    write_rows("data/results/identified_models.csv", models)

    tasks = idz.all_tasks(CASES)
    with ProcessPoolExecutor(os.cpu_count() or 1) as pool:
        runs = list(pool.map(idz.run_one, tasks, chunksize=8))
    write_rows("data/results/identified_design_runs.csv", runs)
    errors = sum(str(r["status"]).startswith("error") for r in runs)
    print(f"\n{len(runs)} new runs, {errors} errors, {sum(not r['finished'] for r in runs)} unfinished")

    stage4 = idz.load_stage4()
    baseline = [r for r in stage4 if r["case"] == "nominal"]
    rows = []
    for c in CONTROLLERS:
        for v in SPEEDS:
            for case in CASES:
                sel = lambda rs: [r for r in rs if r["controller"] == c and r["speed"] == v and r["case"] == case]
                wrong = sel(stage4)
                if c in idz.MODEL_BASED:
                    by = {"nominal": wrong, "identified": [r for r in sel(runs) if r["variant"] == "identified"],
                          "oracle": [r for r in sel(runs) if r["variant"] == "oracle"]}
                else:
                    by = {"nominal": wrong, "identified": wrong, "oracle": wrong}
                for variant, rs in by.items():
                    done = [r for r in rs if r["finished"]]
                    rows.append({"controller": c, "speed": v, "case": case, "variant": variant, "runs": len(rs), "finished": len(done),
                                 "median_rms": round(float(np.median([r["rms_lateral"] for r in done])), 5) if done else "",
                                 "rms_ratio_vs_nominal_plant": round(idz.paired_ratio(rs, [r for r in baseline if r["controller"] == c and r["speed"] == v]), 4),
                                 "rms_ratio_vs_oracle": round(idz.paired_ratio(rs, by["oracle"]), 4),
                                 "effort_ratio_vs_oracle": round(idz.paired_ratio(rs, by["oracle"], "steering_effort"), 4)})
    write_rows("data/results/identified_design.csv", rows)

    print("\nMedian paired RMS lateral error, as a ratio to the same run on the nominal plant with the nominal design")
    print("(wrong model = Stage 4 result | identified model | oracle = true parameters)")
    for c in idz.MODEL_BASED:
        for v in SPEEDS:
            print(f"\n{c}, {v:g} m/s")
            for case in CASES:
                g = {r["variant"]: r for r in rows if r["controller"] == c and r["speed"] == v and r["case"] == case}
                print(f"  {case:10s} wrong {g['nominal']['rms_ratio_vs_nominal_plant']:5.2f}   identified {g['identified']['rms_ratio_vs_nominal_plant']:5.2f}"
                      f"   oracle {g['oracle']['rms_ratio_vs_nominal_plant']:5.2f}   | wrong/oracle {g['nominal']['rms_ratio_vs_oracle']:5.3f}"
                      f"  identified/oracle {g['identified']['rms_ratio_vs_oracle']:6.4f}   steering effort wrong/oracle {g['nominal']['effort_ratio_vs_oracle']:5.3f}")
    worst = max(abs(r["rms_ratio_vs_oracle"] - 1) for r in rows if r["variant"] == "identified")
    index = {(r["path"], r["controller"], r["seed"], r["case"]): r["rms_lateral"] for r in runs if r["variant"] == "oracle" and r["finished"]}
    per_run = [abs(r["rms_lateral"] / index[(r["path"], r["controller"], r["seed"], r["case"])] - 1) for r in runs
               if r["variant"] == "identified" and r["finished"] and (r["path"], r["controller"], r["seed"], r["case"]) in index]
    print(f"\nIdentified against oracle: worst median ratio differs from 1 by {worst:.4f}; worst single run {max(per_run):.4f} ({len(per_run)} paired runs)")

    fig, ax = plt.subplots(2, 3, figsize=(16, 8), sharey="row")
    x = np.arange(len(CASES))
    for i, c in enumerate(idz.MODEL_BASED):
        for j, v in enumerate(SPEEDS):
            for k, (variant, label) in enumerate((("nominal", "nominal design (Stage 4)"), ("identified", "identified model"), ("oracle", "true parameters"))):
                vals = [next(r["rms_ratio_vs_nominal_plant"] for r in rows if r["controller"] == c and r["speed"] == v
                             and r["case"] == case and r["variant"] == variant) for case in CASES]
                ax[i, j].bar(x + 0.27 * (k - 1), vals, 0.27, label=label)
            ax[i, j].axhline(1, color="k", lw=0.8)
            ax[i, j].set(xticks=x, xticklabels=CASES, title=f"{c}, {v:g} m/s")
            ax[i, j].tick_params(axis="x", rotation=40)
            ax[i, j].grid(alpha=0.3, axis="y")
        ax[i, 0].set_ylabel("RMS lateral error / nominal plant, nominal design")
    ax[0, 0].legend(fontsize=8)
    fig.suptitle("Controllers designed on the identified model against the Stage 4 results (simulated identification data)")
    fig.tight_layout()
    fig.savefig("docs/figures/identified_design.png", dpi=140)
    print(f"\nwrote data/results/identified_models.csv, identified_design_runs.csv, identified_design.csv and docs/figures/identified_design.png ({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
