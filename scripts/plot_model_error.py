"""Figure for the model-error cases (Task 15): paired RMS lateral error against the nominal plant."""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import NullLocator

rows = list(csv.DictReader(open("data/results/model_error_summary.csv")))
for r in rows:
    r["speed"], r["ratio"] = float(r["speed"]), float(r["ratio"])
    r["runs"], r["finished"] = int(float(r["runs"])), int(float(r["finished"]))

TITLE = {"mass_p20": "mass +20%", "mass_m20": "mass -20%",
         "stiff_p30": "tire stiffness +30%", "stiff_m30": "tire stiffness -30%",
         "lag_0.1": "steering lag 0.1 s", "lag_0.2": "steering lag 0.2 s",
         "combined": "combined: mass +20%,\nstiffness -30%, lag 0.1 s",
         "wheelbase_p5": "wheelbase +5%", "wheelbase_m5": "wheelbase -5%"}
COL = {"Pure Pursuit": "tab:orange", "Stanley": "tab:green", "Dynamic LQR": "tab:blue",
       "Dynamic MPC": "tab:purple", "Kinematic LQR": "tab:blue", "Kinematic MPC": "tab:purple"}


def fam_rows(fam):
    return [r for r in rows if r["family"] == fam]


def order(fam, key):
    return list(dict.fromkeys(r[key] for r in fam_rows(fam)))


def draw(ax, fam, case, ylim):
    speeds = sorted({r["speed"] for r in fam_rows(fam)})
    for c in order(fam, "controller"):
        pts = sorted((r["speed"], r["ratio"], r["runs"] - r["finished"]) for r in fam_rows(fam)
                     if r["controller"] == c and r["case"] == case)
        xs = [speeds.index(p[0]) for p in pts]
        ax.plot(xs, [p[1] for p in pts], "o-", color=COL[c], ms=4, lw=1.5)
        for x, p in zip(xs, pts):
            if p[2]:
                ax.plot([x], [p[1]], "x", color="red", ms=12, mew=2.4)
    ax.axhline(1.0, color="k", lw=1)
    ax.set_yscale("log")
    ax.set_ylim(*ylim)
    ticks = [t for t in (0.5, 0.7, 1, 1.5, 2, 3, 5, 8) if ylim[0] <= t <= ylim[1]]
    ax.set_yticks(ticks, [f"{t:g}" for t in ticks])
    ax.yaxis.set_minor_locator(NullLocator())
    ax.set_xticks(range(len(speeds)), [f"{v:g}" for v in speeds])
    ax.set_xlim(-0.25, len(speeds) - 0.75)
    ax.set_title(TITLE[case], fontsize=9)
    ax.set_xlabel("design speed (m/s)", fontsize=8)
    ax.grid(axis="y", alpha=0.3)


def ylim_of(fam):
    v = [r["ratio"] for r in fam_rows(fam) if r["ratio"] == r["ratio"]]
    return min(v) / 1.15, max(v) * 1.15


fig, axes = plt.subplots(3, 4, figsize=(15, 10.8))
road_cases = [k for k in order("road", "case") if k != "nominal"]
park_cases = [k for k in order("parking", "case") if k != "nominal"]
for ax, case in zip(axes[:2].ravel(), road_cases):
    draw(ax, "road", case, ylim_of("road"))
for ax, case in zip(axes[2], park_cases):
    draw(ax, "parking", case, ylim_of("parking"))
for ax in axes[:2, 0]:
    ax.set_ylabel("road family\nRMS / nominal RMS", fontsize=9)
axes[2, 0].set_ylabel("parking family\nRMS / nominal RMS", fontsize=9)

leg = axes[1, 3]
leg.axis("off")
road_h = [Line2D([], [], color=COL[c], marker="o", ms=4, label=c) for c in order("road", "controller")]
park_h = [Line2D([], [], color=COL[c], marker="o", ms=4, label=c) for c in order("parking", "controller")]
x_h = Line2D([], [], color="red", marker="x", ls="", ms=11, mew=2.4, label="some runs did not finish\n(ratio uses finished pairs)")
l1 = leg.legend(handles=road_h, title="road family", loc="upper left", bbox_to_anchor=(-0.05, 1.0), fontsize=8)
leg.add_artist(l1)
l2 = leg.legend(handles=park_h, title="parking family", loc="upper left", bbox_to_anchor=(0.5, 1.0), fontsize=8)
leg.add_artist(l2)
leg.legend(handles=[x_h], loc="lower left", fontsize=8)

fig.suptitle("Model error: plant differs from the model the controllers were designed on "
             "(median paired RMS lateral error / nominal; 1 = no change)", fontsize=11)
fig.text(0.5, 0.005, "Case sizes are placeholders, not measured values. Controllers always use nominal parameters; "
         "only the simulated plant changes. Log y-axis.", ha="center", fontsize=8)
fig.tight_layout(rect=(0, 0.02, 1, 0.97))
Path("docs/figures").mkdir(parents=True, exist_ok=True)
fig.savefig("docs/figures/model_error.png", dpi=130)
print("wrote docs/figures/model_error.png")
