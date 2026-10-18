"""Print the matched-turn comparison table for the kinematic and dynamic models."""
import numpy as np

from trackloop.models.comparison import matched_turn
from trackloop.models.dynamic import DynamicParams

DP = DynamicParams()
L = DP.wheelbase
K = DP.understeer_gradient


def main():
    print("kinematic lateral accel target 2.0 m/s^2, turn length 5.0 s, dt 0.01 s")
    print(" v(m/s) delta(rad)  ratio  formula  dpsi(rad)  err(m)  beta(deg)")
    for v in (3, 4, 5, 6, 8, 10, 15, 20, 25, 30):
        o = matched_turn(float(v))
        if o is None:
            print(f"{v:6.0f}  steering limit exceeded")
            continue
        print(f"{v:6.0f}  {o['delta']:9.4f}  {o['ratio']:.4f}  {o['formula']:.4f}"
              f"  {o['dpsi']:9.4f}  {o['err']:6.3f}  {o['beta']:8.3f}")

    speeds = np.arange(3.0, 30.01, 0.1)
    ratios = np.array([matched_turn(v)["ratio"] for v in speeds])
    v_sim = np.interp(0.95, ratios[::-1], speeds[::-1])
    v_formula = np.sqrt(L * (1 / 0.95 - 1) / K)
    v_zero_beta = np.sqrt(DP.lr * L * DP.cr / (DP.mass * DP.lf))
    print(f"speed where ratio falls to 0.95: simulated {v_sim:.2f} m/s, formula {v_formula:.2f} m/s")
    print(f"zero-sideslip speed from formula: {v_zero_beta:.2f} m/s")


if __name__ == "__main__":
    main()
