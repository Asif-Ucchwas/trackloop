"""LQR and MPC recovering from a lateral offset with the real steering rate limit.

Straight path, 10 m/s, offset to the left, dt = 0.05 s, 12 s. The plant keeps the
default limits (0.6 rad, 0.8 rad/s). The MPC knows them; the LQR does not.
"""
import numpy as np

from trackloop import paths
from trackloop.controllers.lqr import LQRController
from trackloop.controllers.mpc import MPCController
from trackloop.models.dynamic import DynamicParams
from trackloop.simulate_dynamic import simulate_dynamic

DT, T_END, BAND = 0.05, 12.0, 0.05


def run(controller, path, offset):
    state0 = [0.0, offset, 0.0, 10.0, 0.0, 0.0, 0.0]
    traj, cte = simulate_dynamic(controller, path, state0, DT, T_END,
                                 params=DynamicParams())
    outside = np.nonzero(np.abs(cte) > BAND)[0]
    if len(outside) == 0:
        settle = 0.0
    elif outside[-1] == len(cte) - 1:
        settle = None
    else:
        settle = (outside[-1] + 1) * DT
    return settle, max(0.0, -cte.min()), np.abs(traj[:, 6]).max()


def main():
    path = paths.straight(300.0)
    print("offset  controller  settle (s)  overshoot (m)  max steer (rad)  solve ms")
    for offset in (0.5, 2.0, 5.0, 8.0):
        for name, controller in (("LQR", LQRController(path)),
                                 ("MPC", MPCController(path, dt=DT, horizon=20))):
            settle, over, steer = run(controller, path, offset)
            times = getattr(controller, "solve_times", None)
            ms = "-"
            if times:
                ms = f"{1e3 * np.mean(times):.2f}/{1e3 * np.max(times):.2f}"
            text = "not settled" if settle is None else f"{settle:.2f}"
            print(f"{offset:5.1f}   {name:5s}  {text:>11s}  {over:12.3f}  "
                  f"{steer:15.3f}  {ms}")
    print("solve ms = mean/max of one QP update and solve (this machine, not a target)")


if __name__ == "__main__":
    main()
