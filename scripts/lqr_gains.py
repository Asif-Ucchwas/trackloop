"""Print the LQR gains, Riccati check and closed-loop poles at several speeds."""
import numpy as np

from trackloop.controllers.lqr import GainSchedule, bryson_weights, lqr_gain


def main():
    q, r = bryson_weights()
    weights = ", ".join(f"{w:.2f}" for w in np.diag(q))
    print(f"Q = diag({weights}), R = {r[0, 0]:.1f}")
    print("  vx    k1      k2      k3      k4     residual   slowest pole  fastest pole")
    for v in (2.0, 5.0, 10.0, 20.0, 30.0):
        k, p, a, b = lqr_gain(v)
        res = np.abs(a.T @ p + p @ a - p @ b @ np.linalg.solve(r, b.T @ p) + q).max()
        poles = np.linalg.eigvals(a - b @ k)
        slow = poles[np.argmin(np.abs(poles))]
        fast = poles[np.argmax(np.abs(poles))]
        print(f"{v:4.0f} {k[0, 0]:7.4f} {k[0, 1]:7.4f} {k[0, 2]:7.4f} {k[0, 3]:7.4f}"
              f"  {res:9.1e}  {slow.real:8.3f}{slow.imag:+.3f}j {fast.real:9.3f}")
    print(f"sqrt(Q11 / R) = {np.sqrt(q[0, 0] / r[0, 0]):.4f}")
    sched = GainSchedule()
    worst, where = 0.0, 0.0
    for v in np.arange(2.05, 29.9, 0.37):
        exact = lqr_gain(v)[0][0]
        err = np.max(np.abs(sched.gain(v) - exact) / np.abs(exact))
        if err > worst:
            worst, where = err, v
    print(f"schedule: {len(sched.speeds)} speeds, worst relative error {worst:.1e} "
          f"at {where:.2f} m/s")


if __name__ == "__main__":
    main()
