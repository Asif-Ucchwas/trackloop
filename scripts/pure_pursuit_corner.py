"""Pure Pursuit corner cutting: straight into a left arc, kinematic model."""
from trackloop.corner_cut import THEORY_OUTWARD, THEORY_PEAK, THEORY_WHERE, measure
from trackloop.models.kinematic import KinematicParams


def main():
    print(f"theory (small angles): peak {THEORY_PEAK} ld^2/R at {THEORY_WHERE} ld, "
          f"then outward {THEORY_OUTWARD} ld^2/R")
    print("  R   ld   v   peak(m)  peak*R/ld^2  where/ld  outward*R/ld^2")
    for radius in (20.0, 40.0, 80.0):
        for ld in (2.0, 4.0, 6.0, 8.0):
            peak, where, dip = measure(radius, ld, 5.0)
            scale = radius / ld**2
            print(f"{radius:4.0f} {ld:4.0f} {5:3.0f} {peak:8.4f} {peak * scale:10.4f} "
                  f"{where / ld:9.3f} {dip * scale:12.4f}")
    print("speed check at R = 40, ld = 4")
    for v in (3.0, 5.0, 10.0):
        peak, where, dip = measure(40.0, 4.0, v)
        print(f"v={v:4.0f}  peak {peak:.4f} m  peak*R/ld^2 {peak * 40.0 / 16.0:.4f}")
    print("with the default steering rate limit (0.8 rad/s)")
    for ld in (2.0, 4.0, 6.0):
        peak, _, _ = measure(40.0, ld, 5.0, params=KinematicParams())
        print(f"ld={ld:3.0f}  peak {peak:.4f} m  peak*R/ld^2 {peak * 40.0 / ld**2:.4f}")


if __name__ == "__main__":
    main()
