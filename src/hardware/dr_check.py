"""Dead-reckoning bench check: does the estimate stay put when the drone does?

    python3 -m hardware.dr_check --port /dev/ttyTHS1 --seconds 60

Put the aircraft on the FLOOR and do not touch it. Position should stay at
zero for the whole run.

Why this exists. The accelerometer has a residual bias after gravity is
removed - 0.15 m/s^2 measured on d45, 2026-09-20, which matches the
organizers' own flight log. Integrated, that is 1.9 m of phantom position
after 5 s and 200 m after a minute. The camera fixes bound it in flight, but
on the start line there are no useful fixes and the aircraft is not moving, so
the estimator holds velocity at zero instead (`on_ground`).

Deciding "on the ground" from height alone does not work: the barometer drifts
about 0.25 m per minute at rest, so any fixed threshold is crossed eventually
while the drone sits still, and integration silently restarts. This uses the
same test the runtime does - height AND a real climb rate, latched.

    PASS  position under ~1 m after 60 s, air=False throughout
    FAIL  position running away, or air latches while it is sitting still
"""

from __future__ import annotations

import argparse
import math
import time


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyTHS1")
    ap.add_argument("--tcp", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--no-ground-hold", action="store_true",
                    help="integrate regardless, to see the raw drift")
    args = ap.parse_args(argv)

    from hardware.bridge import FcBridge
    from hardware.state import FcStateSource
    from seeker.dr_estimator import DeadReckonSource

    br = FcBridge.open(port=args.port, tcp=args.tcp, baud=args.baud)
    br.start()
    src = FcStateSource(br)
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        s = br.state()
        if s.attitude is not None and s.altitude is not None:
            break
        time.sleep(0.05)
    if br.state().altitude is None:
        print("ERROR no MSP_ALTITUDE after 10 s"); br.stop(); return 3
    src.zero_altitude()

    dr = DeadReckonSource()
    t0 = time.monotonic()
    air = False
    last = -1
    print(f"{'t':>6} {'x':>8} {'y':>8} {'|p|':>7} {'vx':>7} {'vy':>7} {'z':>6}  air")
    try:
        while time.monotonic() - t0 < args.seconds:
            t = time.monotonic()
            est = src.estimate()
            if est is not None and src.accel_body is not None:
                z, vz = float(est.p[2]), float(est.v[2])
                # the runtime's test: height AND a real climb, latched
                if not air and z > 0.30 and vz > 0.5:
                    air = True
                    print(f"  AIRBORNE latched at t={t - t0:.1f}s (z={z:.2f} vz={vz:+.2f})")
                dr.integrate(t, est.R, src.accel_body, z, True,
                             on_ground=(not air) and not args.no_ground_hold)
            n = int(t - t0)
            if n % 5 == 0 and n != last:
                last = n
                d = math.hypot(dr.p[0], dr.p[1])
                print(f"{t - t0:6.1f} {dr.p[0]:+8.2f} {dr.p[1]:+8.2f} {d:7.2f} "
                      f"{dr.v[0]:+7.2f} {dr.v[1]:+7.2f} {dr.p[2]:6.2f}  {air}")
            time.sleep(0.02)
    except KeyboardInterrupt:
        print()
    finally:
        br.stop()

    d = math.hypot(dr.p[0], dr.p[1])
    print(f"\n  drift after {args.seconds:.0f} s: {d:.2f} m")
    if args.no_ground_hold:
        print("  (ground hold disabled - this IS the raw accelerometer drift)")
    elif air:
        print("  FAIL: it latched airborne while sitting still. The climb test is "
              "too loose, or something moved it.")
        return 1
    elif d > 1.0:
        print("  FAIL: position ran away with the ground hold on. Something is wrong.")
        return 1
    else:
        print("  PASS: the estimate stays put on the ground.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
