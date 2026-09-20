"""Are the attitude signs right? Tilt the aircraft and watch one number.

    python3 -m hardware.tiltcheck --port /dev/ttyTHS1

PROPS OFF. Nothing arms, nothing spins, no transmitter needed.

WHAT THIS IS FOR. Everything vertical starts from

    az_world = (R @ accel_body)[2] - 9.80665

the aircraft's vertical acceleration with gravity removed. Hold the aircraft
STILL and that must read zero - at ANY attitude. Level, nose up, nose down,
rolled left, rolled right. Gravity does not care which way the aircraft is
pointing, so neither may this number.

If a roll or pitch SIGN is inverted, `R` unrotates gravity the wrong way and
the error is 2 * g * sin(tilt): nothing at all when level, and 9.8 m/s^2 at
30 degrees. That is invisible to any at-rest check, because at rest the
aircraft is level. Integrated for half a second it is 5 m/s of phantom
vertical speed, and the altitude loop answers phantom speed with real
throttle. d45 reported -15.8 m/s while being carried by hand on 2026-09-20.

The aircraft tilts in flight. That is the whole job. So this has to be right.

HOW TO READ IT. Hold each pose still for a second or two - it is the STEADY
value that matters, not the swing while you are moving it.

    level           az must be ~0
    nose up 30      az must be ~0
    nose down 30    az must be ~0
    roll left 30    az must be ~0
    roll right 30   az must be ~0

Anything that grows with tilt is a sign error, and `worst` at the end says how
bad. Under 1 m/s^2 is fine. Over 3 is the bug.

A wrong sign has a signature: the error is g * (cos 2t - 1), so it is always
NEGATIVE, it is zero when level, and it grows fast - about -2.3 m/s^2 at 20
degrees and -5.2 at 31. This tool prints what an inverted sign would predict
at the worst tilt it saw, so you can compare.

Measured on d45, 2026-09-20: -5.24 m/s^2 at 31 degrees against a predicted
-5.20. Betaflight on this firmware reads pitch positive NOSE DOWN, and that
is now the default. `--pitch-nose-up-positive` flips back to the old
assumption if some other aircraft disagrees.
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
    ap.add_argument("--acc-lsb-per-g", default="auto")
    ap.add_argument("--pitch-nose-up-positive", action="store_true",
                    help="flip back to the old assumption, to compare")
    args = ap.parse_args(argv)

    import numpy as np

    from hardware.bridge import FcBridge
    from hardware.state import FcStateSource

    br = FcBridge.open(port=args.port, tcp=args.tcp, baud=args.baud)
    br.start()
    src = FcStateSource(br)
    if args.pitch_nose_up_positive:
        src.pitch_sign = 1.0

    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and br.state().attitude is None:
        time.sleep(0.05)
    if br.state().attitude is None:
        print("ERROR no attitude from the FC after 10 s"); br.stop(); return 2

    # scale, measured level and still, exactly as the flying tools do it
    if args.acc_lsb_per_g == "auto":
        mags, t_cal = [], time.monotonic()
        while time.monotonic() - t_cal < 1.0:
            st = br.state()
            if st.imu is not None:
                mags.append(math.sqrt(sum(float(v) ** 2 for v in st.imu.acc)))
            time.sleep(0.02)
        if not mags:
            print("ERROR no MSP_RAW_IMU"); br.stop(); return 3
        src.acc_lsb_per_g = float(np.median(mags))
    else:
        src.acc_lsb_per_g = float(args.acc_lsb_per_g)
    print(f"accelerometer {src.acc_lsb_per_g:.0f} counts per g\n")
    print("PROPS OFF. Hold each pose STILL for a second. Ctrl+C when done.\n")
    print(f"{'roll':>7} {'pitch':>7} {'|acc|':>7} {'az_world':>9}   verdict")
    print("-" * 52)

    worst = 0.0          # SIGNED, and kept by magnitude - a sign inversion
    worst_at = (0.0, 0.0)  # reads negative, and printing abs() hid that once
    t0 = time.monotonic()
    last = 0.0
    try:
        while time.monotonic() - t0 < args.seconds:
            est = src.estimate()
            if est is None or src.accel_body is None:
                time.sleep(0.05); continue
            a = br.state().attitude
            az = float((est.R @ src.accel_body)[2]) - 9.80665
            mag = float(np.linalg.norm(src.accel_body))
            # only judge it when it is actually still: a moving hand produces
            # real acceleration and that is not what is being measured
            still = abs(mag - 9.80665) < 1.0
            if still and abs(a.pitch_deg) + abs(a.roll_deg) > 10.0 and abs(az) > abs(worst):
                worst, worst_at = az, (a.roll_deg, a.pitch_deg)
            now = time.monotonic()
            if now - last > 0.25:
                last = now
                tag = ("moving - hold it still" if not still else
                       "ok" if abs(az) < 1.0 else
                       "SUSPECT" if abs(az) < 3.0 else "SIGN ERROR")
                print(f"\r{a.roll_deg:+7.1f} {a.pitch_deg:+7.1f} {mag:7.2f} "
                      f"{az:+9.2f}   {tag:22s}", end="", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        br.stop()

    print("\n")
    print(f"  worst az_world while tilted and still: {worst:+.2f} m/s^2")
    print(f"  at roll {worst_at[0]:+.0f}, pitch {worst_at[1]:+.0f}")
    tilt = math.radians(max(abs(worst_at[0]), abs(worst_at[1])))
    predicted = 9.80665 * (math.cos(2.0 * tilt) - 1.0)
    if abs(worst) > 1.0:
        print(f"  an inverted sign at that tilt predicts {predicted:+.2f} m/s^2")
    if abs(worst) < 1.0:
        print("\n  PASS: gravity is removed correctly at every attitude.")
        return 0
    print(f"\n  FAIL: {worst:.1f} m/s^2 of phantom vertical acceleration under tilt.")
    print(f"  Integrated for half a second that is {worst * 0.5:.1f} m/s of vertical")
    print("  speed the aircraft is not doing, and the altitude loop will answer it")
    print("  with throttle. If that figure matches the prediction above, a sign is")
    print("  inverted: re-run with --pitch-nose-up-positive and see which way is")
    print("  right for THIS aircraft, then say so before anything flies.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
