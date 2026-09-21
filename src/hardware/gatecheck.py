"""Does the camera see a target, and does the centring math get it right?

    python3 -m hardware.gatecheck --port /dev/ttyTHS1 --fy 824 --cam-tilt 20 \\
        --fwd 3.0 --right 0.0 --dz 0.15 --target-width 0.3

PROPS OFF. Nothing arms, nothing spins, no transmitter needed - this never
calls set_rc. It only opens the camera and reads attitude.

NO GATE NEEDED. The detector (perception/detectors/hsv_classic.py) looks for
a red/orange blob - hue 0-12 or 169-180, fairly saturated, not too dark - and
uses that blob's bounding-box CENTRE. It does not require a ring or a hole.
A sheet of red/orange paper, a strip of tape, a cone, a red cup: anything
that colour works exactly as well as a real gate for what this script
checks, because gate_dz()/gate_azimuth() only ever look at that centre's
angular position, never the target's size or shape. Measure the WIDTH of
whatever you use and pass --target-width so the range readout (see below)
means something; everything else is unaffected.

WHAT THIS IS FOR. hover.py's --gate-z/--gate-roll fly on two numbers:
gate_dz()'s elevation and gate_azimuth()'s azimuth, both computed from the
detector's pixel offset PLUS the aircraft's own attitude (the mount tilt and
any roll/pitch have to come out before "which way is the target" means
anything). That attitude term is exactly the kind of thing that is invisible
until the aircraft tilts - tiltcheck.py exists because a wrong pitch sign
read zero error sitting still and real trouble the moment it moved. The same
shape of bug is possible here: a wrong tilt sign or a swapped axis in
gate_dz/gate_azimuth would show a plausible number when the aircraft is
level and a wrong one the moment it is not.

So: put the target at a MEASURED position, point the nose at it, and this
script computes the elevation and azimuth that position implies from plain
trigonometry, then compares that against what gate_dz()/gate_azimuth() - the
literal functions hover.py flies with - compute from the live detection and
the live FC attitude. Then ROCK the aircraft (roll it, pitch it, hold it
nose-up) without moving the target. The true answer cannot change - the
target has not moved - so if the reported numbers drift with the tilt, the
attitude compensation is wrong, and that is worth knowing before it flies.

MEASURING THE GEOMETRY. Point the nose at the target along a fixed reference
line (a floor tape line or a straight table edge works) and measure from the
lens:

    --fwd    distance along that line to the target's centre, m
    --right  the target centre's offset to the RIGHT of that line, m (0 if
             it is dead ahead - do this case first)
    --dz     the target centre's height minus the lens height, m (matches
             hardware.camcal's --dz convention). With the body level and the
             camera mounted at --cam-tilt above body-forward, a target at
             the SAME height as the lens sits near the bottom of the frame,
             not the centre: dz ~= fwd * tan(cam_tilt) puts it near the
             image null instead, which is also where a real gate sits
             relative to a level, hovering aircraft.
    --target-width  the target's actual width, m. hardware.runtime's range
             formula assumes a 2.7 m gate; without this the printed range is
             scaled wrong for anything smaller. The elevation/azimuth numbers
             this script actually judges do NOT depend on it at all.

--fy and --cam-tilt should be the numbers hardware.camcal already measured
for this camera - this script does not calibrate them, it checks what they
imply.
"""

from __future__ import annotations

import argparse
import math
import time


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=None, help="omit to skip attitude: R is then held at level (identity), "
                                                  "which cannot exercise the tilt compensation at all")
    ap.add_argument("--tcp", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--camera", default=None, help="GStreamer pipeline or /dev/videoN")
    ap.add_argument("--fy", type=float, default=830.0, help="focal length in px, from hardware.camcal")
    ap.add_argument("--cam-tilt", type=float, default=20.0, help="camera up-tilt above body forward, deg, from hardware.camcal")
    ap.add_argument("--fwd", type=float, required=True, help="lens to target centre, m, along the reference line")
    ap.add_argument("--right", type=float, default=0.0, help="target centre offset to the right of that line, m")
    ap.add_argument("--dz", type=float, default=0.0, help="target centre height minus lens height, m")
    ap.add_argument("--target-width", type=float, default=None,
                    help="actual width of the red/orange stand-in, m (e.g. 0.18 for a sheet of paper). "
                         "Default: assume it's the real 2.7 m gate. Only affects the informational "
                         "range column, never the elevation/azimuth this script judges.")
    ap.add_argument("--tol-deg", type=float, default=3.0,
                    help="pass/fail threshold on elevation/azimuth error. hover.py's own note: 5 deg of tilt "
                         "error is 0.26 m of height offset at 3 m - 3 deg is a tighter bar than a cage test needs "
                         "to survive, which is the point of checking it here instead of finding out in flight")
    ap.add_argument("--min-samples", type=int, default=20,
                    help="fresh detections required before a verdict is given - otherwise a camera that never "
                         "saw the target would silently 'pass' on zero evidence")
    ap.add_argument("--seconds", type=float, default=60.0, help="Ctrl+C to stop earlier")
    args = ap.parse_args(argv)

    from hardware.hover import gate_azimuth, gate_dz
    from hardware.runtime import DEFAULT_PIPELINE, GATE_OUTER_M, CameraThread

    # CameraThread's range formula assumes a GATE_OUTER_M-wide target; scale
    # its output back to metres for whatever width was actually used, so the
    # (informational only - never used for control) range column means
    # something for a sheet of paper instead of a 2.7 m ring.
    target_width = args.target_width if args.target_width else GATE_OUTER_M
    range_scale = target_width / GATE_OUTER_M

    # Ground truth, fixed for the whole run: the target does not move, only
    # the aircraft's attitude does. atan2(right, fwd) matches gate_azimuth's
    # "+ is to the right"; atan2(dz, horiz) matches gate_dz's "+ is above",
    # both against the SAME reference line the operator pointed the nose
    # along - the whole test is meaningless if the nose was not on that line.
    horiz = math.hypot(args.fwd, args.right)
    el_exp = math.degrees(math.atan2(args.dz, horiz))
    az_exp = math.degrees(math.atan2(args.right, args.fwd))
    rng_exp = math.hypot(horiz, args.dz)
    print(f"ground truth: fwd={args.fwd:.2f} right={args.right:.2f} dz={args.dz:.2f} m  ->  "
          f"expect el={el_exp:+.2f} deg, az={az_exp:+.2f} deg, range={rng_exp:.2f} m")
    print(f"tolerance +-{args.tol_deg:.1f} deg. This does not change as you tilt the aircraft - "
          f"the target has not moved.\n")

    src = None
    if args.port or args.tcp:
        from hardware.bridge import FcBridge
        from hardware.state import FcStateSource
        br = FcBridge.open(port=args.port, tcp=args.tcp, baud=args.baud)
        br.start()
        src = FcStateSource(br)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and br.state().attitude is None:
            time.sleep(0.05)
        if br.state().attitude is None:
            print("ERROR no attitude from the FC after 10 s"); br.stop(); return 2
    else:
        print("WARNING no --port: R held at level (identity). Rocking the aircraft will do "
              "nothing to the reported numbers - this only checks the level case.\n")

    camera = CameraThread(args.camera or DEFAULT_PIPELINE, args.fy)
    camera.start()
    t_warm = time.monotonic()
    while camera.frame_wh is None and time.monotonic() - t_warm < 5.0:
        time.sleep(0.05)
    if camera.frame_wh is None:
        print(f"ERROR no frames from the camera after 5 s: {camera.error or 'check the pipeline'}")
        camera.stop()
        if src is not None:
            br.stop()
        return 3
    print(f"camera: {camera.frame_wh[0]}x{camera.frame_wh[1]}\n")

    import numpy as np
    identity_R = np.eye(3)

    tilt_rad = math.radians(args.cam_tilt)
    print(f"{'t':>6} {'roll':>6} {'pitch':>6} {'el':>7} {'el_err':>7} {'az':>7} {'az_err':>7} "
          f"{'rng':>6} {'rng_err':>7} {'area%':>6}   verdict")
    print("-" * 88)

    n = 0                 # fresh detections used for a verdict
    worst_el, worst_az = 0.0, 0.0     # SIGNED and kept by magnitude - a
    worst_el_at, worst_az_at = None, None   # sign error reads consistently one way
    rng_errs = []          # informational only - range is never trusted for control
    t0 = time.monotonic()
    last_print = 0.0
    try:
        while time.monotonic() - t0 < args.seconds:
            t = time.monotonic()
            det = camera.latest()
            fresh = det is not None and (t - det.t) <= 0.3
            roll_deg = pitch_deg = None
            R = identity_R
            if src is not None:
                est = src.estimate()
                if est is not None:
                    R = est.R
                    a = br.state().attitude
                    if a is not None:
                        roll_deg, pitch_deg = a.roll_deg, a.pitch_deg
            el_err = az_err = None
            el_deg = az_deg = rng = rng_err = area = None
            if fresh:
                dz_m, el = gate_dz(det, R, args.fy, tilt_rad, camera.frame_wh)
                az = gate_azimuth(det, args.fy, tilt_rad, camera.frame_wh)
                el_deg, az_deg = math.degrees(el), math.degrees(az)
                el_err, az_err = el_deg - el_exp, az_deg - az_exp
                area = det.area_frac
                if det.range_m is not None:
                    rng = det.range_m * range_scale
                    rng_err = rng - rng_exp
                    rng_errs.append(rng_err)
                n += 1
                if abs(el_err) > abs(worst_el):
                    worst_el, worst_el_at = el_err, (roll_deg, pitch_deg)
                if abs(az_err) > abs(worst_az):
                    worst_az, worst_az_at = az_err, (roll_deg, pitch_deg)

            if t - last_print > 0.25:
                last_print = t
                if fresh:
                    verdict = ("ok" if max(abs(el_err), abs(az_err)) < args.tol_deg else
                               "SUSPECT" if max(abs(el_err), abs(az_err)) < 2 * args.tol_deg else "FAIL")
                    rs = f"{roll_deg:+6.1f}" if roll_deg is not None else "    - "
                    ps = f"{pitch_deg:+6.1f}" if pitch_deg is not None else "    - "
                    rg = f"{rng:6.2f}" if rng is not None else "     -"
                    rge = f"{rng_err:+7.2f}" if rng_err is not None else "      -"
                    print(f"\r{t - t0:6.1f} {rs} {ps} {el_deg:+7.2f} {el_err:+7.2f} "
                          f"{az_deg:+7.2f} {az_err:+7.2f} {rg} {rge} {area * 100:6.2f}   {verdict:8s}",
                          end="", flush=True)
                else:
                    print(f"\r{t - t0:6.1f} {'no target: ' + str(camera.detections) + '/' + str(camera.frames) + ' frames':70s}",
                          end="", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        if src is not None:
            br.stop()

    print("\n")
    if camera.frames:
        print(f"camera feed: {camera.detections}/{camera.frames} frames had a target ({camera.fps:.0f} fps)")
    if n < args.min_samples:
        print(f"\n  FAIL: only {n} fresh detections in {args.seconds:.0f} s (need {args.min_samples}). "
              f"The camera did not see the target enough to judge the math - fix the HSV thresholds "
              f"(perception/detectors/hsv_classic.py), the framing, or the lighting first.")
        return 1

    print(f"  worst elevation error: {worst_el:+.2f} deg (roll {worst_el_at[0]:+.0f}, pitch {worst_el_at[1]:+.0f})"
          if worst_el_at and worst_el_at[0] is not None else f"  worst elevation error: {worst_el:+.2f} deg")
    print(f"  worst azimuth error:   {worst_az:+.2f} deg (roll {worst_az_at[0]:+.0f}, pitch {worst_az_at[1]:+.0f})"
          if worst_az_at and worst_az_at[0] is not None else f"  worst azimuth error:   {worst_az:+.2f} deg")
    if rng_errs:
        rng_errs.sort()
        med = rng_errs[len(rng_errs) // 2]
        print(f"  range error (informational only, never used for control): median {med:+.2f} m, "
              f"worst {max(rng_errs, key=abs):+.2f} m over {len(rng_errs)} samples")
    ok = abs(worst_el) < args.tol_deg and abs(worst_az) < args.tol_deg
    if ok:
        print(f"\n  PASS: both stay within {args.tol_deg:.1f} deg of the measured geometry"
              + (", including while tilted" if src is not None else " at level (attitude untested - no --port)"))
        return 0
    print(f"\n  FAIL: exceeds {args.tol_deg:.1f} deg.")
    if src is not None and (worst_el_at[0] not in (0, None) or worst_el_at[1] not in (0, None)
                            or worst_az_at[0] not in (0, None) or worst_az_at[1] not in (0, None)):
        print("  The worst case came at a nonzero roll/pitch: if the error GROWS with tilt and does "
              "not just sit at a constant offset, suspect a wrong sign or swapped axis in --cam-tilt's "
              "rotation inside gate_dz/gate_azimuth, the same class of bug tiltcheck.py exists to catch.")
    else:
        print("  A constant offset even near level points at --fy, --cam-tilt, or the measured "
              "geometry (--fwd/--right/--dz) being off, not a sign error.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
