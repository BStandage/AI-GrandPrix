"""Autonomous hover: the Jetson holds altitude, nothing else.

    python3 -m hardware.hover --port /dev/ttyTHS1 --alt 1.2 --seconds 30 --arm

THE JETSON SPINS THE MOTORS. Prerequisites, all of them:

  * `hardware.override_test` has PASSED on this aircraft. The pilot taking the
    sticks back is the only abort there is.
  * Props on, drone in the cage, everyone clear.
  * Pilot on the transmitter with a finger on the MSP OVERRIDE switch.

What it does: takes off, holds `--alt` for `--seconds`, then descends at
0.3 m/s and disarms on the ground. Roll, pitch and yaw are held centred, so
with mask 15 the aircraft keeps ITSELF level but does not hold POSITION - it
will drift with any air movement. In a 5x5 m cage that is the thing to watch.

Why bother: it is the smallest possible autonomous flight and it measures the
one number we cannot get on a bench - what throttle THIS aircraft hovers at,
through our own altitude loop. Compare it with the 1291 we fitted from the
organizers' blackbox: if they agree, that whole thrust model transfers.

--gate-z adds AUTONOMOUS ALTITUDE FROM VISION. Point the nose at a gate and
the aircraft holds the height of that gate's centre instead of a fixed number.

Why it is worth flying: holding the gate centre on the camera's horizon is a
RANGE-INDEPENDENT null. You do not need to know how far away the gate is, and
you do not need its height - if the centre sits at zero elevation, you are at
its height. Range error, our noisiest signal, cancels at the null. Compare the
barometer: 0.076 m of resolution and 0.25 m/min of drift.

What it deliberately CANNOT do: translate. Vision drives the THROTTLE and
nothing else - roll, pitch and yaw stay centred exactly as they do in a plain
hover. There is no axis that can command "toward the gate", which is the only
way a vision hold gets you into a net. Lose the gate and it reverts to the
barometer target within --gate-lost-s. The commanded height is clamped to
[--gate-z-min, --gate-z-max] whatever the camera says, and slew-limited.

Vision only takes over during the HOLD phase. Takeoff and landing are always
flown on the barometer.

--dry-run computes and logs everything with the FC disarmed and neutral
sticks. Do that first, indoors, props off - with --gate-z it is also how you
check the sign, by hand, before anything spins.
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path


def gate_dz(det, R, fy: float, tilt_rad: float, wh) -> tuple:
    """How far ABOVE the aircraft the gate's centre is, in metres.

    The detector gives the centre as a normalised offset from the OPTICAL
    axis. Turn that into a ray in the camera frame, rotate it down into the
    body frame by the mount tilt, then into the world by the aircraft's own
    attitude - so the aircraft's pitch is taken out, which matters because the
    camera is bolted to a body that tilts.

    The world ray's vertical component times the range IS the height
    difference. At the null it is zero and the range drops out entirely,
    which is the whole point of doing it this way.

    Returns (dz_m, elevation_rad). dz is None without a usable range; the
    ELEVATION never needs one, which is the whole point - see the control
    note in main()."""
    if det is None or wh is None:
        return None, None
    w_px, h_px = wh
    # tangents from the optical axis (fwd_cam == 1 by construction)
    down_cam = det.offset_y * (h_px / 2.0) / fy
    right_cam = det.offset_x * (w_px / 2.0) / fy
    c, s_ = math.cos(tilt_rad), math.sin(tilt_rad)
    fwd_b = c + s_ * down_cam                    # camera -> body, tilt taken out
    down_b = -s_ + c * down_cam
    import numpy as np
    v = np.array([fwd_b, -right_cam, -down_b])   # FRD -> FLU body
    n = float(np.linalg.norm(v)) or 1.0
    vw = R @ (v / n)                             # body -> world
    up = float(max(-1.0, min(1.0, vw[2])))
    el = math.asin(up)
    return (det.range_m * up if det.range_m is not None else None), el


def gate_azimuth(det, fy: float, tilt_rad: float, wh):
    """Body-frame azimuth of the gate centre, radians, + is to the RIGHT.

    Deliberately BODY frame, not world: the answer is "which way do I tilt",
    and the aircraft tilts in its own frame. The mount tilt still has to come
    out, because a camera pitched up 20 deg turns a bit of vertical offset
    into apparent horizontal offset once the body rolls."""
    if det is None or wh is None:
        return None
    w_px, h_px = wh
    down_cam = det.offset_y * (h_px / 2.0) / fy
    right_cam = det.offset_x * (w_px / 2.0) / fy
    c, s_ = math.cos(tilt_rad), math.sin(tilt_rad)
    fwd_b = c + s_ * down_cam
    return math.atan2(right_cam, fwd_b)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyTHS1")
    ap.add_argument("--tcp", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--config", default=None, help="vehicle toml (thrust curve, gains)")
    ap.add_argument("--alt", type=float, default=1.2,
                    help="hover height, m. Below ~1 m you are in ground effect "
                         "and the hover throttle reads low, which is the number "
                         "you came to measure")
    ap.add_argument("--seconds", type=float, default=30.0, help="hold time at altitude")
    ap.add_argument("--climb", type=float, default=0.5, help="climb rate, m/s")
    ap.add_argument("--descend", type=float, default=0.3, help="descent rate, m/s")
    ap.add_argument("--rc-hz", type=float, default=50.0)
    ap.add_argument("--acc-lsb-per-g", default="auto",
                    help="raw accelerometer counts per g. 'auto' measures it at rest "
                         "in the first second - the aircraft must be still and level.")
    ap.add_argument("--pitch-nose-up-positive", action="store_true",
                    help="this firmware reads pitch positive NOSE DOWN (measured on "
                         "d45, 2026-09-20), which is the default. Pass this only if "
                         "hardware.tiltcheck says otherwise on YOUR aircraft.")
    ap.add_argument("--takeoff-pwm", type=int, default=None,
                    help="throttle held until the aircraft is climbing at 0.7 m/s. "
                         "The config default (1700) was tuned on the 0.8 kg sim plant; "
                         "on the measured curve it is 3.4 g and the aircraft leaps. "
                         "1350 is about 1.3 g, which lifts off gently. Default: the config.")
    ap.add_argument("--no-baro", action="store_true",
                    help="take the barometer out of the loop entirely and hold "
                         "vertical SPEED at zero using the accelerometer. For an "
                         "aircraft whose barometer is unusable with props running. "
                         "It will not hold a height - it drifts slowly and the pilot "
                         "corrects. See the note in the code.")
    ap.add_argument("--climb-s", type=float, default=0.45,
                    help="--no-baro: seconds of takeoff thrust before the speed hold "
                         "takes over. 0.45 s at 1.32 g is about 0.3 m up. This is the "
                         "only altitude control this mode has.")
    ap.add_argument("--vz-gain", type=float, default=2.5,
                    help="m/s^2 of correction per m/s of vertical speed error, --no-baro")
    ap.add_argument("--vz-max", type=float, default=2.5,
                    help="abort if vertical speed exceeds this for 0.4 s")
    ap.add_argument("--raw-alt", action="store_true",
                    help="close the altitude loop on the RAW barometer instead of the "
                         "accel+baro fusion. Only to reproduce the old behaviour.")
    ap.add_argument("--ceiling", type=float, default=None,
                    help="ABORT height, m above the start point. Throttle to minimum "
                         "and disarm if the barometer ever reads above it. Default: "
                         "--alt + 1.0. This is a dumb backstop on the RAW barometer "
                         "and does not trust any filter, because on 2026-09-20 it was "
                         "the filter that was wrong.")
    ap.add_argument("--baro-w", type=float, default=1.0,
                    help="how much authority the barometer has over the fused height, "
                         "rad/s. 3.0 tracks a bench sensor and CHASES a flying one - "
                         "the loop reacts to a spike, the throttle moves, the prop "
                         "wash moves, and the barometer spikes again. Lower leans on "
                         "the accelerometer through the transient.")
    ap.add_argument("--ceiling-hold", type=float, default=0.35,
                    help="the RAW barometer must stay above --ceiling this long to "
                         "abort. A real climb does; a prop-wash spike does not. The "
                         "FUSED height aborts immediately - it cannot spike.")
    ap.add_argument("--gate-z", action="store_true",
                    help="hold the altitude of the gate centre in view, not --alt")
    ap.add_argument("--camera", default=None, help="GStreamer pipeline or /dev/videoN")
    ap.add_argument("--fy", type=float, default=830.0, help="focal length in px at the capture size")
    ap.add_argument("--cam-tilt", type=float, default=20.0, help="camera up-tilt above body forward, deg")
    ap.add_argument("--gate-z-min", type=float, default=0.40, help="floor on the vision target, m")
    ap.add_argument("--gate-z-max", type=float, default=1.80, help="ceiling on the vision target, m")
    ap.add_argument("--gate-lost-s", type=float, default=0.50,
                    help="no detection for this long -> back to the barometer target")
    ap.add_argument("--gate-slew", type=float, default=0.30,
                    help="cap on how fast the vision target may move, m/s")
    ap.add_argument("--gate-el-gain", type=float, default=0.03,
                    help="target climb rate in m/s per degree of elevation error. "
                         "0.03 means a 10 deg error moves the target 0.3 m/s")
    ap.add_argument("--gate-roll", action="store_true",
                    help="also CENTRE the gate with roll: slide sideways until it is "
                         "dead ahead. Converges on one point - the travel is "
                         "range x tan(heading error), so aim the nose at the gate. "
                         "Removes the lateral drift a plain hover has no control over.")
    ap.add_argument("--gate-roll-tilt", type=float, default=4.0,
                    help="hard cap on the commanded roll angle, deg")
    ap.add_argument("--gate-roll-gain", type=float, default=0.25,
                    help="commanded roll angle per degree of azimuth error")
    ap.add_argument("--gate-pitch", action="store_true",
                    help="also hold RANGE to the gate. Completes the three-axis "
                         "vision-relative hold: elevation gives height, azimuth gives "
                         "lateral, range gives distance. By default it may only BACK "
                         "AWAY - see --gate-pitch-fwd.")
    ap.add_argument("--gate-range", type=float, default=0.0,
                    help="distance to hold, m. 0 = whatever it is when the gate is "
                         "first acquired.")
    ap.add_argument("--gate-pitch-tilt", type=float, default=4.0,
                    help="hard cap on BACKING AWAY, deg")
    ap.add_argument("--gate-pitch-fwd", type=float, default=0.0,
                    help="hard cap on moving TOWARD the gate, deg. ZERO by default: "
                         "the range signal is the camera's worst, and in a cage with a "
                         "net a false 'too far' is the one mistake that cannot be "
                         "afforded. Raise it only in open space.")
    ap.add_argument("--gate-pitch-gain", type=float, default=2.0,
                    help="commanded pitch angle per metre of range error")
    ap.add_argument("--gate-range-min", type=float, default=1.0,
                    help="ranges outside [min, max] are discarded before filtering")
    ap.add_argument("--gate-range-max", type=float, default=12.0)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--dry-run", action="store_true")
    g.add_argument("--arm", action="store_true")
    args = ap.parse_args(argv)

    import numpy as np

    from hardware.bridge import FcBridge
    from hardware.state import FcStateSource
    from raceline.config import AIGP_REPO, load_config
    from raceline.rc_backend import AltitudeLoop

    camera = None
    if args.gate_z:
        from hardware.runtime import DEFAULT_PIPELINE, CameraThread
        camera = CameraThread(args.camera or DEFAULT_PIPELINE, args.fy)
        camera.start()
        print(f"vision altitude ON: holding the gate centre, clamped to "
              f"[{args.gate_z_min:.2f}, {args.gate_z_max:.2f}] m. "
              f"Throttle only - roll/pitch/yaw stay centred.")

    cfg = load_config(args.config) if args.config else load_config(None)
    br = FcBridge.open(port=args.port, tcp=args.tcp, baud=args.baud)
    br.start()
    src = FcStateSource(br)
    if args.pitch_nose_up_positive:
        src.pitch_sign = 1.0

    # Wait for BOTH. The bridge polls attitude every tick but altitude,
    # battery and status on a slower rotation, so altitude can still be None
    # several hundred ms after the first attitude arrives. Checking too early
    # reads as "no barometer" on an aircraft that has one.
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        s = br.state()
        if s.attitude is not None and s.altitude is not None:
            break
        time.sleep(0.05)
    s = br.state()
    if s.attitude is None:
        print("ERROR no attitude from the FC after 10 s"); br.stop(); return 2
    if s.altitude is None:
        print("ERROR no MSP_ALTITUDE after 10 s: check `fc-info` lists BARO"); br.stop(); return 3
    # ACCELEROMETER SCALE. This argument existed and was never applied, so
    # every flight so far ran on the 512 counts/g default. That was invisible
    # while the vertical filter ignored the accelerometer on the ground; the
    # moment it started integrating at launch, a wrong scale became a constant
    # phantom acceleration. d45 read vz of -15.8 m/s standing still in a hand
    # on 2026-09-20, which is about -5 m/s^2 of integration - exactly what a
    # 2x scale error looks like.
    if args.acc_lsb_per_g == "auto":
        mags, t_cal = [], time.monotonic()
        while time.monotonic() - t_cal < 1.0:
            st = br.state()
            if st.imu is not None:
                mags.append(math.sqrt(sum(float(v) ** 2 for v in st.imu.acc)))
            time.sleep(0.02)
        if mags:
            import numpy as _n
            med, spread = float(_n.median(mags)), max(mags) - min(mags)
            if spread < 0.1 * med:
                src.acc_lsb_per_g = med
                print(f"accelerometer: {med:.0f} raw counts per g measured at rest "
                      f"({len(mags)} samples, spread {spread:.0f})")
            else:
                print(f"ERROR accelerometer not at rest (median {med:.0f}, spread "
                      f"{spread:.0f}). Put it down still and restart, or pass "
                      f"--acc-lsb-per-g.")
                br.stop(); return 4
        else:
            print("ERROR no MSP_RAW_IMU: cannot scale the accelerometer"); br.stop(); return 4
    else:
        src.acc_lsb_per_g = float(args.acc_lsb_per_g)
        print(f"accelerometer: {src.acc_lsb_per_g:.0f} counts per g (given)")

    # REST CHECK. With the scale applied, a still aircraft must measure zero
    # vertical acceleration once gravity is removed. This is the one line that
    # would have caught the wrong scale before it ever flew, so it is a hard
    # stop rather than a warning: everything downstream integrates this number.
    rest = []
    t_rest = time.monotonic()
    while time.monotonic() - t_rest < 0.5:
        e = src.estimate()
        if e is not None and src.accel_body is not None:
            rest.append(float((e.R @ src.accel_body)[2]) - 9.80665)
        time.sleep(0.02)
    if rest:
        import numpy as _n
        resid = float(_n.median(rest))
        print(f"accelerometer at rest: {resid:+.2f} m/s^2 after gravity "
              f"({len(rest)} samples)")
        if abs(resid) > 1.0:
            print(f"  ERROR that should be near zero. {resid:+.2f} m/s^2 integrates "
                  f"into {abs(resid) * 2:.0f} m/s of phantom climb in two seconds.")
            print(f"  The aircraft was not still or level, or the scale is wrong.")
            br.stop(); return 5

    src.vert_w = args.baro_w
    src.zero_altitude()
    print(f"altitude zeroed. hover target {args.alt:.2f} m for {args.seconds:.0f} s "
          f"(barometer authority {args.baro_w:.1f} rad/s, median of 3)")

    log_path = AIGP_REPO / "out" / "flightlogs" / f"hover_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = open(log_path, "w", newline="", encoding="utf-8")
    w = csv.writer(log)
    w.writerow(["t", "phase", "z", "vz", "z_target", "a_cmd", "thrust_cmd",
                "throttle", "roll", "pitch", "yaw_deg",
                # ACTUAL attitude, not the sticks. hover.py commands 1500/1500
                # throughout, so the roll/pitch columns above are constants and
                # say nothing about what the aircraft did. d44 drifted forward
                # on centred sticks, 2026-09-21, and the log could not
                # distinguish a flight controller whose idea of level is
                # tilted (it really leans, and accelerates) from an
                # aerodynamic asymmetry such as prop guards (it holds level
                # and translates anyway). These two columns settle it.
                "roll_deg", "pitch_deg",
                "armed", "vbat", "amps",
                "gate_seen", "gate_el_deg", "gate_rng", "gate_dz"])
    print(f"log -> {log_path}")

    if args.dry_run:
        print("DRY RUN: the FC stays disarmed and gets neutral sticks")
    else:
        print("LIVE: waiting for the pilot. Throttle low, ARM, then MSP OVERRIDE on.")
        while True:
            st = br.state().status
            br.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500, arm=1000, aux2=1500)
            if st is not None and st.armed and st.msp_override:
                # RE-ZERO HERE, not at startup. The barometer drifts about
                # 0.25 m/min at rest and the wait for the pilot is open-ended,
                # so a zero taken a minute ago is a minute stale. d44's bench
                # run, 2026-09-21, read -0.51 m one second after a zero and
                # took six seconds to settle: at liftoff that is the controller
                # believing it is half a metre low and asking for the climb to
                # match. Zeroing now costs nothing and the aircraft is still on
                # the ground, which is the only moment the zero is true.
                src.zero_altitude()
                print(f"armed, MSP OVERRIDE on, modes {', '.join(st.active_modes)}: "
                      f"altitude re-zeroed, lifting off")
                break
            time.sleep(0.05)

    alt = AltitudeLoop(cfg)
    if args.takeoff_pwm:
        cfg.follower.takeoff_pwm = int(args.takeoff_pwm)
    tk = cfg.follower.takeoff_pwm
    import numpy as _np
    a_tk = float(_np.interp(tk, cfg.thrust.curve_pwm, cfg.thrust.curve_acc))
    print(f"takeoff throttle {tk} -> {a_tk:.1f} m/s^2 ({a_tk / 9.80665:.2f} g), "
          f"net {a_tk - 9.80665:+.1f} m/s^2 upward")
    if a_tk - 9.80665 > 6.0:
        print("  WARNING: that is a hard launch. --takeoff-pwm 1350 is gentler.")
    # The default has to clear the HIGHEST height this run may legitimately
    # ask for, not just --alt: with --gate-z the vision target may climb to
    # --gate-z-max, and a backstop that trips on the aircraft doing exactly
    # what it was told is a backstop people start passing --ceiling to silence.
    top = max(args.alt, args.gate_z_max if args.gate_z else args.alt)
    ceiling = args.ceiling if args.ceiling is not None else top + 1.0
    print(f"ceiling {ceiling:.2f} m: above this it disarms, no questions asked"
          + (f" (vision may ask for {args.gate_z_max:.2f})" if args.gate_z else ""))
    if ceiling <= top + 0.2:
        print(f"  WARNING only {ceiling - top:.2f} m of margin above the highest "
              f"target this run may ask for. Expect nuisance trips.")
    period = 1.0 / args.rc_hz
    t0 = time.monotonic()
    phase, t_phase = "climb", t0
    airborne_latch = False
    vz_bad = 0.0                     # seconds of vertical speed runaway
    ceil_bad = 0.0                   # seconds the RAW baro has been over the line
    rng_hist = []                    # (t, range) over the last second
    gate_range_t = None              # the distance being held
    gate_rng_med = None
    pitch_deg = 0.0
    pitch_want = 0.0
    gate_z_t = None                  # vision target, None = barometer
    gate_el = gate_rng = gate_dz_m = gate_az = None
    roll_deg = 0.0
    z_t = 0.0
    hover_pwms = []
    n = 0
    try:
        while True:
            t = time.monotonic()
            est = src.estimate()
            if est is None:
                time.sleep(period); continue
            # CONTROL on the fused height, not the raw barometer. The
            # accelerometer rides through the moments prop wash makes the
            # barometer lie; the barometer stops the accelerometer drifting.
            # z_raw stays for the CEILING, because a backstop that trusts a
            # filter is trusting the thing most likely to be wrong.
            z_raw = float(est.p[2])
            z = float(getattr(src, "z_filtered", z_raw)) if not args.raw_alt else z_raw
            vz = float(est.v[2])
            # LATCHED. AltitudeLoop falls back to a fixed takeoff throttle
            # while "not airborne", which is right once, on the way up, and
            # wrong every time after. Unlatched, the landing descent drops
            # back through 0.25 m, the loop slams to takeoff_pwm (1.32 g), the
            # aircraft climbs, crosses 0.25 again and the controller resumes -
            # a limit cycle at knee height that never touches down. Visible in
            # the 2026-09-20 table test: thr pinned at 1350 for four seconds
            # while the drone was being lowered THROUGH that band by hand.
            if not airborne_latch and z > 0.25:
                airborne_latch = True
            airborne = airborne_latch
            if args.no_baro and not airborne_latch and vz > 0.7:
                airborne_latch = airborne = True   # no usable height: vz decides

            # VISION. Read EVERY tick, so a props-off dry run on the bench
            # shows el/az/dz immediately - it never reaches the hold phase,
            # because nothing lifts it. APPLIED only in hold: takeoff and
            # landing are always flown on the barometer, and roll is only ever
            # commanded with a live detection while holding.
            roll_deg = 0.0
            roll_want = 0.0
            pitch_deg = 0.0
            pitch_want = 0.0
            if camera is not None:
                det = camera.latest()
                fresh = det is not None and (t - det.t) <= args.gate_lost_s
                dz, el = gate_dz(det, est.R, args.fy, math.radians(args.cam_tilt),
                                 camera.frame_wh) if fresh else (None, None)
                if el is None:
                    if gate_z_t is not None:
                        print(f"  gate lost at t={t - t0:.1f}s: back to {args.alt:.2f} m")
                    gate_z_t = None
                    gate_el = gate_rng = gate_dz_m = gate_az = None
                else:
                    if gate_z_t is None:
                        gate_z_t = z_t if phase == "hold" else z
                        print(f"  gate acquired at t={t - t0:.1f}s: "
                              f"el={math.degrees(el):+.1f} deg")
                    # DRIVEN BY THE ELEVATION ANGLE ALONE, never by the range.
                    # Range comes from the gate's apparent size and on d45's
                    # first bench run it read 2.4, 3.4, 9.8 and then 140 m in
                    # the space of ten seconds. Multiplying a good angle by
                    # that to get a height in metres threw the whole point
                    # away: the null is range-INDEPENDENT, so the controller
                    # must be too. Climb at a rate proportional to how far
                    # above us the gate centre sits, capped at --gate-slew.
                    # A wrong angle can now only move the target slowly, and
                    # the clamp still bounds where it can end up.
                    rate = args.gate_el_gain * math.degrees(el)
                    rate = max(-args.gate_slew, min(args.gate_slew, rate))
                    gate_z_t = max(args.gate_z_min,
                                   min(args.gate_z_max, gate_z_t + rate * period))
                    gate_el, gate_rng, gate_dz_m = el, det.range_m, dz
                    # ROLL CENTRING. Tilt toward the side the gate is on and the
                    # aircraft slides that way until the gate is dead ahead - one
                    # point, not an open chase. Capped hard, and released the
                    # instant the gate is gone.
                    if args.gate_roll:
                        az = gate_azimuth(det, args.fy, math.radians(args.cam_tilt),
                                          camera.frame_wh)
                        gate_az = az
                        if az is not None:
                            roll_want = max(-args.gate_roll_tilt,
                                            min(args.gate_roll_tilt,
                                                args.gate_roll_gain * math.degrees(az)))

                    # RANGE HOLD - the third axis, and the one that keeps the
                    # aircraft the same distance from the gate.
                    #
                    # Range is the worst signal the camera produces: on a bench
                    # it read 2.4, then 9.8, then 140 m inside ten seconds. So
                    # it is median-filtered over the last second and anything
                    # outside [--gate-range-min, --gate-range-max] is thrown
                    # away before it is believed at all.
                    #
                    # ASYMMETRIC BY DEFAULT. --gate-pitch-fwd caps how much
                    # tilt may be commanded TOWARD the gate, and it defaults to
                    # ZERO: the loop may only ever back away. In a 2x2 m cage
                    # with the gate beyond a net, "too far, go forward" is the
                    # one mistake that cannot be afforded, and the aircraft's
                    # own drift is toward the gate anyway - so a hold that only
                    # pushes back is exactly the half that is useful.
                    if args.gate_pitch and det.range_m is not None:
                        r = float(det.range_m)
                        if args.gate_range_min <= r <= args.gate_range_max:
                            rng_hist.append((t, r))
                        rng_hist = [e for e in rng_hist if t - e[0] <= 1.0]
                        if len(rng_hist) >= 3:
                            rs = sorted(e[1] for e in rng_hist)
                            gate_rng_med = rs[len(rs) // 2]
                            if gate_range_t is None:
                                gate_range_t = (args.gate_range if args.gate_range
                                                else gate_rng_med)
                                print(f"  holding {gate_range_t:.2f} m from the gate "
                                      f"(backing off only)" if not args.gate_pitch_fwd
                                      else f"  holding {gate_range_t:.2f} m from the gate")
                            err_r = gate_rng_med - gate_range_t   # + means too far
                            pitch_want = args.gate_pitch_gain * err_r
                            pitch_want = max(-args.gate_pitch_tilt,
                                             min(args.gate_pitch_fwd, pitch_want))

            if phase == "climb":
                z_t = min(args.alt, z_t + args.climb * period)
                vz_ff = args.climb
                # With no usable height there is nothing to compare against, so
                # the climb ends on TIME: takeoff thrust for --climb-s, then
                # hold vertical speed at zero wherever that left it. On d45's
                # measured curve 1350 is +3.2 m/s^2, so 0.45 s puts it about
                # 0.3 m up and climbing at 1.4 m/s, which the speed hold then
                # arrests. Longer is higher; this is the only altitude control
                # there is in this mode.
                if args.no_baro:
                    if t - t0 >= args.climb_s:
                        phase, t_phase = "hold", t
                        print(f"  climb done after {args.climb_s:.2f} s, "
                              f"holding vertical speed at zero (vz={vz:+.2f})")
                elif z >= args.alt - 0.15:
                    phase, t_phase, z_t = "hold", t, args.alt
                    print(f"  at altitude ({z:.2f} m) after {t - t0:.1f} s, holding")
            elif phase == "hold":
                z_t, vz_ff = args.alt, 0.0
                if gate_z_t is not None:
                    z_t = gate_z_t
                    roll_deg = roll_want
                    pitch_deg = pitch_want
                if t - t_phase > args.seconds:
                    phase, t_phase = "descend", t
                    print(f"  hold done, descending")
            else:
                z_t = max(0.0, z_t - args.descend * period)
                vz_ff = -args.descend

            if args.no_baro:
                # NO BAROMETER IN THE LOOP AT ALL. d45, 2026-09-20: with props
                # running the barometer read -2.08 m while the aircraft sat at
                # about 0.3, held that for 0.4 s, then jumped to +0.85. No
                # filter, clamp or rejection rule survives a sensor that wrong;
                # they only bound how badly it fails.
                #
                # The accelerometer is fine - vz read 1.3, 2.1, 3.1, 4.7 m/s
                # through that same launch and every value was right. So hold
                # vertical SPEED at zero instead of height. That is still a
                # closed loop, just on the signal that works.
                #
                # It cannot hold a height: integrated accelerometer bias walks,
                # about 0.4 m/s per 20 s at the -0.02 m/s^2 measured at rest.
                # The aircraft will drift up or down slowly and the pilot flies
                # it back. That is the trade - a drift you can see and correct,
                # instead of a number that lies by two metres.
                if not airborne:
                    thr = int(cfg.follower.takeoff_pwm)
                    alt.a_cmd, alt.thrust = 0.0, 0.0
                else:
                    # VISION WITHOUT ANY HEIGHT AT ALL. With --gate-z the
                    # elevation to the gate sets the vertical SPEED to fly, and
                    # the accelerometer measures that speed. No barometer
                    # appears anywhere in this loop: not as a height, not as a
                    # target, not as a correction.
                    #
                    # This is what makes the vision hold independent. --gate-z
                    # on its own still turns the null into a height target and
                    # then uses the barometer to reach it, so it inherits
                    # whatever the sensor is doing. Here it does not.
                    if phase == "hold" and gate_el is not None:
                        vz_ff = max(-args.gate_slew,
                                    min(args.gate_slew,
                                        args.gate_el_gain * math.degrees(gate_el)))
                    a_cmd = args.vz_gain * (vz_ff - vz)
                    a_cmd = max(-4.0, min(4.0, a_cmd))
                    cos_tilt = max(float(est.R[2, 2]), 0.25)
                    thrust = max(0.0, (9.80665 + a_cmd) / cos_tilt)
                    alt.a_cmd, alt.thrust = a_cmd, thrust
                    thr = int(round(max(cfg.thrust.pwm_min,
                                        min(cfg.thrust.pwm_max,
                                            cfg.pwm_for_thrust(thrust)))))
            else:
                # Hand the loop the FUSED height. est.p[2] is the raw
                # barometer, which is what AltitudeLoop closes on, and on this
                # airframe it spikes by a metre with props running. Setting it
                # here is the only place that reaches the controller - a
                # previous attempt set a local and changed nothing.
                est.p[2] = z
                thr = alt.throttle(t - t0, est, z_t, vz_ff, airborne, integrate=True)
            # RUNAWAY GUARD on vertical SPEED, which is the signal we trust.
            # With no height to compare against, a climb that keeps climbing is
            # the only symptom available.
            if airborne and abs(vz) > args.vz_max:
                vz_bad += period
                if vz_bad > 0.4:
                    print("")
                    print(f"  VERTICAL SPEED RUNAWAY: {vz:+.1f} m/s for 0.4 s "
                          f"(limit {args.vz_max:.1f}). Throttle to minimum, disarming.")
                    br.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500,
                              arm=1000, aux2=1500)
                    break
            else:
                vz_bad = 0.0
            # In --no-baro there is no height to be near, and demanding one
            # threw away the only number these flights exist to measure:
            # d44 held vz inside 0.07 m/s for five seconds at 1224-1229 PWM on
            # 2026-09-21 and the summary still said "never settled at
            # altitude". Steady VERTICAL SPEED is the hover condition here.
            if phase == "hold" and abs(vz) < 0.15 and (
                    args.no_baro or abs(z - args.alt) < 0.15):
                hover_pwms.append(thr)

            # HARD CEILING, and which number it reads matters.
            #
            # It used to trip on EITHER the raw barometer or the fused height,
            # on the argument that a backstop should not trust a filter. That
            # is right when the filter is the suspect and wrong when the SENSOR
            # is: d44, 2026-09-21, aborted a good flight at "1.76 m raw / 0.91
            # filtered" - the barometer spiked a metre with the props running
            # and the fusion correctly ignored it. A backstop that fires on
            # sensor noise instead of runaways is worse than none, because
            # people start flying through it.
            #
            # So: the FUSED height trips immediately, because it is smooth and
            # cannot spike. The raw barometer still trips - that is the check
            # on a filter gone wrong - but only when it has been over the line
            # continuously for --ceiling-hold. A real climb stays above it; a
            # prop-wash spike does not.
            if not args.no_baro:
                ceil_bad = ceil_bad + period if z_raw > ceiling else 0.0
                why = ("fused" if z > ceiling else
                       "raw, sustained" if ceil_bad > args.ceiling_hold else None)
                if why:
                    print("")
                    print(f"  CEILING HIT ({why}): z={z_raw:.2f} m raw / "
                          f"{z:.2f} filtered > {ceiling:.2f}. "
                          f"Throttle to minimum, disarming.")
                    br.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500,
                              arm=1000, aux2=1500)
                    break

            # Landing with no height: descend at --descend for as long as the
            # climb took plus a margin, then cut. It cannot know it has touched
            # down, so it errs on the side of still being low when it cuts.
            if args.no_baro:
                done = phase == "descend" and t - t_phase > (args.climb_s + 2.5)
            else:
                done = phase == "descend" and z < 0.15 and t - t_phase > 2.0
            # roll_deg is zero unless --gate-roll has a live detection in hold
            roll_stick = 1500 + int(round(500.0 * roll_deg / cfg.follower.angle_limit_deg))
            roll_stick = max(1400, min(1600, roll_stick))    # belt and braces
            # pitch: POSITIVE stick is nose-down = forward on this firmware, so a
            # command to back away is a NEGATIVE angle and a stick below 1500
            pitch_stick = 1500 + int(round(500.0 * pitch_deg / cfg.follower.angle_limit_deg))
            pitch_stick = max(1400, min(1600, pitch_stick))
            out = dict(throttle=(1000 if done else thr),
                       roll=(1500 if done else roll_stick),
                       pitch=(1500 if done else pitch_stick),
                       yaw=1500, arm=(1000 if done else 1800), aux2=1500)
            if args.arm:
                br.set_rc(**out)
            else:
                br.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500, arm=1000, aux2=1500)

            s = br.state()
            n += 1
            if n % 5 == 0:
                w.writerow([f"{t - t0:.3f}", phase, f"{z:.3f}", f"{vz:.3f}", f"{z_t:.3f}",
                            f"{alt.a_cmd:.2f}", f"{alt.thrust:.2f}", out["throttle"],
                            out["roll"], out["pitch"], f"{math.degrees(est.yaw):.1f}",
                            f"{s.attitude.roll_deg:.2f}" if s.attitude else "",
                            f"{s.attitude.pitch_deg:.2f}" if s.attitude else "",
                            int(bool(s.status and s.status.armed)),
                            f"{s.battery.voltage_v:.2f}" if s.battery and s.battery.voltage_v else "",
                            f"{s.battery.current_a:.1f}" if s.battery and s.battery.current_a else "",
                            1 if gate_el is not None else 0,
                            f"{math.degrees(gate_el):.2f}" if gate_el is not None else "",
                            f"{gate_rng:.2f}" if gate_rng is not None else "",
                            f"{gate_dz_m:.3f}" if gate_dz_m is not None else ""])
            if n % int(args.rc_hz) == 0:
                log.flush()
                vis = ""
                if camera is not None:
                    vis = (f" | GATE el={math.degrees(gate_el):+5.1f}"
                           + (f" rng={gate_rng:5.1f}" if gate_rng is not None else " rng=  -- ")
                           + (f" az={math.degrees(gate_az):+5.1f} roll={roll_deg:+4.1f}"
                              if gate_az is not None else "")
                           + (f" hold={gate_range_t:4.1f} pitch={pitch_deg:+4.1f}"
                              if gate_range_t is not None else "")
                           if gate_el is not None
                           else f" | gate: none ({camera.detections}/{camera.frames})")
                print(f"t={t - t0:5.1f} {phase:8s} z={z:5.2f} (target {z_t:4.2f}) vz={vz:+5.2f} "
                      f"thr={out['throttle']:4d} a_cmd={alt.a_cmd:+5.2f}{vis}")
            if done:
                print("on the ground, disarmed")
                break
            if not args.arm and t - t0 > args.seconds + 20:
                print("dry run complete"); break
            if t - t0 > args.seconds + 90:
                print("HARD STOP: took too long, disarming"); break
            if not s.healthy and args.arm:
                print("FC link unhealthy: holding throttle at minimum, pilot take over")
                br.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500, arm=1000, aux2=1500)
                break
            if args.arm and s.status is not None and not s.status.msp_override:
                print("pilot took MSP OVERRIDE off: stopping"); break
            time.sleep(max(0.0, period - (time.monotonic() - t)))
    except KeyboardInterrupt:
        print("\ninterrupted: throttle to minimum, disarming")
    finally:
        for _ in range(10):
            br.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500, arm=1000, aux2=1500)
            time.sleep(0.02)
        br.stop()
        log.close()
        if camera is not None:
            camera.stop()
            if camera.error:
                print(f"  camera: {camera.error}")
            else:
                print(f"  camera: {camera.detections}/{camera.frames} frames had a gate "
                      f"({camera.fps:.0f} fps)")

    if hover_pwms:
        hover_pwms.sort()
        med = hover_pwms[len(hover_pwms) // 2]
        print(f"\n  HOVER THROTTLE = {med} PWM   ({len(hover_pwms)} steady samples)")
        print(f"  config says hover_pwm = {cfg.thrust.hover_pwm}")
        print(f"  difference {med - cfg.thrust.hover_pwm:+d} PWM")
        if abs(med - cfg.thrust.hover_pwm) > 40:
            print("  >40 PWM out: the thrust curve does not match this aircraft.")
            print("  Send me the log and I will rescale it.")
    else:
        print("\n  no steady hover samples - it never settled at altitude")
    print(f"  log: {log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
