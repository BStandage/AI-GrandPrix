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

    Returns (dz_m, elevation_rad) or (None, None) without a usable range."""
    if det is None or det.range_m is None or wh is None:
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
    return det.range_m * up, math.asin(up)


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
    ap.add_argument("--acc-lsb-per-g", default="auto")
    ap.add_argument("--pitch-nose-down-positive", action="store_true")
    ap.add_argument("--takeoff-pwm", type=int, default=None,
                    help="throttle held until the aircraft is climbing at 0.7 m/s. "
                         "The config default (1700) was tuned on the 0.8 kg sim plant; "
                         "on the measured curve it is 3.4 g and the aircraft leaps. "
                         "1350 is about 1.3 g, which lifts off gently. Default: the config.")
    ap.add_argument("--gate-z", action="store_true",
                    help="hold the altitude of the gate centre in view, not --alt")
    ap.add_argument("--camera", default=None, help="GStreamer pipeline or /dev/videoN")
    ap.add_argument("--fy", type=float, default=830.0, help="focal length in px at the capture size")
    ap.add_argument("--cam-tilt", type=float, default=20.0, help="camera up-tilt above body forward, deg")
    ap.add_argument("--gate-z-min", type=float, default=0.40, help="floor on the vision target, m")
    ap.add_argument("--gate-z-max", type=float, default=1.80, help="ceiling on the vision target, m")
    ap.add_argument("--gate-lost-s", type=float, default=0.50,
                    help="no detection for this long -> back to the barometer target")
    ap.add_argument("--gate-slew", type=float, default=0.40, help="max target move, m/s")
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
    if args.pitch_nose_down_positive:
        src.pitch_sign = -1.0

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
    src.zero_altitude()
    print(f"altitude zeroed. hover target {args.alt:.2f} m for {args.seconds:.0f} s")

    log_path = AIGP_REPO / "out" / "flightlogs" / f"hover_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = open(log_path, "w", newline="", encoding="utf-8")
    w = csv.writer(log)
    w.writerow(["t", "phase", "z", "vz", "z_target", "a_cmd", "thrust_cmd",
                "throttle", "roll", "pitch", "yaw_deg", "armed", "vbat", "amps",
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
                print(f"armed, MSP OVERRIDE on, modes {', '.join(st.active_modes)}: lifting off")
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
    period = 1.0 / args.rc_hz
    t0 = time.monotonic()
    phase, t_phase = "climb", t0
    airborne_latch = False
    gate_z_t = None                  # vision target, None = barometer
    gate_el = gate_rng = gate_dz_m = None
    z_t = 0.0
    hover_pwms = []
    n = 0
    try:
        while True:
            t = time.monotonic()
            est = src.estimate()
            if est is None:
                time.sleep(period); continue
            z, vz = float(est.p[2]), float(est.v[2])
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

            if phase == "climb":
                z_t = min(args.alt, z_t + args.climb * period)
                vz_ff = args.climb
                if z >= args.alt - 0.15:
                    phase, t_phase, z_t = "hold", t, args.alt
                    print(f"  at altitude ({z:.2f} m) after {t - t0:.1f} s, holding")
            elif phase == "hold":
                z_t, vz_ff = args.alt, 0.0
                # VISION ALTITUDE. Only here - never during takeoff or landing.
                if camera is not None:
                    det = camera.latest()
                    fresh = det is not None and (t - det.t) <= args.gate_lost_s
                    dz, el = gate_dz(det, est.R, args.fy,
                                     math.radians(args.cam_tilt), camera.frame_wh)                         if fresh else (None, None)
                    if dz is None:
                        if gate_z_t is not None:
                            print(f"  gate lost at t={t - t0:.1f}s: back to {args.alt:.2f} m")
                        gate_z_t, gate_el, gate_rng, gate_dz_m = None, None, None, None
                    else:
                        want = z + dz                     # where the gate centre is
                        want = max(args.gate_z_min, min(args.gate_z_max, want))
                        if gate_z_t is None:
                            gate_z_t = z_t                # take over from where we are
                            print(f"  gate acquired at t={t - t0:.1f}s: "
                                  f"el={math.degrees(el):+.1f} deg rng={det.range_m:.1f} m")
                        # slew limit: a bad frame moves the target a little, never a lot
                        step = args.gate_slew * period
                        gate_z_t += max(-step, min(step, want - gate_z_t))
                        gate_el, gate_rng, gate_dz_m = el, det.range_m, dz
                    if gate_z_t is not None:
                        z_t = gate_z_t
                if t - t_phase > args.seconds:
                    phase, t_phase = "descend", t
                    print(f"  hold done, descending")
            else:
                z_t = max(0.0, z_t - args.descend * period)
                vz_ff = -args.descend

            thr = alt.throttle(t - t0, est, z_t, vz_ff, airborne, integrate=True)
            if phase == "hold" and abs(z - args.alt) < 0.15 and abs(vz) < 0.2:
                hover_pwms.append(thr)

            done = phase == "descend" and z < 0.15 and t - t_phase > 2.0
            out = dict(throttle=(1000 if done else thr), roll=1500, pitch=1500,
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
                    vis = (f" | GATE el={math.degrees(gate_el):+5.1f} rng={gate_rng:4.1f} "
                           f"dz={gate_dz_m:+5.2f}" if gate_el is not None
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
