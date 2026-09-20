"""
The on-drone runtime: camera -> detector -> state -> pilot -> flight controller.

    cd src
    python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north 30 --pilot follower --traj ../out/plans/plan_LADDER_60s.json --dry-run
    python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north 30 --pilot follower --traj ../out/plans/plan_LADDER_60s.json --arm
    python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north 30 --pilot seeker --arm

--pilot follower  the race stack on vision-aided dead reckoning: the plan
                  (--traj, with the toml it records or --config) flown by
                  solvers.follower on a state estimate integrated from the
                  FC's IMU and attitude, altitude from its barometer, and a
                  position fix from every gate the detector sees. The
                  estimator counts its own gate crossings; the nose aims at
                  the next gate. ANGLE-mode sticks (AIGP_ANGLE_MODE=1).
--pilot seeker    the fallback: gate to gate on heading, baro and detections.
--dry-run         everything runs, the FC receives neutral sticks, the
                  commands are printed and logged. Nothing can spin.
--arm             the real thing. MSP override covers the four sticks only
                  (msp_override_channels_mask = 15): the PILOT arms, flips
                  MSP OVERRIDE and ANGLE on the transmitter, and can take the
                  sticks back or disarm at any moment. The runtime waits for
                  ARMED + MSP OVERRIDE before the plan clock starts.

Day-1 inputs: --map-north (the FC heading of the map's +y; pass `here` with
the drone on the start line pointing along gate 1 and the heading is read at
startup, which also works with no magnetometer), --acc-lsb-per-g (measured
at rest by default), --fy and --cam-tilt and --cam-hfov (from hardware.camcal).
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

DEFAULT_PIPELINE = ("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1280,height=720,"
                    "framerate=60/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
                    "video/x-raw,format=BGR ! appsink drop=true max-buffers=1")
GATE_OUTER_M = 2.7


class CameraThread(threading.Thread):
    """Grabs frames, runs the HSV detector, keeps the latest detection with a
    range estimate from the ring's pixel height."""

    def __init__(self, source: str, fy_px: float):
        super().__init__(name="camera", daemon=True)
        self.source = source
        self.fy = fy_px
        self.det = None
        self.dets = []              # up to 3 blobs, biggest first, for the estimator to choose from
        self.frames = 0
        self.detections = 0
        self.fps = 0.0
        self.error = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

    def latest(self):
        with self._lock:
            return self.det

    def latest_all(self):
        with self._lock:
            return list(self.dets)

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            import cv2
            from perception.detectors.hsv_classic import gate_mask
            from perception.gate_detection import mask_to_detections
            from seeker.brain import Detection
        except ImportError as e:
            self.error = f"cv2/detector import failed: {e}"
            return
        cap = cv2.VideoCapture(self.source, cv2.CAP_V4L2 if self.source.startswith("/dev/video") else cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self.error = f"camera did not open: {self.source}"
            return
        t_win, n_win = time.monotonic(), 0
        while not self._stop.is_set():
            ok, bgr = cap.read()
            if not ok or bgr is None:
                time.sleep(0.005)
                continue
            t = time.monotonic()
            self.frames += 1
            n_win += 1
            gs = mask_to_detections(gate_mask(bgr), bgr.shape)
            out = []
            h, w = bgr.shape[:2]
            for g in gs[:3]:
                area = float(g.area_frac or g.area / float(h * w))
                rng = None
                box = g.ring_bbox or g.bbox
                if box is not None and box[2] > 4:
                    # pinhole: range = f * W / w_px. The WIDTH: the real gate is a
                    # square frame with a header board on top (organizer DVR,
                    # 2026-09-16), so its height is not the opening's size
                    rng = self.fy * GATE_OUTER_M / float(box[2])
                out.append(Detection(offset_x=float(g.offset_x), offset_y=float(g.offset_y), area_frac=area, t=t, range_m=rng))
            if out:
                self.detections += 1
            with self._lock:
                self.dets = out
                self.det = out[0] if out else None
            if t - t_win >= 1.0:
                self.fps = n_win / (t - t_win)
                t_win, n_win = t, 0
        cap.release()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--port", default=None)
    ap.add_argument("--tcp", default=None, help="Betaflight SITL host[:port] instead of a serial port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--pilot", choices=("follower", "seeker"), default="follower")
    ap.add_argument("--traj", default=None, help="plan JSON for the follower (default out/plans/plan_RACE.json)")
    ap.add_argument("--config", default=None, help="vehicle.toml (default: the plan's, else config/vehicle.toml)")
    ap.add_argument("--camera", default=DEFAULT_PIPELINE, help="GStreamer pipeline or /dev/videoN")
    ap.add_argument("--no-camera", action="store_true", help="run without a camera (dead reckoning only)")
    ap.add_argument("--fy", type=float, default=1000.0, help="camera focal length in pixels at the capture resolution")
    ap.add_argument("--cam-tilt", type=float, default=20.0, help="camera mount tilt above body forward, deg")
    ap.add_argument("--cam-hfov", type=float, default=90.0, help="camera horizontal field of view, deg")
    ap.add_argument("--map-north", required=True,
                    help="FC heading (deg) of the map's +y axis, or `here`: the drone is on the start line "
                         "pointing along gate 1 and the heading is read at startup")
    ap.add_argument("--laps", type=int, default=2)
    ap.add_argument("--rc-hz", type=float, default=50.0)
    ap.add_argument("--acc-lsb-per-g", default="auto",
                    help="raw accelerometer counts per g (2048 on the Archer per its blackbox, 256 on the SITL); "
                         "'auto' measures |acc| over 1 s at rest before takeoff")
    ap.add_argument("--pitch-nose-down-positive", action="store_true")
    ap.add_argument("--heading-drift-dpm", type=float, default=0.0,
                    help="measured gyro heading drift in deg/min (from `bench drift`), "
                         "subtracted linearly over the run. d45 measured +3.0 on the floor. "
                         "0 = no correction")
    ap.add_argument("--angle-mode", action="store_true", default=True, help="ANGLE-mode sticks (default on)")
    ap.add_argument("--acro", action="store_true", help="rate sticks through our attitude loop instead of ANGLE mode")
    ap.add_argument("--max-s", type=float, default=240.0, help="hard stop: land and disarm after this many seconds")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--dry-run", action="store_true")
    g.add_argument("--arm", action="store_true")
    args = ap.parse_args(argv)

    # module-level configuration of the pilots happens at import time
    repo = Path(__file__).resolve().parents[2]
    traj = Path(args.traj) if args.traj else repo / "out" / "plans" / "plan_RACE.json"
    os.environ["AIGP_TRAJ"] = str(traj)
    if args.config:
        os.environ["AIGP_VEHICLE_TOML"] = str(Path(args.config))
    os.environ["AIGP_ANGLE_MODE"] = "0" if args.acro else "1"
    os.environ["AIGP_STATE_SOURCE"] = "deadreckon"
    os.environ["AIGP_LAPS"] = str(args.laps)
    os.environ["AIGP_CAM_TILT_DEG"] = str(args.cam_tilt)
    os.environ["AIGP_CAM_HFOV_DEG"] = str(args.cam_hfov)

    from raceline.config import load_config, AIGP_REPO
    from raceline import course as course_bridge
    from raceline.rc_backend import StateEstimate
    from hardware.bridge import FcBridge
    from hardware.state import FcStateSource
    from seeker.dr_estimator import DeadReckonSource
    from seeker import synthetic_camera as cam

    cfg = load_config(os.environ.get("AIGP_VEHICLE_TOML"))
    course = course_bridge.load_course(laps=args.laps)

    if args.pilot == "follower":
        import solvers.follower as fol
        dr = fol._SOURCE                                    # the DeadReckonSource the follower built
        landmarks = fol._GATE_LANDMARKS                     # every gate: the estimator decides which one it sees
    else:
        from seeker.pilot import SeekerPilot, crossings_from_course
        from seeker.dr_estimator import VerticalFilter
        pilot = SeekerPilot(cfg, crossings_from_course(course), start_xy=(0.0, 0.0), laps=args.laps, t0=time.monotonic())
        dr = None
        vert = VerticalFilter()
        t_vert = None

    bridge = FcBridge.open(port=args.port, tcp=args.tcp, baud=args.baud, rc_hz=args.rc_hz)
    bridge.start()
    src = FcStateSource(bridge, map_north_heading_deg=0.0,
                        pitch_nose_up_positive=not args.pitch_nose_down_positive,
                        acc_lsb_per_g=512.0 if args.acc_lsb_per_g == "auto" else float(args.acc_lsb_per_g))
    camera = None if args.no_camera else CameraThread(args.camera, args.fy)
    if camera:
        camera.start()

    t_wait = time.monotonic()
    while bridge.state().attitude is None or bridge.state().imu is None:
        if time.monotonic() - t_wait > 5.0:
            print("ERROR no attitude/IMU from the FC after 5 s"); bridge.stop(); return 3
        time.sleep(0.05)
    while bridge.state().altitude is None and time.monotonic() - t_wait < 5.0:
        time.sleep(0.05)
    # The bridge polls attitude every tick but altitude on a slower rotation,
    # so it can still be None several hundred ms in. Checking too early reads
    # as "no barometer" on an aircraft that has one (hit on d45, 2026-09-20).
    _deadline = time.monotonic() + 10.0
    while bridge.state().altitude is None and time.monotonic() < _deadline:
        time.sleep(0.05)
    if bridge.state().altitude is None:
        print("ERROR no MSP_ALTITUDE after 10 s: check `fc-info` lists BARO"); bridge.stop(); return 3
    src.zero_altitude()
    st0 = bridge.state().status
    has_mag = bool(st0 and (st0.sensors & 0x04))
    if args.map_north.strip().lower() == "here":
        src.map_north_heading_deg = float(bridge.state().attitude.yaw_deg)
        print(f"map north = FC heading now: {src.map_north_heading_deg:.1f} deg (the drone is on the start line pointing along gate 1)")
    src.heading_drift_dpm = float(args.heading_drift_dpm)
    if src.heading_drift_dpm:
        print(f"heading drift correction: {src.heading_drift_dpm:+.1f} deg/min taken out over the run")
    else:
        src.map_north_heading_deg = float(args.map_north)
        if not has_mag:
            print("WARNING no magnetometer: the FC heading is gyro-integrated and restarts at an arbitrary value "
                  "every boot, so a numeric --map-north is only valid in the boot it was read in. Use --map-north here.")
    print(f"magnetometer: {'present' if has_mag else 'ABSENT (heading drifts: measure it with hardware.bench drift)'}")
    if args.acc_lsb_per_g == "auto":
        # the drone is level and still: |acc| is exactly 1 g in raw counts
        mags = []
        t_cal = time.monotonic()
        while time.monotonic() - t_cal < 1.0:
            st = bridge.state()
            if st.imu is not None:
                mags.append(math.sqrt(sum(float(v) ** 2 for v in st.imu.acc)))
            time.sleep(0.02)
        if mags:
            med, spread = float(np.median(mags)), max(mags) - min(mags)
            if spread < 0.1 * med:
                src.acc_lsb_per_g = med
                print(f"accelerometer: {med:.0f} raw counts per g measured at rest ({len(mags)} samples, spread {spread:.0f})")
            else:
                print(f"WARNING accelerometer not at rest (median {med:.0f}, spread {spread:.0f}): keeping "
                      f"{src.acc_lsb_per_g:.0f} counts per g. Put the drone down still and restart, or pass --acc-lsb-per-g.")
    s = bridge.state()
    e0 = src.estimate()
    print(f"FC link: attitude {s.attitude_hz:.0f} Hz, rc {s.rc_hz:.0f} Hz, rtt {s.link.last_rtt_ms:.1f} ms; "
          f"heading {s.attitude.yaw_deg:.0f} -> world yaw {math.degrees(e0.yaw):.0f} deg; baro zeroed at {src.alt_offset_m:.2f} m; "
          f"accel at rest {np.round(src.accel_body, 2) if src.accel_body is not None else '?'} (expect ~[0 0 9.8])")
    if camera:
        time.sleep(1.0)
        if camera.error:
            print(f"ERROR camera: {camera.error}"); bridge.stop(); return 4
        print(f"camera: {camera.fps:.0f} fps, detections so far {camera.detections}")
    if s.status is not None and s.status.arming_blockers and args.arm:
        print(f"WARNING arming blockers: {', '.join(s.status.arming_blockers)}")

    log_path = AIGP_REPO / "out" / "flightlogs" / f"hw_{args.pilot}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = open(log_path, "w", newline="", encoding="utf-8")
    w = csv.writer(log)
    # cross_* / fix_*_sum are the debrief columns: what the drone can measure
    # about its own error with no ground truth (raceline.debrief reads them)
    w.writerow(["t", "phase_or_event", "x", "y", "z", "vx", "vy", "vz", "yaw_deg", "det_x", "det_y", "det_area", "det_range",
                "fixes", "fix_res", "rej", "unm", "cross_ev", "cross_lat", "cross_dz", "fix_cx_sum", "fix_al_sum",
                "throttle", "roll", "pitch", "yaw", "arm", "att_hz", "rtt_ms", "timeouts", "cam_fps", "vbat"])
    print(f"log -> {log_path}")
    if args.dry_run:
        print("DRY RUN: commands computed and logged; the FC receives neutral sticks")
    else:
        print("LIVE: waiting for the pilot. Throttle low, ARM, then MSP OVERRIDE on (and ANGLE). "
              "The plan clock starts when the FC reports both.")
        while True:
            st = bridge.state().status
            bridge.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500, arm=1000, aux2=1500)
            if st is not None and st.armed and st.msp_override:
                print(f"armed, MSP OVERRIDE on, modes {', '.join(st.active_modes)}: flying")
                break
            time.sleep(0.05)
        if not bridge.state().status.angle_mode and not args.acro:
            print("WARNING ANGLE mode is not active on the FC: the sticks are angle sticks. Flip the ANGLE switch.")

    period = 1.0 / args.rc_hz
    t_start = time.monotonic()
    n = 0
    fixes = 0
    fix_res = 0.0
    t_fix = -1.0
    airborne_latch = False    # see the on_ground note in the loop
    try:
        while True:
            t = time.monotonic()
            est = src.estimate()
            if est is None:
                time.sleep(period); continue
            det = camera.latest() if camera else None
            dets = camera.latest_all() if camera else []
            if det is not None and t - det.t > 0.25:
                det, dets = None, []
            if args.pilot == "follower":
                # state: integrate the FC's IMU on its attitude; altitude from
                # the accel + baro filter fed by the FC's altitude
                if src.accel_body is not None:
                    # ON THE GROUND it is not moving, so hold velocity at zero
                    # rather than integrate the accelerometer's bias while it
                    # waits to be armed (0.15 m/s^2 measured on d45 = 1.9 m of
                    # phantom position after 5 s, 200 m after a minute).
                    #
                    # Height alone cannot decide this: the barometer drifts
                    # ~0.25 m per minute at rest, so any fixed threshold is
                    # eventually crossed while the aircraft sits still. Require
                    # a real climb as well, and LATCH it - once genuinely
                    # airborne we never go back, because a descent through the
                    # threshold mid-flight must not freeze the estimate.
                    if not airborne_latch and float(est.p[2]) > 0.30 and float(est.v[2]) > 0.5:
                        airborne_latch = True
                        print(f"airborne at t={t - t_start:.1f}s "
                              f"(z={float(est.p[2]):.2f} vz={float(est.v[2]):+.2f})")
                    dr.integrate(t, est.R, src.accel_body, float(est.p[2]), True,
                                 on_ground=not airborne_latch)
                est_dr = StateEstimate(p=dr.p.copy(), v=dr.v.copy(), R=est.R, yaw=est.yaw, omega=est.omega)
                # a sighting: the estimator decides which gate it is (same code
                # as the sim) and fixes on it; implausible fixes are dropped
                if dets and t - t_fix >= 1.0 / 30.0:
                    t_fix = t
                    idx, r = dr.observe_any(dets, landmarks)
                    if idx is not None:
                        fixes += 1; fix_res = r
                        est_dr.p[:] = dr.p; est_dr.v[:] = dr.v
                rc = fol.step(t - t_start, est_dr, dr.next_event, True, "")
                out = dict(throttle=rc.throttle, roll=rc.roll, pitch=rc.pitch, yaw=rc.yaw, arm=rc.arm, aux2=rc.aux2)
                label = f"ev{dr.next_event}" + (f"/lm{dr.last_landmark}" if dr.last_landmark is not None else "")
                done = rc.arm == 1000 and t - t_start > 5.0
                est_log = est_dr
            else:
                # altitude through the same accel + baro filter as the follower
                dt_v = 0.0 if t_vert is None else max(0.0, min(0.05, t - t_vert))
                t_vert = t
                az_w = float((est.R @ src.accel_body)[2]) - 9.80665 if src.accel_body is not None else 0.0
                z_f, vz_f = vert.update(dt_v, az_w, float(est.p[2]), True)
                est.p[2] = z_f; est.v[2] = vz_f
                sk = pilot.tick(t, est, det, baro_fresh=True)
                out = dict(throttle=sk.throttle, roll=sk.roll, pitch=sk.pitch, yaw=sk.yaw, arm=sk.arm, aux2=sk.aux2)
                label = sk.phase
                done = sk.done
                est_log = est
            if t - t_start > args.max_s and not done:
                out = dict(throttle=1000, roll=1500, pitch=1500, yaw=1500, arm=1000, aux2=out["aux2"]); done = True
            if args.arm:
                bridge.set_rc(**out)
            else:
                bridge.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500, arm=1000, aux2=out["aux2"])
            s = bridge.state()
            n += 1
            if n % 5 == 0:
                w.writerow([f"{t - t_start:.3f}", label, f"{est_log.p[0]:.2f}", f"{est_log.p[1]:.2f}", f"{est_log.p[2]:.2f}",
                            f"{est_log.v[0]:.2f}", f"{est_log.v[1]:.2f}", f"{est_log.v[2]:.2f}", f"{math.degrees(est_log.yaw):.1f}",
                            f"{det.offset_x:.3f}" if det else "", f"{det.offset_y:.3f}" if det else "",
                            f"{det.area_frac:.4f}" if det else "", f"{det.range_m:.2f}" if det and det.range_m else "",
                            fixes, f"{fix_res:.2f}",
                            dr.rejected if dr is not None else "", dr.unmatched if dr is not None else "",
                            dr.cross_ev if dr is not None else "",
                            f"{dr.cross_lat:.3f}" if dr is not None else "",
                            f"{dr.cross_dz:.3f}" if dr is not None else "",
                            f"{dr.fix_cross_sum:.3f}" if dr is not None else "",
                            f"{dr.fix_along_sum:.3f}" if dr is not None else "",
                            out["throttle"], out["roll"], out["pitch"], out["yaw"], out["arm"],
                            f"{s.attitude_hz:.0f}", f"{s.link.last_rtt_ms:.1f}", s.link.timeouts,
                            f"{camera.fps:.0f}" if camera else "", f"{s.battery.voltage_v:.2f}" if s.battery and s.battery.voltage_v else ""])
            if n % int(args.rc_hz) == 0:
                log.flush()
                print(f"t={t - t_start:6.1f} {label:7s} p=({est_log.p[0]:+5.1f},{est_log.p[1]:+5.1f},{est_log.p[2]:4.2f}) "
                      f"yaw={math.degrees(est_log.yaw):5.0f} det={'%.2fm' % det.range_m if det and det.range_m else '-':>6} "
                      f"fixes={fixes} res={fix_res:5.2f}"
                      + (f" rej={dr.rejected} unm={dr.unmatched} " if dr is not None else " ") +
                      f"stk=({out['roll']},{out['pitch']},{out['throttle']},{out['yaw']}) arm={out['arm']} "
                      f"link {s.attitude_hz:.0f}Hz/{s.link.last_rtt_ms:.0f}ms healthy={s.healthy}")
            if done:
                print("run complete: disarmed")
                break
            if not s.healthy and args.arm:
                print("FC link unhealthy: throttle is being held at minimum; pilot, take over")
                break
            if args.arm and s.status is not None and s.status.box_names and not s.status.msp_override:
                print("MSP OVERRIDE switched off: the pilot has the sticks; stopping")
                break
            time.sleep(max(0.0, period - (time.monotonic() - t)))
    except KeyboardInterrupt:
        print("interrupted: disarming")
    finally:
        log.close()
        if camera:
            camera.stop()
        bridge.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
