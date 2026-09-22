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
SOURCE_FPS = 60.0        # DEFAULT_PIPELINE requests framerate=60/1


# PLAUSIBILITY (d44 flight 4, 2026-09-21). Range is the ring's WIDTH run
# through a pinhole, so a detector that latches onto something that is not the
# gate - a fragment of the ring after contact, scenery, an orange cone - does
# not degrade, it reports nonsense. That flight logged 1.81 m, then 154.62,
# then 6.02, then 12.60, then 16.11 on consecutive frames 0.1 s apart, and
# every one of them was handed to observe_any for a POSITION FIX and to the
# vertical channel. The throttle went to its 1450 clamp.
#
# A gate cannot change range faster than we can fly. Reject the frame instead.
# After REJECT_RESET_S of nothing plausible, give up on the old track and let
# the next detection start a fresh one - otherwise one bad lock-on blinds us
# for the rest of the run.
RANGE_JUMP_MAX_MPS = 12.0     # v_max on the fastest rung is 5.0
RANGE_SANE_MAX_M = 30.0       # no gate beyond this is actionable
REJECT_RESET_S = 1.0
# How stale an accepted map match may be before the gate's elevation stops
# being allowed to steer height.
#
# 0.75, not 0.5. Measured on the organizers' lap footage (archer_AIGP.mp4,
# 2854 frames of a real course): the plausibility filter's longest run of
# consecutive rejections was 28 frames, 0.47 s. At 0.5 s the vertical channel
# would have dropped out on the very next frame of that run, and a threshold
# whose margin is one frame is not a threshold. 0.75 s clears the worst
# observed gap by 60 %.
#
# The cost of the extra 0.25 s is bounded and small: the reference is
# slew-limited at VERT_DZ_SLEW = 0.35 m/s, so the most a stale match can move
# the aircraft before it fades is 0.09 m.
MATCH_STALE_S = 0.75
# Consecutive frames reading "commit" before the latch takes. Commit SNAPS the
# reference to zero, so a single spurious frame costs a full re-slew back up at
# VERT_DZ_SLEW - up to a second. d43's dry run committed at 10.5 m on scenery
# via width_clipped, so spurious commits are not hypothetical. Three frames is
# ~90 ms at the control rate: too short to matter, long enough to need a real
# gate rather than one bad blob.
COMMIT_CONFIRM = 3
COMMIT_MAX_DIST_M = 6.0   # a commit needs the map within this of the next gate (see the latch)


class CameraThread(threading.Thread):
    """Grabs frames, runs the HSV detector, keeps the latest detection with a
    range estimate from the ring's pixel height."""

    def __init__(self, source: str, fy_px: float, record_path=None, record_step: int = 1):
        super().__init__(name="camera", daemon=True)
        self.source = source
        self.fy = fy_px
        self._last_rng = None
        self._last_rng_t = None
        self._cand = None   # a jump awaiting confirmation by the next frame
        self.rejected = 0
        self._rec_path, self._rec_step = record_path, max(1, int(record_step))
        self._rec_q = None
        self._rec_thread = None
        self.det = None
        self.dets = []              # up to 3 blobs, biggest first, for the estimator to choose from
        self.frames = 0
        self.detections = 0
        self.frame_wh = None        # actual capture size, for bearing maths
        self.fps = 0.0
        self.error = None
        self._stop = threading.Event()
        # NOT is_alive(): self._stop shadows Thread._stop(), which is_alive()
        # calls internally, so is_alive() raises TypeError on this class.
        # Signal completion explicitly instead.
        self._done = threading.Event()
        self._lock = threading.Lock()

    def latest(self):
        with self._lock:
            return self.det

    def latest_all(self):
        with self._lock:
            return list(self.dets)

    def stop(self, join_s: float = 3.0):
        """Stop, and WAIT for the recorder to drain. Without the join the writer
        is a daemon thread killed mid-frame when the interpreter exits, which
        truncates the .avi and aborts the process on the way out
        ("terminate called without an active exception", d44 2026-09-21)."""
        self._stop.set()
        t0 = time.monotonic()
        self._done.wait(timeout=join_s)
        if self._rec_thread is not None:
            self._rec_thread.join(timeout=max(0.0, join_s - (time.monotonic() - t0)))

    def _writer(self, path, wh):
        import cv2
        # The container's frame rate must match the rate we actually WRITE, not
        # the rate the camera runs at. --record-step 12 on a 60 fps pipeline is
        # a 5 fps recording; stamping it 30 plays it back six times too fast and
        # every timing you read off it is wrong. The .idx file still carries the
        # exact camera-frame mapping either way.
        fps = max(1.0, SOURCE_FPS / float(self._rec_step))
        vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), fps, wh)
        idx = open(str(path) + ".idx", "w", encoding="utf-8")
        idx.write("out_frame,cam_frame\n")
        k = 0
        while True:
            item = self._rec_q.get()
            if item is None:
                break
            n_, frame = item
            vw.write(frame)
            idx.write(f"{k},{n_}\n")
            k += 1
        vw.release()
        idx.close()

    def run(self):
        try:
            self._run()
        finally:
            self._done.set()

    def _run(self):
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
        if self._rec_path is not None:
            import queue
            ok0, probe = cap.read()
            if ok0 and probe is not None:
                self._rec_q = queue.Queue(maxsize=120)     # ~4 s of slack
                self._rec_thread = threading.Thread(
                    target=self._writer, daemon=True,
                    args=(self._rec_path, (probe.shape[1], probe.shape[0])))
                self._rec_thread.start()
        t_win, n_win = time.monotonic(), 0
        while not self._stop.is_set():
            ok, bgr = cap.read()
            if not ok or bgr is None:
                time.sleep(0.005)
                continue
            t = time.monotonic()
            self.frames += 1
            n_win += 1
            if self.frame_wh is None:
                self.frame_wh = (bgr.shape[1], bgr.shape[0])
            if self._rec_q is not None and self.frames % self._rec_step == 0:
                # NEVER BLOCK THE DETECTOR ON DISK. A full queue drops the
                # frame; a stalled writer must not stop the aircraft seeing.
                try:
                    self._rec_q.put_nowait((self.frames, bgr.copy()))
                except Exception:
                    pass
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
                out.append(Detection(offset_x=float(g.offset_x), offset_y=float(g.offset_y),
                                     area_frac=area, t=t, range_m=rng,
                                     v_usable=bool(g.v_usable), clipped_v=bool(g.clipped_v),
                                     ring_bbox=g.ring_bbox, stacked=bool(getattr(g, "stacked", False)),
                                     offset_y_top=getattr(g, "offset_y_top", None),
                                     offset_y_low=getattr(g, "offset_y_low", None)))
            # plausibility on the BIGGEST blob, which is the one everything
            # downstream uses; drop the whole frame if it fails
            if out and out[0].range_m is not None:
                r0 = out[0].range_m
                stale = self._last_rng_t is None or (t - self._last_rng_t) > REJECT_RESET_S
                ok = r0 <= RANGE_SANE_MAX_M
                if ok and not stale and self._last_rng is not None:
                    jump = abs(r0 - self._last_rng) / max(1e-3, t - self._last_rng_t)
                    if jump > RANGE_JUMP_MAX_MPS:
                        # A JUMP IS NOT AUTOMATICALLY GARBAGE. Passing gate 3 at
                        # 2 m and picking up gate 4 at 9 m is a legitimate jump
                        # and happens on every leg. What separates a handoff
                        # from noise is that a handoff AGREES WITH ITSELF on the
                        # next frame and noise does not - flight 4 went 154.6,
                        # 6.0, 12.6, 16.1 on consecutive frames. So hold the
                        # jump as a candidate and take it only when the frame
                        # after it lands nearby. Costs one frame, ~17 ms.
                        ok = (self._cand is not None
                              and abs(r0 - self._cand[0]) / max(1e-3, t - self._cand[1])
                              <= RANGE_JUMP_MAX_MPS)
                        self._cand = (r0, t)
                if ok:
                    self._last_rng, self._last_rng_t = r0, t
                    self._cand = None
                else:
                    self.rejected += 1
                    out = []
            if out:
                self.detections += 1
            with self._lock:
                self.dets = out
                self.det = out[0] if out else None
            if t - t_win >= 1.0:
                self.fps = n_win / (t - t_win)
                t_win, n_win = t, 0
        if self._rec_q is not None:
            try:
                self._rec_q.put_nowait(None)
            except Exception:
                pass
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
    ap.add_argument("--vert", choices=("baro", "vision"), default="baro",
                    help="where the vertical channel gets its reference. 'vision' "
                         "nulls the GATE'S ELEVATION instead of chasing a barometric "
                         "height - on the flat course every gate centre is 1.35 m, so "
                         "nulling it IS holding 1.35 m, and the barometer leaves the "
                         "vertical channel entirely.")
    ap.add_argument("--no-camera", action="store_true", help="run without a camera (dead reckoning only)")
    ap.add_argument("--fy", type=float, default=1000.0, help="camera focal length in pixels at the capture resolution")
    ap.add_argument("--cam-tilt", type=float, default=10.0, help="camera mount tilt above body forward, deg")
    ap.add_argument("--record", action="store_true",
                    help="save the camera to MJPG beside the CSV, plus a .idx mapping "
                         "video frame -> camera frame. Written on its own thread behind a "
                         "bounded queue: if the disk stalls, frames are DROPPED, never "
                         "blocked - the detector must not wait on storage")
    ap.add_argument("--record-step", type=int, default=1,
                    help="record every Nth frame (2 halves the size and the CPU)")
    ap.add_argument("--cam-hfov", type=float, default=90.0, help="camera horizontal field of view, deg")
    ap.add_argument("--map-north", required=True,
                    help="FC heading (deg) of the map's +y axis, or `here`: the drone is on the start line "
                         "pointing along gate 1 and the heading is read at startup")
    ap.add_argument("--laps", type=int, default=2)
    ap.add_argument("--rc-hz", type=float, default=50.0)
    ap.add_argument("--acc-lsb-per-g", default="auto",
                    help="raw accelerometer counts per g (2048 on the Archer per its blackbox, 256 on the SITL); "
                         "'auto' measures |acc| over 1 s at rest before takeoff")
    ap.add_argument("--pitch-nose-up-positive", action="store_true",
                    help="pitch reads positive NOSE DOWN on this firmware "
                         "(d45, 2026-09-20); pass this only if tiltcheck disagrees")
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
    # ONE SWITCH. The follower reads AIGP_VERT at IMPORT time and the runtime
    # reads --vert at RUN time, and nothing connected them: passing --vert
    # vision without also exporting AIGP_VERT=vision left the follower on the
    # barometer while the runtime dutifully computed elevations nobody used,
    # and exporting AIGP_VERT without --vert left the follower waiting for an
    # elevation that was never sent. Both halves fail SILENTLY and fly a
    # different mode than the one you asked for. Set it here, before the pilot
    # is imported, so the two cannot disagree.
    os.environ["AIGP_VERT"] = args.vert
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

    if args.vert == "vision":
        print("VERTICAL: the GATE'S ELEVATION. The barometer is out of the loop entirely.")
    else:
        print("VERTICAL: the BAROMETER. Every flight that failed on this team asked for a "
              "barometric height - pass --vert vision for the gate-relative mode.")
    if args.pilot == "follower":
        import solvers.follower as fol
        dr = fol._SOURCE                                    # the DeadReckonSource the follower built
        landmarks = fol._GATE_LANDMARKS                     # every gate: the estimator decides which one it sees
        if args.vert == "vision":
            dr.crossing_late_m = 0.75     # count a crossing 0.75 m PAST the plane, never early (see the estimator)
            print("CROSSINGS: counted 0.75 m past the gate plane - late, never early.")
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
                        pitch_nose_up_positive=args.pitch_nose_up_positive,
                        acc_lsb_per_g=512.0 if args.acc_lsb_per_g == "auto" else float(args.acc_lsb_per_g))
    _stamp = time.strftime("%Y%m%d_%H%M%S")
    _flightdir = AIGP_REPO / "out" / "flightlogs"
    _flightdir.mkdir(parents=True, exist_ok=True)
    _rec = (_flightdir / f"hw_{args.pilot}_{_stamp}.avi") if args.record else None
    camera = None if args.no_camera else CameraThread(args.camera, args.fy,
                                                      record_path=_rec,
                                                      record_step=args.record_step)
    if _rec is not None:
        print(f"recording -> {_rec}  (frames may be dropped before the aircraft is)")
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
    # The numeric branch belongs to --map-north, NOT to the drift setting. It
    # was nested under `else: heading_drift_dpm` and so ran whenever the drift
    # happened to be zero - float("here") then killed the run at startup
    # (d44, 2026-09-21, the first dry-run of the fixed stack). It never fired
    # in flight only because every flight so far passed --heading-drift-dpm.
    if args.map_north.strip().lower() == "here":
        src.map_north_heading_deg = float(bridge.state().attitude.yaw_deg)
        print(f"map north = FC heading now: {src.map_north_heading_deg:.1f} deg (the drone is on the start line pointing along gate 1)")
    else:
        src.map_north_heading_deg = float(args.map_north)
        if not has_mag:
            print("WARNING no magnetometer: the FC heading is gyro-integrated and restarts at an arbitrary value "
                  "every boot, so a numeric --map-north is only valid in the boot it was read in. Use --map-north here.")
    src.heading_drift_dpm = float(args.heading_drift_dpm)
    if src.heading_drift_dpm:
        print(f"heading drift correction: {src.heading_drift_dpm:+.1f} deg/min taken out over the run")
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

    from perception.gate_detection import commit_reason as _commit_reason
    log_path = _flightdir / f"hw_{args.pilot}_{_stamp}.csv"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = open(log_path, "w", newline="", encoding="utf-8")
    w = csv.writer(log)
    # cross_* / fix_*_sum are the debrief columns: what the drone can measure
    # about its own error with no ground truth (raceline.debrief reads them)
    w.writerow(["t", "phase_or_event", "x", "y", "z", "vx", "vy", "vz", "yaw_deg", "det_x", "det_y", "det_area", "det_range",
                "fixes", "fix_res", "rej", "unm", "cross_ev", "cross_lat", "cross_dz", "fix_cx_sum", "fix_al_sum",
                "throttle", "roll", "pitch", "yaw", "arm", "att_hz", "rtt_ms", "timeouts", "cam_fps", "vbat",
                # what the vertical channel was doing, and why (added 2026-09-21
                # after an evening spent inferring it from z_target - z)
                "el_deg", "vert_live", "commit_why", "alt_gated", "alt_rejected", "cam_rej"])
    print(f"log -> {log_path}")
    # WHAT IT WAS THINKING, in sentences, beside the numbers. Event driven, so
    # a clean approach is a few lines. See hardware/narrate.py.
    from hardware.narrate import Narrator
    narr = Narrator(path=log_path.with_suffix(".log"))
    print(f"narration -> {log_path.with_suffix('.log')}")
    if args.dry_run:
        print("DRY RUN: commands computed and logged; the FC receives neutral sticks")
    else:
        print("LIVE: waiting for the pilot. Throttle low, ARM, then MSP OVERRIDE on (and ANGLE). "
              "The plan clock starts when the FC reports both.")
        _pad_report_t = 0.0
        while True:
            st = bridge.state().status
            bridge.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500, arm=1000, aux2=1500)
            if st is not None and st.armed and st.msp_override:
                print(f"armed, MSP OVERRIDE on, modes {', '.join(st.active_modes)}: flying")
                if dr is not None:
                    print(f"accel bias learned on the pad: {np.round(dr.bias_body, 3)} m/s^2 "
                          f"body ({dr._bias_n} samples)")
                break
            # LEARN THE ACCELEROMETER BIAS WHILE WE WAIT (d43 race_007). The
            # aircraft is demonstrably at rest here, for as long as the pilot
            # takes - hundreds of samples. After arming there are only ~25
            # ticks of spool-up before the airborne latch, which is not a
            # rest measurement. The estimator subtracts what it learns here
            # in flight, see DeadReckonSource.bias_body.
            if dr is not None:
                est0 = src.estimate()
                if est0 is not None and src.accel_body is not None:
                    dr.integrate(time.monotonic(), est0.R, src.accel_body,
                                 float(est0.p[2]), True, on_ground=True)
                # THE GATE CHECK ON THE PAD (race day 2: no dry run on the start
                # line outside the slot). Every 2 s while waiting: what the
                # camera sees and where, so the pad wait doubles as the check
                # that the detector has g0 and the elevation reads "above".
                _now = time.monotonic()
                if camera is not None and _now - _pad_report_t >= 2.0:
                    _pad_report_t = _now
                    _d = camera.latest()
                    if _d is not None and (_now - _d.t) <= 0.5 and est0 is not None:
                        from hardware.hover import gate_dz as _gate_dz
                        _, _el = _gate_dz(_d, est0.R, args.fy, math.radians(args.cam_tilt), camera.frame_wh)
                        print(f"PAD: gate at {_d.range_m:.1f} m, {math.degrees(_el):+.1f} deg "
                              f"{'ABOVE' if _el > 0 else 'below'}, offset x {_d.offset_x:+.2f}"
                              f"{', STACKED' if getattr(_d, 'stacked', False) else ''}; "
                              f"bias=({dr.bias_body[0]:+.2f},{dr.bias_body[1]:+.2f}) n={dr._bias_n}")
                    else:
                        print(f"PAD: no gate in view; bias n={dr._bias_n}")
            time.sleep(0.05)
        if not bridge.state().status.angle_mode and not args.acro:
            print("WARNING ANGLE mode is not active on the FC: the sticks are angle sticks. Flip the ANGLE switch.")

    period = 1.0 / args.rc_hz
    t_start = time.monotonic()
    n = 0
    fixes = 0
    fix_res = 0.0
    t_fix = -1.0
    t_match = None      # last time a detection was ACCEPTED against the map
    commit_ev, commit_run, commit_latched = None, 0, False
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
            # Defined here, not inside the follower branch: the log row is
            # written past the end of that branch, and a name that only exists
            # on one path is the same trap that killed the first dry run.
            el_raw, vert_live, _why = None, 0, ""
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
                # NO POSITION FIXES ON A COMMITTED GATE (d43 race_006/007,
                # 2026-09-22, both into g0's right edge). Once the ring fills
                # the frame its "centre" is whichever bar is in view: at 1.9 m
                # det_x swung -0.07 -> +0.43 -> -0.47 in one second and the
                # estimate followed at full gain, +0.5 -> -2.2 -> +1.9 m, with
                # the roll stick at 1788 then 1146 behind it. Flight 3 the
                # same after a one-frame dropout: a blob at det_x 0.78 with a
                # 6.6 m range, y pulled 4.0 -> 1.5 in 0.8 s, pitch 1688. The
                # vertical channel already stops steering on this gate at
                # commit (below); the horizontal must too. Dead-reckon the
                # last ~3.7 m - 2.5 s at plan speed - and resume on the next
                # gate, which clears the latch.
                if dets and t - t_fix >= 1.0 / 30.0 and not commit_latched:
                    t_fix = t
                    idx, r = dr.observe_any(dets, landmarks)
                    if idx is not None:
                        fixes += 1; fix_res = r
                        t_match = t
                        est_dr.p[:] = dr.p; est_dr.v[:] = dr.v
                # Hand the follower the gate's elevation, which is what its
                # vertical channel uses instead of a height when AIGP_VERT is
                # vision. Elevation needs no range, and range is the camera's
                # worst signal.
                # el_raw: what the camera measured, whether or not we let it
                # steer. el: what the follower was actually given. Logging both
                # is what turns "she climbed again" into an answer.
                el = None
                if args.vert == "vision":
                    # ONLY STEER HEIGHT ON A GATE THE MAP AGREES IS A GATE.
                    # d44, 2026-09-21, dry run with NO GATE IN FRONT OF IT: the
                    # detector reported a gate at 12.5 m, 38.9 deg up, and the
                    # vertical reference saturated at its +0.35 m cap while the
                    # aircraft sat on the ground. observe_any rejected all 125
                    # of those detections against the map (unm=125, fixes=0) -
                    # the POSITION channel was protected and the VERTICAL one
                    # was not, because it consumed the raw detection directly.
                    #
                    # A blob that cannot be matched to any gate we expect to
                    # see is not a gate, whatever its colour. Require a recent
                    # accepted fix before the elevation is allowed to move the
                    # aircraft. Losing the match fades the reference out the
                    # same way losing the gate does, which is the safe default.
                    matched = (t_match is not None) and (t - t_match) <= MATCH_STALE_S
                    fresh = det is not None and t - det.t <= 0.5 and camera is not None
                    if fresh:
                        from hardware.hover import gate_dz as _gate_dz
                        # THE STACKED GATE: aim the height at the ring we are
                        # flying through, not at the bar between them. The next
                        # event's z says which ring: above 3 m is the top.
                        if getattr(det, "stacked", False) and 0 <= dr.next_event < len(dr.events):
                            _want_top = float(dr.events[dr.next_event][2]) > 3.0
                            _oy = det.offset_y_top if _want_top else det.offset_y_low
                            if _oy is not None:
                                import copy as _copy
                                det = _copy.copy(det); det.offset_y = float(_oy)
                        _, el_raw = _gate_dz(det, est.R, args.fy,
                                             math.radians(args.cam_tilt), camera.frame_wh)
                    if fresh and matched and getattr(det, "v_usable", True):
                        el = el_raw
                        vert_live = 1
                    # A gate we can see but whose height we cannot trust is a
                    # COMMIT, not a dropout. Say which: they fade differently.
                    # COMMIT LATCHES, PER GATE. Inside the commit range the
                    # geometry only gets WORSE, so there is no case where
                    # un-committing on the same gate is right. Without a latch
                    # one flickering frame hands the height back to a
                    # close-range clipped ring, which is flight 4 with extra
                    # steps - and hsv_classic already documents the ring
                    # FRAGMENTING (the AI-GP sign breaks the orange loop), so
                    # the biggest blob dropping under the size threshold at 2 m
                    # is a real event, not a hypothetical. The latch clears
                    # when the next gate becomes active.
                    if dr.next_event != commit_ev:
                        commit_ev, commit_run, commit_latched = dr.next_event, 0, False
                    # ONLY COMMIT TO A GATE THE MAP SAYS IS NEAR (d43
                    # race_005, 2026-09-22): 0.1 s after crossing g0 the
                    # detector saw g0's own ring around the aircraft, called
                    # it "gate 1, wider than the frame", and g1 was LATCHED
                    # committed from 9.8 m out - no height reference and,
                    # now that a commit also stops position fixes, no fixes
                    # either, for the whole g1 approach. The commit ranges by
                    # size are 3.5-4.6 m; a gate the estimate puts beyond
                    # COMMIT_MAX_DIST_M cannot be filling the frame.
                    near = True
                    if 0 <= dr.next_event < len(dr.events):
                        gx, gy = dr.events[dr.next_event][0], dr.events[dr.next_event][1]
                        near = math.hypot(gx - dr.p[0], gy - dr.p[1]) <= COMMIT_MAX_DIST_M
                    aligned = True
                    if 0 <= dr.next_event < len(dr.events):
                        aligned = fol.commit_aligned(dr.p, est.yaw, dr.events[dr.next_event])
                    level_ok = fol.commit_level_ok(el_raw, det.range_m if det is not None else None)
                    if det is not None and not getattr(det, "v_usable", True) and near and aligned and level_ok:
                        commit_run += 1
                        if commit_run >= COMMIT_CONFIRM:
                            commit_latched = True
                    else:
                        commit_run = 0
                    if commit_latched:      # latched: the elevation may not steer again
                        el, vert_live = None, 0
                    fol.set_gate_elevation(el, t - t_start, committed=commit_latched,
                                           range_m=(det.range_m if (det is not None and fresh) else None))
                rc = fol.step(t - t_start, est_dr, dr.next_event, True, "")
                out = dict(throttle=rc.throttle, roll=rc.roll, pitch=rc.pitch, yaw=rc.yaw, arm=rc.arm, aux2=rc.aux2)
                label = f"ev{dr.next_event}" + (f"/lm{dr.last_landmark}" if dr.last_landmark is not None else "")
                _cw, _ch = (camera.frame_wh or (0, 0)) if camera is not None else (0, 0)
                _com, _why = ((False, "") if det is None or not _ch
                              else _commit_reason(det, (_ch, _cw)))
                if camera is not None:
                    # ELEVATION, not metres: metres would need the range, and
                    # the vertical channel deliberately never touches it.
                    _eldeg = math.degrees(el) if el is not None else None
                    narr.tick(t - t_start, event=dr.next_event, det=det,
                              rng=det.range_m if det else None,
                              dz=fol.vision_dz() if args.vert == "vision" else None,
                              el_deg=_eldeg,
                              z=float(est_dr.p[2]), throttle=rc.throttle,
                              airborne=airborne_latch, committed=_com, why=_why,
                              cross_ev=dr.cross_ev, cross_lat=dr.cross_lat,
                              cross_dz=dr.cross_dz)
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
                            f"{camera.fps:.0f}" if camera else "", f"{s.battery.voltage_v:.2f}" if s.battery and s.battery.voltage_v else "",
                            f"{math.degrees(el_raw):.2f}" if el_raw is not None else "",
                            vert_live, _why,
                            getattr(src, "alt_gated", ""), getattr(src, "alt_rejected", ""),
                            camera.rejected if camera else ""])
            if n % int(args.rc_hz) == 0:
                log.flush()
                print(f"t={t - t_start:6.1f} {label:7s} p=({est_log.p[0]:+5.1f},{est_log.p[1]:+5.1f},{est_log.p[2]:4.2f}) "
                      f"yaw={math.degrees(est_log.yaw):5.0f} det={'%.2fm' % det.range_m if det and det.range_m else '-':>6} "
                      f"fixes={fixes} res={fix_res:5.2f}"
                      + (f" rej={dr.rejected} unm={dr.unmatched} " if dr is not None else " ") +
                      f"stk=({out['roll']},{out['pitch']},{out['throttle']},{out['yaw']}) arm={out['arm']} "
                      f"link {s.attitude_hz:.0f}Hz/{s.link.last_rtt_ms:.0f}ms healthy={s.healthy}"
                      + (f" bias=({dr.bias_body[0]:+.2f},{dr.bias_body[1]:+.2f}) n={dr._bias_n}" if dr is not None else ""))
            if done:
                narr.close()
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
        # ORDER MATTERS. The FC gets disarmed FIRST, then the log is closed,
        # and the camera is torn down LAST. Argus teardown is not reliable on
        # this platform - d44, 2026-09-21: "Mutex not initialized", 7 client
        # objects still live, segfault - and with the camera first a crash in
        # NVIDIA's library meant bridge.stop() never ran and the disarm frames
        # were never sent. Nothing that can crash may sit between the end of
        # the flight and safing the aircraft.
        bridge.stop()
        log.close()
        if camera:
            try:
                camera.stop()
            except Exception as e:      # a teardown crash must not mask the run
                print(f"camera teardown complained ({e}); the flight data is already safe")
    return 0


if __name__ == "__main__":
    sys.exit(main())
