"""
The on-drone runtime: camera -> detector -> seeker -> flight controller.

    cd src
    python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north 30 --dry-run
    python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north 30 --arm

--dry-run   everything runs (camera, detector, brain, loops, the MSP link
            streams DISARMED sticks) and the commands are printed. Nothing
            can spin.
--arm       the real thing. Props on, cage or track, safety pilot ready.

Inputs the runtime needs on the day:
  --map-north   compass heading (deg) of the map's +y axis: point the drone
                along gate 1's flight direction on the start line and read
                the heading from `hardware.bench telemetry`.
  --laps        2 for a scored run.
  --camera      a GStreamer pipeline or a /dev/video device (see below).

Camera: the IMX477 needs the Argus ISP for a usable colour image, which
needs a display context (see the Orin quickstart). The default pipeline
below uses nvarguscamerasrc; run from a session that has DISPLAY and
XAUTHORITY set, or pass --camera /dev/video0 for the raw path.

The control loop runs at the bridge's RC rate (50 Hz): each tick reads
the latest attitude/altitude from the bridge, the latest detection from
the camera thread, steps the pilot and hands the sticks to the bridge.
Everything is logged to out/flightlogs/hw_seeker_<timestamp>.csv.
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

from raceline.config import load_config, AIGP_REPO
from raceline import course as course_bridge
from seeker.brain import Detection, SeekerConfig
from seeker.pilot import SeekerPilot, crossings_from_course
from hardware.bridge import FcBridge
from hardware.state import FcStateSource

DEFAULT_PIPELINE = ("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1280,height=720,"
                    "framerate=60/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
                    "video/x-raw,format=BGR ! appsink drop=true max-buffers=1")


class CameraThread(threading.Thread):
    """Grabs frames and runs the HSV detector; keeps only the latest."""

    def __init__(self, source: str, width_hint: int = 1280):
        super().__init__(name="camera", daemon=True)
        self.source = source
        self.det = None
        self.frames = 0
        self.detections = 0
        self.fps = 0.0
        self.error = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

    def latest(self):
        with self._lock:
            return self.det

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            import cv2
            from perception.detectors.hsv_classic import gate_mask
            from perception.gate_detection import mask_to_detections
        except ImportError as e:
            self.error = f"cv2/detector import failed: {e}"
            return
        if self.source.startswith("/dev/video"):
            cap = cv2.VideoCapture(self.source, cv2.CAP_V4L2)
        else:
            cap = cv2.VideoCapture(self.source, cv2.CAP_GSTREAMER)
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
            dets = mask_to_detections(gate_mask(bgr), bgr.shape)
            d = None
            if dets:
                g = dets[0]
                area = float(g.area_frac or g.area / float(bgr.shape[0] * bgr.shape[1]))
                d = Detection(offset_x=float(g.offset_x), offset_y=float(g.offset_y), area_frac=area, t=t)
                self.detections += 1
            with self._lock:
                self.det = d
            if t - t_win >= 1.0:
                self.fps = n_win / (t - t_win)
                t_win, n_win = t, 0
        cap.release()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--port", default=None)
    ap.add_argument("--tcp", default=None, help="Betaflight SITL host[:port] instead of a serial port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--camera", default=DEFAULT_PIPELINE, help="GStreamer pipeline or /dev/videoN")
    ap.add_argument("--no-camera", action="store_true", help="run without a camera (bench: hover/land logic only)")
    ap.add_argument("--map-north", type=float, required=True, help="compass heading of the map's +y axis (deg)")
    ap.add_argument("--laps", type=int, default=2)
    ap.add_argument("--config", default=None, help="vehicle.toml (default config/vehicle.toml)")
    ap.add_argument("--rc-hz", type=float, default=50.0)
    ap.add_argument("--pitch-nose-down-positive", action="store_true",
                    help="set if the bench telemetry shows pitch going POSITIVE when the nose is pushed down")
    ap.add_argument("--max-s", type=float, default=240.0, help="hard stop: land and disarm after this many seconds")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--dry-run", action="store_true")
    g.add_argument("--arm", action="store_true")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    course = course_bridge.load_course(laps=args.laps)
    crossings = crossings_from_course(course)
    print(f"course: {len(crossings)} crossings/lap, {args.laps} laps, start line at the sim origin")

    bridge = FcBridge.open(port=args.port, tcp=args.tcp, baud=args.baud, rc_hz=args.rc_hz)
    bridge.start()
    src = FcStateSource(bridge, map_north_heading_deg=args.map_north,
                        pitch_nose_up_positive=not args.pitch_nose_down_positive)
    cam = None
    if not args.no_camera:
        cam = CameraThread(args.camera)
        cam.start()

    # wait for telemetry, zero the barometer on the ground
    t_wait = time.monotonic()
    while bridge.state().attitude is None:
        if time.monotonic() - t_wait > 5.0:
            print("ERROR no attitude from the FC after 5 s"); bridge.stop(); return 3
        time.sleep(0.05)
    while bridge.state().altitude is None and time.monotonic() - t_wait < 5.0:
        time.sleep(0.05)
    src.zero_altitude()
    s = bridge.state()
    print(f"FC link: attitude {s.attitude_hz:.0f} Hz, rc {s.rc_hz:.0f} Hz, rtt {s.link.last_rtt_ms:.1f} ms; "
          f"heading {s.attitude.yaw_deg:.0f} -> world yaw {math.degrees(src.estimate().yaw):.0f} deg; "
          f"baro zeroed at {src.alt_offset_m:.2f} m")
    if cam is not None:
        time.sleep(1.0)
        if cam.error:
            print(f"ERROR camera: {cam.error}"); bridge.stop(); return 4
        print(f"camera: {cam.fps:.0f} fps, detections so far {cam.detections}")
    if s.status is not None and s.status.arming_blockers and args.arm:
        print(f"WARNING arming blockers: {', '.join(s.status.arming_blockers)}")

    log_path = AIGP_REPO / "out" / "flightlogs" / f"hw_seeker_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = open(log_path, "w", newline="", encoding="utf-8")
    w = csv.writer(log)
    w.writerow(["t", "phase", "crossing", "yaw_deg", "z", "vz", "det_x", "det_y", "det_area",
                "z_target", "yaw_target_deg", "roll_deg", "pitch_deg", "throttle", "roll", "pitch", "yaw",
                "arm", "att_hz", "rtt_ms", "timeouts", "cam_fps", "vbat"])
    print(f"log -> {log_path}")
    print("DRY RUN: sticks are computed and printed; the FC receives DISARMED neutral sticks" if args.dry_run
          else "ARMED RUN: taking off in 0.5 s")

    pilot = SeekerPilot(cfg, crossings, start_xy=(0.0, 0.0), laps=args.laps, t0=time.monotonic())
    period = 1.0 / args.rc_hz
    t_start = time.monotonic()
    n = 0
    try:
        while True:
            t = time.monotonic()
            est = src.estimate()
            det = cam.latest() if cam is not None else None
            if det is not None and t - det.t > 0.25:
                det = None
            out = pilot.tick(t, est, det, baro_fresh=True)
            if t - t_start > args.max_s and not out.done:
                pilot.brain.z_hold = float(est.p[2]) if est is not None else 0.0
                pilot.brain._enter("LAND", t)
            if args.arm:
                bridge.set_rc(throttle=out.throttle, roll=out.roll, pitch=out.pitch, yaw=out.yaw,
                              arm=out.arm, aux2=out.aux2)
            else:
                bridge.set_rc(throttle=1000, roll=1500, pitch=1500, yaw=1500, arm=1000, aux2=out.aux2)
            s = bridge.state()
            n += 1
            if n % 5 == 0 and est is not None:
                w.writerow([f"{t - t_start:.3f}", out.phase, out.crossing, f"{math.degrees(est.yaw):.1f}",
                            f"{est.p[2]:.2f}", f"{est.v[2]:.2f}",
                            f"{det.offset_x:.3f}" if det else "", f"{det.offset_y:.3f}" if det else "",
                            f"{det.area_frac:.4f}" if det else "", f"{out.z_target:.2f}",
                            f"{math.degrees(out.yaw_target):.1f}", f"{out.tilt_deg[0]:.1f}", f"{out.tilt_deg[1]:.1f}",
                            out.throttle, out.roll, out.pitch, out.yaw, out.arm,
                            f"{s.attitude_hz:.0f}", f"{s.link.last_rtt_ms:.1f}", s.link.timeouts,
                            f"{cam.fps:.0f}" if cam else "", f"{s.battery.voltage_v:.2f}" if s.battery and s.battery.voltage_v else ""])
            if n % int(args.rc_hz) == 0:
                log.flush()
                print(f"t={t - t_start:6.1f} {out.phase:7s} k={out.crossing:2d} z={est.p[2] if est else 0:4.2f} "
                      f"yaw={math.degrees(est.yaw) if est else 0:5.0f} det={'%.3f' % det.area_frac if det else '-':>6} "
                      f"stk=({out.roll},{out.pitch},{out.throttle},{out.yaw}) arm={out.arm} "
                      f"link {s.attitude_hz:.0f}Hz/{s.link.last_rtt_ms:.0f}ms healthy={s.healthy}")
            if out.done:
                print("run complete: landed and disarmed")
                break
            if not s.healthy and args.arm:
                print("FC link unhealthy: the bridge is holding the disarm channels; stopping")
                break
            time.sleep(max(0.0, period - (time.monotonic() - t)))
    except KeyboardInterrupt:
        print("interrupted: disarming")
    finally:
        log.close()
        if cam is not None:
            cam.stop()
        bridge.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
