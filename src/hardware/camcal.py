"""
Camera numbers for the runtime, measured on a real gate with a tape. Run
from src/ on the Orin with the drone level on the ground (or held level)
and one gate in view.

    python3 -m hardware.camcal --dist 6.0                       # gate ring centre 6.0 m from the lens
    python3 -m hardware.camcal --dist 6.0 --dz 1.1 --port /dev/ttyTHS1   # + ring centre 1.1 m above the lens

Prints, from the median of --seconds of detections:
    fy        focal length in pixels at the capture resolution  -> runtime --fy
    hfov      horizontal field of view implied by fy            -> runtime --cam-hfov
    tilt      camera mount tilt above body forward (needs --dz; the FC pitch
              is subtracted when --port/--tcp is given)        -> runtime --cam-tilt
    range     what the runtime would report at this distance (should equal --dist)

The ring's outer size is GATE_OUTER_M; measure the real ring and pass
--ring-m if it differs. The size used is the frame's outer WIDTH (the real
gate is a square frame with a header board on top: the height is not it).
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time


def main(argv=None) -> int:
    from hardware.runtime import DEFAULT_PIPELINE, GATE_OUTER_M
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--camera", default=DEFAULT_PIPELINE)
    ap.add_argument("--dist", type=float, required=True, help="lens to ring centre, m, along the floor")
    ap.add_argument("--dz", type=float, default=None, help="ring centre height minus lens height, m")
    ap.add_argument("--ring-m", type=float, default=GATE_OUTER_M, help="outer frame WIDTH, m (measure the real gate)")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--port", default=None)
    ap.add_argument("--tcp", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args(argv)

    import cv2
    from perception.detectors.hsv_classic import gate_mask
    from perception.gate_detection import mask_to_detections

    pitch_deg = 0.0
    if args.port or args.tcp:
        from hardware.bridge import FcBridge
        br = FcBridge.open(port=args.port, tcp=args.tcp, baud=args.baud)
        br.start()
        t0 = time.monotonic()
        while br.state().attitude is None and time.monotonic() - t0 < 5.0:
            time.sleep(0.05)
        a = br.state().attitude
        br.stop()
        if a is None:
            print("no attitude from the FC; tilt will assume the body is level")
        else:
            pitch_deg = a.pitch_deg
            print(f"FC pitch {pitch_deg:+.1f} deg (Betaflight sign: check the bench telemetry with the nose pushed down)")

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2 if args.camera.startswith("/dev/video") else cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print(f"camera did not open: {args.camera}"); return 2
    heights, cys, cxs, frames, shape = [], [], [], 0, None
    t_end = time.monotonic() + args.seconds
    while time.monotonic() < t_end:
        ok, bgr = cap.read()
        if not ok or bgr is None:
            continue
        frames += 1
        shape = bgr.shape
        dets = mask_to_detections(gate_mask(bgr), bgr.shape)
        if not dets:
            continue
        g = dets[0]
        box = g.ring_bbox or g.bbox
        if box is None or box[3] <= 4:
            continue
        heights.append(float(box[2]))          # WIDTH: the gate has a header board, the width is the clean size
        cys.append(box[1] + box[3] / 2.0)
        cxs.append(box[0] + box[2] / 2.0)
    cap.release()
    if not heights:
        print(f"no ring detected in {frames} frames: fix the HSV thresholds (perception/detectors/hsv_classic.py) first")
        return 1
    h, w = shape[:2]
    h_px = statistics.median(heights)
    cy = statistics.median(cys)
    cx = statistics.median(cxs)
    fy = h_px * args.dist / args.ring_m
    hfov = math.degrees(2.0 * math.atan((w / 2.0) / fy))
    print(f"frames {frames}, detections {len(heights)}, capture {w}x{h}")
    print(f"frame width {h_px:.1f} px (spread {min(heights):.0f}..{max(heights):.0f}), centre ({cx:.0f}, {cy:.0f}) px")
    print(f"fy    = {fy:.1f} px            -> --fy {fy:.0f}")
    print(f"hfov  = {hfov:.1f} deg           -> --cam-hfov {hfov:.0f}")
    print(f"range = {fy * args.ring_m / h_px:.2f} m at --fy {fy:.0f} (must equal --dist {args.dist})")
    if args.dz is not None:
        up_in_cam = math.degrees(math.atan((h / 2.0 - cy) / fy))         # ring above the optical axis
        geometric = math.degrees(math.atan2(args.dz, args.dist))          # ring above the lens, world
        tilt = geometric - up_in_cam - pitch_deg                          # axis above body forward
        print(f"tilt  = {tilt:.1f} deg (ring {geometric:.1f} deg above the lens, {up_in_cam:+.1f} deg above the optical "
              f"axis, body pitch {pitch_deg:+.1f}) -> --cam-tilt {tilt:.0f}")
    else:
        print("tilt: pass --dz (ring centre height minus lens height) to measure the mount tilt")
    return 0


if __name__ == "__main__":
    sys.exit(main())
