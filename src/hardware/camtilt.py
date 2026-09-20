"""Camera mount tilt from a target at a known height, using the calibrated
intrinsics. More accurate than marking the frame edges.

    python3 -m hardware.camtilt --dist 0.905 --lens-h 0.830 --target-h 1.25 \
        --fy 830 --cy 387

Why not mark the frame edges: the edges land on a vertical wall through
tan(theta +- vhalf), which is not symmetric about the optical axis, and any
crop in the viewer (a page header, a scaled image) biases one edge more than
the other. On d45 the edge marks implied a 37.6 deg vertical field of view
where the checkerboard measured 46.9 - about 80 % of the frame, and the tilt
inherited whatever that asymmetry was.

This instead finds ONE target whose real height you measured, reads the pixel
row it lands on, and uses the calibrated focal length and principal point:

    elevation in the image  = atan( (cy - y_px) / fy )
    elevation in the world  = atan( (target_h - lens_h) / dist )
    mount tilt              = world - image

Target can be the checkerboard (found automatically) or any point you can
click. All lengths in metres, all heights from the same floor.

Keep the drone LEVEL: this measures the camera against the airframe, so any
body pitch goes straight into the answer. Check roll and pitch first with
`msp_bench.py telemetry`.
"""

from __future__ import annotations

import argparse
import math

INNER = (6, 8)      # inner corners of the printed checkerboard


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--camera", default=None)
    ap.add_argument("--dist", type=float, required=True,
                    help="lens to wall, perpendicular, metres")
    ap.add_argument("--lens-h", type=float, required=True,
                    help="lens height off the floor, metres")
    ap.add_argument("--target-h", type=float, default=None,
                    help="target centre height off the floor, m. Omit when using "
                         "--checkerboard and pass --board-h instead")
    ap.add_argument("--fy", type=float, required=True, help="from camcal_board")
    ap.add_argument("--cy", type=float, default=None,
                    help="principal point row from camcal_board (default: image centre)")
    ap.add_argument("--pitch", type=float, default=0.0,
                    help="body pitch in deg, NOSE-UP positive, if the drone is not level")
    ap.add_argument("--frames", type=int, default=60)
    args = ap.parse_args(argv)

    import cv2
    import numpy as np
    from hardware.runtime import DEFAULT_PIPELINE

    src = args.camera or DEFAULT_PIPELINE
    cap = (cv2.VideoCapture(src, cv2.CAP_GSTREAMER) if not str(src).startswith("/dev/")
           else cv2.VideoCapture(src))
    if not cap.isOpened():
        raise SystemExit("camera did not open")

    ys, shape = [], None
    for _ in range(args.frames):
        ok, frame = cap.read()
        if not ok:
            continue
        shape = frame.shape
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            g, INNER, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK)
        if found:
            ys.append(float(corners.reshape(-1, 2)[:, 1].mean()))
    cap.release()

    if not ys:
        raise SystemExit("checkerboard not found in any frame - is it in view and lit?")
    h, w = shape[:2]
    y_px = float(np.median(ys))
    cy = args.cy if args.cy is not None else h / 2.0

    img_el = math.degrees(math.atan((cy - y_px) / args.fy))
    world_el = math.degrees(math.atan((args.target_h - args.lens_h) / args.dist))
    tilt = world_el - img_el - args.pitch

    print(f"frames {args.frames}, board seen in {len(ys)}, capture {w}x{h}")
    print(f"board centre row      {y_px:7.1f} px   (principal point cy {cy:.1f})")
    print(f"elevation in image    {img_el:+7.2f} deg  (above the optical axis)")
    print(f"elevation in world    {world_el:+7.2f} deg  (above the lens)")
    if args.pitch:
        print(f"body pitch subtracted {args.pitch:+7.2f} deg")
    print(f"\n  MOUNT TILT = {tilt:.1f} deg      ->  --cam-tilt {tilt:.0f}")
    print("\nRepeat at a second distance. The two must agree within ~2 deg;")
    print("if they do not, the drone moved or it is not level.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
