"""Camera intrinsics from a printed checkerboard.

Two steps, both on the drone.

    python3 -m hardware.camcal_board grab                 # capture views
    python3 -m hardware.camcal_board solve --square-mm 25.0

`grab` shows nothing and needs no display: it saves a frame every time it finds
the board, and prints how many it has. Move the board (or the drone) between
captures - different angles and distances, not 20 copies of the same view.

`solve` runs OpenCV's calibration over the saved frames and prints the numbers
the runtime wants, plus the principal point and distortion, which the wall
method cannot give you.

The board is `out/caltarget/checkerboard_letter_25mm.png`. Print at 100 %,
tape it FLAT to card, then MEASURE a square and pass the real number to
--square-mm. Printers scale, and every length here is proportional to it.
"""

from __future__ import annotations

import argparse
import glob
import math
import os
from pathlib import Path

import numpy as np

DEFAULT_DIR = "out/camcal_board"
INNER = (6, 8)        # inner corners of the 7x9-square board


def _open(camera):
    import cv2
    if str(camera).isdigit() or str(camera).startswith("/dev/video"):
        return cv2.VideoCapture(camera if not str(camera).isdigit() else int(camera))
    return cv2.VideoCapture(camera, cv2.CAP_GSTREAMER)


def grab(args) -> int:
    import cv2
    from hardware.runtime import DEFAULT_PIPELINE
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    cap = _open(args.camera or DEFAULT_PIPELINE)
    if not cap.isOpened():
        raise SystemExit("camera did not open")
    kept, seen, last, prev = 0, 0, None, None
    CR = chr(13)
    print(f"looking for a {INNER[0]}x{INNER[1]} inner-corner board. "
          f"Ctrl+C when you have {args.want}.")
    try:
        while kept < args.want:
            ok, frame = cap.read()
            if not ok:
                continue
            seen += 1
            g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                g, INNER, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK)
            if not found:
                continue
            c = corners.reshape(-1, 2).mean(axis=0)
            pts = corners.reshape(-1, 2)

            # STILLNESS. This sensor has a rolling shutter: it reads one row
            # at a time, so a board that is moving is not blurred, it is
            # SKEWED - and a skewed board fits no camera model at all. That is
            # what a 5.9 px RMS with absurd distortion terms looks like
            # (d45, 2026-09-20, where the drone was walked around a fixed
            # board). Require two consecutive frames to agree before believing
            # either of them.
            moved = None if prev is None else float(np.abs(pts - prev).max())
            prev = pts
            if moved is None or moved > args.still_px:
                if seen % 30 == 0:
                    note = "hold it still" if moved is None else f"moving {moved:.0f} px"
                    print(CR + f"  {kept}/{args.want} kept - {note}          ",
                          end="", flush=True)
                continue

            # SHARPNESS, measured on the board itself rather than the whole
            # frame, so a busy background cannot vouch for a soft target.
            x0, y0 = pts.min(axis=0).astype(int)
            x1, y1 = pts.max(axis=0).astype(int)
            patch = g[max(0, y0):y1 + 1, max(0, x0):x1 + 1]
            sharp = float(cv2.Laplacian(patch, cv2.CV_64F).var()) if patch.size else 0.0
            if sharp < args.min_sharp:
                if seen % 30 == 0:
                    print(CR + f"  {kept}/{args.want} kept - too soft ({sharp:.0f} < "
                               f"{args.min_sharp:.0f}): more light or hold stiller   ",
                          end="", flush=True)
                continue

            if last is not None and np.hypot(*(c - last)) < args.min_move_px:
                continue          # same view again: change the angle
            last = c
            kept += 1
            cv2.imwrite(str(out / f"view_{kept:03d}.png"), frame)
            print(CR + f"  kept {kept}/{args.want}  centre {c[0]:.0f},{c[1]:.0f}  "
                       f"sharpness {sharp:.0f}  - now CHANGE THE ANGLE, hold still  ")
    except KeyboardInterrupt:
        print()
    finally:
        cap.release()
    print(f"{kept} views in {out} from {seen} frames")
    if kept < 8:
        print("fewer than 8 views: calibration will be poor. Grab more.")
    return 0


def solve(args) -> int:
    import cv2
    files = sorted(glob.glob(str(Path(args.out) / "view_*.png")))
    if len(files) < 5:
        raise SystemExit(f"only {len(files)} views in {args.out}; grab more")
    sq = args.square_mm / 1000.0
    objp = np.zeros((INNER[0] * INNER[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:INNER[0], 0:INNER[1]].T.reshape(-1, 2) * sq
    objpoints, imgpoints, shape = [], [], None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    for f in files:
        img = cv2.imread(f)
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        shape = g.shape[::-1]
        found, corners = cv2.findChessboardCorners(g, INNER, None)
        if not found:
            print(f"  {Path(f).name}: board not found, skipped"); continue
        corners = cv2.cornerSubPix(g, corners, (11, 11), (-1, -1), crit)
        objpoints.append(objp); imgpoints.append(corners)
    print(f"{len(objpoints)} usable views at {shape[0]}x{shape[1]}")
    if len(objpoints) < 5:
        raise SystemExit("not enough usable views")
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, shape, None, None)

    # PER-VIEW ERROR, and how obliquely each board was seen. Both matter and
    # neither is visible in the single RMS number. One ruined capture can carry
    # the whole solve, and a set of views that are all face-on cannot pin the
    # focal length down at all - the fit slides along a valley where focal
    # length trades against distance, and fx and fy come out split by tens of
    # percent in whichever direction the optimiser happened to fall. d45 saw
    # 915/723 on one attempt and 642/1210 on the next.
    per_view = []
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        err = float(np.linalg.norm(imgpoints[i].reshape(-1, 2) - proj.reshape(-1, 2),
                                   axis=1).mean())
        R, _ = cv2.Rodrigues(rvecs[i])
        # angle between the board's normal and the camera's optical axis: 0 is
        # dead face-on, which measures nothing about focal length
        obliq = math.degrees(math.acos(min(1.0, abs(float(R[2, 2])))))
        per_view.append((err, obliq, Path(files[i]).name))

    worst = sorted(per_view, reverse=True)[:5]
    obliqs = sorted(v[1] for v in per_view)
    print("")
    print("per-view reprojection error (worst 5):")
    for err, obliq, name in worst:
        print(f"  {name:16s} {err:6.2f} px   board seen {obliq:4.0f} deg off face-on")
    print(f"obliqueness across all views: {obliqs[0]:.0f} to {obliqs[-1]:.0f} deg, "
          f"median {obliqs[len(obliqs)//2]:.0f}")

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    w, h = shape
    hfov = math.degrees(2 * math.atan((w / 2.0) / fx))
    vfov = math.degrees(2 * math.atan((h / 2.0) / fy))
    print(f"\nreprojection RMS: {rms:.3f} px   (under ~0.5 is good, over 1.0 means redo it)")
    print(f"fx {fx:8.1f}   fy {fy:8.1f}")
    print(f"cx {cx:8.1f}   cy {cy:8.1f}     image centre is {w/2:.1f}, {h/2:.1f}")
    print(f"principal point offset: {cx - w/2:+.1f}, {cy - h/2:+.1f} px "
          f"= {math.degrees(math.atan((cx - w/2)/fx)):+.2f} deg horizontal bias")
    print(f"hfov {hfov:.1f} deg   vfov {vfov:.1f} deg")
    print(f"distortion k1 k2 p1 p2 k3: " + " ".join(f"{v:+.4f}" for v in dist.ravel()[:5]))
    # VERDICT. A bad solve prints numbers that look exactly like a good one,
    # and the only defence is to refuse to hand them over. d45's replacement
    # camera, 2026-09-20, printed fy 723 and hfov 70 off an RMS of 5.9:
    # plausible figures, and completely wrong.
    bad = []
    if rms > 1.0:
        bad.append(f"reprojection RMS {rms:.2f} px, over the 1.0 limit")
    # Square pixels: fx and fy are the SAME focal length measured twice, and
    # they are fitted independently, so a split between them reads directly on
    # how badly the fit went rather than on the lens. d45 split 914.7 / 722.8.
    split = abs(fx - fy) / max(fx, fy)
    if split > 0.05:
        bad.append(f"fx and fy differ by {split * 100:.0f} percent "
                   f"({fx:.0f} vs {fy:.0f}); the pixels are square, so these "
                   f"must agree to about 1")
    k1, k2, p1v, p2v = (float(v) for v in dist.ravel()[:4])
    if abs(p1v) > 0.01 or abs(p2v) > 0.01:
        bad.append(f"tangential distortion p1={p1v:+.3f} p2={p2v:+.3f}; a lens "
                   f"that is not visibly crooked reads near zero")
    if obliqs[-1] < 25.0:
        bad.append(f"every view is nearly face-on (most oblique {obliqs[-1]:.0f} deg). "
                   f"A flat-on checkerboard cannot separate focal length from "
                   f"distance, so fx and fy are free to drift apart - which is "
                   f"exactly what they did")
    if abs(k2) > 1.0:
        bad.append(f"k2={k2:+.2f} is outside the physical range, which is where "
                   f"an optimiser puts error it cannot otherwise explain")
    if bad:
        print("")
        print("  CALIBRATION FAILED - do NOT fly these numbers:")
        for b in bad:
            print(f"    - {b}")
        print("")
        if worst[0][0] > 3.0 * per_view[len(per_view) // 2][0]:
            print(f"  {worst[0][2]} alone is far worse than the rest "
                  f"({worst[0][0]:.1f} px). Delete that one view and re-solve "
                  f"before re-grabbing everything.")
            print("")
        print("  The cause is always the captures:")
        print("    1. FLAT AND RIGID. Tape the board to foam board or a clipboard.")
        print("       Paper curls a few millimetres and that is enough.")
        print("    2. STILL, and well lit. Motion blur moves corners several pixels")
        print("       straight into the RMS, and a dim room lengthens the exposure.")
        print("    3. DIFFERENT. Near 0.4 m and far 1.5 m, tilted 20-40 degrees each")
        print("       way, and in all four corners of the frame. Similar views let")
        print("       focal length and distortion trade against each other.")
        print("")
        print("  Delete the old captures first - solve uses everything in the folder:")
        print(f"    rm -f {args.out}/*.png")
        print("    python3 -m hardware.camcal_board grab")
        return 1

    print(f"\n  -> --fy {fy:.0f} --cam-hfov {hfov:.0f}")
    print("\nNotes:")
    print("  * the runtime uses a pinhole model with no undistortion, so k1/k2")
    print("    are measured here but not applied. A large k1 means gates near")
    print("    the frame edge carry a bearing error the estimator cannot see.")
    print("  * a non-zero principal-point offset is a CONSTANT bearing bias on")
    print("    every fix. The debrief reports that as a camera boresight error.")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("grab", help="capture board views")
    g.add_argument("--camera", default=None)
    g.add_argument("--out", default=DEFAULT_DIR)
    g.add_argument("--want", type=int, default=20)
    g.add_argument("--still-px", type=float, default=2.0,
                   help="a corner may not move more than this between consecutive "
                        "frames. The sensor has a rolling shutter, so a moving board "
                        "is SKEWED, not merely blurred, and fits no camera model.")
    g.add_argument("--min-sharp", type=float, default=60.0,
                   help="minimum Laplacian variance over the board itself; low means "
                        "motion blur or a dim room")
    g.add_argument("--min-move-px", type=float, default=40.0,
                   help="reject a view whose board centre has not moved this far")
    g.set_defaults(func=grab)
    s = sub.add_parser("solve", help="calibrate from the captured views")
    s.add_argument("--out", default=DEFAULT_DIR)
    s.add_argument("--square-mm", type=float, required=True,
                   help="MEASURED square size of the printed board, mm")
    s.set_defaults(func=solve)
    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
