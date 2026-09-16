"""
Run the HSV gate detector over a recorded video and write an annotated
copy, for tuning the thresholds against real footage (the organizer DVR,
or a recording from the Orin camera). Run from src/.

    python -m perception.video_probe "../event_files/.../two lap (clean)/PICT0009.AVI"
    python -m perception.video_probe clip.avi --out ../out/dvr_probe.mp4 --start 20 --seconds 30 --step 2

Left: the frame with every blob's outer box (thin), the BIGGEST blob's
outer box (thick, the one the runtime would fix on), its opening box and
the range the runtime would compute from the box width at --fy.
Right: the HSV mask. Prints one line per second and a summary: frames
with a detection, blobs per frame, the biggest blob's size and offsets.

Thresholds live in perception/detectors/hsv_classic.py; edit, rerun, watch.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np

from perception.detectors.hsv_classic import gate_mask
from perception.gate_detection import mask_to_detections

GATE_OUTER_M = 2.7


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("video")
    ap.add_argument("--out", default=None, help="annotated mp4 (default: next to the input, _probe.mp4)")
    ap.add_argument("--start", type=float, default=0.0, help="seconds into the video")
    ap.add_argument("--seconds", type=float, default=None, help="how much to process (default: all)")
    ap.add_argument("--step", type=int, default=1, help="process every Nth frame")
    ap.add_argument("--fy", type=float, default=None, help="focal length px, for the range readout (default: frame width, i.e. 90 deg)")
    args = ap.parse_args(argv)

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"cannot open {args.video}"); return 2
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fy = args.fy or float(w)
    out_path = args.out or os.path.splitext(args.video)[0] + "_probe.mp4"
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps / args.step, (2 * w, h))
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(args.start * fps))
    end_frame = n if args.seconds is None else min(n, int((args.start + args.seconds) * fps))
    print(f"{args.video}: {w}x{h} {fps:.1f} fps, {n} frames; processing every {args.step} from {args.start:.0f} s -> {out_path}")

    stats = {"frames": 0, "with_det": 0, "blobs": 0, "big_area": [], "big_w": []}
    t_print = -1.0
    while True:
        pos = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        if pos >= end_frame:
            break
        ok, bgr = cap.read()
        if not ok:
            break
        if (pos - int(args.start * fps)) % args.step:
            continue
        t = pos / fps
        mask = gate_mask(bgr)
        dets = mask_to_detections(mask, bgr.shape)
        stats["frames"] += 1
        stats["blobs"] += len(dets)
        vis = bgr.copy()
        for d in dets[1:]:
            x, y, bw, bh = d.ring_bbox or d.bbox
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 200, 255), 1)
        if dets:
            stats["with_det"] += 1
            g = dets[0]
            x, y, bw, bh = g.ring_bbox or g.bbox
            stats["big_area"].append(g.area_frac); stats["big_w"].append(bw)
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 3)
            if g.opening_bbox:
                ox, oy, ow, oh = g.opening_bbox
                cv2.rectangle(vis, (ox, oy), (ox + ow, oy + oh), (255, 0, 0), 2)
            rng = fy * GATE_OUTER_M / max(bw, 1)
            cv2.putText(vis, f"biggest: off ({g.offset_x:+.2f},{g.offset_y:+.2f}) w {bw}px range {rng:.1f} m  blobs {len(dets)}",
                        (8, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        else:
            cv2.putText(vis, "no detection", (8, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(vis, f"t={t:6.2f}s", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        side = np.hstack([vis, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        writer.write(side)
        if t - t_print >= 1.0:
            t_print = t
            g = dets[0] if dets else None
            print(f"t={t:6.1f} blobs={len(dets):2d} " + (f"biggest off=({g.offset_x:+.2f},{g.offset_y:+.2f}) area={g.area_frac:.4f} w={(g.ring_bbox or g.bbox)[2]}px opening={'yes' if g.opening_bbox else 'no'}" if g else "none"))
    writer.release()
    cap.release()
    f = max(1, stats["frames"])
    print(f"frames {stats['frames']}, with a detection {stats['with_det']} ({100.0 * stats['with_det'] / f:.0f} %), "
          f"blobs per frame {stats['blobs'] / f:.1f}, biggest blob area median {np.median(stats['big_area']) if stats['big_area'] else 0:.4f}, "
          f"width median {np.median(stats['big_w']) if stats['big_w'] else 0:.0f} px")
    print(f"annotated video: {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
