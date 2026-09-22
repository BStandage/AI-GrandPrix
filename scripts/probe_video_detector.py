"""Run the real gate detector over a flown video and report what it would have
told the follower: detection rate, the centre it picked, the range it inferred,
and when it declared the ring clipped / filling the frame (the COMMIT rule).

    cd src && python ../scripts/probe_video_detector.py ../docs/archer_AIGP.mp4 [--fps 4] [--t0 0 --t1 60]

Frames are resized to the drone's 1280x720 before detection so the frame
fractions, clip margins and commit thresholds are the flight ones. The video's
own focal length is unknown, so ranges are relative (set AIGP_PROBE_FY to a
guess); the fill fraction, clipping and centre offsets do not depend on it.
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from perception.detectors.hsv_classic import gate_mask          # noqa: E402
from perception import gate_detection as gd                       # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--fps", type=float, default=4.0)
    ap.add_argument("--t0", type=float, default=0.0)
    ap.add_argument("--t1", type=float, default=1e9)
    ap.add_argument("--w", type=int, default=1280)
    ap.add_argument("--h", type=int, default=720)
    a = ap.parse_args()
    cap = cv2.VideoCapture(a.video)
    vfps = cap.get(cv2.CAP_PROP_FPS)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    step = max(1, int(round(vfps / a.fps)))
    print(f"{a.video}: {n} frames at {vfps:.1f} fps -> sampling every {step} frames; detector frame {a.w}x{a.h}; "
          f"COMMIT_FRAC {gd.COMMIT_FRAC}, CLIP_EDGE_PX {gd.CLIP_EDGE_PX}")
    print(f"{'t':>6} {'n':>2} {'off_x':>6} {'off_y':>6} {'area':>6} {'range':>6} {'rw':>4} {'rh':>4} {'clipV':>5} {'vUse':>4} {'commit':>13}  reason")
    rows = []
    i = 0
    while True:
        ok = cap.grab()
        if not ok:
            break
        if i % step == 0:
            t = i / vfps
            if t >= a.t0 and t <= a.t1:
                ok, bgr = cap.retrieve()
                if not ok:
                    break
                bgr = cv2.resize(bgr, (a.w, a.h))
                mask = gate_mask(bgr)
                dets = gd.mask_to_detections(mask, bgr.shape)
                if dets:
                    d = dets[0]
                    rx, ry, rw, rh = d.ring_bbox
                    committed, why = gd.commit_reason(d, bgr.shape)
                    rows.append((t, len(dets), d.offset_x, d.offset_y, d.area_frac, d.distance_m, rw, rh, d.clipped_v, d.v_usable, committed, why))
                    print(f"{t:6.2f} {len(dets):2d} {d.offset_x:+6.2f} {d.offset_y:+6.2f} {d.area_frac:6.3f} {d.distance_m:6.2f} {rw:4d} {rh:4d} {str(d.clipped_v)[0]:>5} {str(d.v_usable)[0]:>4} {str(committed):>13}  {why}")
                else:
                    rows.append((t, 0) + (None,) * 10)
                    print(f"{t:6.2f}  0")
        i += 1
    tot = len(rows); hit = sum(1 for r in rows if r[1] > 0)
    print(f"\ndetection rate: {hit}/{tot} sampled frames ({100.0 * hit / max(tot, 1):.0f}%)")
    reasons = {}
    for r in rows:
        if r[1] > 0:
            reasons[r[11]] = reasons.get(r[11], 0) + 1
    print("commit reasons over detected frames:", reasons)


if __name__ == "__main__":
    main()
