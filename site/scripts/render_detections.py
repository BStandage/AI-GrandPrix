"""Draw the flight detector's output over the organizers' FPV lap.

    python site/scripts/render_detections.py [--t0 0 --t1 60] [--every 2]

Reads docs/archer_AIGP.mp4, resizes each frame to the aircraft's 1280x720 so
the detector sees exactly what it sees in flight, runs perception.gate_mask +
mask_to_detections, and draws on frames that HAVE a detection: the ring box,
its centre, the horizontal/vertical offsets, the width-based range, and the
commit rule's verdict. Frames with no detection are left untouched.
Encodes H.264 with ffmpeg to site/public/video/hsv_lap.mp4.
"""
import argparse
import os
import subprocess
import sys

import cv2
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))
from perception.detectors.hsv_classic import gate_mask  # noqa: E402
from perception import gate_detection as gd  # noqa: E402

RED = (60, 60, 230)
WHITE = (240, 240, 240)
GREEN = (90, 200, 120)
AMBER = (60, 180, 245)


def draw(bgr, d, committed, why, t):
    h, w = bgr.shape[:2]
    if d.ring_bbox is not None:
        x, y, bw, bh = d.ring_bbox
        cv2.rectangle(bgr, (x, y), (x + bw, y + bh), RED if committed else GREEN, 2)
    cx = int((d.offset_x + 1) * 0.5 * w)
    cy = int((d.offset_y + 1) * 0.5 * h)
    cv2.drawMarker(bgr, (cx, cy), WHITE, cv2.MARKER_CROSS, 26, 2)
    cv2.line(bgr, (w // 2, h // 2), (cx, cy), AMBER, 1)
    cv2.drawMarker(bgr, (w // 2, h // 2), AMBER, cv2.MARKER_TILTED_CROSS, 14, 1)
    lines = [
        f"t {t:5.1f} s",
        f"offset x {d.offset_x:+.2f}  y {d.offset_y:+.2f}",
        f"range ~{d.distance_m:.1f} m (from width)",
        f"fill {d.area_frac * 100:4.1f} %",
        ("COMMIT: " + why) if committed else why,
    ]
    y0 = 28
    for i, s in enumerate(lines):
        col = RED if (committed and i == len(lines) - 1) else WHITE
        cv2.putText(bgr, s, (16, y0 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(bgr, s, (16, y0 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.62, col, 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", default=os.path.join(ROOT, "docs", "archer_AIGP.mp4"))
    ap.add_argument("--out", default=os.path.join(ROOT, "site", "public", "video", "hsv_lap.mp4"))
    ap.add_argument("--t0", type=float, default=0.0)
    ap.add_argument("--t1", type=float, default=1e9)
    ap.add_argument("--every", type=int, default=2, help="keep every Nth frame (2: 60 fps -> 30 fps)")
    a = ap.parse_args()
    cap = cv2.VideoCapture(a.video)
    fps = cap.get(cv2.CAP_PROP_FPS)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    W, H = 1280, 720
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W}x{H}",
         "-r", f"{fps / a.every:.3f}", "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "24",
         "-pix_fmt", "yuv420p", "-movflags", "+faststart", a.out],
        stdin=subprocess.PIPE)
    i = kept = hits = 0
    while True:
        ok = cap.grab()
        if not ok:
            break
        t = i / fps
        if i % a.every == 0 and a.t0 <= t <= a.t1:
            ok, bgr = cap.retrieve()
            if not ok:
                break
            bgr = cv2.resize(bgr, (W, H))
            dets = gd.mask_to_detections(gate_mask(bgr), bgr.shape)
            if dets:
                d = dets[0]
                committed, why = gd.commit_reason(d, bgr.shape)
                draw(bgr, d, committed, why, t)
                hits += 1
            ff.stdin.write(bgr.tobytes())
            kept += 1
        i += 1
    ff.stdin.close()
    ff.wait()
    print(f"{a.out}: {kept} frames, {hits} with a detection ({100.0 * hits / max(kept, 1):.0f}%), "
          f"{os.path.getsize(a.out) // 1024} KB")


if __name__ == "__main__":
    main()
