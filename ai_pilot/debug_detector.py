#
# Dump detector debug images (annotated / mask / extracted opening) to debug/.
# Usage:
#   python debug_detector.py <frame.jpg> [more.jpg ...]
#   python debug_detector.py            # uses a few sample frames
#
import os
import sys

import cv2
import numpy as np

from gate_detector import detect_gates, annotate, gate_mask

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "debug")

SAMPLES = [
    "datasets/session_20260605_221219/frames/0000000119.jpg",
    "datasets/session_20260606_181105/frames/0000000453.jpg",
    "datasets/session_20260606_182218/frames/0000000910.jpg",
]


def dump(path):
    img = cv2.imread(path)
    if img is None:
        print("missing:", path)
        return
    name = os.path.splitext(os.path.basename(path))[0]
    dets, mask = detect_gates(img)
    cv2.imwrite(os.path.join(OUT, f"{name}_1annotated.jpg"), annotate(img, dets))
    cv2.imwrite(os.path.join(OUT, f"{name}_2mask.jpg"), mask)
    t = dets[0] if dets else None
    info = (f"opening={t['has_opening']} off=({t['offset_x']:+.2f},{t['offset_y']:+.2f}) "
            f"dist={t['distance_m']:.1f}m") if t else "no gate"
    print(f"{name}: {info}")


def main():
    os.makedirs(OUT, exist_ok=True)
    paths = sys.argv[1:] or SAMPLES
    for p in paths:
        dump(p)
    print("-> wrote to", OUT)


if __name__ == "__main__":
    main()
