#
# Dump detector debug images (annotated / mask / extracted opening) to debug/.
# Usage:
#   python -m analysis.debug_detector <frame.jpg> [more.jpg ...]
#   python -m analysis.debug_detector            # uses a few sample frames
#
import os
import sys

import cv2
import numpy as np

# Allow running this file directly (python analysis/debug_detector.py) by putting the
# src/ package root on sys.path, not just this subpackage's folder.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from perception.detectors.hsv_classic import gate_mask
from perception.gate_detection import mask_to_detections, annotate
from common.paths import DEBUG_DIR

OUT = DEBUG_DIR

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
    mask = gate_mask(img)
    dets = mask_to_detections(mask, img.shape)
    cv2.imwrite(os.path.join(OUT, f"{name}_1annotated.jpg"), annotate(img, dets))
    cv2.imwrite(os.path.join(OUT, f"{name}_2mask.jpg"), mask)
    t = dets[0] if dets else None
    info = (f"opening={t.has_opening} off=({t.offset_x:+.2f},{t.offset_y:+.2f}) "
            f"dist={t.distance_m:.1f}m") if t else "no gate"
    print(f"{name}: {info}")


def main():
    os.makedirs(OUT, exist_ok=True)
    paths = sys.argv[1:] or SAMPLES
    for p in paths:
        dump(p)
    print("-> wrote to", OUT)


if __name__ == "__main__":
    main()
