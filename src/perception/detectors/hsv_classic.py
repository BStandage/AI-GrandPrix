"""
Classic HSV color segmentation of the red/orange gate ring.

The gate is a bright red/orange ring on a dull grey world, so a fixed HSV color threshold isolates it
cleanly with no training data.

segment() applies the HSV threshold and returns the resulting mask (gate pixels white, everything
else black). mask_to_detections() in gate_detection.py then converts that mask into
GateDetections (the gate's image position, size, and rough range).

You can tune the HSV range against a sample frame by running this script manually:
    python -m perception.detectors.hsv_classic <frame.jpg>
"""

import cv2
import numpy as np

from perception.detector import MaskDetector

# HSV thresholds, each (hue, sat, val) min and max for cv2.inRange. Red falls at both the low and
# high end of OpenCV's 0-179 hue scale, so it takes two ranges to catch all of it.
GATE_HSV_LOWER1 = (0, 80, 70)       # hue 0-22 (red/orange). sat/val mins of 80/70 drop the grey world
GATE_HSV_UPPER1 = (22, 255, 255)
GATE_HSV_LOWER2 = (160, 80, 70)     # hue 160-180 (the rest of red, at the top of the scale)
GATE_HSV_UPPER2 = (180, 255, 255)


def gate_mask(img):
    """Binary mask of the gate ring."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Apply both HSV ranges and OR the results into one mask.
    mask = cv2.bitwise_or(cv2.inRange(hsv, GATE_HSV_LOWER1, GATE_HSV_UPPER1),
                          cv2.inRange(hsv, GATE_HSV_LOWER2, GATE_HSV_UPPER2))

    # Close to fill the gaps the AI-GP text punches in the ring, then open to remove speckle. The
    # opening is recovered later via convex hull, so making the ring solid here only helps.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    return mask


class HsvClassic(MaskDetector):
    """HSV color-threshold detector (the reference mask detector)."""

    def segment(self, image):
        return gate_mask(image)


if __name__ == "__main__":
    # CLI for tuning the HSV range: dumps an annotated frame + the raw mask to debug/.
    import os
    import sys
    from perception.gate_detection import mask_to_detections, annotate
    from common.paths import DEBUG_DIR
    if len(sys.argv) < 2:
        sys.exit("usage: python -m perception.detectors.hsv_classic <path-to-frame.jpg>")
    path = sys.argv[1]
    img = cv2.imread(path)
    if img is None:
        sys.exit(f"could not read image: {path!r}")
    mask = gate_mask(img)
    dets = mask_to_detections(mask, img.shape)
    print(f"{len(dets)} gate(s):")
    for i, d in enumerate(dets):
        print(f"  [{i}] bbox={d.bbox} area={d.area:.0f} opening={d.has_opening} "
              f"offset_x={d.offset_x:+.2f} dist={d.distance_m:.1f}m")

    # Name the outputs after the input frame so successive runs don't overwrite each other.
    os.makedirs(DEBUG_DIR, exist_ok=True)
    name = os.path.splitext(os.path.basename(path))[0]
    annotated_path = os.path.join(DEBUG_DIR, f"{name}_annotated.jpg")
    mask_path = os.path.join(DEBUG_DIR, f"{name}_mask.jpg")
    cv2.imwrite(annotated_path, annotate(img, dets))
    cv2.imwrite(mask_path, mask)
    print(f"wrote {annotated_path} and {mask_path}")
