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
#
# Tuning (Phase 2). Two failure modes, balanced by two different knobs measured on real frames:
#
#   1. Ring FRAGMENTS into separate blobs -> mask_to_detections (RETR_EXTERNAL) reports each blob as
#      its own gate (one gate seen as 3). Caused by too-high a SAT/VAL floor: the glowing ring itself
#      sits at sat 70-140, val 60-130 in dimmer/angled views, so the old 170/110 floor sliced through
#      it. Fix: floor at 85/75 keeps the whole ring. The near-white bloom/HALO is still rejected by
#      SATURATION (halo sat < ~60), which is the real halo discriminator - not the height of the floor.
#
#   2. Orange FLOOR SPILL (the gate's reflection on the road, directly below it) gets pulled in and,
#      once the close welds it to the ring, balloons the gate box downward so the aim point sinks into
#      the road. The spill is separable by HUE: the ring's bright core is hue ~5, the spill is hue ~19
#      (yellower) and dimmer (val ~73). Capping the upper hue at 12 drops the spill while keeping the
#      red ring body. (The ring's own yellow bloom fringe at hue 14-19 is also dropped - that fringe
#      is exactly what inflated the box, so losing it is a feature.)
#
# Tune against a real frame: python -m perception.detectors.hsv_classic <f.jpg>
GATE_HSV_LOWER1 = (0,  85, 75)
GATE_HSV_UPPER1 = (12, 255, 255)
GATE_HSV_LOWER2 = (169, 85, 75)
GATE_HSV_UPPER2 = (180, 255, 255)


def gate_mask(img):
    """Binary mask of the gate ring."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Apply both HSV ranges and OR the results into one mask.
    mask = cv2.bitwise_or(cv2.inRange(hsv, GATE_HSV_LOWER1, GATE_HSV_UPPER1),
                          cv2.inRange(hsv, GATE_HSV_LOWER2, GATE_HSV_UPPER2))

    # Close to RECONNECT the ring into one component: the AI-GP sign + dark opening punch a gap through
    # the top, and unless the broken posts/bars rejoin into a single contour, RETR_EXTERNAL in
    # mask_to_detections splits one gate into several detections. 9x9 x2 bridges those gaps while
    # leaving the central opening intact (recovered later via convex hull). Don't push iterations
    # higher: an over-strong close re-welds any residual floor spill onto the ring. Then open speckle.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8), iterations=2)
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
