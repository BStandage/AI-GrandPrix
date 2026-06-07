#
# Classic-CV gate detector for the high-contrast (Round 1) AI-GP gates.
#
# The gate is a bright orange ring on a desaturated world, so an HSV colour
# threshold segments it cleanly - no training data or ML needed. We threshold
# the orange, find blobs, and return each gate's bounding box + centre, sorted
# nearest-first (biggest = closest). This gives the gate's bearing in the image
# immediately; pose (distance/orientation) comes later via PnP once we have the
# camera intrinsics.
#
# Tune the HSV range with: python gate_detector.py <a_frame.jpg>
#

import cv2
import numpy as np

# Orange gate in OpenCV HSV (H: 0-179). Orange/red sits near H=0, so we take a
# low-H band; very high saturation/value because the gate is vivid against grey.
# The world is desaturated grey, so the orange gate is the only saturated thing.
# Wide hue (red-orange), and modest S/V floors to catch the whole ring incl. shaded
# parts - grey stays out (near-zero saturation), the blue racing line is a different hue.
GATE_HSV_LOWER = (0, 70, 60)
GATE_HSV_UPPER = (28, 255, 255)

MIN_GATE_AREA_FRAC = 0.00015  # ignore blobs smaller than this fraction of the frame

# Camera calibration (regressed from a real approach, see controller.py):
#   focal length ~229 px, HFOV ~109 deg, camera tilted UP ~26 deg.
# Gives a pinhole distance estimate: distance = f * real_height / pixel_height.
FOCAL_LENGTH_PX = 229.0
GATE_REAL_HEIGHT_M = 2.7


def gate_mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, GATE_HSV_LOWER, GATE_HSV_UPPER)
    # close to bridge the gaps the AI-GP text punches in the ring (so it's a solid
    # band), then despeckle. The gate OPENING is recovered separately via convex
    # hull, so solidifying the ring here only helps.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    return mask


def detect_gates(img, min_area_frac=MIN_GATE_AREA_FRAC):
    """Return (detections, mask).

    The gate is a THICK orange ring - what we must fly through is the HOLE, not the
    centre of the orange. So for each orange ring we find its largest interior hole
    (via contour hierarchy) and aim at THAT. Falls back to the ring bbox centre when
    no clear hole is visible (far away, or partly out of frame).

    detections (nearest first) each have:
      bbox/center  the OPENING when found, else the ring
      area         ring area (for nearest-first sorting / proximity)
      has_opening  True if we located the actual hole to fly through
      offset_x/y   opening centre offset from image centre, normalized [-1,1]
      distance_m   pinhole distance from the opening height
    """
    h, w = img.shape[:2]
    mask = gate_mask(img)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = min_area_frac * h * w
    notmask = cv2.bitwise_not(mask)
    kernel = np.ones((3, 3), np.uint8)

    dets = []
    for c in contours:
        ring_area = cv2.contourArea(c)
        if ring_area < min_area:
            continue
        rx, ry, rw, rh = cv2.boundingRect(c)

        # THE GATE OPENING (what we fly through): fill the ring polygon solid, then
        # subtract the orange -> what's left is the hole. Robust to gaps in the ring
        # (the AI-GP text breaks the orange loop, so enclosed-contour methods fail).
        filled = np.zeros((h, w), np.uint8)
        cv2.drawContours(filled, [cv2.convexHull(c)], -1, 255, -1)
        hole = cv2.bitwise_and(filled, notmask)
        hole = cv2.morphologyEx(hole, cv2.MORPH_OPEN, kernel, iterations=1)
        hcs, _ = cv2.findContours(hole, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = max(hcs, key=cv2.contourArea, default=None)

        if big is not None and cv2.contourArea(big) > 0.10 * ring_area:
            x, y, bw, bh = cv2.boundingRect(big)   # aim at the opening
            has_opening = True
        else:
            x, y, bw, bh = rx, ry, rw, rh           # fallback: whole ring
            has_opening = False

        cx, cy = x + bw / 2.0, y + bh / 2.0
        dets.append({
            "bbox": (x, y, bw, bh),
            "center": (cx, cy),
            "area": ring_area,
            "area_frac": ring_area / float(h * w),
            "has_opening": has_opening,
            "offset_x": (cx - w / 2.0) / (w / 2.0),
            "offset_y": (cy - h / 2.0) / (h / 2.0),
            "distance_m": FOCAL_LENGTH_PX * GATE_REAL_HEIGHT_M / max(max(bw, bh), 1),
        })
    dets.sort(key=lambda d: d["area"], reverse=True)
    return dets, mask


def annotate(img, dets):
    out = img.copy()
    for i, d in enumerate(dets):
        x, y, bw, bh = d["bbox"]
        color = (0, 255, 0) if i == 0 else (0, 200, 255)
        cv2.rectangle(out, (x, y), (x + bw, y + bh), color, 2)
        cv2.circle(out, (int(d["center"][0]), int(d["center"][1])), 4, color, -1)
        cv2.putText(out, f"{i}", (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return out


if __name__ == "__main__":
    import sys
    path = sys.argv[1]
    img = cv2.imread(path)
    dets, mask = detect_gates(img)
    print(f"{len(dets)} gate(s):")
    for i, d in enumerate(dets):
        print(f"  [{i}] bbox={d['bbox']} area={d['area']:.0f} fill={d['fill']:.2f} "
              f"offset_x={d['offset_x']:+.2f}")
    cv2.imwrite("gate_annotated.jpg", annotate(img, dets))
    cv2.imwrite("gate_mask.jpg", mask)
    print("wrote gate_annotated.jpg and gate_mask.jpg")
