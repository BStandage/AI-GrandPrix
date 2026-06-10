#
# Classic-CV gate detector for the high-contrast (Round 1) AI-GP gates.
#
# The gate is a bright red/orange square ring on a desaturated grey world, so an HSV
# color threshold segments it cleanly - no training data or ML needed. We
# threshold the red/orange, find blobs, and return each gate's bounding box + centre,
# sorted nearest-first (biggest = closest). This gives the gate's bearing in the
# image immediately; pose (distance/orientation) comes later via PnP.
#
# Tune the HSV range with: python gate_detector.py <a_frame.jpg>
#

import cv2
import numpy as np

# Camera calibration: single source of truth (ground truth, spec VADR-TS-002).
from common.camera import FX as FOCAL_LENGTH_PX, GATE_OUTER_M as GATE_REAL_HEIGHT_M

# RED-ORANGE gate in OpenCV HSV (H: 0-179). The gate measures H~5 (a warm vermilion); we take a
# low band up to H~22 (red through orange, for lighting/edge robustness) PLUS the wrap-around band
# near 180, since red straddles both ends of the hue circle. High S/V floors keep the desaturated
# grey world out and EXCLUDE the bright BLUE racing line (H~100), the most saturated thing in frame.
GATE_HSV_LOWER1 = (0, 80, 70)
GATE_HSV_UPPER1 = (22, 255, 255)
GATE_HSV_LOWER2 = (160, 80, 70)
GATE_HSV_UPPER2 = (180, 255, 255)

MIN_GATE_AREA_FRAC = 0.00015  # ignore blobs smaller than this fraction of the frame
# (FOCAL_LENGTH_PX, GATE_REAL_HEIGHT_M imported from common.camera above. Pinhole distance estimate:
#  distance = f * real_height / pixel_height.)


def ring_corners(contour):
    """Ordered 4 corners (TL,TR,BR,BL) of one red-ring contour: a 4-point polygon approximation of
    the convex hull, falling back to the min-area rotated rect. Same logic vision_pose uses on the
    largest ring, exposed per-contour so the offline collector can solvePnP EVERY detection."""
    hull = cv2.convexHull(contour)
    peri = cv2.arcLength(hull, True)
    quad = None
    for k in (0.02, 0.04, 0.06, 0.08, 0.10):
        approx = cv2.approxPolyDP(hull, k * peri, True)
        if len(approx) == 4:
            quad = approx.reshape(4, 2).astype(np.float32)
            break
    if quad is None:
        quad = cv2.boxPoints(cv2.minAreaRect(contour)).astype(np.float32)
    s = quad.sum(axis=1)
    d = quad[:, 0] - quad[:, 1]
    return np.array([quad[np.argmin(s)], quad[np.argmax(d)],
                     quad[np.argmax(s)], quad[np.argmin(d)]], dtype=np.float32)


def gate_mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(cv2.inRange(hsv, GATE_HSV_LOWER1, GATE_HSV_UPPER1),
                          cv2.inRange(hsv, GATE_HSV_LOWER2, GATE_HSV_UPPER2))
    # close to bridge the gaps the AI-GP text punches in the ring (so it's a solid
    # band), then despeckle. The gate OPENING is recovered separately via convex
    # hull, so solidifying the ring here only helps.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    return mask


def detect_gates(img, min_area_frac=MIN_GATE_AREA_FRAC, with_corners=False):
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
        det = {
            "bbox": (x, y, bw, bh),
            "center": (cx, cy),
            "area": ring_area,
            "area_frac": ring_area / float(h * w),
            "has_opening": has_opening,
            "offset_x": (cx - w / 2.0) / (w / 2.0),
            "offset_y": (cy - h / 2.0) / (h / 2.0),
            # distance from the OUTER ring bbox (rw,rh ~ 2.72 m frame), NOT the aim bbox - when we
            # aim at the inner opening (~1.5 m) the bbox shrinks, and dividing by 2.72 would read
            # ~1.8x too far. The ring is also the most directly-detected extent (the red blob).
            "distance_m": FOCAL_LENGTH_PX * GATE_REAL_HEIGHT_M / max(rw, rh, 1),
        }
        if with_corners:
            det["corners"] = ring_corners(c)   # ordered 4 corners for per-detection solvePnP
        dets.append(det)
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
    if len(sys.argv) < 2:
        sys.exit("usage: python -m perception.gate_detector <path-to-frame.jpg>")
    path = sys.argv[1]
    img = cv2.imread(path)
    if img is None:
        sys.exit(f"could not read image: {path!r}\n"
                 f"  (cv2.imread returned None - check the path exists relative to your current "
                 f"directory. With `-m` the path is relative to src/, e.g. datasets/session_*/frames/xxxx.jpg)")
    dets, mask = detect_gates(img)
    print(f"{len(dets)} gate(s):")
    for i, d in enumerate(dets):
        print(f"  [{i}] bbox={d['bbox']} area={d['area']:.0f} opening={d['has_opening']} "
              f"offset_x={d['offset_x']:+.2f} dist={d['distance_m']:.1f}m")
    cv2.imwrite("gate_annotated.jpg", annotate(img, dets))
    cv2.imwrite("gate_mask.jpg", mask)
    print("wrote gate_annotated.jpg and gate_mask.jpg")
