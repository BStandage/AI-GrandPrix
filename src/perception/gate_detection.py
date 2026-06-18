"""
The perception contract (GateDetection) and the helper that derives it from a mask.

A GateDetection is everything the flight code needs about one gate. Four fields matter to flight:

    offset_x     gate position left/right in the image, from -1 (left edge) to +1 (right edge)
    offset_y     gate position up/down in the image,     from -1 (top edge)  to +1 (bottom edge)
    area         apparent size in pixels (larger = nearer; used to pick the nearest gate)
    distance_m   rough range to the gate, in meters

The offsets are the reliable signal and aim the pilot at the opening. distance_m is weak. The pilot
only nudges its depth estimate with it, so accuracy there matters little. The remaining fields
(area_frac, has_opening, bbox, center, corners, gate_id) are extras for logging and offline tools.
Flight ignores them, so detectors may leave them at their defaults.

mask_to_detections() converts a binary gate mask into the GateDetection list. It finds the ring,
locates the opening to fly through, and measures the bearing and rough range. Mask-based detectors
reuse it so they only segment the gate pixels.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

# Camera calibration: single source of truth (ground truth, spec VADR-TS-002).
from common.camera import FX as FOCAL_LENGTH_PX, GATE_OUTER_M as GATE_REAL_HEIGHT_M

MIN_GATE_AREA_FRAC = 0.00015  # ignore blobs smaller than this fraction of the frame


@dataclass
class GateDetection:
    """One gate as seen by perception. See the module docstring for what each field means."""
    # the four numbers the flight code uses:
    offset_x: float                    # -1 (left edge) .. +1 (right edge)
    offset_y: float                    # -1 (top edge)  .. +1 (bottom edge)
    area: float                        # ring size in pixels (bigger = nearer)
    distance_m: float                  # rough range in meters (weak signal)
    # extras for logging and offline pnp. Flight ignores these, so they are optional:
    area_frac: float = 0.0             # area as a fraction of the frame
    has_opening: bool = False          # true if the actual hole-to-fly-through was located
    bbox: Optional[Tuple] = None       # (x, y, w, h) of the aim target (opening, else ring)
    center: Optional[Tuple] = None     # (cx, cy) pixel center of the aim target
    corners: Optional[np.ndarray] = None   # ordered ring corners (TL,TR,BR,BL) for solvePnP
    gate_id: Optional[int] = None      # ground-truth id, when a synthetic source knows it (sim only)


def ring_corners(contour):
    """Ordered 4 corners (TL,TR,BR,BL) of one red-ring contour: a 4-point polygon approximation of
    the convex hull, falling back to the min-area rotated rect. Used for per-detection solvePnP."""
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


def mask_to_detections(mask, img_shape, min_area_frac=MIN_GATE_AREA_FRAC):
    """Turn a black-and-white gate mask into a list of GateDetection, nearest gate first. This is
    the geometry a mask-based detector inherits, so it only has to mark the gate pixels.

    The gate is a thick ring, and what we fly through is the hole, not the orange. So for each ring
    we find its largest interior hole (fill the convex hull, then subtract the ring) and aim at that,
    falling back to the ring's box when no clear hole is visible (far away, or partly out of frame)."""
    h, w = img_shape[:2]
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

        # the gate opening: fill the ring polygon solid, subtract the orange, and what's left is the
        # hole. Robust to gaps in the ring (the AI-GP text breaks the orange loop).
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
        dets.append(GateDetection(
            offset_x=(cx - w / 2.0) / (w / 2.0),
            offset_y=(cy - h / 2.0) / (h / 2.0),
            area=ring_area,
            # distance from the outer ring box (rw,rh ~ 2.7 m frame), not the aim box. When we aim at
            # the inner opening the box shrinks, and dividing by 2.7 would read ~1.8x too far.
            distance_m=FOCAL_LENGTH_PX * GATE_REAL_HEIGHT_M / max(rw, rh, 1),
            area_frac=ring_area / float(h * w),
            has_opening=has_opening,
            bbox=(x, y, bw, bh),
            center=(cx, cy),
            corners=ring_corners(c),
        ))
    dets.sort(key=lambda d: d.area, reverse=True)
    return dets


def annotate(img, detections):
    """Draw detections on a copy of img (nearest gate green, others amber). For debug dumps."""
    out = img.copy()
    for i, d in enumerate(detections):
        x, y, bw, bh = d.bbox
        color = (0, 255, 0) if i == 0 else (0, 200, 255)
        cv2.rectangle(out, (x, y), (x + bw, y + bh), color, 2)
        cv2.circle(out, (int(d.center[0]), int(d.center[1])), 4, color, -1)
        cv2.putText(out, f"{i}", (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return out
