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
import os

import numpy as np

# Camera calibration: single source of truth (ground truth, spec VADR-TS-002).
from common.camera import FX as FOCAL_LENGTH_PX, GATE_OUTER_M as GATE_REAL_HEIGHT_M

MIN_GATE_AREA_FRAC = 0.00015  # ignore blobs smaller than this fraction of the frame


# Reconstruct the centre of a vertically clipped ring from its width. On by
# default; AIGP_GATE_UNCLIP=0 restores the raw centroid.
UNCLIP_V = os.environ.get("AIGP_GATE_UNCLIP", "1") != "0"
CLIP_EDGE_PX = 3          # within this many px of the edge counts as cut off
# COMMIT. Past this apparent size the gate is too close to steer height on:
# release the vertical reference and fly through on the height already held.
# 0.85 of the frame is about 3.9 m for a 2.7 m ring in a 45.5 deg field.
COMMIT_FRAC = float(os.environ.get("AIGP_GATE_COMMIT_FRAC", "0.85"))

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
    ring_bbox: Optional[Tuple] = None  # outer orange ring box (stable; offsets use its center)
    opening_bbox: Optional[Tuple] = None  # hole-to-fly-through box (None if not found)
    clipped_v: bool = False            # ring ran off the top/bottom edge; offset_y reconstructed
    v_usable: bool = True             # False = offset_y carries no usable elevation, do not steer height on it
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


def principal_point(w, h):
    """Optical centre in pixels. AIGP_CAM_CX / AIGP_CAM_CY override it with
    what camcal_board measured; unset means the image centre, which is what
    the sim's synthetic camera assumes."""
    cx = os.environ.get("AIGP_CAM_CX")
    cy = os.environ.get("AIGP_CAM_CY")
    return (float(cx) if cx else w / 2.0,
            float(cy) if cy else h / 2.0)


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

        ring_bbox = (rx, ry, rw, rh)
        if big is not None and cv2.contourArea(big) > 0.10 * ring_area:
            opening_bbox = cv2.boundingRect(big)   # hole to fly through
            has_opening = True
            x, y, bw, bh = opening_bbox
        else:
            opening_bbox = None
            has_opening = False
            x, y, bw, bh = ring_bbox

        # Flight offsets aim at the OUTER RING center (stable under angle/occlusion).
        # Map elevation must use opening_bbox when present — ring vs opening is ~1 m
        # vertically (atan(1/3)≈18°) and that was the false "+17° tilt" on z.
        cx, cy = rx + rw / 2.0, ry + rh / 2.0
        # VERTICALLY CLIPPED RING (d44, race day 1, 2026-09-21). The camera is
        # tilted UP 20 deg with a 45.5 deg vertical field, so the frame bottom
        # sits at -2.75 deg of elevation: a gate at the aircraft's OWN height
        # is cut off along the bottom at every useful range. The visible
        # centroid then sits ABOVE the true centre, the vertical loop reads
        # that as "the gate is above you", dz_vision pins at its +0.35 m cap
        # and the aircraft climbs - and climbing cuts off more of the gate, so
        # it runs away. Flown: takeoff put her on gate centre correctly, then
        # she climbed to roughly a metre over the top bar.
        #
        # The ring is SQUARE - 2.7 x 2.7 m per the course map - and the WIDTH
        # is never clipped (the horizontal channel was accurate throughout the
        # same flight). So the true height in pixels IS the width, and the
        # true centre is reconstructed from the unclipped edge. Exact for a
        # square gate, not a fudge factor.
        #
        # Guarded on rw > rh: yaw foreshortening makes a gate NARROWER, never
        # taller, so a box wider than it is tall means vertical truncation.
        clipped_v, v_usable = False, True
        if UNCLIP_V:
            at_bottom = (ry + rh) >= (h - CLIP_EDGE_PX)
            at_top = ry <= CLIP_EDGE_PX
            # COMMIT THROUGH THE GATE (d44, race day 1, 2026-09-21). Flown:
            # she tracked beautifully from 7.4 m in to 2.5 m - xtrack 0.22 m,
            # res 0.02 - then at det 1.81 m corrected UP, 0.89 -> 1.44 -> 1.91,
            # and clipped the top bar. She really was low, but a vertical
            # correction begun 1.8 m from a gate has nowhere to go except into
            # a bar. Inside this range the aircraft is committed: hold the
            # height it already has and fly through.
            #
            # Measured by APPARENT SIZE, not range_m - range is the camera's
            # worst signal and read 12.6, 22.9, 59.5 m on this same flight,
            # while the ring's height in pixels is a direct observation.
            if rh >= COMMIT_FRAC * h:
                v_usable = False
            elif at_bottom and at_top:
                # BOTH edges cut: the gate overfills the frame and there is no
                # unclipped vertical edge to rebuild the centre from. Flown on
                # d44 2026-09-21: she held gate centre out to 4 m, then climbed
                # from the moment `det` fell under ~3 m, which is where a 2.7 m
                # ring stops fitting in a 45.5 deg vertical field. Reconstruction
                # silently switched off there and the raw centroid took over.
                # Say so instead of guessing - the follower then fades its
                # vertical reference out and holds the height it already had,
                # which is the height the gate gave it while it could still see
                # the whole thing.
                v_usable = False
            elif rw > rh and at_bottom != at_top:      # exactly one edge cut
                cy = (ry + rw / 2.0) if at_bottom else ((ry + rh) - rw / 2.0)
                clipped_v = True
        # Bearings are measured from the OPTICAL axis, not the image centre.
        # camcal_board on d45 (2026-09-20) put the principal point at
        # (613, 387) in a 1280x720 frame - 27 px off in both axes, which is a
        # CONSTANT -1.84 deg horizontal and +1.87 deg vertical bias on every
        # fix, about 0.26 m of lateral error at 8 m, always the same way.
        # Defaults are the image centre, so the sim is unchanged.
        px, py = principal_point(w, h)
        dets.append(GateDetection(
            offset_x=(cx - px) / (w / 2.0),
            offset_y=(cy - py) / (h / 2.0),
            area=ring_area,
            # distance from the outer ring box (rw,rh ~ 2.7 m frame), not the aim box. When we aim at
            # the inner opening the box shrinks, and dividing by 2.7 would read ~1.8x too far.
            distance_m=FOCAL_LENGTH_PX * GATE_REAL_HEIGHT_M / max(rw, rh, 1),
            area_frac=ring_area / float(h * w),
            has_opening=has_opening,
            bbox=(x, y, bw, bh),
            ring_bbox=ring_bbox,
            opening_bbox=opening_bbox,
            clipped_v=clipped_v,
            v_usable=v_usable,
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
