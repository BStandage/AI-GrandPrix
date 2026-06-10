"""
Camera -> gate pose. The perception half of the vision pilot, shared by the offline validator
(pnp_offline.py) and the live vision-shadow check (vision_rx.py).

Pipeline: detect the red gate ring (gate_detector) -> its 4 corners -> cv2.solvePnP against the
known gate size + focal length -> gate centre in the camera -> rotate into the drone BODY frame
(forward, right, down), the SAME quantity the pursuit controller steers on. No ground truth.
"""

import math

import cv2
import numpy as np

from perception.gate_detector import gate_mask, MIN_GATE_AREA_FRAC
from common.gate_geometry import get_drone_pose, relative_gate
# Camera model: single source of truth (ground truth, spec VADR-TS-002).
from common.camera import FX as FOCAL_PX, GATE_OUTER_M as GATE_SIZE_M, UPTILT_RAD as CAM_UPTILT_RAD


def gate_corners(img):
    """The 4 corners (TL,TR,BR,BL) of the largest red gate ring, or None. 4-point polygon
    approximation of the ring's convex hull; falls back to the min-area rotated rectangle."""
    h, w = img.shape[:2]
    mask = gate_mask(img)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) >= MIN_GATE_AREA_FRAC * h * w]
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    hull = cv2.convexHull(c)
    peri = cv2.arcLength(hull, True)
    quad = None
    for k in (0.02, 0.04, 0.06, 0.08, 0.10):
        approx = cv2.approxPolyDP(hull, k * peri, True)
        if len(approx) == 4:
            quad = approx.reshape(4, 2).astype(np.float32)
            break
    if quad is None:
        quad = cv2.boxPoints(cv2.minAreaRect(c)).astype(np.float32)
    s = quad.sum(axis=1)
    d = quad[:, 0] - quad[:, 1]
    return np.array([quad[np.argmin(s)], quad[np.argmax(d)],
                     quad[np.argmax(s)], quad[np.argmin(d)]], dtype=np.float32)


def pnp_pose_body(corners, w, h):
    """solvePnP the gate -> (forward, right, down) of the gate centre in the BODY frame, or None."""
    half = GATE_SIZE_M / 2.0
    obj = np.array([[-half, -half, 0], [half, -half, 0],
                    [half, half, 0], [-half, half, 0]], dtype=np.float32)  # x right, y down
    K = np.array([[FOCAL_PX, 0, w / 2.0], [0, FOCAL_PX, h / 2.0], [0, 0, 1]], dtype=np.float32)
    # ITERATIVE (not IPPE_SQUARE - that gave ~0.4x the true distance on these gates)
    ok, rvec, tvec = cv2.solvePnP(obj, corners, K, None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    t = np.asarray(tvec).reshape(3)
    x, y, z = float(t[0]), float(t[1]), float(t[2])   # OpenCV cam: x right, y down, z fwd
    fwd, right, down = z, x, y                         # camera -> body-aligned
    c, s = math.cos(CAM_UPTILT_RAD), math.sin(CAM_UPTILT_RAD)
    return (fwd * c + down * s, right, -fwd * s + down * c)   # de-tilt by the camera up-angle


def camera_gate_pose(img):
    """Body-frame (forward, right, down) of the nearest gate from the camera alone, or None."""
    corners = gate_corners(img)
    if corners is None:
        return None
    h, w = img.shape[:2]
    return pnp_pose_body(corners, w, h)


# Camera geometry for projecting a known BODY-frame point back into the image (the inverse of the
# bearing the detector reports). Used only by offline data collection to label each detection
# against ground truth: it tells us WHERE a true gate should appear in frame.
from common.camera import HALF_TAN_X as _HTX, HALF_TAN_Y as _HTY
_C_TILT, _S_TILT = math.cos(CAM_UPTILT_RAD), math.sin(CAM_UPTILT_RAD)


def project_body_to_offset(fwd, right, down):
    """Project a body-frame point (forward, right, down) to normalised image offsets (ox, oy) in
    [-1,1], the exact quantity the detector reports. Inverts the de-tilt + pinhole in pnp_pose_body.
    Returns None if the point is behind the camera. |ox|,|oy| <= 1 means it lands inside the frame."""
    # body -> camera-aligned (rotate DOWN by the up-tilt): inverse of the de-tilt rotation
    fwd_cam = _C_TILT * fwd - _S_TILT * down
    down_cam = _S_TILT * fwd + _C_TILT * down
    right_cam = right
    if fwd_cam <= 1e-6:
        return None
    return (right_cam / fwd_cam) / _HTX, (down_cam / fwd_cam) / _HTY


def shadow_compare(data, img):
    """Run camera PnP and compare it to LIVE ground truth (same frame, no replay mismatch).

    Returns a dict {gate_id, est, gt, pos_err, dist_err} or None. Ground truth is the nearest
    gate in front of the camera, matched the same way the detector locks on."""
    est = camera_gate_pose(img)
    if est is None:
        return None
    pose = get_drone_pose(data)
    gates = data.get("gates")
    if pose is None or not gates:
        return None
    gts = [relative_gate(pose[0], pose[1], g) for g in gates]
    gts = [g for g in gts if g["forward"] > -3.0 and abs(g["azimuth_deg"]) < 55]
    if not gts:
        return None
    gt = min(gts, key=lambda g: g["distance"])
    gt_v = (gt["forward"], gt["right"], gt["down"])
    pos_err = math.dist(est, gt_v)
    dist_err = abs(math.hypot(*est) - gt["distance"])
    return {"gate_id": gt["gate_id"], "est": est, "gt": gt_v,
            "pos_err": pos_err, "dist_err": dist_err}
