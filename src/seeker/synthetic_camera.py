"""
Synthetic gate detector for the sim: the next crossing projected through
the spec camera (640x360, fx = fy = 320, 20 deg up-tilt) from the TRUE
pose. Reports the gate only when it is inside the field of view, roughly
facing the camera and not too close - the same contract a real detector
gives the stack: image offsets, apparent size, and a range estimate from
that size. Used because this elodin build renders no FPV frames.

The consumers never see the true pose; they see this Detection.
"""

from __future__ import annotations

import math

import numpy as np

from raceline.rc_backend import rot_from_quat
from seeker.brain import Detection

CAM_TILT_RAD = math.radians(20.0)
HALF_TAN_X = 1.0          # 320 / 320
HALF_TAN_Y = 0.5625       # 180 / 320
GATE_OUTER_M = 2.7
FACING_LIMIT_RAD = math.radians(60.0)


def detect(world_pos, gate_x, gate_y, gate_z, gate_heading_rad, t) -> Detection | None:
    R = rot_from_quat(world_pos[0:4])                    # body FLU -> world
    p = np.asarray(world_pos[4:7], dtype=float)
    d_b = R.T @ np.array([gate_x - p[0], gate_y - p[1], gate_z - p[2]])
    dist_h = float(np.hypot(gate_x - p[0], gate_y - p[1]))
    if dist_h < 0.4:
        return None
    fwd = d_b[0] * math.cos(CAM_TILT_RAD) + d_b[2] * math.sin(CAM_TILT_RAD)
    up = -d_b[0] * math.sin(CAM_TILT_RAD) + d_b[2] * math.cos(CAM_TILT_RAD)
    left = d_b[1]
    if fwd <= 0.1:
        return None
    x_img = -left / fwd                                   # +right, tan units
    y_img = -up / fwd                                     # +down
    if abs(x_img) > HALF_TAN_X or abs(y_img) > HALF_TAN_Y:
        return None
    yaw = math.atan2(R[1, 0], R[0, 0])
    facing = abs(((gate_heading_rad - yaw + math.pi) % (2 * math.pi)) - math.pi)
    # a ring is a ring from the front or the back; only near edge-on views
    # (60..120 deg off the normal) give the detector nothing to work with
    if FACING_LIMIT_RAD < facing < math.pi - FACING_LIMIT_RAD:
        return None
    app = GATE_OUTER_M / max(dist_h, 0.4)
    area_frac = min(1.0, (app / 2.0) ** 2 * 0.35)
    return Detection(offset_x=x_img / HALF_TAN_X, offset_y=y_img / HALF_TAN_Y,
                     area_frac=area_frac, t=t, range_m=dist_h)


def range_from_area(area_frac: float) -> float:
    """Inverse of the size model above (for detections without a range)."""
    a = max(area_frac, 1e-4)
    return GATE_OUTER_M / (2.0 * math.sqrt(a / 0.35))


def direction_body(det: Detection) -> np.ndarray:
    """Unit vector in body FLU toward the detection (inverts the projection)."""
    x_img = det.offset_x * HALF_TAN_X
    y_img = det.offset_y * HALF_TAN_Y
    cam_fwd = np.array([math.cos(CAM_TILT_RAD), 0.0, math.sin(CAM_TILT_RAD)])
    cam_left = np.array([0.0, 1.0, 0.0])
    cam_up = np.array([-math.sin(CAM_TILT_RAD), 0.0, math.cos(CAM_TILT_RAD)])
    d = cam_fwd - x_img * cam_left - y_img * cam_up
    return d / np.linalg.norm(d)
