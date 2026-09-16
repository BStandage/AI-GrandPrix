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

import os

# Camera geometry. Defaults = the spec camera (640x360, fx 320, 20 deg up).
# Override to study a different mount / lens: AIGP_CAM_TILT_DEG=35
# AIGP_CAM_HFOV_DEG=120 (vertical FOV follows the 16:9 sensor).
CAM_TILT_RAD = math.radians(float(os.environ.get("AIGP_CAM_TILT_DEG", "20")))
_HFOV = math.radians(float(os.environ.get("AIGP_CAM_HFOV_DEG", "90")))
HALF_TAN_X = math.tan(_HFOV / 2.0)              # 1.0 for 90 deg
HALF_TAN_Y = HALF_TAN_X * 9.0 / 16.0            # 0.5625 for the spec camera
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


# --- what a real detector returns: ONE unlabeled detection, imperfect --------
# The consumers must not know which gate it is (the estimator associates it
# from its own position, exactly as on the Orin). AIGP_CAM_NOISE=0 turns the
# imperfections off; the defaults are the stress case the stack is validated
# under, not a measurement of the real detector.
NOISE_ON = os.environ.get("AIGP_CAM_NOISE", "1") != "0"
NOISE = dict(dropout=0.15,        # fraction of frames with no detection though a gate is in view
             sigma_offset=0.01,   # image offset noise, fraction of the half frame (~0.5 deg)
             sigma_range=0.10,    # range noise, fraction of the range
             false_pos=0.02,      # fraction of frames returning a detection of nothing
             latency_s=1.0 / 30)  # the frame is one period old when it is consumed
_rng = np.random.default_rng(int(os.environ.get("AIGP_SEED", "0")))


def detect_any(world_pos, landmarks, t) -> Detection | None:
    """The biggest ring in view of the camera, unlabeled, with the noise
    model applied. `landmarks` are (x, y, z, heading) tuples of every gate."""
    if NOISE_ON and _rng.random() < NOISE["false_pos"]:
        return Detection(offset_x=float(_rng.uniform(-1, 1)), offset_y=float(_rng.uniform(-1, 1)),
                         area_frac=0.01, t=t, range_m=float(_rng.uniform(2.0, 15.0)))
    best = None
    t_frame = t - (NOISE["latency_s"] if NOISE_ON else 0.0)   # the frame is this old when consumed
    for gx, gy, gz, gh in landmarks:
        d = detect(world_pos, gx, gy, gz, gh, t_frame)
        if d is not None and (best is None or d.area_frac > best.area_frac):
            best = d
    if best is None:
        return None
    if NOISE_ON:
        if _rng.random() < NOISE["dropout"]:
            return None
        best.offset_x = float(np.clip(best.offset_x + _rng.normal(0.0, NOISE["sigma_offset"]), -1.0, 1.0))
        best.offset_y = float(np.clip(best.offset_y + _rng.normal(0.0, NOISE["sigma_offset"]), -1.0, 1.0))
        best.range_m = max(0.3, best.range_m * (1.0 + float(_rng.normal(0.0, NOISE["sigma_range"]))))
    return best


class PoseHistory:
    """Keeps recent poses so a detection can be synthesized from the pose
    of `latency_s` ago, the way a real frame is already old when it is used."""

    def __init__(self, latency_s: float):
        self.latency_s = latency_s if NOISE_ON else 0.0
        self.buf = []

    def push(self, t, world_pos):
        self.buf.append((float(t), np.array(world_pos, dtype=float)))
        while len(self.buf) > 1 and self.buf[1][0] <= t - self.latency_s - 0.05:
            self.buf.pop(0)

    def at_delay(self, t):
        want = t - self.latency_s
        best = self.buf[0][1] if self.buf else None
        for tt, wp in self.buf:
            if tt <= want:
                best = wp
            else:
                break
        return best
