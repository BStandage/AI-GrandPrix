"""
Ground-truth camera model (spec VADR-TS-002, sec 3.7-3.8) - the SINGLE source for camera geometry.

Perception (detectors, gate_detection, vision_pose) and the vision pilot all import from here, so the
calibration lives in exactly one place. These are ground truth - do NOT tweak them to fix flight.
"""

import math

# --- pinhole intrinsics (no lens distortion) ---
WIDTH = 640
HEIGHT = 360
FX = 320.0
FY = 320.0
CX = WIDTH / 2.0        # 320
CY = HEIGHT / 2.0       # 180

# --- mounting ---
UPTILT_RAD = math.radians(20.0)   # optical axis tilted UP 20 deg above body-forward

# --- derived: a pixel offset normalised to half-width/half-height -> angle = atan(offset*HALF_TAN)
HALF_TAN_X = (WIDTH / 2.0) / FX    # 1.0    -> a full-width offset is 45 deg
HALF_TAN_Y = (HEIGHT / 2.0) / FY   # 0.5625 -> a full-height offset is 29.4 deg

# --- gate geometry (spec sec 3.7) ---
GATE_OUTER_M = 2.70     # outer frame, square
GATE_INNER_M = 1.50     # inner opening, square


def offset_to_body_dir(ox, oy):
    """Normalised image offset -> UNIT direction vector to the gate in the BODY frame (fwd, right, down),
    correcting for the 20 deg camera up-tilt. Rotate this by the drone's attitude to get the WORLD
    direction (which is what the vertical servo must use - see offset_to_bearing's note)."""
    right_cam = ox * HALF_TAN_X
    down_cam = oy * HALF_TAN_Y
    c, s = math.cos(UPTILT_RAD), math.sin(UPTILT_RAD)
    fwd = c * 1.0 + s * down_cam
    down = -s * 1.0 + c * down_cam
    right = right_cam
    n = math.sqrt(fwd * fwd + right * right + down * down) or 1.0
    return (fwd / n, right / n, down / n)


def offset_to_bearing(ox, oy):
    """Normalised image offset (ox, oy in [-1,1]) -> the gate's true BODY-frame bearing (az, el) in
    radians, correcting for the 20 deg camera up-tilt. This is the inverse of vision_pose's
    project_body_to_offset, and the ONLY correct way to read vertical: it folds the tilt into the
    geometry so the pilot never has to guess an offset_y target.

      az  > 0  ->  gate is to the RIGHT of body-forward
      el  > 0  ->  gate is ABOVE body-forward

    Sanity: (0,0) -> az=0, el=+20 deg (image centre sits on the up-tilted optical axis, i.e. 20 deg
    above body-forward); a gate dead-ahead in the body frame (el=0) sits LOW in frame at oy~+0.65.
    """
    # camera-frame ray from the normalised offsets (fwd_cam = 1 by construction)
    right_cam = ox * HALF_TAN_X
    down_cam = oy * HALF_TAN_Y
    # rotate camera -> body: UP by the up-tilt (inverse of the body->cam down-rotation)
    c, s = math.cos(UPTILT_RAD), math.sin(UPTILT_RAD)
    fwd_b = c * 1.0 + s * down_cam
    down_b = -s * 1.0 + c * down_cam
    right_b = right_cam
    az = math.atan2(right_b, fwd_b)
    el = math.atan2(-down_b, math.hypot(fwd_b, right_b))
    return az, el
