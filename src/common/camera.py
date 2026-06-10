"""
Ground-truth camera model (spec VADR-TS-002, sec 3.7-3.8) - the SINGLE source for camera geometry.

Perception (gate_detector, vision_pose) and the vision pilot all import from here, so the
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
