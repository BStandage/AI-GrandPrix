"""
Course / hardware spec constants — VADR-TS-003 (Issue 00.03, 2026-06-24).

The SINGLE source for the numbers PUBLISHED in the Technical Specification: gate and
chassis geometry, camera intrinsics, coordinate frames, comms timing, and run limits.

COMPLIANCE — read before adding anything here
---------------------------------------------
This file contains ONLY values the spec actually publishes. There are deliberately
**no gate positions** in it, and none must ever be added:

  * The spec publishes gate DIMENSIONS (sec 3.7) but NO gate positions anywhere.
  * Phase 2 (sec 9.3) BLOCKS GATE_INFO, LOCAL_POSITION_NED, ODOMETRY and ATTITUDE.
    Any world-frame gate map therefore comes from a now-blocked feed. Flying to a
    hardcoded map is "manipulation of simulator constraints" (sec 9.2). The VQ2 pilot navigates by vision + IMU only.

So: geometry and intrinsics are fair game (they're in the PDF). Positions are not.
"""

import math

# === Drone chassis — spec sec 3.6 ===========================================
CHASSIS_WIDTH_M = 0.280
CHASSIS_LENGTH_M = 0.280
CHASSIS_HEIGHT_M = 0.160

# === Gate geometry — spec sec 3.7 ===========================================
GATE_OUTER_M = 2.70     # outer frame, square (width = height)
GATE_INNER_M = 1.50     # inner opening, square (width = height) — the flyable hole
GATE_DEPTH_M = 0.26     # frame thickness front-to-back

# Derived flyable tolerance: the drone CENTRE must stay within half the inner opening
# minus half the chassis for the whole airframe to clear the hole.
#   (1.50 - 0.280) / 2 = 0.61 m of lateral/vertical centre error, max.
# (This is the geometric clearance; aero margin / prop wash makes the practical
#  budget tighter — treat 0.61 m as the absolute wall, not the target.)
GATE_HALF_CLEARANCE_M = (GATE_INNER_M - CHASSIS_WIDTH_M) / 2.0   # 0.61

# === Camera intrinsics — spec sec 3.8 =======================================
# Authoritative camera geometry lives in common/camera.py (imported by perception).
# Mirrored here as the spec-of-record; values MUST match camera.py.
CAM_WIDTH_PX = 640
CAM_HEIGHT_PX = 360
CAM_CX = 320.0
CAM_CY = 180.0
CAM_FX = 320.0
CAM_FY = 320.0
CAM_VFOV_DEG = 90.0
CAM_UPTILT_DEG = 20.0                      # optical axis tilted UP 20 deg above body-forward
CAM_UPTILT_RAD = math.radians(CAM_UPTILT_DEG)

# === Coordinate frames — spec sec 3.8 =======================================
# MAVLink2 convention is NED.
#   LOCAL_NED : origin = fixed ground point where the drone armed.
#   BODY_NED  : origin = vehicle. X forward, Y right, Z down.
# Body->Camera: same origin, camera pitched up 20 deg (CAM_UPTILT_RAD). Body->IMU: identity.

# === Comms / timing — spec sec 3.2, 4.2-4.4, 4.6 ============================
PHYSICS_HZ = 120            # simulator physics update rate (sec 3.2 / 4.4)
COMMAND_HZ_MAX = 100        # client->sim command rate must stay BELOW this (sec 4.4)
HEARTBEAT_HZ_MIN = 2        # minimum heartbeat the client must maintain (sec 4.4)
VISION_HZ = 30             # camera stream rate (sec 4.6)
VISION_UDP_PORT = 5600      # default vision stream port (sec 4.6)

# === Allowed / blocked MAVLink messages — spec sec 4.3 + 9.3 ================
# Sim->Client messages the VQ2 pilot MAY consume.
ALLOWED_RX_MESSAGES = (
    "HEARTBEAT",
    "HIGHRES_IMU",
    "TIMESYNC",
)
# Client->Sim control messages.
ALLOWED_TX_MESSAGES = (
    "SET_POSITION_TARGET_LOCAL_NED",
    "SET_ATTITUDE_TARGET",
)
# Blocked in Phase 2 qualification (sec 9.3) — MUST NOT be consumed by a VQ2 run.
BLOCKED_RX_MESSAGES = (
    "ATTITUDE",
    "LOCAL_POSITION_NED",
    "ODOMETRY",
    "GATE_INFO",
)

# === Run limits — spec sec 8.3 ==============================================
MAX_RUN_DURATION_S = 8 * 60   # 8 minutes
