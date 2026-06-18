"""
Shared racing-line follower - the oracle's proven, measured-dynamics controller, factored out so
BOTH pilots use the identical control. The vision pilot does NOT invent its own gains; it estimates
a line and hands it to this follower exactly like the oracle does.

follow_line() takes a Trajectory + the drone pose/velocity (in WHATEVER frame the line is in - true
NED for the oracle, the dead-reckoned LOCAL frame for the vision pilot) and returns the body-rate +
thrust setpoints. All the gains below are the oracle's: tune them here and both pilots move together.

Lifted verbatim from oracle_pilot.update_trajectory_control; see that history for why each gain is
what it is (world-frame accel controller -> thrust-vector tilt, yaw decoupled, measured curves).
"""

import math

from common.dynamics import (CONTROL_HZ, G_ACC, KP_ATT, KP_THRUST_V, MAX_CLIMB, MAX_DESCENT, MAX_RATE,
                      MAX_THRUST, MIN_THRUST, PITCH_SIGN, ROLL_SIGN, YAW_SIGN, clamp,
                      lean_for_speed, thrust_for_climb)
from common.gate_geometry import quat_to_rotmat, relative_gate
from common.trajectory import Trajectory

# --- racing-line build ---------------------------------------------------------------------
TRAJ_V_MAX = 13.0         # m/s target straight speed
TRAJ_APEX_MAX = 0.0       # apex cut toward the inside of each corner; 0 = thread gate centres
TRAJ_A_LAT = 12.0         # m/s^2 design lateral accel for the corner-speed profile
TRAJ_LOOKAHEAD = 5.5      # m ahead for the (cosmetic) yaw carrot

# --- follower gains ------------------------------------------------------------------------
TRAJ_KP_YAW = 0.7         # yaw-rate per rad of bearing (sim amplifies yaw ~3.3x, keep low)
TRAJ_MAX_YAW_RATE = 2.0
TRAJ_BRAKE_PITCH = -0.35  # most we'll pitch back (~20 deg)
TRAJ_ACCEL_PITCH = 0.65   # most forward lean (~37 deg)
TRAJ_MAX_STRAFE = 0.90    # max bank for lateral correction (~52 deg)
TRAJ_PITCH_SLEW = 2.5     # rad/s setpoint slew
TRAJ_ROLL_SLEW = 4.0      # rad/s
TRAJ_KP_H = 3.0           # climb (m/s) per metre below the line
TRAJ_VFF = 0.8            # fraction of the line slope fed forward as climb
TRAJ_VERT_BIAS = 0.3      # m above gate centres
TRAJ_GROUND_MARGIN = 0.3  # m floor guard below the lowest gate centre
TRAJ_KI_THR = 0.0         # altitude integral (disabled)
TRAJ_THR_I_CLAMP = 0.15
TRAJ_KSPEED_THR = 0.0     # extra thrust per m/s (disabled)
TRAJ_KP_POS = 3.0         # m/s^2 of accel per m cross-track error
TRAJ_KD_VEL = 3.5         # m/s^2 per m/s velocity error
TRAJ_LA_TIME = 0.35       # s look-ahead for the cross-track target (distance grows with speed)
TRAJ_LA_MIN = 1.0
TRAJ_LA_MAX = 6.0
TRAJ_LA_WEIGHT = 0.6      # 0 = nearest-point only, 1 = full look-ahead


def build_line(gates, start=(0.0, 0.0, 0.0), v_max=TRAJ_V_MAX):
    """Build the racing line Trajectory through `gates` (each a dict with position_ned)."""
    return Trajectory(gates, start=start, v_max=v_max, apex_max=TRAJ_APEX_MAX, a_lat=TRAJ_A_LAT)


def follow_line(traj, pos, quat, vb, att, state, floor_alt=None, use_yaw=True, climb_override=None,
                roll_override=None):
    """Track `traj` from drone pose `pos` (x,y,z), orientation `quat` (w,x,y,z), body velocity
    `vb` (vx,vy,vz) and MEASURED attitude `att` (roll, pitch) for the inner rate loop. `state` is a
    mutable dict carrying the slew/integral state across ticks ("roll","pitch","thr_i"). `floor_alt`
    (the lowest safe altitude, +up) enables the floor guard.

    Returns (roll_rate, pitch_rate, yaw_rate, thrust, telem)."""
    carrot, prog, xtrack, tangent, v_target, near_z, near_slope_down, near_pt, near_tan, accel_ff = \
        traj.carrot(pos, TRAJ_LOOKAHEAD)

    R = quat_to_rotmat(quat)
    vw = (R[0][0] * vb[0] + R[0][1] * vb[1] + R[0][2] * vb[2],
          R[1][0] * vb[0] + R[1][1] * vb[1] + R[1][2] * vb[2],
          R[2][0] * vb[0] + R[2][1] * vb[1] + R[2][2] * vb[2])
    climb_up = -vw[2]
    v_cur = math.hypot(vw[0], vw[1])
    drone_z = pos[2]

    # Vertical: hold the line's altitude at the nearest point, aiming TRAJ_VERT_BIAS above centre.
    alt_err = drone_z - (near_z - TRAJ_VERT_BIAS)
    thr_i = clamp(state.get("thr_i", 0.0) + TRAJ_KI_THR * alt_err * (1.0 / CONTROL_HZ),
                  -TRAJ_THR_I_CLAMP, TRAJ_THR_I_CLAMP)
    state["thr_i"] = thr_i
    climb_ff = -TRAJ_VFF * near_slope_down * v_cur
    desired_climb = clamp(climb_ff + TRAJ_KP_H * alt_err, -MAX_DESCENT, MAX_CLIMB)

    # Vertical override: the caller (vision pilot) may supply desired_climb from a range-independent
    # source (image-row servo) instead of the line's altitude, which depends on unreliable gate depth.
    if climb_override is not None:
        desired_climb = clamp(climb_override, -MAX_DESCENT, MAX_CLIMB)

    # Floor guard, referenced to the supplied lowest-safe altitude (the real ground under the gates).
    if floor_alt is not None:
        drone_alt = -drone_z
        if drone_alt < floor_alt:
            desired_climb = clamp(max(desired_climb, 1.0 + 5.0 * (floor_alt - drone_alt)),
                                  -MAX_DESCENT, MAX_CLIMB)

    # Horizontal: command a world acceleration that pulls onto the line and matches its velocity.
    tan_h = math.hypot(near_tan[0], near_tan[1]) or 1.0
    tdir = (near_tan[0] / tan_h, near_tan[1] / tan_h)
    v_ref = (tdir[0] * v_target, tdir[1] * v_target)

    la_dist = clamp(TRAJ_LA_TIME * v_cur, TRAJ_LA_MIN, TRAJ_LA_MAX)
    la_pt = traj.track_point(pos, la_dist)
    tgt = ((1.0 - TRAJ_LA_WEIGHT) * near_pt[0] + TRAJ_LA_WEIGHT * la_pt[0],
           (1.0 - TRAJ_LA_WEIGHT) * near_pt[1] + TRAJ_LA_WEIGHT * la_pt[1])
    perr_raw = (tgt[0] - pos[0], tgt[1] - pos[1])
    along = perr_raw[0] * tdir[0] + perr_raw[1] * tdir[1]
    perr = (perr_raw[0] - along * tdir[0], perr_raw[1] - along * tdir[1])
    verr = (v_ref[0] - vw[0], v_ref[1] - vw[1])

    vhat = (vw[0] / v_cur, vw[1] / v_cur) if v_cur > 0.5 else tdir
    a_drag = G_ACC * math.tan(lean_for_speed(v_cur))

    ax = TRAJ_KP_POS * perr[0] + TRAJ_KD_VEL * verr[0] + a_drag * vhat[0] + accel_ff[0]
    ay = TRAJ_KP_POS * perr[1] + TRAJ_KD_VEL * verr[1] + a_drag * vhat[1] + accel_ff[1]

    psi = math.atan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]),
                     1 - 2 * (quat[2] ** 2 + quat[3] ** 2))
    cpsi, spsi = math.cos(psi), math.sin(psi)
    a_fwd = ax * cpsi + ay * spsi
    a_right = -ax * spsi + ay * cpsi
    raw_pitch = clamp(math.atan2(a_fwd, G_ACC), TRAJ_BRAKE_PITCH, TRAJ_ACCEL_PITCH)
    raw_roll = clamp(math.atan2(a_right, G_ACC), -TRAJ_MAX_STRAFE, TRAJ_MAX_STRAFE)

    # Lateral override: the vision pilot supplies a strafe-roll setpoint from a range-independent
    # azimuth servo (the gate's bearing), instead of the line's cross-track which inherits the bad
    # gate range. Slew + the rate loop below still apply.
    if roll_override is not None:
        raw_roll = clamp(roll_override, -TRAJ_MAX_STRAFE, TRAJ_MAX_STRAFE)

    # Yaw points the nose along the line. The oracle uses it (cosmetic - its tracking is yaw-
    # decoupled). The VISION pilot MUST NOT yaw: the camera is the nose, and yawing swings it off
    # the gates, corrupting the estimate and spinning the drone out. With yaw off, the follower still
    # tracks the line by strafing (translation is yaw-decoupled) and the camera stays on the gates.
    if use_yaw:
        rel = relative_gate(pos, quat, {"gate_id": -1, "position_ned": [float(c) for c in carrot]})
        bearing = math.atan2(rel["right"], max(rel["forward"], 1.0))
        yaw_rate = clamp(YAW_SIGN * TRAJ_KP_YAW * bearing, -TRAJ_MAX_YAW_RATE, TRAJ_MAX_YAW_RATE)
    else:
        yaw_rate = 0.0

    des_roll = state.get("roll", 0.0)
    des_roll += clamp(raw_roll - des_roll, -TRAJ_ROLL_SLEW / CONTROL_HZ, TRAJ_ROLL_SLEW / CONTROL_HZ)
    des_pitch = state.get("pitch", 0.0)
    des_pitch += clamp(raw_pitch - des_pitch, -TRAJ_PITCH_SLEW / CONTROL_HZ, TRAJ_PITCH_SLEW / CONTROL_HZ)
    state["roll"], state["pitch"] = des_roll, des_pitch

    roll, pitch = att[0], att[1]            # measured attitude for the inner rate loop

    cos_tilt = max(math.cos(pitch) * math.cos(roll), 0.6)
    thrust = clamp(thrust_for_climb(desired_climb) / cos_tilt + KP_THRUST_V * (desired_climb - climb_up)
                   + thr_i + TRAJ_KSPEED_THR * v_cur, MIN_THRUST, MAX_THRUST)
    roll_rate = clamp(ROLL_SIGN * KP_ATT * (des_roll - roll), -MAX_RATE, MAX_RATE)
    pitch_rate = clamp(PITCH_SIGN * KP_ATT * (des_pitch - pitch), -MAX_RATE, MAX_RATE)

    telem = {"prog": prog, "xtrack": xtrack, "v_target": v_target, "v_cur": v_cur,
             "yaw_rate": yaw_rate, "desired_climb": desired_climb, "thrust": thrust}
    return roll_rate, pitch_rate, yaw_rate, thrust, telem
