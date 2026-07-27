"""
Phase-1 standalone racing-line follower.

Verbatim copy of common.line_follower (build_line + follow_line) with the conservative TRAJ_* gains
swapped for the aggressive P1_* config in pilots.phase1_pilot.config. Kept SEPARATE from
common.line_follower on purpose: the oracle and vision pilots share that file precisely so they fly
identically, and they must NOT inherit these aggressive limits. This copy is fully isolated.

See common/line_follower.py (and oracle_pilot history) for why each gain is what it is.
"""

import math

from common.dynamics import (CONTROL_HZ, G_ACC, KP_ATT, KP_THRUST_V, MAX_CLIMB, MAX_DESCENT, MAX_RATE,
                      MAX_THRUST, MIN_THRUST, PITCH_SIGN, ROLL_SIGN, YAW_SIGN, clamp,
                      lean_for_speed, thrust_for_climb)
from common.gate_geometry import quat_to_rotmat, relative_gate
from common.trajectory import Trajectory

from pilots.phase1_pilot.config import *

# Constants with no P1_* override in config: unchanged from the oracle (common.line_follower).
P1_LOOKAHEAD = 5.5      # m ahead for the (cosmetic) yaw carrot
P1_GROUND_MARGIN = 0.3  # m floor guard below the lowest gate centre
P1_KI_THR = 0.0         # altitude integral (disabled)
P1_THR_I_CLAMP = 0.15
P1_KSPEED_THR = 0.0     # extra thrust per m/s (disabled)


def build_line(gates, start=(0.0, 0.0, 0.0), v_max=P1_V_MAX):
    """Build the racing line Trajectory through `gates` (each a dict with position_ned)."""
    return Trajectory(gates, start=start, v_max=v_max, apex_max=P1_APEX_MAX, a_lat=P1_A_LAT)


def follow_line(traj, pos, quat, vb, att, state, floor_alt=None, use_yaw=True, climb_override=None,
                roll_override=None, dt=None, yaw_override=None):
    """Track `traj` from drone pose `pos` (x,y,z), orientation `quat` (w,x,y,z), body velocity
    `vb` (vx,vy,vz) and MEASURED attitude `att` (roll, pitch) for the inner rate loop. `state` is a
    mutable dict carrying the slew/integral state across ticks ("roll","pitch","thr_i"). `floor_alt`
    (the lowest safe altitude, +up) enables the floor guard.

    Returns (roll_rate, pitch_rate, yaw_rate, thrust, telem)."""
    carrot, prog, xtrack, tangent, v_target, near_z, near_slope_down, near_pt, near_tan, accel_ff = \
        traj.carrot(pos, P1_LOOKAHEAD)

    R = quat_to_rotmat(quat)
    vw = (R[0][0] * vb[0] + R[0][1] * vb[1] + R[0][2] * vb[2],
          R[1][0] * vb[0] + R[1][1] * vb[1] + R[1][2] * vb[2],
          R[2][0] * vb[0] + R[2][1] * vb[1] + R[2][2] * vb[2])
    climb_up = -vw[2]
    v_cur = math.hypot(vw[0], vw[1])
    drone_z = pos[2]

    # Vertical: hold the line's altitude at the nearest point, aiming P1_VERT_BIAS above centre.
    alt_err = drone_z - (near_z - P1_VERT_BIAS)
    # REAL elapsed time per step, not a fixed 1/CONTROL_HZ tick: the loop runs at a jittery 112-137 fps,
    # so slewing/integrating per-tick made the accumulated state (and thus the run) differ every time.
    # Caller passes the true delta; defaults to the nominal tick so the oracle is unchanged.
    step = dt if dt is not None else 1.0 / CONTROL_HZ
    thr_i = clamp(state.get("thr_i", 0.0) + P1_KI_THR * alt_err * step,
                  -P1_THR_I_CLAMP, P1_THR_I_CLAMP)
    state["thr_i"] = thr_i
    climb_ff = -P1_VFF * near_slope_down * v_cur
    desired_climb = clamp(climb_ff + P1_KP_H * alt_err, -MAX_DESCENT, MAX_CLIMB)

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

    la_dist = clamp(P1_LA_TIME * v_cur, P1_LA_MIN, P1_LA_MAX)
    la_pt = traj.track_point(pos, la_dist)
    tgt = ((1.0 - P1_LA_WEIGHT) * near_pt[0] + P1_LA_WEIGHT * la_pt[0],
           (1.0 - P1_LA_WEIGHT) * near_pt[1] + P1_LA_WEIGHT * la_pt[1])
    perr_raw = (tgt[0] - pos[0], tgt[1] - pos[1])
    along = perr_raw[0] * tdir[0] + perr_raw[1] * tdir[1]
    perr = (perr_raw[0] - along * tdir[0], perr_raw[1] - along * tdir[1])
    verr = (v_ref[0] - vw[0], v_ref[1] - vw[1])

    vhat = (vw[0] / v_cur, vw[1] / v_cur) if v_cur > 0.5 else tdir
    a_drag = G_ACC * math.tan(lean_for_speed(v_cur))

    ax = P1_KP_POS * perr[0] + P1_KD_VEL * verr[0] + a_drag * vhat[0] + accel_ff[0]
    ay = P1_KP_POS * perr[1] + P1_KD_VEL * verr[1] + a_drag * vhat[1] + accel_ff[1]

    psi = math.atan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]),
                     1 - 2 * (quat[2] ** 2 + quat[3] ** 2))
    cpsi, spsi = math.cos(psi), math.sin(psi)
    a_fwd = ax * cpsi + ay * spsi
    a_right = -ax * spsi + ay * cpsi
    raw_pitch = clamp(math.atan2(a_fwd, G_ACC), P1_BRAKE_PITCH, P1_ACCEL_PITCH)
    raw_roll = clamp(math.atan2(a_right, G_ACC), -P1_MAX_STRAFE, P1_MAX_STRAFE)

    # Lateral override: the vision pilot supplies a strafe-roll setpoint from a range-independent
    # azimuth servo (the gate's bearing), instead of the line's cross-track which inherits the bad
    # gate range. Slew + the rate loop below still apply.
    if roll_override is not None:
        raw_roll = clamp(roll_override, -P1_MAX_STRAFE, P1_MAX_STRAFE)

    # Yaw points the nose along the line. The oracle uses it (cosmetic - its tracking is yaw-decoupled).
    # The vision pilot normally runs yaw OFF (use_yaw=False): the camera is the nose, and yawing swings it
    # off the gate it's tracking. Translation tracking is yaw-decoupled, so strafing still threads gates
    # with the camera fixed. BUT it may pass a bounded yaw_override for a TRANSITION yaw - swinging the nose
    # toward the next gate only AFTER the current one is passed - to bring an off-axis gate on-axis. Same
    # rate path / clamp envelope, so it's as control-stable as the oracle's yaw.
    if yaw_override is not None:
        yaw_rate = clamp(yaw_override, -MAX_RATE, MAX_RATE)
    elif use_yaw:
        rel = relative_gate(pos, quat, {"gate_id": -1, "position_ned": [float(c) for c in carrot]})
        bearing = math.atan2(rel["right"], max(rel["forward"], 1.0))
        yaw_rate = clamp(YAW_SIGN * P1_YAW_KP * bearing, -P1_MAX_YAW_RATE, P1_MAX_YAW_RATE)
    else:
        yaw_rate = 0.0

    des_roll = state.get("roll", 0.0)
    des_roll += clamp(raw_roll - des_roll, -P1_ROLL_SLEW * step, P1_ROLL_SLEW * step)
    des_pitch = state.get("pitch", 0.0)
    des_pitch += clamp(raw_pitch - des_pitch, -P1_PITCH_SLEW * step, P1_PITCH_SLEW * step)
    state["roll"], state["pitch"] = des_roll, des_pitch

    roll, pitch = att[0], att[1]            # measured attitude for the inner rate loop

    cos_tilt = max(math.cos(pitch) * math.cos(roll), 0.6)
    thrust = clamp(thrust_for_climb(desired_climb) / cos_tilt + KP_THRUST_V * (desired_climb - climb_up)
                   + thr_i + P1_KSPEED_THR * v_cur, MIN_THRUST, MAX_THRUST)
    roll_rate = clamp(ROLL_SIGN * KP_ATT * (des_roll - roll), -MAX_RATE, MAX_RATE)
    pitch_rate = clamp(PITCH_SIGN * KP_ATT * (des_pitch - pitch), -MAX_RATE, MAX_RATE)

    telem = {"prog": prog, "xtrack": xtrack, "v_target": v_target, "v_cur": v_cur,
             "yaw_rate": yaw_rate, "desired_climb": desired_climb, "thrust": thrust}
    return roll_rate, pitch_rate, yaw_rate, thrust, telem
