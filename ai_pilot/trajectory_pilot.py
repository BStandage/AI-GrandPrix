"""
Trajectory pilot: follow a pre-planned racing line through the gates (the default pilot).

It builds a smooth line through the gates once (trajectory.Trajectory) and tracks it with a
WORLD-FRAME acceleration controller: command a world horizontal acceleration that pulls the
drone onto the line and matches the line's velocity, then realise it as a thrust-vector tilt
(roll/pitch). Because the acceleration is commanded in the world frame, yaw is decoupled from
translation, so the drone does not crab and the nose can point wherever we like.

Key fact this depends on: the sim's odometry vx/vy/vz are BODY-frame (forward, right, down),
not world. We rotate them to world before using them (verified: rotated velocity matches
finite-differenced world position to 0.27 m/s; treating them as world is 16 m/s off).
"""

import math

import keyboard

from dynamics import (CONTROL_HZ, G_ACC, KP_ATT, KP_THRUST_V, MAX_CLIMB, MAX_DESCENT, MAX_RATE,
                      MAX_THRUST, MIN_THRUST, PITCH_SIGN, ROLL_SIGN, YAW_SIGN, clamp,
                      lean_for_speed, send_rate_attitude, thrust_for_climb)
from gate_geometry import quat_to_rotmat, relative_gate
from race import load_cached_gates, seconds_to_go, should_fly
from trajectory import Trajectory

TRAJ_LOOKAHEAD = 5.5     # m ahead for the (cosmetic) yaw carrot
TRAJ_APEX_MAX = 0.0      # apex cut toward the inside of each corner; 0 = thread gate centres.
                         # Cutting buys ~no speed on these gentle corners but costs post clearance.

# Yaw only points the nose along the line (cosmetic); tracking no longer depends on it.
TRAJ_KP_YAW = 0.7        # yaw-rate per rad of bearing. The sim amplifies yaw ~3.3x, so keep this low.
TRAJ_MAX_YAW_RATE = 2.0  # rad/s command cap (~6.6 rad/s actual after the 3.3x amplification)

# Roll/pitch tilt limits (the accel->tilt result is clamped to these).
TRAJ_BRAKE_PITCH = -0.20  # most we'll pitch back (~11 deg)
TRAJ_ACCEL_PITCH = 0.50   # most forward lean (~29 deg, ~9 m/s)
TRAJ_MAX_STRAFE = 0.50    # max bank for lateral correction (~29 deg)
TRAJ_V_MAX = 7.0          # m/s target. Higher overshoots the sharp gate-3 V into the inner post.

# Setpoints are slew-rate-limited so they ramp instead of stepping (a step to full lean from
# standstill overshot and diverged). Slewing is a stabiliser; it does not cap top speed.
TRAJ_PITCH_SLEW = 1.5     # rad/s
TRAJ_ROLL_SLEW = 4.0      # rad/s

# Vertical: strong altitude hold to the line + a fraction of the slope as feedforward.
TRAJ_KP_H = 3.0           # climb (m/s) per metre below the line
TRAJ_VFF = 0.8            # fraction of the line's descent rate fed forward
TRAJ_VERT_BIAS = 0.3      # m above gate centres. Bracketed: 0.4 clipped tops, 0.1 clipped bottoms.
TRAJ_KI_THR = 0.0         # altitude integral, disabled (it wound up and fought the descents)
TRAJ_THR_I_CLAMP = 0.15
TRAJ_KSPEED_THR = 0.0     # extra thrust per m/s, disabled (it over-lifted with correct climb_up)

# Horizontal world-frame tracking gains. With KP_POS this is ~critically damped (omega~1.2,
# zeta~1.0). Raise KD if it oscillates; raise KP if it tracks corners too loosely.
TRAJ_KP_POS = 1.5         # m/s^2 of commanded accel per m of cross-track error
TRAJ_KD_VEL = 2.5         # m/s^2 per m/s of velocity error (this term cancels crab)


def update_trajectory_control(mavlink_conn, system_boot_ms, data):
    if keyboard.is_pressed('esc'):
        data["running"] = False

    load_cached_gates(data)
    # If a live track arrives later it replaces the cache, so rebuild the line from it.
    if data.get("_gates_from_cache") and data.get("gates") is not data.get("_cached_gates_obj"):
        data["_gates_from_cache"] = False
        data["_traj"] = None
        print("[traj] live track received - rebuilding racing line.", flush=True)

    if not should_fly(data):
        data["vis_lean"] = 0.0
        data["oracle_thrust"] = 0.0
        data["traj_pitch"] = 0.0      # reset slew + integral state so a (re)start launches clean
        data["traj_roll"] = 0.0
        data["traj_thr_i"] = 0.0
        t_go = seconds_to_go(data)
        data["traj_regime"] = (f"WAIT-GO {t_go:.1f}s" if (t_go and t_go > 0 and data.get("gates")) else "IDLE")
        send_rate_attitude(mavlink_conn, system_boot_ms, 0.0, 0.0, 0.0, 0.0)
        return

    # Build the racing line from the current gates, and rebuild whenever the gate list changes.
    # The odometry frame resets at race start, so a track received before "ready" is in the
    # pre-reset frame; rebuilding on the new gate object means we always fly this run's frame.
    if data.get("_traj") is None or data.get("_traj_gates") is not data.get("gates"):
        data["_traj"] = Trajectory(data["gates"], v_max=TRAJ_V_MAX, apex_max=TRAJ_APEX_MAX)
        data["_traj_gates"] = data["gates"]
        print(f"[traj] racing line built: {data['_traj'].length:.0f} m through "
              f"{len(data['gates'])} gates", flush=True)
    traj = data["_traj"]

    odo = data.get("odometry") or {}
    att = data.get("attitude", {})
    pos = (odo.get("x", 0.0), odo.get("y", 0.0), odo.get("z", 0.0))
    quat = (odo.get("qw", 1.0), odo.get("qx", 0.0), odo.get("qy", 0.0), odo.get("qz", 0.0))

    # carrot (lookahead point, for yaw) + nearest line point + tangent + target speed + the
    # line's own turning acceleration (centripetal feedforward)
    carrot, prog, xtrack, tangent, v_target, near_z, near_slope_down, near_pt, near_tan, accel_ff = \
        traj.carrot(pos, TRAJ_LOOKAHEAD)

    roll = att.get("roll", 0.0)
    pitch = att.get("pitch", 0.0)
    drone_z = odo.get("z", 0.0)

    # World-frame velocity. Odometry vx/vy/vz are body-frame, so rotate them through the
    # attitude quat. This also gives the TRUE climb rate (body vz mixes in forward speed at pitch).
    R = quat_to_rotmat(quat)
    vb = (odo.get("vx", 0.0), odo.get("vy", 0.0), odo.get("vz", 0.0))
    vw = (R[0][0] * vb[0] + R[0][1] * vb[1] + R[0][2] * vb[2],
          R[1][0] * vb[0] + R[1][1] * vb[1] + R[1][2] * vb[2],
          R[2][0] * vb[0] + R[2][1] * vb[1] + R[2][2] * vb[2])
    climb_up = -vw[2]                        # true world vertical speed (up+)
    v_cur = math.hypot(vw[0], vw[1])         # true world horizontal speed

    # Vertical: hold the line's altitude at the nearest point, aiming TRAJ_VERT_BIAS above
    # centre. alt_err > 0 means we're below the target. Slope feedforward + strong P (+ a
    # clamped integral that is currently disabled).
    alt_err = drone_z - (near_z - TRAJ_VERT_BIAS)
    thr_i = clamp(data.get("traj_thr_i", 0.0) + TRAJ_KI_THR * alt_err * (1.0 / CONTROL_HZ),
                  -TRAJ_THR_I_CLAMP, TRAJ_THR_I_CLAMP)
    data["traj_thr_i"] = thr_i
    climb_ff = -TRAJ_VFF * near_slope_down * v_cur
    desired_climb = clamp(climb_ff + TRAJ_KP_H * alt_err, -MAX_DESCENT, MAX_CLIMB)

    # Horizontal: command a world acceleration that pulls onto the line and matches its velocity.
    tan_h = math.hypot(near_tan[0], near_tan[1]) or 1.0
    tdir = (near_tan[0] / tan_h, near_tan[1] / tan_h)   # line direction AT the drone (nearest point)
    v_ref = (tdir[0] * v_target, tdir[1] * v_target)    # desired world velocity along the line

    # Position error to the line, cross-track (perpendicular) component only. The along-track
    # part snaps as the discrete nearest-point index steps and would kick the pitch; forward
    # motion is the velocity term's job.
    perr_raw = (near_pt[0] - pos[0], near_pt[1] - pos[1])
    along = perr_raw[0] * tdir[0] + perr_raw[1] * tdir[1]
    perr = (perr_raw[0] - along * tdir[0], perr_raw[1] - along * tdir[1])
    verr = (v_ref[0] - vw[0], v_ref[1] - vw[1])         # velocity error -> cancels crab

    # Drag feedforward: the measured lean that holds the current speed, along the heading of travel.
    vhat = (vw[0] / v_cur, vw[1] / v_cur) if v_cur > 0.5 else tdir
    a_drag = G_ACC * math.tan(lean_for_speed(v_cur))

    # Total world acceleration = position + velocity feedback + drag FF + the line's centripetal FF.
    ax = TRAJ_KP_POS * perr[0] + TRAJ_KD_VEL * verr[0] + a_drag * vhat[0] + accel_ff[0]
    ay = TRAJ_KP_POS * perr[1] + TRAJ_KD_VEL * verr[1] + a_drag * vhat[1] + accel_ff[1]

    # Project the world acceleration onto the heading (yaw-only) frame -> roll/pitch setpoints.
    psi = math.atan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]),
                     1 - 2 * (quat[2] ** 2 + quat[3] ** 2))
    cpsi, spsi = math.cos(psi), math.sin(psi)
    a_fwd = ax * cpsi + ay * spsi
    a_right = -ax * spsi + ay * cpsi
    raw_pitch = clamp(math.atan2(a_fwd, G_ACC), TRAJ_BRAKE_PITCH, TRAJ_ACCEL_PITCH)
    raw_roll = clamp(math.atan2(a_right, G_ACC), -TRAJ_MAX_STRAFE, TRAJ_MAX_STRAFE)

    # Yaw: point the nose along the line (toward the carrot). Cosmetic now; tracking doesn't need it.
    rel = relative_gate(pos, quat, {"gate_id": -1, "position_ned": [float(c) for c in carrot]})
    bearing = math.atan2(rel["right"], max(rel["forward"], 1.0))
    yaw_rate = clamp(YAW_SIGN * TRAJ_KP_YAW * bearing, -TRAJ_MAX_YAW_RATE, TRAJ_MAX_YAW_RATE)

    # Slew-limit the roll/pitch setpoints so they ramp instead of stepping.
    des_roll = data.get("traj_roll", 0.0)
    des_roll += clamp(raw_roll - des_roll, -TRAJ_ROLL_SLEW / CONTROL_HZ, TRAJ_ROLL_SLEW / CONTROL_HZ)
    des_pitch = data.get("traj_pitch", 0.0)
    des_pitch += clamp(raw_pitch - des_pitch, -TRAJ_PITCH_SLEW / CONTROL_HZ, TRAJ_PITCH_SLEW / CONTROL_HZ)
    data["traj_roll"], data["traj_pitch"] = des_roll, des_pitch

    # Thrust: measured climb curve, tilt-compensated by MEASURED tilt, floored so a steep
    # attitude can't spike it.
    cos_tilt = max(math.cos(pitch) * math.cos(roll), 0.6)
    thrust = clamp(thrust_for_climb(desired_climb) / cos_tilt + KP_THRUST_V * (desired_climb - climb_up)
                   + thr_i + TRAJ_KSPEED_THR * v_cur,
                   MIN_THRUST, MAX_THRUST)
    # Attitude: plain P loop on measured roll/pitch.
    roll_rate = clamp(ROLL_SIGN * KP_ATT * (des_roll - roll), -MAX_RATE, MAX_RATE)
    pitch_rate = clamp(PITCH_SIGN * KP_ATT * (des_pitch - pitch), -MAX_RATE, MAX_RATE)

    data["traj_regime"] = f"RIP {prog * 100:.0f}%"
    data["traj_xtrack"] = xtrack
    data["traj_vtgt"] = v_target
    data["traj_vcur"] = v_cur
    data["traj_yawrate"] = yaw_rate
    data["pursuit_desired_climb"] = desired_climb
    data["oracle_thrust"] = thrust
    send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust)
