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

from common.dynamics import (CONTROL_HZ, G_ACC, KP_ATT, KP_THRUST_V, MAX_CLIMB, MAX_DESCENT, MAX_RATE,
                      MAX_THRUST, MIN_THRUST, PITCH_SIGN, ROLL_SIGN, YAW_SIGN, clamp,
                      lean_for_speed, send_rate_attitude, thrust_for_climb)
from common.gate_geometry import quat_to_rotmat, relative_gate
from common.race import load_cached_gates, seconds_to_go, should_fly
from common.trajectory import Trajectory

TRAJ_LOOKAHEAD = 5.5     # m ahead for the (cosmetic) yaw carrot
TRAJ_APEX_MAX = 0.0      # apex cut toward the inside of each corner; 0 = thread gate centres.
                         # Cutting buys ~no speed on these gentle corners but costs post clearance.

# Yaw only points the nose along the line (cosmetic); tracking no longer depends on it.
TRAJ_KP_YAW = 0.7        # yaw-rate per rad of bearing. The sim amplifies yaw ~3.3x, so keep this low.
TRAJ_MAX_YAW_RATE = 2.0  # rad/s command cap (~6.6 rad/s actual after the 3.3x amplification)

# Roll/pitch tilt limits (the accel->tilt result is clamped to these).
TRAJ_BRAKE_PITCH = -0.35  # most we'll pitch back (~20 deg) - more braking authority into sharp gates
TRAJ_ACCEL_PITCH = 0.65   # most forward lean (~37 deg). 43 deg over-thrust + ballooned over the
                          # gates; backed off. Past ~45 deg the tilt-comp can't hold altitude.
TRAJ_MAX_STRAFE = 0.90    # max bank for lateral correction (~52 deg). Hard banking; sinks a bit in
                          # the turn (fine on the descending swings, caught by the floor guard near
                          # the ground). (1.10/63deg cut the inside of the early gates.)
TRAJ_V_MAX = 13.0         # m/s target straight speed. (15 cut the inside of the early gates.)
TRAJ_A_LAT = 12.0         # m/s^2 design lateral accel for the corner-speed profile - carry more
                          # speed through corners. (Higher = brakes less for the sharp gate-3 V.)

# Setpoints are slew-rate-limited so they ramp instead of stepping (a step to full lean from
# standstill overshot and diverged). Slewing is a stabiliser; it does not cap top speed.
TRAJ_PITCH_SLEW = 2.5     # rad/s (faster ramp into the lean for quicker accel; was 1.5)
TRAJ_ROLL_SLEW = 4.0      # rad/s

# Vertical: strong altitude hold to the line + a fraction of the slope as feedforward.
TRAJ_KP_H = 3.0           # climb (m/s) per metre below the line. (4.0 + full FF overshot the first
TRAJ_VFF = 0.8            # descent into gate 1's bottom; back to the proven record values.)
TRAJ_VERT_BIAS = 0.3      # m above gate centres. Bracketed: 0.4 clipped tops, 0.1 clipped bottoms.
TRAJ_GROUND_MARGIN = 0.3  # m. Floor guard: don't descend more than this below the LOWEST gate centre
                          # (gates 4/5 rest on the floor). Keyed to the real ground, not spawn. Lower
                          # = keeps the drone higher / more floor margin (it was still bumping at 0.6).
TRAJ_KI_THR = 0.0         # altitude integral, disabled (it wound up and fought the descents)
TRAJ_THR_I_CLAMP = 0.15
TRAJ_KSPEED_THR = 0.0     # extra thrust per m/s, disabled (it over-lifted with correct climb_up)

# Horizontal world-frame tracking gains. With KP_POS this is ~critically damped (omega~1.2,
# zeta~1.0). Raise KD if it oscillates; raise KP if it tracks corners too loosely.
TRAJ_KP_POS = 3.0         # m/s^2 of commanded accel per m of cross-track error (aggressive: pull
                          # hard onto the line so it hits offset apexes instead of cutting inside)
TRAJ_KD_VEL = 3.5         # m/s^2 per m/s of velocity error (raised with KP to stay damped)

# Look-ahead (pure-pursuit) for the cross-track target: aim AHEAD on the line so the tracker banks
# into a bend early instead of reacting after it has cut inside. Distance = TRAJ_LA_TIME * speed,
# clamped. WEIGHT blends nearest-point (0.0, purely reactive) -> look-ahead point (1.0). Keep the
# distance modest: too far points past the apex and CUTS the corner.
TRAJ_LA_TIME = 0.35       # s of look-ahead (distance grows with speed)
TRAJ_LA_MIN = 1.0         # m (floor at low speed)
TRAJ_LA_MAX = 6.0         # m (cap so it never over-reaches past an apex)
TRAJ_LA_WEIGHT = 0.6      # 0 = nearest-point only, 1 = full look-ahead. Lower if it starts cutting.


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
        data["_traj"] = Trajectory(data["gates"], v_max=TRAJ_V_MAX, apex_max=TRAJ_APEX_MAX,
                                   a_lat=TRAJ_A_LAT)
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

    # Floor guard, referenced to the ACTUAL ground (the lowest gate - gates 4/5 rest on the floor),
    # NOT spawn. The aggressive descent overshoots the line near the floor and bounces off it after
    # gate 4. Don't let the drone drop more than TRAJ_GROUND_MARGIN below the lowest gate centre;
    # this only ever acts right above the real ground, so it never touches the descent above.
    drone_alt = -drone_z
    floor_alt = min(-g["position_ned"][2] for g in data["gates"]) - TRAJ_GROUND_MARGIN
    if drone_alt < floor_alt:
        # firm upward push - strong gain + a floor so it arrests a FAST descent before the ground,
        # not the gentle KP_H nudge that let it sink through.
        desired_climb = clamp(max(desired_climb, 1.0 + 5.0 * (floor_alt - drone_alt)),
                              -MAX_DESCENT, MAX_CLIMB)


    # Horizontal: command a world acceleration that pulls onto the line and matches its velocity.
    tan_h = math.hypot(near_tan[0], near_tan[1]) or 1.0
    tdir = (near_tan[0] / tan_h, near_tan[1] / tan_h)   # line direction AT the drone (nearest point)
    v_ref = (tdir[0] * v_target, tdir[1] * v_target)    # desired world velocity along the line

    # Position error to the line, cross-track (perpendicular) component only. The along-track
    # part snaps as the discrete nearest-point index steps and would kick the pitch; forward
    # motion is the velocity term's job.
    #
    # LOOK-AHEAD (pure pursuit): aim the cross-track target not at the nearest point but at a point
    # a short, speed-scaled distance AHEAD on the line, blended by TRAJ_LA_WEIGHT. This makes the
    # tracker anticipate a bend (bank in early) instead of reacting after it has cut inside. Kept
    # short on purpose - too far points past the apex and cuts the corner. v_ref still uses the
    # NEAREST tangent (below), so only the position target looks ahead, not the velocity heading.
    la_dist = clamp(TRAJ_LA_TIME * v_cur, TRAJ_LA_MIN, TRAJ_LA_MAX)
    la_pt = traj.track_point(pos, la_dist)
    tgt = ((1.0 - TRAJ_LA_WEIGHT) * near_pt[0] + TRAJ_LA_WEIGHT * la_pt[0],
           (1.0 - TRAJ_LA_WEIGHT) * near_pt[1] + TRAJ_LA_WEIGHT * la_pt[1])
    perr_raw = (tgt[0] - pos[0], tgt[1] - pos[1])
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
