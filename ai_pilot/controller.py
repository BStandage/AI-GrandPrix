import csv
import glob
import json
import math
import os
import time

import keyboard
from pymavlink import mavutil

from gate_geometry import active_gate_relative, relative_gate

# --------------------------------------------------------------------------------------
# RESET COMMAND
# --------------------------------------------------------------------------------------
MAVLINK_CMD_SIM_RESET = 31000

CONTROL_HZ = 250

# --------------------------------------------------------------------------------------
# SHARED LOW-LEVEL CONSTANTS  (measured from the physics characterization)
# --------------------------------------------------------------------------------------
# The sim flies on a throttle/thrust model and honours RATE (acro) setpoints: each
# axis is a body angular RATE (rad/s) plus a collective thrust 0..1. We build outer
# loops on top (P on measured attitude -> rate) to fly attitudes and velocities.
#
# These numbers are MEASURED (characterize mode + analyze_performance.py), not
# guessed - change them only against fresh data.
HOVER_THRUST     = 0.299   # collective thrust at which climb rate crosses 0
KP_ATT           = 3.0     # body-rate per rad of attitude error (4.0 caused PIO)
MAX_RATE         = 6.0     # rad/s clamp on commanded body rates (airframe max ~20-28)
MIN_THRUST       = 0.0
MAX_THRUST       = 1.0

# MEASURED steady-state thrust -> climb-rate (up, m/s), from characterize_20260606_223201
# (clean climb-to-altitude run, analyze_performance.py). STRONGLY NONLINEAR and
# violently climb-biased: hover ~0.299, but 0.50 -> +16.9 m/s and full -> +30.8 m/s,
# while zero thrust sinks -10.2 m/s. A single linear slope is wrong - feedforward
# MUST use this curve.
THRUST_CLIMB_TABLE = [
    (0.00, -10.19),
    (0.20,  -5.46),
    (0.299,  0.00),
    (0.35,   2.82),
    (0.50,  16.93),
    (0.75,  27.14),
    (1.00,  30.80),
]
# MEASURED forward speed (m/s) at a held lean angle (deg): 10->4.0, 20->7.3, 30->9.0.
# Airframe rate limits (commanded 6 rad/s): roll ~29 rad/s, yaw ~20 rad/s - so the
# MAX_RATE=6 clamp is well within authority.


def _thrust_for_climb(climb):
    """Invert the measured thrust->climb curve: collective thrust (LEVEL) that
    produces a given steady climb rate (m/s, up+). Clamped to the table ends."""
    pts = THRUST_CLIMB_TABLE
    if climb <= pts[0][1]:
        return pts[0][0]
    if climb >= pts[-1][1]:
        return pts[-1][0]
    for (t0, c0), (t1, c1) in zip(pts, pts[1:]):
        if c0 <= climb <= c1:
            return t0 + (t1 - t0) * (climb - c0) / (c1 - c0)
    return HOVER_THRUST
# Measured-attitude feedback signs. In this sim roll and pitch need OPPOSITE signs
# (pitch stable at -1, roll diverged at -1 so it uses +1). +pitch = lean forward.
ROLL_SIGN  = 1.0
PITCH_SIGN = -1.0


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust):
    now_ms = int(time.time() * 1000)
    mavlink_conn.mav.set_attitude_target_send(
        now_ms - system_boot_ms,
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,
        [1, 0, 0, 0],   # attitude quaternion (ignored in rate mode)
        roll_rate, pitch_rate, yaw_rate,
        thrust,
    )

# --------------------------------------------------------------------------------------
# KEYBOARD MANUAL CONTROL  (collect data - fly the drone around by hand)
# --------------------------------------------------------------------------------------
# Rate (acro) mode: each stick axis commands a body angular RATE that is ZERO when
# no key is pressed - press to rotate, release to stop and hold. Throttle is the one
# stateful axis (a keyboard has no analog stick): ramp it up to climb, ease it down
# to descend.
#
#   R / F        : throttle up / down  (held value; release to hold)  <- press R to take off
#   UP / DOWN    : pitch forward / back
#   LEFT / RIGHT : roll left / right
#   Q / E        : yaw left / right
#   ESC          : stop the client (ends the session and closes the dataset)
#   (avoid SPACE - the sim uses it to restart)
#
# NOTE: rate mode does not self-level. After a pitch/roll input the drone holds that
# bank, so tap the opposite key to level back out.
MANUAL_ROLL_RATE  = 0.6    # rad/s while LEFT/RIGHT held
MANUAL_PITCH_RATE = 0.6    # rad/s while UP/DOWN held
MANUAL_YAW_RATE   = 0.5    # rad/s while Q/E held
THROTTLE_STEP     = 0.0006  # thrust change per control tick (~0.15/sec at 250 Hz)


def update_keyboard_rate_control(mavlink_conn, system_boot_ms, data):
    # rotational axes: rate while held, 0 when released (no accumulation)
    roll_rate = pitch_rate = yaw_rate = 0.0
    if keyboard.is_pressed('up'): pitch_rate -= MANUAL_PITCH_RATE    # nose down = forward
    if keyboard.is_pressed('down'): pitch_rate += MANUAL_PITCH_RATE
    if keyboard.is_pressed('right'): roll_rate += MANUAL_ROLL_RATE   # roll right
    if keyboard.is_pressed('left'): roll_rate -= MANUAL_ROLL_RATE
    if keyboard.is_pressed('e'): yaw_rate += MANUAL_YAW_RATE         # yaw right
    if keyboard.is_pressed('q'): yaw_rate -= MANUAL_YAW_RATE

    # throttle is the only stateful axis
    thrust = data.get("manual_thrust", 0.0)
    if keyboard.is_pressed('r'): thrust += THROTTLE_STEP
    if keyboard.is_pressed('f'): thrust -= THROTTLE_STEP
    if keyboard.is_pressed('esc'): data["running"] = False

    thrust = _clamp(thrust, 0.0, 1.0)
    data["manual_thrust"] = thrust

    _send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust)

# --------------------------------------------------------------------------------------
# PURSUIT PILOT  (flies the course off ground-truth gate geometry)
# --------------------------------------------------------------------------------------
# Goal: reliably CLEAR the course (speed/optimal pathing come later). The previous
# pilot was a reactive single-gate image-error controller - it reacted to where the
# gate appeared frame-by-frame, had no model of the path ahead, and on the steep
# descending course it floated HIGH, let the gate fall out the bottom of the frame,
# and lost the line. This replaces it with proper INTERCEPT GUIDANCE off the sim's
# ground-truth gate positions (gates + odometry share one NED frame):
#
#   - Always chase the sim's ACTIVE gate (race_status.active_gate_index advances it
#     as we pass each one). active_gate_relative() gives its body-frame geometry.
#   - VERTICAL (the fix): command the climb/descent rate that lands us at the gate's
#     height exactly when we arrive (gap / time-to-go). Because the course drops,
#     this commands the descent BEFORE the gate sinks out of view, instead of
#     reacting once it already looks low. Geometric, no porpoise.
#   - YAW: turn to point at the gate (rate proportional to bearing).
#   - FORWARD: lean to cruise, scaled by how well we're pointed at the gate, so it
#     turns toward the line before building speed (no wide arcs) but never fully
#     stalls. PUNCH straight through once at the gate; RECOVER (turn back) only if
#     we clearly overshot and the sim hasn't advanced the gate.
#   - THRUST: feedforward from the measured thrust->climb curve (tilt-compensated)
#     + light P feedback. Hover thrust is measured, so no integral is needed.
#
# When all gates are done (active index past the last gate, or race finished) it
# holds a hover.
PURSUIT_REQUIRE_RACE = False   # True = wait out the countdown; False = fly whenever gates+pose are ready

PURSUIT_CRUISE_PITCH   = 0.15   # rad (~9 deg) forward lean at cruise -> ~3 m/s. Slow & safe.
# LATERAL centring is done by STRAFE (roll), not yaw. A quad can translate sideways
# without rotating; yawing to chase a lateral offset makes it SIDESLIP the other way
# at close range, which slid it left into the gate post (session 225315). Roll right
# when the gate is to our right -> translate onto the gate centreline.
PURSUIT_KP_LAT         = 0.16   # rad of strafe-roll per metre of lateral offset to the gate
PURSUIT_MAX_STRAFE     = 0.30   # rad cap on strafe roll (~17 deg). Flip KP_LAT sign if it
                                # strafes AWAY from the gate (right offset grows instead of shrinks)
PURSUIT_KP_YAW         = 1.0    # yaw-rate per rad of bearing (just heading now; strafe centres)
# Don't drive at full forward speed until we're laterally lined up, or we blow PAST an
# off-centre gate before the strafe can centre us (missed gate 3's 6 m lateral jog at
# full speed, session 230121). Lateral offset scales cruise down toward a floor.
PURSUIT_CENTER_SCALE   = 2.5    # m of lateral offset that cuts cruise to the floor
PURSUIT_MIN_SPEED_FAC  = 0.3    # never fully stop - keep creeping so it can't hover-stall
PURSUIT_MAX_YAW_RATE   = 3.0    # rad/s cap on yaw
PURSUIT_YAW_SIGN       = -1.0   # CONFIRMED by flight (session 220649): +1.0 yawed AWAY
                                # from the gate (azimuth ran 4->17->88->151 = spin). -1.0
                                # turns toward it. (Ground-truth azimuth sign is opposite
                                # the image offset_x the old vision controller used.)
PURSUIT_LOOKAHEAD      = 6.0    # m: yaw aims at bearing = atan2(right, max(fwd, LOOKAHEAD)),
                                # NOT atan2(right, fwd). Raw bearing spikes as fwd->0 at the
                                # gate (tiny lateral offset -> huge angle), which spun the
                                # drone sideways into the wall (session 224819). The lookahead
                                # floor caps the gain so it punches straight through instead.

# Vertical = CONSTANT-GAIN altitude tracking to the gate (first-order, NO gain spike).
# desired_climb = KP_H * (metres the gate centre is above us). The old intercept law
# (desired_climb = -down / t_go) had gain ~ v/horiz: WEAK far from the gate, so height
# drift built up across the whole approach, then SPIKED on close-in -> climbed above
# gate 0 and lost it out the frame bottom. Constant gain holds gate height the whole
# way, and the altitude loop self-trims around any hover-thrust error (no integral).
PURSUIT_KP_H           = 1.2    # 1/s, desired climb (m/s) per metre of height error
PURSUIT_MAX_CLIMB      = 5.0    # m/s
PURSUIT_MAX_DESCENT    = 6.0    # m/s (course drops steeply; airframe sinks ~10 m/s)
PURSUIT_KP_V           = 0.030  # thrust per m/s climb-rate error (feedback on top of FF)

# Forward-speed gating by bearing: full speed when pointed at the gate, ramp to zero
# forward as the bearing opens up so we turn toward the line first.
PURSUIT_ALIGN_FULL_DEG = 12.0   # within this bearing -> full cruise lean
PURSUIT_ALIGN_ZERO_DEG = 60.0   # beyond this bearing -> no forward lean (turn first)

# Gate-passing logic (forward = metres the gate is ahead of us in body frame)
PURSUIT_PASS_FWD       = 2.0    # within this forward distance -> PUNCH straight through
PURSUIT_LEAN_RAMP      = 0.08   # rad/s slew on the forward lean (no sudden kick)


def _load_cached_gates(data):
    # The ground-truth track layout is a ONE-SHOT broadcast (DATA_TRANSMISSION_
    # HANDSHAKE -> encapsulated track packets). If the client connects after the
    # sim already sent it, data["gates"] never gets populated and the pilot idles
    # forever. The track is STATIC (all cached gates.json are identical), so fall
    # back to the most recent cached layout. Live data overrides if it arrives.
    if data.get("gates") or data.get("_gates_cache_tried"):
        return
    data["_gates_cache_tried"] = True
    ds = os.path.join(os.path.dirname(os.path.abspath(__file__)), "datasets")
    files = sorted(glob.glob(os.path.join(ds, "*", "gates.json")), key=os.path.getmtime, reverse=True)
    for f in files:
        try:
            gates = json.load(open(f)).get("gates")
        except Exception:
            continue
        if gates:
            data["gates"] = gates
            data["_gates_from_cache"] = True
            data["_cached_gates_obj"] = gates   # so we can detect a live track replacing it
            print(f"[pursuit] live track not received; using cached layout "
                  f"{os.path.relpath(f, ds)} ({len(gates)} gates). Will be overridden if the sim sends one.",
                  flush=True)
            return
    print("[pursuit] no live track broadcast and no cached gates.json - cannot navigate (idle). "
          "Restart the RACE in the sim with this client running to get the live track.", flush=True)


def _pursuit_should_fly(data):
    if not (data.get("gates") and data.get("odometry") is not None):
        return False
    rs = data.get("race_status") or {}
    finish = rs.get("race_finish_time_ns", -1)
    if finish is not None and finish >= 0:
        return False   # race over
    if not PURSUIT_REQUIRE_RACE:
        return True
    start = rs.get("race_start_boot_time_ms", -1)
    now = rs.get("sim_boot_time_ms", 0)
    return start is not None and start >= 0 and now >= start


def _forward_speed_factor(azimuth_deg):
    # 1.0 when pointed at the gate, ramping to 0 as the bearing opens up
    a = abs(azimuth_deg)
    if a <= PURSUIT_ALIGN_FULL_DEG:
        return 1.0
    if a >= PURSUIT_ALIGN_ZERO_DEG:
        return 0.0
    return (PURSUIT_ALIGN_ZERO_DEG - a) / (PURSUIT_ALIGN_ZERO_DEG - PURSUIT_ALIGN_FULL_DEG)


def update_pursuit_control(mavlink_conn, system_boot_ms, data):
    if keyboard.is_pressed('esc'):
        data["running"] = False

    _load_cached_gates(data)   # fall back to cached track if the live broadcast was missed

    # if the sim's LIVE track arrives later, it replaces the cached list - trust it
    # (guaranteed to share this run's frame) and re-run the alignment check.
    if data.get("_gates_from_cache") and data.get("gates") is not data.get("_cached_gates_obj"):
        data["_gates_from_cache"] = False
        data.pop("_aligned", None)
        print("[pursuit] live track received - switching from cached layout.", flush=True)

    # Idle (on the ground, no thrust) until cleared to fly. Also resets the lean
    # ramp so a sim restart doesn't carry a wound-up lean.
    if not _pursuit_should_fly(data):
        data["vis_lean"] = 0.0
        data["oracle_thrust"] = 0.0
        data["pursuit_regime"] = "IDLE"
        _send_rate_attitude(mavlink_conn, system_boot_ms, 0.0, 0.0, 0.0, 0.0)
        return

    rel = active_gate_relative(data)
    odo = data.get("odometry") or {}
    att = data.get("attitude", {})

    # all gates cleared (index past the last gate) -> hover in place
    if rel is None:
        data["pursuit_regime"] = "DONE"
        roll_rate = _clamp(ROLL_SIGN * KP_ATT * (0.0 - att.get("roll", 0.0)), -MAX_RATE, MAX_RATE)
        pitch_rate = _clamp(PITCH_SIGN * KP_ATT * (0.0 - att.get("pitch", 0.0)), -MAX_RATE, MAX_RATE)
        data["oracle_thrust"] = HOVER_THRUST
        _send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, 0.0, HOVER_THRUST)
        return

    # ONE-TIME ALIGNMENT CHECK: at the first valid pose the active gate (gate 0)
    # should be roughly dead ahead. If it isn't, the gate map and our odometry
    # don't share a frame (almost always: a CACHED track that doesn't match this
    # run's spawn). Flying anyway just yaws at the off-axis gate forever = spinning
    # in circles. So refuse, and say exactly what's wrong.
    if "_aligned" not in data:
        data["_aligned"] = rel["forward"] > 0.0 and abs(rel["azimuth_deg"]) < 45.0
        src = "CACHED" if data.get("_gates_from_cache") else "live"
        if not data["_aligned"]:
            print(
                f"[pursuit] ABORT: {src} gate {rel['gate_id']} is not ahead "
                f"(fwd={rel['forward']:+.1f}m right={rel['right']:+.1f}m az={rel['azimuth_deg']:+.0f} deg, "
                f"dist={rel['distance']:.1f}m). The gate map and odometry are in different frames - "
                f"flying would just spin. Get the LIVE track: restart the race in the sim with this "
                f"client already running, then relaunch.", flush=True)
        else:
            print(f"[pursuit] alignment OK: {src} gate {rel['gate_id']} is ahead "
                  f"(fwd={rel['forward']:+.1f}m az={rel['azimuth_deg']:+.0f} deg). Flying.", flush=True)
    if not data["_aligned"]:
        data["pursuit_regime"] = "MISALIGNED"
        _send_rate_attitude(mavlink_conn, system_boot_ms, 0.0, 0.0, 0.0, 0.0)
        return

    roll = att.get("roll", 0.0)
    pitch = att.get("pitch", 0.0)
    climb_up = -odo.get("vz", 0.0)                          # vz is +down, so up-speed is -vz

    fwd = rel["forward"]
    az = rel["azimuth_deg"]

    # --- VERTICAL: constant-gain altitude tracking to the gate. height_error =
    # metres the gate centre is above us (+ = climb). First-order, no overshoot,
    # and the gain does NOT spike as we close in (unlike intercept / ÷t_go). ---
    height_error = -rel["down"]
    desired_climb = _clamp(PURSUIT_KP_H * height_error, -PURSUIT_MAX_DESCENT, PURSUIT_MAX_CLIMB)

    # LOOKAHEAD bearing to the gate: atan2(right, max(fwd, L)) so the yaw command and
    # the forward-speed gating can't spike as fwd->0 right at the gate (that spin into
    # the wall). Far out (fwd >> L) it's the true bearing; close in it stays small so
    # we punch STRAIGHT through instead of yawing sideways.
    look_bearing_deg = math.degrees(math.atan2(rel["right"], max(fwd, PURSUIT_LOOKAHEAD)))

    # --- HORIZONTAL regime: pursue toward the gate, punch through it, or recover. ---
    if fwd > -2.0 * PURSUIT_PASS_FWD:
        # PURSUE / PUNCH: drive forward toward the gate, STRAFE to centre laterally,
        # and yaw gently to keep the nose on it. Strafe (not yaw) does the centring so
        # we don't sideslip into the post at close range.
        regime = "PURSUE" if fwd > PURSUIT_PASS_FWD else "PUNCH"
        yaw_rate = _clamp(PURSUIT_YAW_SIGN * PURSUIT_KP_YAW * math.radians(look_bearing_deg),
                          -PURSUIT_MAX_YAW_RATE, PURSUIT_MAX_YAW_RATE)
        des_roll = _clamp(PURSUIT_KP_LAT * rel["right"], -PURSUIT_MAX_STRAFE, PURSUIT_MAX_STRAFE)
        # ease off forward speed while still off the gate centreline, so the strafe can
        # line us up before we reach the plane instead of blowing past an off-centre gate
        center_fac = _clamp(1.0 - abs(rel["right"]) / PURSUIT_CENTER_SCALE, PURSUIT_MIN_SPEED_FAC, 1.0)
        target_lean = PURSUIT_CRUISE_PITCH * _forward_speed_factor(look_bearing_deg) * center_fac
    else:
        # RECOVER: clearly overshot and the sim still wants this gate -> we missed it.
        # Use the TRUE bearing (gate is behind) to turn all the way around, slow down.
        regime = "RECOVER"
        yaw_rate = _clamp(PURSUIT_YAW_SIGN * PURSUIT_KP_YAW * math.radians(az), -PURSUIT_MAX_YAW_RATE, PURSUIT_MAX_YAW_RATE)
        target_lean = PURSUIT_CRUISE_PITCH * 0.3
        des_roll = 0.0

    # slew the forward lean so it eases in/out instead of kicking
    lean = data.get("vis_lean", 0.0)
    lean += _clamp(target_lean - lean, -PURSUIT_LEAN_RAMP / CONTROL_HZ, PURSUIT_LEAN_RAMP / CONTROL_HZ)
    data["vis_lean"] = lean

    # --- THRUST: feedforward the thrust that produces desired_climb off the MEASURED
    # thrust->climb curve, compensated for BOTH lean and strafe tilt, + light P feedback. ---
    cos_tilt = max(math.cos(lean) * math.cos(des_roll), 0.5)
    v_err = desired_climb - climb_up
    ff = _thrust_for_climb(desired_climb) / cos_tilt
    thrust = _clamp(ff + PURSUIT_KP_V * v_err, MIN_THRUST, MAX_THRUST)

    roll_rate = _clamp(ROLL_SIGN * KP_ATT * (des_roll - roll), -MAX_RATE, MAX_RATE)
    pitch_rate = _clamp(PITCH_SIGN * KP_ATT * (lean - pitch), -MAX_RATE, MAX_RATE)

    data["pursuit_regime"] = regime
    data["pursuit_desired_climb"] = desired_climb
    data["oracle_thrust"] = thrust   # reuse for the readout
    _send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust)

# --------------------------------------------------------------------------------------
# TRAJECTORY PILOT  (follows a pre-planned RACING LINE through the gates)
# --------------------------------------------------------------------------------------
# Where `pursuit` points straight at the active gate and slows at every corner (clears
# the course but slow, 34 s), this follows a smooth spline through all the gate centres
# and carries speed THROUGH the corners. Build the line once (trajectory.Trajectory),
# then each tick steer toward a CARROT point a fixed distance ahead along it, reusing
# the same low-level guidance: strafe to stay ON the line, pitch forward along it, yaw
# to face it, hold the line's height via the measured thrust curve. No per-gate
# stop/punch/recover - the line already threads the gates.
#
# This is the geometric racing line. Swapping in a time-optimal reference (CPC,
# arXiv:2108.04537) later changes only WHAT line we track, not this tracker.
TRAJ_LOOKAHEAD     = 5.0    # m ahead on the line to aim the carrot (smaller = tighter to
                           # the line/gates but twitchier; larger = smoother but cuts corners)
TRAJ_CRUISE_PITCH  = 0.22   # rad (~13 deg) -> ~6 m/s. Faster than pursuit's 0.15 since the
                           # smooth line means no stop-and-go. Push up toward 0.35 for speed.


def update_trajectory_control(mavlink_conn, system_boot_ms, data):
    if keyboard.is_pressed('esc'):
        data["running"] = False

    _load_cached_gates(data)
    # live track replaces the cache -> rebuild the line from the real gates
    if data.get("_gates_from_cache") and data.get("gates") is not data.get("_cached_gates_obj"):
        data["_gates_from_cache"] = False
        data["_traj"] = None
        print("[traj] live track received - rebuilding racing line.", flush=True)

    if not _pursuit_should_fly(data):
        data["vis_lean"] = 0.0
        data["oracle_thrust"] = 0.0
        data["traj_regime"] = "IDLE"
        _send_rate_attitude(mavlink_conn, system_boot_ms, 0.0, 0.0, 0.0, 0.0)
        return

    # build the racing line once (the track is static)
    if data.get("_traj") is None:
        from trajectory import Trajectory
        data["_traj"] = Trajectory(data["gates"])
        print(f"[traj] racing line built: {data['_traj'].length:.0f} m through "
              f"{len(data['gates'])} gates", flush=True)
    traj = data["_traj"]

    odo = data.get("odometry") or {}
    att = data.get("attitude", {})
    pos = (odo.get("x", 0.0), odo.get("y", 0.0), odo.get("z", 0.0))
    quat = (odo.get("qw", 1.0), odo.get("qx", 0.0), odo.get("qy", 0.0), odo.get("qz", 0.0))

    # carrot: a point LOOKAHEAD metres along the line from our nearest projection
    carrot, prog, xtrack = traj.carrot(pos, TRAJ_LOOKAHEAD)
    rel = relative_gate(pos, quat, {"gate_id": -1, "position_ned": [float(c) for c in carrot]})

    roll = att.get("roll", 0.0)
    pitch = att.get("pitch", 0.0)
    climb_up = -odo.get("vz", 0.0)
    fwd = rel["forward"]

    # VERTICAL: constant-gain to the carrot's height (same as pursuit, off the line)
    desired_climb = _clamp(PURSUIT_KP_H * (-rel["down"]), -PURSUIT_MAX_DESCENT, PURSUIT_MAX_CLIMB)
    # YAW: face the carrot (lookahead bearing, can't spike)
    look_bearing_deg = math.degrees(math.atan2(rel["right"], max(fwd, PURSUIT_LOOKAHEAD)))
    yaw_rate = _clamp(PURSUIT_YAW_SIGN * PURSUIT_KP_YAW * math.radians(look_bearing_deg),
                      -PURSUIT_MAX_YAW_RATE, PURSUIT_MAX_YAW_RATE)
    # STRAFE: roll to stay centred on the carrot/line
    des_roll = _clamp(PURSUIT_KP_LAT * rel["right"], -PURSUIT_MAX_STRAFE, PURSUIT_MAX_STRAFE)
    # FORWARD: cruise, eased by heading alignment and by how far we are OFF the line
    # (cross-track) - so cutting a corner auto-slows us until we're back on the line.
    center_fac = _clamp(1.0 - xtrack / PURSUIT_CENTER_SCALE, PURSUIT_MIN_SPEED_FAC, 1.0)
    target_lean = TRAJ_CRUISE_PITCH * _forward_speed_factor(look_bearing_deg) * center_fac

    lean = data.get("vis_lean", 0.0)
    lean += _clamp(target_lean - lean, -PURSUIT_LEAN_RAMP / CONTROL_HZ, PURSUIT_LEAN_RAMP / CONTROL_HZ)
    data["vis_lean"] = lean

    cos_tilt = max(math.cos(lean) * math.cos(des_roll), 0.5)
    thrust = _clamp(_thrust_for_climb(desired_climb) / cos_tilt + PURSUIT_KP_V * (desired_climb - climb_up),
                    MIN_THRUST, MAX_THRUST)
    roll_rate = _clamp(ROLL_SIGN * KP_ATT * (des_roll - roll), -MAX_RATE, MAX_RATE)
    pitch_rate = _clamp(PITCH_SIGN * KP_ATT * (lean - pitch), -MAX_RATE, MAX_RATE)

    data["traj_regime"] = f"TRACK {prog * 100:.0f}%"
    data["traj_xtrack"] = xtrack
    data["pursuit_desired_climb"] = desired_climb
    data["oracle_thrust"] = thrust
    _send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust)

# --------------------------------------------------------------------------------------
# PHYSICS CHARACTERIZATION  (measure the drone's envelope -> CSV, then tune for real)
# --------------------------------------------------------------------------------------
# Flies a scripted sequence of constant-command segments and logs commanded inputs +
# resulting state every tick. Analyse the CSV (analyze_performance.py) to extract:
# hover thrust, climb/descent rate vs thrust, vertical accel, forward speed vs lean
# angle, max body rates, and response lag - the numbers the shared constants above
# are set from.
#
# Each entry: (label, duration_s, params). params keys:
#   thrust     : 0..1 collective
#   level      : hold roll=pitch=0 (outer loop)
#   pitch_deg  : hold this forward pitch angle (outer loop) + thrust comp for tilt
#   roll_rate / pitch_rate / yaw_rate : command a raw body rate (probe max/lag)
# Climb CLEAR of the obstacle field before probing, so banks/full-thrust segments
# don't fly into buildings and contaminate the data. "hold_alt" closed-loops to a
# target altitude (m above spawn) via the measured curve, so it reaches and HOLDS
# there instead of an open-loop climb shooting off the map.
CHAR_CLIMB_ALT = 45.0   # m above spawn - above the course's building stacks
CHAR_PROFILE = [
    ("climbout", 16.0, {"hold_alt": CHAR_CLIMB_ALT}),       # rise clear of obstacles, then hold
    ("settle",    3.0, {"hold_alt": CHAR_CLIMB_ALT}),       # steady at altitude before probing
    ("thr_0.20",  2.5, {"thrust": 0.20, "level": True}),
    ("thr_0.35",  2.5, {"thrust": 0.35, "level": True}),
    ("thr_0.50",  2.0, {"thrust": 0.50, "level": True}),
    ("thr_0.75",  1.5, {"thrust": 0.75, "level": True}),
    ("thr_1.00",  1.5, {"thrust": 1.00, "level": True}),    # max climb
    ("recover1",  4.0, {"hold_alt": CHAR_CLIMB_ALT}),       # come back to altitude
    ("thr_0.00",  2.0, {"thrust": 0.00, "level": True}),    # max descent
    ("recover2",  4.0, {"hold_alt": CHAR_CLIMB_ALT}),
    ("pitch_10",  3.0, {"thrust": HOVER_THRUST, "pitch_deg": 10}),
    ("settle4",   2.5, {"hold_alt": CHAR_CLIMB_ALT}),
    ("pitch_20",  3.0, {"thrust": HOVER_THRUST, "pitch_deg": 20}),
    ("settle5",   2.5, {"hold_alt": CHAR_CLIMB_ALT}),
    ("pitch_30",  3.0, {"thrust": HOVER_THRUST, "pitch_deg": 30}),  # high-speed
    ("settle6",   2.5, {"hold_alt": CHAR_CLIMB_ALT}),
    ("rollrate",  2.0, {"thrust": HOVER_THRUST, "roll_rate": 6.0}),  # probe max roll rate
    ("settle7",   2.0, {"hold_alt": CHAR_CLIMB_ALT}),
    ("yawrate",   2.0, {"thrust": HOVER_THRUST, "yaw_rate": 6.0}),   # probe max yaw rate
    ("end",       1.0, {"thrust": 0.0, "level": True}),
]
CHAR_LOG_EVERY = 5   # log every Nth control tick (250 Hz / 5 = 50 Hz)
CHAR_CSV_HEADER = ["t_s", "phase", "cmd_thrust", "cmd_roll_rate", "cmd_pitch_rate", "cmd_yaw_rate",
                   "x", "y", "z", "vx", "vy", "vz", "speed_h",
                   "roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed"]


def _char_phase_at(t):
    acc = 0.0
    for label, dur, params in CHAR_PROFILE:
        if t < acc + dur:
            return label, params
        acc += dur
    return None, None   # profile finished


def update_characterize_control(mavlink_conn, system_boot_ms, data):
    if keyboard.is_pressed('esc'):
        data["running"] = False

    now = time.time()
    if "char_t0" not in data:
        out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "datasets")
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, time.strftime("characterize_%Y%m%d_%H%M%S.csv"))
        f = open(path, "w", newline="")
        w = csv.writer(f)
        w.writerow(CHAR_CSV_HEADER)
        data["char_t0"] = now
        data["char_f"] = f
        data["char_w"] = w
        data["char_path"] = path
        data["char_tick"] = 0
        print(f"CHARACTERIZING -> {path} (will fly aggressively; ESC to abort)", flush=True)

    t = now - data["char_t0"]
    label, params = _char_phase_at(t)
    if label is None:
        data["char_f"].flush()
        data["char_f"].close()
        print(f"Characterization complete: {data['char_path']}", flush=True)
        _send_rate_attitude(mavlink_conn, system_boot_ms, 0.0, 0.0, 0.0, 0.0)
        data["running"] = False
        return

    att = data.get("attitude", {})
    odo = data.get("odometry") or data.get("local_position_ned") or {}
    roll = att.get("roll", 0.0)
    pitch = att.get("pitch", 0.0)
    thrust = params.get("thrust", HOVER_THRUST)
    roll_rate = pitch_rate = yaw_rate = 0.0

    if "hold_alt" in params:
        # closed-loop climb to / hold a target altitude (m above spawn) using the
        # measured thrust->climb curve. Used to get clear of obstacles before probes.
        alt = -odo.get("z", 0.0)
        desired_climb = _clamp(0.8 * (params["hold_alt"] - alt), -4.0, 8.0)
        thrust = _clamp(_thrust_for_climb(desired_climb), 0.0, 1.0)
        roll_rate = _clamp(ROLL_SIGN * KP_ATT * (0.0 - roll), -MAX_RATE, MAX_RATE)
        pitch_rate = _clamp(PITCH_SIGN * KP_ATT * (0.0 - pitch), -MAX_RATE, MAX_RATE)
    elif params.get("level"):
        roll_rate = _clamp(ROLL_SIGN * KP_ATT * (0.0 - roll), -MAX_RATE, MAX_RATE)
        pitch_rate = _clamp(PITCH_SIGN * KP_ATT * (0.0 - pitch), -MAX_RATE, MAX_RATE)
    elif "pitch_deg" in params:
        des_pitch = math.radians(params["pitch_deg"])
        roll_rate = _clamp(ROLL_SIGN * KP_ATT * (0.0 - roll), -MAX_RATE, MAX_RATE)
        pitch_rate = _clamp(PITCH_SIGN * KP_ATT * (des_pitch - pitch), -MAX_RATE, MAX_RATE)
        thrust = thrust / max(math.cos(des_pitch), 0.5)   # hold altitude while leaning
    else:
        # raw body-rate probe (don't clamp - we want to find the real max)
        roll_rate = params.get("roll_rate", 0.0)
        pitch_rate = params.get("pitch_rate", 0.0)
        yaw_rate = params.get("yaw_rate", 0.0)

    thrust = _clamp(thrust, 0.0, 1.0)
    _send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust)

    # log (downsampled) commanded inputs + measured state
    data["char_tick"] += 1
    if data["char_tick"] % CHAR_LOG_EVERY == 0:
        odo = data.get("odometry") or {}
        vx, vy = odo.get("vx", 0.0), odo.get("vy", 0.0)
        data["char_w"].writerow([
            f"{t:.3f}", label, f"{thrust:.3f}", f"{roll_rate:.3f}", f"{pitch_rate:.3f}", f"{yaw_rate:.3f}",
            f"{odo.get('x', 0.0):.3f}", f"{odo.get('y', 0.0):.3f}", f"{odo.get('z', 0.0):.3f}",
            f"{vx:.3f}", f"{vy:.3f}", f"{odo.get('vz', 0.0):.3f}", f"{math.hypot(vx, vy):.3f}",
            f"{roll:.4f}", f"{pitch:.4f}", f"{att.get('yaw', 0.0):.4f}",
            f"{odo.get('rollspeed', 0.0):.3f}", f"{odo.get('pitchspeed', 0.0):.3f}", f"{odo.get('yawspeed', 0.0):.3f}",
        ])

# --------------------------------------------------------------------------------------
# Control Loop
# --------------------------------------------------------------------------------------
# How the control loop drives the drone:
#   "trajectory"   - follow a pre-planned racing line through the gates (FAST; new)
#   "pursuit"      - chase each gate in turn off ground-truth geometry (34 s baseline)
#   "keyboard"     - manual rate-mode flight via the keys above (data collection)
#   "characterize" - fly a scripted profile, log physics envelope to CSV (tuning)
CONTROL_MODE = "trajectory"

# print a status readout this often (seconds)
GATE_READOUT_PERIOD_S = 0.5


class Controller:
    def __init__(self, sim_conn, data, system_boot_ms):
        self.sim_conn = sim_conn
        self.data = data
        self.system_boot_ms = system_boot_ms
        self.tick = 0

    def update(self):
        if CONTROL_MODE == "trajectory":
            update_trajectory_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "pursuit":
            update_pursuit_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "keyboard":
            update_keyboard_rate_control(self.sim_conn, self.system_boot_ms, self.data)
        elif CONTROL_MODE == "characterize":
            update_characterize_control(self.sim_conn, self.system_boot_ms, self.data)
        else:
            raise ValueError(f"unknown CONTROL_MODE: {CONTROL_MODE!r}")

        self._readout()
        time.sleep(1.0 / CONTROL_HZ)

    def _readout(self):
        self.tick += 1
        if self.tick % int(CONTROL_HZ * GATE_READOUT_PERIOD_S) != 0:
            return

        odo = self.data.get("odometry") or self.data.get("local_position_ned") or {}
        vz = odo.get("vz")   # NED: + is downward
        z = odo.get("z")
        climb_str = f"{-vz:+.2f}" if vz is not None else " n/a"

        if CONTROL_MODE == "characterize":
            t = time.time() - self.data.get("char_t0", time.time())
            label, _ = _char_phase_at(t)
            vx, vy = odo.get("vx", 0.0), odo.get("vy", 0.0)
            print(
                f"char[{label}] t={t:5.1f}s  climb={climb_str}m/s  "
                f"speed_h={math.hypot(vx, vy):5.2f}m/s  alt={-(z or 0.0):+6.2f}m",
                flush=True,
            )
            return

        if CONTROL_MODE == "keyboard":
            thr = self.data.get("manual_thrust", 0.0)
            print(f"manual: thr={thr:.3f}  climb={climb_str}m/s  alt={-(z or 0.0):+6.2f}m", flush=True)
            return

        if CONTROL_MODE == "trajectory":
            thr = self.data.get("oracle_thrust", 0.0)
            regime = self.data.get("traj_regime", "?")
            vx, vy = odo.get("vx", 0.0), odo.get("vy", 0.0)
            rel = active_gate_relative(self.data)   # for reference: nearest real gate
            gatestr = f"gate {rel['gate_id']} dist={rel['distance']:4.1f}m" if rel else "no gate"
            print(
                f"traj[{regime}] xtrack={self.data.get('traj_xtrack', 0.0):4.1f}m "
                f"speed={math.hypot(vx, vy):4.1f}m/s climb={climb_str} alt={-(z or 0.0):+6.1f}m "
                f"| {gatestr} thr={thr:.3f}",
                flush=True,
            )
            return

        # pursuit readout: show the regime + the active gate it's chasing
        thr = self.data.get("oracle_thrust", 0.0)
        regime = self.data.get("pursuit_regime", "?")
        rs = self.data.get("race_status") or {}
        rel = active_gate_relative(self.data)
        if rel is not None:
            dc = self.data.get("pursuit_desired_climb", 0.0)
            print(
                f"pursuit[{regime}] gate {rel['gate_id']} dist={rel['distance']:5.1f}m "
                f"fwd={rel['forward']:+6.1f} right={rel['right']:+6.1f} down={rel['down']:+6.1f} "
                f"az={rel['azimuth_deg']:+4.0f} | thr={thr:.3f} climb={climb_str} want={dc:+.2f}m/s",
                flush=True,
            )
        else:
            print(
                f"pursuit[{regime}] armed={self.data.get('armed')} "
                f"gates={len(self.data.get('gates') or [])} "
                f"pose={'ok' if self.data.get('odometry') is not None else 'MISSING'} "
                f"active_gate={rs.get('active_gate_index')} thr={thr:.3f}",
                flush=True,
            )

    # -------------------------------
    # Arm the drone
    # -------------------------------
    def arm(self):
        self.sim_conn.mav.command_long_send(
            self.sim_conn.target_system,
            self.sim_conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # arm
            0, 0, 0, 0, 0, 0
        )

    def send_sim_reset_command(self):
        self.sim_conn.mav.command_long_send(
            self.sim_conn.target_system,
            self.sim_conn.target_component,
            MAVLINK_CMD_SIM_RESET,
            0,  # confirmation
            0, 0, 0, 0, 0, 0, 0
        )
