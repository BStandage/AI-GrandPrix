"""
Vision pilot: fly the course from the CAMERA alone - no ground-truth gate geometry.

This is the Round-1 deliverable. It's the proven pursuit controller (pursuit_pilot.py) refed by
vision: instead of the sim's active_gate_relative (ground truth), it steers on the detected gate's
position IN THE IMAGE (vision_rx -> data["vision_target"]):

  - lateral: strafe (roll) to centre the gate horizontally (offset_x). The image bearing is
    pixel-accurate - it does NOT depend on the noisy PnP distance, so this is robust.
  - vertical: climb/descend to aim the body forward-axis at the gate, from offset_y + the camera's
    26 deg up-tilt (a gate at the drone's own height sits LOW in frame because the cam looks up).
  - forward: cruise lean, eased off when the gate is off-centre / off-bearing so the strafe can
    line up; punch straight through when the gate is close (rough distance only - coarse is fine).

Distance is used ONLY coarsely (punch-through timing), so single-frame PnP noise can't destabilise
it. A light EMA smooths the detector jitter. Deliberately slower than the oracle (accuracy-limited,
and slow = more frames per gate = more robust).

Gating: flies once the race is live (race_status), NOT on ground-truth gates - it must work
without them. Idles until then, and hovers/searches when no gate is in view.
"""

import csv
import math
import os

import keyboard

from common.dynamics import (CONTROL_HZ, HOVER_THRUST, KP_ATT, KP_THRUST_V, MAX_CLIMB, MAX_DESCENT,
                      MAX_RATE, MAX_THRUST, MIN_THRUST, PITCH_SIGN, ROLL_SIGN, YAW_SIGN, clamp,
                      send_rate_attitude, thrust_for_climb)
from common.paths import DATASETS_DIR

# Image -> angle. The detector normalises offsets to half-width/half-height. With fx=fy=320 on a
# 640x360 frame: a full horizontal offset is atan(320/320)=45 deg, full vertical atan(180/320)=29.4
# deg. So angle = atan(offset * HALF_TAN).
VIS_HALF_TAN_X = 1.0          # tan(45 deg)
VIS_HALF_TAN_Y = 0.5625       # tan(29.4 deg) = 180/320

VIS_CRUISE_PITCH = 0.22       # rad (~13 deg) forward lean at cruise -> ~5 m/s. Slow on purpose.
VIS_LEAN_RAMP = 0.10          # rad/s slew on the forward lean

# Lateral centring by strafe (roll), keyed to the image bearing (offset_x in [-1,1]) - the
# pixel-accurate signal, NO distance dependence. (Tried metric dist*offx: distance noise + far-gate
# amplification made it miss wide. Bearing is the robust signal; we handle overshoot by slowing
# down off-centre + lookahead, not by adding noisy distance.) Flip sign if it strafes away.
VIS_KP_LAT = 0.45             # rad of strafe-roll per unit offset_x
VIS_MAX_STRAFE = 0.40         # rad cap (~23 deg)

# YAW IS OFF for vision. Yawing swings the CAMERA - if it overshoots, the gate leaves the frame
# and we go blind (this happened: it yawed ~90 deg and lost the gate). A quad centres a gate by
# STRAFING (translate sideways, camera stays pointed forward), so the gate stays in view the whole
# time. The proven pursuit pilot could yaw because it had ground truth even with the gate off-screen.
VIS_KP_YAW = 0.0             # keep the nose fixed; strafe does all the centring
VIS_MAX_YAW_RATE = 1.0        # rad/s cap (only relevant if KP_YAW is raised later)

# Vertical: visual-servo the gate to a fixed ROW in the frame (offset_y = VIS_TARGET_OFFY), NOT to
# body-forward. The camera tilts up ~26 deg, so "gate centred" means the gate is ABOVE the flight
# path - aiming the body at it makes the drone CLIMB to chase and fly OVER every gate (observed).
# Holding the gate at a lower frame row makes it descend WITH the course. offy > target = gate too
# low in frame = drone too high -> descend.
VIS_TARGET_OFFY = 0.65        # desired gate row in frame (+down). The up-tilted camera puts a gate
                              # at the drone's OWN altitude at offy~0.55-0.85, so 0.30 made it dive
                              # under every gate (chasing the gate up the frame). RAISE if it still
                              # flies UNDER / clips bottoms; LOWER if it flies OVER / clips tops.
VIS_KP_VE = 4.0               # desired climb (m/s) per unit of offset_y error. Lowered from 6 to
                              # damp the close-range vertical oscillation that crashed it at a gate.

# Forward-speed gating by bearing: full speed pointed at the gate, easing to a floor as it opens up.
VIS_ALIGN_FULL_DEG = 10.0
VIS_ALIGN_ZERO_DEG = 50.0
VIS_CENTER_SCALE = 0.45       # units of |offset_x| that cut cruise to the floor (slow when off-centre
                              # so the strafe lines up before reaching the gate -> less overshoot)
VIS_MIN_SPEED_FAC = 0.22      # near-stop when badly off-centre (no full hover-stall)

VIS_PASS_DIST = 4.0           # m: within this rough distance, punch straight through (don't slow)
VIS_LOST_HOLD_S = 0.4         # keep the last command this long after losing the gate, then hover
VIS_TRACK_GATE = 0.35         # max image jump (offset units) to treat a detection as the SAME gate
                              # we're tracking; beyond it, the tracked gate is gone -> chase nearest

# Light EMA on the detector signal (offset_x/y, distance) to take the edge off per-frame jitter.
# Kept high (responsive) - the bearing changes meaningfully as we approach, so heavy smoothing
# would lag the pursuit and aim where the gate WAS.
VIS_EMA = 0.5


def _race_live(data):
    rs = data.get("race_status") or {}
    start = rs.get("race_start_boot_time_ms", -1)
    now = rs.get("sim_boot_time_ms", 0)
    finish = rs.get("race_finish_time_ns", -1)
    over = finish is not None and finish >= 0
    live = start is not None and start >= 0 and now >= start
    return live and not over


def _seconds_to_go(data):
    rs = data.get("race_status") or {}
    start = rs.get("race_start_boot_time_ms", -1)
    now = rs.get("sim_boot_time_ms", 0)
    if start is None or start < 0:
        return None
    return (start - now) / 1000.0


def _forward_speed_factor(bearing_deg):
    a = abs(bearing_deg)
    if a <= VIS_ALIGN_FULL_DEG:
        return 1.0
    if a >= VIS_ALIGN_ZERO_DEG:
        return 0.0
    return (VIS_ALIGN_ZERO_DEG - a) / (VIS_ALIGN_ZERO_DEG - VIS_ALIGN_FULL_DEG)


# --- per-tick lateral debug log (overwritten each run) so we can SEE the strafe behaviour at a
# gate instead of guessing. Columns let us trace offx -> des_roll -> achieved through a transit.
_DBG_W = _DBG_F = None
_DBG_HDR = ["t", "regime", "gate_id", "offx_raw", "offx_sm", "offy_sm", "dist",
            "des_roll", "des_climb", "lean", "roll", "alt"]


def _dbg_log(row):
    global _DBG_W, _DBG_F
    if _DBG_W is None:
        path = os.path.join(DATASETS_DIR, "vision_pilot_dbg.csv")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        _DBG_F = open(path, "w", newline="")
        _DBG_W = csv.writer(_DBG_F)
        _DBG_W.writerow(_DBG_HDR)
        print(f"vision-pilot lateral debug -> {path}", flush=True)
    _DBG_W.writerow(row)
    _DBG_F.flush()


def _idle(mavlink_conn, system_boot_ms, data, regime):
    data["vision_regime"] = regime
    data["vis_lean"] = 0.0
    data["oracle_thrust"] = 0.0
    data["_vis_ema"] = None          # reset the smoother so a restart locks on clean
    data["_vis_tgt_img"] = None      # forget the tracked gate
    send_rate_attitude(mavlink_conn, system_boot_ms, 0.0, 0.0, 0.0, 0.0)


def _select_target(data):
    """Pick the gate to chase with PERSISTENCE - track the same gate across frames instead of
    re-grabbing the biggest blob each frame. When several gates are in view at similar size, the
    biggest flips between them frame-to-frame, whipsawing the strafe (it rolled the drone over).
    We instead choose the detection nearest (in the image) to last frame's target; only when that
    gate has no continuation (it was passed / left view) do we jump to the new nearest (biggest)."""
    gates = data.get("vision_gates") or []
    if not gates:
        data["_vis_tgt_img"] = None
        return None
    prev = data.get("_vis_tgt_img")
    if prev is None:
        tgt = gates[0]
    else:
        tgt = min(gates, key=lambda g: (g["offset_x"] - prev[0]) ** 2 + (g["offset_y"] - prev[1]) ** 2)
        d2 = (tgt["offset_x"] - prev[0]) ** 2 + (tgt["offset_y"] - prev[1]) ** 2
        if d2 > VIS_TRACK_GATE ** 2:   # tracked gate gone (passed/lost) -> take the new nearest
            tgt = gates[0]
    data["_vis_tgt_img"] = (tgt["offset_x"], tgt["offset_y"])
    return tgt


def update_vision_control(mavlink_conn, system_boot_ms, data):
    if keyboard.is_pressed('esc'):
        data["running"] = False

    # Fly only once the race is live - and on VISION, never ground-truth gates.
    if not _race_live(data):
        t_go = _seconds_to_go(data)
        _idle(mavlink_conn, system_boot_ms, data,
              f"WAIT-GO {t_go:.1f}s" if (t_go and t_go > 0) else "IDLE")
        return

    att = data.get("attitude", {})
    odo = data.get("odometry") or {}
    roll = att.get("roll", 0.0)
    pitch = att.get("pitch", 0.0)
    climb_up = -odo.get("vz", 0.0)

    target = _select_target(data)   # persistent tracked gate, NOT just the biggest blob each frame

    # No gate in view: coast on the last command briefly (likely mid-transition between gates),
    # then level off and hover so we don't wander. The next gate is usually already visible.
    if target is None:
        data["_vis_lost"] = data.get("_vis_lost", 0) + 1
        if data["_vis_lost"] * (1.0 / CONTROL_HZ) > VIS_LOST_HOLD_S:
            data["vision_regime"] = "NO-GATE"
            rr = clamp(ROLL_SIGN * KP_ATT * (0.0 - roll), -MAX_RATE, MAX_RATE)
            pr = clamp(PITCH_SIGN * KP_ATT * (0.0 - pitch), -MAX_RATE, MAX_RATE)
            thrust = clamp(thrust_for_climb(0.0) + KP_THRUST_V * (0.0 - climb_up), MIN_THRUST, MAX_THRUST)
            data["oracle_thrust"] = thrust
            send_rate_attitude(mavlink_conn, system_boot_ms, rr, pr, 0.0, thrust)
            return
        # within the hold window: keep flying the last setpoints (handled by falling through with
        # the previous smoothed target)
        sm = data.get("_vis_ema")
        if sm is None:
            _idle(mavlink_conn, system_boot_ms, data, "NO-GATE")
            return
    else:
        data["_vis_lost"] = 0
        # EMA the raw detector signal. Reset on a big jump (we locked onto a new/next gate).
        raw = (target["offset_x"], target["offset_y"], target.get("distance_m", 10.0))
        sm = data.get("_vis_ema")
        if sm is None or abs(raw[0] - sm[0]) > 0.5 or abs(raw[2] - sm[2]) > 8.0:
            sm = raw
        else:
            a = VIS_EMA
            sm = (a * raw[0] + (1 - a) * sm[0],
                  a * raw[1] + (1 - a) * sm[1],
                  a * raw[2] + (1 - a) * sm[2])
        data["_vis_ema"] = sm

    offx, offy, dist = sm

    bearing_x = math.atan(offx * VIS_HALF_TAN_X)             # +right (for the speed-gating only)

    # Vertical: visual-servo the gate to VIS_TARGET_OFFY in the FRAME (see the note above). offy >
    # target = gate too low in frame = drone too high -> descend.
    desired_climb = clamp(-VIS_KP_VE * (offy - VIS_TARGET_OFFY), -MAX_DESCENT, MAX_CLIMB)

    # Lateral: strafe to centre the gate in the image (bearing-based, distance-independent).
    des_roll = clamp(VIS_KP_LAT * offx, -VIS_MAX_STRAFE, VIS_MAX_STRAFE)

    # Yaw: gently point the nose at the gate (cosmetic-ish; strafe does the centring).
    yaw_rate = clamp(YAW_SIGN * VIS_KP_YAW * bearing_x, -VIS_MAX_YAW_RATE, VIS_MAX_YAW_RATE)

    # Forward lean: cruise, eased off when off-bearing or off-centre; full-commit when close.
    bearing_deg = math.degrees(bearing_x)
    if dist <= VIS_PASS_DIST:
        regime = "PUNCH"
        target_lean = VIS_CRUISE_PITCH                       # commit straight through
    else:
        regime = "PURSUE"
        center_fac = clamp(1.0 - abs(offx) / VIS_CENTER_SCALE, VIS_MIN_SPEED_FAC, 1.0)
        target_lean = VIS_CRUISE_PITCH * _forward_speed_factor(bearing_deg) * center_fac

    lean = data.get("vis_lean", 0.0)
    lean += clamp(target_lean - lean, -VIS_LEAN_RAMP / CONTROL_HZ, VIS_LEAN_RAMP / CONTROL_HZ)
    data["vis_lean"] = lean

    # Thrust: measured climb-curve FF, tilt-compensated for lean+strafe, + light climb feedback.
    cos_tilt = max(math.cos(lean) * math.cos(des_roll), 0.5)
    thrust = clamp(thrust_for_climb(desired_climb) / cos_tilt + KP_THRUST_V * (desired_climb - climb_up),
                   MIN_THRUST, MAX_THRUST)

    roll_rate = clamp(ROLL_SIGN * KP_ATT * (des_roll - roll), -MAX_RATE, MAX_RATE)
    pitch_rate = clamp(PITCH_SIGN * KP_ATT * (lean - pitch), -MAX_RATE, MAX_RATE)

    data["vision_regime"] = regime
    data["pursuit_desired_climb"] = desired_climb
    data["oracle_thrust"] = thrust
    data["vis_dbg"] = (offx, offy, dist, desired_climb)

    # per-tick lateral trace (raw vs smoothed offx, commanded vs achieved roll) for diagnosis
    data["_vis_t"] = data.get("_vis_t", 0) + 1
    raw_offx = target["offset_x"] if target is not None else offx
    gid = target.get("gate_id", -1) if (target is not None and isinstance(target, dict)) else -1
    _dbg_log([f"{data['_vis_t'] / CONTROL_HZ:.3f}", regime, gid,
              f"{raw_offx:.3f}", f"{offx:.3f}", f"{offy:.3f}", f"{dist:.1f}",
              f"{des_roll:.3f}", f"{desired_climb:.2f}", f"{lean:.3f}",
              f"{roll:.3f}", f"{-odo.get('z', 0.0):.1f}"])

    send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust)
