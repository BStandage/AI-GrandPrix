"""
Pursuit pilot: chase each gate in turn off ground-truth gate geometry.

This is the proven fallback (cleared the course at ~34 s). It points at the sim's active gate,
descends to arrive at the gate's height, strafes to centre laterally, and punches straight
through. It clears the course reliably but slows at every corner; the trajectory pilot is the
faster default. Kept here as a known-good backup.
"""

import math

import keyboard

from dynamics import (CONTROL_HZ, HOVER_THRUST, KP_ATT, KP_THRUST_V, MAX_CLIMB, MAX_DESCENT,
                      MAX_RATE, MAX_THRUST, MIN_THRUST, PITCH_SIGN, ROLL_SIGN, YAW_SIGN, clamp,
                      send_rate_attitude, thrust_for_climb)
from gate_geometry import active_gate_relative
from race import load_cached_gates, seconds_to_go, should_fly

PURSUIT_CRUISE_PITCH = 0.22    # rad (~13 deg) forward lean at cruise -> ~5 m/s

# Lateral centring is done by strafe (roll), not yaw: a quad translates sideways without
# rotating, and yawing to chase a lateral offset sideslips it into the post at close range.
PURSUIT_KP_LAT = 0.16          # rad of strafe-roll per metre of lateral offset to the gate
PURSUIT_MAX_STRAFE = 0.30      # rad cap on strafe roll (~17 deg). Flip KP_LAT sign if it strafes away.
PURSUIT_KP_YAW = 1.0           # yaw-rate per rad of bearing (heading only; strafe does the centring)

# Ease off forward speed while off-centre so the strafe can line up before reaching the plane.
PURSUIT_CENTER_SCALE = 2.5     # m of lateral offset that cuts cruise to the floor
PURSUIT_MIN_SPEED_FAC = 0.3    # never fully stop, so it can't hover-stall
PURSUIT_MAX_YAW_RATE = 3.0     # rad/s cap on yaw

# Yaw aims at bearing = atan2(right, max(fwd, LOOKAHEAD)) so it can't spike as fwd->0 at the gate.
PURSUIT_LOOKAHEAD = 6.0        # m

PURSUIT_KP_H = 1.2             # desired climb (m/s) per metre of height error to the gate

# Forward-speed gating by bearing: full speed when pointed at the gate, zero when far off.
PURSUIT_ALIGN_FULL_DEG = 12.0
PURSUIT_ALIGN_ZERO_DEG = 60.0

PURSUIT_PASS_FWD = 2.0         # within this forward distance, punch straight through
PURSUIT_LEAN_RAMP = 0.08       # rad/s slew on the forward lean


def forward_speed_factor(azimuth_deg):
    """1.0 when pointed at the gate, ramping to 0 as the bearing opens up."""
    a = abs(azimuth_deg)
    if a <= PURSUIT_ALIGN_FULL_DEG:
        return 1.0
    if a >= PURSUIT_ALIGN_ZERO_DEG:
        return 0.0
    return (PURSUIT_ALIGN_ZERO_DEG - a) / (PURSUIT_ALIGN_ZERO_DEG - PURSUIT_ALIGN_FULL_DEG)


def update_pursuit_control(mavlink_conn, system_boot_ms, data):
    if keyboard.is_pressed('esc'):
        data["running"] = False

    load_cached_gates(data)

    # If a live track arrives later it replaces the cache; trust it and re-run the alignment check.
    if data.get("_gates_from_cache") and data.get("gates") is not data.get("_cached_gates_obj"):
        data["_gates_from_cache"] = False
        data.pop("_aligned", None)
        print("[pursuit] live track received - switching from cached layout.", flush=True)

    # Idle (no thrust) until cleared to fly. Reset the lean ramp so a restart launches clean.
    if not should_fly(data):
        data["vis_lean"] = 0.0
        data["oracle_thrust"] = 0.0
        t_go = seconds_to_go(data)
        if t_go is not None and t_go > 0 and data.get("gates"):
            data["pursuit_regime"] = f"WAIT-GO {t_go:.1f}s"
        else:
            data["pursuit_regime"] = "IDLE"
        send_rate_attitude(mavlink_conn, system_boot_ms, 0.0, 0.0, 0.0, 0.0)
        return

    rel = active_gate_relative(data)
    odo = data.get("odometry") or {}
    att = data.get("attitude", {})

    # All gates cleared -> hover in place.
    if rel is None:
        data["pursuit_regime"] = "DONE"
        roll_rate = clamp(ROLL_SIGN * KP_ATT * (0.0 - att.get("roll", 0.0)), -MAX_RATE, MAX_RATE)
        pitch_rate = clamp(PITCH_SIGN * KP_ATT * (0.0 - att.get("pitch", 0.0)), -MAX_RATE, MAX_RATE)
        data["oracle_thrust"] = HOVER_THRUST
        send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, 0.0, HOVER_THRUST)
        return

    # One-time alignment check: at the first valid pose gate 0 should be roughly dead ahead.
    # If not, the gate map and odometry are in different frames and flying would just spin.
    if "_aligned" not in data:
        data["_aligned"] = rel["forward"] > 0.0 and abs(rel["azimuth_deg"]) < 45.0
        src = "CACHED" if data.get("_gates_from_cache") else "live"
        if not data["_aligned"]:
            print(
                f"[pursuit] ABORT: {src} gate {rel['gate_id']} is not ahead "
                f"(fwd={rel['forward']:+.1f}m right={rel['right']:+.1f}m az={rel['azimuth_deg']:+.0f} deg, "
                f"dist={rel['distance']:.1f}m). Gate map and odometry are in different frames. "
                f"Restart the race with this client running, then relaunch.", flush=True)
        else:
            print(f"[pursuit] alignment OK: {src} gate {rel['gate_id']} is ahead "
                  f"(fwd={rel['forward']:+.1f}m az={rel['azimuth_deg']:+.0f} deg). Flying.", flush=True)
    if not data["_aligned"]:
        data["pursuit_regime"] = "MISALIGNED"
        send_rate_attitude(mavlink_conn, system_boot_ms, 0.0, 0.0, 0.0, 0.0)
        return

    roll = att.get("roll", 0.0)
    pitch = att.get("pitch", 0.0)
    climb_up = -odo.get("vz", 0.0)
    fwd = rel["forward"]
    az = rel["azimuth_deg"]

    # Vertical: constant-gain altitude tracking to the gate (no gain spike close in).
    height_error = -rel["down"]
    desired_climb = clamp(PURSUIT_KP_H * height_error, -MAX_DESCENT, MAX_CLIMB)

    # Lookahead bearing so the yaw and forward-speed gating can't spike as fwd->0 at the gate.
    look_bearing_deg = math.degrees(math.atan2(rel["right"], max(fwd, PURSUIT_LOOKAHEAD)))

    if fwd > -2.0 * PURSUIT_PASS_FWD:
        # Pursue / punch: drive forward, strafe to centre, yaw gently to keep the nose on it.
        regime = "PURSUE" if fwd > PURSUIT_PASS_FWD else "PUNCH"
        yaw_rate = clamp(YAW_SIGN * PURSUIT_KP_YAW * math.radians(look_bearing_deg),
                         -PURSUIT_MAX_YAW_RATE, PURSUIT_MAX_YAW_RATE)
        des_roll = clamp(PURSUIT_KP_LAT * rel["right"], -PURSUIT_MAX_STRAFE, PURSUIT_MAX_STRAFE)
        center_fac = clamp(1.0 - abs(rel["right"]) / PURSUIT_CENTER_SCALE, PURSUIT_MIN_SPEED_FAC, 1.0)
        target_lean = PURSUIT_CRUISE_PITCH * forward_speed_factor(look_bearing_deg) * center_fac
    else:
        # Recover: clearly overshot. Use the true bearing to turn around, and slow down.
        regime = "RECOVER"
        yaw_rate = clamp(YAW_SIGN * PURSUIT_KP_YAW * math.radians(az), -PURSUIT_MAX_YAW_RATE, PURSUIT_MAX_YAW_RATE)
        target_lean = PURSUIT_CRUISE_PITCH * 0.3
        des_roll = 0.0

    # Slew the forward lean so it eases in/out instead of kicking.
    lean = data.get("vis_lean", 0.0)
    lean += clamp(target_lean - lean, -PURSUIT_LEAN_RAMP / CONTROL_HZ, PURSUIT_LEAN_RAMP / CONTROL_HZ)
    data["vis_lean"] = lean

    # Thrust: measured climb-curve FF, compensated for lean and strafe tilt, + light P feedback.
    cos_tilt = max(math.cos(lean) * math.cos(des_roll), 0.5)
    v_err = desired_climb - climb_up
    ff = thrust_for_climb(desired_climb) / cos_tilt
    thrust = clamp(ff + KP_THRUST_V * v_err, MIN_THRUST, MAX_THRUST)

    roll_rate = clamp(ROLL_SIGN * KP_ATT * (des_roll - roll), -MAX_RATE, MAX_RATE)
    pitch_rate = clamp(PITCH_SIGN * KP_ATT * (lean - pitch), -MAX_RATE, MAX_RATE)

    data["pursuit_regime"] = regime
    data["pursuit_desired_climb"] = desired_climb
    data["oracle_thrust"] = thrust
    send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust)
