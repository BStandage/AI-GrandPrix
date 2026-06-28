"""
Phase-1 racing pilot: follow a pre-planned racing line through the GROUND-TRUTH gates, but with the
aggressive Phase-1 limits (pilots.phase1_pilot.config) instead of the conservative oracle gains.

Structurally identical to the oracle (pilots.oracle_pilot): it builds a smooth line through the gates
once and tracks it with a measured-dynamics follower. The ONLY difference is the follower - this pilot
uses pilots.phase1_pilot.phase1_follower (its own copy with the unlocked slew/attitude limits), so the
oracle and vision pilots are left completely unchanged.
"""

import keyboard

from common.dynamics import send_rate_attitude
from common.race import load_cached_gates, seconds_to_go, should_fly
from pilots.phase1_pilot.config import P1_RIDGE_FLOOR_ALT, P1_RIDGE_X_MAX, P1_RIDGE_X_MIN
from pilots.phase1_pilot.phase1_follower import P1_GROUND_MARGIN, build_line, follow_line


def update_phase1_control(mavlink_conn, system_boot_ms, data):
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
        data["_traj_state"] = {}       # reset slew + integral so a (re)start launches clean
        t_go = seconds_to_go(data)
        data["traj_regime"] = (f"WAIT-GO {t_go:.1f}s" if (t_go and t_go > 0 and data.get("gates")) else "IDLE")
        send_rate_attitude(mavlink_conn, system_boot_ms, 0.0, 0.0, 0.0, 0.0)
        return

    # Build the racing line from the current gates, and rebuild whenever the gate list changes.
    if data.get("_traj") is None or data.get("_traj_gates") is not data.get("gates"):
        data["_traj"] = build_line(data["gates"])
        data["_traj_gates"] = data["gates"]
        print(f"[traj] racing line built: {data['_traj'].length:.0f} m through "
              f"{len(data['gates'])} gates", flush=True)
    traj = data["_traj"]

    odo = data.get("odometry") or {}
    att = data.get("attitude", {})
    pos = (odo.get("x", 0.0), odo.get("y", 0.0), odo.get("z", 0.0))
    quat = (odo.get("qw", 1.0), odo.get("qx", 0.0), odo.get("qy", 0.0), odo.get("qz", 0.0))
    vb = (odo.get("vx", 0.0), odo.get("vy", 0.0), odo.get("vz", 0.0))
    att_rp = (att.get("roll", 0.0), att.get("pitch", 0.0))

    # Floor guard referenced to the lowest gate centre (gates 4/5 rest on the floor), not spawn.
    floor_alt = min(-g["position_ned"][2] for g in data["gates"]) - P1_GROUND_MARGIN
    # Course-specific raised floor between gates 3 and 4 (see config): the racing line dips below
    # this ridge, so hold a local altitude floor over its x-window. Released elsewhere so the drone
    # can still descend to the low gates 4/5. Round-1 throwaway - delete with the config block.
    if P1_RIDGE_X_MIN <= pos[0] <= P1_RIDGE_X_MAX:
        floor_alt = max(floor_alt, P1_RIDGE_FLOOR_ALT)

    state = data.setdefault("_traj_state", {})
    roll_rate, pitch_rate, yaw_rate, thrust, telem = follow_line(traj, pos, quat, vb, att_rp, state, floor_alt)

    data["traj_regime"] = f"RIP {telem['prog'] * 100:.0f}%"
    data["traj_xtrack"] = telem["xtrack"]
    data["traj_vtgt"] = telem["v_target"]
    data["traj_vcur"] = telem["v_cur"]
    data["traj_yawrate"] = telem["yaw_rate"]
    data["pursuit_desired_climb"] = telem["desired_climb"]
    data["oracle_thrust"] = thrust
    send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust)
