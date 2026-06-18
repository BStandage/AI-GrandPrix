"""
Developer modes that are not the autonomous pilot:

  keyboard     - fly the drone by hand to collect data
  characterize - fly a scripted profile and log the physics envelope to CSV (then
                 analyze_performance.py turns it into the measured numbers in dynamics.py)
"""

import csv
import math
import os
import time

import keyboard

from common.dynamics import (HOVER_THRUST, KP_ATT, MAX_RATE, PITCH_SIGN, ROLL_SIGN, clamp,
                      send_rate_attitude, thrust_for_climb)
from common.paths import DATASETS_DIR

# --- Keyboard manual control ---------------------------------------------------------------
# Rate (acro) mode: each axis commands a body rate that is zero when the key is released.
# Throttle is the one stateful axis (a keyboard has no analog stick).
#   R / F        : throttle up / down (press R to take off)
#   UP / DOWN    : pitch forward / back
#   LEFT / RIGHT : roll left / right
#   Q / E        : yaw left / right
#   ESC          : stop the client
# Rate mode does not self-level: after a pitch/roll input the drone holds that bank.
MANUAL_ROLL_RATE = 0.6     # rad/s while LEFT/RIGHT held
MANUAL_PITCH_RATE = 0.6    # rad/s while UP/DOWN held
MANUAL_YAW_RATE = 0.5      # rad/s while Q/E held
THROTTLE_STEP = 0.0006     # thrust change per control tick (~0.15/sec at 250 Hz)


def update_keyboard_rate_control(mavlink_conn, system_boot_ms, data):
    roll_rate = pitch_rate = yaw_rate = 0.0
    if keyboard.is_pressed('up'): pitch_rate -= MANUAL_PITCH_RATE    # nose down = forward
    if keyboard.is_pressed('down'): pitch_rate += MANUAL_PITCH_RATE
    if keyboard.is_pressed('right'): roll_rate += MANUAL_ROLL_RATE
    if keyboard.is_pressed('left'): roll_rate -= MANUAL_ROLL_RATE
    if keyboard.is_pressed('e'): yaw_rate += MANUAL_YAW_RATE
    if keyboard.is_pressed('q'): yaw_rate -= MANUAL_YAW_RATE

    thrust = data.get("manual_thrust", 0.0)
    if keyboard.is_pressed('r'): thrust += THROTTLE_STEP
    if keyboard.is_pressed('f'): thrust -= THROTTLE_STEP
    if keyboard.is_pressed('esc'): data["running"] = False

    thrust = clamp(thrust, 0.0, 1.0)
    data["manual_thrust"] = thrust
    send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust)


# --- Physics characterization --------------------------------------------------------------
# Flies a scripted sequence of constant-command segments and logs commanded inputs + measured
# state. Climb clear of the obstacle field first so banks/full-thrust segments don't hit
# buildings. "hold_alt" closed-loops to a target altitude via the measured curve.
CHAR_CLIMB_ALT = 45.0   # m above spawn, above the course's building stacks
CHAR_PROFILE = [
    ("climbout", 16.0, {"hold_alt": CHAR_CLIMB_ALT}),
    ("settle", 3.0, {"hold_alt": CHAR_CLIMB_ALT}),
    ("thr_0.20", 2.5, {"thrust": 0.20, "level": True}),
    ("thr_0.35", 2.5, {"thrust": 0.35, "level": True}),
    ("thr_0.50", 2.0, {"thrust": 0.50, "level": True}),
    ("thr_0.75", 1.5, {"thrust": 0.75, "level": True}),
    ("thr_1.00", 1.5, {"thrust": 1.00, "level": True}),    # max climb
    ("recover1", 4.0, {"hold_alt": CHAR_CLIMB_ALT}),
    ("thr_0.00", 2.0, {"thrust": 0.00, "level": True}),    # max descent
    ("recover2", 4.0, {"hold_alt": CHAR_CLIMB_ALT}),
    ("pitch_10", 3.0, {"thrust": HOVER_THRUST, "pitch_deg": 10}),
    ("settle4", 2.5, {"hold_alt": CHAR_CLIMB_ALT}),
    ("pitch_20", 3.0, {"thrust": HOVER_THRUST, "pitch_deg": 20}),
    ("settle5", 2.5, {"hold_alt": CHAR_CLIMB_ALT}),
    ("pitch_30", 3.0, {"thrust": HOVER_THRUST, "pitch_deg": 30}),  # high speed
    ("settle6", 2.5, {"hold_alt": CHAR_CLIMB_ALT}),
    ("rollrate", 2.0, {"thrust": HOVER_THRUST, "roll_rate": 6.0}),  # probe max roll rate
    ("settle7", 2.0, {"hold_alt": CHAR_CLIMB_ALT}),
    ("yawrate", 2.0, {"thrust": HOVER_THRUST, "yaw_rate": 6.0}),    # probe max yaw rate
    ("end", 1.0, {"thrust": 0.0, "level": True}),
]
CHAR_LOG_EVERY = 5   # log every Nth control tick (250 Hz / 5 = 50 Hz)
CHAR_CSV_HEADER = ["t_s", "phase", "cmd_thrust", "cmd_roll_rate", "cmd_pitch_rate", "cmd_yaw_rate",
                   "x", "y", "z", "vx", "vy", "vz", "speed_h",
                   "roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed"]


def char_phase_at(t):
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
        out_dir = DATASETS_DIR
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
    label, params = char_phase_at(t)
    if label is None:
        data["char_f"].flush()
        data["char_f"].close()
        print(f"Characterization complete: {data['char_path']}", flush=True)
        send_rate_attitude(mavlink_conn, system_boot_ms, 0.0, 0.0, 0.0, 0.0)
        data["running"] = False
        return

    att = data.get("attitude", {})
    odo = data.get("odometry") or data.get("local_position_ned") or {}
    roll = att.get("roll", 0.0)
    pitch = att.get("pitch", 0.0)
    thrust = params.get("thrust", HOVER_THRUST)
    roll_rate = pitch_rate = yaw_rate = 0.0

    if "hold_alt" in params:
        alt = -odo.get("z", 0.0)
        desired_climb = clamp(0.8 * (params["hold_alt"] - alt), -4.0, 8.0)
        thrust = clamp(thrust_for_climb(desired_climb), 0.0, 1.0)
        roll_rate = clamp(ROLL_SIGN * KP_ATT * (0.0 - roll), -MAX_RATE, MAX_RATE)
        pitch_rate = clamp(PITCH_SIGN * KP_ATT * (0.0 - pitch), -MAX_RATE, MAX_RATE)
    elif params.get("level"):
        roll_rate = clamp(ROLL_SIGN * KP_ATT * (0.0 - roll), -MAX_RATE, MAX_RATE)
        pitch_rate = clamp(PITCH_SIGN * KP_ATT * (0.0 - pitch), -MAX_RATE, MAX_RATE)
    elif "pitch_deg" in params:
        des_pitch = math.radians(params["pitch_deg"])
        roll_rate = clamp(ROLL_SIGN * KP_ATT * (0.0 - roll), -MAX_RATE, MAX_RATE)
        pitch_rate = clamp(PITCH_SIGN * KP_ATT * (des_pitch - pitch), -MAX_RATE, MAX_RATE)
        thrust = thrust / max(math.cos(des_pitch), 0.5)   # hold altitude while leaning
    else:
        # raw body-rate probe (don't clamp - we want the real max)
        roll_rate = params.get("roll_rate", 0.0)
        pitch_rate = params.get("pitch_rate", 0.0)
        yaw_rate = params.get("yaw_rate", 0.0)

    thrust = clamp(thrust, 0.0, 1.0)
    send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust)

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
