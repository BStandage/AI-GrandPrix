"""SOLVER TEMPLATE - copy this file, rename it, make it fly.

    cp _template.py my_solver.py
    RACE_SOLVER=solvers.my_solver AIGP_SIM_TIME=30 uv run elodin run sim/main.py

This one arms, takes off, and holds a hover 3 m in front of the first
gate. Boring on purpose: every piece a real solver needs is here and
labeled.

WHERE THE KNOBS ARE (edit config/vehicle.toml, not code):
    speed caps          [limits] v_max_mps, v_gate_mps
    tilt (= accel) cap  [limits] max_tilt_deg      -> cfg.a_lat_full()
    pitch/roll sticks   [follower] stick_clamp, ka_att
    yaw stick           [follower] kyaw, yaw_clamp
    throttle loop       [follower] kp_z, kd_z, ki_z (m/s^2 units) + [thrust] curve
The loader is strict: a typo'd key fails loudly. Add new keys to the
schema in raceline/config.py if your solver needs its own.
"""

from __future__ import annotations

import os

import numpy as np

from raceline import course as course_bridge   # puts the sim repo on sys.path
from raceline.config import load_config
from raceline.rc_backend import (AltitudeLoop, GroundTruthSource, YawLoop,
                                 attitude_sticks)

course_bridge.pq_course()
from solver.api import RCCommand, SensorUpdate  # noqa: E402

# --- one-time setup ---------------------------------------------------------
CFG = load_config(os.environ.get("AIGP_VEHICLE_TOML"))   # the tuning surface
SOURCE = GroundTruthSource()   # sim ground truth; estimator swaps in later
ALT = AltitudeLoop(CFG)        # proven throttle loop (hover ff + PID)
YAW = YawLoop(CFG)             # proven yaw-stick loop

HOVER_TARGET = np.array([0.0, 0.0, 1.5])   # spawn is 3 m before gate g0


def reset_state() -> None:
    """Called between runs; clear anything stateful."""
    ALT.reset()
    YAW.reset()


def autopilot(update: SensorUpdate) -> RCCommand:
    t = update.t

    # 1) Betaflight arming handshake - keep this exactly.
    if t < 0.50:
        return RCCommand(arm=1000, throttle=1000)
    if t < 0.75:
        return RCCommand(arm=1800, throttle=1000)

    # 2) State: position/velocity/attitude in world frame.
    est = SOURCE.estimate(update)

    # 3) YOUR BRAIN GOES HERE. Decide a desired horizontal acceleration
    #    (m/s^2, world frame). This template: PD hold over HOVER_TARGET.
    f = CFG.follower
    a_des = (f.kp_pos * (HOVER_TARGET[:2] - est.p[:2])
             - f.kd_pos * est.v[:2])

    # Clamp to the tilt budget - the ONLY correct speed/aggression limiter.
    a_max = CFG.a_lat_full()             # g * tan(max_tilt_deg)
    n = float(np.hypot(a_des[0], a_des[1]))
    if n > a_max:
        a_des *= a_max / n

    # 4) Loops -> sticks. Don't reinvent these.
    airborne = est.p[2] >= f.min_alt_translation_m
    throttle = ALT.throttle(t, est, float(HOVER_TARGET[2]), 0.0, airborne,
                            update.baro_fresh)
    roll = pitch = yaw_stick = 1500
    if airborne:
        roll, pitch, _ = attitude_sticks(CFG, est, a_des)
        yaw_stick = YAW.stick(est, None)   # pass a heading (rad) to aim

    return RCCommand(arm=1800, throttle=throttle, roll=roll, pitch=pitch,
                     yaw=yaw_stick)
