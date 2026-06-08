"""
Low-level drone model and actuator interface.

The sim flies a throttle/thrust model and accepts RATE (acro) setpoints: each axis is a
body angular rate (rad/s) plus a collective thrust 0..1. The pilots build outer loops on
top of this (P on measured attitude -> rate) to fly attitudes and velocities.

All the numbers here are MEASURED (characterize mode + analyze_performance.py), not guessed.
Change them only against fresh measurement data.
"""

import math
import time

from pymavlink import mavutil

CONTROL_HZ = 250
MAVLINK_CMD_SIM_RESET = 31000

HOVER_THRUST = 0.299   # collective thrust at which climb rate crosses zero
KP_ATT = 3.0           # body-rate per rad of attitude error (4.0 caused PIO)
MAX_RATE = 20.0        # rad/s clamp on commanded body rates. Measured airframe max ~29 (roll/
                       # pitch), ~19 (yaw) via the sysid campaign; 20 unlocks snappy corners/
                       # recoveries with margin. (Was 6.0 - far below the airframe's real limit.)
MIN_THRUST = 0.0
MAX_THRUST = 1.0
G_ACC = 9.81           # m/s^2, for the accel->tilt conversion: tan(tilt) = a_horizontal / g

# Measured attitude-feedback signs. Roll and pitch need opposite signs in this sim
# (pitch is stable at -1, roll diverged at -1 so it uses +1). +pitch leans forward.
ROLL_SIGN = 1.0
PITCH_SIGN = -1.0

# Measured sign of commanded yaw rate. +1 yawed AWAY from the target; -1 turns toward it.
YAW_SIGN = -1.0

# Shared vertical limits and thrust feedback gain (used by both pilots).
MAX_CLIMB = 10.0       # m/s (the airframe climbs far harder than the old table assumed)
MAX_DESCENT = 15.0     # m/s. The sysid No-Go-Zone shows a 30 m/s drop recovers in ~13 m, so a
                       # 15 m/s descent needs only a few metres to arrest - dive hard. (Was 6.0.)
KP_THRUST_V = 0.070    # thrust per m/s of climb-rate error (feedback on top of the FF). Raised
                       # from 0.030: at steep lean the stale/too-hot hover curve made the drone
                       # balloon up over the gates; stronger climb-rate feedback holds altitude
                       # by correcting the FF error directly. (Shared with the pursuit pilot.)

# Measured steady-state thrust -> climb rate (m/s, up+). Strongly nonlinear and climb-biased:
# hover ~0.299, 0.50 -> +16.9 m/s, full -> +30.8 m/s, zero thrust sinks -10.2 m/s. A single
# linear slope is wrong, so the feedforward must use this curve.
THRUST_CLIMB_TABLE = [
    (0.00, -10.19),
    (0.20, -5.46),
    (0.299, 0.00),
    (0.35, 2.82),
    (0.50, 16.93),
    (0.75, 27.14),
    (1.00, 30.80),
]

# Measured forward speed (m/s) vs held forward lean (rad): 10deg->4.0, 20deg->7.3, 30deg->9.0.
SPEED_LEAN_TABLE = [(0.0, 0.0), (4.0, 0.175), (7.3, 0.349), (9.0, 0.524)]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def thrust_for_climb(climb):
    """Collective thrust (while level) that produces a given steady climb rate (m/s, up+).
    Inverts THRUST_CLIMB_TABLE; clamped to the table ends."""
    pts = THRUST_CLIMB_TABLE
    if climb <= pts[0][1]:
        return pts[0][0]
    if climb >= pts[-1][1]:
        return pts[-1][0]
    for (t0, c0), (t1, c1) in zip(pts, pts[1:]):
        if c0 <= climb <= c1:
            return t0 + (t1 - t0) * (climb - c0) / (c1 - c0)
    return HOVER_THRUST


def lean_for_speed(v):
    """Forward lean (rad) that holds a given steady forward speed (m/s). Inverts SPEED_LEAN_TABLE."""
    pts = SPEED_LEAN_TABLE
    if v <= 0.0:
        return 0.0
    if v >= pts[-1][0]:
        return pts[-1][1]
    for (v0, l0), (v1, l1) in zip(pts, pts[1:]):
        if v0 <= v <= v1:
            return l0 + (l1 - l0) * (v - v0) / (v1 - v0)
    return pts[-1][1]


def send_rate_attitude(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust):
    """Send a body-rate + collective-thrust setpoint to the sim.

    NOTE: this does NOT clamp the rates - clamping is the caller's job. The pilots clamp to
    MAX_RATE; the sysid harness deliberately commands beyond it to find the true airframe max.
    """
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


def send_arm(mavlink_conn, arm=True):
    """Arm (or disarm) the vehicle. Single source of truth for the arm command."""
    mavlink_conn.mav.command_long_send(
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,                  # confirmation
        1 if arm else 0,    # 1 = arm, 0 = disarm
        0, 0, 0, 0, 0, 0,
    )


def send_sim_reset(mavlink_conn):
    """Reset the simulator to its initial state (drone back at spawn). Used between sysid trials."""
    mavlink_conn.mav.command_long_send(
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        MAVLINK_CMD_SIM_RESET,
        0,                  # confirmation
        0, 0, 0, 0, 0, 0, 0,
    )
