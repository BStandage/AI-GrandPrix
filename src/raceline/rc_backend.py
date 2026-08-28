"""Shared RC-stick backend: state estimate + the flight-proven control
loops that turn desired accel / altitude / yaw into Betaflight sticks.

Used by the trajectory follower AND the sysid diagnostics, so both fly the
identical plant interface. This module is also the sim->real boundary: on
hardware, GroundTruthSource is replaced by an estimator and the RCCommand
goes to a UART writer — these loops and their toml gains carry.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from raceline.config import G


@dataclass
class StateEstimate:
    p: np.ndarray      # world position [x, y, z]
    v: np.ndarray      # world velocity [vx, vy, vz]
    R: np.ndarray      # body->world rotation, 3x3
    yaw: float         # world yaw (CCW from +x)


def rot_from_quat(q) -> np.ndarray:
    """3x3 body->world rotation from Elodin scalar-last quat [qx,qy,qz,qw]."""
    qx, qy, qz, qw = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


class GroundTruthSource:
    """Sim-only state source: ground-truth pose/velocity from the update."""

    def estimate(self, u) -> StateEstimate:
        R = rot_from_quat(u.world_pos[0:4])
        return StateEstimate(
            p=np.asarray(u.world_pos[4:7], dtype=float),
            v=np.asarray(u.world_vel[3:6], dtype=float),
            R=R,
            yaw=math.atan2(R[1, 0], R[0, 0]),
        )


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class AltitudeLoop:
    """Proven throttle loop (hover ff + P on z + D on vz vs plan + guarded I).

    The integral only runs on small in-flight errors: winding up during the
    takeoff climb measurably biased altitude +0.4 m for ~30 s.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.i_term = 0.0
        self.last_t = 0.0

    def throttle(self, t: float, est: StateEstimate, z_target: float,
                 vz_ff: float, airborne: bool, integrate: bool) -> int:
        f, th = self.cfg.follower, self.cfg.thrust
        dt = max(1e-3, t - self.last_t)
        err = z_target - est.p[2]
        if integrate and airborne and abs(err) < 0.5:
            self.i_term = clamp(self.i_term + err * dt * f.ki_z, -60.0, 60.0)
        if not airborne and est.v[2] < 0.7:
            out = f.takeoff_pwm
        else:
            out = (th.hover_pwm + f.kp_z * err + f.kd_z * (vz_ff - est.v[2])
                   + self.i_term)
        self.last_t = t
        return int(round(clamp(out, th.pwm_min, th.pwm_max)))


def attitude_sticks(cfg, est: StateEstimate, a_des) -> tuple:
    """World-frame desired accel -> roll/pitch sticks via the tilt-vector
    error expressed in body frame (the proven acro loop)."""
    f = cfg.follower
    ax, ay = float(a_des[0]), float(a_des[1])
    n = math.sqrt(ax * ax + ay * ay + G * G)
    zd = np.array([ax / n, ay / n, G / n])
    zb = est.R[:, 2]
    e = np.cross(zb, zd)
    eb = est.R.T @ e
    roll = int(round(clamp(1500.0 + f.ka_att * eb[0],
                           1500 - f.stick_clamp, 1500 + f.stick_clamp)))
    pitch = int(round(clamp(1500.0 + f.ka_att * eb[1],
                            1500 - f.stick_clamp, 1500 + f.stick_clamp)))
    return roll, pitch, eb


class YawLoop:
    """Yaw stick toward a target heading, holding the last valid target."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.hold = None

    def stick(self, est: StateEstimate, yaw_des) -> int:
        f = self.cfg.follower
        if yaw_des is not None:
            self.hold = yaw_des
        if self.hold is None:
            return 1500
        yerr = (self.hold - est.yaw + math.pi) % (2 * math.pi) - math.pi
        # +stick = yaw right = world yaw DECREASES (measured), hence the minus
        return int(round(clamp(1500.0 - f.kyaw * yerr,
                               1500 - f.yaw_clamp, 1500 + f.yaw_clamp)))
