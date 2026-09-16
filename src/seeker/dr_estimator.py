"""
Vision-aided dead reckoning: the state source that lets the follower fly a
plan with no position sensor.

    src = DeadReckonSource()
    est = src.estimate(update)                 # every tick (sim SensorUpdate)
    src.apply_fix(det, gate_xyz)               # whenever a gate is in view

Between gate sightings the horizontal position is integrated from the
accelerometer rotated by the attitude (specific force minus gravity),
altitude and vertical speed come from the barometer, heading from the
attitude. Every sighting of a known gate gives a position fix: the gate's
map position minus the sighting's range along its bearing. The fix is
blended in, and the velocity is nudged toward what the fixes imply, so
drift never grows beyond one leg.

Sim mode reads the sim's SensorUpdate (accel, attitude quaternion, and
ground-truth altitude/vario as the stand-in for the FC's filtered
barometer). The hardware source will feed the same class from
hardware.state.FcStateSource: attitude, raw accel, altitude, vario.
"""

from __future__ import annotations

import math

import numpy as np

from raceline.rc_backend import StateEstimate, rot_from_quat
from seeker import synthetic_camera as cam

G = 9.80665


class DeadReckonSource:
    def __init__(self, fix_gain: float = 0.35, vel_gain: float = 0.5, v_decay_s: float = 30.0,
                 start_xy=(0.0, 0.0)):
        self.fix_gain = fix_gain
        self.vel_gain = vel_gain
        self.v_decay_s = v_decay_s
        self.p = np.array([start_xy[0], start_xy[1], 0.0], dtype=float)
        self.v = np.zeros(3)
        self.t_prev = None
        self.t_last_fix = None
        self.p_at_last_fix = None
        self.fixes = 0
        self.fix_residual = 0.0
        self.R = np.eye(3)
        self.last_est = None

    # --- prediction -------------------------------------------------------------
    def integrate(self, t: float, R: np.ndarray, accel_body, z: float, vz: float):
        if self.t_prev is None:
            self.t_prev = t
        dt = max(0.0, min(0.05, t - self.t_prev))
        self.t_prev = t
        self.R = R
        a_w = R @ np.asarray(accel_body, dtype=float) - np.array([0.0, 0.0, G])
        # horizontal: integrate; vertical: trust the barometer
        self.v[0] += a_w[0] * dt
        self.v[1] += a_w[1] * dt
        if self.v_decay_s > 0:                          # bounded drift when no fixes arrive
            self.v[0] -= self.v[0] * dt / self.v_decay_s
            self.v[1] -= self.v[1] * dt / self.v_decay_s
        self.p[0] += self.v[0] * dt
        self.p[1] += self.v[1] * dt
        self.p[2] = z
        self.v[2] = vz

    def estimate(self, u) -> StateEstimate:
        """Sim SensorUpdate -> StateEstimate."""
        R = rot_from_quat(u.world_pos[0:4])
        # stand-in for the FC's filtered altitude + vario (MSP_ALTITUDE)
        z = float(u.world_pos[6])
        vz = float(u.world_vel[5])
        self.integrate(float(u.t), R, u.accel, z, vz)
        omega_w = np.asarray(u.world_vel[0:3], dtype=float)
        yaw = math.atan2(R[1, 0], R[0, 0])
        self.last_est = StateEstimate(p=self.p.copy(), v=self.v.copy(), R=R, yaw=yaw, omega=R.T @ omega_w)
        return self.last_est

    # --- correction -----------------------------------------------------------------
    def apply_fix(self, det, gate_xyz) -> float:
        """Position fix from a sighting of the gate at gate_xyz. Returns the
        residual (m) between the dead-reckoned and the fixed position."""
        rng = det.range_m if getattr(det, "range_m", None) else cam.range_from_area(det.area_frac)
        d_w = self.R @ cam.direction_body(det)
        d_h = np.array([d_w[0], d_w[1]])
        n = np.linalg.norm(d_h)
        if n < 1e-6:
            return 0.0
        d_h /= n
        p_fix = np.array([gate_xyz[0], gate_xyz[1]]) - rng * d_h
        r = p_fix - self.p[:2]
        self.p[0] += self.fix_gain * r[0]
        self.p[1] += self.fix_gain * r[1]
        # a consistent residual over time means the velocity is wrong: nudge it
        if self.t_last_fix is not None and self.t_prev is not None:
            dt = self.t_prev - self.t_last_fix
            if 0.02 < dt < 2.0:
                self.v[0] += self.vel_gain * r[0] / max(dt, 0.2) * min(1.0, dt)
                self.v[1] += self.vel_gain * r[1] / max(dt, 0.2) * min(1.0, dt)
        self.t_last_fix = self.t_prev
        self.fixes += 1
        self.fix_residual = float(np.hypot(r[0], r[1]))
        return self.fix_residual
