"""
State from the flight controller alone: attitude, heading, barometric
altitude and vertical speed. No position - there is no sensor for it.

    src = FcStateSource(bridge, map_north_heading_deg=...)
    est = src.estimate()      # raceline.rc_backend.StateEstimate

Fills a StateEstimate the way the existing loops expect it:
    p = (0, 0, baro altitude)         horizontal position unknown -> 0
    v = (0, 0, vario)                 horizontal velocity unknown -> 0
    R = body->world from roll/pitch/yaw
    yaw = world yaw, CCW from +x (east), from the FC heading
    omega = body rates from the gyro (deg/s -> rad/s)

Frame facts to pin on the bench (each is one flag here):
- Betaflight heading is 0..360 clockwise from magnetic north. World yaw
  is CCW from the map's +x. `map_north_heading_deg` is the compass
  heading of the map's +y axis, measured on site by pointing the drone
  along gate 1's direction (north on the PDF).
- Betaflight roll is positive right-wing-down. Betaflight pitch sign is
  set by `pitch_nose_up_positive`; verify with the telemetry tool by
  tilting the nose down by hand (default assumes nose-DOWN reads
  negative, i.e. nose-up positive).
- MSP_RAW_IMU gyro is deg/s in the FC's body frame (x forward, y right,
  z down = FRD). Converted to FLU body rates here.
"""

from __future__ import annotations

import math
import time

import numpy as np

from raceline.rc_backend import StateEstimate


def rot_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Body(FLU)->world rotation for yaw about z, then pitch about y (nose
    UP positive), then roll about x (right wing down positive)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return Rz @ Ry @ Rx


class FcStateSource:
    def __init__(self, bridge, map_north_heading_deg: float = 0.0,
                 pitch_nose_up_positive: bool = True, roll_right_positive: bool = True,
                 alt_offset_m: float = 0.0, acc_lsb_per_g: float = 512.0,
                 acc_signs=(1.0, 1.0, 1.0)):
        # MSP_RAW_IMU accel: Betaflight reports its sensor frame (x forward,
        # y left, z up; +1 g on z at rest). acc_lsb_per_g is the raw count of
        # 1 g (512 on most boards, 256 on the SITL): read it at rest on the
        # bench. acc_signs flips axes if the tilt test disagrees.
        self.acc_lsb_per_g = acc_lsb_per_g
        self.acc_signs = acc_signs
        self.bridge = bridge
        self.map_north_heading_deg = map_north_heading_deg
        self.pitch_sign = 1.0 if pitch_nose_up_positive else -1.0
        self.roll_sign = 1.0 if roll_right_positive else -1.0
        self.alt_offset_m = alt_offset_m
        self.last_t = 0.0

    def zero_altitude(self) -> None:
        """Call on the ground before takeoff: baro altitude is relative."""
        s = self.bridge.state()
        if s.altitude is not None:
            self.alt_offset_m = s.altitude.alt_m

    def heading_to_world_yaw(self, heading_deg: float) -> float:
        """Compass heading (CW from north) -> world yaw (CCW from +x east)
        in the map frame whose +y points at `map_north_heading_deg`."""
        rel = heading_deg - self.map_north_heading_deg      # CW from map +y
        yaw_deg = 90.0 - rel                                 # CCW from map +x
        return math.radians((yaw_deg + 180.0) % 360.0 - 180.0)

    def estimate(self) -> StateEstimate | None:
        s = self.bridge.state()
        if s.attitude is None:
            return None
        a = s.attitude
        # Betaflight roll: right wing down positive. In an FLU body frame a
        # positive roll about +x (forward) ALSO drops the right wing? No:
        # +x rotation lifts +y (left) -> left wing up = right wing down. Same.
        roll = self.roll_sign * math.radians(a.roll_deg)
        pitch_nose_up = self.pitch_sign * math.radians(a.pitch_deg)
        # FLU: +y rotation dips the nose, so nose-up is a NEGATIVE pitch.
        pitch = -pitch_nose_up
        yaw = self.heading_to_world_yaw(a.yaw_deg)
        R = rot_zyx(roll, pitch, yaw)
        alt = (s.altitude.alt_m - self.alt_offset_m) if s.altitude is not None else 0.0
        vz = s.altitude.vario_mps if s.altitude is not None else 0.0
        omega = None
        self.accel_body = None
        if s.imu is not None:
            gx, gy, gz = (math.radians(v) for v in s.imu.gyro)   # FRD deg/s
            omega = np.array([gx, -gy, -gz])                      # -> FLU
            g0 = 9.80665
            self.accel_body = np.array([self.acc_signs[i] * s.imu.acc[i] / self.acc_lsb_per_g * g0
                                        for i in range(3)])       # body FLU specific force, m/s^2
        self.last_t = s.t
        return StateEstimate(p=np.array([0.0, 0.0, alt]), v=np.array([0.0, 0.0, vz]),
                             R=R, yaw=yaw, omega=omega)
