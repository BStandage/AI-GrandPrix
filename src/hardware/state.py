"""
State from the flight controller alone: attitude, heading, barometric
altitude and vertical speed. No position - there is no sensor for it.

    src = FcStateSource(bridge, map_north_heading_deg=...)
    est = src.estimate()      # raceline.rc_backend.StateEstimate

Fills a StateEstimate the way the existing loops expect it:
    p = (0, 0, baro altitude)         horizontal position unknown -> 0
    v = (0, 0, vertical speed)        horizontal velocity unknown -> 0

VERTICAL SPEED IS COMPUTED HERE, not taken from the flight controller.
Betaflight's MSP_ALTITUDE carries a vario field and on d45 (2026-09-20,
BF 4.4.3) it reads exactly 0.00 m/s forever - verified by hand, lifting the
aircraft 0.76 m onto a table and back a dozen times while `alt` tracked every
lift cleanly and `vario` never moved. Trusting it cost us the altitude loop's
damping term and, worse, the airborne latch in `hardware.runtime` that gates
dead reckoning - that latch waits for vz > 0.5 m/s, so it would never have
fired and the follower would have flown the whole course believing it was
still sitting on the start line.

So we run our own `VerticalFilter` (accel + baro) and report ITS vz. `p[2]`
stays the raw barometer, because the dead-reckoning path downstream feeds it
into a second VerticalFilter of its own and must not get a twice-filtered
height. The FC's vario is still read and compared, and `fc_vario_alive` says
whether this aircraft reports one at all.
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
        # 1 g (2048 on the Archer's blackbox header, 256 on the SITL): the
        # runtime measures it at rest. acc_signs flips axes if the tilt test
        # disagrees.
        self.acc_lsb_per_g = acc_lsb_per_g
        self.acc_signs = acc_signs
        self.bridge = bridge
        self.map_north_heading_deg = map_north_heading_deg
        self.heading_drift_dpm = 0.0   # deg/min, from `bench drift`; 0 = off
        self._drift_t0 = None
        self.pitch_sign = 1.0 if pitch_nose_up_positive else -1.0
        self.roll_sign = 1.0 if roll_right_positive else -1.0
        self.alt_offset_m = alt_offset_m
        self.last_t = 0.0
        # our own vertical observer - see the note at the top of this file
        from seeker.dr_estimator import VerticalFilter
        self.vert = VerticalFilter()
        self._vert_t = None          # monotonic clock of the last update
        self._alt_obj = None         # identity of the last Altitude, for freshness
        self.fc_vario_alive = False  # has the FC EVER reported a non-zero vario

    def zero_altitude(self) -> None:
        """Call on the ground before takeoff: baro altitude is relative."""
        s = self.bridge.state()
        if s.altitude is not None:
            self.alt_offset_m = s.altitude.alt_m
        # the observer's height is relative to the offset we just took
        from seeker.dr_estimator import VerticalFilter
        self.vert = VerticalFilter()
        self._vert_t = None
        self._alt_obj = None

    def heading_to_world_yaw(self, heading_deg: float) -> float:
        """Compass heading (CW from north) -> world yaw (CCW from +x east)
        in the map frame whose +y points at `map_north_heading_deg`.

        GYRO DRIFT. There is no magnetometer, so the FC's heading is
        gyro-integrated and walks. Measured on d45 (2026-09-20):
        +3.0 deg/min on the floor, +4.0 on a table, same direction both times,
        which is 3 deg of map rotation over a 60 s run and 0.5 m of bearing
        error at 10 m. `heading_drift_dpm`, from `bench drift`, takes the
        measured rate back out. It is a linear correction to a bias that is
        only roughly linear, so it halves the error rather than removing it -
        which is still worth having. Zero = off, and the clock starts at the
        first call."""
        if self.heading_drift_dpm:
            now = time.monotonic()
            if self._drift_t0 is None:
                self._drift_t0 = now
            heading_deg -= self.heading_drift_dpm * (now - self._drift_t0) / 60.0
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
        omega = None
        self.accel_body = None
        if s.imu is not None:
            gx, gy, gz = (math.radians(v) for v in s.imu.gyro)   # FRD deg/s
            omega = np.array([gx, -gy, -gz])                      # -> FLU
            g0 = 9.80665
            self.accel_body = np.array([self.acc_signs[i] * s.imu.acc[i] / self.acc_lsb_per_g * g0
                                        for i in range(3)])       # body FLU specific force, m/s^2
        # vertical speed, ours. The baro correction only counts on a genuinely
        # fresh sample: the bridge polls altitude on a slow rotation and hands
        # back the same object until the next reply, and feeding one reading
        # in as new every tick over-weights it against the accelerometer.
        # clocked off the STATE's timestamp, not the caller's: the bridge
        # stamps s.t each time it gets fresh attitude, so polling estimate()
        # faster than the link runs advances nothing, which is correct.
        dt_v = 0.0 if self._vert_t is None else max(0.0, min(0.05, s.t - self._vert_t))
        self._vert_t = s.t
        fresh = s.altitude is not None and s.altitude is not self._alt_obj
        if s.altitude is not None:
            self._alt_obj = s.altitude
            if s.altitude.vario_mps:
                self.fc_vario_alive = True
        az_w = float((R @ self.accel_body)[2]) - 9.80665 if self.accel_body is not None else 0.0
        _, vz = self.vert.update(dt_v, az_w, alt, fresh)
        self.last_t = s.t
        return StateEstimate(p=np.array([0.0, 0.0, alt]), v=np.array([0.0, 0.0, float(vz)]),
                             R=R, yaw=yaw, omega=omega)
