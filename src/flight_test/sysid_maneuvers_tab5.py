"""
Tab 5 maneuvers: closed-loop point-tracking and station-hold.

Tabs 1-4 are OPEN-LOOP (or attitude-hold) probes - they characterise the airframe's raw
response. Tab 5 is the missing CLOSED-LOOP piece: a full outer position loop on world-frame
ODOMETRY (live ground truth in Round 1), with no vision or gate involved. It answers the one
question the open-loop data can't: "accelerate to X m/s, then stop dead at a known point - how
far does it carry, and how long does it take to lock on?"

Both maneuvers run a cascade exactly like the pilots': an outer P/PD loop on position produces a
DESIRED lean (roll/pitch), and the same attitude P-loop used everywhere else (KP_ATT with the
measured ROLL_SIGN/PITCH_SIGN) turns that lean into a body rate. Altitude is held with thrust
proportional to altitude error on top of the tilt-compensated hover thrust.

Everything is expressed in the SPAWN-HEADING frame captured at t=0: `fwd` is the world-x axis at
the spawn yaw, `lat` is its left perpendicular. The drone is never yaw-commanded, so body-lateral
stays aligned with world-lateral and a constant roll sign maps cleanly onto the lateral axis.

The conservative starting gains (kp_fwd=0.3, kd_fwd=0.8, kp_lat=0.3, kp_alt=0.5) respect the two
hard facts from the existing sysid: the 96 ms rotational lag (so attitude settles in ~150-200 ms
and the position loop must not out-run it) and the weak passive drag (k=0.0343 -> only 0.86 m/s^2
braking at 5 m/s, so active nose-up pitch carries the stop). They will almost certainly need
tuning; this just gives a stable floor to tune from.
"""

import math

from common.dynamics import (HOVER_THRUST, KP_ATT, MAX_RATE, PITCH_SIGN, ROLL_SIGN, clamp,
                              lean_for_speed)
from flight_test.sysid_maneuvers import ang_err

# Lean clamps for the outer loop. 0.6 rad (~34 deg) is a touch past the 0.524 rad / ~9 m/s lean
# at the top of SPEED_LEAN_TABLE, leaving the brake a little more nose-up authority than cruise.
MAX_LEAN = 0.6          # rad, on the commanded pitch/roll from the position loop
HOLD_LEAN = 0.4         # rad, tighter clamp for the station-hold loop (it never needs to cruise)


def _spawn_frame(st, origin, psi0):
    """Forward/lateral position and velocity in the spawn-heading frame.

    Returns (fwd_pos, lat_pos, v_fwd, v_lat): position relative to `origin` and world velocity,
    both projected onto the spawn-heading forward axis and its left perpendicular."""
    dx = st["x"] - origin[0]
    dy = st["y"] - origin[1]
    c, s = math.cos(psi0), math.sin(psi0)
    fwd_pos = dx * c + dy * s
    lat_pos = -dx * s + dy * c
    v_fwd = st["vx_w"] * c + st["vy_w"] * s
    v_lat = -st["vx_w"] * s + st["vy_w"] * c
    return fwd_pos, lat_pos, v_fwd, v_lat


class PointApproach:
    """Accelerate to an entry speed, then close the loop and stop dead at a world-frame point.

    Phase 'accelerate': hold a forward lean (sized from SPEED_LEAN_TABLE) to spin up to
    `entry_speed`, keeping altitude. Phase 'home': pure PD on world-frame position - commanded
    pitch proportional to forward position error minus a velocity-damping term (so it noses UP to
    brake once it is past the target), commanded roll proportional to lateral error, thrust
    proportional to altitude error. Ends when |position_error| < 0.5 m AND |vh| < 0.5 m/s, or on
    the home timeout.

    Exposes self.metrics after completion (entry_speed_actual, peak_overshoot_m, settling_time_s,
    final_error_m, min_stopping_distance_m). settling_time_s is measured from loop closure (the
    start of the home phase), not from t=0, so it is the closed-loop response and not polluted by
    the accelerate ramp."""

    def __init__(self, target_dist, entry_speed, kp_fwd, kd_fwd, kp_lat, kp_alt,
                 hover_thrust=HOVER_THRUST, accel_timeout=6.0, home_timeout=10.0):
        self.target_dist = target_dist
        self.entry_speed = entry_speed
        self.kp_fwd = kp_fwd
        self.kd_fwd = kd_fwd
        self.kp_lat = kp_lat
        self.kp_alt = kp_alt
        self.hover = hover_thrust
        self.accel_timeout = accel_timeout
        self.home_timeout = home_timeout
        self.lean = lean_for_speed(entry_speed)     # forward lean to reach entry_speed
        self.phase = "accelerate"
        self._origin = None                          # (x, y) at t=0
        self._alt0 = None
        self._psi0 = None
        self._home_t0 = None
        self.metrics = {
            "entry_speed_actual": None, "peak_overshoot_m": 0.0, "settling_time_s": None,
            "final_error_m": None, "min_stopping_distance_m": None,
        }

    def step(self, t, st):
        if self._origin is None:
            self._origin = (st["x"], st["y"])
            self._alt0 = st["alt"]
            self._psi0 = st["yaw"]

        fwd_pos, lat_pos, v_fwd, v_lat = _spawn_frame(st, self._origin, self._psi0)
        err_fwd = self.target_dist - fwd_pos        # +ve = still short of the target -> nose down
        err_lat = -lat_pos                          # target lateral is 0
        err_alt = self._alt0 - st["alt"]
        pos_err = math.sqrt(err_fwd ** 2 + err_lat ** 2 + err_alt ** 2)

        # closest approach to the target over the whole run (the "min stopping distance")
        m = self.metrics
        if m["min_stopping_distance_m"] is None or pos_err < m["min_stopping_distance_m"]:
            m["min_stopping_distance_m"] = pos_err

        if self.phase == "accelerate":
            thrust = clamp(self.hover / max(math.cos(self.lean), 0.5), 0.0, 1.0)
            if v_fwd >= self.entry_speed or t > self.accel_timeout:
                self.phase = "home"
                self._home_t0 = t
                m["entry_speed_actual"] = v_fwd
            else:
                rr = clamp(ROLL_SIGN * KP_ATT * ang_err(0.0, st["roll"]), -MAX_RATE, MAX_RATE)
                pr = clamp(PITCH_SIGN * KP_ATT * ang_err(self.lean, st["pitch"]), -MAX_RATE, MAX_RATE)
                return (rr, pr, 0.0, thrust,
                        {"phase": "accelerate", "err_fwd": err_fwd, "err_lat": err_lat,
                         "err_alt": err_alt})

        # home: PD on world-frame position -> desired lean -> attitude P-loop -> body rate
        des_pitch = clamp(self.kp_fwd * err_fwd - self.kd_fwd * v_fwd, -MAX_LEAN, MAX_LEAN)
        des_roll = clamp(self.kp_lat * err_lat, -MAX_LEAN, MAX_LEAN)
        thrust = clamp(self.hover / max(math.cos(des_roll) * math.cos(des_pitch), 0.4)
                       + self.kp_alt * err_alt, 0.0, 1.0)
        rr = clamp(ROLL_SIGN * KP_ATT * ang_err(des_roll, st["roll"]), -MAX_RATE, MAX_RATE)
        pr = clamp(PITCH_SIGN * KP_ATT * ang_err(des_pitch, st["pitch"]), -MAX_RATE, MAX_RATE)

        overshoot = max(0.0, fwd_pos - self.target_dist)
        if overshoot > m["peak_overshoot_m"]:
            m["peak_overshoot_m"] = overshoot

        if pos_err < 0.5 and st["vh"] < 0.5:
            m["settling_time_s"] = t - self._home_t0
            m["final_error_m"] = pos_err
            return None
        if t - self._home_t0 > self.home_timeout:
            m["final_error_m"] = pos_err            # settling_time_s stays None = did not converge
            return None

        return (rr, pr, 0.0, thrust,
                {"phase": "home", "err_fwd": err_fwd, "err_lat": err_lat, "err_alt": err_alt})


class StationHold:
    """Hold the t=0 position for a fixed duration - the closed-loop hover baseline.

    Same cascade as PointApproach but with target = origin and no acceleration phase. Establishes
    how stable the position loop is before any motion is asked of it, so the point-approach drift
    can be read against a known floor. Exposes self.metrics (max_drift_m, rms_error_m,
    altitude_drift_m) after completion."""

    def __init__(self, duration_s, kp_fwd=0.3, kp_lat=0.3, kp_alt=0.5, hover_thrust=HOVER_THRUST):
        self.duration_s = duration_s
        self.kp_fwd = kp_fwd
        self.kp_lat = kp_lat
        self.kp_alt = kp_alt
        self.hover = hover_thrust
        self._origin = None
        self._alt0 = None
        self._psi0 = None
        self._sq_sum = 0.0
        self._n = 0
        self.metrics = {"max_drift_m": 0.0, "rms_error_m": None, "altitude_drift_m": 0.0}

    def step(self, t, st):
        if self._origin is None:
            self._origin = (st["x"], st["y"])
            self._alt0 = st["alt"]
            self._psi0 = st["yaw"]

        if t > self.duration_s:
            self.metrics["rms_error_m"] = math.sqrt(self._sq_sum / self._n) if self._n else None
            return None

        fwd_pos, lat_pos, v_fwd, v_lat = _spawn_frame(st, self._origin, self._psi0)
        err_fwd = -fwd_pos                          # drive position back to the origin
        err_lat = -lat_pos
        err_alt = self._alt0 - st["alt"]
        drift = math.hypot(fwd_pos, lat_pos)

        m = self.metrics
        m["max_drift_m"] = max(m["max_drift_m"], drift)
        m["altitude_drift_m"] = max(m["altitude_drift_m"], abs(err_alt))
        self._sq_sum += drift ** 2
        self._n += 1

        des_pitch = clamp(self.kp_fwd * err_fwd, -HOLD_LEAN, HOLD_LEAN)
        des_roll = clamp(self.kp_lat * err_lat, -HOLD_LEAN, HOLD_LEAN)
        thrust = clamp(self.hover / max(math.cos(des_roll) * math.cos(des_pitch), 0.5)
                       + self.kp_alt * err_alt, 0.0, 1.0)
        rr = clamp(ROLL_SIGN * KP_ATT * ang_err(des_roll, st["roll"]), -MAX_RATE, MAX_RATE)
        pr = clamp(PITCH_SIGN * KP_ATT * ang_err(des_pitch, st["pitch"]), -MAX_RATE, MAX_RATE)
        return (rr, pr, 0.0, thrust,
                {"phase": "hold", "err_fwd": err_fwd, "err_lat": err_lat, "err_alt": err_alt})
