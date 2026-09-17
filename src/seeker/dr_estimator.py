"""
Vision-aided dead reckoning: the state source that lets the follower fly a
plan with no position sensor. One code path for the sim and the Orin.

    src = DeadReckonSource()
    src.integrate(t, R, accel_body, z_meas)    # every tick: attitude, raw accel, baro
    src.observe(det, landmarks)                # every detection: which gate, then a fix
    est = src.estimate(update)                 # sim only: wraps integrate on a SensorUpdate

Inputs, all of which the flight controller and the camera provide:
    R           body FLU -> world rotation (FC attitude, heading in the map frame)
    accel_body  specific force in body FLU, m/s^2 (FC raw accel, scaled)
    z_meas      barometric altitude above the takeoff point (FC baro)
    det         a Detection: image offsets, apparent size, range from size

Horizontal position is integrated from the accelerometer rotated by the
attitude (specific force minus gravity). Altitude and vertical speed come
from a complementary filter of the vertical accel and the barometer
(VerticalFilter), never from the sim's truth. A detection is matched to
the map gate whose predicted bearing from the current estimate is
closest (associate); the fix is the gate's map position minus the range
along the observed bearing, blended in across the line of sight at
`fix_gain` and along it at `fix_gain * along_weight` because the range is
the least trusted number. A fix that implies an implausible jump is dropped.
The estimator counts its own gate crossings.
"""

from __future__ import annotations

import math

import numpy as np

from raceline.rc_backend import StateEstimate, rot_from_quat
from seeker import synthetic_camera as cam

G = 9.80665


class VerticalFilter:
    """Altitude and vertical speed from the vertical accel and the barometer.

    Critically damped observer at `w` rad/s: z' = vz + 2w r, vz' = az + w^2 r,
    with a slow accel-bias state so a wrong accel scale does not run away.
    Feed it every tick; the baro correction only applies on fresh samples."""

    def __init__(self, w: float = 3.0, bias_gain: float = 0.5, lift_m: float = 0.25):
        self.w = w
        self.bias_gain = bias_gain
        self.lift_m = lift_m
        self.z = None
        self.vz = 0.0
        self.bias = 0.0
        self.airborne = False       # latched once the baro has read above lift_m for 3 samples
        self._lift_n = 0

    def update(self, dt: float, az_world: float, z_meas: float | None, fresh: bool = True) -> tuple:
        if self.z is None:
            if z_meas is None:
                return 0.0, 0.0
            self.z, self.vz = float(z_meas), 0.0
            return self.z, self.vz
        if not self.airborne:
            # on the ground the accelerometer is not trusted (contact, and the
            # sim reads free fall while settling): first-order baro track, vz 0
            if fresh and z_meas is not None:
                self.z += 2.0 * self.w * (float(z_meas) - self.z) * dt
                self._lift_n = self._lift_n + 1 if z_meas > self.lift_m else 0
                if self._lift_n >= 3:
                    self.airborne = True
            self.vz = 0.0
            return self.z, self.vz
        self.vz += (az_world - self.bias) * dt
        self.z += self.vz * dt
        if fresh and z_meas is not None:
            r = float(z_meas) - self.z
            self.z += 2.0 * self.w * r * dt
            self.vz += self.w * self.w * r * dt
            self.bias -= self.bias_gain * r * dt
        return self.z, self.vz


class DeadReckonSource:
    def __init__(self, fix_gain: float = 0.6, vel_gain: float = 0.005, v_decay_s: float = 120.0,
                 start_xy=(0.0, 0.0), along_weight: float = 0.17, reject_m: float = 4.0):
        # fix_gain acts across the line of sight (the bearing, precise);
        # along it the gain is fix_gain * along_weight (the range, +-10 %):
        # race_179 clipped g5 because g4's range errors became g5's lateral error
        self.fix_gain = fix_gain
        self.vel_gain = vel_gain
        self.v_decay_s = v_decay_s
        self.along_weight = along_weight
        self.reject_m = reject_m
        self.p = np.array([start_xy[0], start_xy[1], 0.0], dtype=float)
        self.v = np.zeros(3)
        self.t_prev = None
        self.t_last_fix = None
        self.fixes = 0
        self.rejected = 0
        self.unmatched = 0
        self.fix_residual = 0.0
        self.last_landmark = None
        self.last_reason = ""        # why the last detection was not used (diagnostics)
        self.last_full_residual = 0.0  # |p_fix - p| before any weighting (a misassociation shows here)
        self.max_dv_per_fix = 0.3    # m/s: one fix may not rewrite the velocity
        self.hist = []               # (t, p) over the last 0.5 s: a detection is compared with the
                                     # estimate at its OWN time (the frame is old when it arrives)
        self.R = np.eye(3)
        self.vert = VerticalFilter()
        self.baro0 = None
        self.last_est = None
        self.events = []             # (x, y, z, heading) crossings in run order
        self.landmarks = None        # unique gates (set_landmarks); event_lm maps each event to one
        self.event_lm = []
        self.next_event = 0          # our own gate count: advances on our own crossings
        self.crossing_lat_m = 1.5    # generous: the estimate is what we have
        self.crossing_z_m = 1.2
        self.miss_lat_m = 4.0        # crossed the plane this far off centre: a miss, but the gate is behind us
        self.crossing_fix_gain = 0.6  # a counted crossing is a position fix: the drone was inside the opening,
                                      # so pull the lateral estimate toward the gate centre (no camera needed)
        # association: how far the estimate may be wrong, growing with time since a fix
        self.assoc_sigma_m = 1.5
        self.assoc_sigma_rate = 0.5      # m per second without a fix
        self.assoc_min_rad = math.radians(6.0)
        self.assoc_max_rad = math.radians(40.0)
        self.max_fix_range_m = 15.0      # farther gates are not fixed on: a 6 deg tolerance at 20 m is a
                                         # 2 m jump when the wrong one matches (race_238, lap 1 hairpin)

    def set_events(self, events):
        self.events = [(float(x), float(y), float(z), None if h is None else float(h)) for x, y, z, h in events]
        self.next_event = 0
        self.landmarks = None
        self.event_lm = []

    def set_landmarks(self, landmarks):
        """The unique gates, so the map can say which ones can be in view:
        the gate just passed and the next one. Everything else is refused
        whatever it looks like (we know the course)."""
        self.landmarks = list(landmarks)
        key = lambda x, y, z: (round(x, 2), round(y, 2), round(z, 2))
        lm_of = {key(*lm[:3]): i for i, lm in enumerate(self.landmarks)}
        self.event_lm = [lm_of.get(key(e[0], e[1], e[2])) for e in self.events]

    def allowed_now(self):
        if not self.event_lm:
            return None
        # the gate just passed and the next one. NOT the one after: it is far,
        # its fix is weak, and on the approach to gate 5 a sighting of gate 6
        # through it pulled the estimate 1.2 m off (race_261)
        lo, hi = max(0, self.next_event - 1), min(len(self.event_lm), self.next_event + 1)
        return {i for i in self.event_lm[lo:hi] if i is not None}

    def _count_crossings(self, p_prev, p_new):
        while self.next_event < len(self.events):
            gx, gy, gz, gh = self.events[self.next_event]
            if gh is None:
                return
            nx, ny = math.cos(gh), math.sin(gh)
            s0 = (p_prev[0] - gx) * nx + (p_prev[1] - gy) * ny
            s1 = (p_new[0] - gx) * nx + (p_new[1] - gy) * ny
            if not (s0 < 0.0 <= s1):
                return
            f = s0 / (s0 - s1)
            cx = p_prev[0] + (p_new[0] - p_prev[0]) * f
            cy = p_prev[1] + (p_new[1] - p_prev[1]) * f
            cz = p_prev[2] + (p_new[2] - p_prev[2]) * f
            lat = -(cx - gx) * ny + (cy - gy) * nx
            clean = abs(lat) <= self.crossing_lat_m and abs(cz - gz) <= self.crossing_z_m
            # a plane crossing near the gate advances the count even when the
            # estimate says we missed: staying on a gate that is behind us
            # (race_177: circling g4 until the g5 frame) is worse than moving on
            if clean or abs(lat) <= self.miss_lat_m:
                self.next_event += 1
                if clean and self.crossing_fix_gain > 0.0:
                    # we went through the opening: the map says where that is.
                    # Move the estimate across the crossing direction toward the
                    # centre (the blind turn that follows starts from here)
                    self.p[0] += self.crossing_fix_gain * lat * ny
                    self.p[1] -= self.crossing_fix_gain * lat * nx
            else:
                return

    # --- prediction -------------------------------------------------------------
    def integrate(self, t: float, R: np.ndarray, accel_body, z_meas: float | None, baro_fresh: bool = True):
        """One tick: attitude, body specific force, barometric altitude."""
        if self.t_prev is None:
            self.t_prev = t
        dt = max(0.0, min(0.05, t - self.t_prev))
        self.t_prev = t
        self.R = R
        a_w = R @ np.asarray(accel_body, dtype=float) - np.array([0.0, 0.0, G])
        self.v[0] += a_w[0] * dt
        self.v[1] += a_w[1] * dt
        if self.v_decay_s > 0:                          # bounded drift when no fixes arrive
            self.v[0] -= self.v[0] * dt / self.v_decay_s
            self.v[1] -= self.v[1] * dt / self.v_decay_s
        p_prev = self.p.copy()
        self.p[0] += self.v[0] * dt
        self.p[1] += self.v[1] * dt
        z, vz = self.vert.update(dt, float(a_w[2]), z_meas, baro_fresh)
        self.p[2] = z
        self.v[2] = vz
        self.hist.append((t, self.p.copy(), R.copy()))
        while len(self.hist) > 1 and self.hist[0][0] < t - 0.5:
            self.hist.pop(0)
        if self.events:
            self._count_crossings(p_prev, self.p)

    def p_at(self, t_det):
        """The estimate at the detection's time (nearest sample not later than it)."""
        return self.state_at(t_det)[0]

    def state_at(self, t_det):
        """(position, attitude) at the detection's time. The attitude matters
        as much as the position: at 100 deg/s of yaw a frame one period old
        is 3 deg, which is 0.45 m across the line of sight at 8 m, and every
        fix through a turn leaned the same way (race_345 era, stack top)."""
        if t_det is None or not self.hist:
            return self.p.copy(), self.R
        best = self.hist[0]
        for h in self.hist:
            if h[0] <= t_det:
                best = h
            else:
                break
        return best[1].copy(), best[2]

    def estimate(self, u) -> StateEstimate:
        """Sim SensorUpdate -> StateEstimate. Attitude from the quaternion (the
        FC's attitude), accel from the IMU packet, altitude from the sim's
        BAROMETER zeroed at the first sample, never the true position."""
        R = rot_from_quat(u.world_pos[0:4])
        z_meas = None
        if u.baro_fresh:
            if self.baro0 is None:
                self.baro0 = float(u.baro)
            z_meas = float(u.baro) - self.baro0
        self.integrate(float(u.t), R, u.accel, z_meas, u.baro_fresh)
        omega_w = np.asarray(u.world_vel[0:3], dtype=float)
        yaw = math.atan2(R[1, 0], R[0, 0])
        self.last_est = StateEstimate(p=self.p.copy(), v=self.v.copy(), R=R, yaw=yaw, omega=R.T @ omega_w)
        return self.last_est

    # --- correction -----------------------------------------------------------------
    def associate(self, det, landmarks, allowed=None):
        """Which map gate is this detection? The one whose predicted bearing
        from the current estimate is closest to the observed one, inside a
        tolerance that grows with the time since the last fix, with the
        measured range (if any) consistent with the predicted one. None when
        nothing matches or two gates match about equally."""
        return self.associate_scored(det, landmarks, allowed)[0]

    def associate_scored(self, det, landmarks, allowed=None):
        """associate() plus its score (angle error over tolerance, lower is
        better), so several blobs from one frame can be compared."""
        d_obs = cam.direction_body(det)
        if getattr(det, "range_m", None) and det.range_m > 1.2 * self.max_fix_range_m:
            self.last_reason = "far"
            return None, None
        dt_fix = 0.0 if (self.t_last_fix is None or self.t_prev is None) else max(0.0, self.t_prev - self.t_last_fix)
        sigma = self.assoc_sigma_m + self.assoc_sigma_rate * min(dt_fix, 20.0)
        cands = []
        best_ang, best_tol, rng_fail = None, None, False
        if allowed is None:
            allowed = self.allowed_now()
        p_ref, R_ref = self.state_at(getattr(det, "t", None))   # predict from where we were at the frame's time
        for i, lm in enumerate(landmarks):
            if allowed is not None and i not in allowed:
                continue                                    # the map says this gate cannot be the one in view
            d = np.array([lm[0] - p_ref[0], lm[1] - p_ref[1], lm[2] - p_ref[2]])
            rng_pred = float(np.linalg.norm(d))
            if rng_pred < 0.4 or rng_pred > self.max_fix_range_m:
                continue
            d_b = R_ref.T @ (d / rng_pred)
            if d_b[0] <= 0.0:
                continue                                    # behind the drone
            ang = math.acos(max(-1.0, min(1.0, float(np.dot(d_b, d_obs)))))
            tol = min(self.assoc_max_rad, max(self.assoc_min_rad, math.atan2(sigma, rng_pred)))
            if best_ang is None or ang < best_ang:
                best_ang, best_tol = ang, tol
            if ang > tol:
                continue
            rng = getattr(det, "range_m", None)
            # range from ring size is good to ~10 %: a sighting whose range does
            # not fit is another gate on the same line (race_196: the finish
            # gate matched the one 10 m behind it and the estimate ran 9 m ahead)
            if rng and abs(rng - rng_pred) > 1.0 + 0.25 * rng_pred:
                rng_fail = True
                continue
            cands.append((ang / tol, i))
        if not cands:
            self.last_reason = ("range" if rng_fail else
                                f"angle {math.degrees(best_ang):.0f}>{math.degrees(best_tol):.0f}" if best_ang is not None else "none ahead")
            return None, None
        cands.sort()
        if len(cands) > 1 and cands[1][0] - cands[0][0] < 0.25:
            self.last_reason = "ambiguous"
            return None, None
        self.last_reason = ""
        return cands[0][1], cands[0][0]

    def observe_any(self, dets, landmarks, allowed=None):
        """Several blobs from one frame (biggest first): fix on the one that
        matches a map gate best. The biggest blob is not always a gate, and
        in a hairpin the gate in view is not the next one. Returns
        (landmark index or None, residual m)."""
        best = None
        for det in dets or ():
            i, score = self.associate_scored(det, landmarks, allowed)
            if i is not None and (best is None or score < best[0]):
                best = (score, det)
        if best is None:
            self.unmatched += 1
            self.last_landmark = None
            return None, 0.0
        return self.observe(best[1], landmarks, allowed)

    def apply_fix(self, det, gate_xyz) -> float:
        """Position fix from a sighting of the gate at gate_xyz. Returns the
        residual (m) between the dead-reckoned and the fixed position."""
        p_ref, R_ref = self.state_at(getattr(det, "t", None))   # where and how we were pointed when the frame was taken
        d_w = R_ref @ cam.direction_body(det)
        d_h = np.array([d_w[0], d_w[1]])
        n = np.linalg.norm(d_h)
        if n < 1e-6:
            return 0.0
        d_h /= n
        rng = getattr(det, "range_m", None)
        if not rng:
            # bearing only: the fix sits at the predicted range, so it only
            # moves the estimate across the line of sight
            rng = float(np.hypot(gate_xyz[0] - self.p[0], gate_xyz[1] - self.p[1]))
        p_fix = np.array([gate_xyz[0], gate_xyz[1]]) - rng * d_h
        r_full = p_fix - p_ref[:2]
        self.last_full_residual = float(np.hypot(r_full[0], r_full[1]))
        along = float(np.dot(r_full, d_h))
        r_lat = r_full - along * d_h                       # across the line of sight: the bearing, precise
        # far sightings are worth less: the bearing error grows with range, and
        # the range (ring size) is only worth using up close: full weight to
        # 8 m, none beyond 20 m (race_188: 30 m sightings across the field
        # carried 3 m of range noise into the position and the velocity)
        w_lat = 1.0 / (1.0 + (rng / 12.0) ** 2)
        w_rng = max(0.0, min(1.0, (20.0 - rng) / 12.0))
        r_lat = w_lat * r_lat
        r = r_lat + self.along_weight * w_rng * along * d_h
        p_before = self.p.copy()
        self.p[0] += self.fix_gain * r[0]
        self.p[1] += self.fix_gain * r[1]
        if self.events:                                   # a fix can carry the estimate across a gate plane too
            self._count_crossings(p_before, self.p)
        # the lateral residual is also a velocity error: v += beta r_lat / dt
        # (alpha-beta). At 30 Hz fixes beta = vel_gain; after a gap the residual
        # is mostly velocity error times the gap, so absorb half of it. Capped
        # per fix, and never from the range component (race_184: the old nudge
        # was 100x too weak; race_187: uncapped, one bad fix rewrote v by 3 m/s)
        if self.t_last_fix is not None and self.t_prev is not None:
            dt = self.t_prev - self.t_last_fix
            if 0.0 < dt < 3.0:
                k = min(0.5 / dt, self.vel_gain * 30.0)
                dv = k * r_lat
                n = float(np.hypot(dv[0], dv[1]))
                if n > self.max_dv_per_fix:
                    dv *= self.max_dv_per_fix / n
                self.v[0] += dv[0]
                self.v[1] += dv[1]
        self.t_last_fix = self.t_prev
        self.fixes += 1
        self.fix_residual = float(np.hypot(r[0], r[1]))
        return self.fix_residual

    def observe(self, det, landmarks, allowed=None):
        """A detection of an unknown gate: associate it, then fix on it.
        Returns (landmark index or None, residual m). A fix whose residual
        exceeds reject_m is undone and counted as rejected."""
        i = self.associate(det, landmarks, allowed)
        if i is None:
            self.unmatched += 1
            self.last_landmark = None
            return None, 0.0
        p_before, v_before, t_before, ev_before = self.p[:2].copy(), self.v[:2].copy(), self.t_last_fix, self.next_event
        r = self.apply_fix(det, landmarks[i][:3])
        if r > self.reject_m or self.last_full_residual > 2.0 * self.reject_m:
            self.p[:2] = p_before
            self.v[:2] = v_before
            self.t_last_fix = t_before
            self.next_event = ev_before
            self.fixes -= 1
            self.rejected += 1
            self.last_landmark = None
            return None, r
        self.last_landmark = i
        return i, r
