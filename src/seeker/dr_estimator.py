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
import os

import numpy as np

from raceline.rc_backend import StateEstimate, clamp, rot_from_quat
from seeker import synthetic_camera as cam

G = 9.80665
VZ_FROM_ARM = os.environ.get("AIGP_VZ_FROM_ARM", "1") == "1"   # 0 = the old behaviour: vz_inertial starts at the airborne latch


class VerticalFilter:
    """Altitude and vertical speed from the vertical accel and the barometer.

    Critically damped observer at `w` rad/s: z' = vz + 2w r, vz' = az + w^2 r,
    with a slow accel-bias state so a wrong accel scale does not run away.
    Feed it every tick; the baro correction only applies on fresh samples."""

    def __init__(self, w: float = 3.0, bias_gain: float = 0.5, lift_m: float = 0.25,
                 launch_acc: float = 0.5, bias_max: float = 1.0,
                 vz_margin: float = 2.0, slope_window_s: float = 0.4):
        self.w = w
        self.bias_gain = bias_gain
        self.lift_m = lift_m
        # 0.5 m/s^2. This was 2.0, chosen when takeoff was 1.32 g and the net
        # push was 3.1 m/s^2. Lowering takeoff to 1.12 g on 2026-09-21 made the
        # net 1.14 and silently disabled the whole launch detector: vz stayed
        # pinned at zero, the altitude loop had nothing to damp, and d44
        # climbed away exactly as it had before the detector existed. A
        # threshold tied to one throttle setting is a threshold that breaks
        # when somebody changes the throttle. 0.5 still clears the measured
        # accelerometer residual - 0.02 to 0.15 m/s^2 - by three to twenty
        # times, and it must hold for five consecutive ticks.
        self.launch_acc = launch_acc   # m/s^2 of upward push that means "launch"
        self.bias_max = bias_max       # the accel is measured at rest; a real bias is small
        self.vz_margin = vz_margin     # how far vz may stray from the barometer's own slope
        self.slope_window_s = slope_window_s
        self._t = 0.0
        self._zs = []                  # (t, z_meas) over the slope window
        self.z = None
        self.vz = 0.0
        self.bias = 0.0
        self.airborne = False       # latched by the barometer OR by a real push
        self._lift_n = 0
        self._pend_v = 0.0          # velocity built up since the push started
        self._pend_n = 0

    def update(self, dt: float, az_world: float, z_meas: float | None, fresh: bool = True) -> tuple:
        self._t += dt
        if self.z is None:
            if z_meas is None:
                return 0.0, 0.0
            self.z, self.vz = float(z_meas), 0.0
            return self.z, self.vz
        if not self.airborne:
            # on the ground the accelerometer is not trusted (contact, and the
            # sim reads free fall while settling): first-order baro track, vz 0.
            #
            # BUT the barometer alone is far too slow to notice a launch. It
            # arrives about 10 times a second and needs three samples above
            # lift_m, so it latches roughly 0.3 s after the aircraft leaves the
            # ground - by which time a 1.3 g takeoff is already doing 2.5 m/s -
            # and then vz starts counting from ZERO. d45 flew that on
            # 2026-09-20: vz read +0.60 while the aircraft climbed at 3.5 m/s,
            # the altitude loop's damping term saw no climb at all, held
            # roughly hover throttle, and it coasted to the ceiling.
            #
            # So also watch the accelerometer, which answers every tick, and
            # carry the velocity it implies forward as the seed.
            if az_world > self.launch_acc:
                self._pend_v += az_world * dt
                self._pend_n += 1
            else:
                self._pend_v, self._pend_n = 0.0, 0
            if fresh and z_meas is not None:
                self.z += 2.0 * self.w * (float(z_meas) - self.z) * dt
                self._lift_n = self._lift_n + 1 if z_meas > self.lift_m else 0
            if self._lift_n >= 3 or self._pend_n >= 5:
                self.airborne = True
                self.vz = self._pend_v      # start from the truth, not from zero
            else:
                self.vz = 0.0
                return self.z, self.vz
        self.vz += (az_world - self.bias) * dt
        self.z += self.vz * dt
        if fresh and z_meas is not None:
            r = float(z_meas) - self.z
            self.z += 2.0 * self.w * r * dt
            self.vz += self.w * self.w * r * dt
            # The bias state exists to absorb a small constant accelerometer
            # error. It is NOT a licence to invent metres per second: driven by
            # a large r it winds up at bias_gain * r per second, and at 0.5 that
            # is a fabricated 1 m/s^2 within two seconds. Clamped, because the
            # scale is measured at rest at startup and checked against gravity,
            # so anything beyond bias_max is the filter arguing with itself.
            self.bias = clamp(self.bias - self.bias_gain * r * dt,
                              -self.bias_max, self.bias_max)
        # ANTI-DIVERGENCE. The accelerometer gives the fast, smooth answer and
        # the barometer gives the slow, coarse, UNBIASED one. Integrated accel
        # can run away - d45 reported -15.8 and then +11.4 m/s while being
        # carried by hand, and the altitude loop answered a phantom 5 m/s dive
        # with near-full throttle. The barometer cannot run away, so it gets
        # the final say on the RANGE vz is allowed to be in, while the
        # accelerometer still decides where inside that range.
        vzb = self._baro_slope(z_meas, fresh)
        if vzb is not None:
            self.vz = clamp(self.vz, vzb - self.vz_margin, vzb + self.vz_margin)
        return self.z, self.vz

    def _baro_slope(self, z_meas, fresh: bool):
        """Vertical speed straight from the barometer over the last window.

        Coarse - d45's barometer steps 0.076 m at a time - but it has no
        memory and therefore no way to diverge, which is the only property
        being asked of it here."""
        if fresh and z_meas is not None:
            self._zs.append((self._t, float(z_meas)))
            self._zs = [e for e in self._zs if self._t - e[0] <= self.slope_window_s]
        if len(self._zs) < 3:
            return None
        (t0, z0), (t1, z1) = self._zs[0], self._zs[-1]
        return (z1 - z0) / (t1 - t0) if t1 - t0 > 1e-3 else None


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
        # ACCELEROMETER BIAS, LEARNED ON THE PAD (d43 race_006, 2026-09-22).
        # At rest the FC's specific force after gravity removal was
        # [0.23 0.06 0.02] m/s^2 in the body frame, forward. A vision fix
        # corrects velocity ACROSS the line of sight only, never along the
        # range (race_187), and forward IS the line of sight to the gate
        # ahead - so that bias integrated unopposed: 2.2 m/s of phantom
        # velocity at 10 s, 3.8 at 24 s, in a hover. The tracker's release
        # needs |v| < 1 and the hold's damping term pushed the aircraft 4 m
        # forward against it, into the gate. Mean the residual over the
        # on_ground ticks (hundreds of samples while it waits for the pilot)
        # and subtract it in flight. Body frame: an accelerometer trim error
        # is constant there, whatever the yaw.
        self.bias_body = np.zeros(3)
        self._bias_sum = np.zeros(3)
        self._bias_n = 0
        # BARO-FREE VERTICAL SPEED for the committed run (race day 2, Brian:
        # "we can't rely on baro"). The bias-corrected world vertical
        # acceleration integrated from the last reset - accurate for the few
        # seconds a committed run lasts, and it never sees the barometer.
        self.vz_inertial = 0.0
        self.az_w_last = 0.0
        self.dt_last = 0.0
        self.vz_leak_s = 1.5         # JULIAN ATTEMPT 2 (2026-09-22): the accelerometer over MSP reads ~0.35 m/s^2 low in flight, so a 30 s leak let the integrated speed run to -1.6 m/s in 6 s and the vision trim (frozen at commit) could not follow. At 1.5 s the offset is BOUNDED at bias*1.5 = -0.5 m/s and stationary: the trim removes it and it does not grow through the hold. The fast part (real accelerations) still comes through; the slow part is vision. was 30.0: the inertial vz leaks toward zero with this time constant (5 s left a -0.3 m/s residual after the takeoff punch: the leak ate the climb, not the braking), so a
                                     # residual accelerometer bias of 0.02 m/s^2 settles at 0.1 m/s
                                     # instead of growing without bound; a 2.5 s committed run keeps
                                     # 60 % of the speed it entered with, which is what matters
        self.arm_t_s = None          # sim path: armed from this time (vz_inertial integrates from here)
        self.pad_until_s = 0.0       # sim path: samples before this time are "on the pad" and
                                     # teach bias_body, as the runtime's wait loop does on the aircraft
        # AIGP_ACCEL_BIAS=0 switches the learning off at the command line
        # (race day 2: the one genuinely new behaviour in the set, and Brian's
        # rule is one new thing per flight). Off = the flight-4 estimator.
        self.learn_bias = os.environ.get("AIGP_ACCEL_BIAS", "1") != "0"
        self.hist = []               # (t, p, R) over the last 0.5 s: a detection is compared with
                                     # the estimate at its OWN time and ATTITUDE (the frame is old
                                     # when it arrives). Every append must be all three.
        self.R = np.eye(3)
        self.vert = VerticalFilter()
        self.baro0 = None
        self.last_est = None
        self.events = []             # (x, y, z, heading) crossings in run order
        self.landmarks = None        # unique gates (set_landmarks); event_lm maps each event to one
        self.event_lm = []
        self.next_event = 0          # our own gate count: advances on our own crossings
        self.crossing_lat_m = 1.5    # generous: the estimate is what we have
        self.crossing_late_m = 0.0   # count the crossing this far PAST the plane. Race day 2
                                     # (Brian): never a microsecond early - an early count ends
                                     # the committed-straight run and hands the roll back to
                                     # the estimator while the frame is still around the
                                     # aircraft. The runtime sets 0.75 m under vision: the body
                                     # is ~0.4 m long, the gate 0.26 m deep, the range fix at
                                     # commit +-0.35 m. Half a second late at 1.5 m/s costs nothing.
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
        # DEBRIEF BOOKKEEPING. Read by the logs, never by the estimate. These
        # are the only two things the drone can measure about its own error
        # with NO ground truth, so they are what tuning on the real airframe
        # has to run on:
        #   the offset it believed it had as it went through a gate (the drone
        #   physically fitted through a 1.5 m opening, so reality is asserting
        #   the answer), and
        #   the camera-vs-dead-reckoning disagreement at each fix, summed, so a
        #   log at any rate can difference the sums and get the exact mean over
        #   a leg without catching every fix.
        self.cross_ev = -1         # event index of the last counted crossing
        self.cross_lat = 0.0       # + left of centre, looking along the crossing
        self.cross_dz = 0.0        # + above the opening centre
        self.fix_cross_sum = 0.0   # + the camera says we are left of dead reckoning
        self.fix_along_sum = 0.0   # + the camera says we are nearer the gate
        self.fix_cross_last = 0.0  # this fix only; added to the sums iff it is kept
        self.fix_along_last = 0.0

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
            s0 = (p_prev[0] - gx) * nx + (p_prev[1] - gy) * ny - self.crossing_late_m
            s1 = (p_new[0] - gx) * nx + (p_new[1] - gy) * ny - self.crossing_late_m
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
                self.cross_ev = self.next_event - 1        # debrief: where it
                self.cross_lat = float(lat)                # believed it was in
                self.cross_dz = float(cz - gz)             # the opening
                if clean and self.crossing_fix_gain > 0.0:
                    # we went through the opening: the map says where that is.
                    # Move the estimate across the crossing direction toward the
                    # centre (the blind turn that follows starts from here)
                    self.p[0] += self.crossing_fix_gain * lat * ny
                    self.p[1] -= self.crossing_fix_gain * lat * nx
            else:
                return

    # --- prediction -------------------------------------------------------------
    def integrate(self, t: float, R: np.ndarray, accel_body, z_meas: float | None,
                  baro_fresh: bool = True, on_ground: bool = False, armed: bool = False):
        """One tick: attitude, body specific force, barometric altitude.

        ZERO-VELOCITY UPDATE. `on_ground` says the aircraft is demonstrably not
        moving - sitting on the start line before takeoff - so velocity is held
        at zero and position is frozen instead of integrated.

        Without it the accelerometer's residual bias integrates the whole time
        the drone waits to be armed. Measured on d45 (2026-09-20): 0.15 m/s^2
        of residual after gravity removal, which is 1.9 m of phantom position
        after 5 s on the ground and 7.5 m after 10 s. A gate opening is 1.5 m,
        so the estimate is lost before the aircraft leaves the ground. Sitting
        still for 55 s it reached 7 m/s and 200 m.

        This is the honest behaviour of dead reckoning, not a fault: the camera
        fixes are what bound it in flight. On the ground there are no fixes
        worth having, so we use the one thing we know for free - it is not
        moving."""
        if self.t_prev is None:
            self.t_prev = t
        dt = max(0.0, min(0.05, t - self.t_prev))
        self.t_prev = t
        self.R = R
        if on_ground:
            self.v[0] = self.v[1] = self.v[2] = 0.0
            if accel_body is not None and self.learn_bias:
                a_b = np.asarray(accel_body, dtype=float)
                resid = a_b - R.T @ np.array([0.0, 0.0, G])
                # a spool-up or a bump is not rest: only quiet samples count
                if float(np.linalg.norm(resid)) < 0.6:
                    self._bias_sum += resid
                    self._bias_n += 1
                    self.bias_body = self._bias_sum / self._bias_n
            # THE INERTIAL VERTICAL SPEED INTEGRATES FROM ARMING, NOT FROM THE
            # AIRBORNE LATCH (2026-09-22). The runtime's latch needs 0.30 m of
            # height AND a 0.5 m/s baro climb before on_ground clears, so on
            # the aircraft the first half metre per second of climb was never
            # integrated: vz_inertial started about 0.5 m/s LOW and leaked
            # toward zero over 30 s. The vertical loop damps on it, so it flew
            # the approach ~0.2 m high (flight 1 "slightly too high") and the
            # commit hold then answered the phantom descent with a climb
            # (flight 2, the top bar). Once armed the props are the only thing
            # that can move the airframe, so integrate. The guard skips the
            # sim's spool-up artefact (-7 m/s^2 on the ground at t=3.1): a
            # real airframe on the ground cannot see 4 m/s^2 vertical.
            if armed and VZ_FROM_ARM and accel_body is not None:
                a_w = (R @ (np.asarray(accel_body, dtype=float) - self.bias_body)
                       - np.array([0.0, 0.0, G]))
                if abs(float(a_w[2])) < 4.0:
                    self.vz_inertial += float(a_w[2]) * dt
                    if self.vz_leak_s > 0:
                        self.vz_inertial -= self.vz_inertial * dt / self.vz_leak_s
                    self.az_w_last = float(a_w[2])
                    self.dt_last = dt
            z, vz = self.vert.update(dt, 0.0, z_meas, baro_fresh)
            self.p[2] = z
            # Same shape as the flying branch below - (t, p, R). state_at()
            # reads the attitude out of element 2, so a two-element entry from
            # here crashes the first time a detection arrives while the
            # aircraft is still on the ground, which is exactly when a bench
            # residual check runs.
            self.hist.append((t, self.p.copy(), R.copy()))
            while len(self.hist) > 1 and self.hist[0][0] < t - 0.5:
                self.hist.pop(0)
            return
        a_w = (R @ (np.asarray(accel_body, dtype=float) - self.bias_body)
               - np.array([0.0, 0.0, G]))
        self.v[0] += a_w[0] * dt
        self.v[1] += a_w[1] * dt
        self.vz_inertial += float(a_w[2]) * dt
        self.az_w_last = float(a_w[2])                    # diagnostics: what was integrated
        self.dt_last = dt
        if self.vz_leak_s > 0:
            self.vz_inertial -= self.vz_inertial * dt / self.vz_leak_s
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

    def reset_vz_inertial(self):
        """Start the baro-free vertical speed from zero: the aircraft was
        holding level when this is called (at commit)."""
        self.vz_inertial = 0.0

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
        self.integrate(float(u.t), R, u.accel, z_meas, u.baro_fresh,
                       on_ground=(float(u.t) < self.pad_until_s),
                       armed=(self.arm_t_s is not None and float(u.t) >= self.arm_t_s))
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
        # the same split, signed and unweighted, for the debrief: a bearing
        # disagreement that is always one sign is the camera's boresight, a
        # range disagreement that is always one sign is its scale
        self.fix_cross_last = float(-r_full[0] * d_h[1] + r_full[1] * d_h[0])
        self.fix_along_last = along
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
                # LATERAL ONLY, STILL. The along-LOS velocity has no
                # observation here, and d43 race_006/007 (2026-09-22) showed
                # what that costs: the accelerometer's residual bias points
                # along the LOS when the aircraft faces the gate, 0.1 m/s^2
                # integrated to 3.7 m/s in a 30 s hover. The fix for that is
                # bias_body (learned on the pad), NOT a velocity trim from the
                # range: a range flip like race_006's 2.4 -> 6.6 m held for
                # 0.8 s would have kicked v by ~2 m/s through such a trim,
                # and kd_pos = 4 turns that into an 8 m/s^2 lunge. Tried,
                # then taken out before race day 2: unflown dynamics.
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
        self.fix_cross_sum += self.fix_cross_last     # debrief: kept fixes only
        self.fix_along_sum += self.fix_along_last
        self.last_landmark = i
        return i, r
