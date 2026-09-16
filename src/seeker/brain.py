"""
Gate-seeker brain: a state machine that flies gate to gate with

    - the published map    (which gate is next, its heading, its height,
                            and roughly how far it is)
    - a heading            (the FC's compass yaw, in the map frame)
    - a barometric altitude and vertical speed
    - a detection          (where the next gate sits in the camera image,
                            and how big it looks), when one is available

and NOTHING else. It never knows where it is. It outputs, every tick:

    a_des_world  desired horizontal acceleration in the map frame
                 (rc_backend.angle_sticks turns it into tilt angles)
    z_target     altitude to hold (metres above the takeoff point)
    yaw_target   heading to hold (world yaw, CCW from +x)
    phase        for logging, and `done` when the run is over

Speed is open-loop: a fixed forward tilt gives a terminal cruise speed
against drag (8 deg -> roughly 2.3 m/s on this airframe). Slow is the
design: the gate must stay in the picture.

Each crossing k is reached by a LEG planned from the map:

    direct   the gate lies roughly ahead along its own normal:
             SEEK (creep on the leg heading, look) -> TRACK -> COMMIT
    dogleg   it does not (the hairpin g5, the lap close): fly a timed
             dead-reckoned TRANSIT to a point in front of the gate, STOP,
             TURN onto the gate's direction, HOLD and scan for it, then
             SEEK -> TRACK -> COMMIT

Any heading change larger than `turn_in_place_rad` is done from a
standstill (STOP, then TURN): carrying speed through a turn is what
breaks dead reckoning. The stacked gate is a dogleg whose transit length
is zero: COMMIT, STOP, TURN 180, change height, HOLD, SEEK. After the
last crossing: LAND.

Losing the gate in TRACK for longer than `lost_s` falls back to SEEK; a
SEEK that finds nothing within the leg's expected time holds position
and scans the heading.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

G = 9.81


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Detection:
    offset_x: float       # -1 left .. +1 right of the image centre
    offset_y: float       # -1 top .. +1 bottom
    area_frac: float      # ring area as a fraction of the frame
    t: float              # time the frame was taken


@dataclass
class Crossing:
    label: str
    x: float
    y: float
    z: float
    heading_rad: float    # required travel direction through the opening


@dataclass
class SeekerConfig:
    cruise_tilt_deg: float = 8.0       # open-loop forward tilt while seeking/tracking/transit
    commit_tilt_deg: float = 8.0       # through the opening
    stop_tilt_deg: float = 10.0        # braking tilt (against the direction of travel)
    v_creep_est_mps: float = 2.3       # cruise speed the tilt settles at (dead reckoning)
    speed_tau_s: float = 1.8           # first-order speed response toward cruise
    k_yaw: float = 0.6                 # yaw nudge (rad) per unit offset_x while tracking
    k_lat: float = 2.5                 # lateral accel (m/s^2) per unit offset_x
    commit_area_frac: float = 0.12     # ring this big = about to cross, camera can no longer aim
    commit_s: float = 1.6              # time to fly straight through after committing
    past_gate_m: float = 1.5           # where the drone is after COMMIT: this far past the opening
    approach_m: float = 5.0            # the point in front of a gate a dogleg aims for
    direct_angle_rad: float = 1.05     # leg within this of the gate normal -> direct (60 deg)
    turn_in_place_rad: float = 0.5     # heading change above this -> stop first, then turn
    lost_s: float = 0.5                # detection gap in TRACK before falling back to SEEK
    seek_timeout_scale: float = 2.0    # SEEK gives up after leg_length / v_creep * this
    scan_amplitude_rad: float = 0.6    # +-heading scan while holding
    scan_period_s: float = 4.0
    scan_before_seek_s: float = 4.0    # after a turn: scan this long before creeping
    takeoff_z_tol_m: float = 0.25
    takeoff_vz_tol_mps: float = 0.4
    turn_settle_rad: float = 0.15
    stop_v_mps: float = 0.15           # modelled speed at which a STOP is done
    stop_min_s: float = 0.4
    land_rate_mps: float = 0.6
    min_track_area_frac: float = 0.0004


@dataclass
class Command:
    a_des: tuple                       # (ax, ay) world frame, m/s^2
    z_target: float
    yaw_target: float
    phase: str
    crossing: int
    arm: bool = True
    done: bool = False
    note: str = ""


class SeekerBrain:
    def __init__(self, crossings: Sequence[Crossing], cfg: SeekerConfig = SeekerConfig(),
                 start_xy: tuple = (0.0, 0.0), laps: int = 2):
        self.cfg = cfg
        per_lap = list(crossings)
        seq = [per_lap[0]]
        for _ in range(laps):
            seq += per_lap[1:] + [per_lap[0]]       # a lap closes on g0 again
        self.seq: list[Crossing] = seq
        self.start_xy = start_xy
        self.k = 0
        self.phase = "TAKEOFF"
        self.t_phase = None
        self.t_last_seen = None
        self.last_det: Optional[Detection] = None
        self.yaw_hold = None
        self.z_hold = self.seq[0].z
        self.turn_target = None
        self.after_turn = None          # phase to enter when a TURN completes
        self.transit = None             # (heading, length_m)
        self.dist_est = 0.0
        self.v_est = 0.0
        self.t_prev = None
        self.hold_then_seek = False
        self.done = False

    # --- geometry from the map ----------------------------------------------
    def prev_xy(self, k: int) -> tuple:
        if k == 0:
            return self.start_xy
        p = self.seq[k - 1]
        d = self.cfg.past_gate_m
        return (p.x + d * math.cos(p.heading_rad), p.y + d * math.sin(p.heading_rad))

    def leg_length(self, k: int) -> float:
        c = self.seq[k]
        px, py = self.prev_xy(k)
        return math.hypot(c.x - px, c.y - py)

    def plan_leg(self, k: int):
        """('direct', heading) or ('dogleg', transit_heading, transit_len)."""
        c = self.seq[k]
        px, py = self.prev_xy(k)
        ax = c.x - self.cfg.approach_m * math.cos(c.heading_rad)
        ay = c.y - self.cfg.approach_m * math.sin(c.heading_rad)
        dx, dy = ax - px, ay - py
        length = math.hypot(dx, dy)
        h = math.atan2(dy, dx) if length > 0.5 else c.heading_rad
        if length < 1.0 or abs(wrap_pi(h - c.heading_rad)) < self.cfg.direct_angle_rad:
            return ("direct", math.atan2(c.y - py, c.x - px))
        return ("dogleg", h, length)

    def is_turnaround(self, k: int) -> bool:
        if k + 1 >= len(self.seq):
            return False
        a, b = self.seq[k], self.seq[k + 1]
        return (math.hypot(a.x - b.x, a.y - b.y) < 0.5
                and abs(wrap_pi(a.heading_rad - b.heading_rad)) > 2.5)

    # --- helpers -----------------------------------------------------------------
    def _enter(self, phase: str, t: float):
        self.phase = phase
        self.t_phase = t

    def _fwd(self, yaw: float, tilt_deg: float) -> tuple:
        a = G * math.tan(math.radians(tilt_deg))
        return (a * math.cos(yaw), a * math.sin(yaw))

    def _lateral(self, yaw: float, a_right: float) -> tuple:
        return (a_right * math.sin(yaw), -a_right * math.cos(yaw))   # body +y is left

    def _advance(self, t: float, accel_mps2: float):
        """Dead-reckoned speed and distance along the current heading."""
        dt = 0.0 if self.t_prev is None else max(0.0, t - self.t_prev)
        self.t_prev = t
        if accel_mps2 > 0:
            self.v_est += (self.cfg.v_creep_est_mps - self.v_est) * min(1.0, dt / self.cfg.speed_tau_s)
        else:
            self.v_est = max(0.0, self.v_est + accel_mps2 * dt)
        self.dist_est += self.v_est * dt

    def _begin_turn(self, target: float, after: str, t: float, yaw: float):
        """Turn to `target`, from a standstill if the change is large."""
        self.turn_target = target
        self.after_turn = after
        if self.v_est > self.cfg.stop_v_mps and abs(wrap_pi(target - yaw)) > self.cfg.turn_in_place_rad:
            self._enter("STOP", t)
        else:
            self._enter("TURN", t)

    def _start_leg(self, k: int, t: float, yaw: float):
        plan = self.plan_leg(k)
        self.dist_est = 0.0
        self.t_prev = t
        if plan[0] == "direct":
            self.yaw_hold = plan[1]
            self.hold_then_seek = False
            if abs(wrap_pi(plan[1] - yaw)) > self.cfg.turn_in_place_rad:
                self.hold_then_seek = True
                self._begin_turn(plan[1], "HOLD", t, yaw)
            else:
                self._enter("SEEK", t)
        else:
            self.transit = (plan[1], plan[2])
            self._begin_turn(plan[1], "TRANSIT", t, yaw)

    # --- the tick -------------------------------------------------------------------
    def step(self, t: float, yaw: float, z: float, vz: float,
             det: Optional[Detection]) -> Command:
        cfg = self.cfg
        if self.t_phase is None:
            self.t_phase = t
        if det is not None and det.area_frac < cfg.min_track_area_frac:
            det = None
        if det is not None:
            self.last_det = det
            self.t_last_seen = t
        k = self.k
        c = self.seq[k] if k < len(self.seq) else self.seq[-1]

        if self.phase == "TAKEOFF":
            self.yaw_hold = yaw if self.yaw_hold is None else self.yaw_hold
            self.z_hold = self.seq[0].z
            if abs(z - self.z_hold) < cfg.takeoff_z_tol_m and abs(vz) < cfg.takeoff_vz_tol_mps and t - self.t_phase > 1.0:
                self.v_est = 0.0
                self._start_leg(0, t, yaw)
                return self.step(t, yaw, z, vz, det)
            return Command((0.0, 0.0), self.z_hold, self.yaw_hold, "TAKEOFF", k)

        if self.phase == "LAND":
            zt = max(0.0, self.z_hold - cfg.land_rate_mps * (t - self.t_phase))
            self.done = zt <= 0.0 and z < 0.15
            return Command((0.0, 0.0), zt, self.yaw_hold, "LAND", k, arm=not self.done, done=self.done)

        if self.phase == "STOP":
            decel = G * math.tan(math.radians(cfg.stop_tilt_deg))
            self._advance(t, -decel)
            if t - self.t_phase >= cfg.stop_min_s and self.v_est <= cfg.stop_v_mps:
                self.v_est = 0.0
                self._enter("TURN", t)
                return self.step(t, yaw, z, vz, det)
            return Command(self._fwd(self.yaw_hold, -cfg.stop_tilt_deg), self.z_hold, self.yaw_hold, "STOP", k)

        if self.phase == "TURN":
            self.z_hold = c.z
            if abs(wrap_pi(yaw - self.turn_target)) < cfg.turn_settle_rad and abs(z - c.z) < cfg.takeoff_z_tol_m:
                self.yaw_hold = self.turn_target
                nxt = self.after_turn
                self.turn_target, self.after_turn = None, None
                self.t_prev = t
                if nxt == "HOLD":
                    self.hold_then_seek = True
                self._enter(nxt, t)
                return self.step(t, yaw, z, vz, det)
            return Command((0.0, 0.0), c.z, self.turn_target, "TURN", k)

        if self.phase == "TRANSIT":
            h, length = self.transit
            self.yaw_hold = h
            self.z_hold = c.z
            self._advance(t, G * math.tan(math.radians(cfg.cruise_tilt_deg)))
            if self.dist_est >= length:
                self.hold_then_seek = True
                self._begin_turn(c.heading_rad, "HOLD", t, yaw)
                return self.step(t, yaw, z, vz, det)
            return Command(self._fwd(h, cfg.cruise_tilt_deg), self.z_hold, h, "TRANSIT", k)

        if self.phase == "HOLD":
            if det is not None:
                self._enter("TRACK", t)
                return self.step(t, yaw, z, vz, det)
            if self.hold_then_seek and t - self.t_phase > cfg.scan_before_seek_s:
                self.hold_then_seek = False
                self.t_prev = t
                self._enter("SEEK", t)
                return self.step(t, yaw, z, vz, det)
            ph = 2.0 * math.pi * (t - self.t_phase) / cfg.scan_period_s
            yaw_t = self.yaw_hold + cfg.scan_amplitude_rad * math.sin(ph)
            return Command((0.0, 0.0), c.z, yaw_t, "HOLD", k, note="scanning")

        if self.phase == "SEEK":
            self.z_hold = c.z
            if det is not None:
                self._enter("TRACK", t)
                return self.step(t, yaw, z, vz, det)
            timeout = cfg.seek_timeout_scale * max(self.leg_length(k), cfg.approach_m) / cfg.v_creep_est_mps
            if t - self.t_phase > timeout:
                self.hold_then_seek = False
                self._begin_turn(self.yaw_hold, "HOLD", t, yaw)
                return self.step(t, yaw, z, vz, det)
            if abs(wrap_pi(yaw - self.yaw_hold)) < 0.35:
                self._advance(t, G * math.tan(math.radians(cfg.cruise_tilt_deg)))
                return Command(self._fwd(self.yaw_hold, cfg.cruise_tilt_deg), self.z_hold, self.yaw_hold, "SEEK", k)
            self._advance(t, 0.0)
            return Command((0.0, 0.0), self.z_hold, self.yaw_hold, "SEEK", k, note="turning")

        if self.phase == "TRACK":
            if det is None:
                if t - (self.t_last_seen or t) > cfg.lost_s:
                    self._enter("SEEK", t)
                    return self.step(t, yaw, z, vz, None)
                det = self.last_det
            self._advance(t, G * math.tan(math.radians(cfg.cruise_tilt_deg)))
            if det.area_frac >= cfg.commit_area_frac:
                self.yaw_hold = yaw
                self._enter("COMMIT", t)
                return self.step(t, yaw, z, vz, None)
            yaw_t = yaw - cfg.k_yaw * det.offset_x          # gate right -> yaw right (negative)
            a_fwd = self._fwd(yaw, cfg.cruise_tilt_deg)
            a_lat = self._lateral(yaw, cfg.k_lat * det.offset_x)
            self.yaw_hold = yaw_t
            return Command((a_fwd[0] + a_lat[0], a_fwd[1] + a_lat[1]), c.z, yaw_t, "TRACK", k)

        if self.phase == "COMMIT":
            self._advance(t, G * math.tan(math.radians(cfg.commit_tilt_deg)))
            if t - self.t_phase < cfg.commit_s:
                return Command(self._fwd(self.yaw_hold, cfg.commit_tilt_deg), c.z, self.yaw_hold, "COMMIT", k)
            turnaround = self.is_turnaround(k)
            self.k += 1
            if self.k >= len(self.seq):
                self.z_hold = z
                self._enter("LAND", t)
                return self.step(t, yaw, z, vz, None)
            if turnaround:
                self.hold_then_seek = True
                self._begin_turn(self.seq[self.k].heading_rad, "HOLD", t, yaw)
            else:
                self._start_leg(self.k, t, yaw)
            return self.step(t, yaw, z, vz, None)

        raise RuntimeError(f"unknown phase {self.phase}")
