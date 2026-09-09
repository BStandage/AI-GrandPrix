"""Trajectory-follower solver for the elodin sim.

Flies a plan produced by raceline.planner with an ARC-LENGTH CARROT:
track the nearest point on the path, aim at a lookahead sample, and use the
plan's velocity/acceleration there as feedforward. Falling behind costs time
but never diverges - the plan's timestamps are a prediction to compare
against, not a clock to chase.

Select with:
    RACE_SOLVER=solvers.follower AIGP_TRAJ=/path/to/plan.json \
        uv run elodin run sim/main.py

Layers (the estimator swap later touches ONLY StateSource):
  StateSource  SensorUpdate -> StateEstimate(p, v, R, yaw).
               V1 reads the sim's ground-truth state.
  Tracker      StateEstimate + plan -> desired accel / z / vz / yaw. Pure.
  RC backend   accel + yaw -> sticks, via the flight-proven acro
               thrust-vector attitude loop and altitude throttle loop
               (lifted from solver/pq_waypoints.py). Gains and clamps come
               from vehicle.toml - nothing hardcoded here.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np

from raceline import course as course_bridge  # ensures sim repo on sys.path
from raceline import planner as plan_io
from raceline.config import G, load_config

course_bridge.pq_course()          # side effect: sim repo importable
from solver.api import RCCommand, SensorUpdate  # noqa: E402

from raceline.rc_backend import (   # noqa: E402  (the shared proven loops)
    AltitudeLoop, GroundTruthSource, StateEstimate, YawLoop, attitude_sticks)

# Arming phases, matching the baseline's Betaflight handshake.
T_DISARMED_END = 0.50
T_ARM_IDLE_END = 0.75

# RETRY DEBOUNCE: consecutive control ticks the nearest-point search must read
# "past the gate" before a retry actually fires. At the 1000 Hz PID rate this
# is ~20 ms - long enough to reject a single noisy tick, short enough to still
# react fast to a real miss. Needed because the real sim's control-loop timing
# jitters (measured "cannot achieve real-time" on the SAME run that showed a
# clean 12/12 in the fixed-dt kinematic replay, sim_lite) - an unlatched,
# single-tick check can flicker past/not-past the gate boundary under that
# jitter even when sim_lite (perfectly even dt, no jitter) never sees it,
# which reads to a pilot as the drone backing off and re-trying the same gate
# repeatedly instead of flying through once.
RETRY_CONFIRM_TICKS = 20

# MAX RETRIES PER GATE: if a gate frame is ever physically touched, the
# referee (pq_course.RaceTracker) sets crashed=True and PERMANENTLY stops
# crediting any crossing for the rest of the run (a real-world DQ rule) - but
# it never tells the follower this happened (SensorUpdate/next_gate_index has
# no crashed flag at all). Left unbounded, the follower keeps treating that
# same now-uncreditable gate as "next" forever and retries it endlessly -
# THIS is the infinite back-and-forth. After this many confirmed misses on
# the SAME gate, stop fighting it: drop the gate cap and let the drone keep
# flying the rest of the planned path smoothly instead of oscillating in
# place with no possible payoff.
MAX_RETRIES_PER_GATE = 3

_TRAJ_PATH = os.environ.get("AIGP_TRAJ")
if not _TRAJ_PATH:
    raise RuntimeError(
        "solvers.follower needs AIGP_TRAJ=/path/to/plan.json "
        "(produce one with `python -m raceline.planner`, or use race.py)")

CFG = load_config(os.environ.get("AIGP_VEHICLE_TOML"))
PLAN = plan_io.load_plan(_TRAJ_PATH)
print(f"[RACELINE] plan {os.path.basename(_TRAJ_PATH)}: "
      f"{len(PLAN['events'])} events, {PLAN['s_arr'][-1]:.0f} m, "
      f"predicts {PLAN['predicted']['total_s']:.1f} s "
      f"(model prediction, unverified); config {CFG.sha1[:8]}")


class Tracker:
    """Arc-length carrot on a loaded plan. Pure w.r.t. sensors: consumes a
    StateEstimate, returns desired horizontal accel + vertical/yaw targets."""

    def __init__(self, plan: dict, cfg):
        self.s = plan["s_arr"]
        self.pos = plan["pos"]
        self.vel = plan["vel"]
        self.acc = plan["acc"]
        self.n = len(self.s)
        self.cfg = cfg
        # arc position of every crossing event: the tracker may never run
        # ahead of the next UNSCORED gate (lap 1 and lap 2 overlap in XY;
        # at the g7 hairpin the search walked onto the lap-2 branch and the
        # drone 'finished' a lap the referee never saw)
        self.event_s = [e["s"] for e in plan["events"]]
        # Gate center + required crossing heading, straight from the course
        # geometry (when the plan carries it - older plans without
        # heading_rad fall back to the smoothed-spline proxy below).
        self.event_xyz = [(e.get("x"), e.get("y"), e.get("z"))
                          for e in plan["events"]]
        self.event_heading = [e.get("heading_rad") for e in plan["events"]]
        ds = np.diff(self.s)
        self._ds = float(np.median(ds))
        # Forward search window. Kept SHORT so the nearest-point search can
        # never jump across the stacked gate's turnaround (the exit branch
        # shares XY with the approach branch but is >10 m of arc away).
        self._win = max(2, int(4.0 / self._ds))
        self.reset()

    def reset(self):
        self.idx = 0
        self.started = False
        self._miss_streak = 0    # consecutive ticks reading "past the gate"
        self._retry_gate = -1    # which event the retry counter below is for
        self._retry_count = 0    # confirmed retries fired for _retry_gate
        self._abandoned = set()  # gates given up on - cap dropped for these

    def _advance(self, p: np.ndarray, next_event: int) -> int:
        """Monotonic nearest-sample search, forward window, CAPPED at the
        next unscored gate plus margin."""
        hi = min(self.n, self.idx + self._win)
        if 0 <= next_event < len(self.event_s):
            cap = int(np.searchsorted(self.s, self.event_s[next_event] + 3.0))
            hi = min(hi, max(cap, self.idx + 1))
        d = np.linalg.norm(self.pos[self.idx:hi] - p, axis=1)
        self.idx += int(np.argmin(d))
        return self.idx

    def _at_s(self, s_target: float) -> int:
        return min(self.n - 1,
                   int(np.searchsorted(self.s, s_target)))

    def step(self, est: StateEstimate, next_event: int = -1):
        f = self.cfg.follower

        # Start-settle: hover onto the first plan point before releasing the
        # tracker. Without this, takeoff drift becomes a lateral error the
        # first gate window (+-0.75 m) can't absorb - measured miss: g0 at
        # x=-1.0 after the drone drifted during climb-out.
        if not self.started:
            err0 = self.pos[0] - est.p
            if (float(np.linalg.norm(err0)) < 0.6
                    and float(np.linalg.norm(est.v)) < 1.0):
                self.started = True
            else:
                a_des = f.kp_pos * err0[:2] - f.kd_pos * est.v[:2]
                return a_des, float(self.pos[0][2]), 0.0, None, False

        # Once we've given up on a gate (see MAX_RETRIES_PER_GATE below),
        # stop capping progress at it - fly the rest of the course.
        capped_event = next_event if next_event not in self._abandoned else -1
        i = self._advance(est.p, capped_event)
        s_here = float(self.s[i])

        # RETRY: past the next unscored gate without credit (a miss) the
        # gate cap would deadlock us against it forever. Re-approach: target
        # a point 2.5 m BEFORE the opening and cross it again - the ordered
        # referee accepts late crossings, so a near-miss costs seconds, not
        # the race. General rule, no per-gate anything.
        #
        # DEBOUNCED: the nearest-point search above is re-run every tick, and
        # under real control-loop jitter (not present in the fixed-dt sim_lite
        # replay) it can flicker across the s_ev+0.5 line for a tick or two
        # without a genuine miss - an unlatched check fires a full "back up
        # and re-approach" on that single flicker, which is what reads as the
        # drone repeatedly backing off and re-trying the same gate. Require
        # the miss to persist for RETRY_CONFIRM_TICKS before acting on it.
        if 0 <= capped_event < len(self.event_s):
            s_ev = self.event_s[next_event]
            past_gate = s_here > s_ev + 0.5
            self._miss_streak = self._miss_streak + 1 if past_gate else 0
            if past_gate and self._miss_streak >= RETRY_CONFIRM_TICKS:
                self._miss_streak = 0
                if self._retry_gate != next_event:
                    self._retry_gate, self._retry_count = next_event, 0
                self._retry_count += 1
                if self._retry_count > MAX_RETRIES_PER_GATE:
                    # Confirmed misses on this SAME gate, repeatedly, after
                    # already re-approaching along its own crossing normal
                    # each time - further retries won't succeed where these
                    # didn't (most likely a frame touch already froze scoring
                    # for the rest of the run, per the RaceTracker rule).
                    # Give up on it: fall through to normal tracking below
                    # with the gate cap dropped, instead of oscillating here
                    # forever with zero chance of credit.
                    self._abandoned.add(next_event)
                else:
                    j = self._at_s(max(0.0, s_ev - 2.5))
                    self.idx = j
                    heading = self.event_heading[next_event]
                    if heading is not None:
                        # Re-approach ALONG THE GATE'S OWN CROSSING NORMAL,
                        # not 2.5 m back along the smoothed spline's
                        # arc-length. On a sharp turn the spline sample there
                        # can sit on the WRONG leg of the turn (measured: at
                        # a gate that reverses course, the arc-length retry
                        # point landed on the inbound leg, well off the
                        # gate's actual approach line - a straight-line pull
                        # from there can never thread the opening). A point
                        # on the gate's own normal is ALWAYS lined up for a
                        # straight shot through the opening, on any course
                        # geometry.
                        gx, gy, gz = self.event_xyz[next_event]
                        nx, ny = math.cos(heading), math.sin(heading)
                        tx, ty, tz = gx - 2.5 * nx, gy - 2.5 * ny, gz
                    else:   # older plan without heading_rad: old proxy
                        tx, ty, tz = self.pos[j][0], self.pos[j][1], self.pos[j][2]
                    a_des = (1.5 * f.kp_pos * (np.array([tx, ty]) - est.p[:2])
                             - f.kd_pos * est.v[:2])
                    a_max = self.cfg.a_lat_full()
                    n = float(np.hypot(a_des[0], a_des[1]))
                    if n > a_max:
                        a_des *= a_max / n
                    # Face the re-approach direction instead of freezing the
                    # nose: a None yaw here left the drone's heading locked
                    # from whatever it was doing when the miss was detected,
                    # which on a sharp-turn gate points the camera/frame away
                    # from the opening on every retry. Aim at the point
                    # we're actually flying to.
                    yaw_des = (math.atan2(a_des[1], a_des[0])
                               if n > 0.05 else None)
                    return (a_des, float(tz), 0.0, yaw_des, False)

        # RECOVERY: far off the line, plan feedforward is poison (it kept a
        # stalled drone hovering at a stable equilibrium 11 m off-course).
        # Fly straight back to the nearest path point, nothing else.
        d_near = float(np.linalg.norm(self.pos[i] - est.p))
        if d_near > 2.0:
            a_des = (1.5 * f.kp_pos * (self.pos[i][:2] - est.p[:2])
                     - f.kd_pos * est.v[:2])
            a_max = self.cfg.a_lat_full()
            n = float(np.hypot(a_des[0], a_des[1]))
            if n > a_max:
                a_des *= a_max / n
            return a_des, float(self.pos[i][2]), 0.0, None, False
        # velocity-scaled carrot: a fixed distance is a fixed WARNING TIME
        # only at one speed - at full-mode pace 3.5 m was 0.6 s and corners
        # arrived faster than the loop could lean (two gate misses at the
        # window edge). Standard pure-pursuit scaling.
        # scaled by speed, FLOORED low: a 3.5 m carrot at hairpin-crawl
        # speed points across the path fold and stalls the follower there
        look = max(f.lookahead_m, f.lookahead_t * float(np.linalg.norm(est.v)))
        s_carrot = s_here + look
        if 0 <= next_event < len(self.event_s):
            s_carrot = min(s_carrot, self.event_s[next_event] + 2.0)
        ic = self._at_s(s_carrot)

        # accel feedforward and VELOCITY target from where we ARE (the
        # plan's speed here is the speed to hold - chasing the carrot's
        # velocity fed future speed-ups too early, cut corners, and arrived
        # at the stacked-gate climb at 4.9 m/s where the plan said 3);
        # position target from the carrot ahead
        a_des = (self.acc[i][:2]
                 + f.kp_pos * (self.pos[ic][:2] - est.p[:2])
                 + f.kd_pos * (self.vel[i][:2] - est.v[:2]))
        a_max = self.cfg.a_lat_full()
        norm = float(np.hypot(a_des[0], a_des[1]))
        if norm > a_max:
            a_des *= a_max / norm

        z_target = float(self.pos[ic][2])
        vz_ff = float(self.vel[ic][2])

        iy = self._at_s(s_here + f.yaw_lookahead_m)
        tvec = self.vel[iy]
        txy = math.hypot(tvec[0], tvec[1])
        yaw_des = math.atan2(tvec[1], tvec[0]) if txy > 0.3 else None

        done = (self.s[-1] - s_here < 1.0
                and float(np.linalg.norm(self.pos[-1] - est.p)) < 1.5)
        return a_des, z_target, vz_ff, yaw_des, done


# ---------------------------------------------------------------------------
# RC backend (proven loops from solver/pq_waypoints.py, gains from the toml)
# ---------------------------------------------------------------------------

_SOURCE = GroundTruthSource()
_TRACKER = Tracker(PLAN, CFG)
_ALT = AltitudeLoop(CFG)
_YAW = YawLoop(CFG)
_state = {"done_t": None, "dbg_t": 0.0, "trace": None, "trace_n": 0}

# Per-tick trace, decimated to TRACE_EVERY ticks (~100 Hz at the 1 kHz
# control rate), flushed every second so a crash still leaves the file.
# The 1 Hz heartbeat below cannot resolve a 2 s altitude ring; this can.
TRACE_EVERY = 10
TRACE_COLS = ("t,s,gate,x,y,z,vx,vy,vz,z_target,vz_ff,ax_des,ay_des,"
              "cos_tilt,wx,wy,a_z_cmd,thrust_cmd,roll,pitch,throttle,yaw,"
              "m_mean,m_min,m_max\n")


def _motor_stats(update) -> str:
    """mean,min,max of Betaflight's normalized motor outputs (sim only)."""
    m = getattr(update, "motors", None)
    if m is None or len(m) == 0:
        return "nan,nan,nan"
    return f"{float(np.mean(m)):.3f},{float(np.min(m)):.3f},{float(np.max(m)):.3f}"


def _trace_open():
    if os.environ.get("AIGP_NO_TRACE"):
        return None
    try:
        from raceline.config import AIGP_REPO
        from raceline.planner import next_numbered
        path = next_numbered(str(AIGP_REPO / "out" / "flightlogs"
                                 / "race_XXX.csv"))
        path.parent.mkdir(parents=True, exist_ok=True)
        fh = open(path, "w", encoding="utf-8")
        fh.write(TRACE_COLS)
        print(f"[RACELINE] trace -> {path}")
        return fh
    except OSError as e:          # never let logging ground the pilot
        print(f"[RACELINE] trace disabled: {e}")
        return None


def reset_state() -> None:
    _TRACKER.reset()
    _ALT.reset()
    _YAW.reset()
    if _state["trace"] is not None:
        _state["trace"].close()
    _state.update(done_t=None, dbg_t=0.0, trace=None, trace_n=0)


def autopilot(update: SensorUpdate) -> RCCommand:
    t = update.t
    if t < T_DISARMED_END:
        return RCCommand(arm=1000, throttle=1000)
    if t < T_ARM_IDLE_END:
        return RCCommand(arm=1800, throttle=1000)

    est = _SOURCE.estimate(update)
    a_des, z_target, vz_ff, yaw_des, done = _TRACKER.step(
        est, update.next_gate_index)

    if done:
        if _state["done_t"] is None:
            _state["done_t"] = t
        if t - _state["done_t"] > 1.0:
            return RCCommand(arm=1000, throttle=1000)   # land/disarm

    airborne = est.p[2] >= CFG.follower.min_alt_translation_m
    throttle = _ALT.throttle(update.t, est, z_target, vz_ff, airborne,
                             update.baro_fresh)
    roll = pitch = yaw_stick = 1500
    if airborne:
        roll, pitch, eb = attitude_sticks(CFG, est, a_des)
        yaw_stick = _YAW.stick(est, yaw_des)
    cos_tilt = float(est.R[2, 2])
    _state["trace_n"] += 1
    if _state["trace_n"] == 1:
        _state["trace"] = _trace_open()
    fh = _state["trace"]
    if fh is not None and _state["trace_n"] % TRACE_EVERY == 0:
        w = est.omega if est.omega is not None else (0.0, 0.0, 0.0)
        fh.write(f"{t:.3f},{_TRACKER.s[_TRACKER.idx]:.2f},"
                 f"{update.next_gate_index},"
                 f"{est.p[0]:.3f},{est.p[1]:.3f},{est.p[2]:.3f},"
                 f"{est.v[0]:.3f},{est.v[1]:.3f},{est.v[2]:.3f},"
                 f"{z_target:.3f},{vz_ff:.3f},{a_des[0]:.2f},{a_des[1]:.2f},"
                 f"{cos_tilt:.3f},{w[0]:.2f},{w[1]:.2f},"
                 f"{_ALT.a_cmd:.2f},{_ALT.thrust:.2f},"
                 f"{roll},{pitch},{throttle},{yaw_stick},"
                 f"{_motor_stats(update)}\n")
    # heartbeat ALWAYS prints - a crash below 1 m used to go silent for
    # 23 s while the drone skidded 140 m (measured); crashes must narrate
    if t - _state["dbg_t"] >= 1.0:
        _state["dbg_t"] = t
        if fh is not None:
            fh.flush()
        i = _TRACKER.idx
        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos_tilt))))
        print(f"[RL] t={t:5.1f} s={_TRACKER.s[i]:6.1f} "
              f"p=({est.p[0]:+5.1f},{est.p[1]:+5.1f},{est.p[2]:4.2f}) "
              f"v={np.linalg.norm(est.v):4.2f} "
              f"xtrack={np.linalg.norm(_TRACKER.pos[i] - est.p):4.2f} "
              f"zt={z_target:4.2f} tilt={tilt_deg:3.0f} az={_ALT.a_cmd:+4.1f} "
              f"air={airborne} stk=({roll},{pitch},{throttle},{yaw_stick})")

    return RCCommand(arm=1800, throttle=throttle, roll=roll, pitch=pitch,
                     yaw=yaw_stick)
