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

    def _advance(self, p: np.ndarray) -> int:
        """Monotonic nearest-sample search within a forward window."""
        hi = min(self.n, self.idx + self._win)
        d = np.linalg.norm(self.pos[self.idx:hi] - p, axis=1)
        self.idx += int(np.argmin(d))
        return self.idx

    def _at_s(self, s_target: float) -> int:
        return min(self.n - 1,
                   int(np.searchsorted(self.s, s_target)))

    def step(self, est: StateEstimate):
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

        i = self._advance(est.p)
        s_here = float(self.s[i])
        ic = self._at_s(s_here + f.lookahead_m)

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
_state = {"done_t": None, "dbg_t": 0.0}


def reset_state() -> None:
    _TRACKER.reset()
    _ALT.reset()
    _YAW.reset()
    _state.update(done_t=None, dbg_t=0.0)


def autopilot(update: SensorUpdate) -> RCCommand:
    t = update.t
    if t < T_DISARMED_END:
        return RCCommand(arm=1000, throttle=1000)
    if t < T_ARM_IDLE_END:
        return RCCommand(arm=1800, throttle=1000)

    est = _SOURCE.estimate(update)
    a_des, z_target, vz_ff, yaw_des, done = _TRACKER.step(est)

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
        if t - _state["dbg_t"] >= 1.0:
            _state["dbg_t"] = t
            i = _TRACKER.idx
            print(f"[RL] t={t:5.1f} s={_TRACKER.s[i]:6.1f} "
                  f"p=({est.p[0]:+5.1f},{est.p[1]:+5.1f},{est.p[2]:4.2f}) "
                  f"v={np.linalg.norm(est.v):4.2f} "
                  f"xtrack={np.linalg.norm(_TRACKER.pos[i] - est.p):4.2f} "
                  f"stk=({roll},{pitch},{throttle},{yaw_stick})")

    return RCCommand(arm=1800, throttle=throttle, roll=roll, pitch=pitch,
                     yaw=yaw_stick)
