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

import collections
import math
import os
from dataclasses import dataclass

import numpy as np

from raceline import course as course_bridge  # ensures sim repo on sys.path
from raceline import planner as plan_io
from raceline.config import G, load_config

course_bridge.pq_course()          # side effect: sim repo importable
try:
    from solver.api import RCCommand, SensorUpdate  # noqa: E402
except ImportError:      # on the Orin: no sim repo, same dataclasses
    from hardware.api_shim import RCCommand, SensorUpdate  # noqa: E402

from raceline.rc_backend import (   # noqa: E402  (the shared proven loops)
    AltitudeLoop, GroundTruthSource, StateEstimate, YawLoop, attitude_sticks,
    angle_sticks)

# Arming phases, matching the baseline's Betaflight handshake.
T_DISARMED_END = 3.00   # 0.50 -> 3.00 (2026-09-22): the sim aircraft sits on the pad long enough to calibrate the accelerometer bias (the runtime's wait loop gives the aircraft ~10 s; 0.5 s of pad samples left a 0.15 m/s inertial-vz bias that the committed hold turned into a climb into g0's top edge). 0.30 tried twice (2026-09-10, race_137 with the boot-grace hold in place): Betaflight never armed, the drone sat on the ground for the whole run - the arm request must come later than 0.3 s after the first RC frame regardless of the boot grace
T_ARM_IDLE_END = 3.05   # 3.00 + 0.05, see T_DISARMED_END. 0.75 -> 0.55 (2026-09-10): the arm takes at 0.50, the motors only need a few frames at idle before throttle-up

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

# SETTLE ON g0 BEFORE THE CLOCK STARTS. The race clock does not run until g0 is
# crossed, so every second spent getting centred on it is free. During the
# start-settle the tracker holds position horizontally while VERT_VISION
# overwrites z_target with est.p[2] + dz_vis - so she is ALREADY climbing to
# null g0's elevation. This just waits for that climb to converge before
# releasing the horizontal, instead of starting the run at whatever dz happened
# to be and correcting on the move. d44 flight 4 made its vertical correction at
# 1.8 m from the gate, which is the worst possible moment; this makes it at 7 m
# standing still, which is the best one.
#
# TIMEOUT-BOUNDED ON PURPOSE. If the detector never gives a steady reference -
# no gate in view, bad light, nothing there - this releases anyway and the
# behaviour is exactly what it was before. The worst case is the old case.
SETTLE_MIN_S = 2.0         # no release inside this: the takeoff punch is
                           # open loop for ~0.8 s and the climb-out follows
SETTLE_TIMEOUT_S = 4.0     # release regardless. WAS 10 (d43 race_006/007,
                           # 2026-09-22): the hold is a limit cycle that GROWS
                           # - x swung +-0.2 m at 5 s and +-0.8 m at 30 s -
                           # and the estimator's velocity drift crossed the
                           # release's 1 m/s at ~4 s. Sally's flight 4 released
                           # at 3 s and tracked; every long hold has wobbled
                           # and then hit the gate. Short hold, then go.
SETTLE_TIMEOUT_XY_M = 1.5  # on timeout the only thing that still holds us
                           # is being far from the start point

_TRAJ_PATH = os.environ.get("AIGP_TRAJ")
# ANGLE MODE (the hardware control shape): the FC closes the attitude loop,
# our sticks are tilt angles (rc_backend.angle_sticks) and AUX2 is held high
# to engage the ANGLE box configured in the SITL (configure_betaflight.py:
# aux 1 = ANGLE on AUX2 1700-2100, angle_limit 80). Default off = acro.
ANGLE_MODE = os.environ.get("AIGP_ANGLE_MODE", "0") == "1"
AUX2 = 1800 if ANGLE_MODE else 1500
print(f"[RACELINE] control mode: {'ANGLE (aux2 1800, tilt-angle sticks)' if ANGLE_MODE else 'ACRO (rate sticks)'}")
if not _TRAJ_PATH:
    raise RuntimeError(
        "solvers.follower needs AIGP_TRAJ=/path/to/plan.json "
        "(produce one with `python -m raceline.planner`, or use race.py)")

CFG = load_config(os.environ.get("AIGP_VEHICLE_TOML"))
BRAKE_LOOKAHEAD_T = 0.0    # s of anticipation for a planned brake at terminal speed
                           # (see Tracker.step). OFF: the replay loses g7 with
                           # 0.15 (loop entry speed), and the plan brakes into
                           # the g3-g4 arc at only 3.5 m/s^2 anyway.
BRAKE_LA_SHARE0 = 0.7      # anticipation starts once drag holds this share of the tilt (v > ~8 m/s)
CARROT_CROSS_ONLY = True   # position term = cross-track offset to the carrot only (see Tracker.step)
ATT_IDLE_SCALE = False     # superseded by THRUST_VECTOR_MODE (race_057: zero demand at idle fell cleanly but drifted 2 m). scale the horizontal demand by the collective below hover (see autopilot)
THRUST_VECTOR_MODE = True  # throttle = |(a_h, g + a_z)|, tilt = its angle (see autopilot)
VECTOR_FREEFALL_SHARE = 0.85   # a_z floor: -0.85 g (a quad cannot fall faster than g anyway)
VECTOR_TILT_MAX_DEG = 60.0     # tilt cap when descending (gz small)
ACC_LEAD_S = float(os.environ.get("AIGP_ACC_LEAD_S", "0.10"))   # feedforward acceleration taken this far ahead along the plan (attitude lag compensation, see Tracker.step)
AZ_FF_GAIN = 0.0           # plan vertical-accel feedforward into the altitude loop: OFF - the calibrated replay fails the clean-flown plan_030 with it on (overshoots the top gate); untested in flight
YAW_IDLE_BAND = 150        # PWM below hover_pwm under which no yaw is commanded (see autopilot)
THRUST_BUDGET_SHARE = 1.0  # share of the motors' total specific thrust the follower may commit; vertical need first, horizontal gets the rest (see autopilot). 0 disables.
NO_OVERSPEED_PUSH = True   # at/above plan speed, no forward along-track push (see Tracker.step)
OVERSPEED_SHARE0 = 0.5     # ...but only where drag already holds this share of the tilt (v > ~6.9 m/s); the 3.5 m/s loops keep their pull-through
PRIORITY_CLAMP = True      # clamp keeps the cross-track component, trims along-track
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
        # nose heading per sample when the plan carries one (held across
        # the stacked-pair cusp, blended back to the tangent after it)
        self.yaw_arr = plan.get("yaw_arr")
        self.hold_arr = plan.get("hold_arr")   # True inside a reversal fold (the stack)
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
        self._settle_t0 = None   # when the start-settle began
        self._dz_ok_since = None # when dz first entered the level band

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

    def step(self, est: StateEstimate, next_event: int = -1, t: float | None = None):
        f = self.cfg.follower

        # Start-settle: hover onto the first plan point before releasing the
        # tracker. Without this, takeoff drift becomes a lateral error the
        # first gate window (+-0.75 m) can't absorb - measured miss: g0 at
        # x=-1.0 after the drone drifted during climb-out.
        if not self.started:
            err0 = self.pos[0] - est.p
            if t is not None and self._settle_t0 is None:
                self._settle_t0 = t
            held = 0.0 if (t is None or self._settle_t0 is None) else t - self._settle_t0
            # RELEASE RULE, RACE DAY 2 (2026-09-22). Sally's flight 4, the
            # only run that ever tracked to a gate, released at 3 s on
            # position alone. Everything added since - a velocity check, a
            # vertical dwell - only ever kept the aircraft in the hold, and
            # the hold is where every wobble and every gate strike came
            # from. So: up for SETTLE_MIN_S (past the open-loop takeoff
            # punch), over the start point, go. Under VERT_VISION the plan's
            # start altitude is not a target, so the horizontal alone counts.
            # The vertical converges on the move: the gate's elevation slews
            # the reference at 0.35 m/s and the plan takes 11 s to g0.
            axes = slice(0, 2) if VERT_VISION else slice(0, 3)
            pos_ok = float(np.linalg.norm(err0[axes])) < 0.6
            timed_out = held >= SETTLE_TIMEOUT_S
            far = float(np.linalg.norm(err0[:2])) > SETTLE_TIMEOUT_XY_M
            # The minimum hold exists ONLY under VERT_VISION, where the
            # horizontal check alone would release on the pad at t=0. On the
            # barometric path the 3D check already waits for the climb, and
            # holding there is harmful: the hold's height target is the
            # plan's first point (0.23 m), so a 2 s hold after a takeoff to
            # 1.2 m is a commanded dive - the sim flipped at 4 s on exactly
            # that (2026-09-22). No clock (replay, sim_lite): position alone.
            min_ok = (t is None) or (not VERT_VISION) or held >= SETTLE_MIN_S
            if (pos_ok and min_ok) or (timed_out and not far):
                self.started = True
                if t is not None:
                    print(f"[RACELINE] released at t={t:.1f}s "
                          f"({'timed out on' if timed_out else 'over'} g0: "
                          f"dz={_DZ['v']:+.3f} m, {float(np.linalg.norm(err0[:2])):.2f} m off)")
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
                # MISSED = COUNTED, MOVE ON (Brian, race day 2, 2026-09-22):
                # "even if we miss a gate, keep going and count them as
                # scored." No retry, ever: the plan continues, the cap drops,
                # and the estimator's own gate count advances so the next gate
                # becomes the one the camera looks for and the height steers
                # on. (The old retry backed up 2.5 m and re-approached, up to
                # three times.)
                self._miss_streak = 0
                self._abandoned.add(next_event)
                _count_missed_gate(next_event)
                print(f"[RACELINE] g{next_event} MISSED - counted, moving on")
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
        # Drag compensation: the plan's acc is NET of the measured drag
        # ([vehicle] drag_*), so the thrust the loop must ask for is that
        # plus the drag the plant takes back at this speed. Without it the
        # kd term had to carry 9 m/s^2 of drag at 6 m/s as a permanent
        # 2.3 m/s velocity deficit (race_029: every straight 1-6 m/s under
        # plan). Along the CURRENT velocity - that is the direction the
        # plant applies it.
        v_h = float(np.hypot(est.v[0], est.v[1]))
        a_drag_ff = ((self.cfg.a_drag(v_h) / v_h) * est.v[:2]
                     if v_h > 0.3 else np.zeros(2))
        e_pos = self.pos[ic][:2] - est.p[:2]
        v_plan_here = float(np.hypot(self.vel[i][0], self.vel[i][1]))
        # CROSS-TRACK ONLY position term (race_038): the carrot sits
        # lookahead_t*v ahead ALONG the path, so kp * that distance is a
        # permanent forward push (10-11 m/s^2 at kp 4, 8.7 m/s) that
        # cancelled the speed loop's braking (kd -7) on the g3-g4 entry: the
        # drone crossed g4 at 8.7 against a plan of 6.7 and ran outside
        # from there to g6's post. Along-track position has no meaning for
        # a speed-profile follower; the speed loop owns it.
        if CARROT_CROSS_ONLY and v_h > 0.5:
            u = est.v[:2] / v_h
            # ...and measured to the NEAREST path point, not the carrot: on
            # an arc the carrot's chord points inward, and at kp 4 that pull
            # cut every arc 0.3 m inside and g7 by 0.85 m in replay. The
            # carrot keeps its jobs for altitude and yaw below.
            e_pos = self.pos[i][:2] - est.p[:2]
            e_pos = e_pos - float(np.dot(e_pos, u)) * u
        # Speed target: the plan speed HERE, except that an upcoming brake
        # is anticipated (never a speed-up: chasing the carrot's speed fed
        # accelerations too early and cut corners). The anticipation is
        # the time the attitude needs to swing the thrust vector from
        # "holding speed against drag" to "braking", so it scales with
        # the drag share of the tilt: BRAKE_LOOKAHEAD_T at terminal
        # speed, nothing in the slow loops (where replay showed it costs).
        v_tgt = self.vel[i][:2]
        share = self.cfg.a_drag(v_h) / self.cfg.a_lat_full()
        t_la = BRAKE_LOOKAHEAD_T * max(0.0, (share - BRAKE_LA_SHARE0)
                                       / max(1.0 - BRAKE_LA_SHARE0, 1e-6))
        ib = self._at_s(s_here + t_la * v_h)
        vb = float(np.hypot(self.vel[ib][0], self.vel[ib][1]))
        vi = float(np.hypot(v_tgt[0], v_tgt[1]))
        if vb < vi and vi > 1e-6:
            v_tgt = v_tgt * (vb / vi)
        # LEAD the feedforward: the attitude answers ~0.1 s late, so the
        # plan's acceleration is taken ACC_LEAD_S ahead along the path
        # (race_053: 0.3-0.5 m wide in every fast arc, banking for where
        # the drone was). The cross-track and speed terms stay at the
        # nearest point.
        # ...but not inside a reversal fold: leading there hands the drone
        # the pull-out push while it is still at the top of the stack, it
        # holds 20-40 deg of tilt through the drop, the attitude
        # corrections at idle throttle make lift and it hangs (race_055).
        in_fold = self.hold_arr is not None and bool(self.hold_arr[i])
        self.in_fold = in_fold    # the vertical feedforward is applied only here (race_063: on the takeoff climb it overshot g0's top bar)
        i_ff = self._at_s(s_here + ACC_LEAD_S * v_h) if (ACC_LEAD_S > 0.0 and not in_fold) else i
        a_ctrl = (self.acc[i_ff][:2] + f.kp_pos * e_pos
                  + f.kd_pos * (v_tgt - est.v[:2]))
        # No forward push once AT plan speed: the carrot's along-track
        # part (kp * lookahead, 3.6 m/s^2 at 9.9 m/s) otherwise drives the
        # drone past the plan; with the drag feedforward it summed to
        # 27.7 on the g2-g3 straight, pinned the tilt at 68 deg and
        # delivered the drone to the g4 turn at 9.9 m/s with no authority
        # left (race_031). Only the forward NET component is cut, only
        # when not slower than the plan - the loop entries, where the
        # drone is below plan speed, keep the pull-through (removing it
        # there cost +0.13..0.4 m at g7 in replay).
        drag_share = self.cfg.a_drag(v_h) / self.cfg.a_lat_full()
        if (NO_OVERSPEED_PUSH and v_h > 0.5 and v_h >= v_plan_here
                and drag_share >= OVERSPEED_SHARE0):
            u = est.v[:2] / v_h
            fwd = float(np.dot(a_ctrl, u))
            if fwd > 0.0:
                a_ctrl = a_ctrl - fwd * u
        a_des = a_ctrl + a_drag_ff
        # Priority clamp: the ONE thrust vector is bounded by max tilt.
        # A uniform scale-down took the turn away together with the
        # brake (race_031, 3 m wide). Keep the cross-track component
        # (never go wide), give the along-track component what is left
        # (go slow instead).
        a_max = self.cfg.a_lat_full()
        if PRIORITY_CLAMP and v_h > 0.5:
            u = est.v[:2] / v_h
            n_ = np.array([-u[1], u[0]])
            cross = max(-a_max, min(a_max, float(np.dot(a_des, n_))))
            room = math.sqrt(max(a_max * a_max - cross * cross, 0.0))
            along = max(-room, min(room, float(np.dot(a_des, u))))
            a_des = along * u + cross * n_
        else:
            norm = float(np.hypot(a_des[0], a_des[1]))
            if norm > a_max:
                a_des *= a_max / norm

        z_target = float(self.pos[ic][2])
        vz_ff = float(self.vel[ic][2])
        self.az_ff = float(self.acc[ic][2]) if self.acc.shape[1] > 2 else 0.0

        iy = self._at_s(s_here + f.yaw_lookahead_m)
        if self.yaw_arr is not None:
            yaw_des = float(self.yaw_arr[iy])
        else:
            tvec = self.vel[iy]
            txy = math.hypot(tvec[0], tvec[1])
            yaw_des = math.atan2(tvec[1], tvec[0]) if txy > 0.3 else None

        done = (self.s[-1] - s_here < 1.0
                and float(np.linalg.norm(self.pos[-1] - est.p)) < 1.5)
        return a_des, z_target, vz_ff, yaw_des, done


# ---------------------------------------------------------------------------
# RC backend (proven loops from solver/pq_waypoints.py, gains from the toml)
# ---------------------------------------------------------------------------

# STATE SOURCE. ground_truth (default): the sim's exact pose - the number the
# real drone does not have. deadreckon: vision-aided dead reckoning
# (seeker.dr_estimator): IMU integrated on the attitude, barometer for
# altitude, and a position fix from every sighting of the next gate (in the
# sim the sighting comes from the synthetic camera). This is the state
# source the Archer flies on.
STATE_SOURCE = os.environ.get("AIGP_STATE_SOURCE", "ground_truth")
if STATE_SOURCE == "deadreckon":
    from seeker.dr_estimator import DeadReckonSource
    from seeker import synthetic_camera as _cam
    _SOURCE = DeadReckonSource()
    print("[RACELINE] state source: vision-aided DEAD RECKONING (no ground-truth position)")
else:
    _SOURCE = GroundTruthSource()
    print("[RACELINE] state source: ground truth")
_TRACKER = Tracker(PLAN, CFG)
_FIX = {"n": 0, "last_res": 0.0, "t_det": -1.0, "err": 0.0}
_POSES = _cam.PoseHistory(_cam.NOISE["latency_s"]) if STATE_SOURCE == "deadreckon" else None
if STATE_SOURCE == "deadreckon":
    # the drone has no referee: the estimator advances the gate index itself
    # when its own position crosses the next opening's plane
    _SOURCE.set_events([(e["x"], e["y"], e["z"], e.get("heading_rad")) for e in PLAN["events"]])
# YAW POLICY. The plan's yaw follows the path tangent, which points the
# camera away from the gate through a hairpin exactly when the estimator
# needs to see it. With dead reckoning the nose aims at the next gate
# instead (thrust-vector control does not care where the nose points),
# handing back to the tracker's crossing heading inside 2 m of the gate.
AIM_AT_GATE = os.environ.get("AIGP_YAW_AT_GATE", "1" if STATE_SOURCE == "deadreckon" else "0") == "1"
AIM_HANDOFF_M = float(os.environ.get("AIGP_AIM_HANDOFF_M", "2.0"))   # metres before the gate where the nose goes back to the crossing heading
# unique gate landmarks (xyz + crossing heading) from the plan's events; the
# stacked pair is two landmarks at one XY
_GATE_LANDMARKS = []
_seen_lm = set()
for _e in PLAN["events"]:
    _key = (round(_e["x"], 2), round(_e["y"], 2), round(_e["z"], 2))
    if _e.get("heading_rad") is not None and _key not in _seen_lm:
        _seen_lm.add(_key)
        _GATE_LANDMARKS.append((_e["x"], _e["y"], _e["z"], _e["heading_rad"]))
if STATE_SOURCE == "deadreckon":
    _SOURCE.set_landmarks(_GATE_LANDMARKS)      # the map says which gates can be in view
    _SOURCE.arm_t_s = T_ARM_IDLE_END            # sim: armed from here, vz_inertial integrates from here
    _SOURCE.pad_until_s = T_ARM_IDLE_END + 0.5  # sim: learn the accel bias while still on the pad; the
                                                # horizontal ZUPT holds through the spool-up and lift-off
# VERTICAL FROM VISION. AIGP_VERT=vision replaces the barometric height
# reference with the gate's own elevation.
#
# The barometer is unusable on this airframe with props running: it read
# -3.86 m on d45 and +1.76 m on d44 while both were near the ground, and five
# commanded-altitude flights broke two airframes. The one mode that ever flew
# clean never asked for a height at all.
#
# The trick is that the height error need not come from a height. Pass
#   z_target = est.p[2] + dz_vision
# and the loop's error term is EXACTLY dz_vision - est.p[2] cancels, so the
# barometer leaves the vertical channel entirely while every tested line of
# AltitudeLoop (tilt compensation, the balloon guard, the thrust cap, the
# curve inversion) stays in place.
#
# dz_vision comes from the gate's elevation: zero elevation means level with
# the gate's centre, and on the FLAT course every gate centre is 1.35 m. So
# nulling it IS holding 1.35 m, without ever measuring a height. With no gate
# in view the term is zero and the loop is a pure velocity hold on the plan's
# own vz_ff, which is what flew cleanly twice.
VERT_VISION = os.environ.get("AIGP_VERT", "baro") == "vision"
COMMIT_STRAIGHT = os.environ.get("AIGP_COMMIT_STRAIGHT", "1") == "1"   # committed = zero roll, pitch along the nose (see step)
COMMIT_HOLD = os.environ.get("AIGP_COMMIT_HOLD", "1") == "1"   # committed = hold hover throttle instead of the vz hold (see step)
COMMIT_LAT_KP = 3.0     # final approach / committed: m/s^2 per m off the gate's centre line
COMMIT_LAT_KD = 3.0     # ...and per m/s of lateral speed
FINAL_APPROACH_M = 6.0  # the last this-many metres to an aligned gate are flown at its centre line
COMMIT_LAT_MAX = 1.4    # m/s^2 = 8 deg of lean, the plan's own cap. Was 0.6 (3.5 deg): the hairpin g5 arrives at the line still carrying 1.3 m/s of turn, overshot to -0.57 m and touched the frame (sim race_060). Zero when centred, so the straight gates are unaffected.
VERT_EL_GAIN = 0.09        # metres of height correction per degree of elevation. 0.06 -> 0.09 (race day 2): sized for ~5 m now that COMMIT freezes the height at 3.5 m - at 0.06 three sim runs arrived at commit 0.4 m high with the loop still asking for down, and grazed the top edge at 2.09 m.
                           # One degree is r*sin(1 deg) of real height: 0.035 m at
                           # 3 m, 0.14 at 8. Gates are seen from about 3 to 8 m, and
                           # the gain is sized for the SHORT end so it always
                           # under-corrects and converges instead of hunting. It is
                           # deliberately not range-scaled - range is the camera's
                           # worst signal and the whole point of using elevation is
                           # that it does not need one.
VERT_DZ_MAX_TAKEOFF = 0.20 # the cap until g0 is crossed: 9 * 0.20 / 4 = 0.45 m/s of climb, slow and steady
VERT_DZ_MAX = 0.35         # the cap after g0: 0.79 m/s, enough for the stack's 0.72. The note below is from the takeoff cap. hard cap on that correction, m. 0.35 -> 0.20 (race day 2,
                           # Brian: "slow and steady, no steep takeoff"). This cap IS the
                           # climb speed: the loop settles where kp_z * dz = kd_z * vz, so
                           # 9 * 0.35 / 4 = 0.79 m/s before, 9 * 0.20 / 4 = 0.45 m/s now.
                           # In the sim the fast climb-out overshot to 3 m and put the
                           # gate below the camera's field of view, and every flight this
                           # week overshot gate height on the way up.
VERT_EL_STALE_S = 0.5      # a detection older than this is not used
VERT_DZ_SLEW = 0.35        # m/s the reference may move. THE REFERENCE IS
                           # SLEW-LIMITED, not just clamped, and that matters
                           # more than the clamp. The detector is spotty: it
                           # found a gate in 99.6 percent of bench frames and
                           # also found one with the lens covered, so it will
                           # flicker. Unlimited, each acquire/lose cycle steps
                           # the reference by up to 0.35 m, and kp_z is 9.0 -
                           # a 3 m/s^2 pulse appearing and vanishing at the
                           # detection rate. kd_z damps VELOCITY; nothing
                           # damps a step in the reference.
                           #
                           # Slew-limited, a flicker lasting one frame moves
                           # the reference 0.01 m. Only a detection that
                           # PERSISTS gets to move the aircraft, which is
                           # exactly the discrimination we want.
_GATE_EL = None            # (t, elevation_rad), set by the runtime each tick
_GATE_RANGE = None         # the detection's range (m) that came with it, if any
_COMMITTED = False         # inside the commit range: snap the reference, do not fade
_DZ = {"v": 0.0, "t": None}  # the slew-limited reference offset


def set_gate_elevation(el_rad, t, committed=False, range_m=None):
    """The runtime hands the follower the gate's elevation when it has one.

    Elevation, not height: it is the one camera quantity that needs no range,
    and range is the camera's worst signal.

    COMMITTED distinguishes the two ways of having no elevation, which want
    opposite treatment:

    A DROPOUT (el_rad None, committed False) is the detector flickering. The
    gate is coming back, so the reference FADES at VERT_DZ_SLEW - a one-frame
    flicker then moves it 0.01 m instead of stepping it.

    A COMMIT (committed True) is a decision, not a loss. We are inside the
    range where the gate's height cannot be trusted and we have chosen to stop
    steering on it. Fading leaves up to a second of decaying climb command
    running - at 1.5 m/s that is the last 1.5 m of the approach still being
    told to climb, which is the wrong second to be moving vertically. So the
    reference SNAPS to zero and the loop becomes a pure vertical speed hold
    immediately: whatever vz exists is braked out at kd_z and nothing new is
    commanded. Once committed, the aircraft should not change its vertical
    speed again before the gate."""
    global _GATE_EL, _COMMITTED, _GATE_RANGE
    _COMMITTED = bool(committed)
    _GATE_EL = None if el_rad is None else (float(t), float(el_rad))
    _GATE_RANGE = None if range_m is None else float(range_m)
    # the last HOLD_FIT_S of (t, geometric height to the gate centre, inertial
    # vz): the commit hold fits these to learn the inertial vz's offset and
    # the height still to make (see the hold in step)
    if el_rad is not None and range_m is not None and not committed:
        # one sample per detection, not per control tick (the same frame is
        # handed over for up to 0.5 s)
        key = (round(float(el_rad), 6), round(float(range_m), 4))
        if key != _DZ_LAST[0]:
            _DZ_LAST[0] = key
            vzi = float(getattr(_SOURCE, "vz_inertial", float("nan"))) if _SOURCE is not None else float("nan")
            _DZ_HIST.append((float(t), float(range_m) * math.sin(float(el_rad)), vzi, float(range_m)))
            fit = _fit_vision_vz(float(t))
            if fit is not None and abs(fit[2]) < 3.0:
                _VZ_TRIM["off"], _VZ_TRIM["t"] = float(fit[2]), float(t)
    while _DZ_HIST and _DZ_HIST[0][0] < float(t) - HOLD_FIT_S:
        _DZ_HIST.popleft()


HOLD_HEIGHT = os.environ.get("AIGP_HOLD_HEIGHT", "0") == "1"   # the committed hold flies a HEIGHT (fit from the elevation history), not a speed. Sim-tested only; off = build bec1096
VZ_VISION_TRIM = os.environ.get("AIGP_VZ_VISION_TRIM", "1") == "1"   # 0 = the raw inertial vertical speed (Julian, 2026-09-22 attempt 1: way high over g0)
_VZ_TRIM = {"off": 0.0, "t": None}   # the offset of the inertial vertical speed, as the elevation fit last measured it


def vz_inertial_trimmed() -> float:
    """The inertial vertical speed with its measured offset removed.

    JULIAN, RACE DAY 2, ATTEMPT 1: pad bias zero, and in flight the
    integrated accelerometer read -0.2, -0.5, -0.6, -1.2 m/s while the gate's
    elevation said he was CLIMBING at +0.2..+0.6 (the accelerometer under
    the props reads about 0.3 m/s^2 low; the pad cannot see that). The
    vertical loop damps on this speed: +4 m/s^2 of 'stop descending' against
    -1.8 m/s^2 of 'the gate is below you', and he went way high over g0.
    The elevation fit measured the offset the whole time (-0.34, -0.50,
    -0.74, -1.19). So: accelerometer for the fast part, vision for the slow
    part. The offset freezes when the gate is lost or committed."""
    v = float(getattr(_SOURCE, "vz_inertial", 0.0))
    return v - _VZ_TRIM["off"] if VZ_VISION_TRIM else v


HOLD_FIT_S = 1.5            # window of elevation samples the commit hold fits
HOLD_FIT_MIN_N = 8          # ...needs this many samples over at least HOLD_FIT_MIN_SPAN_S
HOLD_FIT_MIN_SPAN_S = 0.8
HOLD_DZ_MAX_M = 0.6         # the height the hold will make after commit, at most
HOLD_FIT_MAX_OFFSET = 0.60  # vision and inertial vertical speed further apart than this: the fit is not trusted (0.25 threw out a correct -0.35 at g1 and the speed rule flew him 1 m high: sim seed 0, 2026-09-22)
HOLD_FIT_MAX_DZ = 0.30      # more height than this still to make at commit: not a level commit, the fit is not trusted
_DZ_HIST = collections.deque()
_DZ_LAST = [None]
HOLD_FIT_DZ_OUTLIER_M = 0.3   # a sample this far from the window's median height is a bad frame, not motion
HOLD_FIT_RANGE_OUTLIER = 0.25 # ...or this fraction off the median range


def _fit_dbg():
    """For the [RL] line: the fit's true vz and height to make, and the newest
    geometric dz and range in the window."""
    if not _DZ_HIST:
        return "-"
    tl, dzl, _, _ = _DZ_HIST[-1]
    r = _GATE_RANGE if _GATE_RANGE is not None else float("nan")
    fit = _fit_vision_vz(tl)
    if fit is None:
        return f"(dzg={dzl:+.2f} r={r:.1f} nofit)"
    return f"(vz={fit[0]:+.2f} dz={fit[1]:+.2f} off={fit[2]:+.2f} dzg={dzl:+.2f} r={r:.1f})"


def _fit_vision_vz(t):
    """Least squares over _DZ_HIST: (true vertical speed from the elevation
    history, height still to make at t, inertial vz offset, n, span).
    None when the window is too thin to trust."""
    pts = [(a, b, c, r) for a, b, c, r in _DZ_HIST if t - a <= HOLD_FIT_S and not math.isnan(c)]
    if len(pts) < HOLD_FIT_MIN_N:
        return None
    # ROBUST: a false detection (the real detector's garbage frames, the
    # synthetic camera's false positives at a random range) puts one sample
    # metres off the line; least squares over 1.5 s then reads metres per
    # second of climb that never happened (seed 2: "vision vz +0.46" while
    # level, and the height hold flew him into the floor at g1). Drop
    # anything far from the window's median height or range, then fit.
    dzs = sorted(b for _, b, _, _ in pts); rs = sorted(r for _, _, _, r in pts)
    mdz = dzs[len(dzs) // 2]; mr = rs[len(rs) // 2]
    pts = [(a, b, c) for a, b, c, r in pts
           if abs(b - mdz) <= HOLD_FIT_DZ_OUTLIER_M and abs(r - mr) <= HOLD_FIT_RANGE_OUTLIER * mr]
    if len(pts) < HOLD_FIT_MIN_N:
        return None
    span = pts[-1][0] - pts[0][0]
    if span < HOLD_FIT_MIN_SPAN_S:
        return None
    n = float(len(pts))
    mt = sum(a for a, _, _ in pts) / n
    md = sum(b for _, b, _ in pts) / n
    mv = sum(c for _, _, c in pts) / n
    sxx = sum((a - mt) ** 2 for a, _, _ in pts)
    if sxx <= 1e-6:
        return None
    slope = sum((a - mt) * (b - md) for a, b, _ in pts) / sxx   # d(dz)/dt = -vz
    vz_vis = -slope
    dz_now = md + slope * (t - mt)
    return vz_vis, dz_now, mv - vz_vis, len(pts), span


def _past_g0():
    """True once the estimator's own gate count is past g0 (deadreckon), else
    False; None when there is no estimator count (sim ground truth)."""
    if STATE_SOURCE == "deadreckon" and hasattr(_SOURCE, "next_event"):
        return int(_SOURCE.next_event) >= 1
    return None


def _vision_dz(t):
    """How far to move vertically to sit level with the gate's centre.

    Slew-limited: see VERT_DZ_SLEW. Losing the gate walks the reference back
    to zero at the same rate rather than dropping it, so a dropout is a fade
    into velocity hold and not a step."""
    want, el = 0.0, None
    if VERT_VISION and _GATE_EL is not None:
        t_el, e = _GATE_EL
        if t - t_el <= VERT_EL_STALE_S:
            el = e
            # two caps: the gentle one until g0 is crossed (the climb-out is
            # where every flight overshot), the plan's own after it - the
            # stacked gate needs 0.72 m/s of climb and 0.20 caps it at 0.45
            # (sim race_070: 3.35 m at a 3.30 m bottom edge)
            cap = VERT_DZ_MAX_TAKEOFF if _past_g0() is False else VERT_DZ_MAX
            want = max(-cap, min(cap, VERT_EL_GAIN * math.degrees(e)))
    dt = 0.02 if _DZ["t"] is None else max(0.0, min(0.1, t - _DZ["t"]))
    _DZ["t"] = t
    if _COMMITTED:
        _DZ["v"] = 0.0          # a decision, not a loss: snap, do not fade
        return 0.0, el
    step = VERT_DZ_SLEW * dt
    _DZ["v"] += max(-step, min(step, want - _DZ["v"]))
    return _DZ["v"], el


def vision_dz() -> float:
    """The slew-limited height offset the aircraft is currently being asked to
    fly, metres, + is up. This IS the vertical command: read it to narrate or
    log what the vertical channel is doing without reaching into _DZ."""
    return float(_DZ["v"])


def _reset_vision_dz():
    global _COMMITTED
    _COMMITTED = False
    _DZ["v"], _DZ["t"] = 0.0, None


_ALT = AltitudeLoop(CFG)
_YAW = YawLoop(CFG)
LAND_RATE_MPS = 1.0   # descent after the finish (see autopilot)
_state = {"done_t": None, "dbg_t": 0.0, "trace": None, "trace_n": 0, "airborne_latch": False,
          "thr_hist": collections.deque(), "hold_thr": None, "vz_lp": 0.0, "hold_t": 0.0,
          "dz_hist": collections.deque(), "vis_t": 0.0, "hold_vz0": 0.0,
          "hold_b": 0.0, "hold_dz": 0.0, "hold_z": 0.0, "hold_tp": 0.0}
VZ_VIS_WINDOW_S = 0.5   # the elevation-rate window for the vision vertical speed
VZ_VIS_TAU_S = 1.0      # how fast the inertial vz is pulled toward it while a gate is in view
HOLD_KD_PWM = 200.0    # us of throttle per m/s of INERTIAL vertical speed while committed (baro-free, smooth): a 0.2 m/s drift is met with 40 us (~2 m/s^2)
HOLD_VZ_TAU_S = 0.7    # the low-pass: a 5 Hz baro spike of 1.5 m/s moves the throttle ~10 us

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
    _state.update(done_t=None, dbg_t=0.0, trace=None, trace_n=0, airborne_latch=False,
                  thr_hist=collections.deque(), hold_thr=None, vz_lp=0.0, hold_t=0.0,
                  dz_hist=collections.deque(), vis_t=0.0, hold_vz0=0.0, hold_b=0.0, hold_dz=0.0, hold_z=0.0, hold_tp=0.0)


_DR = {"fh": None, "n": 0, "det": None}
_VIS = {"ev": None, "run": 0, "latched": False, "t_match": None}
_SIM_COMMIT_FRAC = float(os.environ.get("AIGP_GATE_COMMIT_FRAC", "0.85"))


def _sim_vision_vertical(t, est):
    """The runtime's vision block, for the synthetic camera. Hands the
    follower the gate's elevation (or None) and the commit state."""
    det = _DR["det"]
    fresh = det is not None and (t - det.t) <= 0.5
    if _SOURCE.next_event != _VIS["ev"]:
        if _VIS["ev"] is not None:
            print(f"[SIM] CROSSED g{_VIS['ev']} -> next g{_SOURCE.next_event} (t={t:.1f}, lat {_SOURCE.cross_lat:+.2f} m)")
        _VIS.update(ev=_SOURCE.next_event, run=0, latched=False)
    near = True
    if 0 <= _SOURCE.next_event < len(_SOURCE.events):
        gx, gy = _SOURCE.events[_SOURCE.next_event][0], _SOURCE.events[_SOURCE.next_event][1]
        near = math.hypot(gx - _SOURCE.p[0], gy - _SOURCE.p[1]) <= 6.0
    # apparent size: a 2.7 m ring at range r spans 2.7/r of tan; the frame
    # spans 2*HALF_TAN_Y. Committed when that ratio reaches COMMIT_FRAC.
    commit_range = _cam.GATE_OUTER_M / (_SIM_COMMIT_FRAC * 2.0 * _cam.HALF_TAN_Y)
    fills = fresh and det.range_m is not None and det.range_m <= commit_range
    aligned = True
    if 0 <= _SOURCE.next_event < len(_SOURCE.events):
        aligned = commit_aligned(_SOURCE.p, est.yaw, _SOURCE.events[_SOURCE.next_event])
    level_ok = True
    if fresh and not _VIS["latched"]:
        _dc = det.offset_y * _cam.HALF_TAN_Y; _rc = det.offset_x * _cam.HALF_TAN_X
        _c, _s = math.cos(_cam.CAM_TILT_RAD), math.sin(_cam.CAM_TILT_RAD)
        _v = np.array([_c + _s * _dc, -_rc, -(-_s + _c * _dc)])
        _vw = est.R @ (_v / (float(np.linalg.norm(_v)) or 1.0))
        level_ok = commit_level_ok(math.asin(max(-1.0, min(1.0, float(_vw[2])))), det.range_m)
    if fills and near and aligned and level_ok:
        _VIS["run"] += 1
        if _VIS["run"] >= 3 and not _VIS["latched"]:
            _VIS["latched"] = True
            print(f"[SIM] COMMIT g{_SOURCE.next_event} at det range {det.range_m:.2f} m (t={t:.1f})")
    else:
        _VIS["run"] = 0
    el = None
    matched = _VIS["t_match"] is not None and (t - _VIS["t_match"]) <= 0.75
    if fresh and matched and not _VIS["latched"]:
        down_cam = det.offset_y * _cam.HALF_TAN_Y
        right_cam = det.offset_x * _cam.HALF_TAN_X
        c, s_ = math.cos(_cam.CAM_TILT_RAD), math.sin(_cam.CAM_TILT_RAD)
        v = np.array([c + s_ * down_cam, -right_cam, -(-s_ + c * down_cam)])
        vw = est.R @ (v / (float(np.linalg.norm(v)) or 1.0))
        el = math.asin(max(-1.0, min(1.0, float(vw[2]))))
    set_gate_elevation(el, t, committed=_VIS["latched"], range_m=(det.range_m if fresh else None))


def _dr_trace(t, est, truth, truth_v):
    """Sim-only: estimate vs truth at 100 Hz, for finding where drift comes from."""
    _DR["n"] += 1
    if _DR["n"] % 10:
        return
    if _DR["fh"] is None:
        from raceline.config import AIGP_REPO
        from raceline.planner import next_numbered
        path = next_numbered(str(AIGP_REPO / "out" / "flightlogs" / "dr_XXX.csv"))
        path.parent.mkdir(parents=True, exist_ok=True)
        _DR["fh"] = open(path, "w", encoding="utf-8")
        _DR["fh"].write("t,ev,x,y,z,tx,ty,tz,vx,vy,tvx,tvy,fixes,rej,unm,lm,res,det_area,det_range,"
                        "cross_ev,cross_lat,cross_dz,fix_cx_sum,fix_al_sum,reason" + chr(10))
        print(f"[RACELINE] estimator trace -> {path}")
    lm = _SOURCE.last_landmark
    _DR["fh"].write(f"{t:.2f},{_SOURCE.next_event},{est.p[0]:.3f},{est.p[1]:.3f},{est.p[2]:.3f},"
                    f"{truth[0]:.3f},{truth[1]:.3f},{truth[2]:.3f},{est.v[0]:.3f},{est.v[1]:.3f},"
                    f"{truth_v[0]:.3f},{truth_v[1]:.3f},{_SOURCE.fixes},{_SOURCE.rejected},{_SOURCE.unmatched},"
                    f"{'' if lm is None else lm},{_SOURCE.fix_residual:.3f},"
                    f"{'' if _DR['det'] is None else '%.4f' % _DR['det'].area_frac},"
                    f"{'' if _DR['det'] is None or not _DR['det'].range_m else '%.2f' % _DR['det'].range_m},"
                    # the no-truth debrief columns, identical to the hardware log
                    f"{_SOURCE.cross_ev},{_SOURCE.cross_lat:.3f},{_SOURCE.cross_dz:.3f},"
                    f"{_SOURCE.fix_cross_sum:.3f},{_SOURCE.fix_along_sum:.3f},"
                    f"{_SOURCE.last_reason}" + chr(10))
    if _DR["n"] % 1000 == 0:
        _DR["fh"].flush()


def autopilot(update: SensorUpdate) -> RCCommand:
    t = update.t
    if t < T_DISARMED_END:
        return RCCommand(arm=1000, throttle=1000, aux2=AUX2)
    if t < T_ARM_IDLE_END:
        return RCCommand(arm=1800, throttle=1000, aux2=AUX2)

    est = _SOURCE.estimate(update)
    if STATE_SOURCE == "deadreckon":
        # the camera (30 Hz): ONE unlabeled detection of whatever ring is
        # biggest in view, one frame old, with dropouts, noise and false
        # positives; the estimator decides which gate it is and fixes on it.
        # The same observe() runs on the Orin with the real detector.
        _POSES.push(t, update.world_pos)
        if t - _FIX["t_det"] >= 1.0 / 30.0:
            _FIX["t_det"] = t
            dets = _cam.detect_all(_POSES.at_delay(t), _GATE_LANDMARKS, t)
            _DR["det"] = dets[0] if dets else None
            # NO POSITION FIXES ON A COMMITTED GATE - the same rule as
            # hardware.runtime (d43 race_006/007): a ring that fills the
            # frame has no centre.
            if dets and not _VIS["latched"]:
                idx, res = _SOURCE.observe_any(dets, _GATE_LANDMARKS)
                if idx is not None:
                    _FIX["last_res"] = res
                    _FIX["n"] += 1
                    _VIS["t_match"] = t
                    est = _SOURCE.last_est
                    est.p[:] = _SOURCE.p
                    est.v[:] = _SOURCE.v
        # VERTICAL FROM VISION IN THE SIM, mirroring hardware.runtime so a sim
        # flight exercises the hardware vertical channel and its commit
        # logic (2026-09-22: until now the sim flew the barometer and could
        # not validate the race-day path at all). Same elevation math as
        # hardware.hover.gate_dz on the synthetic camera's tan-unit offsets;
        # same commit latch (3 ticks, map within 6 m); commit by apparent
        # size = the ring spanning COMMIT_FRAC of the frame height.
        if VERT_VISION:
            _sim_vision_vertical(t, est)
        # truth is read here for the LOG ONLY (the DR err column and the
        # sim-only estimator trace out/flightlogs/dr_NNN.csv)
        truth = np.asarray(update.world_pos[4:7], dtype=float)
        truth_v = np.asarray(update.world_vel[3:6], dtype=float)
        _state["tvz"] = float(truth_v[2])
        _FIX["err"] = float(np.hypot(est.p[0] - truth[0], est.p[1] - truth[1]))
        _dr_trace(t, est, truth, truth_v)
    # the gate index: the referee's in the sim, the estimator's own count under
    # dead reckoning (the drone has no referee); the trace logs the referee's
    next_event = _SOURCE.next_event if STATE_SOURCE == "deadreckon" else update.next_gate_index
    return step(update.t, est, next_event, update.baro_fresh, _motor_stats(update))


def _count_missed_gate(ev: int) -> None:
    """Advance the estimator's own gate count past a gate we flew by."""
    if STATE_SOURCE == "deadreckon" and hasattr(_SOURCE, "next_event"):
        _SOURCE.next_event = max(int(_SOURCE.next_event), int(ev) + 1)


def commit_aligned(p, yaw: float, event) -> bool:
    """May we COMMIT to this gate? Only on a real approach: heading within
    COMMIT_ALIGN_DEG of the crossing heading and within COMMIT_ALIGN_LAT_M of
    the gate's centre line. Sim race_051: g5's ring filled the frame during a
    fly-by 3 m off its line, heading the wrong way (the hairpin); a commit
    there freezes the lateral loop exactly when the plan needs it. event =
    (x, y, z, heading_rad)."""
    gx, gy, _gz, gh = event[0], event[1], event[2], event[3]
    if gh is None:
        return True
    dyaw = (float(yaw) - float(gh) + math.pi) % (2 * math.pi) - math.pi
    if abs(dyaw) > math.radians(COMMIT_ALIGN_DEG):
        return False
    lx, ly = -math.sin(gh), math.cos(gh)
    lat = (float(p[0]) - gx) * lx + (float(p[1]) - gy) * ly
    if abs(lat) > COMMIT_ALIGN_LAT_M:
        return False
    # ...and the gate must be AHEAD by a real approach distance. The stacked
    # gate (sim race_061): 0.3 s after the top crossing, 0.5 m past the stack
    # and mid-turn, the nose swung through north and the LOW opening - same
    # x, y - passed heading and lateral at 0.74 m range; fixes froze for the
    # 8 s reversal, the estimate drifted 0.6 m, and he hit the low opening's
    # side while his estimate read dead centre.
    ahead = (gx - float(p[0])) * math.cos(gh) + (gy - float(p[1])) * math.sin(gh)
    return ahead >= COMMIT_MIN_AHEAD_M


def commit_level_ok(el_rad, range_m) -> bool:
    """May we COMMIT yet, vertically? A SIZE commit (policy, ~3.7 m) waits
    until the elevation reads level within COMMIT_LEVEL_M, so the height the
    hold freezes is the gate's height, not the tail of the climb-out (sim:
    g0 crossed at 1.89 and 2.43 m against a 2.10 m top edge because commit
    froze a descent still in progress). Inside COMMIT_FORCE_RANGE_M the ring
    clips both edges and the elevation is gone anyway - commit regardless."""
    if range_m is not None and range_m <= COMMIT_FORCE_RANGE_M:
        return True
    # ...and not moving vertically: the hold carries whatever vertical speed
    # exists at commit (sim race_069: level at 1.44 m but climbing 0.12 m/s,
    # 2.03 m at the plane)
    if abs(vz_inertial_trimmed()) > COMMIT_VZ_MAX:
        return False
    if el_rad is None:
        return True
    return abs(VERT_EL_GAIN * math.degrees(float(el_rad))) <= COMMIT_LEVEL_M


COMMIT_LEVEL_M = 0.12         # the elevation must read within this of level for a size commit
COMMIT_VZ_MAX = 0.10          # ...and the inertial vertical speed within this of zero
COMMIT_FORCE_RANGE_M = 3.2    # both edges clip here (2.7 m ring, 46.7 deg vertical field): commit regardless
COMMIT_ALIGN_DEG = 35.0
COMMIT_MIN_AHEAD_M = 1.5   # the gate must be at least this far ahead along its crossing direction
COMMIT_ALIGN_LAT_M = 1.0    # 1.5 -> 1.0: let the plan finish the hairpin's swing before the commit takes over


def step(t: float, est: StateEstimate, next_event: int, baro_fresh: bool = True,
         motor_stats: str = "") -> RCCommand:
    """One control tick from a state estimate: the tracker, the landing,
    the thrust budget, the sticks and the trace. Pure with respect to the
    sensor source, so the Orin runtime calls it with FC-fed estimates and
    the sim wrapper below calls it with the sim's."""
    a_des, z_target, vz_ff, yaw_des, done = _TRACKER.step(est, next_event, t)
    if AIM_AT_GATE and not done and 0 <= next_event < len(_TRACKER.event_xyz):
        gx, gy, _gz = _TRACKER.event_xyz[next_event]
        dxg, dyg = gx - float(est.p[0]), gy - float(est.p[1])
        if math.hypot(dxg, dyg) > AIM_HANDOFF_M:
            yaw_des = math.atan2(dyg, dxg)
    # COMMITTED = STRAIGHT (Brian, race day 2, 2026-09-22). Inside the commit
    # range there are no fixes, the height is frozen, and the only thing left
    # that could turn the aircraft is the estimator's idea of its lateral
    # position and velocity - the thing that was wrong in every strike this
    # week. So take it out of the loop: keep the tracker's acceleration
    # ALONG THE NOSE (speed regulation), zero it ACROSS the nose. Zero roll,
    # pitch forward, nose already on the gate from the yaw policy. The
    # aircraft flies the line it was on at commit, which the fixes had held
    # to within 0.2 m every approach. AIGP_COMMIT_STRAIGHT=0 restores the
    # dead-reckoned lateral loop.
    # FINAL APPROACH = THE GATE'S CENTRE LINE, NOT THE PLAN'S SPLINE (2026-09-22):
    # the plan does its 1.2 m jog to g1's line in the last 1.5 m, so a commit
    # could not be taken until 1.9 m out and the steer then crossed 0.77 m off.
    # Within FINAL_APPROACH_M of a gate that is ahead and aligned, the lateral
    # target is the gate centre's crossing line; the along component stays
    # the tracker's. Committed runs use the same law with no fixes.
    final_approach = False
    if VERT_VISION and not done and 0 <= next_event < len(_TRACKER.event_xyz):
        _gx, _gy, _ = _TRACKER.event_xyz[next_event]; _gh = _TRACKER.event_heading[next_event]
        if _gh is not None:
            _ahead = (_gx - float(est.p[0])) * math.cos(_gh) + (_gy - float(est.p[1])) * math.sin(_gh)
            _dyaw = (float(est.yaw) - float(_gh) + math.pi) % (2 * math.pi) - math.pi
            final_approach = 0.0 < _ahead <= FINAL_APPROACH_M and abs(_dyaw) <= math.radians(COMMIT_ALIGN_DEG)
    if COMMIT_STRAIGHT and VERT_VISION and (_COMMITTED or final_approach) and not done:
        fwd = np.array([math.cos(float(est.yaw)), math.sin(float(est.yaw))])
        along = float(np.dot(np.asarray(a_des, dtype=float)[:2], fwd))
        across = 0.0
        # ...BUT CENTRE ON THE GATE, GENTLY, ON DEAD RECKONING (sim race_046):
        # the FLAT plan does its 1.2 m jog from g0's line to g1's in the LAST
        # 1.5 m before g1, so a straight run from the commit point crossed g1
        # 0.88 m left - outside the opening. The estimate is smooth inside
        # the commit range (no fixes, inertial only; DR err 0.04-0.2 m in the
        # sim) so steering on it is not "being thrown by a bad signal": aim
        # at the gate centre's crossing line, clamped to a 3.5 deg lean.
        if 0 <= next_event < len(_TRACKER.event_xyz) and next_event < len(_TRACKER.event_s):
            gx, gy, _gz = _TRACKER.event_xyz[next_event]
            gh = _TRACKER.event_heading[next_event] if hasattr(_TRACKER, "event_heading") else None
            if gh is not None:
                nx, ny = math.cos(gh), math.sin(gh)          # crossing direction
                lx, ly = -ny, nx                             # left of the crossing line
                lat = (float(est.p[0]) - gx) * lx + (float(est.p[1]) - gy) * ly   # + = left of centre
                vlat = float(est.v[0]) * lx + float(est.v[1]) * ly
                a_lat = -COMMIT_LAT_KP * lat - COMMIT_LAT_KD * vlat                # toward the line
                a_lat = max(-COMMIT_LAT_MAX, min(COMMIT_LAT_MAX, a_lat))
                across = a_lat
                a_des = fwd * along + np.array([lx, ly]) * a_lat   # world frame; the sticks map it
        if across == 0.0:
            a_des = fwd * along

    if done or _state["done_t"] is not None:
        # LATCHED: once the last crossing is credited the race is over. The
        # old 1 s idle-and-disarm dropped the drone from 1.1 m; on the ground
        # the tracker's recovery branch took over and slid it into g0's frame
        # (race_143, contact 2.7 s after the finish). Hold the park point and
        # descend at LAND_RATE_MPS, disarm on the deck.
        if _state["done_t"] is None:
            _state["done_t"] = t
        dt_done = t - _state["done_t"]
        f = CFG.follower
        park = _TRACKER.pos[-1]
        a_des = f.kp_pos * (park[:2] - est.p[:2]) - f.kd_pos * est.v[:2]
        z_target = max(0.0, float(park[2]) - LAND_RATE_MPS * dt_done)
        vz_ff = -LAND_RATE_MPS if z_target > 0.0 else 0.0
        # NEVER DISARM ON THE BAROMETER UNDER VISION (race day 2). est.p[2]
        # read -0.67 m on the pad and -0.33 m in the air this week; an auto
        # disarm 0.5 s after the finish on that number is a drop from gate
        # height. Under vision the descent is a -1 m/s velocity hold and the
        # pilot takes the landing (card: MSP override off after the last gate).
        if est.p[2] < 0.10 and dt_done > 0.5 and not VERT_VISION:
            return RCCommand(arm=1000, throttle=1000, aux2=AUX2)

    # AIRBORNE LATCHES (d44, race day 1, 2026-09-21). AltitudeLoop returns an
    # OPEN-LOOP takeoff_pwm whenever `not airborne`, and airborne was recomputed
    # every tick from est.p[2] against min_alt_translation_m = 0.0. The height
    # estimate swings through zero under prop wash - flown: +1.60, -0.30, +1.52,
    # -1.44, +1.64, -0.85 in consecutive seconds - so the controller was thrown
    # out of closed loop and back into TAKEOFF several times a second, each time
    # commanding 1250 PWM with no feedback. That is a limit cycle: she bounced
    # off the ground and reclimbed to gate height repeatedly, and `air=False`
    # appears with `thr=1250` on every bounce in the log.
    #
    # An aircraft that has left the ground has left it. Latch it: once true it
    # stays true for the run, so a lying barometer can no longer re-arm the
    # takeoff branch. reset_state() clears it between runs.
    # LATCH ON A REAL CLIMB, NOT ON A HEIGHT (d44, 2026-09-21). The first
    # version of this latched on `est.p[2] >= min_alt_translation_m` alone, and
    # min_alt_translation_m is 0.0. Flight 2 that day STARTED with the
    # barometer reading +0.20 m sitting on the pad, so the latch fired on tick
    # one, `airborne` was true before the aircraft moved, and the open-loop
    # takeoff ramp was skipped entirely - the closed loop got the pad at hover
    # PWM. A height cannot decide this on an airframe whose barometer is the
    # thing we do not trust.
    #
    # A CLIMB can. Require both, the same test hardware/runtime.py already uses
    # for the dead-reckoning latch: above the threshold AND actually going up.
    # Nothing on the pad produces a sustained +0.5 m/s. Once latched it stays
    # latched, so prop wash cannot re-arm the takeoff branch mid-flight, which
    # is the bounce limit cycle flights 1-3 flew (thr=1250 at z=-0.45, -0.79,
    # -1.67). reset_state() clears it between runs.
    if (not _state["airborne_latch"]
            and float(est.p[2]) >= CFG.follower.min_alt_translation_m
            and float(est.v[2]) > 0.5):
        _state["airborne_latch"] = True
    airborne = _state["airborne_latch"]
    if VERT_VISION:
        # est.p[2] cancels out of the loop's error term, so the barometer is
        # not in the vertical channel at all. See the note at VERT_VISION.
        dz_vis, _el = _vision_dz(t)
        z_target = float(est.p[2]) + dz_vis
        # NO BAROMETER IN THE VERTICAL CHANNEL AT ALL (Brian, race day 2).
        # The P term is the gate's elevation; the D term was the baro-fused
        # vz, and on a spiky barometer that term alone slammed the throttle
        # 1128 <-> 1288 on the APPROACH (sim race_055, before commit). Damp on
        # the estimator's leaky inertial vz instead: accelerometer, bias
        # learned on the pad, 5 s leak. The same number damps the committed
        # hold, and because it is never reset it carries whatever vertical
        # speed we had at commit - the earlier reset-to-zero could not see a
        # climb that was already under way (race_055 again, 0.4 m/s into the
        # top bar with the throttle sitting still).
        # (A pull toward a vision-derived vertical speed - the rate of the
        # measured height offset while a gate is in view - was tried here on
        # 2026-09-22 and taken out the same hour: instrumented, that speed
        # read +2.4 / -1.5 m/s against a truth of +0.35 / +0.2, and pulling
        # the inertial estimate toward it put him into the floor between g0
        # and g1. The pure inertial vz tracked the truth within 0.1 m/s on the
        # same run. Range x sin(elevation) at 30 Hz is too noisy to
        # differentiate; the accelerometer is not.)
        vz_i = vz_inertial_trimmed() if hasattr(_SOURCE, "vz_inertial") else float(est.v[2])
        est = StateEstimate(p=est.p, v=np.array([float(est.v[0]), float(est.v[1]), vz_i]),
                            R=est.R, yaw=est.yaw, omega=est.omega)
        # THE PLAN'S vz IS A TAKEOFF CLIMB, NOT A REFERENCE (d43 race_005,
        # 2026-09-22, into g0's top bar). The plan starts on the ground and
        # climbs 0.23 -> 1.35 m into g0 at +0.13..+0.18 m/s. Under vision the
        # aircraft is ALREADY level with g0 when the tracker releases, so that
        # feedforward is a pure climb the elevation term has to fight - and at
        # COMMIT the term snaps to zero and the loop becomes a velocity hold
        # on +0.13 m/s: +0.35 m over the last 2.5 m, measured. The gate owns
        # the vertical here; the plan's vz is only kept for the landing.
        # ...EXCEPT WHEN VISION HAS NOTHING AND THE PLAN HAS A VERTICAL PROFILE
        # (the stacked gate, 2026-09-22): heading back north 1.7 m from the
        # stack the low opening is 58 deg below the camera, out of view, and a
        # zero feedforward would hold 4 m and arrive 2.7 m too high. With no
        # fresh elevation the plan's vz runs the blind descent (damped on the
        # inertial vz, no barometer); the low opening's elevation takes over
        # the moment it is in view. On the FLAT stretches the plan's vz is ~0
        # so nothing changes there.
        el_live = _GATE_EL is not None and (t - _GATE_EL[0]) <= VERT_EL_STALE_S
        if not done and (el_live or _COMMITTED or abs(vz_ff) < 0.3):
            vz_ff = 0.0
    throttle = _ALT.throttle(t, est, z_target, vz_ff, airborne,
                             baro_fresh,
                             (AZ_FF_GAIN * getattr(_TRACKER, 'az_ff', 0.0)) if getattr(_TRACKER, 'in_fold', False) else 0.0)
    # THRUST-VECTOR BUDGET, vertical first (Brian, race_044): the motors
    # make T_MAX of specific thrust in total. The altitude loop states its
    # vertical need (g + a_cmd); the horizontal gets what is left inside
    # THRUST_BUDGET_SHARE * T_MAX, and the tilt clamp (75 deg) only bounds
    # it further. On a level straight that is ~73 deg and 11 m/s; when
    # altitude needs thrust (climb, stack pull-out, a sagging corner) the
    # tilt pulls back by itself. At a fixed 75 deg clamp the horizontal
    # alone took 36.6 of 37.5 and the drone sank 1.44 -> 0.58 m into g5.
    if airborne and THRUST_BUDGET_SHARE > 0.0:
        th_z = 9.81 + float(_ALT.a_cmd)
        t_cap = THRUST_BUDGET_SHARE * float(max(CFG.thrust.curve_acc))
        h_cap = math.sqrt(max(t_cap * t_cap - th_z * th_z, 0.0))
        v_h = float(np.hypot(est.v[0], est.v[1]))
        if v_h > 0.5:
            u = est.v[:2] / v_h
            n_ = np.array([-u[1], u[0]])
            cross = max(-h_cap, min(h_cap, float(np.dot(a_des[:2], n_))))
            room = math.sqrt(max(h_cap * h_cap - cross * cross, 0.0))
            along = max(-room, min(room, float(np.dot(a_des[:2], u))))
            a_des = np.array([*(along * u + cross * n_), *a_des[2:]]) if len(a_des) > 2 else along * u + cross * n_
        else:
            nrm = float(np.hypot(a_des[0], a_des[1]))
            if nrm > h_cap:
                a_des = a_des * (h_cap / nrm)
    # No attitude demand the motors cannot honour (race_056): through the
    # stack's drop the throttle sits at idle while the follower still asks
    # 30-60 deg of tilt for the reversal push; the mixer makes lift out of
    # the roll/pitch corrections (motor mean 0.1-0.4 at throttle 1000) and
    # the drone falls at half of g. Below hover the horizontal demand
    # scales with the collective the mixer has; at idle it is zero and the
    # drone falls at g; the full push returns with the throttle.
    if airborne and ATT_IDLE_SCALE:
        span = float(CFG.thrust.hover_pwm - 1000)
        frac = max(0.0, min(1.0, (float(throttle) - 1000.0) / max(span, 1.0)))
        if frac < 1.0:
            a_des = a_des * frac
    # COHERENT THRUST VECTOR (race_057): the throttle is the magnitude of the
    # vector the follower wants, (a_h, g + a_z) with a_z clipped at 85% of
    # free fall, and the tilt is its angle. Through the stack's drop that is
    # ~10 m/s^2 at ~75 deg: the drone still falls at ~8 m/s^2 AND keeps its
    # horizontal authority, riding on a real collective instead of idle.
    # Idle throttle with zero demand fell cleanly but drifted 2 m off the
    # line with no way to correct (race_057, stack 2.5 s); idle throttle
    # with a demand made lift out of the corrections (race_056, float).
    az_eff = float(_ALT.a_cmd)
    if airborne and THRUST_VECTOR_MODE:
        az_eff = max(az_eff, -VECTOR_FREEFALL_SHARE * 9.81)
        gz = 9.81 + az_eff
        a_h = float(np.hypot(a_des[0], a_des[1]))
        # tilt cap in descent: a wrong-direction horizontal demand at 80 deg
        # is a sideways shove at pull-out (races 046, 051)
        # the cap applies while DESCENDING only (race_059: applied always it
        # held the straights to 17 m/s^2 of drag and 8 m/s)
        # ...and only in a REAL descent (vertical thrust below half of hover):
        # any slightly negative altitude demand on a straight (riding a few
        # cm high) put the cap on at a reduced gz and held the drag
        # feedforward to 13 m/s^2 - race_060 lost 1.5 m/s on every straight
        max_h = gz * math.tan(math.radians(VECTOR_TILT_MAX_DEG)) if gz < 0.5 * 9.81 else float("inf")
        if a_h > max_h and a_h > 1e-6:
            a_des = a_des * (max_h / a_h)
            a_h = max_h
        t_mag = math.sqrt(a_h * a_h + gz * gz)
        throttle = int(round(CFG.pwm_for_thrust(t_mag)))
    # COMMITTED = HOLD HOVER THROTTLE (race day 2, sim race_042 with ANGLE
    # finally flying in the SITL). After commit the vertical was a velocity
    # hold on the barometer's vz. A spiky baro read +1.5 / -1.5 / +1.5 m/s in
    # one second while the true height moved 0.1 m; the loop chased it,
    # throttle 1095 <-> 1348, and the true height ratcheted 1.44 -> 2.95 m
    # into g0's top bar - race_005's strike, reproduced. So: latch the MEAN
    # commanded throttle of the last second before commit (the loop's own
    # hover point at this speed and tilt) and hold it, with only a light,
    # low-passed vz damping against a real drift. No barometer step can move
    # it more than a few us. The tilt is computed against level thrust.
    # NO REFERENCE = HOLD, TOO (sim race_046): after g1 the nose swings 90 deg
    # to g2, no gate is in view for a few seconds, and the velocity hold on
    # the barometer took him from 2.2 to 4.8 m in three seconds. Whenever
    # vision has no fresh elevation - committed OR gate lost - hold the
    # throttle the same way.
    # (The gate-lost variant was flown in the sim and taken out again the
    # same hour: the detection flickers, every flicker re-latched a new mean
    # that included a climb, and he rose to 33 m. Hold on COMMIT only; a lost
    # gate fades the elevation reference to zero as before.)
    # AND OFF BY DEFAULT FOR RACE DAY 2. In the sim the latched throttle
    # drifted in two runs of three (the SITL's hover point wanders with its
    # loop churn); on the aircraft race_006 and race_007 both held height
    # flat through the commit range on the velocity hold with the plan's
    # climb feedforward now zeroed. Two hardware data points beat none.
    # AIGP_COMMIT_HOLD=1 turns the throttle hold on.
    # RACE DAY 2, FINAL: the hold is ON and BARO-FREE. Its damping is the
    # estimator's vz_inertial - bias-corrected accelerometer integrated from
    # the moment of commit - not the barometer's vz. Brian: "we can't rely on
    # baro, we have already determined that."
    hold_now = _COMMITTED and COMMIT_HOLD
    if VERT_VISION and airborne and not done:
        hist = _state["thr_hist"]
        # the low-passed vertical speed runs ALL the time, so at the moment of
        # commit it already knows whether we are still climbing out (sim
        # race_043: commit at 3.6 m mid climb-out latched a climbing throttle
        # and a 30 us/(m/s) damping let him rise 17 m over the gate)
        dt_h = max(0.0, min(0.2, t - _state["hold_t"]))
        _state["hold_t"] = t
        _state["vz_lp"] += (float(est.v[2]) - _state["vz_lp"]) * min(1.0, dt_h / HOLD_VZ_TAU_S)
        if not hold_now:
            hist.append((t, int(throttle)))
            while hist and hist[0][0] < t - 1.0:
                hist.popleft()
            _state["hold_thr"] = None
        else:
            if _state["hold_thr"] is None:
                vals = [v for _, v in hist] or [int(throttle)]
                _state["hold_thr"] = float(sum(vals)) / len(vals)
                # THE HOLD IS A HEIGHT HOLD, NOT A SPEED HOLD (sweep 2026-09-22,
                # build bec1096: seed 2 committed to g0 at 3.0 m descending
                # from the climb-out overshoot, 0.14 m high; damping the
                # inertial vz to zero held a +0.15 m/s TRUE climb for 3 s on a
                # -0.17 m/s offset and he rose into the top bar. Seed 1 rose
                # into the stack's top frame on a yaw-torque lift the speed
                # hold could not see). The inertial vz carries an unknown
                # constant offset; damping it to any fixed number is a guess.
                # So: fit the last 1.5 s of elevation geometry for the TRUE
                # vertical speed and the height still to make, take the
                # offset as (inertial - true), integrate the corrected vz
                # from commit as the height flown, and fly that height to the
                # target at the vertical loop's own gains and cap.
                _vzi_now = vz_inertial_trimmed()
                fit = _fit_vision_vz(t) if HOLD_HEIGHT else None
                _el_level = (_GATE_EL is not None and (t - _GATE_EL[0]) <= 1.0
                             and abs(VERT_EL_GAIN * math.degrees(float(_GATE_EL[1]))) <= COMMIT_LEVEL_M)
                if fit is not None and abs(fit[2]) > HOLD_FIT_MAX_OFFSET or fit is not None and abs(fit[1]) > HOLD_FIT_MAX_DZ:
                    # the two disagree, or we are nowhere near level: this is
                    # the forced mid-climb commit at the stack, where the fit
                    # read 0.43 m/s against 0.83 inertial and the height hold
                    # overshot the top opening by 1.1 m (seed 0, 2026-09-22).
                    # The inertial speed is trusted to ~0.1 m/s now that it
                    # integrates from arming; the old rule takes over.
                    how_fit = f"fit rejected: vision vz {fit[0]:+.2f}, offset {fit[2]:+.2f}, height {fit[1]:+.2f}; "
                    fit = None
                else:
                    how_fit = ""
                if fit is not None:
                    vz_vis, dz0, b, nfit, span = fit
                    b = b - (_VZ_TRIM["off"] if VZ_VISION_TRIM else 0.0)   # the fit's offset is of the RAW speed; the hold runs on the trimmed one
                    how = f"vision vz {vz_vis:+.2f} from {nfit} samples over {span:.1f} s"
                else:
                    # no usable window: the old rule (level -> the reading is
                    # offset; else -> the reading is real) and no height to make
                    b = _vzi_now if (_el_level or abs(_vzi_now) < 0.3) else 0.0
                    dz0 = 0.0
                    how = how_fit + "no elevation window: " + ("level, reading zeroed as offset" if b != 0.0 else "mid-climb, damped to zero")
                _state["hold_b"] = float(b)
                _state["hold_dz"] = max(-HOLD_DZ_MAX_M, min(HOLD_DZ_MAX_M, float(dz0)))
                _state["hold_z"] = 0.0
                _state["hold_tp"] = t
                print(f"[RACELINE] COMMIT: holding throttle {_state['hold_thr']:.0f} "
                      f"(mean of {len(vals)} ticks), inertial vz {_vzi_now:+.2f} m/s, offset {b:+.2f}, "
                      f"height to make {_state['hold_dz']:+.2f} m ({how})")
            vz_i = (vz_inertial_trimmed() if hasattr(_SOURCE, "vz_inertial") else _state["vz_lp"]) - _state["hold_b"]
            dt_p = max(0.0, min(0.2, t - _state["hold_tp"]))
            _state["hold_tp"] = t
            _state["hold_z"] += vz_i * dt_p
            e_z = max(-HOLD_DZ_MAX_M, min(HOLD_DZ_MAX_M, _state["hold_dz"] - _state["hold_z"]))
            _f = getattr(CFG, "follower", None)
            _ratio = float(getattr(_f, "kp_z", 9.0)) / max(1e-3, float(getattr(_f, "kd_z", 4.0)))
            _cap = VERT_DZ_MAX_TAKEOFF if _past_g0() is False else VERT_DZ_MAX
            vz_cmd = max(-_ratio * _cap, min(_ratio * _cap, _ratio * e_z)) if HOLD_HEIGHT else 0.0
            throttle = int(round(_state["hold_thr"] + HOLD_KD_PWM * (vz_cmd - vz_i)))
            throttle = max(int(CFG.thrust.pwm_min), min(int(CFG.thrust.pwm_max), throttle))
            az_eff = 0.0
    roll = pitch = yaw_stick = 1500
    if airborne:
        if ANGLE_MODE:
            roll, pitch, eb = angle_sticks(CFG, est, a_des, az_eff)
        else:
            roll, pitch, eb = attitude_sticks(CFG, est, a_des, az_eff)
        yaw_stick = _YAW.stick(est, yaw_des)
        # No yaw demand when the motors cannot deliver it: through the
        # stack the throttle sits at minimum for ~1 s and the yaw loop kept
        # 60-140 PWM on the stick for a 30 deg error it could never close.
        # Betaflight then lifts two motors to make the torque, the mean
        # motor output stays at 0.2-0.4 instead of idle, and the drone
        # hangs 0.6 s at 3.7 m (race_039-041). Below hover minus
        # YAW_IDLE_BAND the stick stays centred; the heading is held
        # again as soon as there is throttle to hold it with.
        if throttle < CFG.thrust.hover_pwm - YAW_IDLE_BAND:
            yaw_stick = 1500
    cos_tilt = float(est.R[2, 2])
    _state["trace_n"] += 1
    if _state["trace_n"] == 1:
        _state["trace"] = _trace_open()
    fh = _state["trace"]
    if fh is not None and _state["trace_n"] % TRACE_EVERY == 0:
        w = est.omega if est.omega is not None else (0.0, 0.0, 0.0)
        fh.write(f"{t:.3f},{_TRACKER.s[_TRACKER.idx]:.2f},"
                 f"{next_event},"
                 f"{est.p[0]:.3f},{est.p[1]:.3f},{est.p[2]:.3f},"
                 f"{est.v[0]:.3f},{est.v[1]:.3f},{est.v[2]:.3f},"
                 f"{z_target:.3f},{vz_ff:.3f},{a_des[0]:.2f},{a_des[1]:.2f},"
                 f"{cos_tilt:.3f},{w[0]:.2f},{w[1]:.2f},"
                 f"{_ALT.a_cmd:.2f},{_ALT.thrust:.2f},"
                 f"{roll},{pitch},{throttle},{yaw_stick},"
                 f"{motor_stats}\n")
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
              f"air={airborne} stk=({roll},{pitch},{throttle},{yaw_stick})"
              + (f" DR err={_FIX['err']:.2f}m fixes={_FIX['n']} rej={_SOURCE.rejected} unm={_SOURCE.unmatched} res={_FIX['last_res']:.2f} vzi={float(getattr(_SOURCE, 'vz_inertial', 0.0)):+.2f} vzt={vz_inertial_trimmed():+.2f} vvis={_fit_dbg():s} azw={float(getattr(_SOURCE, 'az_w_last', 0.0)):+.2f} dt={float(getattr(_SOURCE, 'dt_last', 0.0)):.4f} tvz={float(getattr(_state, 'tvz', 0.0)) if False else _state.get('tvz', float('nan')):+.2f}" if STATE_SOURCE == "deadreckon" else ""))

    return RCCommand(arm=1800, throttle=throttle, roll=roll, pitch=pitch,
                     yaw=yaw_stick, aux2=AUX2)
