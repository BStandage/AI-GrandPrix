"""Racing-line optimizer: move the line WITHIN the gate openings for
minimum predicted lap time.

What it optimizes: 24 numbers, one per crossing event - the lateral
offset (along the gate bar) where the line passes through that opening.
Bounds are ONE global rule, [optimizer] apex_max_m, identical at every
gate. The cost of a candidate line is the predicted time from the same
speed profiler the plain planner uses, so trades like "enter wider,
carry more speed" win exactly where the physics says they pay.

RESTRICTIONS note: this is an offline solve from the map + global vehicle
limits, the sanctioned "survey -> map -> solve -> fly" step. No flight
results feed back into it, and the offsets are solver OUTPUT recomputed
from scratch for any course - not hand-tuned per-gate constants. Run it
on an unseen map and it produces that course's line in ~a minute.

Usage:
    python -m raceline.line_opt              # optimize, report, write plan
    race.py --optimize                       # same, then fly it


EXPERIMENTAL (2026-09-09): the anchor/pose optimizers in this module (optimize_anchors,
optimize_poses) are NOT used for plan_RACE. On 2026-09-09 every run of them produced a
line that was slower or less flyable than the rule builder in planner.py (tighter g7
loops, kinked joints), and the model time they minimise disagreed with the follower
replay in sign on every loop. Keep for research; validate any output with the follower
replay before flying it.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import replace
from typing import List, Optional, Sequence

import numpy as np

from raceline import course as course_bridge
from raceline import planner
from raceline.config import AIGP_REPO, VehicleConfig, load_config

_OPT_DENSE_DS = 0.15   # coarser spline sampling inside the search loop
_MAXFEV = 4000


class OffsetCourse:
    """Read-only view of a RaceCourse with each crossing shifted along its
    gate-bar axis. Duck-types the parts of RaceCourse the planner uses."""

    def __init__(self, course, offsets: Sequence[float]):
        self._c = course
        evs = [course.event(i) for i in range(course.total_events)]
        self._events = []
        for e, u in zip(evs, offsets):
            bx = -math.sin(e.heading_rad)   # bar axis = heading rotated 90
            by = math.cos(e.heading_rad)
            self._events.append(replace(e, x=e.x + u * bx, y=e.y + u * by))

    @property
    def total_events(self):
        return self._c.total_events

    @property
    def laps(self):
        return self._c.laps

    @property
    def crossings(self):
        return self._c.crossings

    @property
    def gates(self):
        return self._c.gates

    @property
    def cones(self):
        return self._c.cones

    @property
    def source(self):
        return self._c.source

    @property
    def footprint_m(self):
        return self._c.footprint_m

    def event(self, i: int):
        return self._events[i]

    def lap_of(self, i: int):
        return self._c.lap_of(i)


def _race_time(cfg: VehicleConfig, course, offsets,
               dense_ds: float) -> float:
    """Predicted time at the LAST crossing for a candidate line (the park
    segment is excluded - it is not raced)."""
    oc = OffsetCourse(course, offsets)
    anchors, center_idx = planner.build_anchors(oc, cfg)
    dense, s_dense, adi = planner.sample_spline(anchors, dense_ds=dense_ds)
    total = float(s_dense[-1])
    n = max(2, int(total / cfg.planner.sample_ds_m) + 1)
    sg = np.linspace(0.0, total, n)
    P = np.column_stack([np.interp(sg, s_dense, dense[:, k])
                         for k in range(3)])
    centers = np.array([[e.x, e.y, e.z]
                        for e in (oc.event(i)
                                  for i in range(oc.total_events))])
    *_, t, _, _ = planner._speed_profile(P, sg, cfg, centers)
    s_last = float(s_dense[adi[center_idx[-1]]])
    # a frame contact crashes the real run: price it far above any time gain
    return (float(np.interp(s_last, sg, t))
            + 50.0 * planner.frame_violations(P, oc))


def optimize(cfg: VehicleConfig, course=None, maxfev: int = _MAXFEV,
             verbose: bool = True):
    """Search the per-crossing apex offsets for minimum predicted race time.

    Returns (offsets, plan) where plan is the FULL-resolution plan built on
    the optimized line and validated against the sim's crossing tracker.
    """
    from scipy.optimize import minimize

    if course is None:
        course = course_bridge.load_course(laps=cfg.planner.laps)
    n = course.total_events
    a = cfg.optimizer.apex_max_m

    t0 = _race_time(cfg, course, np.zeros(n), _OPT_DENSE_DS)
    if verbose:
        print(f"OPT   start: centered line predicts {t0:.2f} s "
              f"(coarse eval); {n} offsets, bounds +-{a} m")

    evals = [0]

    def cost(u):
        evals[0] += 1
        return _race_time(cfg, course, u, _OPT_DENSE_DS)

    res = minimize(cost, np.zeros(n), method="Powell",
                   bounds=[(-a, a)] * n,
                   options={"maxfev": maxfev, "xtol": 0.01, "ftol": 1e-3})
    u = np.clip(res.x, -a, a)
    if verbose:
        print(f"OPT   done: {evals[0]} candidate lines tried, "
              f"coarse time {t0:.2f} -> {res.fun:.2f} s")

    plan = planner.plan(cfg, OffsetCourse(course, u))
    plan.meta["optimizer"] = {
        "apex_max_m": a,
        "offsets_m": [round(float(x), 3) for x in u],
        "coarse_time_before_s": round(t0, 2),
        "coarse_time_after_s": round(float(res.fun), 2),
    }

    # Validity oracle: the optimized line must still cross every opening in
    # order and direction ON THE REAL (uncentered) course geometry.
    pq = course_bridge.pq_course()
    tracker = pq.RaceTracker(course)
    for t, pos in zip(plan.t, plan.pos):
        tracker.update(float(t), pos)
    if not tracker.complete:
        raise RuntimeError(
            f"optimized line fails the tracker replay "
            f"({tracker.events_passed}/{course.total_events}) - "
            "not writing it; lower apex_max_m")
    return u, plan


# ---------------------------------------------------------------------------
# Free-point optimization: let the line INVENT shapes between gates
# ---------------------------------------------------------------------------

def _build_anchors_free(course, cfg, offsets, free_pts):
    """Anchor chain like planner.build_anchors, but with ONE movable 3D
    point injected into every between-gate leg. free_pts is [(n_events)x3];
    row k is the free point on the leg AFTER event k (the last leg keeps
    none - it only parks)."""
    import math as m
    p = cfg.planner
    oc = OffsetCourse(course, offsets)
    events = [oc.event(i) for i in range(oc.total_events)]
    pre_d, post_d, _reversal = planner.standoffs(
        [(e.x, e.y) for e in events], [e.heading_rad for e in events],
        p.anchor_standoff_m, p.anchor_standoff_turn_m,
        m.radians(p.turn_angle_deg))

    anchors = [np.array([0.0, 0.0, p.takeoff_alt_m])]
    center_idx = []
    for k, e in enumerate(events):
        nv = np.array([m.cos(e.heading_rad), m.sin(e.heading_rad), 0.0])
        ctr = np.array([e.x, e.y, e.z])
        pre = ctr - pre_d[k] * nv
        if np.linalg.norm(pre - anchors[-1]) > 0.15:
            anchors.append(pre)
        center_idx.append(len(anchors))
        anchors.append(ctr)
        anchors.append(ctr + post_d[k] * nv)
        if k < len(events) - 1:
            anchors.append(np.asarray(free_pts[k], dtype=float))
    last = anchors[-1]
    anchors.append(np.array([last[0], last[1], planner.PARK_ALT_M]))
    return np.array(anchors), center_idx


def _leg_midpoints(course, cfg, offsets):
    """Straight-line initial guesses for the free points."""
    oc = OffsetCourse(course, offsets)
    events = [oc.event(i) for i in range(oc.total_events)]
    mids = []
    for k in range(len(events) - 1):
        a, b = events[k], events[k + 1]
        mids.append([(a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2])
    return np.array(mids)


def _race_time_free(cfg, course, offsets, free_pts, dense_ds):
    anchors, center_idx = _build_anchors_free(course, cfg, offsets, free_pts)
    dense, s_dense, adi = planner.sample_spline(anchors, dense_ds=dense_ds)
    total = float(s_dense[-1])
    n = max(2, int(total / cfg.planner.sample_ds_m) + 1)
    sg = np.linspace(0.0, total, n)
    P = np.column_stack([np.interp(sg, s_dense, dense[:, k])
                         for k in range(3)])
    oc = OffsetCourse(course, offsets)
    centers = np.array([[e.x, e.y, e.z]
                        for e in (oc.event(i)
                                  for i in range(oc.total_events))])
    *_, t, _, _ = planner._speed_profile(P, sg, cfg, centers)
    return (float(np.interp(float(s_dense[adi[center_idx[-1]]]), sg, t))
            + 50.0 * planner.frame_violations(P, oc))


def optimize_free(cfg, course=None, rounds: int = 2, verbose: bool = True,
                  seed_noise=None):
    """Two-block coordinate descent: apex offsets globally (Powell), then a
    per-leg sweep moving each free point (3 vars each, Powell), repeated.
    Free points let the line invent loops, spirals, and wide entries the
    fixed shape could never express; gates stay hard constraints."""
    from scipy.optimize import minimize

    if course is None:
        course = course_bridge.load_course(laps=cfg.planner.laps)
    n = course.total_events
    a = cfg.optimizer.apex_max_m

    u = np.zeros(n)
    fp = _leg_midpoints(course, cfg, u)
    if seed_noise is not None:
        rng, spread = seed_noise
        if spread > 0:
            fp = fp + rng.normal(0.0, spread, fp.shape)
            fp[:, 2] = np.clip(fp[:, 2], 0.9, 5.5)
    best = _race_time_free(cfg, course, u, fp, _OPT_DENSE_DS)
    if verbose:
        print(f"OPT2  start {best:.2f} s (free-point line, coarse)")

    for r in range(rounds):
        res = minimize(
            lambda x: _race_time_free(cfg, course, x, fp, _OPT_DENSE_DS),
            u, method="Powell", bounds=[(-a, a)] * n,
            options={"maxfev": 1500, "xtol": 0.01, "ftol": 1e-3})
        u = np.clip(res.x, -a, a)
        for k in range(len(fp)):
            base = fp[k].copy()
            lo = base - np.array([6.0, 6.0, 1.5])
            hi = base + np.array([6.0, 6.0, 4.0])
            lo[2] = max(lo[2], 0.8)

            def leg_cost(q, k=k):
                trial = fp.copy()
                trial[k] = q
                return _race_time_free(cfg, course, u, trial, _OPT_DENSE_DS)

            resk = minimize(leg_cost, fp[k], method="Powell",
                            bounds=list(zip(lo, hi)),
                            options={"maxfev": 120, "xtol": 0.05})
            if resk.fun < best:
                fp[k] = np.clip(resk.x, lo, hi)
                best = float(resk.fun)
        if verbose:
            print(f"OPT2  round {r + 1}: {best:.2f} s")

    # full-resolution plan on the winning line
    anchors, _ = _build_anchors_free(course, cfg, u, fp)
    plan = _plan_from_anchors(cfg, course, u, anchors)
    plan.meta["optimizer"] = {
        "mode": "free-point", "apex_max_m": a,
        "offsets_m": [round(float(x), 3) for x in u],
        "free_points": [[round(float(v), 2) for v in row] for row in fp],
        "coarse_time_s": round(best, 2),
    }
    pq = course_bridge.pq_course()
    tracker = pq.RaceTracker(course)
    for t, pos in zip(plan.t, plan.pos):
        tracker.update(float(t), pos)
    if not tracker.complete:
        raise RuntimeError(
            f"free-point line fails tracker replay "
            f"({tracker.events_passed}/{course.total_events})")
    return u, fp, plan


def _plan_from_anchors(cfg, course, offsets, anchors, center_idx=None):
    """Full-resolution Plan for an explicit anchor chain (planner.plan
    rebuilt around externally supplied anchors). center_idx: index of each
    event's center anchor. Pass it whenever you have it - the nearest-
    anchor fallback picks the LAP-1 anchor for a lap-2 gate (same XY), which
    put the last event before the first and emptied the report window."""
    import hashlib
    oc = OffsetCourse(course, offsets)
    events_list = [oc.event(i) for i in range(oc.total_events)]
    centers = np.array([[e.x, e.y, e.z] for e in events_list])
    if center_idx is None:
        center_idx = [int(np.argmin(np.linalg.norm(anchors - c, axis=1)))
                      for c in centers]
    dense, s_dense, adi = planner.sample_spline(anchors)
    total = float(s_dense[-1])
    n = max(2, int(total / cfg.planner.sample_ds_m) + 1)
    sg = np.linspace(0.0, total, n)
    P = np.column_stack([np.interp(sg, s_dense, dense[:, k])
                         for k in range(3)])
    (T, kappa, dpsi_ds, dkappa_ds, binding, v_lim, v, t, vel,
     acc) = planner._speed_profile(P, sg, cfg, centers)
    events = []
    for k, e in enumerate(events_list):
        s_ev = float(s_dense[adi[center_idx[k]]])
        events.append({"event": k, "lap": course.lap_of(k),
                       "label": course.event(k).label, "s": s_ev,
                       "t": float(np.interp(s_ev, sg, t)),
                       "v": float(np.interp(s_ev, sg, v)),
                       "x": round(e.x, 3), "y": round(e.y, 3),
                       "z": round(e.z, 3),
                       "heading_rad": round(float(e.heading_rad), 4)})
    map_p = course_bridge.map_path()
    meta = {
        "map_source": course.source,
        "map_sha1": hashlib.sha1(map_p.read_bytes()).hexdigest()
        if map_p.exists() else None,
        "config_path": str(cfg.path), "config_sha1": cfg.sha1,
        "params": cfg.raw, "laps": course.laps,
        "crossings_per_lap": len(course.crossings),
        "path_length_m": round(total, 2),
        "frame_violations": planner.frame_violations(P, oc),
    }
    return planner.Plan(s=sg, pos=P, vel=vel, acc=acc, t=t, v=v,
                        v_lim=v_lim, tangent=T, kappa=kappa,
                        dpsi_ds=dpsi_ds, dkappa_ds=dkappa_ds,
                        binding=binding, events=events, meta=meta)


def optimize_multistart(cfg, course=None, starts: int = 6, rounds: int = 2,
                        spread_m: float = 2.0):
    """Force the search out of its local minimum: run optimize_free from
    `starts` randomized free-point seeds (midpoints + noise), keep the best
    VALIDATED result. Deterministic per seed index, embarrassingly simple.
    """
    if course is None:
        course = course_bridge.load_course(laps=cfg.planner.laps)
    best = None
    for k in range(starts):
        rng = np.random.default_rng(1000 + k)
        try:
            u, fp, plan = optimize_free(cfg, course, rounds=rounds,
                                        verbose=False,
                                        seed_noise=(rng, spread_m if k else 0.0))
            t = plan.events[-1]["t"]
            print(f"START {k}: {t:.2f} s "
                  f"(contacts {plan.meta['frame_violations']})")
            if best is None or t < best[2].events[-1]["t"]:
                best = (u, fp, plan)
        except RuntimeError as e:
            print(f"START {k}: rejected ({e})")
    return best


# ---------------------------------------------------------------------------
# Anchor refinement: optimize the through-gate geometry itself
# ---------------------------------------------------------------------------

R_MIN_M = 2.0             # minimum flyable turn radius: the follower cannot
                          # track tighter arcs at any speed (flights 2026-09-09:
                          # 0.6-1.0 m wide in the ~1.5 m g7 loop, missed g7)
_RMIN_PENALTY_S = 0.3     # seconds per path sample tighter than R_MIN_M
CLEARANCE_R_M = 0.55      # drone radius the line must clear every frame with (0.15 + 0.40 m error; 0.30 held at g7 flew into g7):
                          # 0.15 real + 0.30 tracking error (measured 0.27-0.30)
_CLEAR_PENALTY_S = 0.5    # seconds per path sample inside that inflated frame


def _anchor_cost(cfg, course, anchors, center_idx, dense_ds, pq):
    dense, s_dense, adi = planner.sample_spline(anchors, dense_ds=dense_ds)
    total = float(s_dense[-1])
    n = max(2, int(total / cfg.planner.sample_ds_m) + 1)
    sg = np.linspace(0.0, total, n)
    P = np.column_stack([np.interp(sg, s_dense, dense[:, k])
                         for k in range(3)])
    centers = np.array([[e.x, e.y, e.z]
                        for e in (course.event(i)
                                  for i in range(course.total_events))])
    T, kappa, *_rest = planner._speed_profile(P, sg, cfg, centers)
    t = _rest[5]
    s_last = float(s_dense[adi[center_idx[-1]]])
    race = float(np.interp(s_last, sg, t))
    # minimum-radius constraint on the RACED part of the path (the park
    # run-out after the last crossing is excluded)
    raced = sg <= s_last
    tight = int(np.count_nonzero(kappa[raced] > 1.0 / R_MIN_M))
    race += _RMIN_PENALTY_S * tight
    hits = 0
    near = 0
    for pos in P:
        for g in course.gates:
            if pq.gate_frame_hit(g, pos):
                hits += 1
                break
            if pq.gate_frame_hit(g, pos, CLEARANCE_R_M):
                near += 1
                break
    return race + 50.0 * hits + _CLEAR_PENALTY_S * near


def optimize_anchors(cfg, course=None, rounds: int = 2, maxfev: int = 150,
                     verbose: bool = True, only_legs=None, anchors=None,
                     center_idx=None, starts: int = 1, start_spread: float = 1.0,
                     seed: int = 0):
    """Refine the planner's anchor chain: every anchor between two gate
    centers (stubs, bows, corner and clearance points) becomes a free 3D
    point and each leg is searched (Powell) for minimum race time. Gate
    centers stay fixed; the referee's own frame geometry prices contacts
    at 50 s and anything closer than CLEARANCE_R_M at 0.5 s per sample, so
    the search cannot buy time with margin. The crossing direction is no
    longer a rule - it is whatever the time-optimal geometry through the
    opening turns out to be, subject to that clearance. Seeded from the
    rule-built line (a good local basin), so this is refinement, not
    global search."""
    from scipy.optimize import minimize
    if course is None:
        course = course_bridge.load_course(laps=cfg.planner.laps)
    pq = course_bridge.pq_course()
    if anchors is None:
        anchors, center_idx = planner.build_anchors(course, cfg)
    anchors = np.array(anchors, dtype=float).copy()
    base = _anchor_cost(cfg, course, anchors, center_idx, _OPT_DENSE_DS, pq)
    if verbose:
        print(f"OPT3  seed {base:.2f} s (rule-built line, coarse, incl. penalties)")
    # Crossing points are variables too: each gate center may slide along
    # its bar by u_k (bounded by U_MAX). Without this the lane gates sit
    # 0.25 m off center by construction and can never reach the clearance
    # floor, so the search has nothing to trade there.
    n_ev = course.total_events
    ev_true = [course.event(i) for i in range(n_ev)]
    ctr_true = np.array([[e.x, e.y, e.z] for e in ev_true])
    bars = np.array([[-math.sin(e.heading_rad), math.cos(e.heading_rad), 0.0]
                     for e in ev_true])
    u = np.array([float(np.dot(anchors[center_idx[k]] - ctr_true[k], bars[k]))
                  for k in range(n_ev)])
    U_MAX = 0.75 - (CLEARANCE_R_M - 0.15)     # keep the floor reachable

    def _apply_u(arr, uu):
        for k in range(n_ev):
            arr[center_idx[k]] = ctr_true[k] + uu[k] * bars[k]
        return arr

    anchors = _apply_u(anchors, u)
    legs = []
    for j in range(len(center_idx) - 1):
        if only_legs is not None and j not in only_legs:
            continue
        idx = list(range(center_idx[j] + 1, center_idx[j + 1]))
        if idx:
            legs.append((j, idx))
    best = _anchor_cost(cfg, course, anchors, center_idx, _OPT_DENSE_DS, pq)
    for r in range(rounds):
        for j, idx in legs:
            x0 = np.concatenate([anchors[idx].ravel(), [u[j], u[j + 1]]])
            lo = np.concatenate([anchors[idx].ravel() - np.tile([3.0, 3.0, 1.0], len(idx)),
                                 [-U_MAX, -U_MAX]])
            hi = np.concatenate([anchors[idx].ravel() + np.tile([3.0, 3.0, 1.5], len(idx)),
                                 [U_MAX, U_MAX]])
            lo[2:len(idx) * 3:3] = np.maximum(lo[2:len(idx) * 3:3], 0.8)

            def cost(x, idx=idx, j=j):
                trial = anchors.copy()
                trial[idx] = x[:len(idx) * 3].reshape(-1, 3)
                uu = u.copy()
                uu[j], uu[j + 1] = x[-2], x[-1]
                trial = _apply_u(trial, uu)
                return _anchor_cost(cfg, course, trial, center_idx,
                                    _OPT_DENSE_DS, pq)

            # Per-leg MULTISTART: the leg's Powell search from its current
            # shape and from `starts - 1` jittered shapes (mid-leg anchors
            # only - the stubs next to each gate stay, so the crossing
            # direction and order survive). Keeps the best. This is what
            # gets a leg out of the basin its seed put it in (measured:
            # the north/south g7 topologies each won different legs by
            # 0.5 s, i.e. per-leg local minima, not topology).
            rng = np.random.default_rng(seed * 1000 + r * 100 + j)
            cands = [x0]
            mid = list(range(3, max(3, (len(idx) - 1) * 3)))   # skip first/last anchor
            for _ in range(max(0, starts - 1)):
                xj = x0.copy()
                if mid:
                    xj[mid] += rng.normal(0.0, start_spread, len(mid))
                    xj[2:len(idx) * 3:3] = np.clip(xj[2:len(idx) * 3:3], 0.8, 5.5)
                cands.append(np.clip(xj, lo, hi))
            res_best = None
            for xs in cands:
                res = minimize(cost, xs, method="Powell",
                               bounds=list(zip(lo, hi)),
                               options={"maxfev": maxfev, "xtol": 0.03,
                                        "ftol": 1e-3})
                if res_best is None or res.fun < res_best.fun:
                    res_best = res
            res = res_best
            if res.fun < best - 1e-4:
                x = np.clip(res.x, lo, hi)
                anchors[idx] = x[:len(idx) * 3].reshape(-1, 3)
                u[j], u[j + 1] = x[-2], x[-1]
                anchors = _apply_u(anchors, u)
                best = float(res.fun)
        if verbose:
            print(f"OPT3  round {r + 1}: {best:.2f} s")
    plan = _plan_from_anchors(cfg, course, u, anchors,
                              center_idx=list(center_idx))
    plan.meta["crossing_offsets_m"] = [round(float(x), 3) for x in u]
    plan.meta["anchors"] = [[round(float(v), 4) for v in a] for a in anchors]
    plan.meta["center_idx"] = [int(i) for i in center_idx]
    plan.meta["optimizer"] = {"mode": "anchor-refine",
                              "clearance_r_m": CLEARANCE_R_M,
                              "seed_cost_s": round(base, 2),
                              "final_cost_s": round(best, 2)}
    tracker = pq.RaceTracker(course)
    for t, pos in zip(plan.t, plan.pos):
        tracker.update(float(t), pos)
    if not tracker.complete or tracker.gate_contacts:
        raise RuntimeError(
            f"refined line fails the referee replay "
            f"({tracker.events_passed}/{course.total_events}, "
            f"{len(tracker.gate_contacts)} contacts)")
    return anchors, plan


def optimize_poses(cfg, course, anchors, center_idx, rounds: int = 2,
                   maxfev: int = 200, starts: int = 2, spread: float = 1.0,
                   seed: int = 0, a_max_deg: float = 35.0, verbose: bool = True,
                   only_legs=None, fixed_angles=None, angle_grid_deg=()):
    """Anchor refinement with the CROSSING POSE of every gate as shared
    variables: lateral offset u_k along the bar and crossing angle a_k off
    the normal. The two stubs of gate k are DERIVED from (u_k, a_k), so a
    tilted crossing moves both stubs together. In optimize_anchors the
    stubs were independent leg variables and a leg could never tilt its
    gate without kinking against the neighbouring stub - the line ran
    2-3 m straight out of g6 before turning. Here the apex can sit ON the
    gate. Interior anchors stay free per leg with multistart. The
    clearance penalty in _anchor_cost prices the narrower angled opening."""
    from scipy.optimize import minimize
    pq = course_bridge.pq_course()
    anchors = np.array(anchors, dtype=float).copy()
    ci = list(center_idx)
    n_ev = course.total_events
    ev_true = [course.event(i) for i in range(n_ev)]
    ctr_true = np.array([[e.x, e.y, e.z] for e in ev_true])
    heads = np.array([e.heading_rad for e in ev_true])
    bars = np.array([[-math.sin(h), math.cos(h), 0.0] for h in heads])
    cset = set(ci)
    post_i = [ci[k] + 1 if (ci[k] + 1 < len(anchors) and ci[k] + 1 not in cset) else -1
              for k in range(n_ev)]
    pre_i = []
    for k in range(n_ev):
        i = ci[k] - 1
        if k == 0 or i in cset or i == post_i[k - 1]:
            pre_i.append(-1)
        else:
            pre_i.append(i)
    d_post = [float(np.linalg.norm(anchors[post_i[k]][:2] - anchors[ci[k]][:2]))
              if post_i[k] >= 0 else 0.0 for k in range(n_ev)]
    d_pre = [float(np.linalg.norm(anchors[pre_i[k]][:2] - anchors[ci[k]][:2]))
             if pre_i[k] >= 0 else 0.0 for k in range(n_ev)]
    u = np.array([float(np.dot(anchors[ci[k]] - ctr_true[k], bars[k]))
                  for k in range(n_ev)])
    a = np.zeros(n_ev)
    for k in range(n_ev):
        ref = post_i[k] if post_i[k] >= 0 else pre_i[k]
        if ref >= 0:
            v = anchors[ref][:2] - anchors[ci[k]][:2]
            if ref == pre_i[k]:
                v = -v
            if np.hypot(v[0], v[1]) > 1e-6:
                a[k] = planner.wrap_pi(math.atan2(v[1], v[0]) - heads[k])
    a_max = math.radians(a_max_deg)
    a = np.clip(a, -a_max, a_max)
    fixed_angles = dict(fixed_angles or {})      # {event k: radians}
    for k, val in fixed_angles.items():
        a[k] = val
    U_MAX = 0.75 - (CLEARANCE_R_M - 0.15)

    def build(arr, uu, aa):
        for k in range(n_ev):
            c = ctr_true[k] + uu[k] * bars[k]
            arr[ci[k]] = c
            d = np.array([math.cos(heads[k] + aa[k]),
                          math.sin(heads[k] + aa[k]), 0.0])
            if post_i[k] >= 0:
                z = arr[post_i[k]][2]
                arr[post_i[k]] = c + d_post[k] * d
                arr[post_i[k]][2] = z
            if pre_i[k] >= 0:
                z = arr[pre_i[k]][2]
                arr[pre_i[k]] = c - d_pre[k] * d
                arr[pre_i[k]][2] = z
        return arr

    anchors = build(anchors, u, a)
    stub_set = set(i for i in post_i + pre_i if i >= 0)
    legs = []
    for j in range(n_ev - 1):
        if only_legs is not None and j not in only_legs:
            continue
        interior = [i for i in range(ci[j] + 1, ci[j + 1]) if i not in stub_set]
        legs.append((j, interior))
    best = _anchor_cost(cfg, course, anchors, ci, _OPT_DENSE_DS, pq)
    if verbose:
        print(f"OPT4  seed {best:.2f} s (poses; coarse incl. penalties)", flush=True)
    for r in range(rounds):
        for j, interior in legs:
            ni = len(interior)
            base_x = anchors[interior].ravel() if ni else np.zeros(0)
            x0 = np.concatenate([base_x, [u[j], a[j], u[j + 1], a[j + 1]]])
            lo = np.concatenate([base_x - np.tile([3.0, 3.0, 1.0], ni),
                                 [-U_MAX, -a_max, -U_MAX, -a_max]])
            hi = np.concatenate([base_x + np.tile([3.0, 3.0, 1.5], ni),
                                 [U_MAX, a_max, U_MAX, a_max]])
            if ni:
                lo[2:ni * 3:3] = np.maximum(lo[2:ni * 3:3], 0.8)
            for kk, pos_ in ((j, -3), (j + 1, -1)):
                if kk in fixed_angles:
                    lo[pos_] = hi[pos_] = x0[pos_] = fixed_angles[kk]

            def cost(x, interior=interior, j=j, ni=ni):
                trial = anchors.copy()
                if ni:
                    trial[interior] = x[:ni * 3].reshape(-1, 3)
                uu, aa = u.copy(), a.copy()
                uu[j], aa[j], uu[j + 1], aa[j + 1] = x[-4], x[-3], x[-2], x[-1]
                trial = build(trial, uu, aa)
                return _anchor_cost(cfg, course, trial, ci, _OPT_DENSE_DS, pq)

            rng = np.random.default_rng(seed * 1000 + r * 100 + j)
            cands = [x0]
            for _ in range(max(0, starts - 1)):
                xj = x0.copy()
                if ni:
                    xj[:ni * 3] += rng.normal(0.0, spread, ni * 3)
                    xj[2:ni * 3:3] = np.clip(xj[2:ni * 3:3], 0.8, 5.5)
                xj[-3] += rng.normal(0.0, 0.3)
                xj[-1] += rng.normal(0.0, 0.3)
                cands.append(np.clip(xj, lo, hi))
            # ANGLE GRID starts: the crossing angle is multimodal (a square
            # crossing with a late turn is a basin of its own), and Powell
            # from the seed stayed in it at every gate. Measured on g6:
            # forced 30 deg beat the search's 9 deg by 0.2 s over two legs.
            # Start the leg from a grid of angle pairs for its two gates.
            if angle_grid_deg:
                for ga in angle_grid_deg:
                    for gb in angle_grid_deg:
                        xj = x0.copy()
                        xj[-3] = math.radians(ga)
                        xj[-1] = math.radians(gb)
                        cands.append(np.clip(xj, lo, hi))
            res_best = None
            for xs in cands:
                res = minimize(cost, xs, method="Powell", bounds=list(zip(lo, hi)),
                               options={"maxfev": maxfev, "xtol": 0.03,
                                        "ftol": 1e-3})
                if res_best is None or res.fun < res_best.fun:
                    res_best = res
            if res_best.fun < best - 1e-4:
                x = np.clip(res_best.x, lo, hi)
                if ni:
                    anchors[interior] = x[:ni * 3].reshape(-1, 3)
                u[j], a[j], u[j + 1], a[j + 1] = x[-4], x[-3], x[-2], x[-1]
                anchors = build(anchors, u, a)
                best = float(res_best.fun)
        if verbose:
            print(f"OPT4  round {r + 1}: {best:.2f} s", flush=True)
    plan = _plan_from_anchors(cfg, course, u, anchors, center_idx=ci)
    plan.meta["optimizer"] = {"mode": "pose-refine",
                              "clearance_r_m": CLEARANCE_R_M,
                              "final_cost_s": round(best, 2)}
    plan.meta["crossing_offsets_m"] = [round(float(x), 3) for x in u]
    plan.meta["crossing_angles_deg"] = [round(math.degrees(float(x)), 1) for x in a]
    plan.meta["anchors"] = [[round(float(v), 4) for v in row] for row in anchors]
    plan.meta["center_idx"] = [int(i) for i in ci]
    tracker = pq.RaceTracker(course)
    for t, pos in zip(plan.t, plan.pos):
        tracker.update(float(t), pos)
    if not tracker.complete or tracker.gate_contacts:
        raise RuntimeError(f"pose-refined line fails the referee replay "
                           f"({tracker.events_passed}/{course.total_events}, "
                           f"{len(tracker.gate_contacts)} contacts)")
    return plan


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--maxfev", type=int, default=_MAXFEV)
    ap.add_argument("--free", action="store_true",
                    help="free-point mode: the line may invent shapes "
                         "between gates (loops, spirals, wide entries)")
    ap.add_argument("--refine", action="store_true",
                    help="anchor refinement: optimize the through-gate "
                         "geometry of the rule-built line (see "
                         "optimize_anchors)")
    ap.add_argument("--rounds", type=int, default=2)
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    if args.refine:
        _, plan = optimize_anchors(cfg, rounds=args.rounds)
    elif args.free:
        u, fp, plan = optimize_free(cfg)
    else:
        u, plan = optimize(cfg, maxfev=args.maxfev)
    out = args.out or planner.next_numbered(
        str(AIGP_REPO / "out" / "plans" / "plan_opt_XXX.json"))
    planner.write_plan(plan, out)
    print(f"FILES plan -> {out}")
    print(planner.report(plan))
    off = plan.meta["optimizer"].get("offsets_m")
    if off:
        print("OPT   offsets (m, + = left of entry heading):")
        for k, e in enumerate(plan.events):
            print(f"        {e['label']:8s} {off[k]:+.2f}")

    try:
        from raceline import render_plan
        render_plan.render(plan, str(out).replace(".json", ".png"))
        print(f"      render -> {str(out).replace('.json', '.png')}")
    except Exception as e:
        print(f"      render skipped: {e}")


if __name__ == "__main__":
    main()
