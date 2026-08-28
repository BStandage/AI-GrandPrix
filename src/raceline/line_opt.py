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
    return float(np.interp(s_last, sg, t))


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
    pre_d, post_d = planner.standoffs(
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
    return float(np.interp(float(s_dense[adi[center_idx[-1]]]), sg, t))


def optimize_free(cfg, course=None, rounds: int = 2, verbose: bool = True):
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


def _plan_from_anchors(cfg, course, offsets, anchors):
    """Full-resolution Plan for an explicit anchor chain (planner.plan
    rebuilt around externally supplied anchors)."""
    import hashlib
    oc = OffsetCourse(course, offsets)
    events_list = [oc.event(i) for i in range(oc.total_events)]
    # recompute center indices for THIS chain by nearest anchor to centers
    centers = np.array([[e.x, e.y, e.z] for e in events_list])
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
                       "z": round(e.z, 3)})
    map_p = course_bridge.map_path()
    meta = {
        "map_source": course.source,
        "map_sha1": hashlib.sha1(map_p.read_bytes()).hexdigest()
        if map_p.exists() else None,
        "config_path": str(cfg.path), "config_sha1": cfg.sha1,
        "params": cfg.raw, "laps": course.laps,
        "crossings_per_lap": len(course.crossings),
        "path_length_m": round(total, 2),
    }
    return planner.Plan(s=sg, pos=P, vel=vel, acc=acc, t=t, v=v,
                        v_lim=v_lim, tangent=T, kappa=kappa,
                        dpsi_ds=dpsi_ds, dkappa_ds=dkappa_ds,
                        binding=binding, events=events, meta=meta)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--maxfev", type=int, default=_MAXFEV)
    ap.add_argument("--free", action="store_true",
                    help="free-point mode: the line may invent shapes "
                         "between gates (loops, spirals, wide entries)")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    if args.free:
        u, fp, plan = optimize_free(cfg)
    else:
        u, plan = optimize(cfg, maxfev=args.maxfev)
    print(planner.report(plan))
    off = plan.meta["optimizer"]["offsets_m"]
    print("OPT   offsets (m, + = left of entry heading):")
    for k, e in enumerate(plan.events):
        print(f"        {e['label']:8s} {off[k]:+.2f}")

    out = args.out or planner.next_numbered(
        str(AIGP_REPO / "out" / "plans" / "plan_opt_XXX.json"))
    planner.write_plan(plan, out)
    print(f"FILES plan -> {out}")
    try:
        from raceline import render_plan
        render_plan.render(plan, str(out).replace(".json", ".png"))
        print(f"      render -> {str(out).replace('.json', '.png')}")
    except Exception as e:
        print(f"      render skipped: {e}")


if __name__ == "__main__":
    main()
