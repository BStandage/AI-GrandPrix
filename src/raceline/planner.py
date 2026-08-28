"""Offline racing-line planner: course map + vehicle.toml -> timed trajectory.

Pipeline (all GLOBAL parameters, nothing per-gate):
  1. ANCHORS - per crossing k with center c and required-direction normal n:
     (c - d_pre*n, c, c + d_post*n). Standoffs follow the two-value rule:
     the base standoff normally, the larger turn standoff on BOTH sides of a
     junction whose consecutive crossing headings differ by more than
     turn_angle_deg. The g10 out-and-back (headings ~180 deg apart) falls out
     of this rule - the planner never names a gate.
  2. PATH - centripetal Catmull-Rom through the anchors (interpolating, no
     overshoot loops on uneven spacing), resampled to uniform arc length.
  3. SPEED PROFILE - pointwise ceiling
        v_lim = min( v_max,
                     sqrt(a_lat / kappa),            a_lat = margin*g*tan(tilt)
                     yaw_rate_max / |dpsi/ds|,       nose-follows-tangent
                     vz_up / slope, vz_down / |slope|,
                     v_gate within gate_window of ANY crossing center )
     then a forward pass (accel) and backward pass (brake), both on the
     friction circle a_long = a_budget * sqrt(1 - (v^2*kappa/a_lat)^2), so
     braking before corners and accelerating out fall out of the math.
  4. TIMESTAMPS + feedforward accel by differentiating v * tangent.

Output: a Plan (arrays + provenance) and a JSON file with a stable contract
so a better optimizer can replace this module without touching the follower.

All predicted times are MODEL PREDICTIONS, unverified (RESTRICTIONS.md).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from raceline import course as course_bridge
from raceline.config import G, VehicleConfig, load_config, AIGP_REPO

PLAN_VERSION = 1
PARK_ALT_M = 0.8
_DENSE_DS = 0.05      # spline pre-sampling resolution (m)
_DEDUP_M = 0.15       # drop pre/post anchors this close to their neighbor


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Anchors
# ---------------------------------------------------------------------------

def standoffs(centers_xy: List[Tuple[float, float]], headings: List[float],
              base: float, turn: float,
              thresh_rad: float) -> Tuple[List[float], List[float]]:
    """Per-event (pre, post) standoff distances under the two-value rule.

    The junction turn is measured on the actual TRAVEL, not just the crossing
    headings: out-turn = exit heading vs the leg direction to the next
    crossing, in-turn = leg direction vs the next entry heading. A planar
    switchback (leg opposes the entry heading) therefore triggers the larger
    standoff even when the two crossing headings are nearly perpendicular.
    A degenerate XY leg (the stacked out-and-back) falls back to comparing
    the crossing headings directly and widens both sides.
    """
    n = len(headings)
    pre = [base] * n
    post = [base] * n
    for k in range(n - 1):
        dx = centers_xy[k + 1][0] - centers_xy[k][0]
        dy = centers_xy[k + 1][1] - centers_xy[k][1]
        if math.hypot(dx, dy) < 1.0:
            big = abs(wrap_pi(headings[k + 1] - headings[k])) > thresh_rad
            out_big = in_big = big
        else:
            leg = math.atan2(dy, dx)
            out_big = abs(wrap_pi(leg - headings[k])) > thresh_rad
            in_big = abs(wrap_pi(headings[k + 1] - leg)) > thresh_rad
        if out_big:
            post[k] = turn
        if in_big:
            pre[k + 1] = turn
    return pre, post


def build_anchors(course, cfg: VehicleConfig):
    """Anchor chain: takeoff -> (pre, center, post) per event -> park.

    Returns (anchors[N,3], center_anchor_idx[len(events)]).
    """
    p = cfg.planner
    events = [course.event(i) for i in range(course.total_events)]
    pre_d, post_d = standoffs([(c.x, c.y) for c in events],
                              [c.heading_rad for c in events],
                              p.anchor_standoff_m, p.anchor_standoff_turn_m,
                              math.radians(p.turn_angle_deg))

    anchors: List[np.ndarray] = [np.array([0.0, 0.0, p.takeoff_alt_m])]
    center_idx: List[int] = []
    for k, c in enumerate(events):
        n = np.array([math.cos(c.heading_rad), math.sin(c.heading_rad), 0.0])
        ctr = np.array([c.x, c.y, c.z])
        pre = ctr - pre_d[k] * n
        post = ctr + post_d[k] * n
        # Anchor crowding rule (global): when the previous exit anchor and
        # this entry anchor are closer than the base standoff they fight each
        # other and the spline S-wiggles - merge them into their midpoint.
        # The stacked pair is unaffected (its post/pre are 2.7 m apart in z).
        gap = float(np.linalg.norm(pre - anchors[-1]))
        if k > 0 and gap < p.anchor_standoff_m:
            anchors[-1] = 0.5 * (anchors[-1] + pre)
        elif gap > _DEDUP_M:
            anchors.append(pre)
        center_idx.append(len(anchors))
        anchors.append(ctr)
        anchors.append(post)   # centers/posts always kept: post defines exit
    last = anchors[-1]
    anchors.append(np.array([last[0], last[1], PARK_ALT_M]))
    return np.array(anchors), center_idx


# ---------------------------------------------------------------------------
# Centripetal Catmull-Rom
# ---------------------------------------------------------------------------

def _cr_segment(p0, p1, p2, p3, n: int, alpha: float = 0.5) -> np.ndarray:
    """Barry-Goldman evaluation of one centripetal CR segment p1->p2,
    n points including p1, excluding p2."""
    def knot(ti, pa, pb):
        return ti + max(float(np.linalg.norm(pb - pa)), 1e-6) ** alpha
    t0 = 0.0
    t1 = knot(t0, p0, p1)
    t2 = knot(t1, p1, p2)
    t3 = knot(t2, p2, p3)
    out = np.empty((n, 3))
    for j, t in enumerate(np.linspace(t1, t2, n, endpoint=False)):
        a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
        a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
        a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
        b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
        b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
        out[j] = (t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2
    return out


def sample_spline(anchors: np.ndarray):
    """Dense-sample the spline through all anchors.

    Returns (dense[N,3], s[N], anchor_dense_idx) where anchor_dense_idx[i]
    is the dense-sample index of anchor i (used to pin event arc positions).
    """
    ext = np.vstack([2 * anchors[0] - anchors[1], anchors,
                     2 * anchors[-1] - anchors[-2]])
    dense: List[np.ndarray] = []
    anchor_dense_idx: List[int] = []
    for i in range(1, len(ext) - 2):
        anchor_dense_idx.append(len(dense))
        chord = float(np.linalg.norm(ext[i + 1] - ext[i]))
        n = max(8, int(chord / _DENSE_DS))
        dense.extend(_cr_segment(ext[i - 1], ext[i], ext[i + 1], ext[i + 2], n))
    dense.append(ext[-2])
    anchor_dense_idx.append(len(dense) - 1)
    dense_arr = np.array(dense)
    seg = np.linalg.norm(np.diff(dense_arr, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    return dense_arr, s, anchor_dense_idx


def _menger_curvature(P: np.ndarray) -> np.ndarray:
    """kappa = 4*Area/(|a||b||c|) = 2|a x b|/(|a||b||c|) per interior point."""
    k = np.zeros(len(P))
    a = P[1:-1] - P[:-2]
    b = P[2:] - P[1:-1]
    c = P[2:] - P[:-2]
    cross = np.cross(a, b)
    num = 2.0 * np.linalg.norm(cross, axis=1)
    den = (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1)
           * np.linalg.norm(c, axis=1) + 1e-12)
    k[1:-1] = num / den
    k[0], k[-1] = k[1], k[-2]
    return k


# ---------------------------------------------------------------------------
# Plan
# ---------------------------------------------------------------------------

@dataclass
class Plan:
    s: np.ndarray          # [N] arc length
    pos: np.ndarray        # [N,3]
    vel: np.ndarray        # [N,3] v * tangent
    acc: np.ndarray        # [N,3] feedforward
    t: np.ndarray          # [N] timestamps
    v: np.ndarray          # [N] speed
    v_lim: np.ndarray      # [N] pointwise ceiling (diagnostic)
    tangent: np.ndarray    # [N,3]
    kappa: np.ndarray      # [N] 3D curvature (diagnostic)
    dpsi_ds: np.ndarray    # [N] heading rate per meter (diagnostic)
    dkappa_ds: np.ndarray  # [N] curvature sharpness (diagnostic)
    binding: np.ndarray    # [N] name of the ceiling that set v_lim there
    events: List[dict]     # per crossing: event, lap, label, s, t, v, x/y/z
    meta: dict = field(default_factory=dict)

    @property
    def total_s(self) -> float:
        return float(self.t[-1])

    def predicted_lap_times(self, crossings_per_lap: int) -> List[float]:
        """Cumulative time at each lap's final crossing (model prediction)."""
        out = []
        for i in range(crossings_per_lap - 1, len(self.events),
                       crossings_per_lap):
            out.append(self.events[i]["t"])
        return out

    def to_json_dict(self) -> dict:
        return {
            "version": PLAN_VERSION,
            "frame": "sim ENU (MapToSim of course_map)",
            **self.meta,
            "predicted": {
                "total_s": round(self.total_s, 2),
                "note": "model prediction, unverified",
            },
            "events": [
                {**e, "s": round(e["s"], 3), "t": round(e["t"], 3),
                 "v": round(e["v"], 3)}
                for e in self.events
            ],
            "samples": np.column_stack(
                [self.t, self.pos, self.vel, self.acc]).round(4).tolist(),
        }


def _speed_profile(P: np.ndarray, s: np.ndarray, cfg: VehicleConfig,
                   centers: np.ndarray):
    lim = cfg.limits
    ds = float(s[1] - s[0])
    n = len(P)

    T = np.gradient(P, s, axis=0)
    T /= np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-9)
    kappa = _menger_curvature(P)
    txy = np.hypot(T[:, 0], T[:, 1])
    psi = np.unwrap(np.arctan2(T[:, 1], T[:, 0]))
    dpsi_ds = np.abs(np.gradient(psi, s))

    # Curvature SHARPNESS |dkappa/ds| on a ~1 m smoothed kappa (Menger on
    # 0.25 m samples is too noisy to differentiate raw).
    win = max(1, int(round(1.0 / ds)) | 1)
    kern = np.ones(win) / win
    kappa_s = np.convolve(kappa, kern, mode="same")
    dkappa_ds = np.abs(np.gradient(kappa_s, s))

    a_lat = cfg.a_lat_planner()
    # Named pointwise ceilings; v_lim = elementwise min, and the argmin NAME
    # is kept per sample so reports say WHAT binds, not a guess.
    ceilings = {"v_max": np.full(n, float(lim.v_max_mps))}
    ceilings["tilt/curvature"] = np.sqrt(a_lat / np.maximum(kappa, 1e-6))

    # Attitude-slew ceiling: a_lat = v^2*kappa, so at steady speed
    # d(a_lat)/dt ~ v^3 * dkappa/ds. Heavy low-pitch builds (8" Archer) are
    # limited by how fast they can ROTATE to a tilt, not by the tilt itself -
    # without this cap, corner ENTRIES are geometrically fine but physically
    # unreachable, and it presents as tracking error that looks like bad
    # gains.
    ceilings["accel-slew"] = np.cbrt(
        lim.a_lat_rate_max / np.maximum(dkappa_ds, 1e-6))

    # Yaw-rate ceiling: the nose tracks the path tangent, dpsi/dt = v*dpsi/ds.
    # Only where the heading is defined (path not near-vertical).
    v_yaw = np.full(n, np.inf)
    yaw_ok = txy > 0.2
    v_yaw[yaw_ok] = lim.max_yaw_rate_rps / np.maximum(dpsi_ds[yaw_ok], 1e-6)
    ceilings["yaw-rate"] = v_yaw

    tz = T[:, 2]
    v_slope = np.full(n, np.inf)
    up = tz > 1e-3
    v_slope[up] = lim.vz_up_max / tz[up]
    dn = tz < -1e-3
    v_slope[dn] = np.minimum(v_slope[dn], lim.vz_down_max / (-tz[dn]))
    ceilings["climb/descent"] = v_slope

    # Uniform crossing-window cap (ONE global rule for every gate).
    dmin = np.min(np.linalg.norm(P[:, None, :] - centers[None, :, :], axis=2),
                  axis=1)
    v_gate = np.full(n, np.inf)
    v_gate[dmin < lim.gate_window_m] = lim.v_gate_mps
    ceilings["gate-window"] = v_gate

    names = list(ceilings)
    stack = np.vstack([ceilings[k] for k in names])
    v_lim = stack.min(axis=0)
    binding = np.array(names, dtype=object)[stack.argmin(axis=0)]
    floored = v_lim < cfg.planner.v_floor_mps
    binding[floored] = "v_floor(" + binding[floored] + ")"
    v_lim = np.maximum(v_lim, cfg.planner.v_floor_mps)

    # Two-pass with friction circle.
    def a_avail(budget: float, vi: float, ki: float) -> float:
        frac = min(1.0, (vi * vi * ki) / a_lat)
        return max(0.1, budget * math.sqrt(max(0.0, 1.0 - frac * frac)))

    v = v_lim.copy()
    v[0] = 0.0
    for i in range(n - 1):
        aa = a_avail(lim.a_accel_max, v[i], kappa[i])
        v[i + 1] = min(v_lim[i + 1], math.sqrt(v[i] * v[i] + 2 * aa * ds))
    v[-1] = 0.0
    for i in range(n - 1, 0, -1):
        ab = a_avail(lim.a_brake_max, v[i], kappa[i])
        v[i - 1] = min(v[i - 1], math.sqrt(v[i] * v[i] + 2 * ab * ds))

    t = np.zeros(n)
    floor = cfg.planner.v_floor_mps
    for i in range(n - 1):
        t[i + 1] = t[i] + 2 * ds / max(v[i] + v[i + 1], floor)

    vel = v[:, None] * T
    acc = np.gradient(vel, t, axis=0)
    return T, kappa, dpsi_ds, dkappa_ds, binding, v_lim, v, t, vel, acc


def plan(cfg: VehicleConfig, course=None) -> Plan:
    if course is None:
        course = course_bridge.load_course()

    anchors, center_idx = build_anchors(course, cfg)
    dense, s_dense, anchor_dense_idx = sample_spline(anchors)

    total = float(s_dense[-1])
    n = max(2, int(total / cfg.planner.sample_ds_m) + 1)
    sg = np.linspace(0.0, total, n)
    P = np.column_stack([np.interp(sg, s_dense, dense[:, k])
                         for k in range(3)])

    events_list = [course.event(i) for i in range(course.total_events)]
    centers = np.array([[c.x, c.y, c.z] for c in events_list])

    (T, kappa, dpsi_ds, dkappa_ds, binding, v_lim, v, t, vel,
     acc) = _speed_profile(P, sg, cfg, centers)

    events = []
    for k, c in enumerate(events_list):
        s_ev = float(s_dense[anchor_dense_idx[center_idx[k]]])
        events.append({
            "event": k,
            "lap": course.lap_of(k),
            "label": c.label,
            "s": s_ev,
            "t": float(np.interp(s_ev, sg, t)),
            "v": float(np.interp(s_ev, sg, v)),
            "x": round(c.x, 3), "y": round(c.y, 3), "z": round(c.z, 3),
        })

    map_p = course_bridge.map_path()
    meta = {
        "map_source": course.source,
        "map_sha1": hashlib.sha1(map_p.read_bytes()).hexdigest()
        if map_p.exists() else None,
        "config_path": str(cfg.path),
        "config_sha1": cfg.sha1,
        "params": cfg.raw,
        "laps": course.laps,
        "crossings_per_lap": len(course.crossings),
        "path_length_m": round(float(sg[-1]), 2),
    }
    return Plan(s=sg, pos=P, vel=vel, acc=acc, t=t, v=v, v_lim=v_lim,
                tangent=T, kappa=kappa, dpsi_ds=dpsi_ds,
                dkappa_ds=dkappa_ds, binding=binding, events=events,
                meta=meta)


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def next_numbered(path_pattern: str) -> Path:
    """out/plans/plan_XXX.json-style auto-numbering ('XXX' -> 000, 001...)."""
    i = 0
    while True:
        p = Path(path_pattern.replace("XXX", f"{i:03d}"))
        if not p.exists():
            return p
        i += 1


def write_plan(p: Plan, path: os.PathLike) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(p.to_json_dict(), f)
    return path


def load_plan(path: os.PathLike) -> dict:
    """Load a plan JSON into arrays: keys t, pos, vel, acc, s (recomputed),
    events, meta-ish fields kept as-is."""
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    samples = np.asarray(d["samples"], dtype=float)
    d["t_arr"] = samples[:, 0]
    d["pos"] = samples[:, 1:4]
    d["vel"] = samples[:, 4:7]
    d["acc"] = samples[:, 7:10]
    seg = np.linalg.norm(np.diff(d["pos"], axis=0), axis=1)
    d["s_arr"] = np.concatenate([[0.0], np.cumsum(seg)])
    return d


# ---------------------------------------------------------------------------
# Checkpoint report (the step-2 -> step-3 gate)
# ---------------------------------------------------------------------------

def report(p: Plan, baseline_s: Optional[float] = 225.3) -> str:
    per_lap = p.meta["crossings_per_lap"]
    lines = []
    lines.append(f"PLAN  {len(p.events)} events, "
                 f"{p.meta['path_length_m']:.0f} m path, "
                 f"config {p.meta['config_sha1'][:8]}")
    laps = p.predicted_lap_times(per_lap)
    lap_str = "  ".join(
        f"lap{i} {t - (laps[i-1] if i else 0.0):.1f}s" for i, t in enumerate(laps))
    lines.append(f"      predicts total {p.total_s:.1f} s ({lap_str}) "
                 f"- model prediction, unverified"
                 + (f"; baseline {baseline_s:.1f} s" if baseline_s else ""))

    # Speed-profile minimum BETWEEN first and last crossing (excludes the
    # takeoff ramp and final park where v -> 0 by construction).
    s0, s1 = p.events[0]["s"], p.events[-1]["s"]
    mask = (p.s >= s0) & (p.s <= s1)
    idx = np.flatnonzero(mask)
    imin = idx[np.argmin(p.v[idx])]
    s_min, v_min = float(p.s[imin]), float(p.v[imin])
    near = min(p.events, key=lambda e: abs(e["s"] - s_min))
    lines.append(f"CHECK speed-profile minimum: {v_min:.2f} m/s at "
                 f"s={s_min:.1f} m (nearest event: {near['label']}, "
                 f"{s_min - near['s']:+.1f} m along-path)")
    i = imin
    what = ("accel/brake passes" if p.v_lim[i] > p.v[i] + 0.05
            else str(p.binding[i]))
    lines.append(f"      binding constraint there: {what}; "
                 f"kappa={p.kappa[i]:.2f} 1/m, dkappa/ds={p.dkappa_ds[i]:.2f}"
                 f" 1/m^2, dpsi/ds={p.dpsi_ds[i]:.2f} rad/m, "
                 f"v_lim={p.v_lim[i]:.2f} m/s")
    # Where each ceiling class rules the profile (samples between the first
    # and last crossing) - one line to see WHAT this vehicle config is
    # limited by overall.
    from collections import Counter
    counts = Counter(str(b) for b in p.binding[idx])
    total_n = sum(counts.values())
    top = ", ".join(f"{k} {100*c//total_n}%" for k, c in
                    counts.most_common(4))
    lines.append(f"      profile ruled by: {top}")

    lines.append("      per-event crossing speeds (m/s):")
    for e in p.events:
        if e["event"] % per_lap == 0 and e["event"]:
            lines.append("      --- lap boundary ---")
        lines.append(f"        {e['event']:2d} {e['label']:8s} "
                     f"t={e['t']:6.1f}  v={e['v']:4.2f}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=None, help="vehicle.toml path")
    ap.add_argument("--out", default=None,
                    help="plan JSON path (default out/plans/plan_XXX.json)")
    ap.add_argument("--no-render", action="store_true")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    p = plan(cfg)
    print(report(p))

    out = Path(args.out) if args.out else next_numbered(
        str(AIGP_REPO / "out" / "plans" / "plan_XXX.json"))
    write_plan(p, out)
    print(f"FILES plan -> {out}")
    if not args.no_render:
        try:
            from raceline import render_plan
            png = out.with_suffix(".png")
            render_plan.render(p, png)
            print(f"      render -> {png}")
        except Exception as e:   # render is eyeball-only, never blocks a plan
            print(f"      render skipped: {e}")
    return p, out


if __name__ == "__main__":
    main()
