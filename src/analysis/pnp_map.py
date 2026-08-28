"""
VISION-METRIC course map: gate positions chained from PnP at tick anchors.
NO dynamics — distances from the known gate opening (1.5 m / outer 2.7 m PnP).

    python -m analysis.pnp_map \\
        datasets/steady_dbg_20260813_224401.csv \\
        datasets/session_20260813_224359

Upgrades vs archived pnp_map.py (MEGA §1.1 / Stage 2):
  - Pass anchors from last_gate_race_time ticks (not blob gates_passed)
  - Direction prior from course_map_tick.json (same survey)
  - Opening-only + elevation self-cal (kept)
  - Tick-anchored legs + dwell back-prop (NOT raw mid-leg sighting ranges)
  - Prefer 2.5–8 m dwell for *accuracy*, back-propagated to the tick instant
  - Report per-leg median + MAD (σ)

BUG (2026-08-13): summing mid-leg dwell sighting ranges collapsed each leg to
~2.5–8 m and the path to ~124 m (~30% short). Close dwell is right for range
accuracy only when motion from the tick is added back (vision track Δv).
When gate k+1 is not visible at the tick, bridge the gap with the climb-table
plant at survey creep (~1.3 m/s) and tag bridge=\"csv\" (explicit coupling).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil

from analysis.imu_traverse import ImuTraverse
from analysis.race_ticks import best_lap_ticks, ticks_from_last_gate
from common.camera import HALF_TAN_X, HALF_TAN_Y, offset_to_bearing, offset_to_body_dir

UPTILT = math.radians(20.0)
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)
GIGA = os.path.join(SRC, "pilots", "giga_pilot")
RMIN, RMAX = 2.5, 8.0  # dwell sweet spot (m, horizontal) — accuracy, not anchor
# Frames within this of the tick: drone still at gate k (±0.61 m). ~0.15 s @ 30 Hz.
TICK_HALF = 5
# Far-range pinhole is short-biased (mask dilation); never use PnP range for scale
# (fables 2026-08-13: PnP 1.37x long at approach; pinhole honest near-range).


def _median(xs):
    xs = sorted(xs)
    if not xs:
        return None
    return xs[len(xs) // 2]


def _mad(xs, med=None):
    if not xs:
        return None
    med = _median(xs) if med is None else med
    return _median([abs(x - med) for x in xs])


def _sigma(xs):
    """Robust σ ≈ 1.4826 * MAD."""
    med = _median(xs)
    mad = _mad(xs, med)
    if med is None or mad is None:
        return med, None, 0
    return med, 1.4826 * mad, len(xs)


def _backprop_to_anchor(track, i_f):
    """Gate vector at track start from a later measurement.

    Identity (static gate): v(t_j) - v(t_{j+1}) = drone(t_{j+1}) - drone(t_j).
    So leg_at_anchor = v(t_f) + Σ_j (v_j - v_{j+1}) = v(t_anchor) when the
    track is continuous from the anchor. With gaps, Σ over observed pairs still
    approximates the drone displacement between first and last sample.
    """
    vf = track[i_f][1]
    dx = dy = dz = 0.0
    for k in range(i_f):
        a, b = track[k][1], track[k + 1][1]
        dx += a[0] - b[0]
        dy += a[1] - b[1]
        dz += a[2] - b[2]
    return (vf[0] + dx, vf[1] + dy, vf[2] + dz)


def _csv_bridge_disp(by_seq, seq0, seq1, exp_dir, v0_speed=1.3):
    """Drone world displacement seq0→seq1 via climb-table plant at survey creep.

    Used only for the tick→first-sighting gap (typically 2–6 m / 2–4 s) when
    gate k+1 is not yet in frame at the tick. At ~1.3 m/s with measured
    attitude, bridge error is ~0.3 m — far better than prior×median (5+ m).
    Explicit map coupling: tag bridge=\"csv\" on those gates.
    """
    from analysis.fit_plant import body_thrust_accel, load_plant_fit

    G = 9.81
    fit = load_plant_fit()
    # d1=0 is a race-band fit (v^2 dominates there). At survey speeds the
    # missing linear drag (~0.12*v) makes multi-second bridges run long
    # (VQ1 truth: bridged legs +25%). Bridge-only constant; not a plant fork.
    d1, d2 = max(0.12, float(fit.get("d1", 0.0))), float(fit.get("d2", 0.0115))
    seqs = sorted(s for s in by_seq if seq0 <= s <= seq1)
    if len(seqs) < 2:
        return None
    # Seed along the leg at survey creep (standing start underestimates gap)
    vx = v0_speed * math.cos(exp_dir)
    vy = v0_speed * math.sin(exp_dir)
    vz = 0.0
    px = py = pz = 0.0
    bridge_t = 0.0
    for i in range(len(seqs) - 1):
        r0 = by_seq[seqs[i]]
        r1 = by_seq[seqs[i + 1]]
        try:
            dt = float(r1["t"]) - float(r0["t"])
        except (KeyError, ValueError, TypeError):
            continue
        if not (1e-4 < dt <= 0.5):
            continue
        thr = float(r0.get("thr") or 0.30)
        roll = math.radians(float(
            r0.get("roll_meas_deg") or r0.get("roll_cmd_deg") or 0.0))
        pitch = math.radians(float(
            r0.get("pitch_meas_deg") or r0.get("pitch_deg") or 0.0))
        yaw = math.radians(float(
            r0.get("yaw_meas_deg") or r0.get("yaw_deg") or 0.0))
        T = body_thrust_accel(thr, d1, d2)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        cy, sy = math.cos(yaw), math.sin(yaw)
        bx = cy * sp * cr + sy * sr
        by = sy * sp * cr - cy * sr
        bz = cp * cr
        sp_v = math.sqrt(vx * vx + vy * vy + vz * vz)
        dr = (d1 * sp_v + d2 * sp_v * sp_v) if sp_v > 1e-6 else 0.0
        ux = vx / sp_v if sp_v > 1e-6 else 0.0
        uy = vy / sp_v if sp_v > 1e-6 else 0.0
        uz = vz / sp_v if sp_v > 1e-6 else 0.0
        ax = T * bx - dr * ux
        ay = T * by - dr * uy
        az = T * bz - G - dr * uz
        vx += ax * dt
        vy += ay * dt
        vz += az * dt
        px += vx * dt
        py += vy * dt
        pz += vz * dt
        bridge_t += dt
    if bridge_t < 0.05:
        return None
    return (px, py, pz, bridge_t)


def _tick_speed(by_seq, seq0, default=1.3):
    """Ground speed at the tick from tracked-gate closure rate (~2 s before).

    The 1.3 m/s constant was a VQ2-creep assumption; VQ1 surveys fly 4-6 m/s
    and bridges seeded at creep integrated ~4x short (map_vs_truth s=1.52).
    """
    pairs = []
    prev = None
    for s in range(seq0 - 70, seq0 + 1):
        r = by_seq.get(s)
        if not r or not r.get("distance_m"):
            prev = None
            continue
        try:
            t, d = float(r["t"]), float(r["distance_m"])
        except (ValueError, TypeError):
            continue
        if prev is not None:
            dt = t - prev[0]
            dd = prev[1] - d
            if 1e-3 < dt < 0.5 and 0.0 < dd < 3.0:
                pairs.append(dd / dt)
        prev = (t, d)
    v = _median(pairs) if len(pairs) >= 5 else None
    if v is None:
        return default
    return max(0.5, min(12.0, v))


def load_prior(path):
    m = json.load(open(path))
    return {int(g["gate_id"]): g for g in m["gates"]}


def load_passes_from_ticks(dbg_rows, session, clock_offset_s=0.0):
    """(gate_id, csv_frame_seq, t_race) at each exact tick."""
    ticks = best_lap_ticks(session, min_gates=3) or ticks_from_last_gate(session)
    if not ticks:
        return []
    timed = [(float(r["t"]), r) for r in dbg_rows if r.get("t", "") != ""]
    timed.sort(key=lambda x: x[0])
    out = []
    for gid, t_race in ticks:
        t_csv = t_race + clock_offset_s
        best = min(timed, key=lambda tr: abs(tr[0] - t_csv))
        r = best[1]
        fr = r.get("frame", "")
        seq = int(fr) if fr not in ("", None) else None
        out.append((int(gid), seq, float(t_race), best[0]))
    return out


def main():
    ap = argparse.ArgumentParser(description="Tick-anchored PnP metric map")
    ap.add_argument("dbg_csv", help="steady_dbg_*.csv")
    ap.add_argument("session", help="datasets/session_* dir")
    ap.add_argument("--prior", default=None,
                    help="direction-prior map JSON (default: course_map_tick.json)")
    ap.add_argument("--write", default=None,
                    help="output path (default: pilots/giga_pilot/course_map_pnp_tick.json)")
    ap.add_argument("--install", action="store_true",
                    help="also copy to course_map_pnp_tick.json pointer used by map_resolve")
    ap.add_argument("--offset", type=float, default=None,
                    help="csv_t - race_t (default: from tick map meta or 0)")
    args = ap.parse_args()

    dbg_path, sess = args.dbg_csv, args.session
    rows = list(csv.DictReader(open(dbg_path, encoding="utf-8", errors="replace")))
    by_seq = {}
    for r in rows:
        fr = r.get("frame", "")
        if fr not in ("", None):
            by_seq[int(fr)] = r

    prior_path = args.prior or os.path.join(GIGA, "course_map_tick.json")
    if not os.path.isfile(prior_path):
        prior_path = os.path.join(GIGA, "course_map_v2.json")
    gates_old = load_prior(prior_path)
    offset = args.offset
    if offset is None:
        meta = json.load(open(prior_path)).get("anchor") or {}
        offset = float(meta.get("clock_offset_s") or 0.0)
    print(f"prior: {os.path.basename(prior_path)}  clock_offset={offset:+.4f}s")

    seq2fid = {}
    frames_p = os.path.join(sess, "frames.jsonl")
    for line in open(frames_p, encoding="utf-8", errors="replace"):
        d = json.loads(line)
        seq2fid[int(d["seq"])] = int(d["frame_id"])
    by_fid = {}
    vis_p = os.path.join(sess, "vision_frames.jsonl")
    for line in open(vis_p, encoding="utf-8", errors="replace"):
        d = json.loads(line)
        by_fid[int(d["frame_id"])] = d
    print(f"vision: {len(by_fid)} frames  seq_map={len(seq2fid)}")

    passes = load_passes_from_ticks(rows, sess, clock_offset_s=offset)
    print(f"ticks: {len(passes)} gates "
          f"[{passes[0][0]}..{passes[-1][0]}]" if passes else "ticks: 0")

    def det_vec(det, r):
        """World-frame metric camera→OPENING (opening-only).

        Range: pinhole_dist only (PnP solve is 1.37x long with correct K — blob-hull
        corners + planar ambiguity). Far-range pinhole is mask-dilation short →
        tick-window samples carry residual short bias; dwell back-prop adds close
        range accuracy at the tick anchor.
        Elevation: offset_to_bearing (single de-tilt).
        """
        has_op = bool(det.get("has_opening"))
        box = det.get("opening_bbox")
        if box is None and has_op:
            box = det.get("bbox")
        if box is None and not has_op:
            # Far: opening often missing at tick; ring bbox + outer pinhole OK.
            # Near without opening: skip (ring≠opening ~1 m / ~18° el).
            ph0 = det.get("pinhole_dist") or 0.0
            if ph0 >= 8.0:
                box = det.get("bbox")
        if not box:
            return None
        dist = det.get("pinhole_dist")
        if not dist or dist <= 0.5:
            return None
        bx, by, bw, bh = box
        ox = ((bx + bw / 2.0) - 320.0) / 320.0
        oy = ((by + bh / 2.0) - 180.0) / 180.0
        bf, br, bd = offset_to_body_dir(ox, oy)
        psi = math.radians(float(r["yaw_deg"]))
        pitch = math.radians(float(r["pitch_deg"]))
        cp, sp = math.cos(pitch), math.sin(pitch)
        fwd_p = bf * cp - bd * sp
        down_p = bf * sp + bd * cp
        right_p = br
        cy, sy = math.cos(psi), math.sin(psi)
        wx = fwd_p * cy - right_p * sy
        wy = fwd_p * sy + right_p * cy
        wz = -down_p
        if abs(det_vec.el_bias) > 1e-6:
            horiz = math.hypot(wx, wy) or 1.0
            el = math.atan2(wz, horiz) - det_vec.el_bias
            wh = dist * math.cos(el)
            return (wh * wx / horiz, wh * wy / horiz, dist * math.sin(el))
        return (dist * wx, dist * wy, dist * wz)

    det_vec.el_bias = 0.0

    # ---- elevation self-cal ----
    el_meas = []
    for gid, seq_pass, _tr, _tc in passes:
        if seq_pass is None:
            continue
        for off in range(-20, -3):
            fid = seq2fid.get(seq_pass + off)
            f = by_fid.get(fid) if fid is not None else None
            r = by_seq.get(seq_pass + off) or by_seq.get(seq_pass)
            if not f or not f.get("dets") or r is None:
                continue
            best = max(f["dets"], key=lambda d: d.get("area_frac", 0.0))
            if not (0.08 <= best.get("area_frac", 0) <= 0.45):
                continue
            box = best.get("opening_bbox") or (
                best.get("bbox") if best.get("has_opening") else None)
            if not box:
                continue
            ph = best.get("pinhole_dist") or 99.0
            if ph > 6.0:
                continue
            bx, by, bw, bh = box
            ox = ((bx + bw / 2.0) - 320.0) / 320.0
            oy = ((by + bh / 2.0) - 180.0) / 180.0
            _az, el_body = offset_to_bearing(ox, oy)
            pitch = math.radians(float(r["pitch_deg"]))
            el_meas.append(el_body - pitch)
    EL_BIAS = _median(el_meas) if el_meas else 0.0
    if EL_BIAS is not None and abs(EL_BIAS) > math.radians(8.0):
        print(f"elevation bias {math.degrees(EL_BIAS):+.2f} deg UNTRUSTED "
              f"(|bias|>8) — using 0 ({len(el_meas)} frames)")
        EL_BIAS = 0.0
    det_vec.el_bias = EL_BIAS or 0.0
    print(f"elevation bias: {math.degrees(EL_BIAS or 0.0):+.2f} deg "
          f"({len(el_meas)} near-tick frames)  [offset_to_bearing de-tilt]")
    print("range source: pinhole_dist (PnP range discarded — 1.37x long)")
    print(f"leg anchor: tick ±{TICK_HALF} frames; dwell {RMIN}-{RMAX} m "
          f"back-propagated to tick (not used as raw leg length)")

    # Model-free IMU traverse for tick->first-sighting gaps (no plant, no course
    # constants; signs+bias self-calibrated from this flight's own data).
    trav = ImuTraverse(sess, rows, clock_offset_s=offset)
    print(f"imu traverse: {'OK — ' + trav.diag() if trav.ok else 'unavailable (plant bridge fallback)'}")

    def _seq_t(s):
        for k in range(s, s + 8):
            r = by_seq.get(k)
            if r is not None:
                try:
                    return float(r["t"])
                except (ValueError, TypeError):
                    continue
        return None

    def _tick_vel(seq_pass):
        """Drone velocity VECTOR at the tick from the passing gate's vision track.

        Positions relative to a static gate at 2-5 m (near-range pinhole,
        honest) differentiated over the last ~0.8 s before the tick.
        Course-agnostic: measures whatever speed the survey actually flew.
        """
        samples = []  # (t, vec)
        for off in range(-28, -1):
            fid = seq2fid.get(seq_pass + off)
            f = by_fid.get(fid) if fid is not None else None
            r = by_seq.get(seq_pass + off)
            if not f or not f.get("dets") or r is None:
                continue
            best = max(f["dets"], key=lambda d: d.get("area_frac", 0.0))
            if (best.get("area_frac") or 0) < 0.03:
                continue
            v = det_vec(best, r)
            if v is None:
                continue
            try:
                samples.append((float(r["t"]), v))
            except (ValueError, TypeError):
                continue
        if len(samples) < 5:
            return None
        t0 = samples[0][0]
        ts = [t - t0 for t, _ in samples]
        mt = sum(ts) / len(ts)
        var = sum((t - mt) ** 2 for t in ts)
        if var < 1e-3:
            return None
        vel = []
        for ax in range(3):
            xs = [v[ax] for _, v in samples]
            mx = sum(xs) / len(xs)
            slope = sum((t - mt) * (x - mx) for t, x in zip(ts, xs)) / var
            vel.append(-slope)  # gate static: drone vel = -d(vec)/dt
        sp = math.hypot(vel[0], vel[1])
        if not (0.2 <= sp <= 15.0):
            return None
        return tuple(vel)

    def _gap_bridge(seq_pass, fo, exp_dir):
        """Displacement tick -> tick+fo frames. IMU traverse (model-free) with
        vision-track v0 when available; plant bridge as fallback."""
        t0, t1 = _seq_t(seq_pass), _seq_t(seq_pass + fo)
        v0 = _tick_vel(seq_pass)
        if trav.ok and t0 is not None and t1 is not None and v0 is not None:
            d = trav.traverse(t0, t1, v0)
            if d is not None:
                # Sanity: implied mean speed must stay inside the survey
                # envelope. Uncalibrated IMU bias (no spawn hold) can run a
                # multi-second bridge at 10+ m/s — physically impossible for
                # steady creep. Vehicle constant, not a course constant.
                sp_br = math.hypot(d[0], d[1]) / max(d[3], 0.05)
                if sp_br <= 6.0:
                    return d[0], d[1], d[2], d[3], "imu"
        br = _csv_bridge_disp(by_seq, seq_pass, seq_pass + fo, exp_dir,
                              v0_speed=_tick_speed(by_seq, seq_pass))
        if br is not None:
            return br[0], br[1], br[2], br[3], "plant"
        return None

    def collect(seq0, n_frames, exp_dir, ang_tol=0.40, max_dist=None,
                prefer_dwell=True, hi_sane=40.0):
        """Median + σ of gate vectors at the TICK anchor.

        Mid-leg dwell sightings are ~remaining range, NOT gate-to-gate. We either:
          (1) take measurements in the tick window (drone still at gate k), or
          (2) take a close-range dwell vector and back-propagate to the tick along
              the vision track: leg = v(t_f) + Σ (v_j - v_{j+1}).
        """
        by_off = {}
        for off in range(-TICK_HALF, max(n_frames, TICK_HALF + 1)):
            fid = seq2fid.get(seq0 + off)
            f = by_fid.get(fid) if fid is not None else None
            r = by_seq.get(seq0 + off) or by_seq.get(seq0)
            if not f or not f.get("dets") or r is None:
                continue
            for det in f["dets"]:
                v = det_vec(det, r)
                if v is None:
                    continue
                h = math.hypot(v[0], v[1])
                if h < 2.0:
                    continue
                if max_dist is not None and h > max_dist:
                    continue
                # Far ring-only blobs are association poison: a 34 m fringe hit
                # at the ang_tol edge became track[0], and dwell back-prop
                # telescoped every sample to that bogus leg.
                if (not det.get("has_opening")) and h > 15.0:
                    continue
                if (not det.get("has_opening")) and (det.get("area_frac") or 0) < 0.002:
                    continue
                ang = math.atan2(v[1], v[0])
                d_ang = abs((ang - exp_dir + math.pi) % (2 * math.pi) - math.pi)
                # tighter bearing gate at long range
                tol = ang_tol if h <= 12.0 else min(ang_tol, 0.22)
                if d_ang >= tol:
                    continue
                prev = by_off.get(off)
                if prev is None or d_ang < prev[2]:
                    by_off[off] = (v, h, d_ang)

        if not by_off:
            return None

        track = [(off, by_off[off][0]) for off in sorted(by_off) if off >= 0]

        def _sane(v, lo=4.0, hi=None):
            return lo <= math.hypot(v[0], v[1]) <= (hi if hi is not None else hi_sane)

        tick_vs = []
        dwell_bp = []
        for off, (v, h, _) in by_off.items():
            if abs(off) <= TICK_HALF and _sane(v):
                tick_vs.append(v)
        for i, (off, v) in enumerate(track):
            h = math.hypot(v[0], v[1])
            if off <= TICK_HALF:
                continue
            if not (RMIN <= h <= RMAX):
                continue
            bp = _backprop_to_anchor(track, i)
            if _sane(bp):
                dwell_bp.append(bp)

        # Early-leg (first ~0.7 s): still near gate k — last good fallback
        early_vs = [by_off[o][0] for o in sorted(by_off)
                    if 0 <= o <= max(20, TICK_HALF * 4) and _sane(by_off[o][0])]

        # Prefer tick-window (correct anchor). Early-leg next. Dwell back-prop
        # last (equals v(first) on a continuous track; association breaks can
        # inflate). Never use raw mid-leg remaining-range as the leg.
        first_off = track[0][0] if track else 0
        source = None
        if len(tick_vs) >= 3:
            use = tick_vs
            n_dwell = len(dwell_bp)
            source = "tick"
            first_off = 0
        elif len(early_vs) >= 3:
            use = early_vs
            n_dwell = len(dwell_bp)
            source = "early"
            first_off = min(o for o in by_off if 0 <= o <= max(20, TICK_HALF * 4)
                            and _sane(by_off[o][0]))
        elif prefer_dwell and len(dwell_bp) >= 5:
            use = dwell_bp
            n_dwell = len(dwell_bp)
            source = "dwell_bp"
        elif len(dwell_bp) >= 3:
            use = dwell_bp
            n_dwell = len(dwell_bp)
            source = "dwell_bp"
        else:
            return None

        vx = [p[0] for p in use]
        vy = [p[1] for p in use]
        vz = [p[2] for p in use]
        mx, sx, nx = _sigma(vx)
        my, sy, _ = _sigma(vy)
        mz, sz, _ = _sigma(vz)
        return {
            "v": (mx, my, mz),
            "sig": (sx, sy, sz),
            "n": nx,
            "n_dwell": n_dwell,
            "n_tick": len(tick_vs),
            "leg_xy": math.hypot(mx, my),
            "sig_xy": math.hypot(sx or 0.0, sy or 0.0),
            "first_off": int(first_off),
            "source": source,
            "by_off": by_off,  # for bridge path when needed
        }

    # spawn → g0
    psi0 = math.radians(float(gates_old.get(0, {}).get("heading_deg", 97.1)))
    seq_start = min(by_seq.keys()) if by_seq else 1
    c0 = collect(seq_start, 90, psi0, ang_tol=0.55, max_dist=35.0,
                 prefer_dwell=False)
    if c0:
        pos = {0: c0["v"]}
        sig = {0: c0}
        print(f"spawn->g0: leg {c0['leg_xy']:.1f} m  sig_xy={c0['sig_xy']:.2f}  "
              f"n={c0['n']} tick={c0.get('n_tick', 0)} dwell_bp={c0['n_dwell']}")
    else:
        g0 = gates_old[0]
        pos = {0: (g0["x"], g0["y"], g0["z"])}
        sig = {0: {"v": pos[0], "sig": (None, None, None), "n": 0,
                   "n_dwell": 0, "n_tick": 0, "leg_xy": 0.0, "sig_xy": None}}
        print("spawn->g0: NO MEASUREMENT — seeded from prior")

    print(f"\n{'leg':>8} {'legXY':>6} {'prior':>6} {'scale':>6} "
          f"{'sigxy':>6} {'n':>5} {'tick':>5} {'dbp':>5}  note")
    for i, (gid, seq_pass, t_race, t_csv) in enumerate(passes):
        nxt = gid + 1
        if nxt not in gates_old or seq_pass is None or gid not in pos:
            continue
        go, gn = gates_old[gid], gates_old[nxt]
        exp_dir = math.atan2(gn["y"] - go["y"], gn["x"] - go["x"])
        old_leg_d = math.hypot(gn["x"] - go["x"], gn["y"] - go["y"])
        # Scan the FULL inter-tick span. The old min(200,...) cap (~5 s) was a
        # VQ2 short-leg assumption: on VQ1's 24 s legs every close-range frame
        # arrived after the window closed -> n=0 -> fallback (truth score s=1.52).
        if i + 1 < len(passes) and passes[i + 1][1] is not None:
            n_frames = max(30, min(2000, passes[i + 1][1] - seq_pass))
        else:
            n_frames = 600
        # Range caps scale with the prior leg (DR prior is scale-soft; bearing
        # gates do the real filtering). Fixed 35/40 m rejected VQ1's ~48 m legs.
        leg_cap = max(35.0, 0.85 * old_leg_d)
        c = collect(seq_pass, n_frames, exp_dir, max_dist=leg_cap,
                    hi_sane=max(40.0, 0.9 * old_leg_d))
        note = ""
        bridge_tag = None
        if c is not None:
            vx, vy, vz = c["v"]
            first_off = int(c.get("first_off") or 0)
            # Late first sighting: vision remaining-range + CSV/plant gap bridge
            if first_off > TICK_HALF or c.get("source") in ("early", "dwell_bp"):
                # For dwell_bp/early, first_off is track start; bridge tick→there
                fo = first_off if first_off > TICK_HALF else (
                    min((o for o in (c.get("by_off") or {}) if o >= 0), default=0))
                if fo > TICK_HALF:
                    br = _gap_bridge(seq_pass, fo, exp_dir)
                    if br is not None:
                        bx, by_, bz, bt, src = br
                        vx, vy, vz = vx + bx, vy + by_, vz + bz
                        bridge_tag = src
                        note = f"bridge_{src} +{math.hypot(bx, by_):.1f}m/{bt:.1f}s"
            leg_xy = math.hypot(vx, vy)
            pos[nxt] = (pos[gid][0] + vx, pos[gid][1] + vy, pos[gid][2] + vz)
            sig[nxt] = {
                **{k: v for k, v in c.items() if k != "by_off"},
                "v": (vx, vy, vz),
                "leg_xy": leg_xy,
                "bridge": bridge_tag,
            }
            scale = leg_xy / max(old_leg_d, 0.1)
            print(f"g{gid}->g{nxt}  {leg_xy:6.1f} {old_leg_d:6.1f} {scale:6.2f} "
                  f"{c['sig_xy']:6.2f} {c['n']:5d} {c.get('n_tick', 0):5d} "
                  f"{c['n_dwell']:5d}  {note}")
        else:
            # No tick/early/dwell: find first sane sighting in the window + bridge
            by_off = {}
            for off in range(0, n_frames):
                fid = seq2fid.get(seq_pass + off)
                f = by_fid.get(fid) if fid is not None else None
                r = by_seq.get(seq_pass + off) or by_seq.get(seq_pass)
                if not f or not f.get("dets") or r is None:
                    continue
                for det in f["dets"]:
                    v = det_vec(det, r)
                    if v is None:
                        continue
                    h = math.hypot(v[0], v[1])
                    if not (4.0 <= h <= leg_cap):
                        continue
                    if (not det.get("has_opening")) and h > 15.0:
                        continue
                    ang = math.atan2(v[1], v[0])
                    d_ang = abs((ang - exp_dir + math.pi) % (2 * math.pi) - math.pi)
                    tol = 0.40 if h <= 12.0 else 0.22
                    if d_ang >= tol:
                        continue
                    prev = by_off.get(off)
                    if prev is None or d_ang < prev[2]:
                        by_off[off] = (v, h, d_ang)
            if by_off:
                fo = min(by_off)
                v0 = by_off[fo][0]
                br = _gap_bridge(seq_pass, fo, exp_dir)
                if br is not None:
                    bx, by_, bz, bt, src = br
                    vx, vy, vz = v0[0] + bx, v0[1] + by_, v0[2] + bz
                    bridge_tag = src
                    note = (f"BRIDGE_{src} first@{fo} +{math.hypot(bx, by_):.1f}m/"
                            f"{bt:.1f}s")
                else:
                    vx, vy, vz = v0
                    note = f"first_sight@{fo} (no bridge dt)"
                leg_xy = math.hypot(vx, vy)
                pos[nxt] = (pos[gid][0] + vx, pos[gid][1] + vy, pos[gid][2] + vz)
                sig[nxt] = {
                    "v": (vx, vy, vz), "sig": (None, None, None), "n": 1,
                    "n_dwell": 0, "n_tick": 0, "leg_xy": leg_xy, "sig_xy": None,
                    "bridge": bridge_tag, "first_off": fo,
                }
                scale = leg_xy / max(old_leg_d, 0.1)
                print(f"g{gid}->g{nxt}  {leg_xy:6.1f} {old_leg_d:6.1f} {scale:6.2f} "
                      f"{'--':>6} {'1':>5} {'0':>5} {'0':>5}  {note}")
            else:
                measured = []
                for a in range(gid):
                    if a in pos and (a + 1) in pos:
                        measured.append(math.hypot(
                            pos[a + 1][0] - pos[a][0],
                            pos[a + 1][1] - pos[a][1]))
                L = _median(measured) if measured else 10.0
                ux = (gn["x"] - go["x"])
                uy = (gn["y"] - go["y"])
                un = math.hypot(ux, uy) or 1.0
                pos[nxt] = (pos[gid][0] + L * ux / un,
                            pos[gid][1] + L * uy / un,
                            pos[gid][2] + (gn["z"] - go["z"]) * (L / max(old_leg_d, 0.1)))
                sig[nxt] = {
                    "v": pos[nxt], "sig": (None, None, None), "n": 0,
                    "n_dwell": 0, "n_tick": 0, "leg_xy": L, "sig_xy": None,
                    "fallback": True, "bridge": None,
                }
                print(f"g{gid}->g{nxt}  {L:6.1f} {old_leg_d:6.1f} "
                      f"{L / max(old_leg_d, 0.1):6.2f} "
                      f"{'--':>6} {'0':>5} {'0':>5} {'0':>5}  FALLBACK dir×med_leg")

    out_gates = []
    for gid in sorted(pos):
        p = pos[gid]
        p_prev = pos.get(gid - 1)
        p_next = pos.get(gid + 1)
        a = p_prev if p_prev else p
        b = p_next if p_next else p
        dx, dy = b[0] - a[0], b[1] - a[1]
        h = math.hypot(dx, dy) or 1.0
        s = sig.get(gid, {})
        g = {
            "gate_id": gid,
            "x": round(p[0], 2), "y": round(p[1], 2), "z": round(p[2], 2),
            "cross_dir": [round(dx / h, 3), round(dy / h, 3)],
            "heading_deg": round(math.degrees(math.atan2(dy, dx)), 1),
            "anchor": "pnp_tick",
            "n_pnp": int(s.get("n") or 0),
            "n_dwell": int(s.get("n_dwell") or 0),
            "n_tick": int(s.get("n_tick") or 0),
            "sig_xy_m": None if s.get("sig_xy") is None else round(s["sig_xy"], 3),
            "sig_z_m": None if not s.get("sig") or s["sig"][2] is None
            else round(s["sig"][2], 3),
        }
        if s.get("bridge"):
            g["bridge"] = s["bridge"]
        if s.get("fallback"):
            g["fallback"] = True
        if gid in gates_old and "t_pass" in gates_old[gid]:
            g["t_pass"] = gates_old[gid]["t_pass"]
        out_gates.append(g)

    sigs = [g["sig_xy_m"] for g in out_gates if g.get("sig_xy_m") is not None]
    if sigs:
        print(f"\nper-gate sig_xy: n={len(sigs)}  "
              f"median={_median(sigs):.2f} m  max={max(sigs):.2f} m")
    else:
        print("\nno sig")

    legs_pnp = []
    legs_prior = []
    for a, b in zip(out_gates, out_gates[1:]):
        legs_pnp.append(math.hypot(b["x"] - a["x"], b["y"] - a["y"]))
        if a["gate_id"] in gates_old and b["gate_id"] in gates_old:
            ga, gb = gates_old[a["gate_id"]], gates_old[b["gate_id"]]
            legs_prior.append(math.hypot(gb["x"] - ga["x"], gb["y"] - ga["y"]))
    if legs_pnp:
        print(f"path XY sum PnP={sum(legs_pnp):.1f} m  "
              f"prior={sum(legs_prior):.1f} m  "
              f"scale={sum(legs_pnp) / max(sum(legs_prior), 0.1):.2f}")

    out = {
        "source": f"PNP-TICK {os.path.basename(dbg_path)} + {os.path.basename(sess)}",
        "frame": "spawn-relative, yaw-frame world, z up (PnP opening + el self-cal)",
        "prior": os.path.basename(prior_path),
        "el_bias_deg": round(math.degrees(EL_BIAS or 0.0), 3),
        "clock_offset_s": offset,
        "gates": out_gates,
        "range_source": "pinhole_dist",
        "note": (
            "tick-anchored pinhole+bearing chain; opening-only; "
            "legs = tick-window sightings and/or dwell vectors back-propagated "
            "to tick via track Δv; tick→first-sighting gap bridged with "
            "survey-creep plant (bridge=csv) when gate k+1 not yet visible; "
            "PnP range discarded (1.37x long); "
            "el via offset_to_bearing (single de-tilt); σ_xy from MAD"
        ),
    }
    out_path = args.write or os.path.join(GIGA, "course_map_pnp_tick.json")
    ptr = os.path.join(GIGA, "course_map_pnp_tick.json")
    if (os.path.abspath(out_path) == os.path.abspath(ptr)
            and os.path.isfile(ptr)):
        try:
            old = json.load(open(ptr, encoding="utf-8"))
            old_legs = [
                math.hypot(old["gates"][i + 1]["x"] - old["gates"][i]["x"],
                           old["gates"][i + 1]["y"] - old["gates"][i]["y"])
                for i in range(len(old["gates"]) - 1)
            ]
            old_path_len = sum(old_legs) if old_legs else 0.0
            if (100.0 < old_path_len < 140.0
                    and "back-propagated" not in str(old.get("note", ""))):
                vault = os.path.join(GIGA, "_MAP_VAULT")
                os.makedirs(vault, exist_ok=True)
                bad = os.path.join(
                    vault, "course_map_pnp_tick_SHORT124_sighting_bug.json")
                shutil.copy2(ptr, bad)
                print(f"quarantined short map ({old_path_len:.1f} m) -> {bad}")
        except Exception as e:
            print(f"quarantine skip: {e}")

    json.dump(out, open(out_path, "w"), indent=1)
    print(f"\nwrote {out_path}: {len(out_gates)} gates")
    for g in out_gates:
        sx = "--" if g["sig_xy_m"] is None else f"{g['sig_xy_m']:.2f}"
        print(f"  g{g['gate_id']:2d} ({g['x']:+8.2f}, {g['y']:+8.2f}, "
              f"{g['z']:+6.2f})  sig_xy={sx}  n={g['n_pnp']} "
              f"tick={g.get('n_tick', 0)} dwell_bp={g['n_dwell']}")

    if args.install or out_path.endswith("course_map_pnp_tick.json"):
        if os.path.abspath(out_path) != os.path.abspath(ptr):
            shutil.copy2(out_path, ptr)
            print(f"installed {ptr}")


if __name__ == "__main__":
    main()
